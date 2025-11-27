/*
 * adam6224_iter_step.c
 * Версия v25: Добавлен вывод параметров на AO1-AO3 (4-20 mA).
 * - NEW: Инициализация типов выходов ADAM-6224 (AO0=±5V, AO1-3=4-20mA).
 * - NEW: Настраиваемый вывод pH/C/R1/R2 на AO1-AO3 с масштабированием.
 * - NEW: Параметры ao1_source/min/max, ao2_*, ao3_* в iter_params.txt.
 * 
 * Версия v24: Упрощено чтение параметров.
 * Версия v23: Авто-релоад параметров при изменении файла.
 * Версия v22: Исправлена работа с параметрами тока (Tok1/Tok2).
 * Версия v21: EMA фильтры, формула pH (деление на 10/100).
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <modbus/modbus.h>

#include "adamapi.h"

#define ITER_PARAMS_FILE   "/home/root/iter_params.txt"
#define FILE_CURRENT       "/home/root/iter_current.csv"
#define FILE_PREVIOUS      "/home/root/iter_prev_full.csv"
#define LOG_CURRENT        "/home/root/worker_out_current.log"
#define LOG_PREVIOUS       "/home/root/worker_out_prev.log"

#define ADAM6224_IP      "192.168.2.2"
#define ADAM6224_PORT    502
#define ADAM6224_SLAVE   1

#define MODBUS_CTRL_IP   "127.0.0.1"
#define MODBUS_CTRL_PORT 1502
#define MODBUS_CTRL_SLAVE 1

#define CONTROL_REG_ADDR 132
#define CONTROL_REG_COUNT 2
#define RESULTS_ADDR     1000 
#define PHASE_RESULTS_BASE 3000
#define CHEM_RESULTS_ADDR 4000

#define CMD_START   0x0001
#define CMD_STOP    0x0002
#define CMD_RESTART 0x0004

#define AO0_REG_ADDR     0
#define AO_MIN_V   (-5.0)
#define AO_MAX_V   ( 5.0)

/* ADAM-6224 AO Type Configuration Registers */
#define AO_TYPE_REG_BASE 200  /* Регистры 40201-40204 (0-based: 200-203) */
#define AO_TYPE_PM5V     4    /* ±5V */
#define AO_TYPE_4_20MA   1    /* 4-20 mA */

/* AO Source codes */
#define AO_SRC_OFF   0
#define AO_SRC_PH    1
#define AO_SRC_C     2
#define AO_SRC_R1    3
#define AO_SRC_R2    4

#define MAX_PHASES 5
#define CHANNELS 8

typedef struct {
    int start_mV; int end_mV; int step_mV; int period_ms; int settle_ms; int pause_ms;
} IterPhase;

/* Конфигурация аналогового выхода 4-20 mA - все FLOAT */
typedef struct {
    float source;    /* 0=OFF, 1=pH, 2=C, 3=R1, 4=R2 */
    float min_val;   /* Значение при 4 mA */
    float max_val;   /* Значение при 20 mA */
} AOConfig;

typedef struct {
    IterPhase phases[MAX_PHASES];
    int num_phases;
    long repeats;
    float ch_k[CHANNELS];
    float ch_b[CHANNELS];
    float calc_k_sum;
    float calc_b_sum;
    float calc_alpha_c;      /* EMA Alpha для Концентрации */
    float calc_deadband_acid;
    float calc_deadband_alkali;
    float calc_ph_neutral;
    float calc_k_acid;
    float calc_b_acid;
    float calc_k_alkali;
    float calc_b_alkali;
    float calc_alpha_redox1; /* EMA Alpha для Redox 1 */
    float calc_alpha_redox2; /* EMA Alpha для Redox 2 */
    /* Modbus TCP Settings (ADAM-6224) */
    char rs485_ip[64];
    int rs485_port;
    int rs485_slave;
    float tok1_k;            /* Коэф. тока фазы 1 */
    float tok2_k;            /* Коэф. тока фазы 2 */
    /* Analog Outputs 4-20 mA (AO1-AO3) */
    AOConfig ao[3];          /* ao[0]=AO1, ao[1]=AO2, ao[2]=AO3 */
} IterParams;

/* --- EMA STATE --- */
static int ema_initialized = 0;
static float ema_prev_c = 0.0f;
static float ema_prev_redox1 = 0.0f;
static float ema_prev_redox2 = 0.0f;

/* Snapshots */
static float current_cycle_snapshots[MAX_PHASES][CHANNELS];

/* Utils */
static uint16_t voltage_to_code(double v) {
    const double v_min = AO_MIN_V; const double v_max = AO_MAX_V;
    const int code_min = 0; const int code_max = 4095;
    if (v < v_min) { v = v_min; }
    if (v > v_max) { v = v_max; }
    double k = (v - v_min) / (v_max - v_min);
    return (uint16_t)(k * (code_max - code_min) + 0.5);
}
static double code_to_voltage(uint16_t code) {
    const double v_min = AO_MIN_V; const double v_max = AO_MAX_V;
    const int code_min = 0; const int code_max = 4095;
    if (code > code_max) code = code_max;
    double k = ((double)code - code_min) / (double)(code_max - code_min);
    return v_min + k * (v_max - v_min);
}
static double timespec_to_ms(const struct timespec *ts) { return (double)ts->tv_sec * 1000.0 + (double)ts->tv_nsec / 1.0e6; }
static void strtrim(char *s) {
    char *p = s; while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t len = strlen(s); while (len > 0 && isspace((unsigned char)s[len - 1])) s[--len] = '\0';
}
static void timespec_add_ms(struct timespec *ts, int ms) {
    if (ms <= 0) return;
    ts->tv_sec += ms / 1000; ts->tv_nsec += (ms % 1000) * 1000000L;
    while (ts->tv_nsec >= 1000000000L) { ts->tv_nsec -= 1000000000L; ts->tv_sec += 1; }
}
static void wait_with_pause(struct timespec *t_set, int pause_ms) {
    if (pause_ms <= 0) return;
    timespec_add_ms(t_set, pause_ms);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, t_set, NULL);
}

/* Params */
static void init_iter_params(IterParams *p) {
    p->num_phases = 1; p->repeats = 1;
    for (int i = 0; i < MAX_PHASES; ++i) {
        p->phases[i].start_mV = -5000; p->phases[i].end_mV = 5000;
        p->phases[i].step_mV = 100; p->phases[i].period_ms = 100;
        p->phases[i].settle_ms = 50; p->phases[i].pause_ms = 0;
    }
    for (int i = 0; i < CHANNELS; i++) { p->ch_k[i] = 1.0f; p->ch_b[i] = 0.0f; }
    p->calc_k_sum = 1.0f; p->calc_b_sum = 0.0f;
    p->calc_alpha_c = 0.5f;
    p->calc_deadband_acid = 0.1f; p->calc_deadband_alkali = 0.1f;
    p->calc_ph_neutral = 7.0f;
    p->calc_k_acid = 1.0f; p->calc_b_acid = 0.0f;
    p->calc_k_alkali = 1.0f; p->calc_b_alkali = 0.0f;
    p->calc_alpha_redox1 = 0.5f;
    p->calc_alpha_redox2 = 0.5f;
    /* FIX v22: Инициализация коэффициентов тока */
    p->tok1_k = 1.0f;
    p->tok2_k = 1.0f;
    /* Modbus TCP defaults (ADAM-6224) */
    strncpy(p->rs485_ip, ADAM6224_IP, sizeof(p->rs485_ip) - 1);
    p->rs485_ip[sizeof(p->rs485_ip) - 1] = '\0';
    p->rs485_port = ADAM6224_PORT;
    p->rs485_slave = ADAM6224_SLAVE;
    /* v25: Analog Outputs defaults (OFF) */
    for (int i = 0; i < 3; i++) {
        p->ao[i].source = 0.0f;  /* AO_SRC_OFF */
        p->ao[i].min_val = 0.0f;
        p->ao[i].max_val = 100.0f;
    }
}
static void update_phase_count(IterParams *p, int idx) {
    if (idx + 1 > p->num_phases) p->num_phases = idx + 1;
    if (p->num_phases > MAX_PHASES) p->num_phases = MAX_PHASES;
}
static int parse_phase_key(const char *key, int *phase_idx, const char **suffix) {
    const char *p = NULL;
    if (strncmp(key, "step", 4) == 0) p = key + 4; else if (strncmp(key, "phase", 5) == 0) p = key + 5;
    if (p && isdigit((unsigned char)*p)) {
        char *endptr = NULL; long idx = strtol(p, &endptr, 10);
        if (idx >= 1 && idx <= MAX_PHASES && endptr && *endptr == '_') { *phase_idx = (int)idx - 1; *suffix = endptr + 1; return 1; }
    }
    *phase_idx = 0; *suffix = key; return 0;
}
static int load_iter_params(const char *path, IterParams *p) {
    FILE *fp = fopen(path, "r"); if (!fp) { perror("Params File Error"); return -1; }
    init_iter_params(p);
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        strtrim(line); if (line[0] == '\0' || line[0] == '#') continue;
        char *eq = strchr(line, '='); if (!eq) continue; *eq = '\0';
        char *key = line; char *val = eq + 1; strtrim(key); strtrim(val);
        
        /* FIX v24: Упрощенное чтение параметров (отдельные блоки) */
        if (strncmp(key, "calc_", 5) == 0) {
            if (strcmp(key, "calc_k_sum") == 0) p->calc_k_sum = strtof(val, NULL);
            else if (strcmp(key, "calc_b_sum") == 0) p->calc_b_sum = strtof(val, NULL);
            else if (strcmp(key, "calc_filter_size") == 0 || strcmp(key, "calc_alpha_c") == 0) p->calc_alpha_c = strtof(val, NULL);
            else if (strcmp(key, "calc_deadband_acid") == 0) p->calc_deadband_acid = strtof(val, NULL);
            else if (strcmp(key, "calc_deadband_alkali") == 0) p->calc_deadband_alkali = strtof(val, NULL);
            else if (strcmp(key, "calc_ph_neutral") == 0) p->calc_ph_neutral = strtof(val, NULL);
            else if (strcmp(key, "calc_k_acid") == 0) p->calc_k_acid = strtof(val, NULL);
            else if (strcmp(key, "calc_b_acid") == 0) p->calc_b_acid = strtof(val, NULL);
            else if (strcmp(key, "calc_k_alkali") == 0) p->calc_k_alkali = strtof(val, NULL);
            else if (strcmp(key, "calc_b_alkali") == 0) p->calc_b_alkali = strtof(val, NULL);
            else if (strcmp(key, "calc_filter_redox1") == 0 || strcmp(key, "calc_alpha_redox1") == 0) p->calc_alpha_redox1 = strtof(val, NULL);
            else if (strcmp(key, "calc_filter_redox2") == 0 || strcmp(key, "calc_alpha_redox2") == 0) p->calc_alpha_redox2 = strtof(val, NULL);
            continue;
        }
        if (strcmp(key, "tok1_k") == 0) { p->tok1_k = strtof(val, NULL); continue; }
        if (strcmp(key, "tok2_k") == 0) { p->tok2_k = strtof(val, NULL); continue; }
        
        /* v25: Analog Outputs (ao1_*, ao2_*, ao3_*) - все FLOAT */
        if (strncmp(key, "ao", 2) == 0 && isdigit(key[2])) {
            int ao_idx = key[2] - '1';  /* ao1 -> 0, ao2 -> 1, ao3 -> 2 */
            if (ao_idx >= 0 && ao_idx < 3) {
                if (strcmp(key + 3, "_source") == 0) { p->ao[ao_idx].source = strtof(val, NULL); continue; }
                if (strcmp(key + 3, "_min") == 0) { p->ao[ao_idx].min_val = strtof(val, NULL); continue; }
                if (strcmp(key + 3, "_max") == 0) { p->ao[ao_idx].max_val = strtof(val, NULL); continue; }
            }
        }

        if (strncmp(key, "ch", 2) == 0 && isdigit(key[2])) {
            int ch = key[2] - '1';
            if (ch >= 0 && ch < CHANNELS) {
                if (strcmp(key + 3, "_k") == 0) { p->ch_k[ch] = strtof(val, NULL); continue; }
                if (strcmp(key + 3, "_b") == 0) { p->ch_b[ch] = strtof(val, NULL); continue; }
            }
        }
        /* Modbus TCP Settings (ADAM-6224) */
        if (strcmp(key, "rs485_ip") == 0) {
            strncpy(p->rs485_ip, val, sizeof(p->rs485_ip) - 1);
            p->rs485_ip[sizeof(p->rs485_ip) - 1] = '\0';
            continue;
        }
        if (strcmp(key, "rs485_port") == 0) { p->rs485_port = atoi(val); continue; }
        if (strcmp(key, "rs485_slave") == 0) { p->rs485_slave = atoi(val); continue; }
        
        if (strcmp(key, "repeats") == 0) { long r = strtol(val, NULL, 10); p->repeats = (r == 0 || r == -1) ? r : (r < 0 ? 1 : r); continue; }
        int v = atoi(val); int phase_idx = 0; const char *suffix = key;
        parse_phase_key(key, &phase_idx, &suffix);
        if (phase_idx >= MAX_PHASES) phase_idx = MAX_PHASES - 1;
        IterPhase *phase = &p->phases[phase_idx]; update_phase_count(p, phase_idx);
        if (strcmp(suffix, "start_mV") == 0) phase->start_mV = v;
        else if (strcmp(suffix, "end_mV") == 0) phase->end_mV = v;
        else if (strcmp(suffix, "step_mV") == 0) phase->step_mV = v;
        else if (strcmp(suffix, "period_ms") == 0) phase->period_ms = v;
        else if (strcmp(suffix, "settle_ms") == 0) phase->settle_ms = v;
        else if (strcmp(suffix, "pause_ms") == 0) phase->pause_ms = v;
        else if (strcmp(key, "phases") == 0) { if (v >= 1 && v <= MAX_PHASES) p->num_phases = v; }
    }
    fclose(fp); 
    if (p->num_phases < 1) p->num_phases = 1; 
    
    
    return 0;
}

static volatile sig_atomic_t g_stop = 0;
static void handle_sigint(int sig) { (void)sig; g_stop = 1; }
typedef enum { CONTROL_RUNNING = 0, CONTROL_STOPPED = 1 } ControlState;
static float regs_to_float(const uint16_t *regs) { union { float f; uint32_t u; } conv; conv.u = ((uint32_t)regs[0] << 16) | (uint32_t)regs[1]; return conv.f; }

/* FIX v23: Функция чтения времени изменения файла */
static int read_file_mtime(const char *path, time_t *out) { struct stat st; if (stat(path, &st) == -1) return -1; *out = st.st_mtime; return 0; }

static void poll_control_commands(modbus_t *ctx, ControlState *state, int *restart_requested) {
    if (!ctx || !state) return;
    uint16_t regs[CONTROL_REG_COUNT] = {0};
    int rc = modbus_read_registers(ctx, CONTROL_REG_ADDR, CONTROL_REG_COUNT, regs);
    if (rc == -1) {
        if (errno == EPIPE || errno == ECONNRESET || errno == ETIMEDOUT || errno == EBADF) {
            static time_t last_log = 0; time_t now = time(NULL);
            if (now - last_log > 5) { fprintf(stderr, "Lost Local Server. Reconnecting...\n"); last_log = now; }
            modbus_close(ctx); if (modbus_connect(ctx) == -1) return;
            modbus_read_registers(ctx, CONTROL_REG_ADDR, CONTROL_REG_COUNT, regs);
        } else return;
    }
    float cmd = regs_to_float(regs);
    uint16_t mask = 0;
    if (fabsf(cmd - 1.0f) < 0.001f) mask = CMD_START; else if (fabsf(cmd - 2.0f) < 0.001f) mask = CMD_STOP; else if (fabsf(cmd - 3.0f) < 0.001f) mask = CMD_RESTART;
    if (mask != 0) {
        uint16_t zeros[2] = {0}; modbus_write_registers(ctx, CONTROL_REG_ADDR, 2, zeros);
        if (mask & CMD_RESTART) { *restart_requested = 1; *state = CONTROL_RUNNING; }
        if (mask & CMD_STOP) { *state = CONTROL_STOPPED; }
        if (mask & CMD_START) { *state = CONTROL_RUNNING; }
        return;
    }
    if (regs[0] != 0) {
        modbus_write_register(ctx, CONTROL_REG_ADDR, 0);
        if (regs[0] & CMD_RESTART) { *restart_requested = 1; *state = CONTROL_RUNNING; }
        if (regs[0] & CMD_STOP) { *state = CONTROL_STOPPED; }
        if (regs[0] & CMD_START) { *state = CONTROL_RUNNING; }
    }
}

/* --- EMA Helper --- */
static float calculate_ema(float new_val, float prev_val, float alpha) {
    /* Ограничиваем Alpha [0..1] */
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    
    /* Формула: EMA = Old * Alpha + New * (1 - Alpha) */
    /* Примечание: В некоторых источниках наоборот, но делаем как на фото пользователя */
    return prev_val * alpha + new_val * (1.0f - alpha);
}

/* --- v25: ADAM-6224 Analog Output Functions --- */

/* Инициализация типов выходов ADAM-6224 */
static int init_adam6224_ao_types(modbus_t *ctx) {
    uint16_t ao_types[4] = {
        AO_TYPE_PM5V,    /* AO0: ±5V (для итераций) */
        AO_TYPE_4_20MA,  /* AO1: 4-20 mA */
        AO_TYPE_4_20MA,  /* AO2: 4-20 mA */
        AO_TYPE_4_20MA   /* AO3: 4-20 mA */
    };
    
    if (modbus_write_registers(ctx, AO_TYPE_REG_BASE, 4, ao_types) == -1) {
        fprintf(stderr, "Failed to init AO types: %s\n", modbus_strerror(errno));
        return -1;
    }
    printf("ADAM-6224 AO types initialized: AO0=±5V, AO1-3=4-20mA\n");
    return 0;
}

/* Преобразование значения в mA (4-20) */
static float value_to_mA(float value, float min_val, float max_val) {
    if (fabsf(max_val - min_val) < 0.0001f) return 4.0f;  /* Защита от деления на 0 */
    
    float ratio = (value - min_val) / (max_val - min_val);
    
    /* Ограничение 0...1 (автоматически поддерживает инверсию если min > max) */
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    
    return 4.0f + ratio * 16.0f;  /* 4-20 mA */
}

/* Преобразование mA в код для ADAM-6224 (4-20 mA режим) */
static uint16_t mA_to_code(float mA) {
    /* ADAM-6224: 4-20 mA -> 0-4095 */
    if (mA < 4.0f) mA = 4.0f;
    if (mA > 20.0f) mA = 20.0f;
    
    float ratio = (mA - 4.0f) / 16.0f;
    return (uint16_t)(ratio * 4095.0f + 0.5f);
}

/* Запись значения на аналоговый выход */
static void write_ao_output(modbus_t *ctx, int ao_channel, float mA) {
    if (ao_channel < 0 || ao_channel > 3) return;
    
    uint16_t code = mA_to_code(mA);
    if (modbus_write_register(ctx, ao_channel, code) == -1) {
        /* Ошибка записи - не критично, продолжаем */
    }
}

/* Глобальные переменные для хранения последних значений химии */
static float g_last_pH = 7.0f;
static float g_last_C_filt = 0.0f;
static float g_last_R1_avg = 0.0f;
static float g_last_R2_avg = 0.0f;

/* Вывод параметров на AO1-AO3 */
static void update_analog_outputs(modbus_t *ctx, const IterParams *par) {
    float params[5] = {0.0f, g_last_pH, g_last_C_filt, g_last_R1_avg, g_last_R2_avg};
    
    for (int i = 0; i < 3; i++) {
        int src = (int)(par->ao[i].source + 0.5f);  /* Округление FLOAT к INT */
        int ao_channel = i + 1;  /* AO1, AO2, AO3 */
        
        if (src == AO_SRC_OFF || src < 0 || src > 4) {
            /* OFF → 0 mA (ниже 4 mA = 0 код) */
            write_ao_output(ctx, ao_channel, 0.0f);
        } else {
            float val = params[src];
            float mA = value_to_mA(val, par->ao[i].min_val, par->ao[i].max_val);
            write_ao_output(ctx, ao_channel, mA);
        }
    }
}

/* --- MAIN MATH --- */
static void PerformChemistryCalculation(IterParams *par, modbus_t *ctrl_ctx) {
    /* === 1. pH (Channel 1 sum) === */
    if (par->num_phases >= 2) {
        float I_ph1 = current_cycle_snapshots[0][1];
        float I_ph2 = current_cycle_snapshots[1][1];
        /* FIX v22: C_raw = (I_ph1 * tok1_k + I_ph2 * tok2_k) * k_sum + b_sum */
        float C_raw = (I_ph1 * par->tok1_k + I_ph2 * par->tok2_k) * par->calc_k_sum + par->calc_b_sum;

        /* EMA Filter */
        if (!ema_initialized) {
            ema_prev_c = C_raw;
        } else {
            ema_prev_c = calculate_ema(C_raw, ema_prev_c, par->calc_alpha_c);
        }
        float C_filt = ema_prev_c;

        float C_acid = 0.0f, C_alkali = 0.0f, pH = par->calc_ph_neutral;
        if (C_filt < -fabsf(par->calc_deadband_acid)) {
            C_acid = fabsf(C_filt);
            /* FIX v21: Для кислоты делим концентрацию на 10 перед логарифмом */
            float C_log_arg = C_acid / 10.0f;
            if (C_log_arg > 0.000001f) pH = par->calc_k_acid * log10f(C_log_arg) + par->calc_b_acid;
        } else if (C_filt > fabsf(par->calc_deadband_alkali)) {
            C_alkali = C_filt;
            /* FIX v21: Для щёлочи делим концентрацию на 100 перед логарифмом */
            float C_log_arg = C_alkali / 100.0f;
            if (C_log_arg > 0.000001f) pH = par->calc_k_alkali * log10f(C_log_arg) + par->calc_b_alkali;
        }

        /* v25: Сохраняем значения для вывода на AO */
        g_last_pH = pH;
        g_last_C_filt = C_filt;

        if (ctrl_ctx) {
            uint16_t regs[10]; union { float f; uint32_t u; } c;
            c.f = C_raw; regs[0]=c.u>>16; regs[1]=c.u&0xFFFF;
            c.f = C_filt; regs[2]=c.u>>16; regs[3]=c.u&0xFFFF;
            c.f = C_acid; regs[4]=c.u>>16; regs[5]=c.u&0xFFFF;
            c.f = C_alkali; regs[6]=c.u>>16; regs[7]=c.u&0xFFFF;
            c.f = pH; regs[8]=c.u>>16; regs[9]=c.u&0xFFFF;
            modbus_write_registers(ctrl_ctx, CHEM_RESULTS_ADDR, 10, regs);
        }
    } else {
        /* ДИАГНОСТИКА: Почему не считается химия */
        printf("[WARN] Chemistry skipped! Need >= 2 phases, got %d.\n", par->num_phases);
    }

    /* === 2. REDOX (Channels 2 & 3) === */
    if (par->num_phases > 0) {
        int last_ph = par->num_phases - 1;
        float r1_raw = current_cycle_snapshots[last_ph][2]; /* Ch 2 */
        float r2_raw = current_cycle_snapshots[last_ph][3]; /* Ch 3 */

        if (!ema_initialized) {
            ema_prev_redox1 = r1_raw;
            ema_prev_redox2 = r2_raw;
            ema_initialized = 1; /* Теперь фильтры инициализированы */
        } else {
            ema_prev_redox1 = calculate_ema(r1_raw, ema_prev_redox1, par->calc_alpha_redox1);
            ema_prev_redox2 = calculate_ema(r2_raw, ema_prev_redox2, par->calc_alpha_redox2);
        }
        float r1_avg = ema_prev_redox1;
        float r2_avg = ema_prev_redox2;

        /* v25: Сохраняем значения для вывода на AO */
        g_last_R1_avg = r1_avg;
        g_last_R2_avg = r2_avg;

        if (ctrl_ctx) {
            uint16_t regs[8]; union { float f; uint32_t u; } c;
            c.f = r1_raw; regs[0]=c.u>>16; regs[1]=c.u&0xFFFF;
            c.f = r1_avg; regs[2]=c.u>>16; regs[3]=c.u&0xFFFF;
            c.f = r2_raw; regs[4]=c.u>>16; regs[5]=c.u&0xFFFF;
            c.f = r2_avg; regs[6]=c.u>>16; regs[7]=c.u&0xFFFF;
            modbus_write_registers(ctrl_ctx, CHEM_RESULTS_ADDR + 10, 8, regs);
        }
        printf("Redox1: %.3f/%.3f  Redox2: %.3f/%.3f\n", r1_raw, r1_avg, r2_raw, r2_avg);
    }
}

int main(void) {
    signal(SIGINT, handle_sigint);
    IterParams par; if (load_iter_params(ITER_PARAMS_FILE, &par) != 0) return -1;
    /* FIX v23: Запоминаем время изменения файла параметров */
    time_t params_mtime = 0; read_file_mtime(ITER_PARAMS_FILE, &params_mtime);

    FILE *f = NULL;
    FILE *log_file = NULL;
    int fd_io = -1;
    int io_retry = 0;
    while (!g_stop) {
        int ret = AdamIO_Open(&fd_io);
        if (ret >= 0) { printf("ADAM IO Driver Open Success (fd=%d)\n", fd_io); break; }
        if (io_retry++ % 5 == 0) fprintf(stderr, "Waiting for ADAM IO driver (libadamapi)...\n");
        sleep(1);
    }
    if (g_stop) return 0;
    
    AI_SetAutoFilterEnabled(fd_io, 0x00, 0); AI_SetIntegrationMode(fd_io, 0xA0);

    modbus_t *ctx = modbus_new_tcp(par.rs485_ip, par.rs485_port);
    /* FIX v20: Проверка NULL для предотвращения crash при неверном IP */
    if (!ctx) {
        fprintf(stderr, "FATAL: Cannot create Modbus context for ADAM-6224 (%s:%d). Check rs485_ip/rs485_port.\n", 
                par.rs485_ip, par.rs485_port);
        AdamIO_Close(fd_io);
        return -1;
    }
    modbus_set_slave(ctx, par.rs485_slave);
    modbus_set_response_timeout(ctx, 2, 0);
    
    int retry=0;
    while(!g_stop) {
        if(modbus_connect(ctx)==0) { printf("AO Connected to %s:%d.\n", par.rs485_ip, par.rs485_port); break; }
        if(retry++ % 5 == 0) fprintf(stderr, "AO Wait (connecting to %s:%d)...\n", par.rs485_ip, par.rs485_port);
        sleep(1);
    }

    /* v25: Инициализация типов выходов ADAM-6224 */
    init_adam6224_ao_types(ctx);

    modbus_t *ctrl_ctx = modbus_new_tcp(MODBUS_CTRL_IP, MODBUS_CTRL_PORT);
    if (ctrl_ctx) { modbus_set_slave(ctrl_ctx, MODBUS_CTRL_SLAVE); modbus_connect(ctrl_ctx); modbus_set_response_timeout(ctrl_ctx, 1, 0); }

    ControlState ctrl_state = CONTROL_RUNNING;
    int restart_requested = 0;

    while (!g_stop) {
        if (restart_requested) { 
            load_iter_params(ITER_PARAMS_FILE, &par); 
            printf("Restarted (Command).\n"); 
            restart_requested = 0; 
            /* FIX v23: Обновляем время файла, чтобы не сработал авто-релоад сразу же */
            read_file_mtime(ITER_PARAMS_FILE, &params_mtime);
        } else {
            /* FIX v23: Автоматическая перезагрузка параметров при изменении файла */
            time_t current_mtime = 0;
            if (read_file_mtime(ITER_PARAMS_FILE, &current_mtime) == 0) {
                if (current_mtime != params_mtime) {
                    printf("Params file changed. Reloading...\n");
                    load_iter_params(ITER_PARAMS_FILE, &par);
                    params_mtime = current_mtime;
                }
            }
        }
        
        struct timespec t0; clock_gettime(CLOCK_MONOTONIC, &t0); struct timespec t_set = t0;
        float prev_ai[8] = {0};
        long cycle = 0; int abort_loops = 0;

        printf("Starting loop...\n");

        /* === ВАЖНО: Сброс фильтров перед новой серией измерений === */
        ema_initialized = 0;
        ema_prev_c = 0.0f;
        ema_prev_redox1 = 0.0f;
        ema_prev_redox2 = 0.0f;

        for (; (par.repeats == 0 || cycle < par.repeats) && !g_stop && !abort_loops; ++cycle) {
            poll_control_commands(ctrl_ctx, &ctrl_state, &restart_requested);
            if(restart_requested || ctrl_state==CONTROL_STOPPED) break;

            /* Rotate log file for this cycle */
            if (log_file) { fclose(log_file); log_file = NULL; }
            log_file = fopen(LOG_CURRENT, "w");
            if (log_file) {
                /* Redirect stdout to log file */
                if (dup2(fileno(log_file), STDOUT_FILENO) == -1) {
                    fprintf(stderr, "Failed to redirect stdout to log file\n");
                }
                setvbuf(stdout, NULL, _IOLBF, 0); /* Line buffered */
            }

            if (f) fclose(f);
            f = fopen(FILE_CURRENT, "w");
            if (f) {
                fprintf(f, "cycle;phase;idx;time_ms;iter_mV;ao_V;AI0;AI1;AI2;AI3;AI4;AI5;AI6;AI7;Calc0;Calc1;Calc2;Calc3;Calc4;Calc5;Calc6;Calc7\n");
                fflush(f);
            }

            int cycle_success = 1;

            for(int ph=0; ph<par.num_phases && !g_stop && !abort_loops; ++ph) {
                poll_control_commands(ctrl_ctx, &ctrl_state, &restart_requested);
                if(restart_requested || ctrl_state==CONTROL_STOPPED) { cycle_success=0; break; }

                IterPhase *phase = &par.phases[ph];
                int dir = (phase->step_mV > 0) ? 1 : -1;
                int iter_mV = phase->start_mV;
                int idx = 0;
                int phase_had_steps = 0;
                int last_step_valid = 0;
                
                float last_calc[8] = {0};
                
                while (!g_stop && ctrl_state == CONTROL_RUNNING && !restart_requested && ((dir>0 && iter_mV<=phase->end_mV) || (dir<0 && iter_mV>=phase->end_mV))) {
                    poll_control_commands(ctrl_ctx, &ctrl_state, &restart_requested);
                    if(restart_requested || ctrl_state==CONTROL_STOPPED) { cycle_success=0; break; }

                    if (phase->pause_ms == 0) { iter_mV += phase->step_mV; continue; }

                    if (phase_had_steps == 0) { } else timespec_add_ms(&t_set, phase->period_ms);
                    
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t_set, NULL);
                    phase_had_steps = 1;

                    uint16_t code = voltage_to_code((double)iter_mV/1000.0);
                    while(!g_stop) {
                        if(modbus_write_register(ctx, AO0_REG_ADDR, code) != -1) break;
                        if (errno == EPIPE || errno == ECONNRESET || errno == ETIMEDOUT || errno == EBADF || errno == EIO) {
                            struct timespec ts_start, ts_end; clock_gettime(CLOCK_MONOTONIC, &ts_start);
                            while(!g_stop) {
                                modbus_close(ctx); sleep(1);
                                if(modbus_connect(ctx)==0) break;
                            }
                            if(g_stop) break;
                            clock_gettime(CLOCK_MONOTONIC, &ts_end);
                            double delta = (double)(ts_end.tv_sec - ts_start.tv_sec)*1000.0 + (double)(ts_end.tv_nsec - ts_start.tv_nsec)/1e6;
                            int d_ms = (int)delta; if(d_ms > 0) { timespec_add_ms(&t_set, d_ms); timespec_add_ms(&t0, d_ms); }
                        } else { abort_loops=1; cycle_success=0; break; }
                    }
                    if(abort_loops || g_stop) { cycle_success=0; break; }

                    struct timespec t_meas = t_set; timespec_add_ms(&t_meas, phase->settle_ms);
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t_meas, NULL);

                    float ai[8]; for(int i=0; i<8; i++) { unsigned char st; if(AI_GetFloatValue(fd_io, i, &ai[i], &st)!=0) ai[i]=prev_ai[i]; else prev_ai[i]=ai[i]; }

                    float calc[8]; uint16_t res_regs[16];
                    for(int i=0; i<8; i++) {
                        calc[i] = ai[i] * par.ch_k[i] + par.ch_b[i];
                        union { float f; uint32_t u; } conv; conv.f = calc[i];
                        res_regs[i*2] = (conv.u >> 16) & 0xFFFF; res_regs[i*2+1] = conv.u & 0xFFFF;
                        last_calc[i] = calc[i];
                    }
                    if(ctrl_ctx) modbus_write_registers(ctrl_ctx, RESULTS_ADDR, 16, res_regs);

                    struct timespec t_now; clock_gettime(CLOCK_MONOTONIC, &t_now);
                    double t_ms = timespec_to_ms(&t_now) - timespec_to_ms(&t0);
                    
                    if (f) {
                        fprintf(f, "%ld;%d;%d;%.3f;%d;%.3f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f\n",
                            cycle+1, ph+1, idx, t_ms, iter_mV, code_to_voltage(code),
                            ai[0], ai[1], ai[2], ai[3], ai[4], ai[5], ai[6], ai[7],
                            calc[0], calc[1], calc[2], calc[3], calc[4], calc[5], calc[6], calc[7]);
                    }

                    printf("c=%ld p=%d i=%d mV=%d AI0=%.4f\n", cycle+1, ph+1, idx, iter_mV, ai[0]);
                    fflush(stdout);

                    last_step_valid = 1;
                    idx++; iter_mV += phase->step_mV;
                }
                if(restart_requested || ctrl_state==CONTROL_STOPPED || abort_loops) { cycle_success=0; break; }

                if (last_step_valid) {
                    for(int i=0; i<8; i++) { current_cycle_snapshots[ph][i] = last_calc[i]; }
                }

                if (phase->pause_ms > 0 && last_step_valid) {
                    /* FIX v20: Корректное время паузы - первая половина до измерения */
                    int half_pause = phase->pause_ms / 2;
                    int remaining_pause = phase->pause_ms - half_pause;
                    
                    struct timespec t_mid = t_set; timespec_add_ms(&t_mid, half_pause);
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t_mid, NULL);

                    float ai_mid[8]; for(int i=0; i<8; i++) { unsigned char st; if(AI_GetFloatValue(fd_io, i, &ai_mid[i], &st)!=0) ai_mid[i]=prev_ai[i]; else prev_ai[i]=ai_mid[i]; }

                    for(int i=0; i<8; i++) {
                        float val = ai_mid[i] * par.ch_k[i] + par.ch_b[i];
                        current_cycle_snapshots[ph][i] = val; 
                    }

                    float calc_mid[8]; uint16_t phase_res_regs[16];
                    for(int i=0; i<8; i++) {
                        calc_mid[i] = current_cycle_snapshots[ph][i];
                        union { float f; uint32_t u; } conv; conv.f = calc_mid[i];
                        phase_res_regs[i*2] = (conv.u >> 16) & 0xFFFF; phase_res_regs[i*2+1] = conv.u & 0xFFFF;
                    }
                    if(ctrl_ctx) {
                        int addr = PHASE_RESULTS_BASE + (ph * 16);
                        modbus_write_registers(ctrl_ctx, addr, 16, phase_res_regs);
                    }

                    struct timespec t_mid_now; clock_gettime(CLOCK_MONOTONIC, &t_mid_now);
                    double t_mid_ms = timespec_to_ms(&t_mid_now) - timespec_to_ms(&t0);
                    
                    if (f) {
                        fprintf(f, "%ld;%d;%d;%.3f;%d;%.3f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f\n",
                            cycle+1, ph+1, idx, t_mid_ms, 0, 0.0,
                            ai_mid[0], ai_mid[1], ai_mid[2], ai_mid[3], ai_mid[4], ai_mid[5], ai_mid[6], ai_mid[7],
                            calc_mid[0], calc_mid[1], calc_mid[2], calc_mid[3], calc_mid[4], calc_mid[5], calc_mid[6], calc_mid[7]);
                        fflush(f);
                    }
                    /* FIX v20: Ожидаем только оставшуюся часть паузы (вторую половину) */
                    wait_with_pause(&t_set, remaining_pause);
                } else {
                    if (f) fflush(f);
                }
            }
            
            if (cycle_success && !g_stop && !abort_loops) {
                if (f) { fclose(f); f = NULL; }
                rename(FILE_CURRENT, FILE_PREVIOUS);
                
                /* Rotate log file: current -> previous */
                if (log_file) { fflush(log_file); fclose(log_file); log_file = NULL; }
                rename(LOG_CURRENT, LOG_PREVIOUS);
                
                PerformChemistryCalculation(&par, ctrl_ctx);
                
                /* v25: Вывод параметров на AO1-AO3 */
                update_analog_outputs(ctx, &par);
            } else {
                if (f) { fclose(f); f = NULL; }
            }

            load_iter_params(ITER_PARAMS_FILE, &par);
        }
        if (restart_requested || ctrl_state == CONTROL_STOPPED) {
            if(ctrl_state == CONTROL_STOPPED) { struct timespec ts={0,100000000}; nanosleep(&ts, NULL); }
            continue;
        }
        break;
    }

    if (f) fclose(f);
    if (log_file) fclose(log_file);
    modbus_close(ctx); modbus_free(ctx);
    if (ctrl_ctx) { modbus_close(ctrl_ctx); modbus_free(ctrl_ctx); }
    AdamIO_Close(fd_io);
    return 0;
}
