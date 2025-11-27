/*
 * iter_modbus_server.c
 * Версия v11: Исправлены критические баги.
 * - FIX: Утечка сокетов при переполнении клиентов (теперь закрываем лишние соединения).
 * 
 * Версия v10: Настройка Modbus RTU через iter_params.txt.
 * - Добавлена возможность настройки порта, скорости, чётности, битов данных, стоп-битов и Slave ID.
 * 
 * Версия v9: Отложенная запись (Dirty Write) для защиты SD-карты.
 */

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <modbus/modbus.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#define PARAMS_FILE "/home/root/iter_params.txt"
#define SAVE_DELAY_SEC 3  /* Задержка записи на диск (секунды) */

/* Настройки */
#define MODBUS_TCP_PORT 1502
#define MAX_CLIENTS 10
#define RTU_PORT        "/dev/ttyAP0"
#define RTU_BAUD        9600
#define RTU_PARITY      'N'
#define RTU_DATA_BIT    8
#define RTU_STOP_BIT    1
#define SLAVE_ID        1

#define MAX_PHASES 5
#define CHANNELS 8

/* АДРЕСАЦИЯ */
#define INT_HEADER_REGS 6
#define INT_PHASE_REGS_PER_PHASE 12
#define INT_HOLDING_REG_COUNT (INT_HEADER_REGS + MAX_PHASES * INT_PHASE_REGS_PER_PHASE)

#define FLOAT_HEADER_REGS 6
#define FLOAT_PHASE_REGS_PER_PHASE 12
#define FLOAT_BASE INT_HOLDING_REG_COUNT
#define FLOAT_HOLDING_REG_COUNT (FLOAT_HEADER_REGS + MAX_PHASES * FLOAT_PHASE_REGS_PER_PHASE)

#define CONTROL_REG_ADDR (FLOAT_BASE + FLOAT_HOLDING_REG_COUNT)
#define CONTROL_REG_COUNT 2

/* Блоки памяти */
#define CALC_SETTINGS_START 200   
#define CALC_SETTINGS_END   999   

#define CHEM_SETTINGS_START 400   
#define CHEM_SETTINGS_COUNT 30    

#define RESULTS_START       1000  
#define RESULTS_COUNT       (CHANNELS * 2) 

#define PHASE_RESULTS_START 3000  
#define PHASE_RESULTS_COUNT (MAX_PHASES * CHANNELS * 2)

#define CHEM_RESULTS_START  4000  
#define CHEM_RESULTS_COUNT  20    

#define TOTAL_REGS          20000  /* Расширено для поддержки legacy адресов (0x4000+) */

#define CMD_START 0x0001
#define CMD_STOP 0x0002
#define CMD_RESTART 0x0004

/* Структуры */
typedef struct {
    int start_mV; int end_mV; int step_mV; int period_ms; int settle_ms; int pause_ms;
} IterPhase;

typedef struct {
    /* Итерация */
    IterPhase phases[MAX_PHASES];
    int num_phases;
    long repeats;
    /* Масштабирование каналов */
    float ch_k[CHANNELS];
    float ch_b[CHANNELS];
    /* Химия (pH) */
    float calc_k_sum;
    float calc_b_sum;
    float calc_alpha_c;      /* EMA Alpha для Концентрации (бывш. filter_size) */
    float calc_deadband_acid;
    float calc_deadband_alkali;
    float calc_ph_neutral;
    float calc_k_acid;
    float calc_b_acid;
    float calc_k_alkali;
    float calc_b_alkali;
    /* Весовые коэффициенты токов */
    float tok1_k;
    float tok2_k;
    /* Redox Фильтры */
    float calc_alpha_redox1; /* EMA Alpha для Redox 1 */
    float calc_alpha_redox2; /* EMA Alpha для Redox 2 */
    /* Analog Outputs 4-20 mA (AO1-AO3) - все FLOAT для единообразия */
    float ao1_source;  /* 0=OFF, 1=pH, 2=C, 3=R1, 4=R2 */
    float ao1_min;
    float ao1_max;
    float ao2_source;
    float ao2_min;
    float ao2_max;
    float ao3_source;
    float ao3_min;
    float ao3_max;
    /* Modbus RTU Settings (RS485 - панель оператора) */
    char rtu_port[64];
    int rtu_baud;
    char rtu_parity;
    int rtu_data_bit;
    int rtu_stop_bit;
    int rtu_slave_id;
} IterParams;

static volatile sig_atomic_t g_stop = 0;
static void handle_sigint(int sig) { (void)sig; g_stop = 1; }

static int install_signal_handlers(void) {
    struct sigaction sa; memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_sigint; sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL); sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN); return 0;
}

static void strtrim(char *s) {
    char *p = s; while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t len = strlen(s); while (len > 0 && isspace((unsigned char)s[len - 1])) s[--len] = '\0';
}

static void init_iter_params(IterParams *p) {
    p->num_phases = 1; p->repeats = 1;
    for (int i = 0; i < MAX_PHASES; ++i) {
        p->phases[i].start_mV = -5000; p->phases[i].end_mV = 5000;
        p->phases[i].step_mV = 100; p->phases[i].period_ms = 100;
        p->phases[i].settle_ms = 50; p->phases[i].pause_ms = 0;
    }
    for (int i = 0; i < CHANNELS; i++) { p->ch_k[i] = 1.0f; p->ch_b[i] = 0.0f; }
    /* Defaults */
    p->calc_k_sum = 1.0f; p->calc_b_sum = 0.0f;
    p->calc_alpha_c = 0.5f; /* Default Alpha = 0.5 */
    p->calc_deadband_acid = 0.1f; p->calc_deadband_alkali = 0.1f;
    p->calc_ph_neutral = 7.0f;
    p->calc_k_acid = 1.0f; p->calc_b_acid = 0.0f;
    p->calc_k_alkali = 1.0f; p->calc_b_alkali = 0.0f;
    p->calc_alpha_redox1 = 0.5f; /* Default Alpha = 0.5 */
    p->calc_alpha_redox2 = 0.5f;
    p->tok1_k = 1.0f; /* Default = 1.0 (как раньше) */
    p->tok2_k = 1.0f; /* Default = 1.0 */
    /* Analog Outputs defaults (OFF) */
    p->ao1_source = 0.0f; p->ao1_min = 0.0f; p->ao1_max = 14.0f;
    p->ao2_source = 0.0f; p->ao2_min = 0.0f; p->ao2_max = 1000.0f;
    p->ao3_source = 0.0f; p->ao3_min = -500.0f; p->ao3_max = 500.0f;
    /* RTU defaults */
    strncpy(p->rtu_port, RTU_PORT, sizeof(p->rtu_port) - 1);
    p->rtu_port[sizeof(p->rtu_port) - 1] = '\0';
    p->rtu_baud = RTU_BAUD;
    p->rtu_parity = RTU_PARITY;
    p->rtu_data_bit = RTU_DATA_BIT;
    p->rtu_stop_bit = RTU_STOP_BIT;
    p->rtu_slave_id = SLAVE_ID;
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
static int parse_int(const char *s, int *out) {
    if (!s || !out) return -1;
    char *endptr = NULL;
    long v = strtol(s, &endptr, 10);
    if (endptr == s) return -1;
    *out = (int)v;
    return 0;
}

static int load_iter_params(const char *path, IterParams *p, int *parsed_values) {
    FILE *fp = fopen(path, "r"); init_iter_params(p); if (!fp) return -1;
    int parsed = 0; char line[256];
    while (fgets(line, sizeof(line), fp)) {
        if (strlen(line) == sizeof(line) - 1 && line[sizeof(line) - 2] != '\n') { int c; while ((c = fgetc(fp)) != EOF && c != '\n'); continue; }
        strtrim(line); if (line[0] == '\0' || line[0] == '#') continue;
        char *eq = strchr(line, '='); if (!eq) continue; *eq = '\0';
        char *key = line; char *val = eq + 1; strtrim(key); strtrim(val);
        
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
            parsed++; continue;
        }
        /* FIX: tok1_k и tok2_k вынесены из блока calc_ */
        if (strcmp(key, "tok1_k") == 0) { p->tok1_k = strtof(val, NULL); parsed++; continue; }
        if (strcmp(key, "tok2_k") == 0) { p->tok2_k = strtof(val, NULL); parsed++; continue; }
        
        /* v25: Analog Outputs (ao1_*, ao2_*, ao3_*) */
        if (strncmp(key, "ao", 2) == 0 && isdigit(key[2])) {
            int ao_idx = key[2] - '1';
            if (ao_idx == 0) {
                if (strcmp(key + 3, "_source") == 0) { p->ao1_source = strtof(val, NULL); parsed++; continue; }
                if (strcmp(key + 3, "_min") == 0) { p->ao1_min = strtof(val, NULL); parsed++; continue; }
                if (strcmp(key + 3, "_max") == 0) { p->ao1_max = strtof(val, NULL); parsed++; continue; }
            } else if (ao_idx == 1) {
                if (strcmp(key + 3, "_source") == 0) { p->ao2_source = strtof(val, NULL); parsed++; continue; }
                if (strcmp(key + 3, "_min") == 0) { p->ao2_min = strtof(val, NULL); parsed++; continue; }
                if (strcmp(key + 3, "_max") == 0) { p->ao2_max = strtof(val, NULL); parsed++; continue; }
            } else if (ao_idx == 2) {
                if (strcmp(key + 3, "_source") == 0) { p->ao3_source = strtof(val, NULL); parsed++; continue; }
                if (strcmp(key + 3, "_min") == 0) { p->ao3_min = strtof(val, NULL); parsed++; continue; }
                if (strcmp(key + 3, "_max") == 0) { p->ao3_max = strtof(val, NULL); parsed++; continue; }
            }
        }
        
        if (strncmp(key, "ch", 2) == 0 && isdigit(key[2])) {
            int ch = key[2] - '1';
            if (ch >= 0 && ch < CHANNELS) {
                if (strcmp(key + 3, "_k") == 0) { p->ch_k[ch] = strtof(val, NULL); parsed++; continue; }
                if (strcmp(key + 3, "_b") == 0) { p->ch_b[ch] = strtof(val, NULL); parsed++; continue; }
            }
        }
        /* Modbus RTU Settings */
        if (strcmp(key, "rtu_port") == 0) {
            strncpy(p->rtu_port, val, sizeof(p->rtu_port) - 1);
            p->rtu_port[sizeof(p->rtu_port) - 1] = '\0';
            parsed++; continue;
        }
        if (strcmp(key, "rtu_baud") == 0) { 
            int baud = atoi(val);
            if (baud > 0) p->rtu_baud = baud;
            parsed++; continue;
        }
        if (strcmp(key, "rtu_parity") == 0) {
            if (val[0] == 'N' || val[0] == 'E' || val[0] == 'O') p->rtu_parity = val[0];
            parsed++; continue;
        }
        if (strcmp(key, "rtu_data_bit") == 0) {
            int db = atoi(val);
            if (db >= 5 && db <= 8) p->rtu_data_bit = db;
            parsed++; continue;
        }
        if (strcmp(key, "rtu_stop_bit") == 0) {
            int sb = atoi(val);
            if (sb == 1 || sb == 2) p->rtu_stop_bit = sb;
            parsed++; continue;
        }
        if (strcmp(key, "rtu_slave_id") == 0) {
            int id = atoi(val);
            if (id >= 1 && id <= 247) p->rtu_slave_id = id;
            parsed++; continue;
        }
        
        int phase_idx = 0; const char *suffix = NULL;
        parse_phase_key(key, &phase_idx, &suffix); int v = 0;
        if (strcmp(suffix, "start_mV") == 0 && parse_int(val, &v) == 0) { p->phases[phase_idx].start_mV = v; update_phase_count(p, phase_idx); parsed++; }
        else if (strcmp(suffix, "end_mV") == 0 && parse_int(val, &v) == 0) { p->phases[phase_idx].end_mV = v; update_phase_count(p, phase_idx); parsed++; }
        else if (strcmp(suffix, "step_mV") == 0 && parse_int(val, &v) == 0) { p->phases[phase_idx].step_mV = v; update_phase_count(p, phase_idx); parsed++; }
        else if (strcmp(suffix, "period_ms") == 0 && parse_int(val, &v) == 0) { p->phases[phase_idx].period_ms = v; update_phase_count(p, phase_idx); parsed++; }
        else if (strcmp(suffix, "settle_ms") == 0 && parse_int(val, &v) == 0) { p->phases[phase_idx].settle_ms = v; update_phase_count(p, phase_idx); parsed++; }
        else if (strcmp(suffix, "pause_ms") == 0 && parse_int(val, &v) == 0) { p->phases[phase_idx].pause_ms = v; update_phase_count(p, phase_idx); parsed++; }
        else if (strcmp(key, "repeats") == 0) { long rep = strtol(val, NULL, 10); p->repeats = (rep == 0 || rep == -1) ? rep : (rep < 0 ? 1 : rep); parsed++; }
        else if (strcmp(key, "phases") == 0) { int np = 0; if (parse_int(val, &np) == 0 && np >= 1 && np <= MAX_PHASES) p->num_phases = np; parsed++; }
    }
    if (parsed_values) *parsed_values = parsed;
    fclose(fp); return 0;
}

static int save_iter_params(const char *path, const IterParams *p) {
    char tmp_path[512];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
    FILE *fp = fopen(tmp_path, "w"); if (!fp) return -1;
    fprintf(fp, "# Iteration Params\nrepeats=%ld\nphases=%d\n\n", p->repeats, p->num_phases);
    for (int i = 0; i < p->num_phases; ++i) {
        const char *prefix = (i == 0) ? "" : (char[]){'s','t','e','p',(char)('1' + i), '_', '\0'};
        fprintf(fp, "%sstart_mV=%d\n%send_mV=%d\n%sstep_mV=%d\n%speriod_ms=%d\n%ssettle_ms=%d\n%spause_ms=%d\n\n",
            prefix, p->phases[i].start_mV, prefix, p->phases[i].end_mV, prefix, p->phases[i].step_mV,
            prefix, p->phases[i].period_ms, prefix, p->phases[i].settle_ms, prefix, p->phases[i].pause_ms);
    }
    fprintf(fp, "# Channel Scaling (200+)\n");
    for (int i = 0; i < CHANNELS; i++) { fprintf(fp, "ch%d_k=%.4f\nch%d_b=%.4f\n", i+1, p->ch_k[i], i+1, p->ch_b[i]); }
    
    fprintf(fp, "\n# Chemistry Params (400+)\n");
    fprintf(fp, "calc_k_sum=%.4f\ncalc_b_sum=%.4f\n", p->calc_k_sum, p->calc_b_sum);
    fprintf(fp, "calc_alpha_c=%.4f\n", p->calc_alpha_c);
    fprintf(fp, "calc_deadband_acid=%.4f\ncalc_deadband_alkali=%.4f\n", p->calc_deadband_acid, p->calc_deadband_alkali);
    fprintf(fp, "calc_ph_neutral=%.4f\n", p->calc_ph_neutral);
    fprintf(fp, "calc_k_acid=%.4f\ncalc_b_acid=%.4f\n", p->calc_k_acid, p->calc_b_acid);
    fprintf(fp, "calc_k_alkali=%.4f\ncalc_b_alkali=%.4f\n", p->calc_k_alkali, p->calc_b_alkali);
    fprintf(fp, "calc_alpha_redox1=%.4f\n", p->calc_alpha_redox1);
    fprintf(fp, "calc_alpha_redox2=%.4f\n", p->calc_alpha_redox2);
    fprintf(fp, "tok1_k=%.4f\n", p->tok1_k);
    fprintf(fp, "tok2_k=%.4f\n", p->tok2_k);
    
    fprintf(fp, "\n# Analog Outputs 4-20 mA (AO1-AO3)\n");
    fprintf(fp, "# source: 0=OFF, 1=pH, 2=C, 3=R1, 4=R2\n");
    fprintf(fp, "ao1_source=%.0f\n", p->ao1_source);
    fprintf(fp, "ao1_min=%.4f\n", p->ao1_min);
    fprintf(fp, "ao1_max=%.4f\n", p->ao1_max);
    fprintf(fp, "ao2_source=%.0f\n", p->ao2_source);
    fprintf(fp, "ao2_min=%.4f\n", p->ao2_min);
    fprintf(fp, "ao2_max=%.4f\n", p->ao2_max);
    fprintf(fp, "ao3_source=%.0f\n", p->ao3_source);
    fprintf(fp, "ao3_min=%.4f\n", p->ao3_min);
    fprintf(fp, "ao3_max=%.4f\n", p->ao3_max);
    
    fprintf(fp, "\n# Modbus RTU Settings (RS485 - панель оператора)\n");
    fprintf(fp, "rtu_port=%s\n", p->rtu_port);
    fprintf(fp, "rtu_baud=%d\n", p->rtu_baud);
    fprintf(fp, "rtu_parity=%c\n", p->rtu_parity);
    fprintf(fp, "rtu_data_bit=%d\n", p->rtu_data_bit);
    fprintf(fp, "rtu_stop_bit=%d\n", p->rtu_stop_bit);
    fprintf(fp, "rtu_slave_id=%d\n", p->rtu_slave_id);
    
    fclose(fp);
    if (rename(tmp_path, path) != 0) {
        remove(tmp_path);
        return -1;
    }
    return 0;
}

static void int32_to_regs(int32_t v, uint16_t *regs) { uint32_t u = (uint32_t)v; regs[0] = (uint16_t)((u >> 16) & 0xFFFFu); regs[1] = (uint16_t)(u & 0xFFFFu); }
static int32_t regs_to_int32(const uint16_t *regs) { return (int32_t)(((uint32_t)regs[0] << 16) | (uint32_t)regs[1]); }
static void float_to_regs(float v, uint16_t *regs) { union { float f; uint32_t u; } conv; conv.f = v; regs[0] = (uint16_t)((conv.u >> 16) & 0xFFFFu); regs[1] = (uint16_t)(conv.u & 0xFFFFu); }
static float regs_to_float(const uint16_t *regs) { union { float f; uint32_t u; } conv; conv.u = ((uint32_t)regs[0] << 16) | (uint32_t)regs[1]; return conv.f; }
static float control_bits_to_float(uint16_t control_bits) { if (control_bits & CMD_RESTART) return 3.0f; if (control_bits & CMD_STOP) return 2.0f; if (control_bits & CMD_START) return 1.0f; return 0.0f; }

/*
 * Функция синхронизации зеркальных регистров (Internal -> Legacy).
 * Вызывается постоянно, чтобы данные от воркера попадали в Legacy-область.
 */
static void sync_legacy_registers(uint16_t *regs) {
    /* FIX v13: Постоянное обновление зеркальных регистров */
    
    /* Настройки (копируем на случай если они изменились изнутри) */
    memcpy(&regs[0x4045], &regs[402], 2 * sizeof(uint16_t));  // calc_b_sum
    memcpy(&regs[0x4049], &regs[220], 2 * sizeof(uint16_t));  // ch3_b
    memcpy(&regs[0x4043], &regs[400], 2 * sizeof(uint16_t));  // calc_k_sum
    memcpy(&regs[0x4047], &regs[204], 2 * sizeof(uint16_t));  // ch3_k
    memcpy(&regs[0x4037], &regs[404], 2 * sizeof(uint16_t));  // calc_alpha_c
    memcpy(&regs[0x4039], &regs[420], 2 * sizeof(uint16_t));  // calc_alpha_redox1
    /* tok1_k (424) и tok2_k (426) пока не мапятся в Legacy, так как нет адресов */
    
    memcpy(&regs[0x402D], &regs[408], 2 * sizeof(uint16_t));  // deadband_alkali
    memcpy(&regs[0x4025], &regs[406], 2 * sizeof(uint16_t));  // deadband_acid
    memcpy(&regs[0x4035], &regs[418], 2 * sizeof(uint16_t));  // b_alkali
    memcpy(&regs[0x403D], &regs[414], 2 * sizeof(uint16_t));  // b_acid
    memcpy(&regs[0x404B], &regs[410], 2 * sizeof(uint16_t));  // ph_neutral

    /* Результаты (обновляются воркером) */
    memcpy(&regs[0x404F], &regs[4006], 2 * sizeof(uint16_t)); // C_alkali
    memcpy(&regs[0x4051], &regs[4004], 2 * sizeof(uint16_t)); // C_acid
    memcpy(&regs[0x4031], &regs[4012], 2 * sizeof(uint16_t)); // Redox1_avg
    memcpy(&regs[0x404D], &regs[4008], 2 * sizeof(uint16_t)); // pH
    
    /* Снимки фаз */
    memcpy(&regs[0x401B], &regs[3004], 2 * sizeof(uint16_t)); // Ph1_Ch3
    memcpy(&regs[0x401D], &regs[3020], 2 * sizeof(uint16_t)); // Ph2_Ch3
    
    /* Текущие значения (самое быстрое обновление) */
    memcpy(&regs[0x4013], &regs[1000], 2 * sizeof(uint16_t)); // Calc_Ch1
}

static void params_to_registers(const IterParams *p, uint16_t *regs, int reg_count) {
    if (reg_count < TOTAL_REGS) return;
    
    /* Сохраняем значения runtime-регистров, чтобы не затереть нулями */
    uint16_t ctrl_backup[CONTROL_REG_COUNT]; memcpy(ctrl_backup, &regs[CONTROL_REG_ADDR], sizeof(ctrl_backup));
    uint16_t results_backup[RESULTS_COUNT]; memcpy(results_backup, &regs[RESULTS_START], sizeof(results_backup));
    uint16_t phase_results_backup[PHASE_RESULTS_COUNT]; memcpy(phase_results_backup, &regs[PHASE_RESULTS_START], sizeof(phase_results_backup));
    uint16_t chem_results_backup[CHEM_RESULTS_COUNT]; memcpy(chem_results_backup, &regs[CHEM_RESULTS_START], sizeof(chem_results_backup));

    /* ВАЖНО: memset может очистить "дырки" в памяти, которые не описаны в params. 
       Если в системе есть другие важные регистры, лучше обновлять точечно, а не делать memset.
       Пока оставляем как есть, так как структура памяти выглядит плотной для params. */
    memset(regs, 0, sizeof(uint16_t) * reg_count);

    int32_to_regs(1, &regs[0]); int32_to_regs((int32_t)p->repeats, &regs[2]); int32_to_regs((int32_t)p->num_phases, &regs[4]);
    /* Phases INT */
    for (int i = 0; i < MAX_PHASES; ++i) {
        int base = INT_HEADER_REGS + i * INT_PHASE_REGS_PER_PHASE;
        int32_to_regs((int32_t)p->phases[i].start_mV, &regs[base+0]); int32_to_regs((int32_t)p->phases[i].end_mV, &regs[base+2]);
        int32_to_regs((int32_t)p->phases[i].step_mV, &regs[base+4]); int32_to_regs((int32_t)p->phases[i].period_ms, &regs[base+6]);
        int32_to_regs((int32_t)p->phases[i].settle_ms, &regs[base+8]); int32_to_regs((int32_t)p->phases[i].pause_ms, &regs[base+10]);
    }
    /* Phases FLOAT */
    int float_base = FLOAT_BASE;
    float_to_regs(1.0f, &regs[float_base+0]); float_to_regs((float)p->repeats, &regs[float_base+2]); float_to_regs((float)p->num_phases, &regs[float_base+4]);
    for (int i = 0; i < MAX_PHASES; ++i) {
        int base = float_base + FLOAT_HEADER_REGS + i * FLOAT_PHASE_REGS_PER_PHASE;
        float_to_regs((float)p->phases[i].start_mV, &regs[base+0]); float_to_regs((float)p->phases[i].end_mV, &regs[base+2]);
        float_to_regs((float)p->phases[i].step_mV, &regs[base+4]); float_to_regs((float)p->phases[i].period_ms, &regs[base+6]);
        float_to_regs((float)p->phases[i].settle_ms, &regs[base+8]); float_to_regs((float)p->phases[i].pause_ms, &regs[base+10]);
    }
    /* 200+: K, B */
    for (int i = 0; i < CHANNELS; i++) {
        float_to_regs(p->ch_k[i], &regs[CALC_SETTINGS_START + i * 2]);
        float_to_regs(p->ch_b[i], &regs[CALC_SETTINGS_START + 16 + i * 2]);
    }
    /* 400+: Chemistry */
    float_to_regs(p->calc_k_sum, &regs[CHEM_SETTINGS_START + 0]);
    float_to_regs(p->calc_b_sum, &regs[CHEM_SETTINGS_START + 2]);
    float_to_regs(p->calc_alpha_c, &regs[CHEM_SETTINGS_START + 4]);
    float_to_regs(p->calc_deadband_acid, &regs[CHEM_SETTINGS_START + 6]);
    float_to_regs(p->calc_deadband_alkali, &regs[CHEM_SETTINGS_START + 8]);
    float_to_regs(p->calc_ph_neutral, &regs[CHEM_SETTINGS_START + 10]);
    float_to_regs(p->calc_k_acid, &regs[CHEM_SETTINGS_START + 12]);
    float_to_regs(p->calc_b_acid, &regs[CHEM_SETTINGS_START + 14]);
    float_to_regs(p->calc_k_alkali, &regs[CHEM_SETTINGS_START + 16]);
    float_to_regs(p->calc_b_alkali, &regs[CHEM_SETTINGS_START + 18]);
    float_to_regs(p->calc_alpha_redox1, &regs[CHEM_SETTINGS_START + 20]); 
    float_to_regs(p->calc_alpha_redox2, &regs[CHEM_SETTINGS_START + 22]);
    float_to_regs(p->tok1_k, &regs[CHEM_SETTINGS_START + 24]); /* 424 */
    float_to_regs(p->tok2_k, &regs[CHEM_SETTINGS_START + 26]); /* 426 */
    
    /* v25: Analog Outputs (430-447) - все FLOAT */
    /* AO1: 430-431=source, 432-433=min, 434-435=max */
    float_to_regs(p->ao1_source, &regs[CHEM_SETTINGS_START + 30]); /* 430 */
    float_to_regs(p->ao1_min, &regs[CHEM_SETTINGS_START + 32]);    /* 432 */
    float_to_regs(p->ao1_max, &regs[CHEM_SETTINGS_START + 34]);    /* 434 */
    /* AO2: 436-437=source, 438-439=min, 440-441=max */
    float_to_regs(p->ao2_source, &regs[CHEM_SETTINGS_START + 36]); /* 436 */
    float_to_regs(p->ao2_min, &regs[CHEM_SETTINGS_START + 38]);    /* 438 */
    float_to_regs(p->ao2_max, &regs[CHEM_SETTINGS_START + 40]);    /* 440 */
    /* AO3: 442-443=source, 444-445=min, 446-447=max */
    float_to_regs(p->ao3_source, &regs[CHEM_SETTINGS_START + 42]); /* 442 */
    float_to_regs(p->ao3_min, &regs[CHEM_SETTINGS_START + 44]);    /* 444 */
    float_to_regs(p->ao3_max, &regs[CHEM_SETTINGS_START + 46]);    /* 446 */

    memcpy(&regs[CONTROL_REG_ADDR], ctrl_backup, sizeof(ctrl_backup));
    memcpy(&regs[RESULTS_START], results_backup, sizeof(results_backup));
    memcpy(&regs[PHASE_RESULTS_START], phase_results_backup, sizeof(phase_results_backup));
    memcpy(&regs[CHEM_RESULTS_START], chem_results_backup, sizeof(chem_results_backup));

    /* FIX v12: Legacy Mirroring (копируем новые значения в старые адреса для SCADA) */
    /* 402 -> 0x4045 (calc_b_sum) */
    memcpy(&regs[0x4045], &regs[402], 2 * sizeof(uint16_t));
    /* 220 -> 0x4049 (ch3_b) */
    memcpy(&regs[0x4049], &regs[220], 2 * sizeof(uint16_t));
    /* 4006 -> 0x404F (C_alkali) */
    memcpy(&regs[0x404F], &regs[4006], 2 * sizeof(uint16_t));
    /* 4004 -> 0x4051 (C_acid) */
    memcpy(&regs[0x4051], &regs[4004], 2 * sizeof(uint16_t));
    /* 400 -> 0x4043 (calc_k_sum) */
    memcpy(&regs[0x4043], &regs[400], 2 * sizeof(uint16_t));
    /* 204 -> 0x4047 (ch3_k) */
    memcpy(&regs[0x4047], &regs[204], 2 * sizeof(uint16_t));
    /* 404 -> 0x4037 (calc_alpha_c) */
    memcpy(&regs[0x4037], &regs[404], 2 * sizeof(uint16_t));
    /* 420 -> 0x4039 (calc_alpha_redox1) */
    memcpy(&regs[0x4039], &regs[420], 2 * sizeof(uint16_t));
    /* 4012 -> 0x4031 (Redox1_avg) */
    memcpy(&regs[0x4031], &regs[4012], 2 * sizeof(uint16_t));
    /* 4008 -> 0x404D (pH) */
    memcpy(&regs[0x404D], &regs[4008], 2 * sizeof(uint16_t));
    /* 408 -> 0x402D (calc_deadband_alkali) */
    memcpy(&regs[0x402D], &regs[408], 2 * sizeof(uint16_t));
    /* 406 -> 0x4025 (calc_deadband_acid) */
    memcpy(&regs[0x4025], &regs[406], 2 * sizeof(uint16_t));
    /* 418 -> 0x4035 (calc_b_alkali) */
    memcpy(&regs[0x4035], &regs[418], 2 * sizeof(uint16_t));
    /* 414 -> 0x403D (calc_b_acid) */
    memcpy(&regs[0x403D], &regs[414], 2 * sizeof(uint16_t));
    /* 410 -> 0x404B (calc_ph_neutral) */
    memcpy(&regs[0x404B], &regs[410], 2 * sizeof(uint16_t));
    /* 3004 -> 0x401B (Ph1_Ch3) */
    memcpy(&regs[0x401B], &regs[3004], 2 * sizeof(uint16_t));
    /* 3020 -> 0x401D (Ph2_Ch3) */
    memcpy(&regs[0x401D], &regs[3020], 2 * sizeof(uint16_t));
    
    /* FIX v12.1: 1000 -> 0x4013 (Calc_Ch1) */
    memcpy(&regs[0x4013], &regs[1000], 2 * sizeof(uint16_t));
}

static int read_file_mtime(const char *path, time_t *out) { struct stat st; if (stat(path, &st) == -1) return -1; *out = st.st_mtime; return 0; }

static void registers_to_params(const uint16_t *regs, IterParams *p, bool use_float) {
    if (use_float) {
        /* Чтение из FLOAT области (66+) */
        p->repeats = (long)regs_to_float(&regs[FLOAT_BASE + 2]); 
        if (p->repeats < 0 && p->repeats != -1) p->repeats = 1;
        
        p->num_phases = (int)regs_to_float(&regs[FLOAT_BASE + 4]); 
        if (p->num_phases < 1) p->num_phases = 1; 
        if (p->num_phases > MAX_PHASES) p->num_phases = MAX_PHASES;

        for (int i = 0; i < MAX_PHASES; ++i) {
            int base = FLOAT_BASE + FLOAT_HEADER_REGS + i * FLOAT_PHASE_REGS_PER_PHASE;
            p->phases[i].start_mV = (int)regs_to_float(&regs[base+0]); 
            p->phases[i].end_mV = (int)regs_to_float(&regs[base+2]);
            p->phases[i].step_mV = (int)regs_to_float(&regs[base+4]); 
            p->phases[i].period_ms = (int)regs_to_float(&regs[base+6]);
            p->phases[i].settle_ms = (int)regs_to_float(&regs[base+8]); 
            p->phases[i].pause_ms = (int)regs_to_float(&regs[base+10]);
        }
    } else {
        /* Чтение из INT области (0+) - Стандартное поведение */
        p->repeats = (long)regs_to_int32(&regs[2]); 
        if (p->repeats < 0 && p->repeats != -1) p->repeats = 1;
        
        p->num_phases = (int)regs_to_int32(&regs[4]); 
        if (p->num_phases < 1) p->num_phases = 1; 
        if (p->num_phases > MAX_PHASES) p->num_phases = MAX_PHASES;

        for (int i = 0; i < MAX_PHASES; ++i) {
            int base = INT_HEADER_REGS + i * INT_PHASE_REGS_PER_PHASE;
            p->phases[i].start_mV = (int)regs_to_int32(&regs[base+0]); 
            p->phases[i].end_mV = (int)regs_to_int32(&regs[base+2]);
            p->phases[i].step_mV = (int)regs_to_int32(&regs[base+4]); 
            p->phases[i].period_ms = (int)regs_to_int32(&regs[base+6]);
            p->phases[i].settle_ms = (int)regs_to_int32(&regs[base+8]); 
            p->phases[i].pause_ms = (int)regs_to_int32(&regs[base+10]);
        }
    }

    /* Остальные параметры (Calc/Chem) уникальны и всегда читаются одинаково */
    for (int i = 0; i < CHANNELS; i++) {
        p->ch_k[i] = regs_to_float(&regs[CALC_SETTINGS_START + i * 2]);
        p->ch_b[i] = regs_to_float(&regs[CALC_SETTINGS_START + 16 + i * 2]);
    }
    p->calc_k_sum = regs_to_float(&regs[CHEM_SETTINGS_START + 0]);
    p->calc_b_sum = regs_to_float(&regs[CHEM_SETTINGS_START + 2]);
    p->calc_alpha_c = regs_to_float(&regs[CHEM_SETTINGS_START + 4]);
    p->calc_deadband_acid = regs_to_float(&regs[CHEM_SETTINGS_START + 6]);
    p->calc_deadband_alkali = regs_to_float(&regs[CHEM_SETTINGS_START + 8]);
    p->calc_ph_neutral = regs_to_float(&regs[CHEM_SETTINGS_START + 10]);
    p->calc_k_acid = regs_to_float(&regs[CHEM_SETTINGS_START + 12]);
    p->calc_b_acid = regs_to_float(&regs[CHEM_SETTINGS_START + 14]);
    p->calc_k_alkali = regs_to_float(&regs[CHEM_SETTINGS_START + 16]);
    p->calc_b_alkali = regs_to_float(&regs[CHEM_SETTINGS_START + 18]);
    p->calc_alpha_redox1 = regs_to_float(&regs[CHEM_SETTINGS_START + 20]);
    p->calc_alpha_redox2 = regs_to_float(&regs[CHEM_SETTINGS_START + 22]);
    p->tok1_k = regs_to_float(&regs[CHEM_SETTINGS_START + 24]);
    p->tok2_k = regs_to_float(&regs[CHEM_SETTINGS_START + 26]);
    
    /* v25: Analog Outputs (430-447) - все FLOAT */
    p->ao1_source = regs_to_float(&regs[CHEM_SETTINGS_START + 30]);
    p->ao1_min = regs_to_float(&regs[CHEM_SETTINGS_START + 32]);
    p->ao1_max = regs_to_float(&regs[CHEM_SETTINGS_START + 34]);
    p->ao2_source = regs_to_float(&regs[CHEM_SETTINGS_START + 36]);
    p->ao2_min = regs_to_float(&regs[CHEM_SETTINGS_START + 38]);
    p->ao2_max = regs_to_float(&regs[CHEM_SETTINGS_START + 40]);
    p->ao3_source = regs_to_float(&regs[CHEM_SETTINGS_START + 42]);
    p->ao3_min = regs_to_float(&regs[CHEM_SETTINGS_START + 44]);
    p->ao3_max = regs_to_float(&regs[CHEM_SETTINGS_START + 46]);
}

static bool write_hits_block(int start, int count, int b_start, int b_size) { return (start + count - 1 >= b_start && start <= b_start + b_size - 1); }

/*
 * Новая функция: обрабатывает запись в регистры, но НЕ пишет на диск сразу.
 * Возвращает true, если настройки изменились и нужно запланировать сохранение.
 */
static bool process_modbus_write(int start_reg, int reg_count, modbus_mapping_t *mapping, IterParams *current_params) {
    bool hit_iter_int = write_hits_block(start_reg, reg_count, 0, FLOAT_BASE);
    bool hit_iter_float = write_hits_block(start_reg, reg_count, FLOAT_BASE, FLOAT_HOLDING_REG_COUNT);
    bool hit_calc = write_hits_block(start_reg, reg_count, CALC_SETTINGS_START, CALC_SETTINGS_END - CALC_SETTINGS_START);
    bool hit_ctrl = write_hits_block(start_reg, reg_count, CONTROL_REG_ADDR, CONTROL_REG_COUNT);
    /* FIX v12: Проверка записи в Legacy область (0x4000+) */
    bool hit_legacy = write_hits_block(start_reg, reg_count, 0x4000, 1000); 
    
    bool settings_changed = false;

    // 1. Обработка команд управления (сразу, без задержек)
    if (hit_ctrl) {
        uint16_t *c_regs = &mapping->tab_registers[CONTROL_REG_ADDR];
        float cmd = regs_to_float(c_regs);
        uint16_t bits = 0;
        if (fabsf(cmd - 1.0f) < 0.001f) bits = CMD_START; else if (fabsf(cmd - 2.0f) < 0.001f) bits = CMD_STOP; else if (fabsf(cmd - 3.0f) < 0.001f) bits = CMD_RESTART;
        if (bits == 0 && c_regs[0] != 0) bits = c_regs[0];
        
        if (bits != 0) {
            mapping->tab_registers[CONTROL_REG_ADDR] = bits; mapping->tab_registers[CONTROL_REG_ADDR+1] = 0;
            float_to_regs(control_bits_to_float(bits), &mapping->tab_registers[CONTROL_REG_ADDR]);
            printf("Command received: 0x%04X\n", bits);
        } else { 
            mapping->tab_registers[CONTROL_REG_ADDR] = 0; mapping->tab_registers[CONTROL_REG_ADDR+1] = 0; 
        }
        // Изменение командного регистра не требует сохранения в файл
    }

    // 2. Обработка настроек (Internal)
    if (hit_iter_int || hit_iter_float || hit_calc) {
        // Определяем источник правды: если запись затронула FLOAT область, читаем оттуда
        bool use_float = hit_iter_float;
        
        // Читаем из Modbus регистров в структуру C
        registers_to_params(mapping->tab_registers, current_params, use_float);
        
        
        // Сразу обновляем регистры обратно из структуры (синхронизация INT <-> FLOAT и Legacy)
        params_to_registers(current_params, mapping->tab_registers, mapping->nb_registers);
        
        settings_changed = true; 
        printf("Settings changed in memory (Reg start: %d, Source: %s). Write pending...\n", start_reg, use_float ? "FLOAT" : "INT");
    }

    // 3. Обработка настроек (Legacy Mapping: Legacy -> Internal)
    if (hit_legacy) {
        // Если записали в старые адреса -> переносим в новые (структуру)
        
        // Пример: Запись в 0x4045 (calc_b_sum)
        if (write_hits_block(start_reg, reg_count, 0x4045, 2)) {
            current_params->calc_b_sum = regs_to_float(&mapping->tab_registers[0x4045]);
        }
        if (write_hits_block(start_reg, reg_count, 0x4049, 2)) {
            // В таблице написано "220", это ch11_b (или ch3_b?), нужно уточнение. 
            // Предположим ch3_b (т.к. 220 это ch11_b в полном списке 8 каналов... стоп, 220 это 200 + 2*10? Нет.)
            // 200=ch1_k, 202=ch2_k ... 214=ch8_k. 216=ch1_b ... 230=ch8_b.
            // Адрес 220 (DEC) = 0xDC. Это ch3_b (216 + 2*2 = 220). Да, это Канал 3 Смещение.
            current_params->ch_b[2] = regs_to_float(&mapping->tab_registers[0x4049]);
        }
        if (write_hits_block(start_reg, reg_count, 0x4043, 2)) {
            current_params->calc_k_sum = regs_to_float(&mapping->tab_registers[0x4043]);
        }
        if (write_hits_block(start_reg, reg_count, 0x4047, 2)) {
            // 204 = 0xCC = ch3_k (Канал 3 Коэф)
            current_params->ch_k[2] = regs_to_float(&mapping->tab_registers[0x4047]);
        }
        if (write_hits_block(start_reg, reg_count, 0x4037, 2)) {
            current_params->calc_alpha_c = regs_to_float(&mapping->tab_registers[0x4037]);
        }
        if (write_hits_block(start_reg, reg_count, 0x4039, 2)) {
            current_params->calc_alpha_redox1 = regs_to_float(&mapping->tab_registers[0x4039]);
        }
        /* Добавляем обработку новых параметров в Internal, если бы у них были Legacy адреса (пока нет) */
        /* if (write_hits_block(start_reg, reg_count, ???, 2)) current_params->tok1_k = ... */

        if (write_hits_block(start_reg, reg_count, 0x402D, 2)) {
            current_params->calc_deadband_alkali = regs_to_float(&mapping->tab_registers[0x402D]);
        }
        if (write_hits_block(start_reg, reg_count, 0x4025, 2)) {
            current_params->calc_deadband_acid = regs_to_float(&mapping->tab_registers[0x4025]);
        }
        if (write_hits_block(start_reg, reg_count, 0x4035, 2)) {
            current_params->calc_b_alkali = regs_to_float(&mapping->tab_registers[0x4035]);
        }
        if (write_hits_block(start_reg, reg_count, 0x403D, 2)) {
            current_params->calc_b_acid = regs_to_float(&mapping->tab_registers[0x403D]);
        }
        if (write_hits_block(start_reg, reg_count, 0x404B, 2)) {
            current_params->calc_ph_neutral = regs_to_float(&mapping->tab_registers[0x404B]);
        }

        // Синхронизируем обратно во все регистры (Internal + Legacy)
        params_to_registers(current_params, mapping->tab_registers, mapping->nb_registers);
        settings_changed = true;
        printf("Legacy settings changed (Reg start: 0x%X). Write pending...\n", start_reg);
    }

    return settings_changed;
}

int main(void) {
    if (install_signal_handlers() == -1) return 1;
    IterParams params; time_t params_mtime = 0;
    
    // Переменные для отложенной записи
    bool dirty = false;
    time_t last_change_time = 0;

    load_iter_params(PARAMS_FILE, &params, NULL); read_file_mtime(PARAMS_FILE, &params_mtime);
    

    modbus_t *ctx_tcp = modbus_new_tcp("0.0.0.0", MODBUS_TCP_PORT);
    modbus_t *ctx_rtu = NULL; 

    modbus_mapping_t *mapping = modbus_mapping_new(0, 0, TOTAL_REGS, 0);
    if (!mapping) return 1;
    params_to_registers(&params, mapping->tab_registers, mapping->nb_registers);

    int server_socket = modbus_tcp_listen(ctx_tcp, MAX_CLIENTS);
    if (server_socket < 0) return 1;
    int flags = fcntl(server_socket, F_GETFL, 0); if (flags >= 0) fcntl(server_socket, F_SETFL, flags | O_NONBLOCK);

    int client_sockets[MAX_CLIENTS] = {0};
    printf("Server v9 (Delayed Save). TCP: %d\n", MODBUS_TCP_PORT);

    fd_set readfds; uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    while (!g_stop) {
        // --- Блок отложенного сохранения ---
        if (dirty) {
            time_t now = time(NULL);
            if (difftime(now, last_change_time) >= SAVE_DELAY_SEC) {
                printf("Saving dirty params to disk...\n");
                if (save_iter_params(PARAMS_FILE, &params) == 0) {
                    read_file_mtime(PARAMS_FILE, &params_mtime); // Обновляем mtime, чтобы не перечитывать свой же файл
                    dirty = false;
                    printf("Saved successfully.\n");
                } else {
                    fprintf(stderr, "Error saving params!\n");
                    // Можно сбросить dirty, чтобы не спамить ошибками, или попробовать позже
                    last_change_time = now; // Попробуем еще через SAVE_DELAY_SEC
                }
            }
        }
        // -----------------------------------

        /* FIX v13: Постоянная синхронизация данных воркера в Legacy-регистры */
        sync_legacy_registers(mapping->tab_registers);

        if (ctx_rtu == NULL) {
            ctx_rtu = modbus_new_rtu(params.rtu_port, params.rtu_baud, params.rtu_parity, params.rtu_data_bit, params.rtu_stop_bit);
            if (ctx_rtu) {
                modbus_set_slave(ctx_rtu, params.rtu_slave_id);
                if (modbus_connect(ctx_rtu) == -1) {
                    fprintf(stderr, "RTU Open Failed (Port: %s, Baud: %d, Slave: %d). Retrying...\n", 
                            params.rtu_port, params.rtu_baud, params.rtu_slave_id);
                    modbus_free(ctx_rtu); ctx_rtu = NULL; sleep(1);
                } else printf("RTU Port Opened: %s (Baud: %d, Parity: %c, Data: %d, Stop: %d, Slave: %d)\n", 
                              params.rtu_port, params.rtu_baud, params.rtu_parity, 
                              params.rtu_data_bit, params.rtu_stop_bit, params.rtu_slave_id);
            }
        }

        // Слежение за внешними изменениями файла (если файл изменил кто-то другой, а у нас нет несохраненных правок)
        if (!dirty) { 
            time_t current_mtime = 0;
            if (read_file_mtime(PARAMS_FILE, &current_mtime) == 0) {
                if (current_mtime != params_mtime) {
                    int parsed = 0; IterParams new_params = params;
                    if (load_iter_params(PARAMS_FILE, &new_params, &parsed) == 0 && parsed > 0) {
                        params = new_params; params_mtime = current_mtime;
                        params_to_registers(&params, mapping->tab_registers, mapping->nb_registers);
                        printf("Params reloaded from external file change.\n");
                    }
                }
            }
        }

        FD_ZERO(&readfds); FD_SET(server_socket, &readfds); int max_sd = server_socket;
        int rtu_fd = (ctx_rtu) ? modbus_get_socket(ctx_rtu) : -1;
        if (rtu_fd > 0) { FD_SET(rtu_fd, &readfds); if (rtu_fd > max_sd) max_sd = rtu_fd; }
        for (int i = 0; i < MAX_CLIENTS; i++) { if (client_sockets[i] > 0) { FD_SET(client_sockets[i], &readfds); if (client_sockets[i] > max_sd) max_sd = client_sockets[i]; } }

        struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 100000;
        int activity = select(max_sd + 1, &readfds, NULL, NULL, &tv);

        if ((activity < 0) && (errno != EINTR)) {
            // Ошибка Select
            continue;
        }

        // Обработка RTU
        if (rtu_fd > 0 && FD_ISSET(rtu_fd, &readfds)) {
            int rc = modbus_receive(ctx_rtu, query);
            if (rc > 0) {
                modbus_reply(ctx_rtu, query, rc, mapping);
                int func = query[1];
                if (func == MODBUS_FC_WRITE_SINGLE_REGISTER || func == MODBUS_FC_WRITE_MULTIPLE_REGISTERS) {
                    int start_reg = (query[2] << 8) | query[3];
                    int reg_count = (func == MODBUS_FC_WRITE_MULTIPLE_REGISTERS) ? ((query[4] << 8) | query[5]) : 1;
                    if (process_modbus_write(start_reg, reg_count, mapping, &params)) {
                        dirty = true;
                        last_change_time = time(NULL);
                    }
                }
            } else if (rc == -1) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    fprintf(stderr, "RTU Error. Resetting...\n");
                    modbus_close(ctx_rtu); modbus_free(ctx_rtu); ctx_rtu = NULL;
                }
            }
        }

        // Обработка TCP
        if (FD_ISSET(server_socket, &readfds)) {
            int new_socket = accept(server_socket, NULL, NULL);
            if (new_socket >= 0) {
                int saved = 0;
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (client_sockets[i] == 0) {
                        client_sockets[i] = new_socket;
                        saved = 1;
                        break;
                    }
                }
                /* FIX v11: Закрываем сокет, если все слоты заняты (предотвращение утечки) */
                if (!saved) {
                    fprintf(stderr, "Max clients (%d) reached, rejecting connection\n", MAX_CLIENTS);
                    close(new_socket);
                }
            }
        }
        for (int i = 0; i < MAX_CLIENTS; i++) {
            int sd = client_sockets[i];
            if (sd > 0 && FD_ISSET(sd, &readfds)) {
                modbus_set_socket(ctx_tcp, sd);
                int rc = modbus_receive(ctx_tcp, query);
                if (rc > 0) {
                    modbus_reply(ctx_tcp, query, rc, mapping);
                    int func = query[7];
                    if (func == MODBUS_FC_WRITE_SINGLE_REGISTER || func == MODBUS_FC_WRITE_MULTIPLE_REGISTERS) {
                        int start_reg = ((int)query[8] << 8) | (int)query[9];
                        int reg_count = (func == MODBUS_FC_WRITE_MULTIPLE_REGISTERS) ? (((int)query[10] << 8) | (int)query[11]) : 1;
                        if (process_modbus_write(start_reg, reg_count, mapping, &params)) {
                            dirty = true;
                            last_change_time = time(NULL);
                        }
                    }
                } else if (rc == -1 && errno != EAGAIN) { close(sd); client_sockets[i] = 0; } else if (rc == 0) { close(sd); client_sockets[i] = 0; }
            }
        }
    }

    // Сохранение при выходе (SIGINT/SIGTERM)
    if (dirty) {
        printf("Exiting... Saving pending changes.\n");
        save_iter_params(PARAMS_FILE, &params);
    }

    close(server_socket); modbus_mapping_free(mapping); modbus_free(ctx_tcp); if (ctx_rtu) { modbus_close(ctx_rtu); modbus_free(ctx_rtu); }
    return 0;
}