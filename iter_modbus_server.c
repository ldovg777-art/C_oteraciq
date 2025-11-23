/*
 * iter_modbus_server.c
 * Версия v8: Добавлены настройки фильтров для Redox (адреса 420, 422).
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
#define CHEM_SETTINGS_COUNT 30    /* Увеличили запас */

#define RESULTS_START       1000  
#define RESULTS_COUNT       (CHANNELS * 2) 

#define PHASE_RESULTS_START 3000  
#define PHASE_RESULTS_COUNT (MAX_PHASES * CHANNELS * 2)

#define CHEM_RESULTS_START  4000  
#define CHEM_RESULTS_COUNT  20    /* Увеличили под Redox */

#define TOTAL_REGS          4200

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
    float calc_filter_size;
    float calc_deadband_acid;
    float calc_deadband_alkali;
    float calc_ph_neutral;
    float calc_k_acid;
    float calc_b_acid;
    float calc_k_alkali;
    float calc_b_alkali;
    /* Redox Фильтры */
    float calc_filter_redox1;
    float calc_filter_redox2;
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
    p->calc_filter_size = 1.0f;
    p->calc_deadband_acid = 0.1f; p->calc_deadband_alkali = 0.1f;
    p->calc_ph_neutral = 7.0f;
    p->calc_k_acid = 1.0f; p->calc_b_acid = 0.0f;
    p->calc_k_alkali = 1.0f; p->calc_b_alkali = 0.0f;
    p->calc_filter_redox1 = 1.0f;
    p->calc_filter_redox2 = 1.0f;
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
        
        /* Парсинг Химии */
        if (strncmp(key, "calc_", 5) == 0) {
            if (strcmp(key, "calc_k_sum") == 0) p->calc_k_sum = strtof(val, NULL);
            else if (strcmp(key, "calc_b_sum") == 0) p->calc_b_sum = strtof(val, NULL);
            else if (strcmp(key, "calc_filter_size") == 0) p->calc_filter_size = strtof(val, NULL);
            else if (strcmp(key, "calc_deadband_acid") == 0) p->calc_deadband_acid = strtof(val, NULL);
            else if (strcmp(key, "calc_deadband_alkali") == 0) p->calc_deadband_alkali = strtof(val, NULL);
            else if (strcmp(key, "calc_ph_neutral") == 0) p->calc_ph_neutral = strtof(val, NULL);
            else if (strcmp(key, "calc_k_acid") == 0) p->calc_k_acid = strtof(val, NULL);
            else if (strcmp(key, "calc_b_acid") == 0) p->calc_b_acid = strtof(val, NULL);
            else if (strcmp(key, "calc_k_alkali") == 0) p->calc_k_alkali = strtof(val, NULL);
            else if (strcmp(key, "calc_b_alkali") == 0) p->calc_b_alkali = strtof(val, NULL);
            else if (strcmp(key, "calc_filter_redox1") == 0) p->calc_filter_redox1 = strtof(val, NULL);
            else if (strcmp(key, "calc_filter_redox2") == 0) p->calc_filter_redox2 = strtof(val, NULL);
            parsed++; continue;
        }

        /* Парсинг Каналов */
        if (strncmp(key, "ch", 2) == 0 && isdigit(key[2])) {
            int ch = key[2] - '1';
            if (ch >= 0 && ch < CHANNELS) {
                if (strcmp(key + 3, "_k") == 0) { p->ch_k[ch] = strtof(val, NULL); parsed++; continue; }
                if (strcmp(key + 3, "_b") == 0) { p->ch_b[ch] = strtof(val, NULL); parsed++; continue; }
            }
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
    FILE *fp = fopen(path, "w"); if (!fp) return -1;
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
    fprintf(fp, "calc_filter_size=%.0f\n", p->calc_filter_size);
    fprintf(fp, "calc_deadband_acid=%.4f\ncalc_deadband_alkali=%.4f\n", p->calc_deadband_acid, p->calc_deadband_alkali);
    fprintf(fp, "calc_ph_neutral=%.4f\n", p->calc_ph_neutral);
    fprintf(fp, "calc_k_acid=%.4f\ncalc_b_acid=%.4f\n", p->calc_k_acid, p->calc_b_acid);
    fprintf(fp, "calc_k_alkali=%.4f\ncalc_b_alkali=%.4f\n", p->calc_k_alkali, p->calc_b_alkali);
    fprintf(fp, "calc_filter_redox1=%.0f\n", p->calc_filter_redox1);
    fprintf(fp, "calc_filter_redox2=%.0f\n", p->calc_filter_redox2);
    
    fclose(fp); return 0;
}

/* Utils */
static void int32_to_regs(int32_t v, uint16_t *regs) { uint32_t u = (uint32_t)v; regs[0] = (uint16_t)((u >> 16) & 0xFFFFu); regs[1] = (uint16_t)(u & 0xFFFFu); }
static int32_t regs_to_int32(const uint16_t *regs) { return (int32_t)(((uint32_t)regs[0] << 16) | (uint32_t)regs[1]); }
static void float_to_regs(float v, uint16_t *regs) { union { float f; uint32_t u; } conv; conv.f = v; regs[0] = (uint16_t)((conv.u >> 16) & 0xFFFFu); regs[1] = (uint16_t)(conv.u & 0xFFFFu); }
static float regs_to_float(const uint16_t *regs) { union { float f; uint32_t u; } conv; conv.u = ((uint32_t)regs[0] << 16) | (uint32_t)regs[1]; return conv.f; }
static float control_bits_to_float(uint16_t control_bits) { if (control_bits & CMD_RESTART) return 3.0f; if (control_bits & CMD_STOP) return 2.0f; if (control_bits & CMD_START) return 1.0f; return 0.0f; }

static void params_to_registers(const IterParams *p, uint16_t *regs, int reg_count) {
    if (reg_count < TOTAL_REGS) return;
    
    uint16_t ctrl_backup[CONTROL_REG_COUNT]; memcpy(ctrl_backup, &regs[CONTROL_REG_ADDR], sizeof(ctrl_backup));
    uint16_t results_backup[RESULTS_COUNT]; memcpy(results_backup, &regs[RESULTS_START], sizeof(results_backup));
    uint16_t phase_results_backup[PHASE_RESULTS_COUNT]; memcpy(phase_results_backup, &regs[PHASE_RESULTS_START], sizeof(phase_results_backup));
    uint16_t chem_results_backup[CHEM_RESULTS_COUNT]; memcpy(chem_results_backup, &regs[CHEM_RESULTS_START], sizeof(chem_results_backup));

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
    float_to_regs(p->calc_filter_size, &regs[CHEM_SETTINGS_START + 4]);
    float_to_regs(p->calc_deadband_acid, &regs[CHEM_SETTINGS_START + 6]);
    float_to_regs(p->calc_deadband_alkali, &regs[CHEM_SETTINGS_START + 8]);
    float_to_regs(p->calc_ph_neutral, &regs[CHEM_SETTINGS_START + 10]);
    float_to_regs(p->calc_k_acid, &regs[CHEM_SETTINGS_START + 12]);
    float_to_regs(p->calc_b_acid, &regs[CHEM_SETTINGS_START + 14]);
    float_to_regs(p->calc_k_alkali, &regs[CHEM_SETTINGS_START + 16]);
    float_to_regs(p->calc_b_alkali, &regs[CHEM_SETTINGS_START + 18]);
    float_to_regs(p->calc_filter_redox1, &regs[CHEM_SETTINGS_START + 20]); // 420
    float_to_regs(p->calc_filter_redox2, &regs[CHEM_SETTINGS_START + 22]); // 422

    memcpy(&regs[CONTROL_REG_ADDR], ctrl_backup, sizeof(ctrl_backup));
    memcpy(&regs[RESULTS_START], results_backup, sizeof(results_backup));
    memcpy(&regs[PHASE_RESULTS_START], phase_results_backup, sizeof(phase_results_backup));
    memcpy(&regs[CHEM_RESULTS_START], chem_results_backup, sizeof(chem_results_backup));
}

static int read_file_mtime(const char *path, time_t *out) { struct stat st; if (stat(path, &st) == -1) return -1; *out = st.st_mtime; return 0; }

static void registers_to_params(const uint16_t *regs, IterParams *p) {
    p->repeats = (long)regs_to_int32(&regs[2]); if (p->repeats < 0 && p->repeats != -1) p->repeats = 1;
    p->num_phases = (int)regs_to_int32(&regs[4]); if (p->num_phases < 1) p->num_phases = 1; if (p->num_phases > MAX_PHASES) p->num_phases = MAX_PHASES;
    for (int i = 0; i < MAX_PHASES; ++i) {
        int base = INT_HEADER_REGS + i * INT_PHASE_REGS_PER_PHASE;
        p->phases[i].start_mV = (int)regs_to_int32(&regs[base+0]); p->phases[i].end_mV = (int)regs_to_int32(&regs[base+2]);
        p->phases[i].step_mV = (int)regs_to_int32(&regs[base+4]); p->phases[i].period_ms = (int)regs_to_int32(&regs[base+6]);
        p->phases[i].settle_ms = (int)regs_to_int32(&regs[base+8]); p->phases[i].pause_ms = (int)regs_to_int32(&regs[base+10]);
    }
    for (int i = 0; i < CHANNELS; i++) {
        p->ch_k[i] = regs_to_float(&regs[CALC_SETTINGS_START + i * 2]);
        p->ch_b[i] = regs_to_float(&regs[CALC_SETTINGS_START + 16 + i * 2]);
    }
    p->calc_k_sum = regs_to_float(&regs[CHEM_SETTINGS_START + 0]);
    p->calc_b_sum = regs_to_float(&regs[CHEM_SETTINGS_START + 2]);
    p->calc_filter_size = regs_to_float(&regs[CHEM_SETTINGS_START + 4]);
    p->calc_deadband_acid = regs_to_float(&regs[CHEM_SETTINGS_START + 6]);
    p->calc_deadband_alkali = regs_to_float(&regs[CHEM_SETTINGS_START + 8]);
    p->calc_ph_neutral = regs_to_float(&regs[CHEM_SETTINGS_START + 10]);
    p->calc_k_acid = regs_to_float(&regs[CHEM_SETTINGS_START + 12]);
    p->calc_b_acid = regs_to_float(&regs[CHEM_SETTINGS_START + 14]);
    p->calc_k_alkali = regs_to_float(&regs[CHEM_SETTINGS_START + 16]);
    p->calc_b_alkali = regs_to_float(&regs[CHEM_SETTINGS_START + 18]);
    p->calc_filter_redox1 = regs_to_float(&regs[CHEM_SETTINGS_START + 20]);
    p->calc_filter_redox2 = regs_to_float(&regs[CHEM_SETTINGS_START + 22]);
}

static bool write_hits_block(int start, int count, int b_start, int b_size) { return (start + count - 1 >= b_start && start <= b_start + b_size - 1); }

static void check_and_save_changes(int start_reg, int reg_count, modbus_mapping_t *mapping, IterParams *current_params, time_t *params_mtime) {
    bool hit_iter = write_hits_block(start_reg, reg_count, 0, 200);
    bool hit_calc = write_hits_block(start_reg, reg_count, CALC_SETTINGS_START, CALC_SETTINGS_END - CALC_SETTINGS_START);
    bool hit_ctrl = write_hits_block(start_reg, reg_count, CONTROL_REG_ADDR, CONTROL_REG_COUNT);

    if (hit_ctrl) {
        uint16_t *c_regs = &mapping->tab_registers[CONTROL_REG_ADDR];
        float cmd = regs_to_float(c_regs);
        uint16_t bits = 0;
        if (fabsf(cmd - 1.0f) < 0.001f) bits = CMD_START; else if (fabsf(cmd - 2.0f) < 0.001f) bits = CMD_STOP; else if (fabsf(cmd - 3.0f) < 0.001f) bits = CMD_RESTART;
        if (bits == 0 && c_regs[0] != 0) bits = c_regs[0];
        if (bits != 0) {
            mapping->tab_registers[CONTROL_REG_ADDR] = bits; mapping->tab_registers[CONTROL_REG_ADDR+1] = 0;
            float_to_regs(control_bits_to_float(bits), &mapping->tab_registers[CONTROL_REG_ADDR]);
        } else { mapping->tab_registers[CONTROL_REG_ADDR] = 0; mapping->tab_registers[CONTROL_REG_ADDR+1] = 0; }
    }
    if (hit_iter || hit_calc) {
        registers_to_params(mapping->tab_registers, current_params);
        save_iter_params(PARAMS_FILE, current_params);
        read_file_mtime(PARAMS_FILE, params_mtime);
        params_to_registers(current_params, mapping->tab_registers, mapping->nb_registers);
        printf("Settings updated and saved.\n");
    }
}

int main(void) {
    if (install_signal_handlers() == -1) return 1;
    IterParams params; time_t params_mtime = 0;
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
    printf("Server v8 (Redox Support). TCP: %d\n", MODBUS_TCP_PORT);

    fd_set readfds; uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    while (!g_stop) {
        if (ctx_rtu == NULL) {
            ctx_rtu = modbus_new_rtu(RTU_PORT, RTU_BAUD, RTU_PARITY, RTU_DATA_BIT, RTU_STOP_BIT);
            if (ctx_rtu) {
                modbus_set_slave(ctx_rtu, SLAVE_ID);
                if (modbus_connect(ctx_rtu) == -1) {
                    fprintf(stderr, "RTU Open Failed. Retrying...\n");
                    modbus_free(ctx_rtu); ctx_rtu = NULL; sleep(1);
                } else printf("RTU Port Opened: %s\n", RTU_PORT);
            }
        }

        time_t current_mtime = 0;
        if (read_file_mtime(PARAMS_FILE, &current_mtime) == 0) {
            if (current_mtime != params_mtime) {
                int parsed = 0; IterParams new_params = params;
                if (load_iter_params(PARAMS_FILE, &new_params, &parsed) == 0 && parsed > 0) {
                    params = new_params; params_mtime = current_mtime;
                    params_to_registers(&params, mapping->tab_registers, mapping->nb_registers);
                    printf("Params reloaded.\n");
                }
            }
        }

        FD_ZERO(&readfds); FD_SET(server_socket, &readfds); int max_sd = server_socket;
        int rtu_fd = (ctx_rtu) ? modbus_get_socket(ctx_rtu) : -1;
        if (rtu_fd > 0) { FD_SET(rtu_fd, &readfds); if (rtu_fd > max_sd) max_sd = rtu_fd; }
        for (int i = 0; i < MAX_CLIENTS; i++) { if (client_sockets[i] > 0) { FD_SET(client_sockets[i], &readfds); if (client_sockets[i] > max_sd) max_sd = client_sockets[i]; } }

        struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 100000;
        if (select(max_sd + 1, &readfds, NULL, NULL, &tv) < 0) continue;

        if (rtu_fd > 0 && FD_ISSET(rtu_fd, &readfds)) {
            int rc = modbus_receive(ctx_rtu, query);
            if (rc > 0) {
                // printf("[RTU] Query received\n");
                modbus_reply(ctx_rtu, query, rc, mapping);
                int func = query[1];
                if (func == MODBUS_FC_WRITE_SINGLE_REGISTER || func == MODBUS_FC_WRITE_MULTIPLE_REGISTERS) {
                    int start_reg = (query[2] << 8) | query[3];
                    int reg_count = (func == MODBUS_FC_WRITE_MULTIPLE_REGISTERS) ? ((query[4] << 8) | query[5]) : 1;
                    check_and_save_changes(start_reg, reg_count, mapping, &params, &params_mtime);
                }
            } else if (rc == -1) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    fprintf(stderr, "RTU Error. Resetting...\n");
                    modbus_close(ctx_rtu); modbus_free(ctx_rtu); ctx_rtu = NULL;
                }
            }
        }

        if (FD_ISSET(server_socket, &readfds)) {
            int new_socket = accept(server_socket, NULL, NULL);
            if (new_socket >= 0) { for(int i=0;i<MAX_CLIENTS;i++) if(client_sockets[i]==0) { client_sockets[i]=new_socket; break; } }
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
                        check_and_save_changes(start_reg, reg_count, mapping, &params, &params_mtime);
                    }
                } else if (rc == -1 && errno != EAGAIN) { close(sd); client_sockets[i] = 0; } else if (rc == 0) { close(sd); client_sockets[i] = 0; }
            }
        }
    }
    close(server_socket); modbus_mapping_free(mapping); modbus_free(ctx_tcp); if (ctx_rtu) { modbus_close(ctx_rtu); modbus_free(ctx_rtu); }
    return 0;
}