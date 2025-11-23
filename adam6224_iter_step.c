/*
 * adam6224_iter_step.c
 * Версия v13: Добавлена поддержка Redox 1 (Ch2) и Redox 2 (Ch3).
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
#include <modbus/modbus.h>

#include "adamapi.h"

#define ITER_PARAMS_FILE   "/home/root/iter_params.txt"
#define FILE_CURRENT       "/home/root/iter_current.csv"
#define FILE_PREVIOUS      "/home/root/iter_prev_full.csv"

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

#define MAX_PHASES 5
#define CHANNELS 8

typedef struct {
    int start_mV; int end_mV; int step_mV; int period_ms; int settle_ms; int pause_ms;
} IterPhase;

typedef struct {
    IterPhase phases[MAX_PHASES];
    int num_phases;
    long repeats;
    float ch_k[CHANNELS];
    float ch_b[CHANNELS];
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
    float calc_filter_redox1;
    float calc_filter_redox2;
} IterParams;

/* --- FILTER BUFFERS --- */
#define MAX_FILTER_SIZE 20

static float filter_buffer_chem[MAX_FILTER_SIZE];
static int filter_count_chem = 0;
static int filter_idx_chem = 0;

static float filter_buffer_redox1[MAX_FILTER_SIZE];
static int filter_count_redox1 = 0;
static int filter_idx_redox1 = 0;

static float filter_buffer_redox2[MAX_FILTER_SIZE];
static int filter_count_redox2 = 0;
static int filter_idx_redox2 = 0;

/* Snapshots */
static float current_cycle_snapshots[MAX_PHASES][CHANNELS];

/* Utils */
static uint16_t voltage_to_code(double v) {
    const double v_min = AO_MIN_V; const double v_max = AO_MAX_V;
    const int code_min = 0; const int code_max = 4095;
    if (v < v_min) v = v_min; if (v > v_max) v = v_max;
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
static int load_iter_params(const char *path, IterParams *p) {
    FILE *fp = fopen(path, "r"); if (!fp) { perror("Params File Error"); return -1; }
    init_iter_params(p);
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        strtrim(line); if (line[0] == '\0' || line[0] == '#') continue;
        char *eq = strchr(line, '='); if (!eq) continue; *eq = '\0';
        char *key = line; char *val = eq + 1; strtrim(key); strtrim(val);
        
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
            continue;
        }
        if (strncmp(key, "ch", 2) == 0 && isdigit(key[2])) {
            int ch = key[2] - '1';
            if (ch >= 0 && ch < CHANNELS) {
                if (strcmp(key + 3, "_k") == 0) { p->ch_k[ch] = strtof(val, NULL); continue; }
                if (strcmp(key + 3, "_b") == 0) { p->ch_b[ch] = strtof(val, NULL); continue; }
            }
        }
        if (strcmp(key, "repeats") == 0) { long r = strtol(val, NULL, 10); p->repeats = (r == 0) ? 0 : (r < 0 ? 1 : r); continue; }
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
    fclose(fp); if (p->num_phases < 1) p->num_phases = 1; return 0;
}
static int validate_iter_params(IterParams *p) { if (p->repeats < 0) p->repeats = 1; return 0; }

static volatile int g_stop = 0;
static void handle_sigint(int sig) { (void)sig; g_stop = 1; }
typedef enum { CONTROL_RUNNING = 0, CONTROL_STOPPED = 1 } ControlState;
static float regs_to_float(const uint16_t *regs) { union { float f; uint32_t u; } conv; conv.u = ((uint32_t)regs[0] << 16) | (uint32_t)regs[1]; return conv.f; }

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
        if (mask & CMD_STOP) *state = CONTROL_STOPPED; if (mask & CMD_START) *state = CONTROL_RUNNING;
        return;
    }
    if (regs[0] != 0) {
        modbus_write_register(ctx, CONTROL_REG_ADDR, 0);
        if (regs[0] & CMD_RESTART) { *restart_requested = 1; *state = CONTROL_RUNNING; }
        if (regs[0] & CMD_STOP) *state = CONTROL_STOPPED; if (regs[0] & CMD_START) *state = CONTROL_RUNNING;
    }
}

/* --- Calculation Helper --- */
static void update_filter(float val, float *buffer, int *count, int *idx, int size) {
    if (size < 1) size = 1; if (size > MAX_FILTER_SIZE) size = MAX_FILTER_SIZE;
    if (*count < size) { buffer[(*count)++] = val; }
    else { buffer[*idx] = val; *idx = (*idx + 1) % size; }
}
static float get_filter_avg(float *buffer, int count) {
    if (count == 0) return 0.0f;
    float sum = 0.0f; for (int i = 0; i < count; i++) sum += buffer[i];
    return sum / count;
}

/* --- MAIN MATH --- */
static void PerformChemistryCalculation(IterParams *par, modbus_t *ctrl_ctx) {
    /* === 1. pH (Channel 1 sum) === */
    if (par->num_phases >= 2) {
        float I_ph1 = current_cycle_snapshots[0][1];
        float I_ph2 = current_cycle_snapshots[1][1];
        float C_raw = (I_ph1 + I_ph2) * par->calc_k_sum + par->calc_b_sum;

        update_filter(C_raw, filter_buffer_chem, &filter_count_chem, &filter_idx_chem, (int)par->calc_filter_size);
        float C_filt = get_filter_avg(filter_buffer_chem, filter_count_chem);

        float C_acid = 0.0f, C_alkali = 0.0f, pH = par->calc_ph_neutral;
        if (C_filt < -fabsf(par->calc_deadband_acid)) {
            C_acid = fabsf(C_filt);
            if (C_acid > 0.000001f) pH = par->calc_k_acid * log10f(C_acid) + par->calc_b_acid;
        } else if (C_filt > fabsf(par->calc_deadband_alkali)) {
            C_alkali = C_filt;
            if (C_alkali > 0.000001f) pH = par->calc_k_alkali * log10f(C_alkali) + par->calc_b_alkali;
        }

        if (ctrl_ctx) {
            uint16_t regs[10]; union { float f; uint32_t u; } c;
            c.f = C_raw; regs[0]=c.u>>16; regs[1]=c.u&0xFFFF;
            c.f = C_filt; regs[2]=c.u>>16; regs[3]=c.u&0xFFFF;
            c.f = C_acid; regs[4]=c.u>>16; regs[5]=c.u&0xFFFF;
            c.f = C_alkali; regs[6]=c.u>>16; regs[7]=c.u&0xFFFF;
            c.f = pH; regs[8]=c.u>>16; regs[9]=c.u&0xFFFF;
            modbus_write_registers(ctrl_ctx, CHEM_RESULTS_ADDR, 10, regs);
        }
    }

    /* === 2. REDOX (Channels 2 & 3) === */
    /* Данные берем из ПОСЛЕДНЕЙ фазы (индекс num_phases - 1) */
    if (par->num_phases > 0) {
        int last_ph = par->num_phases - 1;
        float r1_raw = current_cycle_snapshots[last_ph][2]; /* Ch 2 */
        float r2_raw = current_cycle_snapshots[last_ph][3]; /* Ch 3 */

        update_filter(r1_raw, filter_buffer_redox1, &filter_count_redox1, &filter_idx_redox1, (int)par->calc_filter_redox1);
        float r1_avg = get_filter_avg(filter_buffer_redox1, filter_count_redox1);

        update_filter(r2_raw, filter_buffer_redox2, &filter_count_redox2, &filter_idx_redox2, (int)par->calc_filter_redox2);
        float r2_avg = get_filter_avg(filter_buffer_redox2, filter_count_redox2);

        if (ctrl_ctx) {
            uint16_t regs[8]; union { float f; uint32_t u; } c;
            c.f = r1_raw; regs[0]=c.u>>16; regs[1]=c.u&0xFFFF;
            c.f = r1_avg; regs[2]=c.u>>16; regs[3]=c.u&0xFFFF;
            c.f = r2_raw; regs[4]=c.u>>16; regs[5]=c.u&0xFFFF;
            c.f = r2_avg; regs[6]=c.u>>16; regs[7]=c.u&0xFFFF;
            /* Пишем по адресу 4010 (CHEM_RESULTS_ADDR + 10) */
            modbus_write_registers(ctrl_ctx, CHEM_RESULTS_ADDR + 10, 8, regs);
        }
        printf("Redox1: %.3f/%.3f  Redox2: %.3f/%.3f\n", r1_raw, r1_avg, r2_raw, r2_avg);
    }
}

int main(void) {
    signal(SIGINT, handle_sigint);
    IterParams par; if (load_iter_params(ITER_PARAMS_FILE, &par) != 0) return -1;

    FILE *f = NULL;

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

    modbus_t *ctx = modbus_new_tcp(ADAM6224_IP, ADAM6224_PORT);
    if (ctx) { modbus_set_slave(ctx, ADAM6224_SLAVE); modbus_set_response_timeout(ctx, 2, 0); }
    int retry=0;
    while(!g_stop) {
        if(modbus_connect(ctx)==0) { printf("AO Connected.\n"); break; }
        if(retry++ % 5 == 0) fprintf(stderr, "AO Wait...\n");
        sleep(1);
    }

    modbus_t *ctrl_ctx = modbus_new_tcp(MODBUS_CTRL_IP, MODBUS_CTRL_PORT);
    if (ctrl_ctx) { modbus_set_slave(ctrl_ctx, MODBUS_CTRL_SLAVE); modbus_connect(ctrl_ctx); modbus_set_response_timeout(ctrl_ctx, 1, 0); }

    ControlState ctrl_state = CONTROL_RUNNING;
    int restart_requested = 0;

    while (!g_stop) {
        if (restart_requested) { load_iter_params(ITER_PARAMS_FILE, &par); printf("Restarted.\n"); restart_requested = 0; }
        
        struct timespec t0; clock_gettime(CLOCK_MONOTONIC, &t0); struct timespec t_set = t0;
        float prev_ai[8] = {0};
        long cycle = 0; int abort_loops = 0;

        printf("Starting loop...\n");

        for (; (par.repeats == 0 || cycle < par.repeats) && !g_stop && !abort_loops; ++cycle) {
            poll_control_commands(ctrl_ctx, &ctrl_state, &restart_requested);
            if(restart_requested || ctrl_state==CONTROL_STOPPED) break;

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
                    }
                    if(ctrl_ctx) modbus_write_registers(ctrl_ctx, RESULTS_ADDR, 16, res_regs);

                    struct timespec t_now; clock_gettime(CLOCK_MONOTONIC, &t_now);
                    double t_ms = timespec_to_ms(&t_now) - timespec_to_ms(&t0);
                    
                    if (f) {
                        fprintf(f, "%ld;%d;%d;%.3f;%d;%.3f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f\n",
                            cycle+1, ph+1, idx, t_ms, iter_mV, code_to_voltage(code),
                            ai[0], ai[1], ai[2], ai[3], ai[4], ai[5], ai[6], ai[7],
                            calc[0], calc[1], calc[2], calc[3], calc[4], calc[5], calc[6], calc[7]);
                        fflush(f);
                    }

                    printf("c=%ld p=%d i=%d mV=%d AI0=%.4f\n", cycle+1, ph+1, idx, iter_mV, ai[0]);
                    fflush(stdout);

                    last_step_valid = 1;
                    idx++; iter_mV += phase->step_mV;
                }
                if(restart_requested || ctrl_state==CONTROL_STOPPED || abort_loops) { cycle_success=0; break; }

                if (phase->pause_ms > 0 && last_step_valid) {
                    struct timespec t_mid = t_set; timespec_add_ms(&t_mid, phase->pause_ms / 2);
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
                    wait_with_pause(&t_set, phase->pause_ms);
                }
            }
            
            if (cycle_success && !g_stop && !abort_loops) {
                if (f) { fclose(f); f = NULL; }
                rename(FILE_CURRENT, FILE_PREVIOUS);
                PerformChemistryCalculation(&par, ctrl_ctx);
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
    modbus_close(ctx); modbus_free(ctx);
    if (ctrl_ctx) { modbus_close(ctrl_ctx); modbus_free(ctrl_ctx); }
    AdamIO_Close(fd_io);
    return 0;
}