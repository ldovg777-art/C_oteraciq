/*
 * adam6224_iter_step.c
 *
 * Итерационный шаг с установкой AO0 на ADAM-6224
 * и измерением всех 8 AI-каналов ADAM-6717.
 *
 * Особенности:
 * - все 8 каналов измеряются последовательно после settle-задержки;
 * - при ошибке чтения берётся предыдущее успешное значение;
 * - code_read удалён, AO считывается только по рассчитанному code_set;
 * - ao_V сохраняется в CSV;
 * - stdout оставлен для отладки (8 каналов одной строкой);
 * - период шага выдерживается строго через CLOCK_MONOTONIC + ABSOLUTE sleep;
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <modbus/modbus.h>

#include "adamapi.h"

#define ITER_PARAMS_FILE   "/home/root/iter_params.txt"

#define ADAM6224_IP      "192.168.2.2"
#define ADAM6224_PORT    502
#define ADAM6224_SLAVE   1

#define AO0_REG_ADDR     0

#define AO_MIN_V   (-5.0)
#define AO_MAX_V   ( 5.0)


#define MAX_PHASES 5

typedef struct {
    int start_mV;
    int end_mV;
    int step_mV;
    int period_ms;
    int settle_ms;
    int pause_ms;
} IterPhase;

typedef struct {
    IterPhase phases[MAX_PHASES];
    int num_phases;
    long repeats;
} IterParams;


static uint16_t voltage_to_code(double v)
{
    const double v_min = AO_MIN_V;
    const double v_max = AO_MAX_V;
    const int code_min = 0;
    const int code_max = 4095;

    if (v < v_min) v = v_min;
    if (v > v_max) v = v_max;

    double k = (v - v_min) / (v_max - v_min);
    double code_f = k * (code_max - code_min);

    int code_i = (int)(code_f + 0.5);
    if (code_i < code_min) code_i = code_min;
    if (code_i > code_max) code_i = code_max;

    return (uint16_t)code_i;
}

static double code_to_voltage(uint16_t code)
{
    const double v_min = AO_MIN_V;
    const double v_max = AO_MAX_V;
    const int code_min = 0;
    const int code_max = 4095;

    if (code > code_max) code = code_max;

    double k = ((double)code - code_min) / (double)(code_max - code_min);
    return v_min + k * (v_max - v_min);
}

static double timespec_to_ms(const struct timespec *ts)
{
    return (double)ts->tv_sec * 1000.0 + (double)ts->tv_nsec / 1.0e6;
}

static void strtrim(char *s)
{
    char *p = s;
    while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);

    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) {
        s[--len] = '\0';
    }
}


static void timespec_add_ms(struct timespec *ts, int ms)
{
    if (ms <= 0)
        return;

    ts->tv_sec += ms / 1000;
    long rem_ms = ms % 1000;
    ts->tv_nsec += rem_ms * 1000000L;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_nsec -= 1000000000L;
        ts->tv_sec  += 1;
    }
}

static void wait_with_pause(struct timespec *t_set, int pause_ms)
{
    if (pause_ms <= 0)
        return;

    timespec_add_ms(t_set, pause_ms);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, t_set, NULL);
}

static void init_iter_params(IterParams *p)
{
    p->num_phases = 1;
    p->repeats = 1;
    for (int i = 0; i < MAX_PHASES; ++i) {
        p->phases[i].start_mV  = -5000;
        p->phases[i].end_mV    =  5000;
        p->phases[i].step_mV   =   100;
        p->phases[i].period_ms =   100;
        p->phases[i].settle_ms =    50;
        p->phases[i].pause_ms  =     0;
    }
}

static void update_phase_count(IterParams *p, int idx)
{
    if (idx + 1 > p->num_phases)
        p->num_phases = idx + 1;
    if (p->num_phases > MAX_PHASES)
        p->num_phases = MAX_PHASES;
}

static int parse_phase_key(const char *key, int *phase_idx, const char **suffix)
{
    const char *p = NULL;
    if (strncmp(key, "step", 4) == 0) {
        p = key + 4;
    } else if (strncmp(key, "phase", 5) == 0) {
        p = key + 5;
    }

    if (p && isdigit((unsigned char)*p)) {
        char *endptr = NULL;
        long idx = strtol(p, &endptr, 10);
        if (idx >= 1 && idx <= MAX_PHASES && endptr && *endptr == '_') {
            *phase_idx = (int)idx - 1;
            *suffix = endptr + 1;
            return 1;
        }
    }

    *phase_idx = 0;
    *suffix = key;
    return 0;
}

static int load_iter_params(const char *path, IterParams *p)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        perror("Не удалось открыть файл параметров");
        return -1;
    }

    init_iter_params(p);

    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        strtrim(line);
        if (line[0] == '\0' || line[0] == '#')
            continue;

        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';

        char *key = line;
        char *val = eq + 1;
        strtrim(key); strtrim(val);

        if (strcmp(key, "repeats") == 0) {
            char *endptr = NULL;
            long r = strtol(val, &endptr, 10);
            if (endptr == val) {
                continue;
            }
            if (r == 0) {
                p->repeats = 0;
            } else if (r < 0) {
                p->repeats = 1;
            } else {
                p->repeats = r;
            }
            continue;
        }

        int v = atoi(val);

        int phase_idx = 0;
        const char *suffix = key;
        parse_phase_key(key, &phase_idx, &suffix);
        if (phase_idx >= MAX_PHASES)
            phase_idx = MAX_PHASES - 1;

        IterPhase *phase = &p->phases[phase_idx];
        update_phase_count(p, phase_idx);

        if (strcmp(suffix, "start_mV") == 0)            phase->start_mV = v;
        else if (strcmp(suffix, "end_mV") == 0)         phase->end_mV = v;
        else if (strcmp(suffix, "step_mV") == 0)        phase->step_mV = v;
        else if (strcmp(suffix, "period_ms") == 0)      phase->period_ms = v;
        else if (strcmp(suffix, "settle_ms") == 0)      phase->settle_ms = v;
        else if (strcmp(suffix, "pause_ms") == 0)       phase->pause_ms = v;
        else if (strcmp(key, "phases") == 0) {
            if (v >= 1 && v <= MAX_PHASES)
                p->num_phases = v;
        }
    }

    fclose(fp);
    if (p->num_phases < 1)
        p->num_phases = 1;
    return 0;
}

static int validate_iter_params(IterParams *p)
{
    if (p->repeats < 0)
        p->repeats = 1;

    for (int i = 0; i < p->num_phases; ++i) {
        IterPhase *phase = &p->phases[i];
        if (phase->step_mV == 0) {
            fprintf(stderr, "Ошибка (фаза %d): step_mV=0\n", i + 1);
            return -1;
        }

        int span = phase->end_mV - phase->start_mV;
        if (span == 0) {
            fprintf(stderr,
                    "Внимание (фаза %d): start=end, будет один шаг\n",
                    i + 1);
        } else {
            if ((span > 0 && phase->step_mV < 0) ||
                (span < 0 && phase->step_mV > 0)) {
                fprintf(stderr,
                        "Ошибка (фаза %d): знак step_mV не согласован с направлением\n",
                        i + 1);
                return -1;
            }
        }

        if (phase->period_ms < 1)
            phase->period_ms = 1;
        if (phase->settle_ms < 1)
            phase->settle_ms = phase->period_ms / 2;
        if (phase->settle_ms >= phase->period_ms)
            phase->settle_ms = phase->period_ms - 1;
        if (phase->settle_ms < 0)
            phase->settle_ms = 0;
        if (phase->pause_ms < 0)
            phase->pause_ms = 0;
    }

    return 0;
}


static volatile int g_stop = 0;

static void handle_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}



int main(void)
{
    IterParams par;
    if (load_iter_params(ITER_PARAMS_FILE, &par) != 0) {
        return -1;
    }

    if (validate_iter_params(&par) != 0) {
        return -1;
    }

    printf("Параметры (фаз: %d):\n", par.num_phases);
    for (int i = 0; i < par.num_phases; ++i) {
        IterPhase *phase = &par.phases[i];
        printf("  Фаза %d:\n", i + 1);
        printf("    start_mV  = %d\n", phase->start_mV);
        printf("    end_mV    = %d\n", phase->end_mV);
        printf("    step_mV   = %d\n", phase->step_mV);
        printf("    period_ms = %d\n", phase->period_ms);
        printf("    settle_ms = %d\n", phase->settle_ms);
        printf("    pause_ms  = %d\n", phase->pause_ms);
    }
    printf("  repeats = %ld (0 = бесконечный цикл)\n", par.repeats);
    printf("\n");

    /* Заготовка лога CSV */
    char fname[128];
    {
        time_t now = time(NULL);
        struct tm tm_now;
        localtime_r(&now, &tm_now);

        snprintf(fname, sizeof(fname),
                 "iter_8ch_%04d%02d%02d_%02d%02d%02d.csv",
                 tm_now.tm_year + 1900,
                 tm_now.tm_mon + 1,
                 tm_now.tm_mday,
                 tm_now.tm_hour,
                 tm_now.tm_min,
                 tm_now.tm_sec);
    }

    FILE *f = fopen(fname, "w");
    if (!f) {
        perror("Ошибка открытия CSV");
        return -1;
    }

    fprintf(f,
        "cycle;phase;idx;time_ms;iter_mV;iter_V;code_set;ao_V;"
        "AI0;AI1;AI2;AI3;AI4;AI5;AI6;AI7\n");

    /* ADAM-6717 */
    int fd_io = -1;
    int ret = AdamIO_Open(&fd_io);
    if (ret < 0) {
        fprintf(stderr, "Ошибка AdamIO_Open\n");
        fclose(f);
        return -1;
    }
    printf("ADAM-6717 открыт, fd=%d\n", fd_io);

    AI_SetAutoFilterEnabled(fd_io, 0x00, 0);
    AI_SetIntegrationMode(fd_io, 0xA0); // high speed

    /* ADAM-6224 */
    modbus_t *ctx = modbus_new_tcp(ADAM6224_IP, ADAM6224_PORT);
    if (!ctx) {
        fprintf(stderr, "Ошибка modbus_new_tcp\n");
        AdamIO_Close(fd_io);
        fclose(f);
        return -1;
    }

    if (modbus_set_slave(ctx, ADAM6224_SLAVE) == -1) {
        fprintf(stderr, "Ошибка modbus_set_slave\n");
        modbus_free(ctx);
        AdamIO_Close(fd_io);
        fclose(f);
        return -1;
    }

    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Ошибка modbus_connect: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        AdamIO_Close(fd_io);
        fclose(f);
        return -1;
    }

    signal(SIGINT, handle_sigint);

    struct timespec t0, t_set;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    t_set = t0;

    /* Массив предыдущих значений для 8 каналов */
    float prev_ai[8];
    for (int i = 0; i < 8; i++)
        prev_ai[i] = 0.0f;

    long total_microsteps = 0;
    int first_step = 1;
    int abort_loops = 0;

    printf("Запуск итерации 8-канального измерения...\n\n");

    for (long cycle = 0;
         (par.repeats == 0 || cycle < par.repeats) && !g_stop && !abort_loops;
         ++cycle)
    {
        long cycle_num = cycle + 1;

        for (int phase_idx = 0;
             phase_idx < par.num_phases && !g_stop && !abort_loops;
             ++phase_idx)
        {
            IterPhase *phase = &par.phases[phase_idx];
            int dir = (phase->step_mV > 0) ? 1 : -1;
            int idx = 0;
            int iter_mV = phase->start_mV;

            while (!g_stop &&
                   ((dir > 0 && iter_mV <= phase->end_mV) ||
                    (dir < 0 && iter_mV >= phase->end_mV)))
            {
                /* ABSOLUTE ожидание начала шага */
                if (!first_step) {
                    timespec_add_ms(&t_set, phase->period_ms);
                } else {
                    first_step = 0;
                }

                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t_set, NULL);

                /* Установка AO0 */
                double iter_V = (double)iter_mV / 1000.0;
                if (iter_V < AO_MIN_V) iter_V = AO_MIN_V;
                if (iter_V > AO_MAX_V) iter_V = AO_MAX_V;

                uint16_t code_set = voltage_to_code(iter_V);
                ret = modbus_write_register(ctx, AO0_REG_ADDR, code_set);
                if (ret == -1) {
                    fprintf(stderr, "Ошибка modbus_write_register: %s\n",
                            modbus_strerror(errno));
                    abort_loops = 1;
                    break;
                }

                /* Ожидание settle */
                struct timespec t_meas = t_set;
                timespec_add_ms(&t_meas, phase->settle_ms);

                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t_meas, NULL);

                /* Время шага */
                struct timespec t_now;
                clock_gettime(CLOCK_MONOTONIC, &t_now);
                double t_ms = timespec_to_ms(&t_now) - timespec_to_ms(&t0);

                /* Измерение 8 каналов */
                float ai[8];
                for (int ch = 0; ch < 8; ch++) {
                    unsigned char st = 0;
                    int ai_ret = AI_GetFloatValue(fd_io, ch, &ai[ch], &st);
                    if (ai_ret != 0) {
                        ai[ch] = prev_ai[ch]; // использовать предыдущее
                    } else {
                        prev_ai[ch] = ai[ch];
                    }
                }

                /* AO: расчётное значение */
                double ao_V = code_to_voltage(code_set);

                /* Запись CSV */
                fprintf(f,
                    "%ld;%d;%d;%.3f;%d;%.6f;%u;%.6f;"
                    "%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f\n",
                    cycle_num,
                    phase_idx + 1, idx, t_ms,
                    iter_mV, iter_V,
                    (unsigned int)code_set,
                    ao_V,
                    (double)ai[0], (double)ai[1], (double)ai[2], (double)ai[3],
                    (double)ai[4], (double)ai[5], (double)ai[6], (double)ai[7]
                );

                /* stdout — отладочный вывод */
                printf(
                    "cycle=%ld phase=%d idx=%d t=%.3f ms iter=%d mV (%.3f В) AO_code=%u AO_V=%.3f "
                    "AI=[%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f]\n",
                    cycle_num,
                    phase_idx + 1, idx, t_ms,
                    iter_mV, iter_V,
                    (unsigned int)code_set,
                    ao_V,
                    ai[0], ai[1], ai[2], ai[3], ai[4], ai[5], ai[6], ai[7]
                );
                fflush(stdout);

                ++idx;
                ++total_microsteps;
                iter_mV += phase->step_mV;
            }

            if (abort_loops || g_stop)
                break;

            wait_with_pause(&t_set, phase->pause_ms);
        }

        if (abort_loops || g_stop)
            break;

        /* Перечитать файл уставок после завершения цикла */
        IterParams new_par;
        if (load_iter_params(ITER_PARAMS_FILE, &new_par) == 0) {
            if (validate_iter_params(&new_par) == 0) {
                par = new_par;
                printf("Параметры итерации обновлены после цикла %ld\n",
                       cycle_num);
            } else {
                fprintf(stderr,
                        "Новые параметры из файла некорректны, оставляем предыдущие\n");
            }
        } else {
            fprintf(stderr,
                    "Не удалось перечитать файл параметров, оставляем предыдущие\n");
        }
    }

    printf("\nЗавершение. Микрошагов всего: %ld\n", total_microsteps);

    modbus_close(ctx);
    modbus_free(ctx);
    AdamIO_Close(fd_io);
    fclose(f);

    return 0;
}
