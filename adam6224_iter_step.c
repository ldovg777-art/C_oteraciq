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


typedef struct {
    int start_mV;
    int end_mV;
    int step_mV;
    int period_ms;
    int settle_ms;
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


static int load_iter_params(const char *path, IterParams *p)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        perror("Не удалось открыть файл параметров");
        return -1;
    }

    p->start_mV  = -5000;
    p->end_mV    =  5000;
    p->step_mV   =   100;
    p->period_ms =   100;
    p->settle_ms =    50;

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

        int v = atoi(val);

        if (strcmp(key, "start_mV") == 0)       p->start_mV = v;
        else if (strcmp(key, "end_mV") == 0)    p->end_mV = v;
        else if (strcmp(key, "step_mV") == 0)   p->step_mV = v;
        else if (strcmp(key, "period_ms") == 0) p->period_ms = v;
        else if (strcmp(key, "settle_ms") == 0) p->settle_ms = v;
    }

    fclose(fp);
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

    printf("Параметры:\n");
    printf("  start_mV   = %d\n", par.start_mV);
    printf("  end_mV     = %d\n", par.end_mV);
    printf("  step_mV    = %d\n", par.step_mV);
    printf("  period_ms  = %d\n", par.period_ms);
    printf("  settle_ms  = %d\n\n", par.settle_ms);

    if (par.step_mV == 0) {
        fprintf(stderr, "Ошибка: step_mV=0\n");
        return -1;
    }

    int span = par.end_mV - par.start_mV;
    if (span == 0) {
        fprintf(stderr,
                "Внимание: start=end, будет один шаг\n\n");
    } else {
        if ((span > 0 && par.step_mV < 0) ||
            (span < 0 && par.step_mV > 0)) {
            fprintf(stderr,
                    "Ошибка: знак step_mV не согласован с направлением\n");
            return -1;
        }
    }

    if (par.period_ms < 1) par.period_ms = 1;
    if (par.settle_ms < 1) par.settle_ms = par.period_ms / 2;
    if (par.settle_ms >= par.period_ms)
        par.settle_ms = par.period_ms - 1;

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
        "cycle;idx;time_ms;iter_mV;iter_V;code_set;ao_V;"
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

    int dir = (par.step_mV > 0) ? 1 : -1;
    int idx = 0;
    int cycle = 0;

    printf("Запуск итерации 8-канального измерения...\n\n");

    for (int iter_mV = par.start_mV;
         !g_stop &&
         ((dir > 0 && iter_mV <= par.end_mV) ||
          (dir < 0 && iter_mV >= par.end_mV));
         iter_mV += par.step_mV, ++idx)
    {
        /* ABSOLUTE ожидание начала шага */
        if (idx > 0) {
            long add_ns = (long)par.period_ms * 1000000L;
            t_set.tv_nsec += add_ns;
            while (t_set.tv_nsec >= 1000000000L) {
                t_set.tv_nsec -= 1000000000L;
                t_set.tv_sec  += 1;
            }
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
            break;
        }

        /* Ожидание settle */
        struct timespec t_meas = t_set;
        long add_ns_meas = (long)par.settle_ms * 1000000L;
        t_meas.tv_nsec += add_ns_meas;
        while (t_meas.tv_nsec >= 1000000000L) {
            t_meas.tv_nsec -= 1000000000L;
            t_meas.tv_sec  += 1;
        }

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
            "%d;%d;%.3f;%d;%.6f;%u;%.6f;"
            "%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f\n",
            cycle, idx, t_ms,
            iter_mV, iter_V,
            (unsigned int)code_set,
            ao_V,
            (double)ai[0], (double)ai[1], (double)ai[2], (double)ai[3],
            (double)ai[4], (double)ai[5], (double)ai[6], (double)ai[7]
        );

        /* stdout — отладочный вывод */
        printf(
            "cycle=%d idx=%d t=%.3f ms iter=%d mV (%.3f В) AO_code=%u AO_V=%.3f "
            "AI=[%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f]\n",
            cycle, idx, t_ms,
            iter_mV, iter_V,
            (unsigned int)code_set,
            ao_V,
            ai[0], ai[1], ai[2], ai[3], ai[4], ai[5], ai[6], ai[7]
        );
        fflush(stdout);
    }

    printf("\nЗавершение. Микрошагов: %d\n", idx);

    modbus_close(ctx);
    modbus_free(ctx);
    AdamIO_Close(fd_io);
    fclose(f);

    return 0;
}
