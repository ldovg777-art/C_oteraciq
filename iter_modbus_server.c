/*
 * iter_modbus_server.c
 *
 * Простой Modbus/TCP-сервер для ADAM-6717, который отражает содержимое
 * /home/root/iter_params.txt в Holding Registers и позволяет панели
 * оператора читать/писать уставки итерации в формате float.
 *
 * Карта регистров (Holding Registers, 0-базовая адресация):
 *   0-1   — протокол/сборка, float (1.0f)
 *   2-3   — repeats (кол-во циклов, 0 = бесконечно), float
 *   4-5   — phases (число фаз 1..5), float
 *   Далее по 12 регистров (6 float) на каждую фазу (MAX_PHASES = 5):
 *       start_mV, end_mV, step_mV, period_ms, settle_ms, pause_ms
 *     Фаза 1: регистры 6-17, фаза 2: 18-29, ... фаза 5: 54-65.
 *
 * Формат float: IEEE754, порядок слов big-endian (старшее слово первым),
 * совместим с типичной HMI, читающей/записывающей 32-бит float по Modbus.
 *
 * После записи регистров сервер пересчитывает значения в целые параметры
 * итерации, сохраняет их в iter_params.txt (перетирая комментарии) и
 * возвращает в регистры уже нормализованные данные (ограничение фаз
 * 1..MAX_PHASES и округление значений до int).
 */

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <modbus/modbus.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define PARAMS_FILE "/home/root/iter_params.txt"
#define MODBUS_PORT 1502
#define MAX_PHASES 5

#define HEADER_REGS 6
#define PHASE_REGS_PER_PHASE 12
#define HOLDING_REG_COUNT (HEADER_REGS + MAX_PHASES * PHASE_REGS_PER_PHASE)

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

static volatile sig_atomic_t g_stop = 0;

static void handle_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

static void strtrim(char *s)
{
    char *p = s;
    while (*p && isspace((unsigned char)*p))
        p++;
    if (p != s)
        memmove(s, p, strlen(p) + 1);

    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) {
        s[--len] = '\0';
    }
}

static void init_iter_params(IterParams *p)
{
    p->num_phases = 1;
    p->repeats = 1;
    for (int i = 0; i < MAX_PHASES; ++i) {
        p->phases[i].start_mV = -5000;
        p->phases[i].end_mV = 5000;
        p->phases[i].step_mV = 100;
        p->phases[i].period_ms = 100;
        p->phases[i].settle_ms = 50;
        p->phases[i].pause_ms = 0;
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

static int parse_int(const char *s, int *out)
{
    if (!s || !out)
        return -1;

    errno = 0;
    char *endptr = NULL;
    long v = strtol(s, &endptr, 10);
    if (errno == ERANGE || endptr == s)
        return -1;
    while (endptr && *endptr) {
        if (!isspace((unsigned char)*endptr))
            return -1;
        endptr++;
    }
    *out = (int)v;
    return 0;
}

static int load_iter_params(const char *path, IterParams *p)
{
    FILE *fp = fopen(path, "r");
    init_iter_params(p);
    if (!fp) {
        perror("iter_modbus_server: не удалось открыть iter_params.txt, используем значения по умолчанию");
        return -1;
    }

    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        strtrim(line);
        if (line[0] == '\0' || line[0] == '#')
            continue;

        char *eq = strchr(line, '=');
        if (!eq)
            continue;
        *eq = '\0';

        char *key = line;
        char *val = eq + 1;

        int phase_idx = 0;
        const char *suffix = NULL;
        parse_phase_key(key, &phase_idx, &suffix);

        int v = 0;
        if (strcmp(suffix, "start_mV") == 0 && parse_int(val, &v) == 0) {
            p->phases[phase_idx].start_mV = v;
            update_phase_count(p, phase_idx);
        } else if (strcmp(suffix, "end_mV") == 0 && parse_int(val, &v) == 0) {
            p->phases[phase_idx].end_mV = v;
            update_phase_count(p, phase_idx);
        } else if (strcmp(suffix, "step_mV") == 0 && parse_int(val, &v) == 0) {
            p->phases[phase_idx].step_mV = v;
            update_phase_count(p, phase_idx);
        } else if (strcmp(suffix, "period_ms") == 0 && parse_int(val, &v) == 0) {
            p->phases[phase_idx].period_ms = v;
            update_phase_count(p, phase_idx);
        } else if (strcmp(suffix, "settle_ms") == 0 && parse_int(val, &v) == 0) {
            p->phases[phase_idx].settle_ms = v;
            update_phase_count(p, phase_idx);
        } else if (strcmp(suffix, "pause_ms") == 0 && parse_int(val, &v) == 0) {
            p->phases[phase_idx].pause_ms = v;
            update_phase_count(p, phase_idx);
        } else if (strcmp(key, "repeats") == 0) {
            long rep = strtol(val, NULL, 10);
            if (rep == 0 || rep == -1) {
                p->repeats = rep;
            } else {
                p->repeats = rep < 0 ? 1 : rep;
            }
        } else if (strcmp(key, "phases") == 0) {
            int np = 0;
            if (parse_int(val, &np) == 0 && np >= 1 && np <= MAX_PHASES)
                p->num_phases = np;
        }
    }

    fclose(fp);
    if (p->num_phases < 1)
        p->num_phases = 1;
    if (p->num_phases > MAX_PHASES)
        p->num_phases = MAX_PHASES;
    return 0;
}

static int save_iter_params(const char *path, const IterParams *p)
{
    FILE *fp = fopen(path, "w");
    if (!fp) {
        perror("iter_modbus_server: не удалось сохранить iter_params.txt");
        return -1;
    }

    fprintf(fp, "repeats=%ld\n", p->repeats);
    fprintf(fp, "phases=%d\n\n", p->num_phases);

    for (int i = 0; i < p->num_phases; ++i) {
        const char *prefix = (i == 0) ? "" : (char[]){'s','t','e','p',(char)('1' + i), '_', '\0'};
        fprintf(fp, "%sstart_mV=%d\n", prefix, p->phases[i].start_mV);
        fprintf(fp, "%send_mV=%d\n", prefix, p->phases[i].end_mV);
        fprintf(fp, "%sstep_mV=%d\n", prefix, p->phases[i].step_mV);
        fprintf(fp, "%speriod_ms=%d\n", prefix, p->phases[i].period_ms);
        fprintf(fp, "%ssettle_ms=%d\n", prefix, p->phases[i].settle_ms);
        fprintf(fp, "%spause_ms=%d\n\n", prefix, p->phases[i].pause_ms);
    }

    fclose(fp);
    return 0;
}

static void float_to_regs(float v, uint16_t *regs)
{
    union {
        float f;
        uint32_t u;
    } cvt;
    cvt.f = v;
    regs[0] = (uint16_t)((cvt.u >> 16) & 0xFFFFu);
    regs[1] = (uint16_t)(cvt.u & 0xFFFFu);
}

static float regs_to_float(const uint16_t *regs)
{
    union {
        float f;
        uint32_t u;
    } cvt;
    cvt.u = ((uint32_t)regs[0] << 16) | (uint32_t)regs[1];
    return cvt.f;
}

static void params_to_registers(const IterParams *p, uint16_t *regs, int reg_count)
{
    if (reg_count < HOLDING_REG_COUNT)
        return;

    memset(regs, 0, sizeof(uint16_t) * reg_count);

    float_to_regs(1.0f, &regs[0]);
    float_to_regs((float)p->repeats, &regs[2]);
    float_to_regs((float)p->num_phases, &regs[4]);

    for (int i = 0; i < MAX_PHASES; ++i) {
        int base = HEADER_REGS + i * PHASE_REGS_PER_PHASE;
        float_to_regs((float)p->phases[i].start_mV, &regs[base + 0]);
        float_to_regs((float)p->phases[i].end_mV, &regs[base + 2]);
        float_to_regs((float)p->phases[i].step_mV, &regs[base + 4]);
        float_to_regs((float)p->phases[i].period_ms, &regs[base + 6]);
        float_to_regs((float)p->phases[i].settle_ms, &regs[base + 8]);
        float_to_regs((float)p->phases[i].pause_ms, &regs[base + 10]);
    }
}

static long round_to_long(float v)
{
    return lroundf(v);
}

static void registers_to_params(const uint16_t *regs, IterParams *p)
{
    float repeats_f = regs_to_float(&regs[2]);
    float phases_f = regs_to_float(&regs[4]);

    long repeats = round_to_long(repeats_f);
    if (repeats == 0 || repeats == -1) {
        p->repeats = repeats;
    } else {
        p->repeats = repeats < 0 ? 1 : repeats;
    }

    long num_phases = round_to_long(phases_f);
    if (num_phases < 1)
        num_phases = 1;
    if (num_phases > MAX_PHASES)
        num_phases = MAX_PHASES;
    p->num_phases = (int)num_phases;

    for (int i = 0; i < MAX_PHASES; ++i) {
        int base = HEADER_REGS + i * PHASE_REGS_PER_PHASE;
        p->phases[i].start_mV = (int)round_to_long(regs_to_float(&regs[base + 0]));
        p->phases[i].end_mV = (int)round_to_long(regs_to_float(&regs[base + 2]));
        p->phases[i].step_mV = (int)round_to_long(regs_to_float(&regs[base + 4]));
        p->phases[i].period_ms = (int)round_to_long(regs_to_float(&regs[base + 6]));
        p->phases[i].settle_ms = (int)round_to_long(regs_to_float(&regs[base + 8]));
        p->phases[i].pause_ms = (int)round_to_long(regs_to_float(&regs[base + 10]));
    }
}

static int is_write_function(int func)
{
    return func == MODBUS_FC_WRITE_SINGLE_REGISTER ||
           func == MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
}

int main(void)
{
    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    IterParams params;
    load_iter_params(PARAMS_FILE, &params);

    modbus_t *ctx = modbus_new_tcp("0.0.0.0", MODBUS_PORT);
    if (!ctx) {
        fprintf(stderr, "Не удалось создать контекст Modbus\n");
        return 1;
    }

    modbus_mapping_t *mapping = modbus_mapping_new(0, 0, HOLDING_REG_COUNT, 0);
    if (!mapping) {
        fprintf(stderr, "Не удалось выделить modbus mapping: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return 1;
    }

    params_to_registers(&params, mapping->tab_registers, mapping->nb_registers);

    int listen_fd = modbus_tcp_listen(ctx, 5);
    if (listen_fd < 0) {
        fprintf(stderr, "Ошибка listen на порту %d: %s\n", MODBUS_PORT, modbus_strerror(errno));
        modbus_mapping_free(mapping);
        modbus_free(ctx);
        return 1;
    }

    printf("Modbus/TCP сервер запущен на порту %d, регистров: %d\n", MODBUS_PORT, mapping->nb_registers);
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

    while (!g_stop) {
        int rc = modbus_tcp_accept(ctx, &listen_fd);
        if (rc == -1) {
            if (errno == EINTR && g_stop)
                break;
            continue;
        }

        for (;;) {
            rc = modbus_receive(ctx, query);
            if (rc == -1) {
                break; /* клиент закрыл соединение */
            }

            int func = query[7];
            if (modbus_reply(ctx, query, rc, mapping) == -1) {
                break;
            }

            if (is_write_function(func)) {
                registers_to_params(mapping->tab_registers, &params);
                save_iter_params(PARAMS_FILE, &params);
                params_to_registers(&params, mapping->tab_registers, mapping->nb_registers);
            }
        }

        modbus_close(ctx);
    }

    close(listen_fd);
    modbus_mapping_free(mapping);
    modbus_free(ctx);
    printf("Modbus/TCP сервер остановлен\n");
    return 0;
}
