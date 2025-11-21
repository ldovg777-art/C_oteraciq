/*
 * iter_modbus_server.c
 *
 * Простой Modbus/TCP-сервер для ADAM-6717, который отражает содержимое
 * /home/root/iter_params.txt в Holding Registers и позволяет панели
 * оператора читать/писать уставки итерации в виде 32-битных целых
 * (исходный формат) и продублированных 32-битных float.
 *
 * Карта регистров (Holding Registers, 0-базовая адресация, порядок слов
 * big-endian везде):
 *   INT-блок (совместимость, int32 BE), 0–65:
 *     0-1   — протокол/сборка, int32 (1)
 *     2-3   — repeats (кол-во циклов, 0 = бесконечно)
 *     4-5   — phases (число фаз 1..5)
 *     Далее по 12 регистров (6 int32) на каждую фазу (MAX_PHASES = 5):
 *         start_mV, end_mV, step_mV, period_ms, settle_ms, pause_ms
 *       Фаза 1: регистры 6-17, фаза 2: 18-29, ... фаза 5: 54-65.
 *   FLOAT-блок (дубль тех же уставок для HMI), 66–131:
 *     Структура идентичная INT-блоку, но все значения закодированы как
 *     IEEE-754 float (32 бита) с тем же порядком слов (big-endian word
 *     order). Поддерживает запись/чтение HMI в формате float без потери
 *     младших единиц. При записи в FLOAT-блок значения округляются до int
 *     и сохраняются в iter_params.txt.
 *   Регистры управления, 132–133 (Holding Registers):
 *     бит 0 (0x0001) — START/RESUME
 *     бит 1 (0x0002) — STOP (переход в остановку без выхода из процесса)
 *     бит 2 (0x0004) — RESTART (перечитать iter_params.txt и начать с цикла 1)
 *     Пара 132–133 также используется как float (порядок слов как в FLOAT-блоке)
 *     для HMI: значения 1.0/2.0/3.0 транслируются в START/STOP/RESTART и
 *     отражаются в обоих регистрах в формате float (1-базовая адресация: 133/134).
 *
 * Для HMI: каждое 32-битное значение занимает ДВА последовательных регистра.
 * Сервер использует порядок big-endian (старшее слово по меньшему адресу),
 * поэтому на панели нужно писать/читать пары регистров с чётного адреса в
 * порядке HI→LO. Если HMI выставлена на little-endian word order, включите
 * swap слов на её стороне, чтобы избежать «сдвинутых» значений.
 *
 * После записи сервер пересчитывает значения (из INT- или FLOAT-блока),
 * сохраняет их в iter_params.txt (перетирая комментарии) и возвращает в
 * регистры нормализованные данные (ограничение фаз 1..MAX_PHASES и
 * округление значений до int).
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

#define PARAMS_FILE "/home/root/iter_params.txt"
#define MODBUS_PORT 1502
#define MAX_PHASES 5

#define INT_HEADER_REGS 6
#define INT_PHASE_REGS_PER_PHASE 12
#define INT_HOLDING_REG_COUNT (INT_HEADER_REGS + MAX_PHASES * INT_PHASE_REGS_PER_PHASE)

#define FLOAT_HEADER_REGS 6
#define FLOAT_PHASE_REGS_PER_PHASE 12
#define FLOAT_BASE INT_HOLDING_REG_COUNT
#define FLOAT_HOLDING_REG_COUNT (FLOAT_HEADER_REGS + MAX_PHASES * FLOAT_PHASE_REGS_PER_PHASE)

#define CONTROL_REG_ADDR (FLOAT_BASE + FLOAT_HOLDING_REG_COUNT)
#define CONTROL_REG_COUNT 2

#define HOLDING_REG_COUNT (INT_HOLDING_REG_COUNT + FLOAT_HOLDING_REG_COUNT + CONTROL_REG_COUNT)

#define CMD_START 0x0001
#define CMD_STOP 0x0002
#define CMD_RESTART 0x0004

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

static int install_signal_handlers(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_sigint;
    sigemptyset(&sa.sa_mask);

    if (sigaction(SIGINT, &sa, NULL) == -1) {
        perror("iter_modbus_server: не удалось установить обработчик SIGINT");
        return -1;
    }

    if (sigaction(SIGTERM, &sa, NULL) == -1) {
        perror("iter_modbus_server: не удалось установить обработчик SIGTERM");
        return -1;
    }

    return 0;
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

static int load_iter_params(const char *path, IterParams *p, int *parsed_values)
{
    FILE *fp = fopen(path, "r");
    init_iter_params(p);
    if (!fp) {
        perror("iter_modbus_server: не удалось открыть iter_params.txt, используем значения по умолчанию");
        return -1;
    }

    int parsed = 0;
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
            parsed++;
        } else if (strcmp(suffix, "end_mV") == 0 && parse_int(val, &v) == 0) {
            p->phases[phase_idx].end_mV = v;
            update_phase_count(p, phase_idx);
            parsed++;
        } else if (strcmp(suffix, "step_mV") == 0 && parse_int(val, &v) == 0) {
            p->phases[phase_idx].step_mV = v;
            update_phase_count(p, phase_idx);
            parsed++;
        } else if (strcmp(suffix, "period_ms") == 0 && parse_int(val, &v) == 0) {
            p->phases[phase_idx].period_ms = v;
            update_phase_count(p, phase_idx);
            parsed++;
        } else if (strcmp(suffix, "settle_ms") == 0 && parse_int(val, &v) == 0) {
            p->phases[phase_idx].settle_ms = v;
            update_phase_count(p, phase_idx);
            parsed++;
        } else if (strcmp(suffix, "pause_ms") == 0 && parse_int(val, &v) == 0) {
            p->phases[phase_idx].pause_ms = v;
            update_phase_count(p, phase_idx);
            parsed++;
        } else if (strcmp(key, "repeats") == 0) {
            long rep = strtol(val, NULL, 10);
            if (rep == 0 || rep == -1) {
                p->repeats = rep;
            } else {
                p->repeats = rep < 0 ? 1 : rep;
            }
            parsed++;
        } else if (strcmp(key, "phases") == 0) {
            int np = 0;
            if (parse_int(val, &np) == 0 && np >= 1 && np <= MAX_PHASES)
                p->num_phases = np;
            parsed++;
        }
    }

    fclose(fp);
    if (p->num_phases < 1)
        p->num_phases = 1;
    if (p->num_phases > MAX_PHASES)
        p->num_phases = MAX_PHASES;

    if (parsed_values)
        *parsed_values = parsed;
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

static void int32_to_regs(int32_t v, uint16_t *regs)
{
    uint32_t u = (uint32_t)v;
    regs[0] = (uint16_t)((u >> 16) & 0xFFFFu);
    regs[1] = (uint16_t)(u & 0xFFFFu);
}

static int32_t regs_to_int32(const uint16_t *regs)
{
    uint32_t u = ((uint32_t)regs[0] << 16) | (uint32_t)regs[1];
    return (int32_t)u;
}

static void float_to_regs(float v, uint16_t *regs)
{
    union {
        float f;
        uint32_t u;
    } conv;

    conv.f = v;
    regs[0] = (uint16_t)((conv.u >> 16) & 0xFFFFu);
    regs[1] = (uint16_t)(conv.u & 0xFFFFu);
}

static float regs_to_float(const uint16_t *regs)
{
    union {
        float f;
        uint32_t u;
    } conv;

    conv.u = ((uint32_t)regs[0] << 16) | (uint32_t)regs[1];
    return conv.f;
}

static int float_to_int_rounded(float v)
{
    if (v > (float)INT_MAX)
        return INT_MAX;
    if (v < (float)INT_MIN)
        return INT_MIN;

    if (v >= 0.0f)
        return (int)(v + 0.5f);
    return (int)(v - 0.5f);
}

static float control_bits_to_float(uint16_t control_bits)
{
    if (control_bits & CMD_RESTART)
        return 3.0f;
    if (control_bits & CMD_STOP)
        return 2.0f;
    if (control_bits & CMD_START)
        return 1.0f;
    return 0.0f;
}

static void params_to_registers(const IterParams *p, uint16_t *regs, int reg_count)
{
    if (reg_count < HOLDING_REG_COUNT)
        return;

    uint16_t control_shadow_bits = 0;
    float control_shadow_float = 0.0f;
    if (reg_count > CONTROL_REG_ADDR) {
        control_shadow_bits = regs[CONTROL_REG_ADDR];
        if (reg_count > CONTROL_REG_ADDR + 1)
            control_shadow_float = regs_to_float(&regs[CONTROL_REG_ADDR]);
    }

    memset(regs, 0, sizeof(uint16_t) * reg_count);

    int32_to_regs(1, &regs[0]);
    int32_to_regs((int32_t)p->repeats, &regs[2]);
    int32_to_regs((int32_t)p->num_phases, &regs[4]);

    for (int i = 0; i < MAX_PHASES; ++i) {
        int base = INT_HEADER_REGS + i * INT_PHASE_REGS_PER_PHASE;
        int32_to_regs((int32_t)p->phases[i].start_mV, &regs[base + 0]);
        int32_to_regs((int32_t)p->phases[i].end_mV, &regs[base + 2]);
        int32_to_regs((int32_t)p->phases[i].step_mV, &regs[base + 4]);
        int32_to_regs((int32_t)p->phases[i].period_ms, &regs[base + 6]);
        int32_to_regs((int32_t)p->phases[i].settle_ms, &regs[base + 8]);
        int32_to_regs((int32_t)p->phases[i].pause_ms, &regs[base + 10]);
    }

    int float_base = FLOAT_BASE;
    float_to_regs(1.0f, &regs[float_base + 0]);
    float_to_regs((float)p->repeats, &regs[float_base + 2]);
    float_to_regs((float)p->num_phases, &regs[float_base + 4]);

    for (int i = 0; i < MAX_PHASES; ++i) {
        int base = float_base + FLOAT_HEADER_REGS + i * FLOAT_PHASE_REGS_PER_PHASE;
        float_to_regs((float)p->phases[i].start_mV, &regs[base + 0]);
        float_to_regs((float)p->phases[i].end_mV, &regs[base + 2]);
        float_to_regs((float)p->phases[i].step_mV, &regs[base + 4]);
        float_to_regs((float)p->phases[i].period_ms, &regs[base + 6]);
        float_to_regs((float)p->phases[i].settle_ms, &regs[base + 8]);
        float_to_regs((float)p->phases[i].pause_ms, &regs[base + 10]);
    }

    float control_value = control_shadow_float;
    if (control_value == 0.0f && control_shadow_bits != 0)
        control_value = control_bits_to_float(control_shadow_bits);

    float_to_regs(control_value, &regs[CONTROL_REG_ADDR]);
}

static int read_file_mtime(const char *path, time_t *out)
{
    struct stat st;
    if (stat(path, &st) == -1)
        return -1;
    *out = st.st_mtime;
    return 0;
}

static void reload_params_if_updated(const char *path, IterParams *params, uint16_t *regs, int reg_count, time_t *cached_mtime)
{
    time_t current_mtime = 0;
    if (read_file_mtime(path, &current_mtime) == -1)
        return;

    if (*cached_mtime != 0 && current_mtime == *cached_mtime)
        return;

    static time_t last_warn_mtime = 0;

    IterParams new_params;
    int parsed = 0;
    if (load_iter_params(path, &new_params, &parsed) == 0) {
        if (parsed > 0) {
            *params = new_params;
            *cached_mtime = current_mtime;
            params_to_registers(params, regs, reg_count);
            printf("iter_params.txt обновлён извне, значения перечитаны в регистры\n");
            last_warn_mtime = 0;
        } else {
            if (last_warn_mtime != current_mtime) {
                fprintf(stderr,
                        "iter_modbus_server: пропустили перезагрузку iter_params.txt — файл пуст или ещё записывается внешним редактором\n");
                last_warn_mtime = current_mtime;
            }
        }
    }
}

static void registers_int_block_to_params(const uint16_t *regs, IterParams *p)
{
    long repeats = (long)regs_to_int32(&regs[2]);
    if (repeats == 0 || repeats == -1) {
        p->repeats = repeats;
    } else {
        p->repeats = repeats < 0 ? 1 : repeats;
    }

    long num_phases = (long)regs_to_int32(&regs[4]);
    if (num_phases < 1)
        num_phases = 1;
    if (num_phases > MAX_PHASES)
        num_phases = MAX_PHASES;
    p->num_phases = (int)num_phases;

    for (int i = 0; i < MAX_PHASES; ++i) {
        int base = INT_HEADER_REGS + i * INT_PHASE_REGS_PER_PHASE;
        p->phases[i].start_mV = (int)regs_to_int32(&regs[base + 0]);
        p->phases[i].end_mV = (int)regs_to_int32(&regs[base + 2]);
        p->phases[i].step_mV = (int)regs_to_int32(&regs[base + 4]);
        p->phases[i].period_ms = (int)regs_to_int32(&regs[base + 6]);
        p->phases[i].settle_ms = (int)regs_to_int32(&regs[base + 8]);
        p->phases[i].pause_ms = (int)regs_to_int32(&regs[base + 10]);
    }
}

static void registers_float_block_to_params(const uint16_t *regs, IterParams *p)
{
    float repeats = regs_to_float(&regs[FLOAT_BASE + 2]);
    int rep_i = float_to_int_rounded(repeats);
    if (rep_i == 0 || rep_i == -1) {
        p->repeats = (long)rep_i;
    } else {
        p->repeats = rep_i < 0 ? 1 : (long)rep_i;
    }

    float num_phases = regs_to_float(&regs[FLOAT_BASE + 4]);
    int np = float_to_int_rounded(num_phases);
    if (np < 1)
        np = 1;
    if (np > MAX_PHASES)
        np = MAX_PHASES;
    p->num_phases = np;

    for (int i = 0; i < MAX_PHASES; ++i) {
        int base = FLOAT_BASE + FLOAT_HEADER_REGS + i * FLOAT_PHASE_REGS_PER_PHASE;
        p->phases[i].start_mV = float_to_int_rounded(regs_to_float(&regs[base + 0]));
        p->phases[i].end_mV = float_to_int_rounded(regs_to_float(&regs[base + 2]));
        p->phases[i].step_mV = float_to_int_rounded(regs_to_float(&regs[base + 4]));
        p->phases[i].period_ms = float_to_int_rounded(regs_to_float(&regs[base + 6]));
        p->phases[i].settle_ms = float_to_int_rounded(regs_to_float(&regs[base + 8]));
        p->phases[i].pause_ms = float_to_int_rounded(regs_to_float(&regs[base + 10]));
    }
}

static void registers_to_params(const uint16_t *regs, IterParams *p, int use_float_block)
{
    if (use_float_block)
        registers_float_block_to_params(regs, p);
    else
        registers_int_block_to_params(regs, p);
}

static int is_write_function(int func)
{
    return func == MODBUS_FC_WRITE_SINGLE_REGISTER ||
           func == MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
}

static bool write_hits_block(int start_reg, int reg_count, int block_start, int block_size)
{
    int end_reg = start_reg + reg_count - 1;
    int block_end = block_start + block_size - 1;
    return end_reg >= block_start && start_reg <= block_end;
}

static bool write_hits_range(int start_reg, int reg_count, int range_start, int range_size)
{
    return write_hits_block(start_reg, reg_count, range_start, range_size);
}

int main(void)
{
    if (install_signal_handlers() == -1)
        return 1;

    IterParams params;
    time_t params_mtime = 0;
    load_iter_params(PARAMS_FILE, &params, NULL);
    read_file_mtime(PARAMS_FILE, &params_mtime);

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
            reload_params_if_updated(PARAMS_FILE, &params, mapping->tab_registers, mapping->nb_registers, &params_mtime);

            rc = modbus_receive(ctx, query);
            if (rc == -1) {
                break; /* клиент закрыл соединение */
            }

            int func = query[7];
            if (modbus_reply(ctx, query, rc, mapping) == -1) {
                break;
            }

            if (is_write_function(func)) {
                int start_reg = ((int)query[8] << 8) | (int)query[9];
                int reg_count = 1;
                if (func == MODBUS_FC_WRITE_MULTIPLE_REGISTERS)
                    reg_count = ((int)query[10] << 8) | (int)query[11];

                bool hit_int = write_hits_block(start_reg, reg_count, 0, INT_HOLDING_REG_COUNT);
                bool hit_float = write_hits_block(start_reg, reg_count, FLOAT_BASE, FLOAT_HOLDING_REG_COUNT);
                bool hit_control_float = write_hits_block(start_reg, reg_count, CONTROL_REG_ADDR, CONTROL_REG_COUNT);

                IterParams new_params = params;
                IterParams int_view;
                IterParams float_view;

                if (hit_int)
                    registers_int_block_to_params(mapping->tab_registers, &int_view);
                if (hit_float)
                    registers_float_block_to_params(mapping->tab_registers, &float_view);
                if (hit_control_float && reg_count >= CONTROL_REG_COUNT) {
                    float cmd = regs_to_float(&mapping->tab_registers[CONTROL_REG_ADDR]);
                    uint16_t control_bits = 0;
                    if (fabsf(cmd - 1.0f) < 0.001f)
                        control_bits = CMD_START;
                    else if (fabsf(cmd - 2.0f) < 0.001f)
                        control_bits = CMD_STOP;
                    else if (fabsf(cmd - 3.0f) < 0.001f)
                        control_bits = CMD_RESTART;

                    if (control_bits != 0) {
                        mapping->tab_registers[CONTROL_REG_ADDR] = control_bits;
                        mapping->tab_registers[CONTROL_REG_ADDR + 1] = 0;
                        float_to_regs(control_bits_to_float(control_bits), &mapping->tab_registers[CONTROL_REG_ADDR]);
                    }
                }

                if (hit_int && write_hits_range(start_reg, reg_count, 2, 2))
                    new_params.repeats = int_view.repeats;
                if (hit_float && write_hits_range(start_reg, reg_count, FLOAT_BASE + 2, 2))
                    new_params.repeats = float_view.repeats;

                if (hit_int && write_hits_range(start_reg, reg_count, 4, 2))
                    new_params.num_phases = int_view.num_phases;
                if (hit_float && write_hits_range(start_reg, reg_count, FLOAT_BASE + 4, 2))
                    new_params.num_phases = float_view.num_phases;

                for (int i = 0; i < MAX_PHASES; ++i) {
                    int base_int = INT_HEADER_REGS + i * INT_PHASE_REGS_PER_PHASE;
                    int base_float = FLOAT_BASE + FLOAT_HEADER_REGS + i * FLOAT_PHASE_REGS_PER_PHASE;

                    if (hit_int && write_hits_range(start_reg, reg_count, base_int + 0, 2))
                        new_params.phases[i].start_mV = int_view.phases[i].start_mV;
                    if (hit_float && write_hits_range(start_reg, reg_count, base_float + 0, 2))
                        new_params.phases[i].start_mV = float_view.phases[i].start_mV;

                    if (hit_int && write_hits_range(start_reg, reg_count, base_int + 2, 2))
                        new_params.phases[i].end_mV = int_view.phases[i].end_mV;
                    if (hit_float && write_hits_range(start_reg, reg_count, base_float + 2, 2))
                        new_params.phases[i].end_mV = float_view.phases[i].end_mV;

                    if (hit_int && write_hits_range(start_reg, reg_count, base_int + 4, 2))
                        new_params.phases[i].step_mV = int_view.phases[i].step_mV;
                    if (hit_float && write_hits_range(start_reg, reg_count, base_float + 4, 2))
                        new_params.phases[i].step_mV = float_view.phases[i].step_mV;

                    if (hit_int && write_hits_range(start_reg, reg_count, base_int + 6, 2))
                        new_params.phases[i].period_ms = int_view.phases[i].period_ms;
                    if (hit_float && write_hits_range(start_reg, reg_count, base_float + 6, 2))
                        new_params.phases[i].period_ms = float_view.phases[i].period_ms;

                    if (hit_int && write_hits_range(start_reg, reg_count, base_int + 8, 2))
                        new_params.phases[i].settle_ms = int_view.phases[i].settle_ms;
                    if (hit_float && write_hits_range(start_reg, reg_count, base_float + 8, 2))
                        new_params.phases[i].settle_ms = float_view.phases[i].settle_ms;

                    if (hit_int && write_hits_range(start_reg, reg_count, base_int + 10, 2))
                        new_params.phases[i].pause_ms = int_view.phases[i].pause_ms;
                    if (hit_float && write_hits_range(start_reg, reg_count, base_float + 10, 2))
                        new_params.phases[i].pause_ms = float_view.phases[i].pause_ms;
                }

                params = new_params;

                save_iter_params(PARAMS_FILE, &params);
                read_file_mtime(PARAMS_FILE, &params_mtime);
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
