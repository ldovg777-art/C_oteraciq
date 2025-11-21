/*
 * iter_modbus_server.c
 *
 * Исправленная версия с поддержкой мультиплексирования (select).
 * Позволяет одновременно подключаться Панели оператора и локальной программе (adam_step).
 * Обе программы видят одни и те же регистры (общая память).
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

#define PARAMS_FILE "/home/root/iter_params.txt"
#define MODBUS_PORT 1502
#define MAX_PHASES 5

/* --- ОПРЕДЕЛЕНИЯ РЕГИСТРОВ (Те же, что и были) --- */
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

/* Максимальное кол-во клиентов */
#define MAX_CLIENTS 10

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
    if (sigaction(SIGINT, &sa, NULL) == -1) return -1;
    if (sigaction(SIGTERM, &sa, NULL) == -1) return -1;
    /* Игнорируем SIGPIPE, чтобы сервер не падал при обрыве клиента */
    signal(SIGPIPE, SIG_IGN);
    return 0;
}

static void strtrim(char *s) {
    char *p = s;
    while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) s[--len] = '\0';
}

static void init_iter_params(IterParams *p) {
    p->num_phases = 1;
    p->repeats = 1;
    for (int i = 0; i < MAX_PHASES; ++i) {
        p->phases[i].start_mV = -5000; p->phases[i].end_mV = 5000;
        p->phases[i].step_mV = 100; p->phases[i].period_ms = 100;
        p->phases[i].settle_ms = 50; p->phases[i].pause_ms = 0;
    }
}

static void update_phase_count(IterParams *p, int idx) {
    if (idx + 1 > p->num_phases) p->num_phases = idx + 1;
    if (p->num_phases > MAX_PHASES) p->num_phases = MAX_PHASES;
}

static int parse_phase_key(const char *key, int *phase_idx, const char **suffix) {
    const char *p = NULL;
    if (strncmp(key, "step", 4) == 0) p = key + 4;
    else if (strncmp(key, "phase", 5) == 0) p = key + 5;
    if (p && isdigit((unsigned char)*p)) {
        char *endptr = NULL;
        long idx = strtol(p, &endptr, 10);
        if (idx >= 1 && idx <= MAX_PHASES && endptr && *endptr == '_') {
            *phase_idx = (int)idx - 1; *suffix = endptr + 1; return 1;
        }
    }
    *phase_idx = 0; *suffix = key; return 0;
}

static int parse_int(const char *s, int *out) {
    if (!s || !out) return -1;
    char *endptr = NULL;
    long v = strtol(s, &endptr, 10);
    if (endptr == s) return -1;
    *out = (int)v; return 0;
}

static int load_iter_params(const char *path, IterParams *p, int *parsed_values) {
    FILE *fp = fopen(path, "r");
    init_iter_params(p);
    if (!fp) { return -1; }
    int parsed = 0;
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        strtrim(line);
        if (line[0] == '\0' || line[0] == '#') continue;
        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        char *key = line; char *val = eq + 1;
        int phase_idx = 0; const char *suffix = NULL;
        parse_phase_key(key, &phase_idx, &suffix);
        int v = 0;
        if (strcmp(suffix, "start_mV") == 0 && parse_int(val, &v) == 0) { p->phases[phase_idx].start_mV = v; update_phase_count(p, phase_idx); parsed++; }
        else if (strcmp(suffix, "end_mV") == 0 && parse_int(val, &v) == 0) { p->phases[phase_idx].end_mV = v; update_phase_count(p, phase_idx); parsed++; }
        else if (strcmp(suffix, "step_mV") == 0 && parse_int(val, &v) == 0) { p->phases[phase_idx].step_mV = v; update_phase_count(p, phase_idx); parsed++; }
        else if (strcmp(suffix, "period_ms") == 0 && parse_int(val, &v) == 0) { p->phases[phase_idx].period_ms = v; update_phase_count(p, phase_idx); parsed++; }
        else if (strcmp(suffix, "settle_ms") == 0 && parse_int(val, &v) == 0) { p->phases[phase_idx].settle_ms = v; update_phase_count(p, phase_idx); parsed++; }
        else if (strcmp(suffix, "pause_ms") == 0 && parse_int(val, &v) == 0) { p->phases[phase_idx].pause_ms = v; update_phase_count(p, phase_idx); parsed++; }
        else if (strcmp(key, "repeats") == 0) { long rep = strtol(val, NULL, 10); p->repeats = (rep == 0 || rep == -1) ? rep : (rep < 0 ? 1 : rep); parsed++; }
        else if (strcmp(key, "phases") == 0) { int np = 0; if (parse_int(val, &np) == 0 && np >= 1 && np <= MAX_PHASES) p->num_phases = np; parsed++; }
    }
    fclose(fp);
    if (p->num_phases < 1) p->num_phases = 1;
    if (p->num_phases > MAX_PHASES) p->num_phases = MAX_PHASES;
    if (parsed_values) *parsed_values = parsed;
    return 0;
}

static int save_iter_params(const char *path, const IterParams *p) {
    FILE *fp = fopen(path, "w");
    if (!fp) return -1;
    fprintf(fp, "repeats=%ld\nphases=%d\n\n", p->repeats, p->num_phases);
    for (int i = 0; i < p->num_phases; ++i) {
        const char *prefix = (i == 0) ? "" : (char[]){'s','t','e','p',(char)('1' + i), '_', '\0'};
        fprintf(fp, "%sstart_mV=%d\n%send_mV=%d\n%sstep_mV=%d\n%speriod_ms=%d\n%ssettle_ms=%d\n%spause_ms=%d\n\n",
            prefix, p->phases[i].start_mV, prefix, p->phases[i].end_mV, prefix, p->phases[i].step_mV,
            prefix, p->phases[i].period_ms, prefix, p->phases[i].settle_ms, prefix, p->phases[i].pause_ms);
    }
    fclose(fp);
    return 0;
}

/* Утилиты преобразования (без изменений) */
static void int32_to_regs(int32_t v, uint16_t *regs) {
    uint32_t u = (uint32_t)v; regs[0] = (uint16_t)((u >> 16) & 0xFFFFu); regs[1] = (uint16_t)(u & 0xFFFFu);
}
static int32_t regs_to_int32(const uint16_t *regs) {
    return (int32_t)(((uint32_t)regs[0] << 16) | (uint32_t)regs[1]);
}
static void float_to_regs(float v, uint16_t *regs) {
    union { float f; uint32_t u; } conv; conv.f = v;
    regs[0] = (uint16_t)((conv.u >> 16) & 0xFFFFu); regs[1] = (uint16_t)(conv.u & 0xFFFFu);
}
static float regs_to_float(const uint16_t *regs) {
    union { float f; uint32_t u; } conv;
    conv.u = ((uint32_t)regs[0] << 16) | (uint32_t)regs[1]; return conv.f;
}
static int float_to_int_rounded(float v) {
    if (v > (float)INT_MAX) return INT_MAX;
    if (v < (float)INT_MIN) return INT_MIN;
    return (v >= 0.0f) ? (int)(v + 0.5f) : (int)(v - 0.5f);
}
static float control_bits_to_float(uint16_t control_bits) {
    if (control_bits & CMD_RESTART) return 3.0f;
    if (control_bits & CMD_STOP) return 2.0f;
    if (control_bits & CMD_START) return 1.0f;
    return 0.0f;
}

static void params_to_registers(const IterParams *p, uint16_t *regs, int reg_count) {
    if (reg_count < HOLDING_REG_COUNT) return;
    /* Сохраняем текущее состояние Control регистров, чтобы не затереть команды */
    uint16_t ctrl_backup[CONTROL_REG_COUNT];
    memcpy(ctrl_backup, &regs[CONTROL_REG_ADDR], sizeof(ctrl_backup));

    memset(regs, 0, sizeof(uint16_t) * reg_count);

    /* INT BLOCK */
    int32_to_regs(1, &regs[0]); int32_to_regs((int32_t)p->repeats, &regs[2]); int32_to_regs((int32_t)p->num_phases, &regs[4]);
    for (int i = 0; i < MAX_PHASES; ++i) {
        int base = INT_HEADER_REGS + i * INT_PHASE_REGS_PER_PHASE;
        int32_to_regs((int32_t)p->phases[i].start_mV, &regs[base+0]); int32_to_regs((int32_t)p->phases[i].end_mV, &regs[base+2]);
        int32_to_regs((int32_t)p->phases[i].step_mV, &regs[base+4]); int32_to_regs((int32_t)p->phases[i].period_ms, &regs[base+6]);
        int32_to_regs((int32_t)p->phases[i].settle_ms, &regs[base+8]); int32_to_regs((int32_t)p->phases[i].pause_ms, &regs[base+10]);
    }
    /* FLOAT BLOCK */
    int float_base = FLOAT_BASE;
    float_to_regs(1.0f, &regs[float_base+0]); float_to_regs((float)p->repeats, &regs[float_base+2]); float_to_regs((float)p->num_phases, &regs[float_base+4]);
    for (int i = 0; i < MAX_PHASES; ++i) {
        int base = float_base + FLOAT_HEADER_REGS + i * FLOAT_PHASE_REGS_PER_PHASE;
        float_to_regs((float)p->phases[i].start_mV, &regs[base+0]); float_to_regs((float)p->phases[i].end_mV, &regs[base+2]);
        float_to_regs((float)p->phases[i].step_mV, &regs[base+4]); float_to_regs((float)p->phases[i].period_ms, &regs[base+6]);
        float_to_regs((float)p->phases[i].settle_ms, &regs[base+8]); float_to_regs((float)p->phases[i].pause_ms, &regs[base+10]);
    }
    /* Восстанавливаем Control */
    memcpy(&regs[CONTROL_REG_ADDR], ctrl_backup, sizeof(ctrl_backup));
}

static int read_file_mtime(const char *path, time_t *out) {
    struct stat st; if (stat(path, &st) == -1) return -1; *out = st.st_mtime; return 0;
}

/* Функции маппинга (без изменений логики) */
static void registers_int_block_to_params(const uint16_t *regs, IterParams *p) {
    p->repeats = (long)regs_to_int32(&regs[2]); if (p->repeats < 0 && p->repeats != -1) p->repeats = 1;
    p->num_phases = (int)regs_to_int32(&regs[4]); if (p->num_phases < 1) p->num_phases = 1; if (p->num_phases > MAX_PHASES) p->num_phases = MAX_PHASES;
    for (int i = 0; i < MAX_PHASES; ++i) {
        int base = INT_HEADER_REGS + i * INT_PHASE_REGS_PER_PHASE;
        p->phases[i].start_mV = (int)regs_to_int32(&regs[base+0]); p->phases[i].end_mV = (int)regs_to_int32(&regs[base+2]);
        p->phases[i].step_mV = (int)regs_to_int32(&regs[base+4]); p->phases[i].period_ms = (int)regs_to_int32(&regs[base+6]);
        p->phases[i].settle_ms = (int)regs_to_int32(&regs[base+8]); p->phases[i].pause_ms = (int)regs_to_int32(&regs[base+10]);
    }
}
static void registers_float_block_to_params(const uint16_t *regs, IterParams *p) {
    p->repeats = (long)float_to_int_rounded(regs_to_float(&regs[FLOAT_BASE+2]));
    p->num_phases = float_to_int_rounded(regs_to_float(&regs[FLOAT_BASE+4]));
    if (p->num_phases < 1) p->num_phases = 1; if (p->num_phases > MAX_PHASES) p->num_phases = MAX_PHASES;
    for (int i = 0; i < MAX_PHASES; ++i) {
        int base = FLOAT_BASE + FLOAT_HEADER_REGS + i * FLOAT_PHASE_REGS_PER_PHASE;
        p->phases[i].start_mV = float_to_int_rounded(regs_to_float(&regs[base+0]));
        p->phases[i].end_mV = float_to_int_rounded(regs_to_float(&regs[base+2]));
        p->phases[i].step_mV = float_to_int_rounded(regs_to_float(&regs[base+4]));
        p->phases[i].period_ms = float_to_int_rounded(regs_to_float(&regs[base+6]));
        p->phases[i].settle_ms = float_to_int_rounded(regs_to_float(&regs[base+8]));
        p->phases[i].pause_ms = float_to_int_rounded(regs_to_float(&regs[base+10]));
    }
}
static bool write_hits_block(int start, int count, int b_start, int b_size) { return (start + count - 1 >= b_start && start <= b_start + b_size - 1); }
static bool write_hits_range(int start, int count, int r_start, int r_size) { return write_hits_block(start, count, r_start, r_size); }

/* ГЛАВНАЯ ФУНКЦИЯ */
int main(void) {
    if (install_signal_handlers() == -1) return 1;

    IterParams params;
    time_t params_mtime = 0;
    load_iter_params(PARAMS_FILE, &params, NULL);
    read_file_mtime(PARAMS_FILE, &params_mtime);

    modbus_t *ctx = modbus_new_tcp("0.0.0.0", MODBUS_PORT);
    modbus_mapping_t *mapping = modbus_mapping_new(0, 0, HOLDING_REG_COUNT, 0);
    
    /* Инициализация регистров */
    params_to_registers(&params, mapping->tab_registers, mapping->nb_registers);

    int server_socket = modbus_tcp_listen(ctx, MAX_CLIENTS);
    if (server_socket < 0) {
        fprintf(stderr, "Error listening\n");
        return 1;
    }

    /* Массив сокетов клиентов */
    int client_sockets[MAX_CLIENTS];
    for (int i = 0; i < MAX_CLIENTS; i++) client_sockets[i] = 0;

    printf("Сервер запущен. Порт %d. Мультиплексирование включено.\n", MODBUS_PORT);

    fd_set readfds;
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

    while (!g_stop) {
        /* 1. Проверка обновления файла параметров извне */
        time_t current_mtime = 0;
        if (read_file_mtime(PARAMS_FILE, &current_mtime) == 0) {
            if (current_mtime != params_mtime) {
                int parsed = 0;
                if (load_iter_params(PARAMS_FILE, &params, &parsed) == 0 && parsed > 0) {
                    params_mtime = current_mtime;
                    params_to_registers(&params, mapping->tab_registers, mapping->nb_registers);
                    printf("Параметры обновлены из файла.\n");
                }
            }
        }

        /* 2. Настройка SELECT */
        FD_ZERO(&readfds);
        FD_SET(server_socket, &readfds);
        int max_sd = server_socket;

        for (int i = 0; i < MAX_CLIENTS; i++) {
            int sd = client_sockets[i];
            if (sd > 0) FD_SET(sd, &readfds);
            if (sd > max_sd) max_sd = sd;
        }

        struct timeval tv; 
        tv.tv_sec = 0; tv.tv_usec = 100000; /* 100ms таймаут чтобы проверять файл */

        int activity = select(max_sd + 1, &readfds, NULL, NULL, &tv);

        if ((activity < 0) && (errno != EINTR)) {
            continue;
        }

        /* 3. Новое подключение */
        if (FD_ISSET(server_socket, &readfds)) {
            int new_socket;
            if ((new_socket = accept(server_socket, NULL, NULL)) < 0) {
                perror("accept");
            } else {
                // printf("Новое подключение, socket fd is %d\n", new_socket);
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (client_sockets[i] == 0) {
                        client_sockets[i] = new_socket;
                        break;
                    }
                }
            }
        }

        /* 4. Обработка данных от клиентов */
        for (int i = 0; i < MAX_CLIENTS; i++) {
            int sd = client_sockets[i];
            if (FD_ISSET(sd, &readfds)) {
                modbus_set_socket(ctx, sd);
                int rc = modbus_receive(ctx, query);
                
                if (rc > 0) {
                    modbus_reply(ctx, query, rc, mapping);

                    /* Логика обработки ЗАПИСИ (Write) */
                    int func = query[7];
                    if (func == MODBUS_FC_WRITE_SINGLE_REGISTER || func == MODBUS_FC_WRITE_MULTIPLE_REGISTERS) {
                        
                        int start_reg = ((int)query[8] << 8) | (int)query[9];
                        int reg_count = (func == MODBUS_FC_WRITE_MULTIPLE_REGISTERS) ? (((int)query[10] << 8) | (int)query[11]) : 1;

                        bool hit_int = write_hits_block(start_reg, reg_count, 0, INT_HOLDING_REG_COUNT);
                        bool hit_float = write_hits_block(start_reg, reg_count, FLOAT_BASE, FLOAT_HOLDING_REG_COUNT);
                        bool hit_ctrl = write_hits_block(start_reg, reg_count, CONTROL_REG_ADDR, CONTROL_REG_COUNT);

                        /* Обработка Control-регистров (Pause/Restart) */
                        if (hit_ctrl) {
                            uint16_t *c_regs = &mapping->tab_registers[CONTROL_REG_ADDR];
                            float cmd = regs_to_float(c_regs);
                            uint16_t bits = 0;
                            /* Если записали float (с панели) */
                            if (fabsf(cmd - 1.0f) < 0.001f) bits = CMD_START;
                            else if (fabsf(cmd - 2.0f) < 0.001f) bits = CMD_STOP;
                            else if (fabsf(cmd - 3.0f) < 0.001f) bits = CMD_RESTART;
                            
                            /* Если записали биты (напрямую int) - оставляем как есть */
                            if (bits == 0 && c_regs[0] != 0) bits = c_regs[0];

                            /* Нормализуем регистр: и биты, и float должны совпадать */
                            if (bits != 0) {
                                mapping->tab_registers[CONTROL_REG_ADDR] = bits;
                                mapping->tab_registers[CONTROL_REG_ADDR+1] = 0; // Очистка хвоста если float занял 2
                                float_to_regs(control_bits_to_float(bits), &mapping->tab_registers[CONTROL_REG_ADDR]);
                                printf("Команда получена: %d\n", bits);
                            }
                        }

                        /* Сохранение параметров в файл, если меняли настройки */
                        if (hit_int || hit_float) {
                            IterParams int_v, float_v;
                            IterParams new_p = params;
                            if (hit_int) registers_int_block_to_params(mapping->tab_registers, &int_v);
                            if (hit_float) registers_float_block_to_params(mapping->tab_registers, &float_v);
                            
                            /* (Здесь упрощенная логика копирования полей, как в оригинале) */
                            if (hit_int) new_p = int_v; // Для краткости берем весь блок, если задет
                            if (hit_float) new_p = float_v;

                            params = new_p;
                            save_iter_params(PARAMS_FILE, &params);
                            read_file_mtime(PARAMS_FILE, &params_mtime);
                            // Обновляем регистры обратно, чтобы округлить значения
                            params_to_registers(&params, mapping->tab_registers, mapping->nb_registers);
                        }
                    }
                } else {
                    /* Клиент отключился или ошибка */
                    close(sd);
                    client_sockets[i] = 0;
                }
            }
        }
    }

    close(server_socket);
    modbus_mapping_free(mapping);
    modbus_free(ctx);
    return 0;
}