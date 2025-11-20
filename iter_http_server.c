/*
 * iter_http_server.c
 *
 * Минимальный однопоточный HTTP-сервер для ADAM-6717, который отдаёт
 * последнюю строку CSV-логов iter_8ch_* и содержимое iter_params.txt для
 * панели оператора (клиент по Ethernet).
 */

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>

#define LISTEN_PORT 8080
#define BACKLOG 4
#define LOG_DIR "/home/root"
#define PARAMS_FILE "/home/root/iter_params.txt"

static volatile sig_atomic_t g_stop = 0;

static void handle_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

static int str_has_suffix(const char *s, const char *suffix)
{
    size_t ls = strlen(s);
    size_t lsf = strlen(suffix);
    if (ls < lsf)
        return 0;
    return strcmp(s + ls - lsf, suffix) == 0;
}

static int find_latest_csv(char *out_path, size_t out_size)
{
    DIR *dir = opendir(LOG_DIR);
    if (!dir)
        return -1;

    time_t latest_mtime = 0;
    char latest_name[256] = {0};
    struct dirent *de;

    while ((de = readdir(dir)) != NULL) {
        if (strncmp(de->d_name, "iter_8ch_", 10) != 0)
            continue;
        if (!str_has_suffix(de->d_name, ".csv"))
            continue;

        char full[512];
        snprintf(full, sizeof(full), "%s/%s", LOG_DIR, de->d_name);

        struct stat st;
        if (stat(full, &st) != 0)
            continue;
        if (!S_ISREG(st.st_mode))
            continue;

        if (st.st_mtime >= latest_mtime) {
            latest_mtime = st.st_mtime;
            strncpy(latest_name, full, sizeof(latest_name) - 1);
            latest_name[sizeof(latest_name) - 1] = '\0';
        }
    }

    closedir(dir);

    if (latest_name[0] == '\0')
        return -1;

    strncpy(out_path, latest_name, out_size - 1);
    out_path[out_size - 1] = '\0';
    return 0;
}

static int read_last_line(const char *path, char *line, size_t line_sz)
{
    FILE *fp = fopen(path, "r");
    if (!fp)
        return -1;

    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return -1;
    }

    long pos = ftell(fp);
    if (pos < 0) {
        fclose(fp);
        return -1;
    }

    long chunk = (long)line_sz - 1;
    if (chunk < 256)
        chunk = 256;

    long start = pos - chunk;
    if (start < 0)
        start = 0;

    if (fseek(fp, start, SEEK_SET) != 0) {
        fclose(fp);
        return -1;
    }

    size_t n = fread(line, 1, line_sz - 1, fp);
    fclose(fp);
    line[n] = '\0';

    char *last_nl = strrchr(line, '\n');
    if (!last_nl)
        return -1;
    if (last_nl > line && *(last_nl - 1) == '\r')
        last_nl--;
    *last_nl = '\0';

    char *prev_nl = strrchr(line, '\n');
    const char *start_line = prev_nl ? prev_nl + 1 : line;
    memmove(line, start_line, strlen(start_line) + 1);
    return 0;
}

static void send_simple_response(int fd, int status, const char *status_text,
                                 const char *content_type, const char *body)
{
    char header[256];
    int body_len = body ? (int)strlen(body) : 0;
    int len = snprintf(header, sizeof(header),
                       "HTTP/1.1 %d %s\r\n"
                       "Content-Type: %s; charset=utf-8\r\n"
                       "Content-Length: %d\r\n"
                       "Connection: close\r\n"
                       "Cache-Control: no-store\r\n\r\n",
                       status, status_text, content_type, body_len);
    if (len < 0)
        return;
    send(fd, header, (size_t)len, 0);
    if (body_len > 0)
        send(fd, body, (size_t)body_len, 0);
}

static void json_escape(const char *src, char *dst, size_t dst_sz)
{
    size_t j = 0;
    for (size_t i = 0; src[i] && j + 2 < dst_sz; ++i) {
        unsigned char c = (unsigned char)src[i];
        if (c == '"' || c == '\\') {
            if (j + 2 >= dst_sz)
                break;
            dst[j++] = '\\';
            dst[j++] = (char)c;
        } else if (c >= 0x20 && c <= 0x7E) {
            dst[j++] = (char)c;
        }
    }
    dst[j] = '\0';
}

static void handle_status(int client_fd)
{
    char path[512];
    if (find_latest_csv(path, sizeof(path)) != 0) {
        const char *body = "{\"data_status\":\"no_data\"}";
        send_simple_response(client_fd, 200, "OK", "application/json", body);
        return;
    }

    char last_line[4096];
    if (read_last_line(path, last_line, sizeof(last_line)) != 0 ||
        last_line[0] == '\0') {
        const char *body = "{\"data_status\":\"empty\"}";
        send_simple_response(client_fd, 200, "OK", "application/json", body);
        return;
    }

    char *fields[20];
    int field_count = 0;
    char *saveptr = NULL;
    char *tok = strtok_r(last_line, ";", &saveptr);
    while (tok && field_count < 20) {
        fields[field_count++] = tok;
        tok = strtok_r(NULL, ";", &saveptr);
    }

    if (field_count < 16) {
        const char *body = "{\"data_status\":\"invalid\"}";
        send_simple_response(client_fd, 200, "OK", "application/json", body);
        return;
    }

    char safe_path[256];
    json_escape(path, safe_path, sizeof(safe_path));

    char json[2048];
    snprintf(json, sizeof(json),
             "{\"data_status\":\"ok\",\"file\":\"%s\","  \
             "\"cycle\":%ld,\"phase\":%ld,\"idx\":%ld,\"time_ms\":%s," \
             "\"iter_mV\":%s,\"iter_V\":%s,\"code_set\":%s,\"ao_V\":%s," \
             "\"AI\":[%s,%s,%s,%s,%s,%s,%s,%s]}",
             safe_path,
             strtol(fields[0], NULL, 10),
             strtol(fields[1], NULL, 10),
             strtol(fields[2], NULL, 10),
             fields[3], fields[4], fields[5], fields[6], fields[7],
             fields[8], fields[9], fields[10], fields[11], fields[12],
             fields[13], fields[14], fields[15]);

    send_simple_response(client_fd, 200, "OK", "application/json", json);
}

static void handle_params(int client_fd)
{
    FILE *fp = fopen(PARAMS_FILE, "r");
    if (!fp) {
        send_simple_response(client_fd, 404, "Not Found", "text/plain",
                             "iter_params.txt not found\n");
        return;
    }

    char buf[4096];
    size_t n = fread(buf, 1, sizeof(buf) - 1, fp);
    fclose(fp);
    buf[n] = '\0';
    send_simple_response(client_fd, 200, "OK", "text/plain", buf);
}

static void handle_root(int client_fd)
{
    const char *body =
        "{\"status\":\"ok\",\"endpoints\":[\"/status\",\"/params\"]}";
    send_simple_response(client_fd, 200, "OK", "application/json", body);
}

static void process_client(int client_fd)
{
    char req[512];
    int n = recv(client_fd, req, sizeof(req) - 1, 0);
    if (n <= 0)
        return;
    req[n] = '\0';

    char method[8];
    char path[256];
    if (sscanf(req, "%7s %255s", method, path) != 2)
        return;

    if (strcmp(method, "GET") != 0) {
        send_simple_response(client_fd, 405, "Method Not Allowed",
                             "text/plain", "Method not allowed\n");
        return;
    }

    if (strcmp(path, "/status") == 0) {
        handle_status(client_fd);
    } else if (strcmp(path, "/params") == 0) {
        handle_params(client_fd);
    } else if (strcmp(path, "/") == 0) {
        handle_root(client_fd);
    } else {
        send_simple_response(client_fd, 404, "Not Found", "text/plain",
                             "Not found\n");
    }
}

int main(void)
{
    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    int srv_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (srv_fd < 0) {
        perror("socket");
        return 1;
    }

    int opt = 1;
    setsockopt(srv_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(LISTEN_PORT);

    if (bind(srv_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(srv_fd);
        return 1;
    }

    if (listen(srv_fd, BACKLOG) < 0) {
        perror("listen");
        close(srv_fd);
        return 1;
    }

    printf("HTTP-сервер запущен на порту %d (Ctrl+C для остановки)\n", LISTEN_PORT);

    while (!g_stop) {
        struct sockaddr_in cli_addr;
        socklen_t cli_len = sizeof(cli_addr);
        int cli_fd = accept(srv_fd, (struct sockaddr *)&cli_addr, &cli_len);
        if (cli_fd < 0) {
            if (errno == EINTR)
                continue;
            perror("accept");
            break;
        }

        process_client(cli_fd);
        close(cli_fd);
    }

    close(srv_fd);
    return 0;
}
