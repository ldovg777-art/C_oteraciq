#!/bin/sh

# Пути к исполняемым файлам
SERVER_BIN="./iter_modbus_server_arm"
WORKER_BIN="./adam_step"

# Лог файлы
LOG_FILE="/home/root/watchdog.log"
WORKER_ERR="/home/root/worker_err.log"

# Примечание: stdout теперь управляется самой программой adam_step
# Логи записываются в /home/root/worker_out_current.log и worker_out_prev.log

# Функция запуска сервера
start_server() {
    echo "$(date): Starting Server..." >> $LOG_FILE
    $SERVER_BIN > /dev/null 2>&1 &
}

# Функция запуска воркера
start_worker() {
    echo "$(date): Starting Worker..." >> $LOG_FILE
    $WORKER_BIN 2> $WORKER_ERR &
}

echo "--- Watchdog Started at $(date) ---" >> $LOG_FILE

while true; do
    # 1. Проверка Сервера (iter_modbus_server_arm)
    # Считаем количество запущенных процессов
    SRV_COUNT=$(ps | grep "iter_modbus_server_arm" | grep -v "grep" | wc -l)

    if [ "$SRV_COUNT" -eq 0 ]; then
        echo "$(date): Server is DOWN. Starting..." >> $LOG_FILE
        start_server
    elif [ "$SRV_COUNT" -gt 1 ]; then
        echo "$(date): WARNING: Found $SRV_COUNT copies of Server. Killing all and restarting..." >> $LOG_FILE
        killall iter_modbus_server_arm
        sleep 1
        start_server
    fi

    # 2. Проверка Воркера (adam_step)
    WRK_COUNT=$(ps | grep "adam_step" | grep -v "grep" | wc -l)

    if [ "$WRK_COUNT" -eq 0 ]; then
        echo "$(date): Worker is DOWN. Starting..." >> $LOG_FILE
        start_worker
    elif [ "$WRK_COUNT" -gt 1 ]; then
        echo "$(date): WARNING: Found $WRK_COUNT copies of Worker. Killing all and restarting..." >> $LOG_FILE
        killall adam_step
        sleep 1
        start_worker
    fi

    # Пауза перед следующей проверкой (60 секунд)
    sleep 60
done
