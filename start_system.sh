#!/bin/sh

# --- ГЛАВНОЕ: Указываем точные пути к библиотекам ---
# /home/root/lib - для libadamapi.so
# /usr/lib - для libmodbus.so
export LD_LIBRARY_PATH=/home/root/lib:/usr/lib:/usr/local/lib:/lib:$LD_LIBRARY_PATH

# Ждем инициализации системы
sleep 15

# Переходим в рабочую папку
cd /home/root/

# Логгируем старт
echo "--- Boot sequence started at $(date) ---" > /home/root/boot_diag.log

# Убиваем старые копии
killall iter_modbus_server_arm 2>/dev/null
killall adam_step 2>/dev/null
sleep 2

# 1. Запускаем СЕРВЕР
./iter_modbus_server_arm > /dev/null 2>&1 &
echo "Server process launched." >> /home/root/boot_diag.log

# Даем серверу время
sleep 5

# 2. Запускаем ИТЕРАЦИЮ (Воркер)
# Важно: разделяем вывод ошибок (stderr) в отдельный файл для диагностики
./adam_step > /home/root/worker_out.log 2> /home/root/worker_err.log &
echo "Worker process launched with PID $!" >> /home/root/boot_diag.log

# Проверка через 2 секунды: жив ли процесс?
sleep 2
if pgrep adam_step > /dev/null; then
    echo "SUCCESS: adam_step is running." >> /home/root/boot_diag.log
else
    echo "FAILURE: adam_step died immediately. Check worker_err.log" >> /home/root/boot_diag.log
fi