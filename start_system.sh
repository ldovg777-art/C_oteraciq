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

# 1. Запускаем Watchdog (он сам запустит процессы и будет за ними следить)
./watchdog.sh &
echo "Watchdog process launched with PID $!" >> /home/root/boot_diag.log

# Даем время на старт
sleep 5

# Проверка
if pgrep adam_step > /dev/null; then
    echo "SUCCESS: System is running under watchdog." >> /home/root/boot_diag.log
else
    echo "WARNING: Processes not yet visible. Check watchdog.log" >> /home/root/boot_diag.log
fi