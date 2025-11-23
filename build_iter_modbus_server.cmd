@echo off
setlocal

cd /d C:\ADAM_BUILD

echo ===   iter_modbus_server.c ===

docker run --rm -v "%cd%":/work -w /work debian:11 ^
  bash -lc "dpkg --add-architecture armhf && apt-get update && apt-get install -y gcc-arm-linux-gnueabihf libmodbus-dev:armhf && arm-linux-gnueabihf-gcc -O2 -static iter_modbus_server.c -o iter_modbus_server_arm -lmodbus -lm"

if errorlevel 1 (
    echo.
    echo ***   iter_modbus_server_arm ***
) else (
    echo.
    echo ***   . : iter_modbus_server_arm ***
)

endlocal
pause
