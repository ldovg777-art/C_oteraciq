@echo off
setlocal

cd /d C:\ADAM_BUILD

echo === Начало сборки adam6224_iter_step.c ===

docker run --rm -v "%cd%":/work -w /work debian:11 ^
  bash -lc "dpkg --add-architecture armhf && apt-get update && apt-get install -y gcc-arm-linux-gnueabihf libmodbus-dev:armhf && arm-linux-gnueabihf-gcc -O2 adam6224_iter_step.c -o adam6224_iter_step_arm -I./includes -L./libs -ladamapi -L/usr/arm-linux-gnueabihf/lib -lmodbus"

if errorlevel 1 (
    echo.
    echo *** Ошибка сборки adam6224_iter_step_arm ***
) else (
    echo.
    echo *** Сборка завершена успешно. Бинарник: adam6224_iter_step_arm ***
)

endlocal
pause
