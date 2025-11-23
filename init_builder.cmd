@echo off
echo Building Docker image 'adam-builder'...
echo This may take a few minutes (downloading Debian, compiling libmodbus)...
docker build -t adam-builder .
if %ERRORLEVEL% EQU 0 (
    echo.
    echo SUCCESS: Image 'adam-builder' created.
    echo Now you can use build_fast.cmd to compile your code instantly.
) else (
    echo.
    echo FAILURE: Could not build Docker image.
)
pause
