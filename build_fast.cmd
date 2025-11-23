@echo off
echo Building project using 'adam-builder'...
docker run --rm -v "%cd%":/work -w /work adam-builder make
if %ERRORLEVEL% EQU 0 (
    echo.
    echo SUCCESS: Build complete.
) else (
    echo.
    echo FAILURE: Build failed.
)
pause
