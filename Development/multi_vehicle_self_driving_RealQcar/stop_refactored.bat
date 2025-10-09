@echo off
REM Stop all QCars cleanly
REM Reads configuration from config.txt

echo ============================================================
echo  Multi-Vehicle Control System - STOP
echo ============================================================
echo.

cd /d "%~dp0"
python python/stop_refactored.py --config ../config.txt

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Failed to stop vehicles
    pause
    exit /b 1
)

exit /b 0
