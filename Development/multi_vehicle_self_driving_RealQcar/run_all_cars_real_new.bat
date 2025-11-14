@echo off
REM Start all QCars with refactored vehicle control system
REM Reads configuration from config.txt

echo ============================================================
echo  Multi-Vehicle Refactored Control System - START
echo ============================================================
echo.

cd /d "%~dp0"
python python/start_refactored.py --config ../config.txt

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Failed to start vehicles
    pause
    exit /b 1
)

exit /b 0
