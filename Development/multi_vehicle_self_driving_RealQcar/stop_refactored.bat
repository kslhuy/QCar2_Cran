@echo off
REM Stop all QCars cleanly - Enhanced version
REM Reads configuration from config.txt
REM Now also stops QUARC models controlling hardware

echo ============================================================
echo  Multi-Vehicle Control System - ENHANCED STOP
echo ============================================================
echo  Stopping both Python programs AND hardware control...
echo ============================================================
echo.

cd /d "%~dp0"
python python/stop_refactored.py --config config.txt

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Failed to stop vehicles properly
    echo [INFO] You may need to run stop_all_cars.bat manually
    pause
    exit /b 1
)

echo.
echo ============================================================
echo  Stop operation completed successfully
echo ============================================================
echo  Both software and hardware should be stopped.
echo ============================================================

exit /b 0
