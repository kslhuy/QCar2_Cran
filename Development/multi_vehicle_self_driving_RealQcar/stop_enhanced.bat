@echo off
REM Enhanced stop script that stops both Python programs and QUARC models
REM Based on stop_refactored.bat but with QUARC model stopping

echo ============================================================
echo  Multi-Vehicle Control System - ENHANCED STOP
echo ============================================================
echo  This script will stop:
echo   1. Python vehicle control programs
echo   2. QUARC models (hardware control)
echo ============================================================
echo.

cd /d "%~dp0"

REM First, run the Python stop script for programs
echo [1/2] Stopping Python programs...
python python/stop_refactored.py --config config.txt

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [WARNING] Python stop script had issues
)

REM Read configuration for IPs
echo.
echo [2/2] Stopping QUARC models (hardware control)...

REM Parse config.txt for QCAR_IPS
setlocal enabledelayedexpansion
set "configFile=config.txt"

for /f "usebackq tokens=1,* delims==" %%A in (!configFile!) do (	
    set "key=%%A"
    set "value=%%B"
    if "!key!" == "QCAR_IPS" (
        set "QCAR_IPS=!value!"
    )
)

REM Remove [ and ]
set "QCAR_IPS=!QCAR_IPS:[=!"
set "QCAR_IPS=!QCAR_IPS:]=!"

echo Found QCar IPs: !QCAR_IPS!

REM Stop QUARC models on each QCar
for %%I in (!QCAR_IPS!) do (
    echo   Stopping QUARC models on %%I...
    quarc_run -q -Q -t tcpip://%%I:17000 *.rt-linux_qcar2
    timeout /t 1 /nobreak >nul
)

REM Test if our Python test script works
echo.
echo [TEST] Running QUARC test to verify shutdown...
python test_stop_quarc.py

echo.
echo ============================================================
echo  Enhanced shutdown complete
echo ============================================================
echo  Both software and hardware should be stopped.
echo  If hardware is still running, you may need to:
echo   1. Check QUARC installation
echo   2. Run stop_all_cars.bat as fallback
echo ============================================================
pause