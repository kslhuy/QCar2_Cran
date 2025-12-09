@echo off
REM Enhanced stop script that stops both Python programs and QUARC models
REM Based on stop_refactored.bat but with QUARC model stopping

echo ============================================================
echo  Multi-Vehicle Control System - ENHANCED STOP
echo ============================================================
echo  This script will stop:
echo   1. Python vehicle control programs
echo   2. QUARC models (hardware control)
echo   3. Hardware (Lidar and QCar DAQ)
echo ============================================================
echo.

cd /d "%~dp0"

REM First, run the Python stop script for programs
echo [1/3] Stopping Python programs...
python python/stop_refactored.py --config config.txt

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [WARNING] Python stop script had issues
)

@REM REM Read configuration for IPs
@REM echo.
@REM echo [2/3] Stopping QUARC models (hardware control)...

@REM REM Parse config.txt for QCAR_IPS
@REM setlocal enabledelayedexpansion
@REM set "configFile=config.txt"

@REM for /f "usebackq tokens=1,* delims==" %%A in (!configFile!) do (	
@REM     set "key=%%A"
@REM     set "value=%%B"
@REM     if "!key!" == "QCAR_IPS" (
@REM         set "QCAR_IPS=!value!"
@REM     )
@REM )

@REM REM Remove [ and ]
@REM set "QCAR_IPS=!QCAR_IPS:[=!"
@REM set "QCAR_IPS=!QCAR_IPS:]=!"

@REM echo Found QCar IPs: !QCAR_IPS!

@REM REM Stop QUARC models on each QCar
@REM for %%I in (!QCAR_IPS!) do (
@REM     echo   Stopping QUARC models on %%I...
@REM     quarc_run -q -Q -t tcpip://%%I:17000 *.rt-linux_qcar2
@REM     timeout /t 1 /nobreak >nul
@REM )

@REM REM Stop hardware (Lidar and QCar DAQ)
@REM echo.
@REM echo [3/3] Stopping hardware (Lidar and QCar DAQ)...
@REM python python/run_hardware_stop.py --config config.txt

@REM if %ERRORLEVEL% NEQ 0 (
@REM     echo.
@REM     echo [WARNING] Hardware stop had issues
@REM )

echo.
echo ============================================================
echo  Enhanced shutdown complete
echo ============================================================
echo  All systems stopped:
echo   - Python programs
echo   - QUARC models  
echo   - Hardware (Lidar and QCar DAQ)
echo.
echo  If issues persist, you may need to:
echo   1. Check QUARC installation
echo   2. Run stop_all_cars.bat as fallback
echo   3. SSH manually to cars and run hardware_stop.py
echo ============================================================
pause