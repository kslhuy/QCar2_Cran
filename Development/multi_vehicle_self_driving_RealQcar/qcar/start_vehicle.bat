@echo off
REM Quick start script for QCar vehicle control
REM This script automatically uses config_example.yaml

echo ========================================
echo QCar Vehicle Control Startup
echo ========================================
echo.

REM Change to the qcar directory
cd /d "%~dp0"

REM Check if config_example.yaml exists
if not exist "config_example.yaml" (
    echo ERROR: config_example.yaml not found!
    echo Please ensure config_example.yaml is in the same directory as this script.
    pause
    exit /b 1
)

echo Configuration file: config_example.yaml
echo.

REM Parse command line arguments or prompt
if "%1"=="" (
    echo Usage: start_vehicle.bat [car_id] [host_ip]
    echo Example: start_vehicle.bat 0 192.168.2.200
    echo.
    set /p CAR_ID="Enter Car ID (0, 1, 2, ...): "
    set /p HOST_IP="Enter Host PC IP (or press Enter to use config): "
) else (
    set CAR_ID=%1
    set HOST_IP=%2
)

echo.
echo Starting Vehicle Control...
echo   Car ID: %CAR_ID%
if not "%HOST_IP%"=="" (
    echo   Host IP: %HOST_IP%
    python vehicle_control_refactored.py --car-id %CAR_ID% --host %HOST_IP%
) else (
    echo   Host IP: (from config)
    python vehicle_control_refactored.py --car-id %CAR_ID%
)

pause
