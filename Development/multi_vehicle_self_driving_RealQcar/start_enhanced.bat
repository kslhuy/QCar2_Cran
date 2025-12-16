@echo off
REM Enhanced Start Script for Multi-Vehicle Control System
REM Uses YAML configuration with individual vehicle settings

echo ============================================================
echo  Multi-Vehicle Control System - Enhanced Start
echo ============================================================
echo.

cd /d "%~dp0"
python python/start_enhanced.py --config fleet_config.yaml

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Failed to start vehicles
    pause
    exit /b 1
)

exit /b 0
