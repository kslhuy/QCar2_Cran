@echo off
echo ==========================================
echo Quick Ground Station Connection Test  
echo ==========================================
echo.
echo This will test a single fake vehicle connection.
echo.

set /p car_id="Enter Car ID (0-9, default=0): " 
if "%car_id%"=="" set car_id=0

echo.
echo Starting Fake Vehicle %car_id%...
echo Connecting to Ground Station at localhost:%car_id%000...
echo.
echo Commands you can test from Ground Station:
echo - Start/Stop vehicle
echo - Set velocity (0.0 - 2.0 m/s)  
echo - Emergency stop
echo - Set path nodes
echo - Enable/disable platoon mode
echo.
echo Press Ctrl+C to stop the fake vehicle
echo.

python fake_vehicle.py %car_id%

echo.
echo Vehicle simulation ended.
pause