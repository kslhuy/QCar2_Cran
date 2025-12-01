@echo off
echo ========================================
echo Testing Ground Station with GroundStationClient Fake Vehicles
echo ========================================

echo.
echo Starting Enhanced GUI Controller (Ground Station)...
echo   This will start the Ground Station server on ports 5000-5001
echo.
start "Ground Station" /d "." powershell -Command "python enhanced_gui_controller.py"

echo Waiting 3 seconds for Ground Station to start...
timeout /t 3 /nobreak > nul

echo.
echo Starting GroundStationClient Fake Vehicle 0...
echo   Using real GroundStationClient for network communication
echo   Connecting to localhost:5000
echo.
start "Fake Car 0 (GSC)" /d "." powershell -Command "python fake_vehicle_groundstation.py 0"

echo Waiting 2 seconds before starting second vehicle...
timeout /t 2 /nobreak > nul

echo.
echo Starting GroundStationClient Fake Vehicle 1...
echo   Using real GroundStationClient for network communication  
echo   Connecting to localhost:5001
echo.
start "Fake Car 1 (GSC)" /d "." powershell -Command "python fake_vehicle_groundstation.py 1"

echo.
echo ========================================
echo Test Setup Complete!
echo ========================================
echo.
echo The following windows should have opened:
echo   1. Ground Station GUI (Enhanced GUI Controller)
echo   2. Fake Car 0 with GroundStationClient
echo   3. Fake Car 1 with GroundStationClient
echo.
echo Expected behavior:
echo   - Cars should connect and appear in GUI
echo   - Vehicles will use REAL GroundStationClient architecture
echo   - No port conflicts (proper network handling)
echo   - Commands should work through real CommandHandler
echo   - State transitions through real StateMachine
echo   - Vehicle states should show correctly (not "Unknown")
echo.
echo You can test commands like:
echo   - Start/Stop vehicles
echo   - Set velocity
echo   - Enable platoon modes
echo   - Emergency stop
echo.
echo Close this window to continue...
pause