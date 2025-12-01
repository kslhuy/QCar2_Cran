@echo off
echo ========================================
echo Testing Ground Station with REAL VehicleLogic Fake Vehicles
echo ========================================

echo.
echo Starting Enhanced GUI Controller (Ground Station)...
echo   This will start the Ground Station server on ports 5000-5001
echo   The GUI will allow you to control the fake vehicles
echo.
start "Ground Station" /d "." powershell -Command "python enhanced_gui_controller.py"

echo Waiting 5 seconds for Ground Station to start...
timeout /t 5 /nobreak > nul

echo.
echo Starting Real VehicleLogic Fake Vehicle 0...
echo   Using REAL VehicleLogic class from vehicle_logic.py
echo   With mock QCar/GPS/YOLO hardware
echo   Full StateMachine + CommandHandler + Controllers
echo   Connecting to localhost:5000
echo.
start "Real Logic Car 0" /d "." powershell -Command "python fake_vehicle_real_logic.py 0"

echo Waiting 3 seconds before starting second vehicle...
timeout /t 3 /nobreak > nul

echo.
echo Starting Real VehicleLogic Fake Vehicle 1...
echo   Using REAL VehicleLogic class from vehicle_logic.py
echo   With mock QCar/GPS/YOLO hardware  
echo   Full StateMachine + CommandHandler + Controllers
echo   Connecting to localhost:5001
echo.
start "Real Logic Car 1" /d "." powershell -Command "python fake_vehicle_real_logic.py 1"

echo.
echo ========================================
echo Real VehicleLogic Test Setup Complete!
echo ========================================
echo.
echo The following windows should have opened:
echo   1. Ground Station GUI (Enhanced GUI Controller)
echo   2. Real Logic Fake Car 0 (using VehicleLogic class)
echo   3. Real Logic Fake Car 1 (using VehicleLogic class)
echo.
echo Expected behavior:
echo   - Cars connect to Ground Station automatically
echo   - Each car runs the REAL VehicleLogic.run() method
echo   - Full StateMachine initialization and state transitions
echo   - Real CommandHandler processes all commands
echo   - Real controllers (speed, steering, state estimator)
echo   - Real safety systems (collision avoidance, watchdog)
echo   - Vehicle states show actual StateMachine states
echo   - Telemetry includes real VehicleLogic data
echo.
echo What this tests:
echo   Complete VehicleLogic class functionality
echo   StateMachine initialization and transitions
echo   CommandHandler command processing
echo   All controller classes
echo   Safety system integration
echo   Ground Station communication
echo   Configuration system
echo   Logging system
echo   Performance monitoring
echo.
echo You can test all commands through the GUI:
echo   - Start/Stop vehicles (tests StateMachine transitions)
echo   - Set velocity reference (tests SpeedController)
echo   - Enable platoon modes (tests PlatoonController)
echo   - Emergency stop (tests safety systems)
echo   - Path commands (tests path following logic)
echo.
echo Monitor the console windows for:
echo   - VehicleLogic initialization messages
echo   - StateMachine state transitions
echo   - CommandHandler processing logs
echo   - Mock hardware sensor readings
echo   - Performance statistics
echo.
echo This verifies that ALL real vehicle classes work correctly
echo without requiring actual QCar hardware!
echo.
echo Close this window to continue...
pause