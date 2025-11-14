@echo off
echo ==========================================
echo QCar Ground Station Test Setup
echo ==========================================
echo.
echo This script will start:
echo 1. Ground Station GUI (listens for vehicles)  
echo 2. Two StateMachine-based fake vehicles (Car 0 and Car 1)
echo    - Using REAL StateMachine for state management
echo    - Using REAL CommandHandler for command processing
echo    - Behaves exactly like real vehicles
echo.
echo Each component will run in a separate window.
echo Close any window to stop that component.
echo.
pause

echo Starting Ground Station GUI...
start "Ground Station GUI" python enhanced_gui_controller.py

echo Waiting 3 seconds for Ground Station to start...
timeout /t 3 /nobreak > nul

echo Starting StateMachine Fake Vehicle 0...
start "StateMachine Fake Car 0" python fake_vehicle_statemachine.py 0

echo Waiting 2 seconds...
timeout /t 2 /nobreak > nul

echo Starting StateMachine Fake Vehicle 1...
start "StateMachine Fake Car 1" python fake_vehicle_statemachine.py 1

echo.
echo ==========================================
echo All components started!
echo ==========================================
echo.
echo Check the following windows:
echo - "Ground Station GUI" - Main control interface
echo - "StateMachine Fake Car 0" - Real StateMachine vehicle 0
echo - "StateMachine Fake Car 1" - Real StateMachine vehicle 1
echo.
echo The StateMachine fake vehicles use IDENTICAL architecture to real vehicles:
echo - Full StateMachine system for state management
echo - Real CommandHandler for processing commands
echo - Proper state transitions (INITIALIZING -> WAITING_FOR_START -> FOLLOWING_PATH)
echo - Command flags processed exactly like real vehicles
echo.
echo Watch the vehicle states transition properly when you send commands!
echo The vehicles should appear as connected in the GUI.
echo You can now test sending commands from the GUI.
echo.
echo Close this window or press any key to exit...
pause > nul