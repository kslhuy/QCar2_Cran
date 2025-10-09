@echo off
@REM Test YOLO Camera Script
@REM This script tests if the YOLO camera is working by running it with probing enabled

echo Starting YOLO Camera Test...
echo.
echo 1. Make sure the QCar is powered on and connected
echo 2. Observer window will open automatically
echo 3. You should see the camera feed with YOLO detections
echo.
pause

@REM Start observer locally in a new window
start "Observer" python python\observer.py

@REM Wait 3 seconds for observer to initialize
timeout /t 3

@REM Read config file to get IP
set "configFile=config.txt"
for /f "usebackq tokens=1,* delims==" %%A in (%configFile%) do (	
    set "key=%%A"
    set "value=%%B"
    set "!key!=!value!"
)

@REM SSH to QCar and run YOLO server with probing enabled
echo.
echo Connecting to QCar and starting YOLO camera...
echo IP: %PROBING_IP%
echo Local IP: %LOCAL_IP%
echo.

ssh nvidia@%PROBING_IP% "cd %REMOTE_PATH% && python yolo_server.py -p True -i %LOCAL_IP% -w %WIDTH% -ht %HEIGHT%"
