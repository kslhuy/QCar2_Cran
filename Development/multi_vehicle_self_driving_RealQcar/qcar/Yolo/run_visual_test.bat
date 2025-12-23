@echo off
REM Quick launcher for YOLO visual testing
echo.
echo ========================================
echo    YOLO Visual Testing Launcher
echo ========================================
echo.
echo Choose your test method:
echo.
echo [1] Visual Monitor (with server - shows camera feed)
echo [2] Complete Local Test (no server needed - EASIEST!)
echo [3] Simple Text Monitor (lightweight)
echo [4] Mock Test (no hardware needed)
echo [5] Exit
echo.
set /p choice="Enter your choice (1-5): "

if "%choice%"=="1" goto visual_monitor
if "%choice%"=="2" goto local_test
if "%choice%"=="3" goto text_monitor
if "%choice%"=="4" goto mock_test
if "%choice%"=="5" goto end
echo Invalid choice!
pause
goto end

:visual_monitor
echo.
echo Starting Visual Monitor...
echo.
echo This will start TWO windows:
echo   1. YOLO Server (this window)
echo   2. Visual Monitor (will open automatically in new window)
echo.
pause

REM Start YOLO server in this window
start "YOLO Visual Monitor" cmd /k "python visual_yolo_monitor.py --car-id 0 --show-probe"
echo Starting YOLO server...
python yolo_server_virtual.py --caridx 0 --probing True
goto end

:local_test
echo.
echo Starting Complete Local Test...
echo No server needed - this runs everything locally!
echo.
pause
python test_yolo_with_images.py --car-id 0
goto end

:text_monitor
echo.
echo Starting Text Monitor...
echo.
echo This will start TWO windows:
echo   1. YOLO Server (this window)
echo   2. Text Monitor (will open automatically in new window)
echo.
pause

start "YOLO Text Monitor" cmd /k "python simple_yolo_monitor.py --car-id 0"
echo Starting YOLO server...
python yolo_server_virtual.py --caridx 0
goto end

:mock_test
echo.
echo Starting Mock Test...
echo This tests communication without camera/YOLO hardware
echo.
pause
python test_yolo_communication.py --mock --duration 30
goto end

:end
echo.
echo Test completed!
pause
