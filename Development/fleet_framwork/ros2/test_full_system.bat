@echo off
REM Quick test script for full vehicle control system ROS integration

echo ================================================
echo Testing Full Vehicle Control System with ROS 2
echo ================================================
echo.

REM Source ROS 2 setup
call install\setup.bat

REM Run the full system node with parameters
echo Starting vehicle_control_full_system node...
echo.

ros2 run ros2test vehicle_control_full_system ^
    --ros-args ^
    -p car_id:=0 ^
    -p v_ref:=0.75 ^
    -p controller_rate:=100

pause
