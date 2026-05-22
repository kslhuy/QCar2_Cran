@echo off

:: Terminal 1: Spawn 2 Qcar
start "Spawn QCars" cmd /k "python C:\Users\miku3\Documents\QlabDev\QCar2_Cran\Development\QCar2_multi-vehicle_control\initCars_Studio.py"

:: Terminal 2: Start GUI
start "GUI" cmd /k "python C:\Users\miku3\Documents\QlabDev\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar\qcar\GUI\app_main.py"

:: Terminal 3: Vehicle 0 Logic
start "Vehicle 0" cmd /k "python C:\Users\miku3\Documents\QlabDev\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar\qcar\vehicle_main.py --host 127.0.0.1 --port 5000 --car-id 0"

exit