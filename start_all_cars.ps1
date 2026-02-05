# 激活虚拟环境并启动car 0
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd .\Development\multi_vehicle_self_driving_RealQcar\qcar; & .\env\Scripts\Activate.ps1; python vehicle_main.py --car-id 0"

# 启动car 1
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd .\Development\multi_vehicle_self_driving_RealQcar\qcar; & .\env\Scripts\Activate.ps1; python vehicle_main.py --car-id 1"

# 启动car 2
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd .\Development\multi_vehicle_self_driving_RealQcar\qcar; & .\env\Scripts\Activate.ps1; python vehicle_main.py --car-id 2"

# 启动car 3
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd .\Development\multi_vehicle_self_driving_RealQcar\qcar; & .\env\Scripts\Activate.ps1; python vehicle_main.py --car-id 3"