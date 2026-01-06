# To Quickly Run It

## Config the python virtual enivronment

1. Copy the folder "env" to into the folder "QCAR2_CRAN"
2. Open a python script. After that, the environment will be active, usually.

## Open the QLab, choose downscape lite

## Run the following cmd in the powershell

```powershell
# (In Simulator) # For spawn Qcar
cd .\Development\QCar2_multi-vehicle_control\
python .\initPlatoon.py     

#  Start GUI  (another cmd)
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\GUI\      
python .\enhanced_gui_controller.py

# Run the real logic of vehicle 0   (another cmd)

cd .\Development\multi_vehicle_self_driving_RealQcar\qcar      
python .\vehicle_main.py --host 127.0.0.1 --port 5000 --car-id 0
python vehicle_main.py --car-id 0 --config configs/car0.yaml
python vehicle_main.py --car-id 1 --config configs/car1.yaml
python vehicle_main.py --car-id 2 --config configs/car2.yaml
python vehicle_main.py --car-id 3 --config configs/car3.yaml

# Run the real logic of vehicle 1 (another cmd) 

cd .\Development\multi_vehicle_self_driving_RealQcar\qcar      
python .\vehicle_main.py --host 127.0.0.1 --port 5000 --car-id 1

```
## Quick Test Mode (Fake Vehicle use math equation like We do with Matlab)

```powershell

# (In Simulator) # For spawn Qcar
cd .\Development\QCar2_multi-vehicle_control\
python .\initPlatoon.py
   
# Start GUI 
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\GUI\      
python .\enhanced_gui_controller.py

# Test with fake vehicles (no hardware , no Qlabs , math equation)
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\GUI\    
python fake_vehicle_real_logic.py 0 --config configs/car0.yaml
python fake_vehicle_real_logic.py 1 --config configs/car1.yaml
python fake_vehicle_real_logic.py 2 --config configs/car2.yaml
python fake_vehicle_real_logic.py 3 --config configs/car3.yaml

