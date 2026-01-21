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
python .\app_main.py

# Run the real logic of vehicle 0   (another cmd)

cd .\Development\multi_vehicle_self_driving_RealQcar\qcar      
python vehicle_main.py --car-id 0 
python vehicle_main.py --car-id 1 
python vehicle_main.py --car-id 2 
python vehicle_main.py --car-id 3 

# use this to plot the result 
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\Observer\ShengyaObs
python .\plot_distributed_luenberger.py -i

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
python .\app_main.py

# Test with fake vehicles (no hardware , no Qlabs , math equation)
# No need to config here / , all the config is in observer internal 
# Move the extras config file in Development\multi_vehicle_self_driving_RealQcar\qcar\Observer\extra_configs 
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\GUI\    
python fake_vehicle_real_logic.py 0 
python fake_vehicle_real_logic.py 1 
python fake_vehicle_real_logic.py 2 
python fake_vehicle_real_logic.py 3 

# use this to plot the result 
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\Observer\ShengyaObs
python .\plot_distributed_luenberger.py -i

