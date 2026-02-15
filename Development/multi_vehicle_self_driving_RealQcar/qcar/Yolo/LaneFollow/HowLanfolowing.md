# How to Run Lanfolowing test 

Run normal flow : (Activate The manual mode in GUI to control the cars)


# (In Simulator) # For spawn 2 Qcar
cd .\Development\QCar2_multi-vehicle_control\
python .\initCars.py     

#  Start GUI  (another cmd)
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\GUI\      
python .\app_main.py


# Run the real logic of vehicle 0   (another cmd)

cd .\Development\multi_vehicle_self_driving_RealQcar\qcar      
python .\vehicle_main.py --host 127.0.0.1 --port 5000 --car-id 0


# Run the QCar2_lane_following_new to use the lane following algos   (another cmd)
cd Development\multi_vehicle_self_driving_RealQcar\qcar\Yolo\LaneFollow
python QCar2_lane_following_new