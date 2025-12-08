""" 
MultiVehicle QCar 2 example. 

Make sure you have Quanser Interactive Labs open in Self Driving Car Studio/Cityscape or Cityscape lite
environment before running this example.  

The run.bat file will run all the necessary parts. If you want to run it separate, 
run initCars.py first to spawn the cars in the space. 
Then using two different terminals run vehicle_control.py and vehicle_control2.py to control the vehicles.
Both will ask what type of car you are using, for both, write 2 and click enter. (it is a QCar 2). 
 """
import sys
import time
import numpy as np
from qvl.multi_agent import MultiAgent, readRobots
from qvl.qlabs import QuanserInteractiveLabs
from qvl.real_time import QLabsRealTime
from qvl.free_camera import QLabsFreeCamera
from qvl.qcar2 import QLabsQCar2


# QLabsRealTime().terminate_all_real_time_models()

qlabs = QuanserInteractiveLabs()
    
print("Connecting to QLabs...")
try:
    qlabs.open("localhost")
    # qlabs.destroy_all_spawned_actors()
    # QLabsRealTime().terminate_all_real_time_models()
    print("Connected to QLabs")
except:
    print("Unable to connect to QLabs")
    quit() 
# QCars.append({
#     "RobotType": "QCar2", 
#     "Location": [-11.324, -7.393, 0.005], 
#     "Rotation": [0, 0, -0.7330382858376184], 
#     'Radians': True,
#     "Scale": 1,
# })

# QCars.append({
#     "RobotType": "QC2", 
#     "Location": [1.52, 8.904, 0.005], 
#     "Rotation": [0, 0, -0.12], 
#     'Radians': True,
#     "Scale": 1
# })
QCar0 = QLabsQCar2(qlabs)
QCar1 = QLabsQCar2(qlabs)

QCar0.spawn_id(actorNumber=0, location=[-11.324, -7.393, 0.005], rotation=[0, 0, -0.7330382858376184], waitForConfirmation=True)

QCar1.spawn_id(actorNumber=1, location=[1.52, 8.904, 0.005], rotation=[0, 0, -0.12], waitForConfirmation=True)

QCar0.set_transform_and_request_state(location=[1.8, 8.904, 0.005], rotation=[0, 0, -0.12], enableDynamics=True, headlights=False, leftTurnSignal=False, rightTurnSignal=False, brakeSignal=False, reverseSignal=False)
QCar1.set_transform_and_request_state(location=[-12, -8, 0.005], rotation=[0, 0, 0], enableDynamics=True, headlights=False, leftTurnSignal=False, rightTurnSignal=False, brakeSignal=False, reverseSignal=False)

# print("Connected")  

# QLabsRealTime().terminate_all_real_time_models()
# time.sleep(1)
# qlabs.destroy_all_spawned_actors()

# # create a camera in this qlabs instance
# camera = QLabsFreeCamera(qlabs)
# camera.spawn_degrees(location=[30, -16, 35], rotation=[0, 44, 141.502])
# # to switch our view from our current camera to the new camera we just initialized
# camera.possess()

# qlabs.close()
