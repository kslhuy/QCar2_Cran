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

#
num_cars = 3
car_type = "QC2"
init_location = [40, -6.2, 1.131]
init_distance = 9
init_rotation = [0, 0, 0]

qlabs = QuanserInteractiveLabs()

print("Connecting to QLabs...")
try:
    qlabs.open("localhost")
    qlabs.destroy_all_spawned_actors()
    QLabsRealTime().terminate_all_real_time_models()
    print("Connected to QLabs")
except:
    print("Unable to connect to QLabs")
    quit()

print("Connected")

QLabsRealTime().terminate_all_real_time_models()
time.sleep(1)
qlabs.destroy_all_spawned_actors()

# create a camera in this qlabs instance
camera = QLabsFreeCamera(qlabs)
camera.spawn_degrees(location=[50, -16, 35], rotation=[0, 44, 141.502])
# to switch our view from our current camera to the new camera we just initialized
camera.possess()

qlabs.close()

print("Disconnected from camera qlabs session")

QCars = []

for i in range(num_cars):
    QCars.append(
        {
            "ActorNumber": i,
            "RobotType": car_type,
            "Location": [
                init_location[0] - i * init_distance,
                init_location[1],
                init_location[2],
            ],
            "Rotation": init_rotation,
            "Radians": True,
            "Scale": 1,
        }
    )


mySpawns = MultiAgent(QCars)

# LED colours per car – extend list if more cars are added
_led_colors = [[40, 0, 0], [0, 40, 0], [0, 0, 40], [40, 40, 0]]
for _i, _actor in enumerate(mySpawns.robotActors):
    _color = _led_colors[_i] if _i < len(_led_colors) else [20, 20, 20]
    _actor.set_led_strip_uniform(color=_color)

# Set default transforms for all vehicles using predefined Location and Rotation
print("\n=== Multi-Vehicle Default Transform Setup ===")
try:
    for car_id in range(num_cars):
        location_i = QCars[car_id]["Location"]
        rotation_i = QCars[car_id]["Rotation"]
        mySpawns.robotActors[car_id].set_transform_and_request_state(
            location=location_i,
            rotation=rotation_i,
            enableDynamics=True,
            headlights=False,
            leftTurnSignal=False,
            rightTurnSignal=False,
            brakeSignal=False,
            reverseSignal=False,
        )
        print(f"✓ Car {car_id} transform set to default!")

    print("All vehicles configured with default transforms.")

except KeyboardInterrupt:
    print("\n\nProgram terminated by user.")
except Exception as e:
    print(f"Error: {e}")


qlabs.close()
