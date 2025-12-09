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
camera.spawn_degrees(location=[30, -16, 35], rotation=[0, 44, 141.502])
# to switch our view from our current camera to the new camera we just initialized
camera.possess()

qlabs.close()

print("Disconnected from camera qlabs session") 

QCars = []


QCars.append({
    "RobotType": "QCar2", 
    "Location": [-11.324, -7.393, 0.005], 
    "Rotation": [0, 0, -0.7330382858376184], 
    'Radians': True,
    "Scale": 1,
})

QCars.append({
    "RobotType": "QC2", 
    "Location": [1.52, 8.904, 0.005], 
    "Rotation": [0, 0, -0.12], 
    'Radians': True,
    "Scale": 1
})

# QCars.append({
#     "RobotType": "QC2", 
#     "Location": [-18.302, 5.93, 0], 
#     "Rotation": [0, 0, -1.515], 
#     'Radians': True,
#     "Scale": 1
# })

# QCars.append({
#     "RobotType": "QC2", 
#     "Location": [-17.832, 11.507, 0], 
#     "Rotation": [0, 0, -1.664], 
#     'Radians': True,
#     "Scale": 1
# })

# QCars.append({
#     "RobotType": "QC2", 
#     "Location": [22.5478, 00.814, 0], 
#     "Rotation": [0, 0, 1.5707963267948966], 
#     'Radians': True,
#     "Scale": 1
# })



# QCars.append({
#     "RobotType": "QC2", 
#     "Location": [22.5478, 00.814, 0], 
#     "Rotation": [0, 0, 1.5707963267948966], 
#     'Radians': True,
#     "Scale": 1
# })

mySpawns = MultiAgent(QCars)
mySpawns.robotActors[0].set_led_strip_uniform(color=[40,0,0])
mySpawns.robotActors[1].set_led_strip_uniform(color=[0,40,0])

# Function to get position and rotation from user
def get_transform_input(car_number):
    print(f"\n--- Setting Position for Car {car_number} ---")
    
    # Get location
    print(f"Enter location for Car {car_number} (x, y, z), 'D' for default, 'S' to skip, or 'X' to quit:")
    try:
        loc_input = input("Location (e.g., 1.8, 8.904, 0.005): ").strip()
        
        # Check for exit command
        if loc_input.upper() == 'X':
            return None, None
        
        # Check for skip command
        if loc_input.upper() == 'S':
            return 'SKIP', 'SKIP'
        
        # Check for default command
        if loc_input.upper() == 'D':
            location = [-11.324, -7.393, 0.005] if car_number == 0 else [1.52, 8.904, 0.005]
            rotation = [0, 0, -0.73] if car_number == 0 else [0, 0, -0.12]
            print(f"Using default location: {location}")
            print(f"Using default rotation: {rotation}")
            return location, rotation
        
        location = [float(x.strip()) for x in loc_input.split(",")]
        if len(location) != 3:
            raise ValueError("Please provide exactly 3 values")
    except ValueError as e:
        print(f"Invalid input: {e}. Using default location.")
        location = [1.8, 8.904, 0.005] if car_number == 0 else [-12, -8, 0.005]
    
    # Get rotation
    print(f"Enter rotation for Car {car_number} (x, y, z in radians), 'D' for default, 'S' to skip, or 'X' to quit:")
    try:
        rot_input = input("Rotation (e.g., 0, 0, -0.12): ").strip()
        
        # Check for exit command
        if rot_input.upper() == 'X':
            return None, None
        
        # Check for skip command
        if rot_input.upper() == 'S':
            return 'SKIP', 'SKIP'
        
        # Check for default command
        if rot_input.upper() == 'D':
            rotation = [0, 0, -0.12] if car_number == 0 else [0, 0, 0]
            print(f"Using default rotation: {rotation}")
            return location, rotation
        
        rotation = [float(x.strip()) for x in rot_input.split(",")]
        if len(rotation) != 3:
            raise ValueError("Please provide exactly 3 values")
    except ValueError as e:
        print(f"Invalid input: {e}. Using default rotation.")
        rotation = [0, 0, -0.12] if car_number == 0 else [0, 0, 0]
    
    return location, rotation

# Continuous loop to set transforms for both vehicles
print("\n=== Multi-Vehicle Transform Control ===")
print("Commands: Type 'D' for default, 'S' to skip, 'X' to quit\n")

while True:
    try:
        # Get user input for Car 0
        location_0, rotation_0 = get_transform_input(0)
        
        # Check if user wants to exit
        if location_0 is None or rotation_0 is None:
            print("\nProgram terminated by user.")
            break
        
        # Check if user wants to skip Car 0
        if location_0 != 'SKIP':
            mySpawns.robotActors[0].set_transform_and_request_state(location=location_0, rotation=rotation_0, enableDynamics=True, headlights=False, leftTurnSignal=False, rightTurnSignal=False, brakeSignal=False, reverseSignal=False)
            print("✓ Car 0 transform set!")
        else:
            print("⊘ Car 0 skipped")
        
        # Get user input for Car 1
        location_1, rotation_1 = get_transform_input(1)
        
        # Check if user wants to exit
        if location_1 is None or rotation_1 is None:
            print("\nProgram terminated by user.")
            break
        
        # Check if user wants to skip Car 1
        if location_1 != 'SKIP':
            mySpawns.robotActors[1].set_transform_and_request_state(location=location_1, rotation=rotation_1, enableDynamics=True, headlights=False, leftTurnSignal=False, rightTurnSignal=False, brakeSignal=False, reverseSignal=False)
            print("✓ Car 1 transform set!")
        else:
            print("⊘ Car 1 skipped")
        
        print("\nReady for new input. Type 'X' to quit.\n")
        
    except KeyboardInterrupt:
        print("\n\nProgram terminated by user.")
        break
    except Exception as e:
        print(f"Error: {e}")
        continue


qlabs.close()


