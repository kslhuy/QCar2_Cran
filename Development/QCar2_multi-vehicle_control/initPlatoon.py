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
num_cars = 4
car_type = "QC2"
init_location = [0, -6.2, 1.131]
init_distance = 15
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
camera.spawn_degrees(location=[30, -16, 35], rotation=[0, 44, 141.502])
# to switch our view from our current camera to the new camera we just initialized
camera.possess()

qlabs.close()

print("Disconnected from camera qlabs session") 

QCars = []

for i in range(num_cars):
    QCars.append({
        "ActorNumber": i,
        "RobotType": car_type,
        "Location": [
            init_location[0] + i * init_distance,
            init_location[1],
            init_location[2],
        ],
        "Rotation": init_rotation,  
        "Radians": True,
        "Scale": 1,
    })



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
            location = [init_location[0] + i * init_distance,
            init_location[1],
            init_location[2],] 
            rotation = init_rotation
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
            rotation = init_rotation
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
        exit_requested = False
        for car_id in range(num_cars):
            # Get user input for Car i
            location_i, rotation_i = get_transform_input(car_id)
            # Check if user wants to exit
            if location_i is None or rotation_i is None: # If the user input is X
                exit_requested = True
                break
            # Check if user wants to skip Car i
            if location_i!= 'SKIP':
                mySpawns.robotActors[i].set_transform_and_request_state(location=location_i, rotation=rotation_i, enableDynamics=True, headlights=False, leftTurnSignal=False, rightTurnSignal=False, brakeSignal=False, reverseSignal=False)
                print(f"✓ Car {car_id} transform set!")
            else:
                print(f"⊘ Car {car_id} skipped")

        if exit_requested:
            print("\n\nProgram terminated by user.")
            break

        print("All vehicles configured.")
        break

        
    except KeyboardInterrupt:
        print("\n\nProgram terminated by user.")
        break
    except Exception as e:
        print(f"Error: {e}")
        continue


qlabs.close()


