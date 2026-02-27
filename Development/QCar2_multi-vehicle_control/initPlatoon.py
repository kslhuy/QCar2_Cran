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
from qvl.environment_outdoors import QLabsEnvironmentOutdoors
from qvl.system import QLabsSystem
from qvl.real_time import QLabsRealTime
from qvl.free_camera import QLabsFreeCamera
from qvl.walls import QLabsWalls
from qvl.basic_shape import QLabsBasicShape

# 
num_cars = 4
car_type = "QC2"
init_location = [75, -6.2, 1.131]
init_distance = 15  # distance between cars
init_rotation = [0, 0, 0]
wave_road  = False  # Set to True to create wavy road at 500m, False for flat road

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

hSystem = QLabsSystem(qlabs)
### Outdoor Environment
# hEnvironmentOutdoors2 = QLabsEnvironmentOutdoors(qlabs)
# hEnvironmentOutdoors2.set_weather_preset(hEnvironmentOutdoors2.BLIZZARD)
# hSystem.set_title_string('Blizzard')

# create a camera in this qlabs instance
camera = QLabsFreeCamera(qlabs)
camera.spawn_degrees(location=[450, -21, 8], rotation=[0, 11, 122])
# to switch our view from our current camera to the new camera we just initialized
camera.possess()

# ============== Create Wavy Road at 500m (Conditional) ==============
if wave_road:
    print("\n=== Creating Wavy Road at 500m ===")

    # Wavy road parameters
    wavy_road_start_x = 550  # 500m from starting point (x = 75 + 500 = 575m)
    road_y = init_location[1]  # Same y as the main road

    # Wave parameters (gentle slopes)
    wave_amplitude = 1  # Height variation (meters) - gentle slope
    num_segments = 34  # Number of road segments
    segment_length = 5.0  # Length of each segment (meters)
    num_waves = 4  # Number of complete waves (ensures start and end touch ground)
    total_road_length = (num_segments - 1) * segment_length  # Total length = 85m
    wave_length = total_road_length / num_waves  # Auto-calculate wavelength (42.5m)
    road_width = 8.0  # Width of the road (meters)
    road_thickness = 0.1  # Thickness of road surface

    # Use cosine wave: ensures start/end at ground (z=1.131), lowest point at ground
    # z = ground + amplitude - amplitude * cos(phase) → range [ground, ground + 2*amplitude]
    road_base_z = init_location[2] + wave_amplitude  # Offset so lowest point is at ground level (z=1.131)

    hBasicShape = QLabsBasicShape(qlabs)

    for i in range(num_segments):
        # Calculate x position for this segment
        x_pos = wavy_road_start_x - i * segment_length
        
        # Calculate phase for this segment
        phase = 2 * np.pi * i * segment_length / wave_length
        
        # Calculate z position using cosine wave (start/end at ground)
        # cos(0) = 1 → z = amplitude - amplitude = 0 (ground)
        # cos(π) = -1 → z = amplitude + amplitude = 2*amplitude (peak)
        z_pos = road_base_z - wave_amplitude * np.cos(phase)
        
        # Calculate slope angle for road segment rotation (pitch)
        # Derivative of -cos is sin, scaled appropriately
        slope_angle = np.arctan(wave_amplitude * (2 * np.pi / wave_length) * np.sin(phase))
        
        # Spawn road segment as a flat cube
        success = hBasicShape.spawn_id(
            actorNumber=100 + i,  # Unique actor number starting from 100
            location=[x_pos, road_y, z_pos],
            rotation=[0, slope_angle, 0],  # Pitch rotation for slope
            scale=[segment_length, road_width, road_thickness],
            configuration=QLabsBasicShape.SHAPE_CUBE,
            waitForConfirmation=False
        )
        
        if success:
            # Set road color (asphalt gray)
            hBasicShape.set_material_properties(
                color=[0.2, 0.2, 0.2],  # Dark gray asphalt color
                roughness=0.8,
                metallic=False,
                waitForConfirmation=False
            )
            # Disable dynamics so road stays fixed
            hBasicShape.set_enable_dynamics(enableDynamics=False, waitForConfirmation=False)

    print(f"Created {num_segments} road segments forming wavy road at x = {wavy_road_start_x}m")
    print(f"  Wave amplitude: {wave_amplitude}m, wavelength: {wave_length}m")
    print(f"  Road length: {num_segments * segment_length}m")
else:
    print("\n=== Wavy Road Creation Skipped (wave_road = False) ===")
# ============== End Wavy Road ==============

qlabs.close()

print("Disconnected from camera qlabs session") 

QCars = []

for i in range(num_cars):
    QCars.append({
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
    })



mySpawns = MultiAgent(QCars)
mySpawns.robotActors[0].set_led_strip_uniform(color=[40,0,0])
mySpawns.robotActors[1].set_led_strip_uniform(color=[0,40,0])

mySpawns.robotActors[3].possess()  # 跟随车辆3（最后一辆）

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
        print(f"[OK] Car {car_id} transform set to default!")

    print("All vehicles configured with default transforms.")

except KeyboardInterrupt:
    print("\n\nProgram terminated by user.")
except Exception as e:
    print(f"Error: {e}")


qlabs.close()


