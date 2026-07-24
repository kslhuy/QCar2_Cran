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


class StaggeredMultiAgent(MultiAgent):
    """Start QCar RT models sequentially with a settling delay."""

    def __init__(self, agent_list, model_start_delay=2.0):
        self.model_start_delay = model_start_delay
        self.qcar_model_count = sum(
            agent.get("RobotType", "").lower() in ("qc2", "qcar2", "qcar 2")
            for agent in agent_list
        )
        self.started_qcar_models = 0
        super().__init__(agent_list)

    def _createQC2(self, actor_number, scale):
        name, robot_config = super()._createQC2(actor_number, scale)

        self.started_qcar_models += 1
        print(
            f"[RT] QCar {actor_number} started "
            f"({self.started_qcar_models}/{self.qcar_model_count}); "
            f"settling for {self.model_start_delay:.1f}s..."
        )
        time.sleep(self.model_start_delay)

        return name, robot_config


# 
num_cars = 4
car_type = "QC2"
init_location = [75, -6.2, 1.131]
init_distance = 15  # distance between cars
init_rotation = [0, 0, 0]
wave_road  = True  # Set to True to create wavy road at 500m, False for flat road
rt_model_start_delay = 2.0
rt_models_settle_delay = 3.0
rt_models_shutdown_delay = 2.0
actor_cleanup_delay = 1.0

# ===== Global Variables - Exported for Other Modules =====
# These will be set during initialization and made available for import
qlabs = None  # Will hold QuanserInteractiveLabs instance
mySpawns = None  # Will hold MultiAgent with robots
camera = None  # Will hold QLabsFreeCamera instance

# Initialize QLabs connection
qlabs = QuanserInteractiveLabs()
    
print("Connecting to QLabs...")
try:
    qlabs.open("localhost")
    print("Connected to QLabs")

    print("[INIT] Terminating existing RT models...")
    QLabsRealTime().terminate_all_real_time_models()
    time.sleep(rt_models_shutdown_delay)

    print("[INIT] Destroying all existing spawned actors...")
    qlabs.destroy_all_spawned_actors()
    time.sleep(actor_cleanup_delay)
    print("[INIT] Existing RT models and actors cleared")
except:
    print("Unable to connect to QLabs")
    quit() 

print("Connected")  

hSystem = QLabsSystem(qlabs)
### Outdoor Environment
# hEnvironmentOutdoors2 = QLabsEnvironmentOutdoors(qlabs)
# hEnvironmentOutdoors2.set_weather_preset(hEnvironmentOutdoors2.BLIZZARD)
# hSystem.set_title_string('Blizzard')

# create a camera in this qlabs instance
# Camera will be attached to car 3 after vehicles are spawned.

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

# NOTE: Do NOT close qlabs here! Keep connection open for other processes
# (camera_tracker_main.py, vehicle_main.py) to connect to same QLabs instance
# qlabs.close()  # REMOVED - breaks multi-process coordination 

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



mySpawns = StaggeredMultiAgent(
    QCars,
    model_start_delay=rt_model_start_delay,
)
print(
    f"[RT] All QCar models started; waiting "
    f"{rt_models_settle_delay:.1f}s before configuring actors..."
)
time.sleep(rt_models_settle_delay)

mySpawns.robotActors[0].set_led_strip_uniform(color=[40,0,0])
mySpawns.robotActors[1].set_led_strip_uniform(color=[0,40,0])

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

    # Spawn free camera attached to car 3 (4th car, 0-indexed)
    # Calculate relative position: camera stays at fixed offset from car 3
    car3_initial_location = [
        init_location[0] - 3 * init_distance,
        init_location[1],
        init_location[2],
    ]
    camera_initial_world_location = [27.898, 8.143, 9.485]
    camera_initial_rotation_deg = [0, 11.903, -43.203]
    
    # Compute relative offset from car 3
    camera_relative_location = [
        -1,
        10,
        5,
    ]

    camera = QLabsFreeCamera(mySpawns.qlabs)
    # Get car 3 (4th car) reference for parent information
    car3 = mySpawns.robotActors[3]
    
    # Spawn camera directly parented to car 3 with relative transform
    camera_status = camera.spawn_id_and_parent_with_relative_transform_degrees(
        actorNumber=9000,
        location=camera_relative_location,
        rotation=camera_initial_rotation_deg,
        parentClassID=car3.classID,
        parentActorNumber=car3.actorNumber,
        parentComponent=0,
        waitForConfirmation=True
    )
    if camera_status != 0:
        raise RuntimeError(f"Camera spawn failed with status={camera_status}")

    camera.possess()
    print("[OK] Camera spawned and parented to car 2 at relative position.")
    print("[INFO] Camera will now follow car 2's movements while maintaining relative transform.")
    # Variables mySpawns, camera, and qlabs are now available at module level for import

except KeyboardInterrupt:
    print("\n\nProgram terminated by user.")
except Exception as e:
    print(f"Error during initialization: {e}")


# Script execution: if run directly, show info message
if __name__ == "__main__":
    print("\n" + "="*70)
    print("initPlatoon.py - QLabs Initialization Script")
    print("="*70)
    print("[INFO] Initialization complete!")
    print("[INFO] Global variables set: mySpawns, camera, qlabs")
    print("[INFO] To start camera tracking, run: python camera_tracker_main.py")
    print("="*70 + "\n")


