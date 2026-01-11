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
import os
import yaml
import numpy as np
from qvl.multi_agent import MultiAgent, readRobots
from qvl.qlabs import QuanserInteractiveLabs
from qvl.real_time import QLabsRealTime
from qvl.free_camera import QLabsFreeCamera
from qvl.crosswalk import QLabsCrosswalk
from qvl.roundabout_sign import QLabsRoundaboutSign
from qvl.yield_sign import QLabsYieldSign
from qvl.traffic_light import QLabsTrafficLight
from qvl.basic_shape import QLabsBasicShape
from qvl.stop_sign import QLabsStopSign

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
environment_objects = {
    'crosswalks': [],
    'traffic_lights': [],
    'yield_signs': [],
    'roundabout_signs': [],
    'basic_shapes': [],
    'stop_signs': []
}


def spawn_environment_objects(qlabs_instance):
    """Spawn all environment objects (crosswalks, traffic lights, signs, etc.)"""
    global environment_objects
    
    print("\nSpawning environment objects...")
    
    # region: crosswalk
    NUMCROSSWALKS = 4
    environment_objects['crosswalks'] = []
    
    for i in range(NUMCROSSWALKS):
        environment_objects['crosswalks'].append(QLabsCrosswalk(qlabs_instance))
    
    environment_objects['crosswalks'][0].spawn(location=[-5, 9.5, 0],
                    rotation=[0, 0, np.pi/2], scale=[1, 1, 0.75],
                    configuration=0)
    
    environment_objects['crosswalks'][1].spawn(location=[1.3, 16, 0],
                    rotation=[0, 0, 0], scale=[1, 1, 0.75],
                    configuration=0)
    
    environment_objects['crosswalks'][2].spawn(location=[7.7, 9.5, 0],
                    rotation=[0, 0, np.pi/2], scale=[1, 1, 0.75],
                    configuration=0)
    
    environment_objects['crosswalks'][3].spawn(location=[1.3, 3, 0],
                    rotation=[0, 0, 0], scale=[1, 1, 0.75],
                    configuration=0)
    print("  ✓ Spawned 4 crosswalks")
    # endregion
    
    # region: Traffic Light
    NUMTRAFFICLIGHTS = 4
    environment_objects['traffic_lights'] = []
    
    for i in range(NUMTRAFFICLIGHTS):
        environment_objects['traffic_lights'].append(QLabsTrafficLight(qlabs_instance))
    
    environment_objects['traffic_lights'][0].spawn(location=[-3.77, 13, 0],
                    rotation=[0, 0, np.pi/2],
                    configuration=0)
    
    environment_objects['traffic_lights'][1].spawn(location=[4.9, 14.8, 0],
                    rotation=[0, 0, 0],
                    configuration=0)
    
    environment_objects['traffic_lights'][2].spawn(location=[6.7, 5.7, 0],
                    rotation=[0, 0, -np.pi/2],
                    configuration=0)
    
    environment_objects['traffic_lights'][3].spawn(location=[-2, 4.27, 0],
                    rotation=[0, 0, np.pi],
                    configuration=0)
    print("  ✓ Spawned 4 traffic lights")
    # endregion
    
    # region: Yield sign
    environment_objects['yield_signs'] = []
    yieldSign = QLabsYieldSign(qlabs_instance)
    yieldSign.spawn(location=[0.4, -13, 0],
                    rotation=[0, 0, np.pi])
    environment_objects['yield_signs'].append(yieldSign)
    print("  ✓ Spawned 1 yield sign")
    # endregion
    
    # region: roundabout
    NUMROUNDABOUTSIGNS = 3
    environment_objects['roundabout_signs'] = []
    
    for i in range(NUMROUNDABOUTSIGNS):
        environment_objects['roundabout_signs'].append(QLabsRoundaboutSign(qlabs_instance))
    
    environment_objects['roundabout_signs'][0].spawn(location=[24.5, 33, 0],
                    rotation=[0, 0, -np.pi/2])
    
    environment_objects['roundabout_signs'][1].spawn(location=[4.5, 40, 0],
                    rotation=[0, 0, np.pi])
    
    environment_objects['roundabout_signs'][2].spawn(location=[10.6, 28.5, 0],
                    rotation=[0, 0, np.pi])
    print("  ✓ Spawned 3 roundabout signs")
    # endregion
    
    # # region: Spawning Basic Shapes
    # environment_objects['basic_shapes'] = []
    # hBasicShape = QLabsBasicShape(qlabs_instance)
    
    # # Plus
    # hBasicShape.spawn_id_box_walls_from_end_points(
    #     actorNumber=2,
    #     startLocation=[-13.64, 3.82, 0.0],
    #     endLocation=[-3.08, -7.062, 0.0],
    #     height=5,
    #     thickness=3,
    #     color=[(154/255), (101/255), (14/255)],
    #     waitForConfirmation=False
    # )
    
    # hBasicShape.spawn_id_box_walls_from_end_points(
    #     actorNumber=3,
    #     startLocation=[-3.93, 4.00, -0],
    #     endLocation=[-10.034, -3.102, -0],
    #     height=5,
    #     thickness=3,
    #     color=[(154/255), (101/255), (14/255)],
    #     waitForConfirmation=False
    # )
    
    # # Roundabout Box
    # hBasicShape.spawn_id_box_walls_from_end_points(
    #     actorNumber=4,
    #     startLocation=[12.104, 38.266, 0],
    #     endLocation=[18.345, 38.433, 0],
    #     height=5,
    #     thickness=4,
    #     color=[(154/255), (101/255), (14/255)],
    #     waitForConfirmation=False
    # )
    
    # # Basic Building Box
    # hBasicShape.spawn_id_box_walls_from_end_points(
    #     actorNumber=5,
    #     startLocation=[5.969, 0.072, 0.385],
    #     endLocation=[16.578, -0.016, 0],
    #     height=5,
    #     thickness=8.5,
    #     color=[(154/255), (101/255), (14/255)],
    #     waitForConfirmation=False
    # )
    # environment_objects['basic_shapes'].append(hBasicShape)
    # print("  ✓ Spawned basic shapes (buildings)")
    # # endregion
    
    # region: Spawn stopsign
    environment_objects['stop_signs'] = []
    stopSign = QLabsStopSign(qlabs_instance)
    stopSign.spawn(location=[-0.508, -7.327, 0.2], rotation=[0, 0, np.pi/2],
                scale=[1, 1, 1], configuration=0, waitForConfirmation=True)
    environment_objects['stop_signs'].append(stopSign)
    print("  ✓ Spawned 1 stop sign")
    # endregion
    
    print("Environment objects spawned successfully!\n")


def destroy_environment_objects(qlabs_instance):
    """Destroy all spawned environment objects"""
    global environment_objects
    
    print("\nDestroying environment objects...")
    
    # Destroy all objects by type
    for obj_type, obj_list in environment_objects.items():
        for obj in obj_list:
            try:
                obj.destroy()
            except:
                pass
    
    # Clear the dictionary
    environment_objects = {
        'crosswalks': [],
        'traffic_lights': [],
        'yield_signs': [],
        'roundabout_signs': [],
        'basic_shapes': [],
        'stop_signs': []
    }
    
    print("Environment objects destroyed!\n")


def resolve_config_path(config_path: str) -> str:
    # Resolve path relative to this script, sibling folders, or absolute
    if os.path.isabs(config_path):
        return config_path
    script_dir = os.path.dirname(__file__)
    candidates = [
        os.path.join(script_dir, config_path),
        os.path.normpath(os.path.join(script_dir, '..', config_path)),
        os.path.abspath(config_path),
    ]
    for c in candidates:
        if os.path.exists(c):
            return os.path.normpath(c)
    # fallback to provided path
    return os.path.normpath(os.path.join(script_dir, config_path))


def load_fleet_config(path: str = '../multi_vehicle_self_driving_RealQcar/fleet_config.yaml') -> dict:
    cfg_path = resolve_config_path(path)
    print(f"Loading fleet config from: {cfg_path}")
    if not os.path.exists(cfg_path):
        raise FileNotFoundError(f"fleet_config.yaml not found at {cfg_path}")
    with open(cfg_path, 'r') as f:
        return yaml.safe_load(f)


def build_qcars_from_config(cfg: dict) -> list:
    qcars = []
    nodes = cfg.get('nodes', {})
    paths = cfg.get('paths', {})

    # fallback node poses (same reference used in qcar/config_main.py)
    fallback_node_poses = {
        0: [0.0, 0.13024, -1.5707963267948966],
        1: [0.26861999999999997, 0.0814, 1.5707963267948966],
        2: [1.12739, -1.084655, 0.0],
        3: [1.12739, -0.814, 3.141592653589793],
        4: [2.25478, 0.0814, 1.5707963267948966],
        5: [1.984125, 0.0814, -1.5707963267948966],
        6: [1.01343, 1.100935, 3.141592653589793],
        7: [1.235245, 0.83028, 0.0],
        8: [-0.74888, 1.100935, 3.141592653589793],
        9: [-0.74888, 0.83028, 0.0],
        10: [-1.28205, -0.45991, -0.7330382858376184]
    }

    for v in cfg.get('vehicles', []):
        if not v.get('enabled', True):
            continue

        # Determine RobotType from vehicle_type
        vtype = v.get('vehicle_type', 'Qcar')
        robot_type = 'QCar2' if str(vtype).lower().startswith('qcar') else 'QC2'

        # Determine starting node/pose
        path_number = v.get('path_number', 0)
        path_def = paths.get(path_number, paths.get(str(path_number), {}))
        node_list = path_def.get('nodes') if isinstance(path_def, dict) else None
        if not node_list:
            # fallback: try first path or node 0
            node_list = [0]

        first_node_id = node_list[0]
        node_key = first_node_id if first_node_id in nodes else str(first_node_id)
        node_entry = nodes.get(node_key, nodes.get(first_node_id))

        print (f"[spawn] vehicle {v.get('car_id')} assigned to path {path_number} starting at node {first_node_id}")
        print (f"[spawn] node entry: {node_entry}")
        if node_entry and 'pose' in node_entry:
            pose = node_entry['pose']
            location = [pose[0], pose[1], 0.005]
            rotation = [0, 0, pose[2]]
            print(f"[spawn] vehicle {v.get('car_id')} uses node {first_node_id} pose from fleet_config: {pose}")
        else:
            # Try fallback poses defined in code (keeps behavior consistent with config_main)
            pose = fallback_node_poses.get(first_node_id)
            if pose:
                location = [pose[0], pose[1], 0.005]
                rotation = [0, 0, pose[2]]
                print(f"[spawn] vehicle {v.get('car_id')} uses fallback node {first_node_id} pose: {pose}")
            else:
                # absolute fallback
                location = [0.0, 0.0, 0.005]
                rotation = [0, 0, 0.0]
                print(f"[spawn] vehicle {v.get('car_id')} has no node pose available, using {location} {rotation}")

        qcars.append({
            'RobotType': robot_type,
            'Location': [coord * 10 for coord in location],  # scale up for QLabs
            'Rotation': rotation,
            'Radians': True,
            'Scale': 1,
            'config': v,
        })

    return qcars


# Try to load fleet config and build QCars list; if fails, fall back to the original hardcoded spawns
try:
    cfg = load_fleet_config()
    QCars = build_qcars_from_config(cfg)
    if not QCars:
        print('No enabled vehicles found in fleet_config.yaml; falling back to defaults')
        raise RuntimeError('empty fleet')
    print(f"Built {len(QCars)} spawn definitions from fleet_config.yaml")
except Exception as e:
    print(f"[Warning] Could not build spawns from fleet_config.yaml: {e}")
    # legacy defaults
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


# print(QCars[0])
# print(QCars[1])
mySpawns = MultiAgent(QCars)
# Set LED colors for spawned vehicles (safe for any number of vehicles)
colors = [[40,0,0], [0,40,0], [0,0,40], [40,40,0]]
for i, actor in enumerate(mySpawns.robotActors):
    actor.set_led_strip_uniform(color=colors[i % len(colors)])
def get_transform_input(car_number):
    print(f"\n--- Setting Position for Car {car_number} ---")

    default_loc = QCars[car_number].get("Location", [0.0, 0.0, 0.005])
    default_rot = QCars[car_number].get("Rotation", [0.0, 0.0, 0.0])

    # We only care about x,y,theta for the prompt
    dx, dy = float(default_loc[0]), float(default_loc[1])
    dtheta = float(default_rot[2])

    prompt = (
        f"Enter X,Y or X,Y,THETA for Car {car_number}\n"
        f"  - D = default ({dx:.3f}, {dy:.3f}, {dtheta:.3f})\n"
        f"  - S = skip this car\n"
        f"  - X = quit (no shutdown)\n"
        f"  - Q = quit and shutdown QLabs real-time models\n"
        f"  - E = toggle environment objects\n"
        f"> "
    )

    s = input(prompt).strip()

    if not s:
        # empty input -> treat as default
        return default_loc, default_rot

    cmd = s.upper()
    if cmd == "X":
        return None, None
    if cmd == "Q":
        print("[shutdown] Terminating QLabs real-time models and exiting...")
        QLabsRealTime().terminate_all_real_time_models()
        # qlabs.destroy_all_spawned_actors()
        return None, None
    if cmd == "S":
        return "SKIP", "SKIP"
    if cmd == "D":
        return default_loc, default_rot
    if cmd == "E":
        # Toggle environment objects
        qlabs_temp = QuanserInteractiveLabs()
        if qlabs_temp.open("localhost"):
            env_action = input("  [S]pawn or [D]estroy environment objects? ").strip().upper()
            if env_action == 'S':
                spawn_environment_objects(qlabs_temp)
            elif env_action == 'D':
                destroy_environment_objects(qlabs_temp)
            qlabs_temp.close()
        else:
            print("Could not connect to QLabs")
        return "SKIP", "SKIP"

    # Parse numeric input: X,Y or X,Y,THETA
    parts = [p.strip() for p in s.split(",") if p.strip() != ""]
    if len(parts) not in (2, 3):
        print("Invalid format. Using default.")
        return default_loc, default_rot

    try:
        x = float(parts[0])
        y = float(parts[1])
        theta = float(parts[2]) if len(parts) == 3 else dtheta
    except ValueError:
        print("Invalid numbers. Using default.")
        return default_loc, default_rot

    location = [x, y, 0.005]
    rotation = [0.0, 0.0, theta]
    return location, rotation

    

# Ask user if they want to spawn environment objects
print("\n=== Environment Setup ===")
env_choice = input("Spawn environment objects (crosswalks, traffic lights, signs)? (Y/N): ").strip().upper()
if env_choice == 'Y':
    # Need to reconnect to QLabs for spawning
    qlabs_temp = QuanserInteractiveLabs()
    if qlabs_temp.open("localhost"):
        spawn_environment_objects(qlabs_temp)
        qlabs_temp.close()
    else:
        print("Could not connect to QLabs for environment spawning")

# Continuous loop to set transforms for both vehicles
print("\n=== Multi-Vehicle Transform Control ===")
print("Commands:")
print("  D = default position")
print("  S = skip this car")
print("  X = quit (no shutdown)")
print("  Q = quit and shutdown QLabs")
print("  E = spawn/destroy environment objects")
print("  X,Y or X,Y,THETA = set position\n")

while True:
    try:
        loc0, rot0 = get_transform_input(0)
        if loc0 is None:  # quit
            print("\nProgram terminated by user.")
            break

        if loc0 != "SKIP":
            if len(mySpawns.robotActors) > 0:
                mySpawns.robotActors[0].set_transform_and_request_state(
                    location=loc0,  # already [x,y,z]
                    rotation=rot0,  # already [0,0,theta]
                    enableDynamics=True,
                    headlights=False,
                    leftTurnSignal=False,
                    rightTurnSignal=False,
                    brakeSignal=False,
                    reverseSignal=False,
                )
                print("✓ Car 0 transform set!")
            else:
                print("⊘ No spawned Car 0 to set transform for")
        else:
            print("⊘ Car 0 skipped")

        loc1, rot1 = get_transform_input(1)
        if loc1 is None:
            print("\nProgram terminated by user.")
            break

        if loc1 != "SKIP":
            if len(mySpawns.robotActors) > 1:
                mySpawns.robotActors[1].set_transform_and_request_state(
                    location=loc1,
                    rotation=rot1,
                    enableDynamics=True,
                    headlights=False,
                    leftTurnSignal=False,
                    rightTurnSignal=False,
                    brakeSignal=False,
                    reverseSignal=False,
                )
                print("✓ Car 1 transform set!")
            else:
                print("⊘ No spawned Car 1 to set transform for")
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


