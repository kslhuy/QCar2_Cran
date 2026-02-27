# region: package imports
import os
import time

# environment objects

from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.free_camera import QLabsFreeCamera
from qvl.real_time import QLabsRealTime
from qvl.basic_shape import QLabsBasicShape
from qvl.system import QLabsSystem
from qvl.walls import QLabsWalls
from qvl.qcar_flooring import QLabsQCarFlooring
from qvl.crosswalk import QLabsCrosswalk
from qvl.traffic_light import QLabsTrafficLight
from qvl.person import QLabsPerson

from qvl.roundabout_sign import QLabsRoundaboutSign
from qvl.yield_sign import QLabsYieldSign
from qvl.stop_sign import QLabsStopSign


# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++#

# This scenario was designed to by used in the Plane world for the Self Driving Car Studio

# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++#

# endregion


# Function to setup QLabs, Spawn in QCar, and run real time model
def setup(initialPosition=[-1.205, -0.83, 0.005], initialOrientation=[0, 0, -44.7]):
    # Try to connect to Qlabs

    os.system("cls")
    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    try:
        qlabs.open("localhost")
        print("Connected to QLabs")
    except:
        print("Unable to connect to QLabs")
        quit()

    # Delete any previous QCar instances and stop any running spawn models
    qlabs.destroy_all_spawned_actors()
    QLabsRealTime().terminate_all_real_time_models()

    # Set the Workspace Title
    hSystem = QLabsSystem(qlabs)
    x = hSystem.set_title_string(
        "ACC Self Driving Car Competition", waitForConfirmation=True
    )

    ### Flooring

    x_offset = 0.13
    y_offset = 1.67
    hFloor = QLabsQCarFlooring(qlabs)
    hFloor.spawn_degrees(
        [x_offset, y_offset, 0.001], rotation=[0, 0, -90], configuration=0
    )

    ### region: Walls
    hWall = QLabsWalls(qlabs)
    hWall.set_enable_dynamics(False)

    for y in range(5):
        hWall.spawn_degrees(
            location=[-2.4 + x_offset, (-y * 1.0) + 2.55 + y_offset, 0.001],
            rotation=[0, 0, 0],
        )

    for x in range(5):
        hWall.spawn_degrees(
            location=[-1.9 + x + x_offset, 3.05 + y_offset, 0.001], rotation=[0, 0, 90]
        )

    for y in range(6):
        hWall.spawn_degrees(
            location=[2.4 + x_offset, (-y * 1.0) + 2.55 + y_offset, 0.001],
            rotation=[0, 0, 0],
        )

    for x in range(4):
        hWall.spawn_degrees(
            location=[-0.9 + x + x_offset, -3.05 + y_offset, 0.001], rotation=[0, 0, 90]
        )

    hWall.spawn_degrees(
        location=[-2.03 + x_offset, -2.275 + y_offset, 0.001], rotation=[0, 0, 48]
    )
    hWall.spawn_degrees(
        location=[-1.575 + x_offset, -2.7 + y_offset, 0.001], rotation=[0, 0, 48]
    )

    # Spawn first QCar at the given initial pose
    car1 = QLabsQCar2(qlabs)
    car1.spawn_id(
        actorNumber=0,
        location=initialPosition,
        rotation=initialOrientation,
        scale=[0.1, 0.1, 0.1],
        configuration=0,
        waitForConfirmation=True,
    )

    # # Spawn second QCar at specified pose
    # car2 = QLabsQCar2(qlabs)
    # car2.spawn_id(
    #     actorNumber=1,
    #     location=[0.26861999999999997, 1.849815, 0.006],
    #     rotation=[0, 0, 1.5707963267948966],
    #     scale=[0.1, 0.1, 0.1],
    #     configuration=0,
    #     waitForConfirmation=True,
    # )

    rtModel = os.path.normpath(
        os.path.join(os.environ["RTMODELS_DIR"], "QCar2/QCar2_Workspace_studio")
    )
    QLabsRealTime().start_real_time_model(rtModel)

    # spawn cameras 1. birds eye, 2. edge 1, possess the qcar

    camera1Loc = [0.15, 1.7, 5]
    camera1Rot = [0, 90, 0]
    camera1 = QLabsFreeCamera(qlabs)
    camera1.spawn_degrees(location=camera1Loc, rotation=camera1Rot)

    camera1.possess()

    camera2Loc = [-0.36 + x_offset, -3.691 + y_offset, 2.652]
    camera2Rot = [0, 47, 90]
    camera2 = QLabsFreeCamera(qlabs)
    camera2.spawn_degrees(location=camera2Loc, rotation=camera2Rot)

    # Spawning crosswalks
    myCrossWalk = QLabsCrosswalk(qlabs)
    myCrossWalk.spawn_degrees(
        location=[-2 + x_offset, -1.475 + y_offset, 0.01],
        rotation=[0, 0, 0],
        scale=[0.1, 0.1, 0.075],
        configuration=0,
    )

    mySpline = QLabsBasicShape(qlabs)
    mySpline.spawn_degrees(
        location=[2.05 + x_offset, -1.5 + y_offset, 0.01],
        rotation=[0, 0, 0],
        scale=[0.27, 0.02, 0.001],
        waitForConfirmation=False,
    )

    # Spawning traffic lights (4 total)
    trafficLight1 = QLabsTrafficLight(qlabs)
    trafficLight2 = QLabsTrafficLight(qlabs)
    trafficLight3 = QLabsTrafficLight(qlabs)
    trafficLight4 = QLabsTrafficLight(qlabs)

    traffic_lights = [trafficLight1, trafficLight2, trafficLight3, trafficLight4]

    trafficLight1.spawn_id_degrees(
        actorNumber=1,
        location=[0.6, 1.55, 0.006],
        rotation=[0, 0, 0],
        scale=[0.1, 0.1, 0.1],
        configuration=0,
        waitForConfirmation=False,
    )
    trafficLight2.spawn_id_degrees(
        actorNumber=2,
        location=[-0.6, 1.28, 0.006],
        rotation=[0, 0, 90],
        scale=[0.1, 0.1, 0.1],
        configuration=0,
        waitForConfirmation=False,
    )
    trafficLight3.spawn_id_degrees(
        actorNumber=3,
        location=[-0.37, 0.3, 0.006],
        rotation=[0, 0, 180],
        scale=[0.1, 0.1, 0.1],
        configuration=0,
        waitForConfirmation=False,
    )
    trafficLight4.spawn_id_degrees(
        actorNumber=4,
        location=[0.75, 0.48, 0.006],
        rotation=[0, 0, -90],
        scale=[0.1, 0.1, 0.1],
        configuration=0,
        waitForConfirmation=False,
    )



    # Spawn stop signs (4 total)
    myStopSign = QLabsStopSign(qlabs)

    # Parking lot stop signs
    myStopSign.spawn_degrees(
        location=[-1.5, 3.6, 0.006],
        rotation=[0, 0, -35],
        scale=[0.1, 0.1, 0.1],
        waitForConfirmation=False,
    )

    myStopSign.spawn_degrees(
        location=[-1.5, 2.2, 0.006],
        rotation=[0, 0, 35],
        scale=[0.1, 0.1, 0.1],
        waitForConfirmation=False,
    )

    # X+ side stop signs
    myStopSign.spawn_degrees(
        location=[2.410, 0.206, 0.006],
        rotation=[0, 0, -90],
        scale=[0.1, 0.1, 0.1],
        waitForConfirmation=False,
    )

    myStopSign.spawn_degrees(
        location=[1.766, 1.697, 0.006],
        rotation=[0, 0, 90],
        scale=[0.1, 0.1, 0.1],
        waitForConfirmation=False,
    )
    print("  ✓ Spawned 4 stop signs")

    # Spawn roundabout signs (3 total)
    myRoundaboutSign = QLabsRoundaboutSign(qlabs)

    myRoundaboutSign.spawn_degrees(
        location=[2.392, 2.522, 0.006],
        rotation=[0, 0, -90],
        scale=[0.1, 0.1, 0.1],
        waitForConfirmation=False,
    )

    myRoundaboutSign.spawn_degrees(
        location=[0.698, 2.483, 0.006],
        rotation=[0, 0, -145],
        scale=[0.1, 0.1, 0.1],
        waitForConfirmation=False,
    )

    myRoundaboutSign.spawn_degrees(
        location=[0.007, 3.973, 0.006],
        rotation=[0, 0, 135],
        scale=[0.1, 0.1, 0.1],
        waitForConfirmation=False,
    )
    print("  ✓ Spawned 3 roundabout signs")

    # Spawn yield signs (4 total)
    myYieldSign = QLabsYieldSign(qlabs)

    # One way exit yield
    myYieldSign.spawn_degrees(
        location=[0.0, -1.3, 0.006],
        rotation=[0, 0, -180],
        scale=[0.1, 0.1, 0.1],
        waitForConfirmation=False,
    )

    # Roundabout yields
    myYieldSign.spawn_degrees(
        location=[2.4, 3.2, 0.006],
        rotation=[0, 0, -90],
        scale=[0.1, 0.1, 0.1],
        waitForConfirmation=False,
    )

    myYieldSign.spawn_degrees(
        location=[1.1, 2.8, 0.006],
        rotation=[0, 0, -145],
        scale=[0.1, 0.1, 0.1],
        waitForConfirmation=False,
    )

    myYieldSign.spawn_degrees(
        location=[0.49, 3.8, 0.006],
        rotation=[0, 0, 135],
        scale=[0.1, 0.1, 0.1],
        waitForConfirmation=False,
    )
    print("  ✓ Spawned 4 yield signs")

    # Spawning pedestrian
    pedestrian = QLabsPerson(qlabs)
    pedestrian.spawn_id_degrees(
        actorNumber=1,
        location=[-1.439, 0.146, 0.006],
        rotation=[0, 0, 0],
        scale=[0.1, 0.1, 0.1],
        configuration=6,
    )

    return car1, traffic_lights, pedestrian


# function to terminate the real time model running
def terminate():
    rtModel = os.path.normpath(
        os.path.join(os.environ["RTMODELS_DIR"], "QCar2/QCar2_Workspace_studio")
    )
    QLabsRealTime().terminate_real_time_model(rtModel)


def main_loop(traffic_lights, pedestrian):
    """Main loop that cycles traffic lights and moves the pedestrian."""
    t1, t2, t3, t4 = traffic_lights
    traffic_state = 0

    # Pedestrian waypoints (your locations)
    PEDESTRIAN_LOC0 = [-0.984, -0.083, 0.006]
    PEDESTRIAN_LOC1 = [-1.119, 0.224, 0.006]
    PEDESTRIAN_LOC2 = [-2.178, 0.224, 0.006]
    PEDESTRIAN_LOC3 = [-2.165, 0.817, 0.006]

    setpointFlag = 0
    locationCounter = 0

    print("[SERVER] Traffic light sequence started")
    print("[SERVER] Pedestrian sequence started")

    try:
        while True:
            # --- Traffic lights ---
            if traffic_state == 0:  # EW Green, NS Red
                t1.set_color(color=QLabsTrafficLight.COLOR_RED)
                t3.set_color(color=QLabsTrafficLight.COLOR_RED)
                t2.set_color(color=QLabsTrafficLight.COLOR_GREEN)
                t4.set_color(color=QLabsTrafficLight.COLOR_GREEN)
            elif traffic_state == 1:  # EW Yellow, NS Red
                t1.set_color(color=QLabsTrafficLight.COLOR_RED)
                t3.set_color(color=QLabsTrafficLight.COLOR_RED)
                t2.set_color(color=QLabsTrafficLight.COLOR_YELLOW)
                t4.set_color(color=QLabsTrafficLight.COLOR_YELLOW)
            elif traffic_state == 2:  # EW Red, NS Green
                t1.set_color(color=QLabsTrafficLight.COLOR_GREEN)
                t3.set_color(color=QLabsTrafficLight.COLOR_GREEN)
                t2.set_color(color=QLabsTrafficLight.COLOR_RED)
                t4.set_color(color=QLabsTrafficLight.COLOR_RED)
            elif traffic_state == 3:  # EW Red, NS Yellow
                t1.set_color(color=QLabsTrafficLight.COLOR_YELLOW)
                t3.set_color(color=QLabsTrafficLight.COLOR_YELLOW)
                t2.set_color(color=QLabsTrafficLight.COLOR_RED)
                t4.set_color(color=QLabsTrafficLight.COLOR_RED)

            traffic_state = (traffic_state + 1) % 4

            # --- Pedestrian: varying speeds with pauses (from ACC pedestrian scenario) ---
            if locationCounter == 0:
                setpointFlag = pedestrian.move_to(location=PEDESTRIAN_LOC1, speed=pedestrian.JOG, waitForConfirmation=True)
            if locationCounter == 1:
                setpointFlag = pedestrian.move_to(location=PEDESTRIAN_LOC2, speed=pedestrian.WALK, waitForConfirmation=True)
                time.sleep(4)
            if locationCounter == 2:
                setpointFlag = pedestrian.move_to(location=PEDESTRIAN_LOC3, speed=pedestrian.JOG, waitForConfirmation=True)
            if locationCounter == 3:
                setpointFlag = pedestrian.move_to(location=PEDESTRIAN_LOC2, speed=pedestrian.JOG, waitForConfirmation=True)
            if locationCounter == 4:
                setpointFlag = pedestrian.move_to(location=PEDESTRIAN_LOC1, speed=pedestrian.WALK, waitForConfirmation=True)
                time.sleep(4)
            if locationCounter == 5:
                setpointFlag = pedestrian.move_to(location=PEDESTRIAN_LOC0, speed=pedestrian.JOG, waitForConfirmation=True)

            if setpointFlag == 1:
                locationCounter += 1
                locationCounter = locationCounter % 6

            time.sleep(2)

    except KeyboardInterrupt:
        print("\n[SERVER] Stopped by user")
    except Exception as e:
        print(f"Error in main loop: {e}")


if __name__ == "__main__":
    cars, traffic_lights, pedestrian = setup()
    try:
        main_loop(traffic_lights, pedestrian)
    except KeyboardInterrupt:
        pass
    finally:
        terminate()
