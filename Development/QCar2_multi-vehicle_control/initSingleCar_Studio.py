"""
Single Vehicle QCar 2 example - Simplified.

Make sure you have Quanser Interactive Labs open in Self Driving Car Studio/Cityscape or Cityscape lite
environment before running this example.

This script spawns ONLY ONE QCar at the center (0,0) and sets up the environment.
"""

import sys
import time
import os
import yaml
import numpy as np
import threading
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
from qvl.qcar_flooring import QLabsQCarFlooring
from qvl.walls import QLabsWalls


class QCarInitializer:
    """
    QCar Initializer Class - Handles environment and vehicle initialization
    Similar to QcarFleet.py but focused on initialization and manual control
    """

    def __init__(self, qlab_type="Studio"):
        """
        Initialize the QCar environment

        :param qlab_type: Type of QLabs environment ("Studio" or other)
        """
        self.qlab_type = qlab_type
        self.qlabs = QuanserInteractiveLabs()
        self.QCars = []
        self.environment_objects = {
            "crosswalks": [],
            "traffic_lights": [],
            "yield_signs": [],
            "roundabout_signs": [],
            "basic_shapes": [],
            "stop_signs": [],
        }
        self.multiagent_spawns = None

        # Traffic light control
        self.traffic_lights = []
        self.traffic_light_thread = None
        self.stop_traffic_lights = threading.Event()

        # Initialize environment based on type
        self.InitEnv()

    def InitEnv(self):
        """
        Initialize the QLabs environment
        For Studio mode, automatically spawns environment objects
        """
        print("Connecting to QLabs...")
        try:
            self.qlabs.open("localhost")
            self.qlabs.destroy_all_spawned_actors()
            QLabsRealTime().terminate_all_real_time_models()
            print("Connected to QLabs")
        except Exception as e:
            print(f"Unable to connect to QLabs: {e}")
            quit()

        print("Connected")

        QLabsRealTime().terminate_all_real_time_models()
        time.sleep(1)
        self.qlabs.destroy_all_spawned_actors()

        # Create and possess camera
        camera = QLabsFreeCamera(self.qlabs)
        camera.spawn_degrees(
            location=[-0.812, -1.651, 1.233], rotation=[0, 37.328, 75.423]
        )
        camera.possess()

        # # For Studio mode, automatically spawn environment like QcarFleet.py
        # if self.qlab_type == "Studio":
        #     print("\n=== Spawning Studio Environment ===")
        #     self._spawn_studio_environment()
        #     print("Studio environment spawned successfully!\n")

        # Keep the QLabs connection open for traffic light control
        print("QLabs connection kept open for traffic light control\n")

    def _spawn_studio_environment(self):
        """
        Spawn the Studio environment automatically (like Setup_Real_Scenario.py)
        This includes floor, walls, crosswalks, traffic signs, and guidance lines
        """
        # Setup environment with offsets (matching Setup_Real_Scenario.py)
        x_offset = 0.13
        y_offset = 1.67

        # Spawn floor
        hFloor = QLabsQCarFlooring(self.qlabs)
        hFloor.spawn_degrees(
            [x_offset, y_offset, 0.001], rotation=[0, 0, -90], configuration=0
        )
        print("  ✓ Spawned floor")

        # Spawn walls
        hWall = QLabsWalls(self.qlabs)
        hWall.set_enable_dynamics(False)

        for y in range(5):
            hWall.spawn_degrees(
                location=[-2.4 + x_offset, (-y * 1.0) + 2.55 + y_offset, 0.001],
                rotation=[0, 0, 0],
            )
        for x in range(5):
            hWall.spawn_degrees(
                location=[-1.9 + x + x_offset, 3.05 + y_offset, 0.001],
                rotation=[0, 0, 90],
            )
        for y in range(6):
            hWall.spawn_degrees(
                location=[2.4 + x_offset, (-y * 1.0) + 2.55 + y_offset, 0.001],
                rotation=[0, 0, 0],
            )
        for x in range(4):
            hWall.spawn_degrees(
                location=[-0.9 + x + x_offset, -3.05 + y_offset, 0.001],
                rotation=[0, 0, 90],
            )
        hWall.spawn_degrees(
            location=[-2.03 + x_offset, -2.275 + y_offset, 0.001], rotation=[0, 0, 48]
        )
        hWall.spawn_degrees(
            location=[-1.575 + x_offset, -2.7 + y_offset, 0.001], rotation=[0, 0, 48]
        )
        print("  ✓ Spawned walls")

        # Spawn stop signs (4 total)
        myStopSign = QLabsStopSign(self.qlabs)

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
        myRoundaboutSign = QLabsRoundaboutSign(self.qlabs)

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
        myYieldSign = QLabsYieldSign(self.qlabs)

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

        # Spawn crosswalks (6 total)
        myCrossWalk = QLabsCrosswalk(self.qlabs)

        myCrossWalk.spawn_degrees(
            location=[-2 + x_offset, -1.475 + y_offset, 0.01],
            rotation=[0, 0, 0],
            scale=[0.1, 0.1, 0.075],
            configuration=0,
        )

        myCrossWalk.spawn_degrees(
            location=[-0.5, 0.95, 0.006],
            rotation=[0, 0, 90],
            scale=[0.1, 0.1, 0.075],
            configuration=0,
        )

        myCrossWalk.spawn_degrees(
            location=[0.15, 0.32, 0.006],
            rotation=[0, 0, 0],
            scale=[0.1, 0.1, 0.075],
            configuration=0,
        )

        myCrossWalk.spawn_degrees(
            location=[0.75, 0.95, 0.006],
            rotation=[0, 0, 90],
            scale=[0.1, 0.1, 0.075],
            configuration=0,
        )

        myCrossWalk.spawn_degrees(
            location=[0.13, 1.57, 0.006],
            rotation=[0, 0, 0],
            scale=[0.1, 0.1, 0.075],
            configuration=0,
        )

        myCrossWalk.spawn_degrees(
            location=[1.45, 0.95, 0.006],
            rotation=[0, 0, 90],
            scale=[0.1, 0.1, 0.075],
            configuration=0,
        )
        print("  ✓ Spawned 6 crosswalks")

        # Spawn traffic lights (4 total - for intersection)
        trafficLight1 = QLabsTrafficLight(self.qlabs)
        trafficLight2 = QLabsTrafficLight(self.qlabs)
        trafficLight3 = QLabsTrafficLight(self.qlabs)
        trafficLight4 = QLabsTrafficLight(self.qlabs)

        # Intersection 1
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

        # Store traffic light references for control
        self.traffic_lights = [
            trafficLight1,
            trafficLight2,
            trafficLight3,
            trafficLight4,
        ]
        print("  ✓ Spawned 4 traffic lights")

        # Spawn signage line guidance (white lines - 3 basic shapes)
        mySpline = QLabsBasicShape(self.qlabs)

        mySpline.spawn_degrees(
            location=[2.21, 0.2, 0.006],
            rotation=[0, 0, 0],
            scale=[0.27, 0.02, 0.001],
            waitForConfirmation=False,
        )

        mySpline.spawn_degrees(
            location=[1.951, 1.68, 0.006],
            rotation=[0, 0, 0],
            scale=[0.27, 0.02, 0.001],
            waitForConfirmation=False,
        )

        mySpline.spawn_degrees(
            location=[-0.05, -1.02, 0.006],
            rotation=[0, 0, 90],
            scale=[0.38, 0.02, 0.001],
            waitForConfirmation=False,
        )
        print("  ✓ Spawned 3 guidance lines")

        time.sleep(1)  # Allow time for environment setup

    def _traffic_light_control_loop(self):
        """
        Background thread to control traffic light sequence
        Matches the behavior from Setup_Real_Scenario.py
        """
        if len(self.traffic_lights) != 4:
            print("Warning: Traffic lights not properly initialized")
            return

        trafficLight1, trafficLight2, trafficLight3, trafficLight4 = self.traffic_lights
        intersection1Flag = 0

        print("Starting Traffic Light Sequence")

        while not self.stop_traffic_lights.is_set():
            try:
                # Intersection 1 traffic light sequence
                if intersection1Flag == 0:
                    trafficLight1.set_color(color=QLabsTrafficLight.COLOR_RED)
                    trafficLight3.set_color(color=QLabsTrafficLight.COLOR_RED)
                    trafficLight2.set_color(color=QLabsTrafficLight.COLOR_GREEN)
                    trafficLight4.set_color(color=QLabsTrafficLight.COLOR_GREEN)

                if intersection1Flag == 1:
                    trafficLight1.set_color(color=QLabsTrafficLight.COLOR_RED)
                    trafficLight3.set_color(color=QLabsTrafficLight.COLOR_RED)
                    trafficLight2.set_color(color=QLabsTrafficLight.COLOR_YELLOW)
                    trafficLight4.set_color(color=QLabsTrafficLight.COLOR_YELLOW)

                if intersection1Flag == 2:
                    trafficLight1.set_color(color=QLabsTrafficLight.COLOR_GREEN)
                    trafficLight3.set_color(color=QLabsTrafficLight.COLOR_GREEN)
                    trafficLight2.set_color(color=QLabsTrafficLight.COLOR_RED)
                    trafficLight4.set_color(color=QLabsTrafficLight.COLOR_RED)

                if intersection1Flag == 3:
                    trafficLight1.set_color(color=QLabsTrafficLight.COLOR_YELLOW)
                    trafficLight3.set_color(color=QLabsTrafficLight.COLOR_YELLOW)
                    trafficLight2.set_color(color=QLabsTrafficLight.COLOR_RED)
                    trafficLight4.set_color(color=QLabsTrafficLight.COLOR_RED)

                intersection1Flag = (intersection1Flag + 1) % 4

                # Wait 5 seconds before next state change
                time.sleep(5)

            except Exception as e:
                print(f"Error in traffic light control: {e}")
                break

        print("Traffic light sequence stopped")

    def start_traffic_light_control(self):
        """
        Start the traffic light control in a background thread
        """
        if self.traffic_lights and not self.traffic_light_thread:
            self.stop_traffic_lights.clear()
            self.traffic_light_thread = threading.Thread(
                target=self._traffic_light_control_loop, daemon=True
            )
            self.traffic_light_thread.start()
            print("Traffic light control started")

    def stop_traffic_light_control(self):
        """
        Stop the traffic light control thread
        """
        if self.traffic_light_thread:
            self.stop_traffic_lights.set()
            self.traffic_light_thread.join(timeout=2.0)
            self.traffic_light_thread = None
            print("Traffic light control stopped")

    def cleanup(self):
        """
        Cleanup resources - stop traffic lights and close QLabs connection
        """
        self.stop_traffic_light_control()
        try:
            if self.qlabs:
                self.qlabs.close()
                print("QLabs connection closed")
        except Exception as e:
            print(f"Error closing QLabs: {e}")


# Initialize the environment using the new class structure
# This will automatically spawn the Studio environment
print("\n=== Initializing QCar Environment (Single Car) ===")
initializer = QCarInitializer(qlab_type="Studio")

# Start traffic light control in background
initializer.start_traffic_light_control()

# Define SINGLE CAR at CENTER
QCars = [
    {
        "RobotType": "QCar2",
        "Location": [0.0, 0.0, 0.005],  # CENTER
        "Rotation": [0, 0, 0],
        "Radians": True,
        "Scale": 0.1,
        "config": {"car_id": "0", "vehicle_type": "QCar2"},
    }
]

print(f"Spawning 1 QCar at {QCars[0]['Location']}...")
mySpawns = MultiAgent(QCars)

# Set LED color (Green or Default Red)
if len(mySpawns.robotActors) > 0:
    mySpawns.robotActors[0].set_led_strip_uniform(color=[0, 40, 0])  # slightly green

# Environment objects global for compatibility (not strictly needed but kept)
environment_objects = initializer.environment_objects


def get_transform_input():
    print(f"\n--- Setting Position for Single Car ---")
    default_loc = QCars[0]["Location"]
    default_rot = QCars[0]["Rotation"]

    prompt = (
        f"Enter X,Y or X,Y,THETA\n"
        f"  - D = default ({default_loc[0]}, {default_loc[1]}, {default_rot[2]})\n"
        f"  - X = quit (no shutdown)\n"
        f"  - Q = quit and shutdown QLabs real-time models\n"
        f"> "
    )
    s = input(prompt).strip()

    if not s:
        return default_loc, default_rot

    cmd = s.upper()
    if cmd == "X":
        return None, None
    if cmd == "Q":
        print("[shutdown] Terminating QLabs real-time models and exiting...")
        QLabsRealTime().terminate_all_real_time_models()
        return None, None
    if cmd == "D":
        return default_loc, default_rot

    # Parse numeric input: X,Y or X,Y,THETA
    parts = [p.strip() for p in s.split(",") if p.strip() != ""]
    if len(parts) not in (2, 3):
        print("Invalid format. Using default.")
        return default_loc, default_rot

    try:
        x = float(parts[0])
        y = float(parts[1])
        theta = float(parts[2]) if len(parts) == 3 else default_rot[2]
    except ValueError:
        print("Invalid numbers. Using default.")
        return default_loc, default_rot

    location = [x, y, 0.005]
    rotation = [0.0, 0.0, theta]
    return location, rotation


# Continuous loop
print("\n=== Single Vehicle Transform Control ===")
while True:
    try:
        loc, rot = get_transform_input()
        if loc is None:  # quit
            print("\nProgram terminated by user.")
            break

        if len(mySpawns.robotActors) > 0:
            mySpawns.robotActors[0].set_transform_and_request_state(
                location=loc,
                rotation=rot,
                enableDynamics=True,
                headlights=False,
                leftTurnSignal=False,
                rightTurnSignal=False,
                brakeSignal=False,
                reverseSignal=False,
            )
            print(f"✓ Car moved to {loc}")

    except KeyboardInterrupt:
        print("\nInterrupted.")
        break
    except Exception as e:
        print(f"Error: {e}")
        break

initializer.cleanup()
