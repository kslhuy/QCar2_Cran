import math
import numpy as np
import threading
import time
from Controller.idm_control import IDMControl
from pal.utilities.math import wrap_to_pi

class DummyVehicle:
     def __init__(self, state, vehicle_id=0):
            self.state = state
            self.vehicle_id = vehicle_id

class Follower:
    def __init__(self, qcar, idm_controller, vehicle_id=1, max_steering=0.6, lookahead_distance=0.2):
        self.qcar = qcar                          # Leader car ID
        self.idm = idm_controller                 # The IDM-based controller
        self.id = vehicle_id                      # Follower car ID
        self.max_steering = max_steering          # Maximum steering angle limit
        self.lookahead_distance = lookahead_distance  # Lookahead distance in meters

        self.running = False
        self.thread = None
    
    def run(self, leader):
        self.running = True
        while self.running:
            self.follow(leader)
            time.sleep(0.1)  # Small delay for loop timing

    def stop(self):
        """ Stop the follower loop. """
        self.running = False
        if self.thread is not None:
            self.thread.join()

    def start(self, leader):
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self.run, args=(leader,))
            self.thread.start()

    def follow(self, leader):
        _, pos_leader, rot_leader, _ = leader.get_world_transform()
        _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()

        target_x = pos_leader[0] - self.lookahead_distance * math.cos(rot_leader[2])
        target_y = pos_leader[1] - self.lookahead_distance * math.sin(rot_leader[2])

        dx = target_x - pos_follower[0]
        dy = target_y - pos_follower[1]

        follower_heading = rot_follower[2]
        target_angle = math.atan2(dy, dx)
        heading_error = wrap_to_pi(target_angle - follower_heading)

        # Simple proportional controller for steering
        steering_cmd = -5.0 * heading_error
        steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))

        # Get follower velocity (or estimate it)
        v_follower = getattr(self.qcar, 'motorTach', 0.5)
        follower_state = [pos_follower[0], pos_follower[1], follower_heading, v_follower]

        # Create a dummy leader vehicle for the IDM controller
        leader_state = [pos_leader[0], pos_leader[1], rot_leader[2], v_follower]
        dummy_leader = DummyVehicle(leader_state, vehicle_id = self.id-1)

        # Override IDM's vehicle perception with dummy data
        self.idm.controller.get_surrounding_vehicles = lambda *args, **kwargs: (None, [dummy_leader], None, None)

        # Use IDM to get desired acceleration input
        _, input_u, _ = self.idm.get_optimal_input(
            host_car_id=self.id,
            state=follower_state,
            last_input=None,
            lane_id=None,
            input_log=None,
            initial_lane_id=None,
            direction_flag=None,
            type_state="true",
            acc_flag=0
        )

        speed_cmd = max(0, input_u[0])  # Only allow positive speed (no braking)

        # Send control command to the QCar
        self.qcar.set_velocity_and_request_state(
            forward         =speed_cmd,
            turn            =steering_cmd,
            headlights      =False,
            leftTurnSignal  =False,
            rightTurnSignal =False,
            brakeSignal     =False,
            reverseSignal=False
        )