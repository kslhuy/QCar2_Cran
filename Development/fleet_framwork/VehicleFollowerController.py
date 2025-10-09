import math
import time
import logging
from typing import Optional, Tuple

from src.Controller.CACC import CACC
from src.Controller.idm_control import IDMControl
from src.Controller.DummyController import DummyController, DummyVehicle
from pal.utilities.math import wrap_to_pi


class VehicleFollowerController:
    """
    Dedicated follower controller class that handles all follower-specific control logic.
    This separates the control logic from the Vehicle class, making it more modular.

    Supports controller_type:
      - "CACC"  : uses CACC.compute_cacc_acceleration
      - "IDM"   : uses legacy IDM (through DummyController wrapper)
      - "LOOKAHEAD": translated MATLAB Extended Look-Ahead Controller (acc + steering)
    """

    def __init__(self, vehicle_id: int, controller_type: str = "CACC", config=None, logger=None):
        self.vehicle_id = vehicle_id
        self.controller_type = controller_type
        self.config = config or {}
        self.logger = logger or logging.getLogger(f"FollowerController_{vehicle_id}")

        # Control / vehicle parameters (defaults; override via config)
        self.road_type = self.config.get('road_type', None)
        self.enable_steering_control = self.config.get('enable_steering_control', True)
        self.lookahead_distance = self.config.get('lookahead_distance', 0.4)
        self.max_steering = self.config.get('max_steering', 0.55) # in radians (~30 degrees)
        self.k_steering = self.config.get('k_steering', 2.0)

        # TODO : Need to be more dynamic in future
        self.prev_theta_lead = -0.7177  # initial previous leader heading (for curvature calc) 

        # Parameters used by the translated MATLAB controller
        # param_sys
        self.l_r = self.config.get('l_r', 0.141)      # rear axle dist
        self.l_f = self.config.get('l_f', 0.115)      # front axle dist
        self.C1 = self.config.get('C1', 1.0)        # tire stiffness front (unused in current delta formula)
        self.C2 = self.config.get('C2', 1.0)        # tire stiffness rear (unused in current delta formula)
        self.mass = self.config.get('mass', 5.0) # vehicle mass kg ??? 

        # param_opt
        self.hi = self.config.get('hi', 0.3)  # time-gap policy
        self.ri = self.config.get('ri', 1 )  # standstill distance

        # controller gains (MATLAB used k1=k2=3.5)
        self.k1 = self.config.get('k1', 1)
        self.k2 = self.config.get('k2', 1)

        self.leader_state = None
        self.prev_pos = None
        self.prev_time = None
        self.velocity = 0.0
        self.initialized = False

        self.logger.info(f"Follower controller initialized for vehicle {vehicle_id} with {controller_type}")
        self._init_controller()

    def _init_controller(self):
        """Initialize the appropriate longitudinal controller for this vehicle."""
        try:
            dummy_controller_params = self.config.get('dummy_controller_params', {})
            if str(self.vehicle_id) in dummy_controller_params:
                dummy_params = dummy_controller_params[str(self.vehicle_id)]
            elif isinstance(dummy_controller_params, dict) and 'alpha' in dummy_controller_params:
                dummy_params = dummy_controller_params
            else:
                dummy_params = None

            dummy_controller = DummyController(self.vehicle_id, dummy_params)

            if self.controller_type == "CACC":
                self.controller = CACC(dummy_controller)
            elif self.controller_type == "IDM":
                self.controller = IDMControl(dummy_controller)
            elif self.controller_type == "LOOKAHEAD":
                # For LOOKAHEAD we still create a dummy controller for compatibility with _compute_legacy_control
                # but the main logic will use the translated look-ahead algorithm below.
                self.controller = dummy_controller
            else:
                # Fallback: keep the dummy controller so legacy path works
                self.controller = dummy_controller

            self.initialized = True
            print(f"Vehicle {self.vehicle_id}: Controller {self.controller_type} initialized successfully")

        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Error initializing controller: {e}")
            self.initialized = False

    def compute_control(self, current_pos: list, current_rot: list, current_velocity: float,
                        leader_pos: list, leader_rot: list, leader_velocity: float,
                        leader_timestamp: float, dt: float) -> Tuple[float, float]:
        """
        Compute control commands for the follower vehicle.
        
        Args:
            current_pos: Current position [x, y, z] of the follower
            current_rot: Current rotation [roll, pitch, yaw] of the follower
            current_velocity: Current velocity of the follower
            leader_pos: Leader position [x, y, z]
            leader_rot: Leader rotation [roll, pitch, yaw]
            leader_velocity: Leader velocity
            leader_timestamp: Timestamp of leader data
            dt: Time step
            
        Returns:
            Tuple of (forward_acceleration_command, steering_angle)
        """
        if not self.initialized:
            print(f"Vehicle {self.vehicle_id}: Controller not initialized")
            return 0.0, 0.0

        try:
            follower_state = [current_pos[0], current_pos[1], current_rot[2], current_velocity]
            leader_state = [leader_pos[0], leader_pos[1], leader_rot[2], leader_velocity]

            if self.controller_type == "LOOKAHEAD":
                speed_cmd, steering_cmd = self._compute_lookahead_control(follower_state, leader_state , dt)
                # no negative speed prevention here — keep acceleration as-is (clipping optional)
                if not self.enable_steering_control:
                    steering_cmd = 0.0  # No steering for straight roads
            else:
                speed_cmd = self._compute_longitudinal_control(follower_state, leader_state)

                # Only compute steering if enabled
                if self.enable_steering_control:
                    steering_cmd = self._compute_lateral_control(follower_state, leader_state)
                else:
                    steering_cmd = 0.0  # No steering for straight roads

            # Clip commands based on road type
            if self.road_type == "OpenRoad":
                steering_cmd = 0.0  # override to zero for straight roads
                speed_cmd = max(-2.0, min(2.0, speed_cmd))

            elif self.road_type == "Studio":
                speed_cmd = max(0, min(1, speed_cmd))  # clip acceleration to reasonable range
                # speed_cmd = max(-0.05, speed_cmd)
            else:
                speed_cmd = max(0, min(1, speed_cmd))  # clip acceleration to reasonable range

            steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))
            self.logger.debug(f"Control commands - Acc: {speed_cmd:.3f}, Steering: {steering_cmd:.3f}")
            return speed_cmd, steering_cmd

        except Exception as e:
            self.logger.error(f"Error computing control: {e}")
            return 0.0, 0.0

    def _compute_longitudinal_control(self, follower_state: list, leader_state: list) -> float:
        """
        Compute the longitudinal control (acceleration command).
        For 'CACC' uses controller.compute_cacc_acceleration.
        For 'LOOKAHEAD' uses the translated MATLAB look-ahead acceleration law.
        For 'IDM' or others uses the legacy interface.
        """
        try:
            if self.controller_type == "CACC":
                # existing optimized path
                acc_cmd = self.controller.compute_cacc_acceleration(follower_state, leader_state)
                # acc_cmd = max(acc_cmd, -5.0)  # optional safety clipping (braking)
                # return acc_cmd

            # elif self.controller_type == "LOOKAHEAD":
            #     acc_cmd, _ = self._compute_lookahead_control(follower_state, leader_state)
            #     # no negative speed prevention here — keep acceleration as-is (clipping optional)
            #     # return acc_cmd

            else:
                # For IDM or other controllers, use the legacy interface with DummyVehicle
                acc_cmd = self._compute_legacy_control(follower_state, leader_state)

            # Ensure acceleration command is non-negative
            # acc_cmd = max(-0.05, acc_cmd)

            return acc_cmd

        except Exception as e:
            self.logger.error(f"Error in longitudinal control: {e}")
            return 0.0

    def _compute_legacy_control(self, follower_state: list, leader_state: list) -> float:
        """
        Legacy control computation for non-CACC controllers.
        This maintains the old behavior for IDM and other controllers.
        """
        dummy_leader = DummyVehicle(leader_state, vehicle_id=0)

        if not hasattr(self, '_original_get_surrounding_vehicles'):
            # save original if exists
            self._original_get_surrounding_vehicles = getattr(self.controller, 'get_surrounding_vehicles', None)

        # temporarily provide get_surrounding_vehicles to the underlying controller if required
        def fake_get_surrounding_vehicles(*args, **kwargs):
            # (car_fc, others...) - but previous code expected (None, [dummy_leader], None, None)
            return (None, [dummy_leader], None, None)

        # attach if controller exposes attribute
        if hasattr(self.controller, 'get_surrounding_vehicles'):
            self.controller.get_surrounding_vehicles = fake_get_surrounding_vehicles

        try:
            # call the old-style get_optimal_input signature
            # emulate previous usage: return _, input_u, _
            _, input_u, _ = self.controller.get_optimal_input(
                host_car_id=self.vehicle_id,
                state=follower_state,
                last_input=None,
                lane_id=None,
                input_log=None,
                initial_lane_id=None,
                direction_flag=None,
                type_state="true",
                acc_flag=0
            )
            speed_cmd = input_u[0]
            return speed_cmd

        finally:
            # restore original method if we overrode it
            if hasattr(self.controller, 'get_surrounding_vehicles') and hasattr(self, '_original_get_surrounding_vehicles'):
                self.controller.get_surrounding_vehicles = self._original_get_surrounding_vehicles

    def _compute_lateral_control(self, follower_state: list, leader_state: list) -> float:
        """
        Compute lateral control. If using LOOKAHEAD, use delta from lookahead controller.
        Otherwise use previous pure-pursuit approach.
        """
        try:
            # if self.controller_type == "LOOKAHEAD":
            #     # Use lookahead to compute acc and delta
            #     _, delta = self._compute_lookahead_control(follower_state, leader_state)
            #     # clip by steering limits
            #     delta = max(-self.max_steering, min(self.max_steering, delta))
            #     return delta

            # default: pure pursuit (existing behavior)
            current_pos = follower_state[:2]
            current_rot_z = follower_state[2]
            leader_pos = leader_state[:2]
            leader_rot_z = leader_state[2]

            target_x = leader_pos[0] - self.lookahead_distance * math.cos(leader_rot_z)
            target_y = leader_pos[1] - self.lookahead_distance * math.sin(leader_rot_z)

            dx = target_x - current_pos[0]
            dy = target_y - current_pos[1]
            target_angle = math.atan2(dy, dx)
            heading_error = wrap_to_pi(target_angle - current_rot_z)

            steering_cmd = -self.k_steering * heading_error
            # steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))
            return steering_cmd

        except Exception as e:
            self.logger.error(f"Error in lateral control: {e}")
            return 0.0

    # ---------- Translated MATLAB helper methods ----------
    def _compute_lookahead_control(self, follower_state: list, leader_state: list, dt: float = 0.01) -> Tuple[float, float]:
        """
        Translate MATLAB look_ahead_Control.get_optimal_input into Python.
        follower_state = [x, y, theta, v]
        leader_state   = [x_lead, y_lead, theta_lead, v_lead]

        Returns:
            (acceleration_command, steering_angle_delta)
        """
        # Unpack
        x, y, theta, v = follower_state
        x_lead, y_lead, theta_lead, v_lead = leader_state

        print(f"Follower state: x={x}, y={y}, theta={theta}, v={v}")
        print(f"Leader state: x_lead={x_lead}, y_lead={y_lead}, theta_lead={theta_lead}, v_lead={v_lead}")
        # If no leader provided, return zeros
        if x_lead is None:
            return 0.0, 0.0

        # compute curvature its should be yaw rate (rad/s) not heading theta_lead
        if abs(theta_lead) > 30:
            theta_lead = math.radians(theta_lead)
        yaw_rate_lead = (theta_lead - self.prev_theta_lead)/dt

        kappa_lead = self._compute_curvature(v_lead, yaw_rate_lead, dt)
        self.prev_theta_lead = theta_lead  # update for next call

        # compute s_bar (MATLAB formula)
        s_bar = self._compute_s_bar(kappa_lead, self.ri, self.hi, v)

        # # extension vector in leader frame (MATLAB used sx = s_bar*sin(theta_lead), sy = -s_bar*cos(theta_lead))
        # sx_lead = s_bar * math.sin(theta_lead)
        # sy_lead = -s_bar * math.cos(theta_lead)


        # # Compute tracking errors z1..z4
        # # z1 = x_lead + sx_lead - x - (ri + hi * v) * cos(theta);
        # # z2 = y_lead + sy_lead - y - (ri + hi * v) * sin(theta);
        # # z3 = v_lead * cos(theta_lead) - v * cos(theta);
        # # z4 = v_lead * sin(theta_lead) - v * sin(theta);
        # z1 = x_lead + sx_lead - x - (self.ri + self.hi * v) * math.cos(theta)
        # z2 = y_lead + sy_lead - y - (self.ri + self.hi * v) * math.sin(theta)
        # z3 = v_lead * math.cos(theta_lead) - v * math.cos(theta)
        # z4 = v_lead * math.sin(theta_lead) - v * math.sin(theta)


        # Revised extension vector in leader frame (MATLAB used sx = s_bar*cos(theta_lead), sy = s_bar*sin(theta_lead))
        # Variant A (cos, sin)
        # print(f"theta_lead: {theta_lead}")
        # print(f"theta: {theta}")

        # sx_lead = s_bar * math.cos(theta_lead)
        # sy_lead = s_bar * math.sin(theta_lead)
        sx_lead = s_bar * math.sin(theta_lead)
        sy_lead = -s_bar * math.cos(theta_lead)

        print(f"yaw_rate_lead :{yaw_rate_lead} , kappa_lead: {kappa_lead} ,  s_bar: {s_bar}, sx_lead: {sx_lead}, sy_lead: {sy_lead}")

        d = self.ri + self.hi * v  # inter-vehicle distance scalar
        # alpha = math.atan(kappa_lead * d)
        alpha = 0
        z1 = (x_lead + sx_lead) - (x + (d * math.cos(theta)))
        z2 = (y_lead + sy_lead) - (y + (d * math.sin(theta)))

        z3 = v_lead * math.cos(theta_lead) - v * math.cos(theta + alpha)
        z4 = v_lead * math.sin(theta_lead) - v * math.sin(theta + alpha)
        print(f"z1: {z1}, z2: {z2}, z3: {z3}, z4: {z4}, alpha: {alpha}")

        # if abs(alpha) < math.pi / 2 - 1e-6:  # Avoid div by zero
        #     acc = self.k1 * z1 + (z3 / math.cos(alpha))
        #     omega = - (self.k2 * z2 + (z4 / math.cos(alpha)))
        # else:
        #     # Handle large alpha (rare, but fallback to 0 or log error)
        #     acc, omega = 0, 0

        # # control laws
        acc = self.k1 * z1 + z3
        omega = -(self.k2 * z2 + z4) # omega is yaw-rate-like
        # convert omega (yaw-rate-like) to steering angle delta (bicycle model approximation)
        delta = (math.atan(omega * self.l_r / max(v, 0.01)))
        print(f"acc (pre-clip): {acc}, delta (pre-clip): {delta}")
        print("------")


        # print(f"delta: {delta}")
        # clip commands
        # acc = max(0, min(1, acc))  # clip acceleration to reasonable range
        # delta = max(-self.max_steering, min(self.max_steering, delta))


        return acc, delta

    def _compute_curvature(self, v: float, yaw_rate: float, dt: float) -> float:
        # yaw rate (rad/s), not heading
        # # so here instead use v , need to use delta_S = v * dt
        # delta_S = v * dt
        return yaw_rate / max(v, 0.01)  # avoid div by zero

    def _compute_s_bar(self, kappa: float, ri: float, hi: float, v: float) -> float:
        # MATLAB:
        # if kappa == 0: s_bar = 0
        # else s_bar = (-1 + sqrt(1 + kappa^2 * (ri + hi * v)^2)) / kappa
        if abs(kappa) < 1e-9:
            return 0.0
        tmp = 1.0 + (kappa ** 2) * (ri + hi * v) ** 2
        return (-1.0 + math.sqrt(max(0.0, tmp))) / kappa

    # ---------- Utilities ----------
    def update_parameters(self, **kwargs):
        if 'lookahead_distance' in kwargs:
            self.lookahead_distance = kwargs['lookahead_distance']
            self.logger.info(f"Updated lookahead distance to {self.lookahead_distance}")
        if 'max_steering' in kwargs:
            self.max_steering = kwargs['max_steering']
            self.logger.info(f"Updated max steering to {self.max_steering}")
        if 'k_steering' in kwargs:
            self.k_steering = kwargs['k_steering']
            self.logger.info(f"Updated steering gain to {self.k_steering}")
        # update lookahead-specific params if provided
        for name in ('l_r', 'l_f', 'C1', 'C2', 'mass', 'hi', 'ri', 'k1', 'k2'):
            if name in kwargs:
                setattr(self, name, kwargs[name])
                self.logger.info(f"Updated {name} to {getattr(self, name)}")

    def get_control_state(self) -> dict:
        return {
            'vehicle_id': self.vehicle_id,
            'controller_type': self.controller_type,
            'initialized': self.initialized,
            'lookahead_distance': self.lookahead_distance,
            'max_steering': self.max_steering,
            'k_steering': self.k_steering
        }

    def stop_control(self):
        self.logger.info(f"Stopping follower controller for vehicle {self.vehicle_id}")
        self.initialized = False
