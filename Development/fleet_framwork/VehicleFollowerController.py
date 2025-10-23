import math
import time
import logging
import numpy as np
from typing import Optional, Tuple

from src.Controller.CACC import CACC
from src.Controller.idm_control import IDMControl
from src.Controller.DummyController import DummyController, DummyVehicle
from pal.utilities.math import wrap_to_pi

# Import trajectory following components
try:
    from src.OpenRoad import OpenRoad
    from hal.products.mats import SDCSRoadMap
    from src.Controller.ControllerLeader import SpeedController, SteeringController
    TRAJECTORY_AVAILABLE = True
except ImportError:
    TRAJECTORY_AVAILABLE = False
    OpenRoad = None
    SDCSRoadMap = None
    SpeedController = None
    SteeringController = None


class VehicleFollowerController:
    """
    Dedicated follower controller class that handles all follower-specific control logic.
    This separates the control logic from the Vehicle class, making it more modular.

    Provides TWO independent control methods:
    
    1. Vehicle-Following Control (compute_vehicle_following_control):
       - "CACC"  : uses CACC.compute_cacc_acceleration
       - "IDM"   : uses legacy IDM (through DummyController wrapper)
       - "LOOKAHEAD": translated MATLAB Extended Look-Ahead Controller (acc + steering)
    
    2. Trajectory-Following Control (compute_trajectory_following_control):
       - Follows predefined waypoints independently (like leader vehicle)
       - Requires init_trajectory_controller() to be called first
    
    Users can use either method independently or combine both for hybrid control.
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
        self.prev_yaw_rate_lead = 0.0  # previous yaw rate for filtering

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

        # Trajectory following components (for TRAJECTORY controller type)
        self.waypointSequence = None
        self.InitialPose = None
        self.speedController = None
        self.steeringController = None
        self.start_time = None
        self.K_p = self.config.get('K_p', 0.1)  # Speed controller proportional gain
        self.K_i = self.config.get('K_i', 0.08)  # Speed controller integral gain
        self.K_stanley = self.config.get('K_stanley', 0.8)  # Stanley steering gain
        self.startDelay = self.config.get('startDelay', 1)  # Start delay in seconds

        self.logger.info(f"Follower controller initialized for vehicle {vehicle_id} with {controller_type}")
        self._init_controller()
        
        # Optionally initialize trajectory controller (can be used alongside vehicle-following)
        # Users can call init_trajectory_controller() explicitly if needed
        self.init_trajectory_controller()

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

    # ═══════════════════════════════════════════════════════════
    # Trajectory following components (for TRAJECTORY controller type)
    # ═══════════════════════════════════════════════════════════
    def init_trajectory_controller(self, road_type: str = None, node_sequence: list = None):
        """
        Initialize trajectory following controller (can be called independently).
        This allows using trajectory following alongside or instead of vehicle-following.
        
        Args:
            road_type: Optional road type override ('OpenRoad' or 'Studio')
            node_sequence: Optional node sequence override (e.g., [0, 1, 2, 3])
        """
        if not TRAJECTORY_AVAILABLE:
            self.logger.error("Trajectory following components not available")
            return False
        
        # Get configuration parameters (use overrides if provided)
        road_type = road_type or self.config.get('road_type', 'OpenRoad')
        node_sequence = node_sequence or self.config.get('node_sequence', [0, 1])
        
        self.logger.info(f"Initializing trajectory controller with road_type={road_type}, node_sequence={node_sequence}")
        
        try:
            # Generate path
            self.waypointSequence, self.InitialPose = self._generate_trajectory_path(road_type, node_sequence)
            
            # Initialize speed controller
            self.speedController = SpeedController(
                kp=self.K_p,
                ki=self.K_i
            )
            
            # Initialize steering controller
            if self.enable_steering_control:
                self.steeringController = SteeringController(
                    waypoints=self.waypointSequence,
                    k=self.K_stanley
                )
            
            self.logger.info(f"Trajectory controller initialized with {len(self.waypointSequence)} waypoints")
            return True
            
        except Exception as e:
            self.logger.error(f"Error initializing trajectory controller: {e}")
            return False

    def _generate_trajectory_path(self, road_type: str, node_sequence: list):
        """Generate trajectory path for the vehicle."""
        if road_type == "OpenRoad":
            roadmap = OpenRoad()
        elif road_type == "Studio":
            roadmap = SDCSRoadMap()
        else:
            raise ValueError(f"Unknown road type: {road_type}")
        
        waypointSequence = roadmap.generate_path(node_sequence)
        InitialPose = roadmap.get_node_pose(node_sequence[0]).squeeze()
        
        self.logger.info(f"Generated trajectory with {waypointSequence.shape[1] if len(waypointSequence.shape) > 1 else len(waypointSequence)} waypoints")
        
        return waypointSequence, InitialPose

    def _get_trajectory_vref(self, t: float) -> float:
        """Get reference velocity for trajectory following based on time."""
        road_type = self.config.get('road_type', 'OpenRoad')
        
        if road_type == "OpenRoad":
            if t < 5:
                v_ref = 2
            elif t < 10:
                v_ref = 0
            elif t < 15:
                v_ref = -0.5
            elif t < 20:
                v_ref = 1
            else:
                v_ref = 2
            return v_ref
        elif road_type == "Studio":
            return 0.3
        else:
            return 0.3

   

    def compute_trajectory_following_control(self, current_pos: list, current_rot: list, 
                                            current_velocity: float, dt: float = 0.1) -> Tuple[float, float]:
        """
        Compute control commands for trajectory following (independent path following).
        This method focuses on following a predefined trajectory.
        
        Args:
            current_pos: Current position [x, y, z]
            current_rot: Current rotation [roll, pitch, yaw]
            current_velocity: Current velocity
            dt: Time step
            
        Returns:
            Tuple of (forward_speed_command, steering_angle)
        """
        if not TRAJECTORY_AVAILABLE:
            self.logger.error("Trajectory following components not available")
            return 0.0, 0.0

        if self.waypointSequence is None or self.speedController is None:
            self.logger.error("Trajectory controller not initialized. Call _init_trajectory_controller() first.")
            return 0.0, 0.0

        try:
            # Initialize start time if needed
            if self.start_time is None:
                self.start_time = time.time()
                self.logger.info("Trajectory following started")
            
            # Calculate elapsed time
            t = time.time() - self.start_time
            
            # Get reference velocity
            vref = self._get_trajectory_vref(t)
            
            # Extract position and orientation
            x, y = current_pos[0], current_pos[1]
            th = current_rot[2]  # Heading in radians
            
            # Position for steering controller (with look-ahead point)
            p = np.array([x, y]) + np.array([np.cos(th), np.sin(th)]) * 0.2
            
            # Use provided velocity
            v = current_velocity
            
            # Control logic
            if t < self.startDelay:
                u = 0
                delta = 0
            else:
                # Speed control
                u = self.speedController.update(v, vref, dt)
                
                # Steering control
                if self.enable_steering_control and self.steeringController:
                    delta = self.steeringController.update(p, th, v)
                else:
                    delta = 0
            
            # Convert and clamp commands
            forward_speed = u
            steering_angle = delta
            
            # Clamp to safe ranges
            forward_speed = max(-2.0, min(2.0, forward_speed))
            steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))
            
            self.logger.debug(f"Trajectory-following control - Speed: {forward_speed:.3f}, Steering: {steering_angle:.3f}")
            return forward_speed, steering_angle

        except Exception as e:
            self.logger.error(f"Error computing trajectory-following control: {e}")
            return 0.0, 0.0

    # ═══════════════════════════════════════════════════════════
    # Vehicle following components (for TRAJECTORY controller type)
    # ═══════════════════════════════════════════════════════════

    def compute_vehicle_following_control(self, current_pos: list, current_rot: list, current_velocity: float,
                                          leader_pos: list, leader_rot: list, leader_velocity: float,
                                          leader_timestamp: float = None, dt: float = 0.1) -> Tuple[float, float]:
        """
        Compute control commands for vehicle-following (CACC, IDM, LOOKAHEAD).
        This method focuses on following another vehicle.
        
        Args:
            current_pos: Current position [x, y, z] of the follower
            current_rot: Current rotation [roll, pitch, yaw] of the follower
            current_velocity: Current velocity of the follower
            leader_pos: Leader position [x, y, z]
            leader_rot: Leader rotation [roll, pitch, yaw]
            leader_velocity: Leader velocity
            leader_timestamp: Timestamp of leader data (optional)
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
                speed_cmd, steering_cmd = self._compute_lookahead_control(follower_state, leader_state, dt)
                if not self.enable_steering_control:
                    steering_cmd = 0.0  # No steering for straight roads
            else:
                speed_cmd = self._compute_longitudinal_control(follower_state, leader_state)

                # Only compute steering if enabled
                if self.enable_steering_control:
                    # steering_cmd = self._compute_lateral_control(follower_state, leader_state)
                    steering_cmd = self._compute_lateral_control_simple(follower_state, leader_state)
                else:
                    steering_cmd = 0.0  # No steering for straight roads

            # Clip commands based on road type
            if self.road_type == "OpenRoad":
                # OpenRoad: higher speed limits, but steering depends on enable_steering_control
                speed_cmd = max(-2.0, min(2.0, speed_cmd))
                if not self.enable_steering_control:
                    steering_cmd = 0.0  # Force zero only if explicitly disabled
            elif self.road_type == "Studio":
                # Studio: lower speed limits for indoor environment
                speed_cmd = max(-0.01, min(0.3, speed_cmd))
            else:
                # Default: conservative limits
                speed_cmd = max(-0.01, min(0.3, speed_cmd))

            # Always apply steering limits (even if zero)
            steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))
            
            self.logger.debug(f"Vehicle-following control - Acc: {speed_cmd:.3f}, Steering: {steering_cmd:.3f}")
            return speed_cmd, steering_cmd

        except Exception as e:
            self.logger.error(f"Error computing vehicle-following control: {e}")
            return 0.0, 0.0
    
    def _compute_longitudinal_control(self, follower_state: list, leader_state: list) -> float:
        """
        Compute the longitudinal control (acceleration command).
        For 'CACC' uses controller.compute_cacc_acceleration.
        For 'LOOKAHEAD' uses the translated MATLAB look-ahead acceleration law.
        For 'IDM' or others uses the legacy interface.
        """
        try:
            self.logger.debug(f"--- Vehicle {self.vehicle_id}: Computing longitudinal control ---")
            self.logger.debug(f"Vehicle {self.vehicle_id}: Follower state: {follower_state}")
            self.logger.debug(f"Vehicle {self.vehicle_id}: Leader state: {leader_state}")
            self.logger.debug(f"Vehicle {self.vehicle_id}: Controller type: {self.controller_type}")

            if self.controller_type == "CACC":
                # existing optimized path
                acc_cmd = self.controller.compute_cacc_acceleration(follower_state, leader_state)
                self.logger.debug(f"Vehicle {self.vehicle_id}: CACC acceleration command: {acc_cmd:.6f}")
                
            else:
                # For IDM or other controllers, use the legacy interface with DummyVehicle
                acc_cmd = self._compute_legacy_control(follower_state, leader_state)
                self.logger.debug(f"Vehicle {self.vehicle_id}: Legacy acceleration command: {acc_cmd:.6f}")

            # Ensure acceleration command is non-negative
            # acc_cmd = max(-0.05, acc_cmd)
            self.logger.debug(f"Vehicle {self.vehicle_id}: Final acceleration command: {acc_cmd:.6f}")

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
            self.logger.debug(f"--- Vehicle {self.vehicle_id}: Computing lateral control --- ")
            # self.logger.debug(f"Vehicle {self.vehicle_id}: Follower state: {follower_state}")
            # self.logger.debug(f"Vehicle {self.vehicle_id}: Leader state: {leader_state}")

            # default: pure pursuit (existing behavior)
            current_pos = follower_state[:2]
            current_rot_z = follower_state[2]
            leader_pos = leader_state[:2]
            leader_rot_z = leader_state[2]

            self.logger.debug(f"Vehicle {self.vehicle_id}: Current pos: {current_pos}, rot_z: {current_rot_z}")
            self.logger.debug(f"Vehicle {self.vehicle_id}: Leader pos: {leader_pos}, rot_z: {leader_rot_z}")
            self.logger.debug(f"Vehicle {self.vehicle_id}: Lookahead distance: {self.lookahead_distance}")

            # Compute distance to leader for adaptive lookahead
            dx_to_leader = leader_pos[0] - current_pos[0]
            dy_to_leader = leader_pos[1] - current_pos[1]
            distance_to_leader = math.sqrt(dx_to_leader**2 + dy_to_leader**2)
            
            # Adaptive lookahead: use smaller value between configured lookahead and half the distance
            # This prevents overshooting when close to the leader
            adaptive_lookahead = min(self.lookahead_distance, max(0.2, distance_to_leader * 0.5))
            
            self.logger.debug(f"Vehicle {self.vehicle_id}: Distance to leader: {distance_to_leader:.3f}, Adaptive lookahead: {adaptive_lookahead:.3f}")

            # Determine motion direction based on leader velocity
            # leader_vel = leader_state[3]
            # motion_direction = 1 if leader_vel >= 0 else -1
            motion_direction = 1 
            # Target point ahead of leader in its heading direction
            target_x = leader_pos[0] + motion_direction * adaptive_lookahead * math.cos(leader_rot_z)
            target_y = leader_pos[1] + motion_direction * adaptive_lookahead * math.sin(leader_rot_z)

            self.logger.debug(f"Vehicle {self.vehicle_id}: Target point: ({target_x:.3f}, {target_y:.3f})")

            # Compute heading error
            dx = target_x - current_pos[0]
            dy = target_y - current_pos[1]
            target_angle = math.atan2(dy, dx)
            heading_error = wrap_to_pi(target_angle - current_rot_z)

            self.logger.debug(f"Vehicle {self.vehicle_id}: dx={dx:.3f}, dy={dy:.3f}, target_angle={target_angle:.3f}, heading_error={heading_error:.3f}")

            # Apply proportional control with sign correction
            steering_cmd = self.k_steering * heading_error
            self.logger.debug(f"Vehicle {self.vehicle_id}: Steering gain k_steering={self.k_steering}, steering_cmd={steering_cmd:.3f}")
            
            return steering_cmd

        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Error in lateral control: {e}")
            return 0.0
        

    def _compute_lateral_control_simple(self, follower_state: list, leader_state: list) -> float:
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

            steering_cmd = self.k_steering * heading_error
            # steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))
            return steering_cmd

        except Exception as e:
            self.logger.error(f"Error in lateral control: {e}")
            return 0.0

    # ---------- Translated MATLAB helper methods ----------
    def _compute_lookahead_control(self, follower_state: list, leader_state: list, dt: float = 0.01) -> Tuple[float, float]:
        """
        Improved lookahead controller for vehicle following.
        follower_state = [x, y, theta, v]
        leader_state   = [x_lead, y_lead, theta_lead, v_lead]

        Returns:
            (acceleration_command, steering_angle_delta)
        """
        # Unpack
        x, y, theta, v = follower_state
        x_lead, y_lead, theta_lead, v_lead = leader_state

        # If no leader provided, return zeros
        if x_lead is None:
            return 0.0, 0.0

        # Compute yaw rate from heading change with filtering to reduce noise
        if abs(theta_lead) > 30:
            theta_lead = math.radians(theta_lead)
        if abs(theta) > 30:
            theta = math.radians(theta)
            
        yaw_rate_lead = (theta_lead - self.prev_theta_lead) / max(dt, 0.001)
        
        # Simple low-pass filter on yaw rate to reduce noise
        alpha_filter = 0.5
        yaw_rate_lead = alpha_filter * yaw_rate_lead + (1 - alpha_filter) * self.prev_yaw_rate_lead
        self.prev_yaw_rate_lead = yaw_rate_lead
        self.prev_theta_lead = theta_lead

        # Compute curvature (kappa = yaw_rate / velocity)
        kappa_lead = yaw_rate_lead / max(abs(v_lead), 0.01)

        # Compute desired spacing distance
        d = self.ri + self.hi * v  # inter-vehicle distance

        # Compute s_bar (extension along curved path)
        s_bar = self._compute_s_bar(kappa_lead, self.ri, self.hi, v)

        # Extension vector: point ahead on leader's curved path
        # Use leader's heading to project the virtual point
        sx_lead = s_bar * math.cos(theta_lead)
        sy_lead = s_bar * math.sin(theta_lead)

        # Virtual target point
        x_target = x_lead + sx_lead
        y_target = y_lead + sy_lead

        # Current follower position with desired spacing offset
        x_follower_desired = x + d * math.cos(theta)
        y_follower_desired = y + d * math.sin(theta)

        # Position errors
        z1 = x_target - x_follower_desired
        z2 = y_target - y_follower_desired

        # Velocity errors (projected along heading directions)
        z3 = v_lead * math.cos(theta_lead) - v * math.cos(theta)
        z4 = v_lead * math.sin(theta_lead) - v * math.sin(theta)

        # Control laws
        acc = self.k1 * z1 + z3
        omega = self.k2 * z2 + z4  # yaw rate command

        # Convert yaw rate to steering angle using bicycle model
        # delta = atan(omega * L / v) where L is wheelbase
        L = self.l_r + self.l_f
        delta = math.atan2(omega * L, max(abs(v), 0.1))

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
        """Get current control state information for both controllers."""
        state = {
            'vehicle_id': self.vehicle_id,
            'controller_type': self.controller_type,
            'initialized': self.initialized,
            'lookahead_distance': self.lookahead_distance,
            'max_steering': self.max_steering,
            'k_steering': self.k_steering
        }
        
        # Add trajectory controller state (independent of controller_type)
        if self.waypointSequence is not None:
            elapsed_time = time.time() - self.start_time if self.start_time else 0
            state.update({
                'trajectory_controller_initialized': True,
                'trajectory_active': self.start_time is not None,
                'elapsed_time': elapsed_time,
                'waypoint_count': len(self.waypointSequence),
                'reference_velocity': self._get_trajectory_vref(elapsed_time) if self.start_time else 0
            })
        else:
            state['trajectory_controller_initialized'] = False
        
        return state

    def stop_control(self):
        """Stop both vehicle-following and trajectory-following controllers."""
        self.logger.info(f"Stopping follower controller for vehicle {self.vehicle_id}")
        self.initialized = False

    def stop_trajectory_following(self):
        """Stop trajectory following and reset timing."""
        if self.start_time is not None:
            self.start_time = None
            self.logger.info(f"Trajectory following stopped for vehicle {self.vehicle_id}")

    def reset_trajectory_following(self):
        """Reset trajectory following timing (restart from beginning)."""
        self.start_time = None
        self.logger.info(f"Trajectory following reset for vehicle {self.vehicle_id}")
