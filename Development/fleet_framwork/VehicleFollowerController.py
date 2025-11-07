import math
import time
import logging
import traceback
import numpy as np
import yaml
import os
from typing import Optional, Tuple

from src.Controller.CACC import CACC
from src.Controller.idm_control import IDMControl
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

        # ===== LOAD CONTROLLER CONFIG FROM YAML FILE =====
        # This is the main config file that the controller reads directly
        # No need to pass complex config through QcarFleet or VehicleProcess!
        self.controller_config = self._load_controller_config()
        
        # Get follower mode (with per-vehicle override support)
        self.follower_mode = self._get_follower_mode()
        self.logger.info(f"Vehicle {vehicle_id}: Using follower mode '{self.follower_mode}'")

        # ===== LOAD MODE-SPECIFIC CONFIGURATION =====

        self.max_throttle_cmd = self.controller_config.get('max_throttle_cmd', 0.7)
        self.max_steering = self.controller_config.get('max_steering', 0.55)

        # Load main config path settings (for trajectory) - MUST BE BEFORE mode-specific config
        self.road_type = self.config.get('road_type', None)

        # Each mode has its own config section in controller_config.yaml
        if self.follower_mode == "vehicle_following":
            self._load_vehicle_following_config()
        elif self.follower_mode == "trajectory":
            self._load_trajectory_config()
        elif self.follower_mode == "hybrid":
            self._load_hybrid_config()
        else:
            self.logger.error(f"Unknown follower mode: {self.follower_mode}. Defaulting to vehicle_following")
            self.follower_mode = "vehicle_following"
            self._load_vehicle_following_config()
        
        self.leader_state = None
        self.prev_pos = None
        self.prev_time = None
        self.velocity = 0.0
        self.initialized = False

        self.logger.info(f"Follower controller initialized for vehicle {vehicle_id} with mode '{self.follower_mode}'")
        self._init_controller()
        
        # Initialize trajectory controller if needed
        if self.follower_mode in ["trajectory", "hybrid"]:
            self.init_trajectory_controller(road_type=self.road_type)

    def _load_controller_config(self) -> dict:
        """Load controller configuration from controller_config.yaml file."""
        config_file = os.path.join(os.path.dirname(__file__), 'controller_config.yaml')
        
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f) or {}
            # self.logger.info(f"Loaded controller config from {config_file}")
            print(f"Vehicle {self.vehicle_id}: Successfully loaded controller_config.yaml from {config_file}")
            return config
        except Exception as e:
            self.logger.error(f"Error loading controller config: {e}")
            print(f"Vehicle {self.vehicle_id}: ERROR - Failed to load controller config: {e}")
            return {}

    def _get_follower_mode(self) -> str:
        """Get follower mode with per-vehicle override support."""
        # Check per-vehicle overrides first
        overrides = self.controller_config.get('vehicle_mode_overrides', {}) or {}
        if self.vehicle_id in overrides:
            mode = overrides[self.vehicle_id]
            self.logger.info(f"Vehicle {self.vehicle_id}: Using mode override '{mode}'")
            print(f"Vehicle {self.vehicle_id}: Using follower mode override: '{mode}'")
            return mode
        
        # Use default mode from config
        mode = self.controller_config.get('follower_mode', 'vehicle_following')
        print(f"Vehicle {self.vehicle_id}: Using default follower mode: '{mode}'")
        return mode

    def _load_vehicle_following_config(self):
        """Load vehicle-following mode configuration."""
        vf_config = self.controller_config.get('vehicle_following', {})
        
        # Control / vehicle parameters
        self.alpha = vf_config.get('alpha', 1.2)
        self.beta = vf_config.get('beta', 1.5)
        self.v0 = vf_config.get('v0', 0.4)
        self.delta = vf_config.get('delta', 3)
        self.T = vf_config.get('T', 0.3)
        self.s0 = vf_config.get('s0', 1)
        self.ri = vf_config.get('ri', 1)
        
        # self.s0 = self.config.get('s0', self.s0)
        # self.ri = self.s0

        self.hi = vf_config.get('hi', 0.3)
        self.K_gains = vf_config.get('K_gains')
        
        # Physical parameters
        self.l_r = vf_config.get('l_r', 0.141)
        self.l_f = vf_config.get('l_f', 0.115)
        self.C1 = vf_config.get('C1', 1.0)
        self.C2 = vf_config.get('C2', 1.0)
        self.mass = vf_config.get('mass', 5.0)
        
        # Controller gains
        self.k1 = vf_config.get('k1', 1.0)
        self.k2 = vf_config.get('k2', 1.0)

        self.lookahead_distance = self.config.get('lookahead_distance', 7) ## TODO : Fix here
        print(f"Vehicle {self.vehicle_id}: lookahead_distance set to {self.lookahead_distance}")
        self.startDelay = vf_config.get('startDelay', 1)  
        self.start_time = None
  
        
        # Steering
        self.enable_steering_control = vf_config.get('enable_steering_control', True) 
        # self.enable_steering_control = vf_config.get('enable_steering_control', True) 

        if self.road_type == "OpenRoad":
            self.k_steering = vf_config.get('k_steering_openroad', 0.2)
        else:
            self.k_steering = vf_config.get('k_steering', 0.5)

        
        self.prev_theta_lead = -0.7177
        self.prev_yaw_rate_lead = 0.0
        
        # Actuator delay parameters
        self.tau = self.controller_config.get('control', {}).get('tau', 0.1)  # Actuator time constant
        self.delayed_speed_cmd = 0.0  # Previous delayed speed command
        
        self.logger.info(f"Vehicle {self.vehicle_id}: Loaded vehicle-following config")

    def _load_trajectory_config(self):
        """Load trajectory-following mode configuration."""
        traj_config = self.controller_config.get('trajectory_following', {})
        
        # Speed controller
        self.K_p = traj_config.get('K_p', 0.1)
        self.K_i = traj_config.get('K_i', 0.08)
        
        # Steering controller
        self.K_stanley = traj_config.get('K_stanley', 0.8)
        self.enable_steering_control = traj_config.get('enable_steering_control', False)
        self.lookahead_distance = self.config.get('lookahead_distance', 0.5)  ## TODO : Fix here 
        self.startDelay = traj_config.get('startDelay', 1)
        

        
        # Trajectory components (will be initialized later)
        self.waypointSequence = None
        self.InitialPose = None
        self.speedController = None
        self.steeringController = None
        self.start_time = None
        
        # Actuator delay parameters
        self.tau = self.controller_config.get('control', {}).get('tau', 0.1)  # Actuator time constant
        self.delayed_speed_cmd = 0.0  # Previous delayed speed command
        
        self.logger.info(f"Vehicle {self.vehicle_id}: Loaded trajectory-following config")

    def _load_hybrid_config(self):
        """Load hybrid mode configuration (combines vehicle-following and trajectory)."""
        # Load both vehicle-following and trajectory configs
        self._load_vehicle_following_config()
        self._load_trajectory_config()
        
        # Load hybrid-specific settings
        hybrid_config = self.controller_config.get('hybrid', {})
        self.hybrid_priority = hybrid_config.get('priority', 'vehicle_following')
        self.hybrid_distance_threshold = hybrid_config.get('distance_threshold', 2.0)
        self.hybrid_leader_timeout = hybrid_config.get('leader_data_timeout', 1.0)
        self.hybrid_hysteresis_offset = hybrid_config.get('hysteresis_offset', 0.5)
        
        self.hybrid_current_mode = 'trajectory'  # Start with trajectory
        self.last_leader_data_time = 0
        
        # Actuator delay parameters (already loaded from individual configs above)
        # self.tau and self.delayed_speed_cmd are already set by _load_vehicle_following_config()
        
        self.logger.info(f"Vehicle {self.vehicle_id}: Loaded hybrid config with priority '{self.hybrid_priority}'")

    def _init_controller(self):
        """Initialize the appropriate controller for this vehicle."""
        try:
            if self.controller_type == "CACC":
                # Use parameters already loaded from config
                # K_gains should be 1x2 for CACC (spacing_gain, velocity_gain)
                K_gains = np.array(self.K_gains).reshape(1, 2) if len(self.K_gains) == 2 else np.array([[0.2, 0.05]])
                self.controller = CACC(s0=self.ri, h=self.hi, K=K_gains, logger=self.logger)
            elif self.controller_type == "IDM":
                self.controller = IDMControl(alpha=self.alpha, beta=self.beta, v0=self.v0, 
                                           delta=self.delta, T=self.T, s0=self.s0, logger=self.logger)
            elif self.controller_type == "LOOKAHEAD":
                self.controller = None  # LOOKAHEAD doesn't need a controller object
            else:
                self.controller = None

            self.initialized = True
            self.reset_actuator_delay()  # Initialize actuator delay state
            print(f"Vehicle {self.vehicle_id}: ✅ Controller {self.controller_type} initialized")

        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: ❌ Error initializing controller: {e}")
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
        # road_type = road_type or self.config.get('road_type', 'OpenRoad')
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
                v_ref = self.max_throttle_cmd
            elif t < 10:
                v_ref = 0
            elif t < 15:
                v_ref = -0.5
            elif t < 20:
                v_ref = 0.5
            else:
                v_ref = self.max_throttle_cmd
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
            forward_speed = max(-self.max_throttle_cmd, min(self.max_throttle_cmd, forward_speed))
            steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))
            
            # Apply actuator delay to speed command
            delayed_forward_speed = self._apply_actuator_delay(forward_speed, dt)
            
            # self.logger.debug(f"Trajectory-following control - Speed: {forward_speed:.3f}, Delayed Speed: {delayed_forward_speed:.3f}, Steering: {steering_angle:.3f}")
            return delayed_forward_speed, steering_angle

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

            # Initialize start time if needed
            if self.start_time is None:
                self.start_time = time.time()
                self.logger.info("Following Vehicle started")
            
            # Calculate elapsed time
            t = time.time() - self.start_time

            # Control logic
            if t < self.startDelay:
                # u = 0
                # delta = 0
                return 0.0, 0.0

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
            speed_cmd = max(-self.max_throttle_cmd, min(self.max_throttle_cmd, speed_cmd))
            if not self.enable_steering_control:
                steering_cmd = 0.0  # Force zero only if explicitly disabled
            
            # Always apply steering limits (even if zero)
            steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))
            
            # Apply actuator delay to speed command
            delayed_speed_cmd = self._apply_actuator_delay(speed_cmd, dt)
            
            self.logger.debug(f"Vehicle-following control - Raw Acc: {speed_cmd:.3f}, Delayed Acc: {delayed_speed_cmd:.3f}, Steering: {steering_cmd:.3f}")
            return delayed_speed_cmd, steering_cmd

        except Exception as e:
            self.logger.error(f"Error computing vehicle-following control: {e}")
            return 0.0, 0.0
    
    def _compute_longitudinal_control(self, follower_state: list, leader_state: list) -> float:
        """
        Compute the longitudinal control (acceleration command).
        For 'CACC' uses controller.compute_cacc_acceleration.
        For 'IDM' uses controller.compute_idm_acceleration.
        For 'LOOKAHEAD' uses the translated MATLAB look-ahead acceleration law.
        """
        try:
            self.logger.debug(f"--- Vehicle {self.vehicle_id}: Computing longitudinal control ---")
            self.logger.debug(f"Vehicle {self.vehicle_id}: Follower state: {follower_state}")
            self.logger.debug(f"Vehicle {self.vehicle_id}: Leader state: {leader_state}")

            if self.controller_type == "CACC":
                # Use CACC controller's direct acceleration method
                acc_cmd = self.controller.compute_cacc_acceleration(follower_state, leader_state)
                
            elif self.controller_type == "IDM":
                # Use IDM controller's direct acceleration method
                acc_cmd = self.controller.compute_idm_acceleration(follower_state, leader_state)
                
            elif self.controller_type == "LOOKAHEAD":
                # Use the look-ahead control algorithm (no separate controller object)
                acc_cmd = self._compute_lookahead_acceleration(follower_state, leader_state)
                
            else:
                self.logger.warning(f"Unknown controller type '{self.controller_type}', using zero acceleration")
                acc_cmd = 0.0

            # self.logger.debug(f"Vehicle {self.vehicle_id}: Final acceleration command: {acc_cmd:.6f}")
            return acc_cmd

        except Exception as e:
            self.logger.error(f"Error in longitudinal control: {e}")
            return 0.0

    def _compute_lookahead_acceleration(self, follower_state: list, leader_state: list) -> float:
        """Extract acceleration command from lookahead control."""
        try:
            acc_cmd, _ = self._compute_lookahead_control(follower_state, leader_state)
            return acc_cmd
        except Exception as e:
            self.logger.error(f"Error in lookahead acceleration computation: {e}")
            return 0.0

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
        # Note: theta values are now in radians from CSV config files
            
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

    def _apply_actuator_delay(self, speed_cmd: float, dt: float) -> float:
        """
        Apply first-order actuator delay to speed command.
        Uses the formula: delayed_cmd = (1 - dt/tau) * prev_delayed_cmd + (dt/tau) * cmd
        
        Args:
            speed_cmd: Raw speed command from controller
            dt: Time step
            
        Returns:
            Delayed speed command
        """
        if self.tau <= 0:
            # No delay if tau is zero or negative
            return speed_cmd
            
        # First-order low-pass filter representing actuator dynamics
        alpha = dt / (self.tau + dt)  # Filter coefficient
        self.delayed_speed_cmd = (1 - alpha) * self.delayed_speed_cmd + alpha * speed_cmd
        
        return self.delayed_speed_cmd

    def reset_actuator_delay(self):
        """Reset actuator delay state (useful for restarting or switching modes)."""
        self.delayed_speed_cmd = 0.0
        self.logger.debug(f"Vehicle {self.vehicle_id}: Actuator delay state reset")



    def compute_control(self, current_pos: list, current_rot: list, current_velocity: float,
                       leader_pos: list = None, leader_rot: list = None, leader_velocity: float = None,
                       leader_timestamp: float = None, dt: float = 0.1) -> Tuple[float, float]:
        """
        Universal control method that handles all follower modes automatically:
        - vehicle_following: Follows a target vehicle (requires leader data)
        - trajectory: Follows predefined waypoints independently 
        - hybrid: Combines both modes based on conditions
        
        Args:
            current_pos: Current position [x, y, z]
            current_rot: Current rotation [roll, pitch, yaw] 
            current_velocity: Current velocity
            leader_pos: Leader position [x, y, z] (optional, for vehicle_following)
            leader_rot: Leader rotation [roll, pitch, yaw] (optional, for vehicle_following)
            leader_velocity: Leader velocity (optional, for vehicle_following)
            leader_timestamp: Timestamp of leader data (optional)
            dt: Time step
            
        Returns:
            Tuple of (forward_speed_command, steering_angle)
        """
        if not self.initialized:
            return 0.0, 0.0
            
        try:
            if self.follower_mode == "vehicle_following":
                if leader_pos is None or leader_rot is None or leader_velocity is None:
                    self.logger.warning(f"Vehicle {self.vehicle_id}: No leader data for vehicle_following mode")
                    return 0.0, 0.0
                return self.compute_vehicle_following_control(
                    current_pos, current_rot, current_velocity,
                    leader_pos, leader_rot, leader_velocity, leader_timestamp, dt
                )
                
            elif self.follower_mode == "trajectory":
                return self.compute_trajectory_following_control(
                    current_pos, current_rot, current_velocity, dt
                )
                
            elif self.follower_mode == "hybrid":
                # Check if leader data is available and recent
                leader_available = (leader_pos is not None and leader_rot is not None and 
                                  leader_velocity is not None)
                
                if leader_available:
                    # Calculate distance to leader
                    distance = np.sqrt((leader_pos[0] - current_pos[0])**2 + 
                                     (leader_pos[1] - current_pos[1])**2)
                    
                    # Use vehicle following if leader is close enough or priority is vehicle_following
                    if (self.hybrid_priority == "vehicle_following" or 
                        distance < self.hybrid_distance_threshold):
                        return self.compute_vehicle_following_control(
                            current_pos, current_rot, current_velocity,
                            leader_pos, leader_rot, leader_velocity, leader_timestamp, dt
                        )
                
                # Fall back to trajectory following
                return self.compute_trajectory_following_control(
                    current_pos, current_rot, current_velocity, dt
                )
                
            else:
                self.logger.error(f"Vehicle {self.vehicle_id}: Unknown follower_mode '{self.follower_mode}'")
                return 0.0, 0.0
                
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Error in compute_control: {e}")
            return 0.0, 0.0

    def get_control_state(self) -> dict:
        """Get current control state information for both controllers."""
        state = {
            'vehicle_id': self.vehicle_id,
            'controller_type': self.controller_type,
            'follower_mode': self.follower_mode,
            'initialized': self.initialized,
            'lookahead_distance': self.lookahead_distance,
            'max_steering': self.max_steering,
            'k_steering': self.k_steering,
            'actuator_tau': self.tau,
            'delayed_speed_cmd': self.delayed_speed_cmd
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
        self.reset_actuator_delay()  # Reset actuator delay state when stopping
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
        # update actuator delay parameter if provided
        if 'tau' in kwargs:
            self.tau = kwargs['tau']
            self.logger.info(f"Updated actuator time constant (tau) to {self.tau}")
        
        # update lookahead-specific params if provided
        for name in ('l_r', 'l_f', 'C1', 'C2', 'mass', 'hi', 'ri', 'k1', 'k2'):
            if name in kwargs:
                setattr(self, name, kwargs[name])
                self.logger.info(f"Updated {name} to {getattr(self, name)}")
