"""
Platoon Controller - Manages vehicle platoon formation and coordination
"""
import time
import numpy as np
from typing import Optional, Dict, List, Tuple, Any
from dataclasses import dataclass


@dataclass
class PlatoonConfig:
    """Configuration for platoon behavior"""
    # Formation parameters
    formation_speed: float = 0.3  # Speed during formation (m/s)
    active_speed: float = 0.75    # Speed when platoon is active (m/s)
    
    # Spacing parameters
    target_spacing: float = 1.5   # Target distance to vehicle ahead (m)
    spacing_tolerance: float = 0.3  # Acceptable spacing error (m)
    min_safe_spacing: float = 0.8  # Minimum safe distance (m)
    max_spacing: float = 3.0      # Maximum spacing before lost (m)
    
    # Formation timeouts
    search_timeout: float = 30.0  # Time to search for leader before giving up (s)
    forming_timeout: float = 20.0  # Time to form spacing before giving up (s)
    lost_timeout: float = 10.0    # Time without leader before giving up (s)
    
    # Velocity adjustment gains
    spacing_kp: float = 0.3       # Proportional gain for spacing control
    spacing_ki: float = 0.05      # Integral gain for spacing control
    velocity_filter: float = 0.2  # Low-pass filter for velocity adjustments


class PlatoonController:
    """Controller for platoon formation and maintenance"""
    
    def __init__(self, config: PlatoonConfig = None, logger=None):
        self.config = config or PlatoonConfig()
        self.logger = logger
        
        # Platoon state
        self.enabled = False
        self.is_leader = False
        self.platoon_id = None
        self.leader_car_id = None
        
        # Setup tracking - CRITICAL: tracks if SETUP_PLATOON_FORMATION has been received
        self.setup_complete = False  # Set to True after SETUP_PLATOON_FORMATION command
        self.formation_data = {}     # Store formation data for reference
        self.my_position = None      # This vehicle's position in the platoon (1=leader, 2+=follower)
        
        # Leader tracking
        self.leader_detected = False
        self.leader_distance = None
        self.leader_velocity = None
        self.last_leader_seen = None
        
        # Formation tracking
        self.formation_start_time = None
        self.spacing_stable_time = None
        self.spacing_stable_duration = 2.0  # Seconds of stable spacing to consider ready
        
        # Control state
        self.spacing_error_integral = 0.0
        self.last_spacing_error = 0.0
        self.filtered_velocity_adjustment = 0.0
        
        # Follower status
        self.followers_ready = set()  # Set of follower IDs that are ready
        self.expected_followers = []  # List of expected follower IDs
        
    def enable_platoon_mode(self, platoon_id: str, is_leader: bool, 
                           leader_id: Optional[int] = None,
                           follower_ids: Optional[List[int]] = None):
        """
        Enable platoon mode
        
        Args:
            platoon_id: Unique identifier for this platoon
            is_leader: Whether this vehicle is the platoon leader
            leader_id: Car ID of the leader (for followers)
            follower_ids: List of follower car IDs (for leader)
        """
        self.enabled = True
        self.is_leader = is_leader
        self.platoon_id = platoon_id
        self.leader_car_id = leader_id
        self.expected_followers = follower_ids or []
        self.followers_ready.clear()
        
        self.formation_start_time = time.time()
        self.spacing_error_integral = 0.0
        
        if self.logger:
            role = "LEADER" if is_leader else "FOLLOWER"
            self.logger.logger.info(
                f"Platoon mode enabled: ID={platoon_id}, Role={role}, "
                f"Leader={leader_id}, Followers={follower_ids}"
            )
    
    def disable_platoon_mode(self):
        """
        Disable platoon mode (pause platoon operation)
        
        NOTE: This does NOT clear formation_data, my_position, or is_leader flag.
        Those settings are preserved so platoon can be re-started without re-configuration.
        Only clears runtime state like leader_detected and followers_ready.
        """
        self.enabled = False
        # Keep is_leader, leader_car_id, my_position, formation_data, setup_complete intact!
        # Only clear runtime detection state
        self.leader_detected = False
        self.followers_ready.clear()
        
        if self.logger:
            role = "LEADER" if self.is_leader else f"FOLLOWER-{self.my_position}"
            self.logger.logger.info(f"Platoon mode paused (role preserved: {role})")
    
    def clear_platoon_configuration(self):
        """
        Completely clear all platoon configuration (formation, position, role)
        Use this when you want to fully reset platoon setup, not just pause it.
        """
        self.enabled = False
        self.is_leader = False
        self.platoon_id = None
        self.leader_car_id = None
        self.leader_detected = False
        self.followers_ready.clear()
        self.expected_followers = []
        
        # Clear configuration
        self.setup_complete = False
        self.formation_data = {}
        self.my_position = None
        
        if self.logger:
            self.logger.logger.info("Platoon configuration completely cleared")
    
    def enable_as_leader(self):
        """Simple enable as leader"""
        self.enabled = True
        self.is_leader = True
        self.formation_start_time = time.time()
        self.spacing_error_integral = 0.0
        
        if self.logger:
            self.logger.logger.info("Enabled as platoon LEADER")
    
    def enable_as_follower(self):
        """Simple enable as follower"""
        self.enabled = True
        self.is_leader = False
        self.formation_start_time = time.time()
        self.spacing_error_integral = 0.0
        
        if self.logger:
            self.logger.logger.info("Enabled as platoon FOLLOWER")
    
    def disable(self):
        """Simple disable"""
        self.disable_platoon_mode()
    
    def setup_from_global_formation(self, my_car_id: int, formation: Dict[int, int], leader_id: int):
        """
        Configure platoon from global formation data
        
        Args:
            my_car_id: This vehicle's ID
            formation: Dict mapping car_id -> position (e.g., {0: 1, 1: 2, 2: 3})
            leader_id: ID of the leader vehicle (position 1)
        """
        print(f"\n[DEBUG] ===== setup_from_global_formation CALLED =====\n")
        print(f"[DEBUG] setup_from_global_formation called: car_id={my_car_id}, formation={formation}, leader_id={leader_id}")
        
        # Handle both string and integer keys (JSON conversion can make keys strings)
        vehicle_found = False
        my_position = None
        
        if my_car_id in formation:
            my_position = formation[my_car_id]
            vehicle_found = True
        elif str(my_car_id) in formation:
            my_position = formation[str(my_car_id)]
            vehicle_found = True
        
        if not vehicle_found:
            if self.logger:
                self.logger.logger.warning(f"Car {my_car_id} not found in formation data")
            print(f"[DEBUG] Car {my_car_id} not in formation {formation}")
            return False
        
        is_leader = (my_position == 1)
        
        print(f"[DEBUG] Vehicle {my_car_id} position: {my_position}, is_leader: {is_leader}")
        
        # Configure as leader or follower
        if is_leader:
            self.enable_as_leader()
            # Store follower info - handle both string and int keys
            self.expected_followers = []
            for car_id, pos in formation.items():
                # Convert car_id to int if it's a string
                if isinstance(car_id, str):
                    car_id = int(car_id)
                if pos > 1:
                    self.expected_followers.append(car_id)
        else:
            self.enable_as_follower()
            self.leader_car_id = leader_id
        
        # Store formation data for reference
        self.formation_data = formation.copy()
        self.my_position = my_position
        
        # ✅ CRITICAL: Mark setup as complete after configuration
        self.setup_complete = True
        
        print(f"[DEBUG] Stored formation_data: {self.formation_data}, my_position: {self.my_position}")
        print(f"[DEBUG] setup_complete set to: {self.setup_complete}")
        
        if self.logger:
            role = "LEADER" if is_leader else f"FOLLOWER (position {my_position})"
            followers = [int(car_id) if isinstance(car_id, str) else car_id for car_id, pos in formation.items() if pos > 1] if is_leader else []
            self.logger.logger.info(
                f"Configured from global formation: Role={role}, "
                f"Leader={leader_id}, Formation={formation}, "
                f"Expected followers={followers}, setup_complete=True"
            )
        
        return True
    
    def get_formation_info(self) -> Dict:
        """Get current formation information"""
        return {
            'formation_data': getattr(self, 'formation_data', {}),
            'my_position': getattr(self, 'my_position', 0),
            'is_leader': self.is_leader,
            'leader_id': self.leader_car_id,
            'expected_followers': self.expected_followers
        }
    
    # ===== (NOT USE) PERCEPTION-BASED LEADER TRACKING (for followers) =====
    
    def update_leader_info(self, detected: bool, distance: Optional[float] = None,
                          velocity: Optional[float] = None):
        """
        Update information about the leader based on perception data (for followers)
        
        Args:
            detected: Whether leader is currently detected via YOLO/perception sensors
            distance: Distance to leader from YOLO/perception (meters)
            velocity: Leader velocity from network telemetry (m/s)
        """
        self.leader_detected = detected
        
        if detected:
            self.last_leader_seen = time.time()
            self.leader_distance = distance
            
        if velocity is not None:
            self.leader_velocity = velocity
    
    def is_spacing_stable(self) -> bool:
        """Check if spacing to leader is stable based on perception data (for followers)"""
        if not self.leader_detected or self.leader_distance is None:
            self.spacing_stable_time = None
            return False
        
        # Check if spacing is within tolerance
        spacing_error = abs(self.leader_distance - self.config.target_spacing)
        is_good = spacing_error < self.config.spacing_tolerance
        
        if is_good:
            if self.spacing_stable_time is None:
                self.spacing_stable_time = time.time()
            
            stable_duration = time.time() - self.spacing_stable_time
            return stable_duration >= self.spacing_stable_duration
        else:
            self.spacing_stable_time = None
            return False
    
    def has_lost_leader(self) -> bool:
        """Check if follower has lost perception of leader"""
        if not self.leader_detected or self.last_leader_seen is None:
            return True
        
        time_since_seen = time.time() - self.last_leader_seen
        return time_since_seen > self.config.lost_timeout
    
    def compute_follower_velocity(self, current_velocity: float, 
                                  base_velocity: float) -> float:
        """
        Compute velocity for follower to maintain spacing using perception data
        
        Args:
            current_velocity: Current vehicle velocity (m/s)
            base_velocity: Base/target velocity (m/s)
            
        Returns:
            Adjusted velocity to maintain spacing (m/s)
        """
        if not self.leader_detected or self.leader_distance is None:
            # No leader detected via perception sensors, maintain base velocity
            return base_velocity
        
        # Compute spacing error based on perceived distance
        spacing_error = self.leader_distance - self.config.target_spacing
        
        # Update integral term (with anti-windup)
        self.spacing_error_integral += spacing_error * 0.005  # Assuming ~200Hz
        self.spacing_error_integral = np.clip(
            self.spacing_error_integral,
            -0.5,  # Max integral contribution: 0.5 m/s
            0.5
        )
        
        # PI controller for velocity adjustment
        velocity_adjustment = (
            self.config.spacing_kp * spacing_error +
            self.config.spacing_ki * self.spacing_error_integral
        )
        
        # Low-pass filter for smooth adjustments
        alpha = self.config.velocity_filter
        self.filtered_velocity_adjustment = (
            alpha * velocity_adjustment +
            (1 - alpha) * self.filtered_velocity_adjustment
        )
        
        # Compute target velocity (combining perception and communication data)
        if self.leader_velocity is not None:
            # Use leader velocity from communication + spacing adjustment from perception
            target_velocity = self.leader_velocity + self.filtered_velocity_adjustment
        else:
            # Use base velocity + spacing adjustment from perception only
            target_velocity = base_velocity + self.filtered_velocity_adjustment
        
        # Safety limits
        if self.leader_distance < self.config.min_safe_spacing:
            # Too close! Slow down significantly
            target_velocity = min(target_velocity, current_velocity * 0.5)
        
        # Clip to reasonable range
        target_velocity = np.clip(target_velocity, 0.0, base_velocity * 1.5)
        
        # Debug logging (occasional) - Initialize counter if needed
        if self.logger:
            self._debug_counter = getattr(self, '_debug_counter', 0) + 1
            if self._debug_counter % 100 == 0:  # Log every 100 calls
                self.logger.logger.debug(
                    f"Follower control (perception-based): "
                    f"perceived_dist={self.leader_distance:.2f}m, "
                    f"spacing_error={spacing_error:.2f}m, "
                    f"velocity_adj={self.filtered_velocity_adjustment:.3f}m/s, "
                    f"target_velocity={target_velocity:.2f}m/s"
                )
        
        return target_velocity
    
    # ===== COMMUNICATION-BASED FOLLOWER STATUS (for leader) =====
    
    def update_follower_status(self, follower_id: int, is_ready: bool):
        """
        Update follower readiness status via communication (for leader)
        
        Args:
            follower_id: ID of the follower vehicle
            is_ready: Whether follower has achieved proper spacing
        """
        if is_ready:
            self.followers_ready.add(follower_id)
        else:
            self.followers_ready.discard(follower_id)
    
    def are_all_followers_ready(self) -> bool:
        """Check if all expected followers are ready via communication"""
        if not self.is_leader:
            return False
        
        return len(self.followers_ready) == len(self.expected_followers)
    
    # ===== COMMUNICATION-BASED CONTROL (for followers) =====
    
    # def compute_follower_velocity_v2v(self, current_velocity: float, base_velocity: float, 
    #                                   direct_leader_data: Optional[Dict[str, Any]] = None) -> float:
    #     """
    #     Compute velocity for follower using direct leader communication data (V2V)
        
    #     Args:
    #         current_velocity: Current vehicle velocity (m/s)
    #         base_velocity: Base/target velocity (m/s)
    #         direct_leader_data: Direct leader state from V2V communication
            
    #     Returns:
    #         Adjusted velocity for formation following (m/s)
    #     """
    #     if direct_leader_data is None:
    #         # No V2V data available, fallback to base velocity
    #         return base_velocity
        
    #     leader_velocity = direct_leader_data.get('velocity', 0.0)
        
    #     # Use leader velocity as primary reference
    #     # Follow leader speed with conservative approach for safety
    #     target_velocity = leader_velocity * 0.95  # 95% of leader speed for safety margin
        
    #     # Clip to reasonable range
    #     target_velocity = np.clip(target_velocity, 0.0, base_velocity)
        
    #     # Debug logging - periodic for monitoring (reduced frequency)
    #     if self.logger:
    #         self._debug_counter_v2v = getattr(self, '_debug_counter_v2v', 0) + 1
    #         if self._debug_counter_v2v % 500 == 0:  # Every 500 calls (~25 seconds at 20Hz)
    #             self.logger.logger.debug(
    #                 f"[V2V FOLLOW] Leader: {leader_velocity:.2f}m/s -> Target: {target_velocity:.2f}m/s"
    #             )
        
    #     return target_velocity
    
    # def compute_follower_steering_v2v(self, current_x: float, current_y: float, current_theta: float,
    #                                   current_velocity: float, direct_leader_data: Optional[Dict[str, Any]] = None,
    #                                   target_distance: float = 2.0) -> Optional[float]:
    #     """
    #     Compute steering for follower using direct leader communication data (V2V)
        
    #     Args:
    #         current_x, current_y, current_theta: Current vehicle pose
    #         current_velocity: Current vehicle velocity
    #         direct_leader_data: Direct leader state from V2V communication
    #         target_distance: Target following distance (m)
            
    #     Returns:
    #         Steering angle (radians) or None if no valid target
    #     """
    #     if direct_leader_data is None:
    #         return None
        
    #     leader_x = direct_leader_data.get('x', 0.0)
    #     leader_y = direct_leader_data.get('y', 0.0)
    #     leader_theta = direct_leader_data.get('theta', 0.0)
        
    #     # Calculate target point behind the leader
    #     target_x = leader_x - target_distance * np.cos(leader_theta)
    #     target_y = leader_y - target_distance * np.sin(leader_theta)
        
    #     # Pure pursuit steering calculation
    #     dx = target_x - current_x
    #     dy = target_y - current_y
    #     target_distance_actual = np.sqrt(dx**2 + dy**2)
        
    #     if target_distance_actual < 0.1:  # Too close to target
    #         return 0.0
        
    #     # Calculate heading error
    #     target_heading = np.arctan2(dy, dx)
    #     heading_error = target_heading - current_theta
        
    #     # Normalize heading error to [-pi, pi]
    #     heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
    #     # Simple proportional steering control
    #     steering_gain = 2.0  # Adjust based on vehicle dynamics
    #     lookahead_distance = max(1.0, current_velocity * 0.5)  # Dynamic lookahead
        
    #     steering_angle = np.arctan2(2.0 * np.sin(heading_error) * lookahead_distance, target_distance_actual)
        
    #     # Limit steering angle
    #     max_steering = np.pi / 6  # 30 degrees max
    #     steering_angle = np.clip(steering_angle, -max_steering, max_steering)
        
    #     if self.logger:
    #         self._debug_counter_steering_v2v = getattr(self, '_debug_counter_steering_v2v', 0) + 1
    #         if self._debug_counter_steering_v2v % 100 == 0:
    #             self.logger.logger.debug(
    #                 f"Follower steering (V2V-based): "
    #                 f"target=({target_x:.2f},{target_y:.2f}), "
    #                 f"heading_err={np.degrees(heading_error):.1f}°, "
    #                 f"steering={np.degrees(steering_angle):.1f}°"
    #             )
        
    #     return steering_angle
    
    # def get_direct_leader_data_from_v2v(self, v2v_manager, my_vehicle_id: int) -> Optional[Dict[str, Any]]:
    #     """
    #     Get direct leader's state data from V2V manager
        
    #     Args:
    #         v2v_manager: V2V manager instance
    #         my_vehicle_id: This vehicle's ID
            
    #     Returns:
    #         Direct leader's state data or None
    #     """
    #     try:
    #         # Use get_latest_local_state 
    #         if not v2v_manager or not hasattr(v2v_manager, 'get_latest_local_state'):
    #             if self.logger:
    #                 self.logger.logger.debug(f"V2V manager not available or missing get_latest_local_state")
    #             return None
            
    #         # Find direct leader (vehicle with position = my_position - 1)
    #         if hasattr(self, 'formation_data') and hasattr(self, 'my_position'):
    #             my_position = self.my_position
    #             if my_position > 1:  # Not the leader
    #                 direct_leader_position = my_position - 1
                    
    #                 # Find vehicle ID with this position - handle both string and int keys
    #                 for vehicle_id, position in self.formation_data.items():
    #                     # Convert vehicle_id to int if it's a string for querying V2V
    #                     if isinstance(vehicle_id, str):
    #                         vehicle_id_int = int(vehicle_id)
    #                     else:
    #                         vehicle_id_int = vehicle_id
                        
    #                     if position == direct_leader_position:
    #                         # Use get_latest_local_state
    #                         leader_data = v2v_manager.get_latest_local_state(vehicle_id_int)
    #                         return leader_data
            
    #         # Fallback: use leader_car_id if available
    #         if self.leader_car_id is not None:
    #             return v2v_manager.get_latest_local_state(self.leader_car_id)
            
    #         return None
            
    #     except Exception as e:
    #         if self.logger:
    #             self.logger.logger.warning(f"Error getting direct leader data from V2V: {e}")
    #         return None
    
    def update_leader_velocity_from_v2v(self, v2v_manager, my_vehicle_id: int):
        """
        Update leader velocity from V2V communication data
        
        Args:
            v2v_manager: V2V manager instance
            my_vehicle_id: This vehicle's ID
        """
        try:
            direct_leader_data = self.get_direct_leader_data_from_v2v(v2v_manager, my_vehicle_id)
            if direct_leader_data is not None:
                leader_velocity = direct_leader_data.get('velocity', None)
                if leader_velocity is not None:
                    self.leader_velocity = leader_velocity
                    if self.logger:
                        self.logger.logger.debug(f"Updated leader velocity from V2V: {leader_velocity:.2f}m/s")
        except Exception as e:
            if self.logger:
                self.logger.logger.warning(f"Error updating leader velocity from V2V: {e}")
    
    # ===== FORMATION TIMING AND STATUS =====
    
    def has_formation_timeout(self) -> bool:
        """Check if formation has timed out"""
        if self.formation_start_time is None:
            return False
        
        elapsed = time.time() - self.formation_start_time
        return elapsed > self.config.forming_timeout
    
    # ===== HYBRID CONTROL (combines perception + communication) =====
    
    # ===== VELOCITY SETTINGS FOR DIFFERENT PHASES =====
    
    def get_leader_velocity(self) -> float:
        """Get velocity for leader during formation"""
        return self.config.formation_speed
    
    def get_active_velocity(self) -> float:
        """Get velocity for active platoon"""
        return self.config.active_speed
    
    def get_telemetry(self) -> Dict:
        """Get platoon telemetry data for network transmission"""
        return {
            'platoon_enabled': self.enabled,
            'platoon_id': self.platoon_id if self.platoon_id else '',
            'platoon_role': 'Leader' if self.is_leader else 'Follower' if self.enabled else 'None',
            'platoon_active': self.enabled,  # For GUI compatibility
            'leader_id': self.leader_car_id if self.leader_car_id else '',
            'leader_detected': self.leader_detected,
            'leader_distance': self.leader_distance if self.leader_distance is not None else 0.0,
            'spacing_stable': self.is_spacing_stable() if not self.is_leader else None,
            'followers_ready': ','.join(map(str, self.followers_ready)) if self.is_leader and self.followers_ready else '',
            'all_ready': self.are_all_followers_ready() if self.is_leader else None,
            'formation_ready': self.is_spacing_stable() if not self.is_leader else self.are_all_followers_ready(),
            'desired_speed': self.get_active_velocity() if self.enabled else 0.0,
            'spacing_error': (self.leader_distance - self.config.target_spacing) if self.leader_distance is not None else 0.0
        }
    
    def reset(self):
        """Reset controller state"""
        self.spacing_error_integral = 0.0
        self.filtered_velocity_adjustment = 0.0
        self.spacing_stable_time = None
