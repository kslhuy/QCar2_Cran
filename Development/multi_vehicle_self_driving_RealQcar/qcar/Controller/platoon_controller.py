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
    
    
    def get_direct_leader_vehicle_id(self) -> Optional[int]:
        """Resolve the direct leader vehicle ID from the current formation data."""
        try:
            if hasattr(self, "formation_data") and hasattr(self, "my_position"):
                my_position = self.my_position
                if my_position > 1:
                    direct_leader_position = my_position - 1
                    for vehicle_id, position in self.formation_data.items():
                        vehicle_id_int = int(vehicle_id)
                        if position == direct_leader_position:
                            return vehicle_id_int

            if self.leader_car_id is not None:
                return int(self.leader_car_id)

            return None
        except Exception as e:
            if self.logger:
                self.logger.logger.warning(
                    f"Error resolving direct leader vehicle id: {e}"
                )
            return None

    def get_direct_leader_data_from_v2v(
        self,
        v2v_manager,
        my_vehicle_id: int,
        channel: str = "attacked",
    ) -> Optional[Dict[str, Any]]:
        """
        Get direct leader's state data from V2V manager.

        Args:
            v2v_manager: V2V manager instance
            my_vehicle_id: This vehicle's ID
            channel: V2V channel selector ('attacked' or 'clean')

        Returns:
            Direct leader's state data or None
        """
        try:
            if not v2v_manager or not hasattr(v2v_manager, "get_latest_local_state_raw"):
                if self.logger:
                    self.logger.logger.debug(
                        "V2V manager not available or missing get_latest_local_state_raw"
                    )
                return None

            leader_vehicle_id = self.get_direct_leader_vehicle_id()
            if leader_vehicle_id is None:
                return None

            return v2v_manager.get_latest_local_state_raw(
                leader_vehicle_id, channel=channel
            )

        except Exception as e:
            if self.logger:
                self.logger.logger.warning(
                    f"Error getting direct leader data from V2V: {e}"
                )
            return None
    
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
