"""
Platoon Controller - Manages vehicle platoon formation and coordination
"""
import time
import numpy as np
from typing import Optional, Dict, List
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
        """Disable platoon mode"""
        self.enabled = False
        self.is_leader = False
        self.platoon_id = None
        self.leader_car_id = None
        self.leader_detected = False
        self.followers_ready.clear()
        self.expected_followers = []
        
        if self.logger:
            self.logger.logger.info("Platoon mode disabled")
    
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
    
    def update_leader_info(self, detected: bool, distance: Optional[float] = None,
                          velocity: Optional[float] = None):
        """
        Update information about the leader (for followers)
        
        Args:
            detected: Whether leader is currently detected via YOLO
            distance: Distance to leader from YOLO (meters)
            velocity: Leader velocity from network telemetry (m/s)
        """
        self.leader_detected = detected
        
        if detected:
            self.last_leader_seen = time.time()
            self.leader_distance = distance
            
        if velocity is not None:
            self.leader_velocity = velocity
    
    def update_follower_status(self, follower_id: int, is_ready: bool):
        """
        Update follower readiness status (for leader)
        
        Args:
            follower_id: ID of the follower vehicle
            is_ready: Whether follower has achieved proper spacing
        """
        if is_ready:
            self.followers_ready.add(follower_id)
        else:
            self.followers_ready.discard(follower_id)
    
    def are_all_followers_ready(self) -> bool:
        """Check if all expected followers are ready"""
        if not self.is_leader:
            return False
        
        return len(self.followers_ready) == len(self.expected_followers)
    
    def is_spacing_stable(self) -> bool:
        """Check if spacing to leader is stable (for followers)"""
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
        """Check if follower has lost connection to leader"""
        if not self.leader_detected or self.last_leader_seen is None:
            return True
        
        time_since_seen = time.time() - self.last_leader_seen
        return time_since_seen > self.config.lost_timeout
    
    def has_formation_timeout(self) -> bool:
        """Check if formation has timed out"""
        if self.formation_start_time is None:
            return False
        
        elapsed = time.time() - self.formation_start_time
        return elapsed > self.config.forming_timeout
    
    def compute_follower_velocity(self, current_velocity: float, 
                                  base_velocity: float) -> float:
        """
        Compute velocity for follower to maintain spacing
        
        Args:
            current_velocity: Current vehicle velocity (m/s)
            base_velocity: Base/target velocity (m/s)
            
        Returns:
            Adjusted velocity to maintain spacing (m/s)
        """
        if not self.leader_detected or self.leader_distance is None:
            # No leader detected, maintain base velocity
            return base_velocity
        
        # Compute spacing error
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
        
        # Compute target velocity
        if self.leader_velocity is not None:
            # Use leader's actual velocity if available
            target_velocity = self.leader_velocity + self.filtered_velocity_adjustment
        else:
            # Otherwise adjust from base velocity
            target_velocity = base_velocity + self.filtered_velocity_adjustment
        
        # Safety limits
        if self.leader_distance < self.config.min_safe_spacing:
            # Too close! Slow down significantly
            target_velocity = min(target_velocity, current_velocity * 0.5)
        
        # Clip to reasonable range
        target_velocity = np.clip(target_velocity, 0.0, base_velocity * 1.5)
        
        # Debug logging (occasional)
        if self.logger and hasattr(self, '_debug_counter'):
            self._debug_counter = getattr(self, '_debug_counter', 0) + 1
            if self._debug_counter % 100 == 0:  # Log every 100 calls
                self.logger.logger.debug(
                    f"Platoon follower control: "
                    f"dist={self.leader_distance:.2f}m, "
                    f"error={spacing_error:.2f}m, "
                    f"adj={self.filtered_velocity_adjustment:.3f}m/s, "
                    f"target_v={target_velocity:.2f}m/s"
                )
        
        return target_velocity
    
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
