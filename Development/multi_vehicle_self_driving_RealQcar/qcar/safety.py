"""
Safety monitoring and validation for vehicle control
"""
import numpy as np
from typing import Tuple, Optional
import time


class ControlValidator:
    """Validates control commands and state values"""
    
    def __init__(self, config=None, logger=None):
        self.logger = logger
        
        # Limits from config
        if config:
            self.max_throttle = config.speed.max_throttle
            self.max_steering = config.steering.max_steering_angle
        else:
            self.max_throttle = 0.3
            self.max_steering = np.pi / 6
        
        self.validation_failures = {
            'throttle': 0,
            'steering': 0,
            'state': 0,
            'velocity': 0
        }
    
    def validate_throttle(self, u: float) -> Tuple[bool, float]:
        """
        Validate and clamp throttle command
        
        Args:
            u: Throttle command
            
        Returns:
            (is_valid, clamped_value)
        """
        is_valid = -self.max_throttle <= u <= self.max_throttle
        clamped = np.clip(u, -self.max_throttle, self.max_throttle)
        
        if not is_valid:
            self.validation_failures['throttle'] += 1
            if self.logger:
                self.logger.log_warning(
                    f"Throttle out of bounds: {u:.3f}, clamped to {clamped:.3f}"
                )
        
        return is_valid, clamped
    
    def validate_steering(self, delta: float) -> Tuple[bool, float]:
        """
        Validate and clamp steering command
        
        Args:
            delta: Steering angle command
            
        Returns:
            (is_valid, clamped_value)
        """
        is_valid = -self.max_steering <= delta <= self.max_steering
        clamped = np.clip(delta, -self.max_steering, self.max_steering)
        
        if not is_valid:
            self.validation_failures['steering'] += 1
            if self.logger:
                self.logger.log_warning(
                    f"Steering out of bounds: {delta:.3f}, clamped to {clamped:.3f}"
                )
        
        return is_valid, clamped
    
    def validate_state(self, x: float, y: float, theta: float) -> bool:
        """
        Validate state values
        
        Args:
            x, y: Position coordinates
            theta: Heading angle
            
        Returns:
            True if all values are valid
        """
        is_valid = (
            not np.isnan(x) and not np.isinf(x) and
            not np.isnan(y) and not np.isinf(y) and
            not np.isnan(theta) and not np.isinf(theta) and
            -np.pi <= theta <= np.pi
        )
        
        if not is_valid:
            self.validation_failures['state'] += 1
            if self.logger:
                self.logger.log_warning(
                    f"Invalid state: x={x:.3f}, y={y:.3f}, theta={theta:.3f}"
                )
        
        return is_valid
    
    def validate_velocity(self, v: float, max_velocity: float = 2.0) -> bool:
        """
        Validate velocity value
        
        Args:
            v: Velocity
            max_velocity: Maximum allowed velocity
            
        Returns:
            True if velocity is valid
        """
        is_valid = not np.isnan(v) and not np.isinf(v) and 0 <= v <= max_velocity
        
        if not is_valid:
            self.validation_failures['velocity'] += 1
            if self.logger:
                self.logger.log_warning(f"Invalid velocity: {v:.3f}")
        
        return is_valid
    
    def get_failure_counts(self) -> dict:
        """Get validation failure counts"""
        return self.validation_failures.copy()


class SensorHealthMonitor:
    """Monitor sensor health and handle failures"""
    
    def __init__(self, config=None, logger=None):
        self.logger = logger
        
        # Timeout thresholds
        if config:
            self.gps_timeout_max = config.safety.gps_timeout_max
        else:
            self.gps_timeout_max = 100
        
        # Counters
        self.gps_timeout_counter = 0
        self.gps_failures = 0
        
        # Status
        self.gps_healthy = True
        self.last_gps_update = time.time()
    
    def check_gps_health(self, gps_updated: bool) -> bool:
        """
        Check GPS sensor health
        
        Args:
            gps_updated: Whether GPS was updated in this cycle
            
        Returns:
            True if GPS is healthy
        """
        if gps_updated:
            self.gps_timeout_counter = 0
            self.last_gps_update = time.time()
            
            if not self.gps_healthy:
                if self.logger:
                    self.logger.logger.info("GPS signal restored")
                self.gps_healthy = True
        else:
            self.gps_timeout_counter += 1
            
            if self.gps_timeout_counter > self.gps_timeout_max:
                if self.gps_healthy:
                    self.gps_failures += 1
                    if self.logger:
                        self.logger.log_warning(
                            "GPS signal lost! Using dead reckoning"
                        )
                    self.gps_healthy = False
        
        return self.gps_healthy
    
    def get_time_since_gps_update(self) -> float:
        """Get time since last GPS update in seconds"""
        return time.time() - self.last_gps_update
    
    def get_health_status(self) -> dict:
        """Get sensor health status"""
        return {
            'gps_healthy': self.gps_healthy,
            'gps_timeout_counter': self.gps_timeout_counter,
            'gps_failures': self.gps_failures,
            'time_since_gps_update': self.get_time_since_gps_update()
        }


class CollisionAvoidance:
    """Collision detection and avoidance"""
    
    def __init__(self, config=None, logger=None):
        self.logger = logger
        
        # Safety thresholds
        if config:
            self.emergency_stop_distance = config.safety.emergency_stop_distance
        else:
            self.emergency_stop_distance = 0.2
        
        self.collision_warnings = 0
        self.emergency_stops = 0
    
    def check_collision_risk(
        self, 
        car_distance: float, 
        person_distance: float,
        current_velocity: float
    ) -> Tuple[bool, str]:
        """
        Check for collision risk
        
        Args:
            car_distance: Distance to nearest car
            person_distance: Distance to nearest person
            current_velocity: Current vehicle velocity
            
        Returns:
            (should_emergency_stop, reason)
        """
        # Check car collision
        if car_distance < self.emergency_stop_distance:
            self.emergency_stops += 1
            if self.logger:
                self.logger.log_warning(
                    f"EMERGENCY STOP: Car too close ({car_distance:.2f}m)"
                )
            return True, "car_collision_imminent"
        
        # Check person collision
        if person_distance < self.emergency_stop_distance:
            self.emergency_stops += 1
            if self.logger:
                self.logger.log_warning(
                    f"EMERGENCY STOP: Person too close ({person_distance:.2f}m)"
                )
            return True, "person_collision_imminent"
        
        # Check warning distances
        warning_distance = self.emergency_stop_distance * 2
        if car_distance < warning_distance or person_distance < warning_distance:
            self.collision_warnings += 1
            min_dist = min(car_distance, person_distance)
            return False, f"collision_warning_{min_dist:.2f}m"
        
        return False, "clear"
    
    def get_statistics(self) -> dict:
        """Get collision avoidance statistics"""
        return {
            'collision_warnings': self.collision_warnings,
            'emergency_stops': self.emergency_stops
        }


class WatchdogTimer:
    """Watchdog timer for detecting system hangs"""
    
    def __init__(self, timeout: float = 1.0, logger=None):
        self.timeout = timeout
        self.logger = logger
        self.last_reset = time.time()
        self.timeout_count = 0
    
    def reset(self):
        """Reset the watchdog timer"""
        self.last_reset = time.time()
    
    def check(self) -> bool:
        """
        Check if watchdog has timed out
        
        Returns:
            True if timeout occurred
        """
        elapsed = time.time() - self.last_reset
        
        if elapsed > self.timeout:
            self.timeout_count += 1
            if self.logger:
                self.logger.log_error(
                    f"Watchdog timeout! Elapsed: {elapsed:.3f}s"
                )
            return True
        
        return False
    
    def get_elapsed(self) -> float:
        """Get time since last reset"""
        return time.time() - self.last_reset
