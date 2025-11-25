"""
Refactored Vehicle Controller with improved architecture
"""
import numpy as np
import time
from typing import Optional, Dict, Any
from threading import Lock

from pal.utilities.math import wrap_to_pi
from hal.content.qcar_functions import QCarEKF


class SpeedController:
    """PI speed controller with anti-windup"""
    
    def __init__(self, config=None, logger=None):
        self.logger = logger
        
        if config:
            self.kp = config.speed.K_p
            self.ki = config.speed.K_i
            self.max_throttle = config.speed.max_throttle
        else:
            self.kp = 0.1
            self.ki = 1.0
            self.max_throttle = 0.3
        
        self.ei = 0  # Integral error
        self.last_error = 0
        
        # Anti-windup
        self.ei_max = 1.0
        
    def update(self, v: float, v_ref: float, dt: float) -> float:
        """
        Update speed controller
        
        Args:
            v: Current velocity
            v_ref: Reference velocity
            dt: Time step
            
        Returns:
            Throttle command
        """
        # Calculate error
        e = v_ref - v
        
        # Integral with anti-windup
        self.ei += dt * e
        self.ei = np.clip(self.ei, -self.ei_max, self.ei_max)
        
        # PI control
        u = self.kp * e + self.ki * self.ei
        
        # Clamp output
        u = np.clip(u, -self.max_throttle, self.max_throttle)
        
        self.last_error = e
        
        return u
    
    def reset(self):
        """Reset controller state"""
        self.ei = 0
        self.last_error = 0
        
        if self.logger:
            self.logger.log_control_event("speed_controller_reset", {})


class SteeringController:
    """Stanley steering controller"""
    
    def __init__(self, waypoints: np.ndarray, config=None, logger=None, cyclic: bool = True):
        self.logger = logger
        
        if config:
            self.k = config.steering.K_stanley
            self.max_steering_angle = config.steering.max_steering_angle
        else:
            self.k = 0.7
            self.max_steering_angle = np.pi / 6
        
        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0
        self.cyclic = cyclic
        
        # Reference values
        self.p_ref = np.array([0.0, 0.0])
        self.th_ref = 0.0
        
        # Errors
        self.cross_track_error = 0.0
        self.heading_error = 0.0
        
        # Thread safety
        self._lock = Lock()
    
    def update(self, p: np.ndarray, th: float, speed: float) -> float:
        """
        Update steering controller
        
        Args:
            p: Current position [x, y]
            th: Current heading
            speed: Current speed
            
        Returns:
            Steering angle command
        """
        with self._lock:
            # Get current waypoints
            wp_1 = self.wp[:, np.mod(self.wpi, self.N - 1)]
            wp_2 = self.wp[:, np.mod(self.wpi + 1, self.N - 1)]
            
            # Path vector
            v = wp_2 - wp_1
            v_mag = np.linalg.norm(v)
            
            # Handle zero-length segment
            if v_mag < 1e-6:
                v_uv = np.array([1.0, 0.0])
            else:
                v_uv = v / v_mag
            
            # Path tangent angle
            tangent = np.arctan2(v_uv[1], v_uv[0])
            
            # Progress along current segment
            s = np.dot(p - wp_1, v_uv)
            
            # Check if we should advance to next waypoint
            if s >= v_mag:
                if self.cyclic or self.wpi < self.N - 2:
                    self.wpi += 1
                    if self.logger:
                        self.logger.log_control_event(
                            "waypoint_reached",
                            {"index": self.wpi}
                        )
            
            # Closest point on path
            ep = wp_1 + v_uv * s
            
            # Cross-track error vector
            ct = ep - p
            
            # Direction of cross-track error
            dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)
            
            # Signed cross-track error
            ect = np.linalg.norm(ct) * np.sign(dir)
            
            # Heading error
            psi = wrap_to_pi(tangent - th)
            
            # Store for telemetry
            self.p_ref = ep
            self.th_ref = tangent
            self.cross_track_error = ect
            self.heading_error = psi
            
            # Stanley control law
            # Avoid division by zero
            speed_safe = max(speed, 0.1)
            delta = wrap_to_pi(psi + np.arctan2(self.k * ect, speed_safe))
            
            # Clamp steering angle
            delta = np.clip(delta, -self.max_steering_angle, self.max_steering_angle)
            
            return delta
    
    def get_reference_pose(self) -> tuple:
        """Get current reference pose"""
        with self._lock:
            return self.p_ref.copy(), self.th_ref
    
    def get_errors(self) -> tuple:
        """Get current control errors"""
        with self._lock:
            return self.cross_track_error, self.heading_error
    
    def get_waypoint_index(self) -> int:
        """Get current waypoint index"""
        with self._lock:
            return self.wpi
    
    def reset(self, waypoints: Optional[np.ndarray] = None):
        """Reset controller state"""
        with self._lock:
            if waypoints is not None:
                self.wp = waypoints
                self.N = len(waypoints[0, :])
            
            self.wpi = 0
            self.p_ref = np.array([0.0, 0.0])
            self.th_ref = 0.0
            self.cross_track_error = 0.0
            self.heading_error = 0.0
            
            if self.logger:
                self.logger.log_control_event("steering_controller_reset", {})


class StateEstimator:
    """State estimation wrapper for GPS/EKF fusion"""
    
    def __init__(self, gps, initial_pose=None, logger=None, use_ekf=True):
        self.gps = gps
        self.logger = logger
        self.use_ekf = use_ekf
        
        # Initialize EKF if using state estimation
        if self.use_ekf and initial_pose is not None:
            self.ekf = QCarEKF(x_0=initial_pose)
        else:
            self.ekf = None
        
        # Current state (from EKF or GPS)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.velocity = 0.0
        
        # State validity
        self.state_valid = False
        self.gps_updated = False
        
        # Lock for thread safety
        self._lock = Lock()
        
        # Store last control inputs for EKF
        self.last_motor_tach = 0.0
        self.last_steering = 0.0
    
    def update(self, motor_tach: float = 0.0, steering: float = 0.0, 
               dt: float = 0.005, gyro_z: float = 0.0) -> bool:
        """
        Update state estimate using EKF with GPS and IMU fusion
        
        Args:
            motor_tach: Motor tachometer reading (velocity)
            steering: Current steering angle
            dt: Time step
            gyro_z: Gyroscope Z-axis reading
            
        Returns:
            True if update was successful
        """
        with self._lock:
            try:
                # Read GPS
                self.gps_updated = self.gps.readGPS()
                
                if self.use_ekf and self.ekf is not None:
                    # EKF-based estimation (matches original vehicle_control.py)
                    if self.gps_updated:
                        # GPS measurement available - update with GPS
                        y_gps = np.array([
                            self.gps.position[0],
                            self.gps.position[1],
                            self.gps.orientation[2]
                        ])
                        
                        self.ekf.update(
                            [motor_tach, steering],
                            dt,
                            y_gps,
                            gyro_z
                        )
                    else:
                        # No GPS - update with dead reckoning only
                        self.ekf.update(
                            [motor_tach, steering],
                            dt,
                            None,
                            gyro_z
                        )
                    
                    # Extract state from EKF
                    self.x = self.ekf.x_hat[0, 0]
                    self.y = self.ekf.x_hat[1, 0]
                    self.theta = self.ekf.x_hat[2, 0]
                    self.velocity = motor_tach  # Use motor tach for velocity
                    
                    self.state_valid = True
                    
                else:
                    # GPS-only estimation (fallback)
                    if self.gps_updated:
                        self.x = self.gps.position[0]
                        self.y = self.gps.position[1]
                        self.theta = self.gps.orientation[2]
                        
                        # Velocity from motor tach or GPS
                        self.velocity = motor_tach
                        
                        self.state_valid = True
                    else:
                        # No new GPS data
                        self.state_valid = False
                
                # Store for next iteration
                self.last_motor_tach = motor_tach
                self.last_steering = steering
                
                return self.state_valid
                
            except Exception as e:
                if self.logger:
                    self.logger.log_error("State estimation failed", e)
                self.state_valid = False
                return False
    
    def get_state(self) -> tuple:
        """Get current state estimate"""
        with self._lock:
            return self.x, self.y, self.theta, self.velocity, self.state_valid
    
    def get_position(self) -> np.ndarray:
        """Get current position"""
        with self._lock:
            return np.array([self.x, self.y])
    
    def get_pose(self) -> tuple:
        """Get current pose"""
        with self._lock:
            return self.x, self.y, self.theta
    
    def was_updated(self) -> bool:
        """Check if GPS was updated in last cycle"""
        with self._lock:
            return self.gps_updated
