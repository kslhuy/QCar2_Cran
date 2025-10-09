import math
import numpy as np
import logging
from typing import Optional, Tuple
from pal.utilities.math import wrap_to_pi


class SimplifiedFollowerController:
    """
    A clean, high-performance follower controller that implements CACC and IDM
    algorithms directly without the overhead of DummyVehicle wrappers.
    """
    
    def __init__(self, vehicle_id: int, controller_type: str = "CACC", config=None, logger=None):
        """
        Initialize the simplified follower controller.
        
        Args:
            vehicle_id: ID of the follower vehicle
            controller_type: Type of controller ("CACC" or "IDM")
            config: Configuration object containing vehicle parameters
            logger: Logger instance for this controller
        """
        self.vehicle_id = vehicle_id
        self.controller_type = controller_type
        self.config = config
        self.logger = logger or logging.getLogger(f"SimplifiedFollowerController_{vehicle_id}")
        
        # Control parameters - use config if available, otherwise defaults
        self.lookahead_distance = config.lookahead_distance if config else 0.4
        self.max_steering = config.max_steering if config else 0.6
        self.k_steering = 2.0  # Steering gain
        
        # Leader tracking
        self.leader_vehicle = None
        
        # Initialize controller parameters
        self._init_controller_params()
        
        self.logger.info(f"Simplified follower controller initialized for vehicle {vehicle_id} with {controller_type}")
    
    def _init_controller_params(self):
        """Initialize controller-specific parameters."""
        if self.config is not None:
            params = self.config.get_dummy_controller_params(self.vehicle_id)
        else:
            # Default parameters
            params = {
                'alpha': 1.0,
                'beta': 1.5,
                'v0': 1.0,
                'delta': 4,
                'T': 0.4,
                's0': 7,  # for IDM
                'ri': 7,  # for CACC
                'hi': 0.5,
                'K': np.array([[1, 0.0], [0.0, 1]])  # for CACC
            }
        
        # Cache parameters for performance
        self.alpha = params['alpha']
        self.beta = params['beta']
        self.v0 = params['v0']
        self.delta_exp = params['delta']
        self.T = params['T']
        self.s0 = params.get('s0', params.get('ri', 7))  # IDM uses s0, CACC uses ri
        self.h = params['hi']
        self.K = params['K']
        
        # Acceleration limits
        self.max_acc = 30.0
        self.min_acc = -30.0
    
    def set_leader(self, leader_vehicle):
        """Set the leader vehicle for this follower to track."""
        self.leader_vehicle = leader_vehicle
        self.logger.info(f"Leader vehicle set to {leader_vehicle.vehicle_id if leader_vehicle else 'None'}")
    
    def compute_control(self, current_pos: list, current_rot: list, velocity: float, dt: float) -> Tuple[float, float]:
        """
        Compute control commands for the follower vehicle.
        
        Args:
            current_pos: Current position [x, y, z] of the follower
            current_rot: Current rotation [roll, pitch, yaw] of the follower
            velocity: Current velocity of the follower
            dt: Time step
            
        Returns:
            Tuple of (forward_speed, steering_angle)
        """
        if self.leader_vehicle is None:
            self.logger.warning("No leader vehicle set, returning zero commands")
            return 0.0, 0.0
        
        try:
            # Get leader state
            leader_pos = self.leader_vehicle.current_pos
            leader_rot = self.leader_vehicle.current_rot
            leader_velocity = self.leader_vehicle.velocity
            
            # Compute longitudinal control (speed command)
            speed_cmd = self._compute_longitudinal_control(
                current_pos, current_rot, velocity,
                leader_pos, leader_rot, leader_velocity
            )
            
            # Compute lateral control (steering command)
            steering_cmd = self._compute_lateral_control(
                current_pos, current_rot,
                leader_pos, leader_rot
            )
            
            # Log control commands for debugging
            self.logger.debug(f"Control commands - Speed: {speed_cmd:.3f}, Steering: {steering_cmd:.3f}")
            
            return speed_cmd, steering_cmd
            
        except Exception as e:
            self.logger.error(f"Error computing control: {e}")
            return 0.0, 0.0
    
    def _compute_longitudinal_control(self, current_pos: list, current_rot: list, velocity: float,
                                    leader_pos: list, leader_rot: list, leader_velocity: float) -> float:
        """
        Compute the longitudinal control using direct implementation of CACC or IDM.
        
        Returns:
            Speed command (acceleration)
        """
        try:
            if self.controller_type == "CACC":
                return self._compute_cacc_acceleration(
                    current_pos, current_rot, velocity,
                    leader_pos, leader_rot, leader_velocity
                )
            elif self.controller_type == "IDM":
                return self._compute_idm_acceleration(
                    current_pos, current_rot, velocity,
                    leader_pos, leader_rot, leader_velocity
                )
            else:
                self.logger.error(f"Unknown controller type: {self.controller_type}")
                return 0.0
                
        except Exception as e:
            self.logger.error(f"Error in longitudinal control: {e}")
            return 0.0
    
    def _compute_cacc_acceleration(self, current_pos: list, current_rot: list, velocity: float,
                                 leader_pos: list, leader_rot: list, leader_velocity: float) -> float:
        """Direct CACC implementation without DummyVehicle overhead."""
        # Calculate spacing
        dx = leader_pos[0] - current_pos[0]
        dy = leader_pos[1] - current_pos[1]
        spacing = math.sqrt(dx*dx + dy*dy)
        
        # Calculate target spacing and errors
        spacing_target = self.s0 + self.h * velocity
        spacing_error = spacing - spacing_target
        velocity_error = leader_velocity - velocity
        
        # CACC control law
        control_vector = np.array([spacing_error, velocity_error])
        u_coop = self.K @ control_vector
        acceleration = u_coop[0]
        
        # Apply limits
        acceleration = max(self.min_acc, min(acceleration, self.max_acc))
        
        return acceleration
    
    def _compute_idm_acceleration(self, current_pos: list, current_rot: list, velocity: float,
                                leader_pos: list, leader_rot: list, leader_velocity: float) -> float:
        """Direct IDM implementation without DummyVehicle overhead."""
        # Calculate spacing
        dx = leader_pos[0] - current_pos[0]
        dy = leader_pos[1] - current_pos[1]
        spacing = math.sqrt(dx*dx + dy*dy)
        
        # IDM model parameters
        velocity_ratio = velocity / self.v0 if self.v0 > 0 else 0
        
        # Desired spacing calculation
        velocity_diff = velocity - leader_velocity
        desired_spacing = self.s0 + max(0, velocity * self.T + 
                                      (velocity * velocity_diff) / 
                                      (2 * math.sqrt(self.alpha * self.beta)))
        
        # IDM acceleration formula
        acceleration = self.alpha * (1 - velocity_ratio**self.delta_exp - 
                                   (desired_spacing / spacing)**2) if spacing > 0 else 0
        
        # Apply limits
        acceleration = max(self.min_acc, min(acceleration, self.max_acc))
        
        return acceleration
    
    def _compute_lateral_control(self, current_pos: list, current_rot: list,
                               leader_pos: list, leader_rot: list) -> float:
        """
        Compute the lateral control (steering command) using pure pursuit.
        
        Returns:
            Steering command
        """
        try:
            # Calculate target position (behind leader with lookahead)
            target_x = leader_pos[0] - self.lookahead_distance * math.cos(leader_rot[2])
            target_y = leader_pos[1] - self.lookahead_distance * math.sin(leader_rot[2])
            
            # Calculate steering command using pure pursuit
            dx = target_x - current_pos[0]
            dy = target_y - current_pos[1]
            target_angle = math.atan2(dy, dx)
            heading_error = wrap_to_pi(target_angle - current_rot[2])
            
            # Apply steering gain and limit
            steering_cmd = -self.k_steering * heading_error
            steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))
            
            return steering_cmd
            
        except Exception as e:
            self.logger.error(f"Error in lateral control: {e}")
            return 0.0
    
    def update_parameters(self, **kwargs):
        """Update control parameters dynamically."""
        if 'lookahead_distance' in kwargs:
            self.lookahead_distance = kwargs['lookahead_distance']
            self.logger.info(f"Updated lookahead distance to {self.lookahead_distance}")
            
        if 'max_steering' in kwargs:
            self.max_steering = kwargs['max_steering']
            self.logger.info(f"Updated max steering to {self.max_steering}")
            
        if 'k_steering' in kwargs:
            self.k_steering = kwargs['k_steering']
            self.logger.info(f"Updated steering gain to {self.k_steering}")
    
    def get_control_state(self) -> dict:
        """Get the current state of the controller."""
        return {
            'vehicle_id': self.vehicle_id,
            'controller_type': self.controller_type,
            'has_leader': self.leader_vehicle is not None,
            'leader_id': self.leader_vehicle.vehicle_id if self.leader_vehicle else None,
            'lookahead_distance': self.lookahead_distance,
            'max_steering': self.max_steering,
            'k_steering': self.k_steering,
            'control_params': {
                'alpha': self.alpha,
                'beta': self.beta,
                'v0': self.v0,
                's0': self.s0,
                'h': self.h
            }
        }
    
    def stop_control(self):
        """Stop the controller and clean up resources."""
        self.logger.info(f"Stopping simplified follower controller for vehicle {self.vehicle_id}")
        self.leader_vehicle = None
