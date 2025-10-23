import numpy as np
import math

class CACC:
    def __init__(self, controller):
        self.controller = controller
        self.param_opt = controller.param_opt
        self.param_sys = controller.param_sys
        self.goal = controller.goal
        self.straightlane = controller.straightlane
        self.vehicle_id = controller.vehicle_id
        
        # Cache frequently accessed parameters for performance
        self._cache_parameters()
    
    def _cache_parameters(self):
        """Cache frequently accessed parameters to avoid repeated dictionary lookups."""
        self.alpha = self.param_opt['alpha']
        self.beta = self.param_opt['beta']
        self.v0 = self.param_opt['v0']
        self.delta_exp = self.param_opt['delta']
        self.T = self.param_opt['T']
        self.s0 = self.param_opt['ri']
        self.K = self.param_opt['K']
        self.h = self.param_opt['hi']
        
        # Acceleration limits - FIXED: More reasonable limits for QCar
        # QCar is small and slow, so use gentler acceleration limits
        self.max_acc = 1.5   # Maximum acceleration (m/s²) - reduced from 5.0
        self.min_acc = -1.5  # Maximum deceleration (m/s²) - reduced from -5.0
        
        # Smoothing filter for acceleration commands
        self.prev_acc = 0.0  # Previous acceleration command
        self.alpha_filter = 0.5  # Smoothing factor (0 = no new input, 1 = no smoothing)

    def compute_cacc_acceleration(self, follower_state, leader_state):
        """
        Compute CACC acceleration directly from follower and leader states.
        This is a cleaner interface that doesn't rely on the complex controller framework.
        
        Args:
            follower_state: [x, y, theta, v] of follower vehicle
            leader_state: [x, y, theta, v] of leader vehicle
            
        Returns:
            float: Acceleration command
        """
        # Unpack states
        x, y, theta, v = follower_state
        x_j, y_j, theta_j, v_j = leader_state
        
        # Calculate actual distance between vehicles
        pos = np.array([x, y])
        pos_j = np.array([x_j, y_j])
        d_vec = pos_j - pos
        
        # Use Euclidean distance for spacing measurement
        spacing = np.linalg.norm(d_vec)
        
        # Alternative: Project distance along follower's heading (more accurate for lane following)
        # This measures the distance component in the direction the follower is moving
        # unit_heading_follower = np.array([math.cos(theta), math.sin(theta)])
        # spacing = d_vec @ unit_heading_follower
        
        # Calculate target spacing using constant time headway policy
        # s* = s0 + h * v (spacing increases with velocity)
        spacing_target = self.s0 + self.h * v
        
        # Calculate errors
        spacing_error = spacing - spacing_target  # Positive if too far, negative if too close
        velocity_error = v_j - v  # Positive if leader is faster
        
        # Compute cooperative acceleration using CACC control law
        # u = K * [spacing_error; velocity_error]
        control_vector = np.array([spacing_error, velocity_error])
        u_coop = self.K @ control_vector
        acc = u_coop[0]
        
        # Apply acceleration limits for safety
        acc = max(self.min_acc, min(acc, self.max_acc))
        
        # Apply exponential smoothing to reduce jerkiness
        # Smoothed output = alpha * new_value + (1 - alpha) * previous_value
        acc_smoothed = self.alpha_filter * acc + (1 - self.alpha_filter) * self.prev_acc
        self.prev_acc = acc_smoothed
        
        return acc_smoothed

    def get_optimal_input(self, host_car_id, state, last_input, lane_id, input_log,
                          initial_lane_id, direction_flag, type_state, acc_flag):
        """
        Legacy interface for backward compatibility.
        Uses the controller framework to get surrounding vehicles.
        """
        acc_flag = 0

        # Unpack current state
        x, y, theta, v = self.unpack_state(state)

        # Get leading vehicles using controller framework
        _, surrounding_vehicles, _, _ = self.controller.get_surrounding_vehicles(
            x, lane_id, direction_flag, host_car_id)

        num_vehicles = len(surrounding_vehicles)

        if num_vehicles == 0:
            acc = 0  # No car to follow
        else:
            pos = np.array([x, y])
            u_coop = np.zeros(2)
            
            for car_j in surrounding_vehicles:
                if type_state == "true":
                    x_j = car_j.state[0]
                    y_j = car_j.state[1]
                    v_j = car_j.state[3]
                else:
                    est = self.controller.vehicle.observer.est_global_state_current
                    x_j = est[0, car_j.vehicle_ic]
                    y_j = est[1, car_j.vehicle_ic]
                    v_j = est[3, car_j.vehicle_id]

                pos_j = np.array([x_j, y_j])
                d_vec = pos_j - pos
                s = np.linalg.norm(d_vec)

                spacing_target = ((host_car_id - car_j.vehicle_id) * self.s0 + self.h * v)
                spacing_error = s - spacing_target
                velocity_error = v_j - v

                u_coop += self.K @ np.array([spacing_error, velocity_error])
                
            u_coop /= num_vehicles
            acc = u_coop[0]  # Cooperative acceleration
            acc = max(self.min_acc, min(acc, self.max_acc))

        # Constant steering
        delta = 0
        input_u = [acc, delta]
        e = 0  # Error placeholder
        return acc_flag, input_u, e

    def unpack_input(self, input_u):
        acc = input_u[0]
        beta = input_u[1]
        return acc, beta

    def unpack_state(self, state):
        x, y, psi, v = state
        return x, y, psi, v
