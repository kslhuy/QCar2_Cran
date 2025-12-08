import numpy as np
import math

class CACC:
    def __init__(self, s0=7, h=0.5, K=None, max_acc=1.5, min_acc=-1.5, logger=None):
        """
        CACC controller initialization.
        
        Args:
            s0: Minimum spacing (meters)
            h: Time headway (seconds)  
            K: Control gains matrix [1x2] = [spacing_gain, velocity_gain]
            max_acc: Maximum acceleration (m/s²)
            min_acc: Minimum acceleration (m/s²)
            logger: Logger instance
        """
        self.s0 = s0  # spacing policy
        self.h = h    # time headway
        # K should be 1x2 matrix: [spacing_error_gain, velocity_error_gain]
        self.K = K if K is not None else np.array([[0.2, 0.05]])
        # self.max_acc = max_acc
        # self.min_acc = min_acc
        self.logger = logger
        self.prev_acc = 0.0
        self.alpha_filter = 0.5

    def compute_cacc_acceleration(self, follower_state, leader_state):
        """
        Compute CACC acceleration command.
        
        CACC uses 2 errors:
        1. Spacing error: current_spacing - desired_spacing  
        2. Velocity error: leader_velocity - follower_velocity
        
        Control law: acc = K[0] * spacing_error + K[1] * velocity_error
        """
        x, y, theta, v = follower_state
        x_j, y_j, theta_j, v_j = leader_state
        
        # Calculate distance and target spacing
        spacing = math.hypot(x_j - x, y_j - y)
        spacing_target = self.s0 + self.h * v
        
        # Calculate errors
        spacing_error = spacing - spacing_target  # Positive = too far, negative = too close
        velocity_error = v_j - v                  # Positive = leader faster, negative = leader slower
        
        # CACC control law: acc = K_spacing * e_spacing + K_velocity * e_velocity
        error_vector = np.array([spacing_error, velocity_error])
        acc = (self.K @ error_vector)[0]  # K is [1x2], error is [2x1], result is scalar
        
        # # Apply limits and smoothing
        # acc = max(self.min_acc, min(acc, self.max_acc))
        # acc_smoothed = self.alpha_filter * acc + (1 - self.alpha_filter) * self.prev_acc
        # self.prev_acc = acc_smoothed
        
        if self.logger:
            self.logger.debug(f"CACC: spacing={spacing:.2f}m, target={spacing_target:.2f}m, "
                            f"e_s={spacing_error:.2f}m, e_v={velocity_error:.2f}m/s, acc={acc:.2f}m/s2")
        
        return acc


