import math

class IDMControl:
    def __init__(self, alpha=1.0, beta=1.5, v0=1.0, delta=4, T=0.4, s0=7, logger=None):
        """Simple IDM controller initialization."""
        self.alpha = alpha
        self.beta = beta
        self.v0 = v0
        self.delta = delta
        self.T = T
        self.s0 = s0
        self.logger = logger

    def compute_idm_acceleration(self, follower_state, leader_state):
        """Compute IDM acceleration."""
        x, y, theta, v = follower_state
        x_j, y_j, theta_j, v_j = leader_state

        # Calculate spacing and relative velocity
        s = math.hypot(x_j - x, y_j - y)
        delta_v = v - v_j
        
        # IDM formula
        s_star = self.s0 + self.T * v + (v * delta_v) / (2 * (self.alpha * self.beta)**0.5)
        acc = self.alpha * (1 - (v / self.v0)**self.delta - (s_star / s)**2)

        if self.logger:
            self.logger.debug(f"IDM: spacing={s:.2f}m, s_star={s_star:.2f}m, acc={acc:.2f}m/s²")

        return acc


