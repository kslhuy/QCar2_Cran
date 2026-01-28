import time
import numpy as np
import random

class MockQCarGPS:
    """Mock QCar GPS sensor."""
    def __init__(self, initial_pose=None):
        if initial_pose is None:
            initial_pose = [0.0, 0.0, 0.0]
        self.x = initial_pose[0]
        self.y = initial_pose[1]
        self.theta = initial_pose[2]
        
        # Configuration
        self.update_rate = 5.0  # Hz
        self.noise_std_pos = 0.05
        self.noise_std_theta = 0.02
        self.apply_noise = True
        
        self.last_update_time = time.time()
        self.last_data = np.array([self.x, self.y, self.theta])
        self.new_data_ready = False

    def readGPS(self):
        """
        Simulate GPS read with 5Hz update rate.
        Returns: [valid, x, y, theta]
        """
        current_time = time.time()
        
        # Check if enough time has passed for new sample
        if current_time - self.last_update_time >= (1.0 / self.update_rate):
            self.last_update_time = current_time
            self.new_data_ready = True
            
            # Update internal state from vehicle reference (pushed by vehicle usually)
            # But here we assume x, y, theta are updated externally or passed in read?
            # Actually, standard pattern is vehicle updates its true state, GPS reads it and adds noise.
            # But MockQCarGPS was embedded. Let's assume the vehicle pushes true state to it.
            
            # Apply noise
            x_meas = self.x
            y_meas = self.y
            theta_meas = self.theta
            
            if self.apply_noise:
                x_meas += random.gauss(0, self.noise_std_pos)
                y_meas += random.gauss(0, self.noise_std_pos)
                theta_meas += random.gauss(0, self.noise_std_theta)
            
            self.last_data = np.array([x_meas, y_meas, theta_meas])
            return True
            
        else:
            return False

    @property
    def position(self):
        """Return [x, y, z] position."""
        return np.array([self.x, self.y, 0.0])

    @property
    def orientation(self):
        """Return [roll, pitch, yaw] orientation."""
        # Simulated QCar uses [roll, pitch, yaw]
        return np.array([0.0, 0.0, self.theta])

    def update_true_state(self, x, y, theta):
        """Update the ground truth from the vehicle."""
        self.x = x
        self.y = y
        self.theta = theta    
