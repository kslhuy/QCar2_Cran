import numpy as np
from pal.utilities.math import wrap_to_pi

class SpeedController:
    def __init__(self, kp=0, ki=0):
        self.maxThrottle = 0.3
        self.kp = kp
        self.ki = ki
        self.ei = 0

    # ==============  SECTION A -  Speed Control  ====================
    def update(self, v, v_ref, dt):
        e = v_ref - v
        self.ei += dt*e
        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )

# The Idea Behind SteeringController

# The job of the steering controller is to make the vehicle follow a sequence of waypoints.

# The waypoints are given as an array wp ∈ ℝ²×N (2D positions).

# The vehicle’s state is given by its position p = (x, y) and heading θ (orientation).

# The controller computes the steering angle δ that aligns the vehicle to follow the road defined by the waypoints.

# The controller is based on geometric path tracking:

# Project the vehicle onto the nearest point on the current path segment.

# Compute the cross-track error (perpendicular distance from vehicle to path).

# Compute the heading error (difference between vehicle orientation and path tangent).

# Apply a correction law to calculate a steering angle.

class SteeringController:
    def __init__(self, waypoints, k=1, cyclic=True):
        self.maxSteeringAngle = np.pi/6
        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0
        self.k = k
        self.cyclic = cyclic
        self.p_ref = (0, 0)
        self.th_ref = 0
    # ==============  SECTION B -  Steering Control  ====================
    def update(self, p, th, speed):
        wp_1 = self.wp[:, np.mod(self.wpi, self.N-1)]  # starting waypoint
        wp_2 = self.wp[:, np.mod(self.wpi+1, self.N-1)]  # next waypoint
        
        v = wp_2 - wp_1 # vector along path segment
        v_mag = np.linalg.norm(v) # unit tangent vector , gives the direction of the road segment.
        try:
            v_uv = v / v_mag
        except ZeroDivisionError:
            return 0

        tangent = np.arctan2(v_uv[1], v_uv[0]) # tangent angle of the path segment , (desired heading along the path)

        s = np.dot(p-wp_1, v_uv)  # distance along the path , how far along the segment the vehicle has traveled.

        if s >= v_mag: #(past end of segment), the controller moves to the next waypoint.
            if  self.cyclic or self.wpi < self.N-2:
                self.wpi += 1
        ep = wp_1 + v_uv*s  # projection point , projection of vehicle onto road (nearest point on the line).
        ct = ep - p # cross-track vector , vector from vehicle to the road.
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent) # whether vehicle is left (+) or right (–) of the road.
        ect = np.linalg.norm(ct) * np.sign(dir)  #signed cross-track error.
        psi = wrap_to_pi(tangent-th) # difference between road heading and vehicle heading.
        # print (v, "<vector along path segment>", psi, "difference heading")
        self.p_ref = ep
        self.th_ref = tangent

        return np.clip(
            wrap_to_pi(psi + np.arctan2(self.k*ect, speed)),
            -self.maxSteeringAngle,
            self.maxSteeringAngle)