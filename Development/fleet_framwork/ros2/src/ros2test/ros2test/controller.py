import numpy as np
from pal.utilities.math import wrap_to_pi


class SpeedController:

    def __init__(self, kp=0, ki=0):
        self.maxThrottle = 0.6

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
        wp_1 = self.wp[:, np.mod(self.wpi, self.N-1)]
        wp_2 = self.wp[:, np.mod(self.wpi+1, self.N-1)]
        
        v = wp_2 - wp_1
        v_mag = np.linalg.norm(v)
        try:
            v_uv = v / v_mag
        except ZeroDivisionError:
            return 0

        tangent = np.arctan2(v_uv[1], v_uv[0])

        s = np.dot(p-wp_1, v_uv)

        if s >= v_mag:
            if  self.cyclic or self.wpi < self.N-2:
                self.wpi += 1

        ep = wp_1 + v_uv*s
        ct = ep - p
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

        ect = np.linalg.norm(ct) * np.sign(dir)
        psi = wrap_to_pi(tangent-th)

        self.p_ref = ep
        self.th_ref = tangent
    

        return np.clip(
            wrap_to_pi(psi + np.arctan2(self.k*ect, speed)),
            -self.maxSteeringAngle,
            self.maxSteeringAngle)
        