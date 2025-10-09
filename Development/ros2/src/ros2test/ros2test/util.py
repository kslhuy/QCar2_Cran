from math import atan2
import numpy as np


def yaw_to_quaternion(yaw):
    # Converts yaw angle to quaternion (x,y,z,w)
    qx = 0.0
    qy = 0.0
    qz = np.sin(yaw / 2.0)
    qw = np.cos(yaw / 2.0)
    return [qx, qy, qz, qw]

def quaternion_to_yaw(x, y, z, w):
    """
    Converts a quaternion (x, y, z, w) into yaw (rotation around Z axis).
    """
    # Yaw (Z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    return yaw