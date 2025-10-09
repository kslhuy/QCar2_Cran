class LiDARWrapper:
    """
    Wrapper for LiDAR sensor
    """
    def __init__(self, qcar_handle):
        self.qcar = qcar_handle
    
    def get_scan(self):
        """
        Get LiDAR scan data
        
        Returns:
            angles: Array of angles (radians)
            distances: Array of distances (meters)
        """
        success, angles, distances = self.qcar.get_lidar(samplePoints=360)
        if success:
            return angles, distances
        return None, None

