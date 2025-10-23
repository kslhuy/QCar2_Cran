import math
from hal.utilities.image_processing import ImageProcessing
import numpy as np
import cv2
class RealSenseWrapper:
    """
    Wrapper for RealSense camera (simulated with QCar cameras)
    """
    def __init__(self, qcar_handle , geometry_manager):
        self.qcar = qcar_handle
        
        
        # Simulated camera intrinsics for QCar camera

        # Camera parameters
        self.camera_fov_h = geometry_manager.camera_fov_h  # Horizontal FOV
        self.camera_fov_v = geometry_manager.camera_fov_v  # Vertical FOV


        self.fx = geometry_manager.fx
        self.fy = geometry_manager.fy
        self.cx = geometry_manager.cx
        self.cy = geometry_manager.cy
        # These would typically come from camera calibration
        self.intrinsics = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy], 
            [0, 0, 1]
        ], dtype=np.float32)
        
        self.img_proc = ImageProcessing()
        # self.img_proc.calibrate_camera()
    
    def get_frames_RT(self,myYolo=None):
        pass
        

    def get_frames(self,myYolo=None):
        """
        Get RGB and depth frames
        
        Returns:
            rgb_image: RGB image (numpy array)
            depth_image: Depth image (numpy array) - from depth camera, properly scaled
        """
        # Get RGB image
        success_rgb, rgb_image = self.qcar.get_image(camera=self.qcar.CAMERA_RGB)
        # Crop bottom pixels for better performance
        if success_rgb:
            rgb_image = myYolo.pre_process(rgb_image) # Resize since we crpopped

        # Get depth image from depth camera
        success_depth, depth_image = self.qcar.get_image(camera=self.qcar.CAMERA_DEPTH)
        
        if success_rgb and success_depth:

            if len(depth_image.shape) == 3:
                if depth_image.shape[2] == 3:  # Only if truly BGR-like
                    depth_gray = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
                elif depth_image.shape[2] == 1:  # Your case: Already single-channel
                    depth_gray = np.squeeze(depth_image, axis=-1)
                else:
                    raise ValueError("Unexpected channel count")
                depth_image = depth_gray  # Now 2D (480, 640)
                
                # Enhanced depth processing for QCar
                # QCar depth values need proper scaling - adjust based on actual range
                if depth_image.dtype == np.uint8:
                    # Scale 8-bit values to realistic depth range (0-5m)
                    # Darker pixels = farther, lighter pixels = closer (inverse of typical depth)
                    depth_image = (255 - depth_image.astype(np.float32)) / 255.0 * 5  # 0-5 meters
                    depth_image = (depth_image * 1000).astype(np.uint16)  # Convert to mm as uint16
                else:
                    # Convert to uint16 format for depth processing
                    depth_image = depth_image.astype(np.uint16)
                
            return rgb_image, depth_image
        elif success_rgb:
            # Fallback: Get CSI_FRONT image and simulate depth
            success_csi, csi_image = self.qcar.get_image(camera=self.qcar.CAMERA_CSI_FRONT)
            if success_csi:
                # Convert CSI to simulated depth with improved scaling
                gray_csi = cv2.cvtColor(csi_image, cv2.COLOR_BGR2GRAY)
                
                # Resize to match RGB image dimensions if needed
                target_height, target_width = rgb_image.shape[:2]
                if gray_csi.shape != (target_height, target_width):
                    gray_csi = cv2.resize(gray_csi, (target_width, target_height), interpolation=cv2.INTER_LINEAR)
                
                # Simulate depth based on intensity (brighter = closer)
                # Use more realistic depth range and scaling
                depth_normalized = (255 - gray_csi.astype(np.float32)) / 255.0
                depth_image = (depth_normalized * 5.0 * 1000).astype(np.uint16)  # 0-5m in mm
                return rgb_image, depth_image
        
        return None, None
    
    def get_intrinsics(self):
        """Get camera intrinsic parameters"""
        return self.intrinsics
    
    def extract_depth_from_bbox(self, detection, depth_image, verbose=False):
        """
        Extract depth information from detection bounding box using statistical methods
        
        Args:
            detection: Detection object with bbox information
            depth_image: Depth image (numpy array)
            verbose: Enable debug output
            
        Returns:
            Average depth value in meters (float) or None if failed
        """
        if depth_image is None:
            return None
            
        try:
            # Get bounding box coordinates
            if hasattr(detection, 'bbox') and detection.bbox:
                bbox = detection.bbox
                x1, y1, x2, y2 = bbox
            elif hasattr(detection, 'center_x') and hasattr(detection, 'width'):
                # Calculate bbox from center and dimensions
                center_x = detection.center_x
                center_y = detection.center_y
                width = detection.width
                height = detection.height
                x1 = int(center_x - width/2)
                y1 = int(center_y - height/2)
                x2 = int(center_x + width/2)
                y2 = int(center_y + height/2)
            else:
                if verbose:
                    print("No valid bbox information in detection")
                return None
            
            # Ensure coordinates are within image bounds
            height, width = depth_image.shape[:2]
            x1 = max(0, min(x1, width-1))
            y1 = max(0, min(y1, height-1))
            x2 = max(x1+1, min(x2, width))
            y2 = max(y1+1, min(y2, height))
            
            # Extract depth values from bounding box region
            depth_roi = depth_image[y1:y2, x1:x2]
            
            # Convert depth values to meters (assuming depth is in mm)
            if depth_roi.dtype == np.uint16:
                depth_roi_meters = depth_roi.astype(np.float32) / 1000.0
            else:
                depth_roi_meters = depth_roi.astype(np.float32)
            
            # Filter out invalid depth values (0 or very large values)
            valid_depths = depth_roi_meters[(depth_roi_meters > 0.1) & (depth_roi_meters < 50.0)]
            
            if len(valid_depths) == 0:
                if verbose:
                    print("No valid depth values in bounding box")
                return None
            
            # Calculate statistics
            mean_depth = np.mean(valid_depths)
            median_depth = np.median(valid_depths)
            std_depth = np.std(valid_depths)
            
            # Use median for better robustness against outliers
            # But fall back to mean if standard deviation is too high
            if std_depth < mean_depth * 0.3:  # If std is less than 30% of mean
                final_depth = mean_depth
            else:
                final_depth = median_depth
                
            if verbose:
                print(f"Depth extraction: mean={mean_depth:.3f}m, median={median_depth:.3f}m, std={std_depth:.3f}m")
                print(f"Final depth: {final_depth:.3f}m from {len(valid_depths)} valid pixels")
            
            return float(final_depth)
            
        except Exception as e:
            if verbose:
                print(f"Error extracting depth from bbox: {e}")
            return None

