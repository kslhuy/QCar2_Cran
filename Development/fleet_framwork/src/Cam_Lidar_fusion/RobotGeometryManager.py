import numpy as np
import math
import cv2
from pytransform3d import rotations as pr
try:
    import qcar_utils_HUY
except ImportError:
    # Try relative import if direct import fails
    try:
        from . import qcar_utils_HUY
    except ImportError:
        print("Warning: Could not import qcar_utils_HUY in RobotGeometryManager")
        qcar_utils_HUY = None

from hal.utilities.geometry import Geometry, MobileRobotGeometry
class RobotGeometryManager:
    """
    Manages coordinate frame transformations for the QCar robot using the geometry class
    """
    def __init__(self,is_physical=False):
        # Initialize geometry manager
        self.geometry = MobileRobotGeometry()
        # Camera parameters
        self.camera_fov_h = math.radians(68)  # Horizontal FOV
        self.camera_fov_v = math.radians(42.5)  # Vertical FOV
        
        
        self.fx = 640 / (2 * np.tan(self.camera_fov_h / 2))
        self.fy = 480 / (2 * np.tan(self.camera_fov_v / 2))
        
        # self.fx = 616
        # self.fy = 616    

        self.cx = 320.0
        self.cy = 240.0

        
        self.scale = 0.1 if is_physical else 1.0  # Scale for physical QCar
        self.max_distance_lidar = 40.0*self.scale  # Max LiDAR range in meters
        self.min_distance_lidar = 0.1*self.scale  # Min LiDAR range in meters
        # Define coordinate frames
        self.setup_coordinate_frames()

        print("Robot Geometry Manager initialized with coordinate frames")
    
    def setup_coordinate_frames(self):
        """Set up all coordinate frames relative to the robot body.
        
        Uses QCar 2 extrinsic matrices from PDF (p. 8-9) for RealSense camera and RPLIDAR.
        - Transformations are T_S^B: p_body = T_S^B * p_sensor (homogeneous).
        - For virtual sim: positions are 10x physical (self.scale=1.0 uses full virtual units).
        - Chaining: get_transform('lidar', 'camera') = T_camera^lidar = inv(T_camera^B) * T_lidar^B
          (transforms p_lidar to p_camera).
        """
        
        # RealSense camera frame {C} to body {B} (from PDF: realsense_to_body T_C^B)
        # Position (10x for virtual): [0.095, 0.032, 0.172] -> [0.95, 0.32, 1.72]
        # Rotation R_C^B (camera Z->body X, camera X->body -Y, camera Y->body -Z)
        camera_position = np.array([0.95 * self.scale, 0.32 * self.scale, 1.72 * self.scale])
        camera_rotation = np.array([
            [0, 0, 1],    # Camera Z -> Body X (forward)
            [-1, 0, 0],   # Camera X -> Body -Y (right in body: +Y is left, so -Y is right)
            [0, -1, 0]    # Camera Y -> Body -Z (down)
        ])
        self.geometry.add_frame(
            name='camera', 
            p=camera_position, 
            R=camera_rotation, 
            base='body'
        )
        
        # RPLIDAR frame {L} to body {B} (from PDF: rplidar_to_body T_L^B)
        # Position (10x for virtual): [-0.012, 0, 0.193] -> [-0.12, 0, 1.93]
        # FIXED: Rotation R_L^B (180° yaw flip: LiDAR X-> -Body X, Y-> -Body Y, Z-> Body Z)
        # This accounts for LiDAR mounting orientation (scans in XY plane, facing forward but frame flipped)
        lidar_position = np.array([-0.12 * self.scale, 0.0 * self.scale, 1.93 * self.scale])
        lidar_rotation = np.array([
            [-1, 0, 0],   # LiDAR X -> Body -X (backward in body)
            [0, -1, 0],   # LiDAR Y -> Body -Y (right in body)
            [0, 0, 1]     # LiDAR Z -> Body Z (up)
        ])

        # lidar_rotation = np.eye(3)  # Same orientation as body

        self.geometry.add_frame(
            name='lidar', 
            p=lidar_position, 
            R=lidar_rotation, 
            base='body'
        )
        
        # print("Coordinate frames set up:")
        # print(f"  - world (default frame)")
        # print(f"  - body (robot center)")
        # print(f"  - camera (RealSense D435 forward-looking)")
        # print(f"  - lidar (RPLIDAR A2M12 top-mounted)")
        
        # Verify coordinate frame setup
        # self.verify_coordinate_frames()
    
    def verify_coordinate_frames(self):
        """Verify that coordinate frames are set up correctly"""
        print("\n=== Coordinate Frame Verification ===")
        

 
    
    def adjust_camera_rotation(self, yaw_adjustment_rad):
        """Adjust camera rotation based on calibration results"""
        print(f"Adjusting camera yaw by {math.degrees(yaw_adjustment_rad):.1f}°")
        
        # Get current camera rotation
        current_rotation = self.geometry.get_rotation_rm('camera', 'body')
        
        # Create yaw adjustment matrix
        yaw_adjustment = pr.active_matrix_from_extrinsic_euler_xyz([0, 0, yaw_adjustment_rad])
        
        # Apply adjustment
        new_rotation = current_rotation @ yaw_adjustment
        self.geometry.set_rotation_rm(new_rotation, 'camera', 'body')
        
        print("Camera rotation adjusted!")
        self.verify_coordinate_frames()


    def lidar_to_camera_projection_loop(self, angles, distances, image_width=640, image_height=480):
        """
        Project LiDAR points to camera image coordinates
        
        Args:
            angles: LiDAR scan angles (radians)
            distances: LiDAR scan distances (meters)  
            image_width: Camera image width
            image_height: Camera image height
            
        Returns:
            projected_points: List of (u, v, depth) tuples for valid projections
            lidar_points_3d: 3D points in camera frame
        """
        projected_points = []
        lidar_points_3d = []
        
        # Convert LiDAR scan to 3D points in LiDAR frame
        # FIXED: QCar LiDAR convention - angle 0 is forward, positive angles go counter-clockwise
        # Standard LiDAR coordinate system: X=forward, Y=left, Z=up
        lidar_x = distances * np.cos(angles)  # X=forward (distance * cos(angle))
        lidar_y = -distances * np.sin(angles)  # Y=left (distance * sin(angle))
        lidar_z = np.zeros_like(distances)    # Z=up (planar scan at height=0)
        
        # Transform LiDAR points to camera frame
        for i in range(len(angles)):
            if distances[i] > self.min_distance_lidar and distances[i] < self.max_distance_lidar:  # Valid range
                # Point in LiDAR frame
                point_lidar = np.array([lidar_x[i], lidar_y[i], lidar_z[i], 1.0])
                
                # Transform to camera frame
                T_camera_lidar = self.geometry.get_transform('lidar', 'camera')
                point_camera = T_camera_lidar @ point_lidar
                
                # Extract 3D coordinates in camera frame
                x_cam, y_cam, z_cam = point_camera[0], point_camera[1], point_camera[2]
                
                # Check if point is in front of camera
                if z_cam > self.min_distance_lidar:  # At least 1cm in front
                    # Project to image coordinates using pinhole camera model
                    # Assuming camera intrinsic matrix
                    fx = self.fx
                    fy = self.fy
                    cx = self.cx
                    cy = self.cy

                    u = (fx * x_cam / z_cam + cx)
                    v = (fy * y_cam / z_cam + cy)
                    
                    # Check if projection is within image bounds
                    if 0 <= u < image_width and 0 <= v < image_height:
                        projected_points.append((u, v, z_cam))
                        lidar_points_3d.append((x_cam, y_cam, z_cam))
        
        return projected_points, lidar_points_3d

    def lidar_to_camera_projection(self, angles, distances, image_width=640, image_height=480, segmentation_masks=None , erosion_kernel_size = 1, shrink_factor_param=-0.2, detection_objects=None):
        """
        Project LiDAR points to camera image coordinates using extrinsics and fixed intrinsics.
        OPTIMIZED: Now supports pre-filtering with segmentation masks for efficiency.
        
        Args:
            angles: LiDAR scan angles (radians; 0=forward, + CCW from RPLIDAR SDK)
            distances: LiDAR scan distances (meters)
            image_width, image_height: Camera image dimensions (overrides for cx/cy if needed)
            segmentation_masks: List of segmentation masks from YOLO for efficient filtering (optional)
            detection_objects: List of detection objects for bounding box fallback (optional)
            
        Returns:
            projected_points: Nx3 array [[u, v, depth], ...] for valid projections (filtered by masks if provided)
            lidar_points_3d: Nx3 array [[x_cam, y_cam, z_cam], ...] (filtered by masks if provided) : coordinates in camera frame
        """
        # Update centers if res changes
        cx, cy = image_width / 2, image_height / 2
        
        # Filter valid scans once
        valid_mask = (distances > self.min_distance_lidar) & (distances < self.max_distance_lidar)
        if not np.any(valid_mask):
            return np.empty((0, 3)), np.empty((0, 3)) , np.empty((0, 3))
        
        angles = angles[valid_mask]
        distances = distances[valid_mask]
        
        # 3D points in LiDAR frame 
        lidar_x = - distances * np.cos(angles)  # X=forward (distance * cos(angle))
        lidar_y = distances * np.sin(angles)  # Y=left (distance * sin(angle))
        lidar_z = np.zeros_like(distances)    # Z=up (planar scan at height=0)

        lidar_3d_in_lidar = np.column_stack([lidar_x, lidar_y, lidar_z])  # Nx3
        
        # Stack homogeneous points: Nx4
        points_lidar_hom = np.column_stack([lidar_x, lidar_y, lidar_z, np.ones_like(distances)])
        
        # Transform batch: p_camera = T_camera^lidar @ points_lidar_hom.T  (4xN)
        T_camera_lidar = self.geometry.get_transform('lidar' , 'camera' )  # Assumes p_camera = T @ p_lidar
        points_camera_hom = T_camera_lidar @ points_lidar_hom.T
        points_camera_3d = points_camera_hom[:3].T  # Nx3 [x,y,z]

        # Filter front-facing (z_cam > 0.05)
        front_mask = points_camera_3d[:, 2] > 0.05
        if not np.any(front_mask):
            return np.empty((0, 3)), np.empty((0, 3))
        
        lidar_3d_in_lidar = lidar_3d_in_lidar[front_mask]
        lidar_points_3d_in_cam = points_camera_3d[front_mask]
        
        # Vectorized pinhole projection
        u = (self.fx * (lidar_points_3d_in_cam[:, 0] / lidar_points_3d_in_cam[:, 2]) + cx)
        v = (self.fy * (lidar_points_3d_in_cam[:, 1] / lidar_points_3d_in_cam[:, 2]) + cy)
        depths = lidar_points_3d_in_cam[:, 2]

        # Convert to integers with rounding (more accurate)
        u = np.round(u).astype(int)
        v = np.round(v).astype(int)


        projected_points = np.column_stack([u, v, depths])
        # Bounds check
        in_bounds = (0 <= u) & (u < image_width) & (0 <= v) & (v < image_height) 
        valid_proj = in_bounds
        
        # OPTIMIZED: Apply segmentation mask filtering if provided (not use)
        if segmentation_masks is not None and len(segmentation_masks) > 0:
            mask_filter = self._apply_segmentation_mask_filter(u, v, segmentation_masks, image_height, image_width, detection_objects , erosion_kernel_size=erosion_kernel_size, shrink_factor=shrink_factor_param)
            valid_proj = valid_proj & mask_filter
        
        if np.any(valid_proj):
            projected_points = np.column_stack([u[valid_proj], v[valid_proj], depths[valid_proj]])
            lidar_points_3d_in_cam = lidar_points_3d_in_cam[valid_proj]
            lidar_3d_in_lidar = lidar_3d_in_lidar[valid_proj]
        else:
            projected_points = np.empty((0, 3))
            lidar_points_3d_in_cam = np.empty((0, 3))
            lidar_3d_in_lidar = np.empty((0, 3))

        return projected_points, lidar_points_3d_in_cam , lidar_3d_in_lidar
    
    def project_semantic_objects_to_lidar(self, detection_objects, image_width=640, image_height=480):
        """
        Project semantic objects from RealSense camera to RPLIDAR coordinate frame using extrinsics.
        
        Full 3D pinhole projection + matrix chaining. Assumes distance is depth along ray.
        Outputs lidar_x/y for plot: x_plot = p_L[1] (lateral, +left?), y_plot = -p_L[0] (+forward).
        """
        projected_objects = []
        
        # Validate inputs
        if not detection_objects:
            return projected_objects
        
        # Use class camera intrinsics (consistent with lidar_to_camera_projection)
        cx = self.cx 
        cy = self.cy 
        fx = self.fx 
        fy = self.fy 
        
        # Get transformation matrix: p_lidar = T_lidar^camera @ p_camera
        # to Lidar from Camera
        T_lidar_camera = self.geometry.get_transform('camera','lidar')  # 4x4 homogeneous
        
        for detection in detection_objects:
            try:
                if not (hasattr(detection, 'distance') and detection.distance > 0):
                    continue
                    
                # Validate detection has required attributes
                if not all(hasattr(detection, attr) for attr in ['center_x', 'center_y', 'bbox']):
                    continue
                    
                # Pixel coordinates
                center_u = detection.center_x
                center_v = detection.center_y
                dist = detection.distance
                
                # Validate pixel coordinates are within image bounds
                if not (0 <= center_u < image_width and 0 <= center_v < image_height):
                    continue
                
                # 3D point in camera frame {C}: p_c = dist * K^{-1} * [u, v, 1]^T
                # CORRECTED: Use consistent intrinsics (matches lidar_to_camera_projection)
                x_c = (center_u - cx) * dist / fx
                y_c = (center_v - cy) * dist / fy
                z_c = dist
                p_c_hom = np.array([x_c, y_c, z_c, 1.0])
                
                # Transform to LiDAR frame {L}
                p_l_hom = T_lidar_camera @ p_c_hom
                p_l = p_l_hom[:3]  # [x_l, y_l, z_l]
            

                
                # CORRECTED: Match lidar_to_camera_projection exactly
                lidar_x = p_l[0]   
                lidar_y = p_l[1]   
                lidar_z = p_l[2]   # Height (for debug)
                
                # Detection dict (keep originals + projected)
                corrected_detection = {
                    'bbox': detection.bbox if hasattr(detection, 'bbox') else None,
                    'center_x': detection.center_x,
                    'center_y': detection.center_y,
                    'width': detection.width if hasattr(detection, 'width') else 0,
                    'height': detection.height if hasattr(detection, 'height') else 0,
                    'area': detection.area if hasattr(detection, 'area') else 0,
                    'confidence': detection.conf if hasattr(detection, 'conf') else 0,
                    'class_id': detection.class_id if hasattr(detection, 'class_id') else -1,
                    'class_name': detection.name if hasattr(detection, 'name') else 'unknown',
                    'distance': detection.distance,
                    'lidar_x': lidar_x,
                    'lidar_y': lidar_y,
                    'lidar_z': lidar_z
                }
                
                # FIXED: Compute polar coordinates properly
                # In LiDAR frame: angle 0 = +X (forward), positive angle = +Y (left)
                # lidar_angle = math.atan2(p_l[1], p_l[0])  # Matches projection convention exactly
                # lidar_dist = np.linalg.norm(p_l[:2])
                lidar_angle = np.arctan2(lidar_y, lidar_x)  # Angle from +X axis
                lidar_dist = np.sqrt(lidar_x**2 + lidar_y**2)  # Radial distance
                # lidar_dist  = np.hypot(p_l[0], p_l[1])

                corrected_detection['lidar_angle_rad'] = lidar_angle
                corrected_detection['lidar_angle_deg'] = math.degrees(lidar_angle)
                corrected_detection['lidar_dist'] = lidar_dist
                
                # Traffic light if available
                if hasattr(detection, 'lightColor'):
                    corrected_detection['traffic_light_color'] = detection.lightColor
                    
                projected_objects.append(corrected_detection)
                
            except Exception as e:
                print(f"Warning: Error projecting detection to LiDAR: {e}")
                continue
    
        return projected_objects
    
   
    

    def draw_lidar_on_camera_image_bbox(self, image, projected_points, lidar_points_3d, detection_objects=None, proximity_threshold=50 , shrink_factor=-0.2):
        """
        Draw only LiDAR points that are close to detected object bounding boxes
        
        Args:
            image: Camera image (numpy array)
            projected_points: List of (u, v, depth) tuples
            lidar_points_3d: 3D points in camera frame
            detection_objects: List of detection objects with bounding boxes (optional)
            proximity_threshold: Distance threshold in pixels for LiDAR points to be considered near objects
        
        Returns:
            Annotated image with filtered LiDAR points near detected objects
        """
        annotated_image = image.copy()
        image_height, image_width = image.shape[:2]
        
        # If no detection objects provided, draw all LiDAR points (fallback to original behavior)
        if detection_objects is None or len(detection_objects) == 0:
            for (u, v, depth), (x, y, z) in zip(projected_points, lidar_points_3d):
                # Color based on distance (closer = red, farther = blue)
                max_distance = 20.0
                distance_ratio = min(depth / max_distance, 1.0)
                
                # Create color gradient: close=red(255,0,0), far=blue(0,0,255)
                color_r = int(255 * (1 - distance_ratio))
                color_b = int(255 * distance_ratio)
                color = (color_b, 0, color_r)  # BGR format for OpenCV
                
                cv2.circle(annotated_image, (int(u), int(v)), 4, color, -1)
                cv2.circle(annotated_image, (int(u), int(v)), 4, (255, 255, 255), 1)
            return annotated_image
        
        # Filter LiDAR points based on proximity to detected objects
        for (u, v, depth), (x, y, z) in zip(projected_points, lidar_points_3d):
            # Check if this LiDAR point is near any detected object
            point_near_object = False
            
            for detection in detection_objects:
                # Check if point is within the bounding box (with some expansion)
                if qcar_utils_HUY.rect_contains_point(detection, (u, v), image_width, image_height, shrink_factor=shrink_factor):  # Negative shrink = expand
                    point_near_object = True
                    break
                
                # # Alternative: Check distance to bbox center for proximity
                # if hasattr(detection, 'center_x') and hasattr(detection, 'center_y'):
                #     center_x = detection.center_x
                #     center_y = detection.center_y
                # elif 'center_x' in detection and 'center_y' in detection:
                #     center_x = detection['center_x']
                #     center_y = detection['center_y']
                # else:
                #     # Try to get center from bbox if available
                #     if hasattr(detection, 'bbox') and len(detection.bbox) >= 4:
                #         bbox = detection.bbox
                #         center_x = (bbox[0] + bbox[2]) / 2
                #         center_y = (bbox[1] + bbox[3]) / 2
                #     elif 'bbox' in detection and len(detection['bbox']) >= 4:
                #         bbox = detection['bbox']
                #         center_x = (bbox[0] + bbox[2]) / 2
                #         center_y = (bbox[1] + bbox[3]) / 2
                #     else:
                #         continue
                
                # # Calculate distance from LiDAR point to object center
                # distance_to_center = ((u - center_x)**2 + (v - center_y)**2)**0.5
                
                # if distance_to_center <= proximity_threshold:
                #     point_near_object = True
                #     break
            
            # Only draw LiDAR points that are near detected objects
            if point_near_object:
                # Color based on distance (closer = red, farther = blue)
                max_distance = 20.0
                distance_ratio = min(depth / max_distance, 1.0)
                
                # Create color gradient: close=red(255,0,0), far=blue(0,0,255)
                color_r = int(255 * (1 - distance_ratio))
                color_b = int(255 * distance_ratio)
                color = (color_b, 0, color_r)  # BGR format for OpenCV
                
                # Draw point with slightly larger size to make them more visible - FIXED: Convert coordinates to integers
                cv2.circle(annotated_image, (int(u), int(v)), 4, color, -1)
                # Add small white border for better visibility - FIXED: Convert coordinates to integers
                cv2.circle(annotated_image, (int(u), int(v)), 4, (255, 255, 255), 1)
        
        return annotated_image

    def draw_lidar_on_camera_image(self, image, projected_points, detection_objects=None, segmentation_masks=None, erosion_kernel_size=3 , shrink_factor=-0.2):
        """
        Draw only LiDAR points that are inside eroded segmentation masks
        
        Args:
            image: Camera image (numpy array)
            projected_points: List or Nx3 array of (u, v, depth)
            lidar_points_3d: List or Nx3 array of (x, y, z) in camera frame
            detection_objects: List of detection objects with bounding boxes (optional, for fallback)
            segmentation_masks: List of segmentation masks corresponding to detection objects
            erosion_kernel_size: Size of erosion kernel to refine mask boundaries (default=3)
        
        Returns:
            Annotated image with filtered LiDAR points inside eroded segmentation masks
        """
        annotated_image = image
        # image_height, image_width = image.shape[:2]
        # crop_height = 40  # Number of pixels to crop from bottom
        # height = annotated_image.shape[0]
        # annotated_image = annotated_image[0:height-crop_height, :]  # Remove bottom 40 pixels and use cropped image


        # Fallback: Draw all if no detections
        # if detection_objects is None or len(detection_objects) == 0:
        for (u, v, depth) in projected_points:
            pixel_u, pixel_v = int(round(u)), int(round(v))  # Round to nearest pixel
            color = self._get_distance_color(depth)  # Extracted for reuse

            # Draw filled colored circle + white border
            cv2.circle(annotated_image, (pixel_u, pixel_v), 3, color, -1)
            cv2.circle(annotated_image, (pixel_u, pixel_v), 4, (255, 255, 255), 1)
        return annotated_image
        


    def _get_distance_color(self, depth, max_distance=20.0):
        """Helper for distance-based color (BGR: close=red, far=blue)"""
        ratio = min(depth / max_distance, 1.0)
        r = int(255 * (1 - ratio))
        b = int(255 * ratio)
        return (b, 0, r)  # BGR

    def _apply_segmentation_mask_filter(self, u, v, segmentation_masks, image_height, image_width, detection_objects=None, erosion_kernel_size=1, shrink_factor=-0.2):
        """
        Apply segmentation mask filtering to projected LiDAR points using the same logic as draw_lidar_on_camera_image()
        Now includes erosion and proper coordinate handling for improved filtering accuracy.
        
        Args:
            u, v: Projected LiDAR pixel coordinates
            segmentation_masks: List of segmentation masks from YOLO
            image_height, image_width: Image dimensions
            detection_objects: List of detection objects for bounding box fallback (optional)
            erosion_kernel_size: Size of erosion kernel to refine mask boundaries (default=3)
            
        Returns:
            mask_filter: Boolean array indicating which points are within segmentation masks
        """
        if not segmentation_masks or len(segmentation_masks) == 0:
            return np.ones(len(u), dtype=bool)  # Return all points if no masks
        
        # Initialize filter (start with all False, add True for points in any mask)
        mask_filter = np.zeros(len(u), dtype=bool)
        
        try:
            # Pre-process masks with erosion (same as draw_lidar_on_camera_image)
            eroded_masks = []
            erosion_kernel = np.ones((erosion_kernel_size, erosion_kernel_size), np.uint8)
            
            for mask in segmentation_masks:
                try:
                    mask_np = mask # Already numpy array
                    # Binary + erode (same as drawing function)
                    binary_mask = (mask_np > 0.5).astype(np.uint8)
                    eroded = cv2.erode(binary_mask, erosion_kernel, iterations=1)
                    eroded_masks.append(eroded)
                    
                except Exception as e:
                    print(f"Warning: Error processing segmentation mask: {e}")
                    continue
            
            # Check each LiDAR point (same coordinate handling as drawing function)
            for i in range(len(u)):
                point_inside_mask = False
                pixel_u, pixel_v = int(round(u[i])), int(round(v[i]))  # Same rounding as drawing function
                
                if 0 <= pixel_u < image_width and 0 <= pixel_v < image_height:
                    # Check eroded masks first
                    if eroded_masks:
                        if any(eroded[pixel_v, pixel_u] > 0 for eroded in eroded_masks):
                            point_inside_mask = True
                    # Fallback to bbox if available (same as drawing function)
                    elif detection_objects is not None:
                        for detection in detection_objects:
                            if detection is not None and hasattr(detection, 'bbox'):  # Safety check
                                if qcar_utils_HUY.rect_contains_point(detection, (u[i], v[i]), image_width, image_height, shrink_factor=shrink_factor):
                                    point_inside_mask = True
                                    break
                
                mask_filter[i] = point_inside_mask
            
        except Exception as e:
            print(f"Warning: Error in segmentation mask filtering: {e}")
            # Return all points if filtering fails
            return np.ones(len(u), dtype=bool)
        
        return mask_filter
    




    def debug_coordinate_transformation(self, pixel_x, pixel_y, distance, image_width=640, image_height=480):
        """
        Debug function to test coordinate transformations
        
        Args:
            pixel_x, pixel_y: Pixel coordinates
            distance: Distance to object
            image_width, image_height: Image dimensions
        """
        print(f"\n=== DEBUG: Coordinate Transformation ===")
        print(f"Input: pixel({pixel_x}, {pixel_y}), distance={distance}m")
        
        # Pixel angle calculation (matches camera FOV)
        pixel_angle = (pixel_x - image_width/2) / (image_width/2) * (self.camera_fov_h/2)
        
        # LiDAR coordinates (matches visualization system)
        lidar_x = np.sin(pixel_angle) * distance  # LEFT-RIGHT
        lidar_y = np.cos(pixel_angle) * distance  # FORWARD-BACKWARD
        
        print(f"Pixel angle: {math.degrees(pixel_angle):.1f}°")
        print(f"LiDAR coordinates: X={lidar_x:.2f} (left-right), Y={lidar_y:.2f} (forward-backward)")
        
        # Expected behavior guide
        print(f"\nExpected behavior:")
        print(f"- Center image (320,240): angle≈0°, X≈0, Y≈{distance}")
        print(f"- Right side (480,240): angle>0°, X>0 (object to right in plot)")
        print(f"- Left side (160,240): angle<0°, X<0 (object to left in plot)")
        print(f"- All objects at same distance should have Y≈{distance}")
        
        # Show how it appears in LiDAR visualization
        print(f"\nIn LiDAR plot:")
        print(f"- This object will appear at plot coordinates ({lidar_x:.2f}, {lidar_y:.2f})")
        print(f"- Plot X-axis = left(-) to right(+)")
        print(f"- Plot Y-axis = backward(-) to forward(+)")
        print("=== End Debug ===\n")
    
    def update_robot_pose(self, x, y, theta):
        """Update robot pose in world frame"""
        self.geometry.set_pose_2d([x, y, theta], frame='body', base='world')


    ## --- Not used currently, but useful for testing ---
    ################################################################################
    def validate_lidar_camera_consistency(self, test_angles=None, test_distances=None, verbose=True):
        """
        Validate consistency between lidar_to_camera_projection and project_semantic_objects_to_lidar
        
        Args:
            test_angles: Array of test LiDAR angles (default: forward, left, right)
            test_distances: Array of test distances (default: [5.0, 10.0, 15.0])
            verbose: Enable detailed output
            
        Returns:
            bool: True if transformations are consistent
        """
        if test_angles is None:
            test_angles = np.array([0.0, np.pi/4, -np.pi/4, np.pi/2, -np.pi/2])  # 0°, 45°, -45°, 90°, -90°
        if test_distances is None:
            test_distances = np.array([5.0, 10.0, 15.0])
            
        if verbose:
            print("\n=== LiDAR-Camera Transformation Consistency Test ===")
            
        all_consistent = True
        
        for dist in test_distances:
            distances = np.full_like(test_angles, dist)
            
            # Step 1: LiDAR -> Camera projection
            projected_points, lidar_points_3d = self.lidar_to_camera_projection(
                test_angles, distances, image_width=640, image_height=480
            )
            
            if len(projected_points) == 0:
                if verbose:
                    print(f"Distance {dist}m: No valid projections")
                continue
                
            # Step 2: Create mock detections at projected points
            mock_detections = []
            for i, (u, v, depth) in enumerate(projected_points):
                mock_detection = type('MockDetection', (), {
                    'center_x': u,
                    'center_y': v,
                    'distance': depth,
                    'bbox': [u-10, v-10, u+10, v+10],
                    'width': 20,
                    'height': 20,
                    'area': 400,
                    'conf': 0.9,
                    'class_id': 0,
                    'name': f'test_{i}'
                })()
                mock_detections.append(mock_detection)
            
            # Step 3: Camera -> LiDAR projection (inverse)
            reprojected = self.project_semantic_objects_to_lidar(mock_detections)
            
            # Step 4: Compare original vs reprojected
            if verbose:
                print(f"\nDistance {dist}m - Consistency Check:")
                
            for i, (angle, orig_dist) in enumerate(zip(test_angles, distances)):
                if i >= len(reprojected):
                    continue
                    
                reproj = reprojected[i]
                
                # Original LiDAR coordinates (from lidar_to_camera_projection logic)
                orig_x = -orig_dist * np.cos(angle)  # Forward = -x
                orig_y = -orig_dist * np.sin(angle)  # Left = -y
                
                # Compare with reprojected
                reproj_x_raw = reproj['lidar_x']  # This should match orig_y (lidar_y from projection)
                reproj_y_raw = reproj['lidar_y']  # This should match orig_x (lidar_x from projection)
                
                # Check consistency: 
                # reproj_x should match orig_y (both are lateral/lidar_y)
                # reproj_y should match orig_x (both are forward/lidar_x)
                x_error = abs(reproj_x_raw - orig_y)  # lateral error
                y_error = abs(reproj_y_raw - orig_x)  # forward error
                dist_error = abs(reproj['distance'] - orig_dist)
                
                # More lenient distance check due to sensor offset translations
                # The camera and LiDAR are physically separated, so depth will differ slightly
                is_consistent = (x_error < 0.1 and y_error < 0.1 and dist_error < 0.5)  # Allow 50cm distance error
                all_consistent = all_consistent and is_consistent
                
                if verbose:
                    status = "✓" if is_consistent else "✗"
                    print(f"  {status} Angle {math.degrees(angle):6.1f}°:")
                    print(f"     Original LiDAR: ({orig_x:6.2f}, {orig_y:6.2f}) [lidar_x, lidar_y]")
                    print(f"     Reprojected:    ({reproj_x_raw:6.2f}, {reproj_y_raw:6.2f}) [lidar_x, lidar_y]")
                    print(f"     Should match:   ({orig_y:6.2f}, {orig_x:6.2f}) [Y->X, X->Y]")
                    print(f"     Errors: lateral={x_error:.3f}, forward={y_error:.3f}, dist={dist_error:.3f}")
        
        if verbose:
            result = "CONSISTENT" if all_consistent else "INCONSISTENT"
            print(f"\n=== Result: Transformations are {result} ===\n")
            
        return all_consistent
