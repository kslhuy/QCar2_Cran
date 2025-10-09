"""
Camera-LiDAR Fusion Component for Vehicle Relative Distance Measurement

This component provides relative distance measurements to other vehicles using
camera-based object detection fused with LiDAR data. It's designed as a modular
component that can be integrated into vehicle control systems.

Key Features:
- YOLO-based vehicle detection from camera images
- LiDAR point cloud fusion for accurate distance measurement
- Coordinate transformation handling for proper geometric alignment
- Real-time visualization capabilities (similar to qcar2_tuto.py)
- Optimized for real-time automotive applications

Visualization Features:
- OpenCV camera feed with LiDAR overlay
- PyQtGraph LiDAR point cloud visualization
- Real-time detection overlays with distance/confidence info
- Keyboard controls: 'l' (toggle LiDAR overlay), 'p' (toggle LiDAR plot), 'q' (quit)

Usage Example:
    # Initialize with visualization enabled
    fusion = CamLidarFusion(qcar, vehicle_id=0, enable_visualization=True)

    # In main loop
    while True:
        success = fusion.update_sensor_data()
        if success:
            distances = fusion.get_relative_distances()
            # Visualization updates automatically if enabled

    # Or manually show current image
    fusion.show_image()
"""

import numpy as np
import cv2
import math
import time
from typing import List, Dict, Optional, Tuple, Any
import sys
import os
import logging

# Add visualization imports
try:
    VISUALIZATION_AVAILABLE = True
except ImportError:
    VISUALIZATION_AVAILABLE = False
    print("Warning: OpenCV not available, visualization features disabled")

# Add the src/Cam_Lidar_fusion directory to Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
fusion_dir = os.path.join(current_dir, 'src', 'Cam_Lidar_fusion')
if fusion_dir not in sys.path:
    sys.path.insert(0, fusion_dir)

# Import fusion dependencies
try:
    from src.Cam_Lidar_fusion.qcar_utils_HUY import *
    from src.Cam_Lidar_fusion.RobotGeometryManager import RobotGeometryManager
    from src.Cam_Lidar_fusion.YOLOv8Wrapper_Huy import YOLOv8Wrapper_Huy
    from src.Cam_Lidar_fusion.RealSenseWrapper import RealSenseWrapper
    from src.Cam_Lidar_fusion.LiDARWrapper import LiDARWrapper
    DEPENDENCIES_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Failed to import fusion dependencies: {e}")
    DEPENDENCIES_AVAILABLE = False


class CamLidarFusion:
    """
    Camera-LiDAR fusion component for relative vehicle distance measurement.

    This component integrates camera-based object detection with LiDAR distance
    measurements to provide accurate relative positioning of other vehicles.
    """

    def __init__(self, qcar , vehicle_id: int, image_width: int = 640, image_height: int = 480,
                 scale_factor: float = 0.1, verbose: bool = False, logger: logging.Logger = None,
                 enable_visualization_camera: bool = False, show_lidar_plot: bool = True):
        """
        Initialize the camera-LiDAR fusion component.

        Args:
            vehicle_id: ID of the vehicle this component belongs to
            image_width: Camera image width
            image_height: Camera image height
            scale_factor: Scale factor for coordinate transformations (0.1 for physical)
            verbose: Enable verbose debug output
            logger: Optional logger instance
            enable_visualization: Enable visualization windows (requires pyqtgraph and cv2)
            show_lidar_overlay: Show LiDAR points overlay on camera image
        """
        self.vehicle_id = vehicle_id
        self.image_width = image_width
        self.image_height = image_height
        self.scale_factor = scale_factor
        self.verbose = verbose
        self.logger = logger or logging.getLogger(f'CamLidarFusion_V{vehicle_id}')

        # Visualization settings
        self.enable_visualization = enable_visualization_camera  # OpenCV always available
        self.show_lidar_plot = show_lidar_plot  # Show LiDAR visualization window

        # LiDAR visualization parameters
        self.lidar_vis_size = 400  # Size of LiDAR visualization window (pixels)
        self.lidar_vis_scale = 30  # Pixels per meter for LiDAR visualization (increased for better zoom)
        self.lidar_vis_max_range = 15  # Maximum range to display (meters) - reduced for better focus

        # Component status
        self.initialized = False
        self.last_update_time = 0.0

        # Initialize components
        self.geometry_manager = None
        self.yolo_detector = None
        self.realsense_wrapper = None
        self.lidar_wrapper = None

        # Fusion parameters
        self.shrink_factor = -0.9  # Bounding box shrink factor for LiDAR filtering
        self.erosion_kernel_size = 1  # Erosion kernel size for mask cleaning

        self.dilate_kernel_size_param = 10  # Dilation kernel size for segmentation mask cleaning
        self.iterations_param = 1  # Morphological operation iterations

        # Results storage
        self.last_relative_distances = {}

        # Performance tracking
        self.update_count = 0
        self.avg_processing_time = 0.0

        # Visualization components
        self.lidar_app = None
        self.lidar_plot = None
        self.lidar_data_plot = None
        self.semantic_objects_plot = None
        self.last_rgb_image = None
        self.last_projected_points = []

        # Load dependencies
        # Dependencies are imported at module level

        # Initialize if dependencies are available
        if DEPENDENCIES_AVAILABLE:
            self._initialize_components(qcar)
        else:
            self.logger.warning(f"Vehicle {vehicle_id}: CamLidarFusion dependencies not available")

    def _check_dependencies(self) -> bool:
        """Check if all required dependencies are available."""
        return DEPENDENCIES_AVAILABLE

    def _initialize_components(self , qcar):
        """Initialize geometry manager and YOLO detector."""
        try:
            # Initialize geometry manager
            self.geometry_manager = RobotGeometryManager(True)
            # Override scale factor for physical setup
            self.geometry_manager.scale = self.scale_factor


            # Initialize YOLO detector
            self.yolo_detector = YOLOv8Wrapper_Huy(
                imageHeight=self.image_height,
                imageWidth=self.image_width
            )

            # Initialize sensor wrappers if not already done
            self.realsense_wrapper = RealSenseWrapper(qcar, self.geometry_manager)
            self.lidar_wrapper = LiDARWrapper(qcar)

            # Initialize visualization if enabled
            if self.enable_visualization:
                self._initialize_visualization()

            self.initialized = True
            self.logger.info(f"Vehicle {self.vehicle_id}: CamLidarFusion initialized successfully")

        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Failed to initialize CamLidarFusion: {e}")
            self.initialized = False

    def _initialize_visualization(self):
        """Initialize visualization windows and components using OpenCV."""
        try:
            # Initialize OpenCV window thread
            cv2.startWindowThread()

            print(f"Vehicle {self.vehicle_id}: OpenCV visualization initialized")
            print(f"Controls: 'l'=LiDAR overlay | 'p'=LiDAR plot | 'q'=quit visualization")

        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Failed to initialize visualization: {e}")
            self.enable_visualization = False

    def update_sensor_data(self) -> bool:
        """
        Update sensor data from QCar and perform fusion.

        Args:
            qcar: QLabsQCar2 instance for sensor access

        Returns:
            success: True if fusion completed successfully
        """
        if not self.initialized:
            return False

        start_time = time.time()

        try:
            # Get camera image and depth
            rgb_image, depth_image = self.realsense_wrapper.get_frames(self.yolo_detector)
            if rgb_image is None:
                if self.verbose:
                    print(f"Vehicle {self.vehicle_id}: Failed to get camera image")
                return False

            # Get LiDAR data
            angles, distances = self.lidar_wrapper.get_scan()
            if angles is None or distances is None:
                if self.verbose:
                    print(f"Vehicle {self.vehicle_id}: Failed to get LiDAR data")
                return False

            # Process with YOLO (matching qcar2_tuto.py parameters)
            dilate_kernel_size_param = 10  # Dilation kernel size for segmentation mask cleaning
            iterations_param = 1  # Morphological operation iterations

            annotated_image, detections = self.yolo_detector.process_qcar_image_with_yolo(
                rgb_image, depth_image=depth_image, apply_fov_alignment=False,
                kernel_size=dilate_kernel_size_param, iterations=iterations_param,
                verbose=self.verbose
            )

            if detections is None or len(detections) == 0:
                if self.verbose:
                    print(f"Vehicle {self.vehicle_id}: No detections found")
                
                # Store RGB image for visualization even with no detections
                self.last_rgb_image = rgb_image.copy()
                self.last_update_time = time.time()
                
                # Update visualization to show camera feed without detections
                if self.enable_visualization:
                    self._update_visualization([], [])
                
                return False

            # Extract segmentation masks
            # segmentation_masks = self.yolo_detector.extract_masks_from_annotated_image(annotated_image)

            # Project LiDAR points to camera coordinates (matching qcar2_tuto.py)
            projected_points, lidar_3d_points, lidar_3d_in_lidar = self.geometry_manager.lidar_to_camera_projection(
                angles, distances, self.image_width, self.image_height,
                segmentation_masks=None,
                erosion_kernel_size=self.erosion_kernel_size,
                shrink_factor_param=self.shrink_factor,
                detection_objects=detections
            )

            # Fuse LiDAR with detections (using qcar2_tuto.py fusion logic)
            fused_detections, filtered_lidar_points = self._fuse_lidar_with_detections(
                detections, self.geometry_manager, angles, distances,
                projected_points, lidar_3d_points,
                depth_image=None,
                segmentation_masks=None,
                erosion_kernel_size=self.erosion_kernel_size,
                shrink_factor_param=self.shrink_factor,
                image_width=self.image_width,
                image_height=self.image_height,
                verbose=self.verbose
            )

            # Project semantic objects to LiDAR coordinates (matching qcar2_tuto.py)
            corrected_detections = self.geometry_manager.project_semantic_objects_to_lidar(
                fused_detections, self.image_width, self.image_height
            )

            # Store results (use corrected detections for consistency with qcar2_tuto.py)
            self.last_relative_distances = {}
            if corrected_detections:
                # Store ALL detections, not just the closest one
                for i, detection in enumerate(corrected_detections):
                    if isinstance(detection, dict) and 'lidar_dist' in detection:
                        detection_key = f"detection_{i}"

                        self.last_relative_distances[detection_key] = {
                            'distance': detection.get('lidar_dist', 0.0),
                            'angle': detection.get('angle', 0.0),
                            'confidence': detection.get('fusion_confidence', 0.0),
                            'class_name': detection.get('class_name', 'vehicle'),
                            'lidar_points_count': detection.get('lidar_points_count', 0),
                            'timestamp': time.time(),
                            'bbox': detection.get('bbox', None),
                            'center_x': detection.get('center_x', 0),
                            'center_y': detection.get('center_y', 0)
                        }

                # # Keep track of closest detection for backward compatibility
                # if corrected_detections:
                #     closest_detection = min(corrected_detections,
                #                           key=lambda d: d.get('lidar_dist', float('inf')) if isinstance(d, dict) and 'lidar_dist' in d else float('inf'))
                # else:
                #     closest_detection = None

            # Store data for visualization
            self.last_rgb_image = rgb_image.copy()
            self.last_annotated_image = annotated_image.copy()
            self.last_angles = angles
            self.last_distances = distances
            self.last_projected_points = projected_points
            self.last_filtered_lidar_points = filtered_lidar_points
            self.last_corrected_detections = corrected_detections
            self.last_update_time = time.time()

            # Update visualization if enabled (even with no detections to keep windows alive)
            if self.enable_visualization:
                self._update_visualization(corrected_detections, filtered_lidar_points)

            # Update performance tracking
            self.update_count += 1
            processing_time = time.time() - start_time
            self.avg_processing_time = (
                (self.avg_processing_time * (self.update_count - 1)) + processing_time
            ) / self.update_count

            if self.verbose and self.update_count % 10 == 0:
                print(f"Vehicle {self.vehicle_id}: Fusion processed {len(corrected_detections)} detections "
                      f"in {processing_time:.3f}s (avg: {self.avg_processing_time:.3f}s)")

            return True

        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Error in sensor data update: {e}")
            return False

    def _get_camera_image(self, qcar) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get RGB and depth camera images from QCar using RealSenseWrapper."""
        try:
            if self.realsense_wrapper is None:
                return None, None
            
            # Get frames from RealSense wrapper
            rgb_image, depth_image = self.realsense_wrapper.get_frames(self.yolo_detector)
            
            return rgb_image, depth_image

        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Error getting camera images: {e}")
            return None, None

    def _get_lidar_data(self, qcar) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get LiDAR scan data from QCar using LiDARWrapper."""
        try:
            if self.lidar_wrapper is None:
                return None, None
            
            # Get scan from LiDAR wrapper
            angles, distances = self.lidar_wrapper.get_scan()
            
            if angles is not None and distances is not None:
                return angles, distances
            else:
                return None, None

        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Error getting LiDAR data: {e}")
            return None, None

    def _perform_fusion(self, rgb_image: np.ndarray, angles: np.ndarray,
                       distances: np.ndarray, depth_image: Optional[np.ndarray] = None) -> Tuple[Dict[str, Dict[str, float]], List]:
        """
        Perform camera-LiDAR fusion to get relative distances.

        Args:
            rgb_image: RGB camera image
            angles: LiDAR scan angles
            distances: LiDAR scan distances
            depth_image: Depth camera image (optional)

        Returns:
            relative_distances: Dictionary of detected objects with relative measurements
            projected_points: List of projected LiDAR points for visualization
        """
        # This method is now integrated into update_sensor_data() to match qcar2_tuto.py pipeline
        return {}, []

    def _update_visualization(self, corrected_detections, filtered_lidar_points):
        """Update visualization windows with current data (matching qcar2_tuto.py)."""
        try:
            # Update camera view with detections and LiDAR overlay
            if self.last_rgb_image is not None:
                # Use RGB image as base (always available)
                display_image = self.last_rgb_image.copy()

                # If we have annotated image with detections, use that instead
                if self.last_annotated_image is not None and len(corrected_detections) > 0:
                    display_image = self.last_annotated_image.copy()

                # Draw LiDAR points on camera image (matching qcar2_tuto.py)
                if len(filtered_lidar_points) > 0:
                    display_image = self.geometry_manager.draw_lidar_on_camera_image(
                        display_image,
                        filtered_lidar_points,
                        detection_objects=self.last_corrected_detections,  # Use corrected detections for reference
                        segmentation_masks=None,  # Could be added if needed
                        erosion_kernel_size=self.erosion_kernel_size,
                        shrink_factor=self.shrink_factor
                    )

                # Add detection information (matching qcar2_tuto.py style)
                self._draw_detection_info(display_image, corrected_detections)

                # Add status information
                info_text = f"Vehicle {self.vehicle_id} | Frame {self.update_count} | Objects: {len(corrected_detections)}"
                cv2.putText(display_image, info_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                cv2.imshow(f'Camera-LiDAR Fusion - Vehicle {self.vehicle_id}', display_image)

            # Update LiDAR plot (matching qcar2_tuto.py coordinate system)
            if self.show_lidar_plot:
                self._update_lidar_plot(corrected_detections)

            # Handle keyboard input
            self._handle_keyboard_input()

        except Exception as e:
            if self.verbose:
                print(f"Vehicle {self.vehicle_id}: Error updating visualization: {e}")

    def _draw_detection_info(self, image: np.ndarray, corrected_detections):
        """Draw detection information on camera image (matching qcar2_tuto.py)."""
        for detection in corrected_detections:
            if isinstance(detection, dict):
                # Handle dictionary format from corrected_detections
                if 'bbox' in detection and 'lidar_dist' in detection:
                    bbox = detection['bbox']
                    distance = detection['lidar_dist']
                    class_name = detection.get('class_name', 'vehicle')

                    # Position text at center of detection
                    text_x = detection.get('center_x', self.image_width // 2)
                    text_y = detection.get('center_y', self.image_height // 2)

                    # Create info text
                    info_text = f"{distance:.1f}m"
                    cv2.putText(image, info_text, (text_x, text_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(image, info_text, (text_x, text_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
            else:
                # Handle object format (fallback)
                distance = getattr(detection, 'distance', 0.0)
                class_name = getattr(detection, 'name', 'vehicle')

                center_x = getattr(detection, 'center_x', self.image_width // 2)
                center_y = getattr(detection, 'center_y', self.image_height // 2)

                info_text = f"{distance:.1f}m"
                cv2.putText(image, info_text, (center_x, center_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(image, info_text, (center_x, center_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

    def _update_lidar_plot(self, corrected_detections):
        """Update the LiDAR visualization using OpenCV (top-down view)."""
        try:
            # Create LiDAR visualization canvas
            lidar_vis = np.zeros((self.lidar_vis_size, self.lidar_vis_size, 3), dtype=np.uint8)

            # Fill with dark background
            lidar_vis[:] = (20, 20, 20)  # Dark gray background

            # Draw grid lines
            center_x, center_y = self.lidar_vis_size // 2, self.lidar_vis_size // 2

            # Draw range circles
            for r in range(5, self.lidar_vis_max_range + 1, 5):
                pixel_radius = int(r * self.lidar_vis_scale)
                if pixel_radius < center_x:
                    cv2.circle(lidar_vis, (center_x, center_y), pixel_radius, (50, 50, 50), 1)

            # Draw crosshairs
            cv2.line(lidar_vis, (center_x, 0), (center_x, self.lidar_vis_size), (50, 50, 50), 1)
            cv2.line(lidar_vis, (0, center_y), (self.lidar_vis_size, center_y), (50, 50, 50), 1)

            # Draw LiDAR point cloud
            if hasattr(self, 'last_angles') and hasattr(self, 'last_distances'):
                # Convert polar to cartesian coordinates (matching qcar2_tuto.py)
                x_lidar = np.sin(self.last_angles) * self.last_distances  # LEFT-RIGHT
                y_lidar = np.cos(self.last_angles) * self.last_distances  # FORWARD-BACKWARD

                for i in range(len(x_lidar)):
                    # Convert to pixel coordinates (center is vehicle position)
                    pixel_x = int(center_x + x_lidar[i] * self.lidar_vis_scale)
                    pixel_y = int(center_y - y_lidar[i] * self.lidar_vis_scale)  # Flip Y for top-down view

                    # Only draw points within bounds
                    if (0 <= pixel_x < self.lidar_vis_size and
                        0 <= pixel_y < self.lidar_vis_size and
                        self.last_distances[i] <= self.lidar_vis_max_range):

                        # Color based on distance
                        intensity = max(0, 255 - int(self.last_distances[i] * 255 / self.lidar_vis_max_range))
                        color = (intensity, intensity, intensity)
                        cv2.circle(lidar_vis, (pixel_x, pixel_y), 1, color, -1)

            # Draw detected objects
            if corrected_detections:
                for detection in corrected_detections:
                    # Handle both dict and object formats
                    lidar_x = None
                    lidar_y = None

                    if isinstance(detection, dict):
                        lidar_x = detection.get('lidar_x')
                        lidar_y = detection.get('lidar_y')
                    else:
                        # Try to access as object attributes
                        lidar_x = getattr(detection, 'lidar_x', None)
                        lidar_y = getattr(detection, 'lidar_y', None)

                    if lidar_x is not None and lidar_y is not None:
                        # Convert to pixel coordinates
                        pixel_x = int(center_x + lidar_y * self.lidar_vis_scale)  # Note: x = lidar_y in qcar2_tuto.py
                        pixel_y = int(center_y - (-lidar_x) * self.lidar_vis_scale)  # Note: y = -lidar_x in qcar2_tuto.py

                        # Only draw if within bounds
                        if 0 <= pixel_x < self.lidar_vis_size and 0 <= pixel_y < self.lidar_vis_size:
                            # Draw detection as green square
                            cv2.rectangle(lidar_vis,
                                        (pixel_x - 3, pixel_y - 3),
                                        (pixel_x + 3, pixel_y + 3),
                                        (0, 255, 0), -1)

                            # Add distance label
                            distance = detection.get('lidar_dist', 0.0) if isinstance(detection, dict) else getattr(detection, 'distance', 0.0)
                            label = f"{distance:.1f}m"
                            cv2.putText(lidar_vis, label, (pixel_x + 5, pixel_y - 5),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)

            # Draw vehicle at center
            cv2.circle(lidar_vis, (center_x, center_y), 3, (255, 0, 0), -1)  # Red dot for vehicle

            # Add info text
            info_text = f"Vehicle {self.vehicle_id} | LiDAR View"
            cv2.putText(lidar_vis, info_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            # Add scale info
            scale_text = f"Scale: {self.lidar_vis_scale} px/m | Range: {self.lidar_vis_max_range}m"
            cv2.putText(lidar_vis, scale_text, (10, self.lidar_vis_size - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)

            # Show the LiDAR visualization
            cv2.imshow(f'LiDAR View - Vehicle {self.vehicle_id}', lidar_vis)

        except Exception as e:
            if self.verbose:
                print(f"Vehicle {self.vehicle_id}: Error updating LiDAR plot: {e}")

    def _handle_keyboard_input(self):
        """Handle keyboard input for visualization controls."""
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.enable_visualization = False
            cv2.destroyAllWindows()
            print(f"Vehicle {self.vehicle_id}: Visualization disabled by user")
        elif key == ord('l'):
            self.show_lidar_plot = not self.show_lidar_plot
            print(f"Vehicle {self.vehicle_id}: LiDAR plot -> {'ON' if self.show_lidar_plot else 'OFF'}")
        

    def _fuse_lidar_with_detections(self, detection_objects, geometry_manager, angles, distances,
                                   projected_lidar_points, lidar_3d_points,
                                   segmentation_masks=None, depth_image=None, erosion_kernel_size=1,
                                   shrink_factor_param=-0.2, image_width=640, image_height=480, verbose=False):
        """
        Fuse LiDAR data with YOLO detections to get accurate distances.
        Enhanced Early_Fusion-style LiDAR-camera fusion matching qcar2_tuto.py

        Args:
            detection_objects: YOLO detection objects
            geometry_manager: Robot geometry manager for coordinate transformations
            angles: LiDAR scan angles
            distances: LiDAR scan distances
            projected_lidar_points: Pre-projected LiDAR points (u,v,depth)
            lidar_3d_points: 3D LiDAR points in camera frame
            segmentation_masks: Segmentation masks from YOLO
            depth_image: Depth camera image (optional)
            erosion_kernel_size: Erosion kernel size for mask cleaning
            shrink_factor_param: Bounding box shrink factor
            image_width: Camera image width
            image_height: Camera image height
            verbose: Enable debug output

        Returns:
            fused_objects: List of detection objects enhanced with distance/angle data
            filtered_lidar_points: List of filtered LiDAR points used in fusion
        """
        fused_objects = []
        filtered_lidar_points = []

        # Use already projected LiDAR points
        if verbose:
            print(f"Vehicle {self.vehicle_id}: Using {len(projected_lidar_points)} pre-processed LiDAR points")

        # Process each YOLO detection
        for i, detection in enumerate(detection_objects):
            if not hasattr(detection, 'center_x'):
                continue

            detection_distances = []
            depth_distances = []
            filtering_method = "none"
            current_filtered_points = []

            # Method 1 (preferred): LiDAR point-in-segmentation-mask fusion
            if detection.mask is not None:
                mask_np = detection.mask

                # Filter LiDAR points within segmentation mask
                for j, lidar_point in enumerate(projected_lidar_points):
                    u, v, depth = lidar_point
                    if 0 <= int(v) < mask_np.shape[0] and 0 <= int(u) < mask_np.shape[1]:
                        if mask_np[int(v), int(u)] > 0:
                            detection_distances.append(depth)
                            current_filtered_points.append(lidar_point)
                            filtered_lidar_points.append(lidar_point)

                if len(detection_distances) > 0:
                    filtering_method = "segmentation_mask"

            else:
                # Fallback to bounding box if no mask available
                if verbose:
                    print(f"Vehicle {self.vehicle_id}: Using bounding box fallback for detection {i}")
                for j, lidar_point in enumerate(projected_lidar_points):
                    u, v, depth = lidar_point
                    if self._rect_contains_point(detection, (u, v), image_width, image_height, shrink_factor=shrink_factor_param):
                        detection_distances.append(depth)
                        current_filtered_points.append(lidar_point)
                        filtered_lidar_points.append(lidar_point)

                if len(detection_distances) > 0:
                    filtering_method = "bounding_box"

            # Method 2: Depth camera extraction (if available)
            depth_distance = None
            if depth_image is not None:
                depth_distance = self._extract_depth_from_bbox(detection, depth_image, verbose=verbose)
                if depth_distance is not None:
                    depth_distances.append(depth_distance)

            # Apply Early_Fusion outlier filtering and get best distance
            if len(detection_distances) > 0:
                best_lidar_distance = self._get_best_distance(detection_distances, technique="closest")
            else:
                best_lidar_distance = None

            # Fuse LiDAR and depth measurements
            final_distance, confidence_score = self._fuse_distance_measurements_early_fusion_style(
                best_lidar_distance, depth_distance, len(detection_distances), verbose=True
            )

            if final_distance is not None:
                # Calculate angle from pixel coordinates
                pixel_angle_deg = math.degrees((detection.center_x / image_width - 0.5) * geometry_manager.camera_fov_h)

                # Update detection with fusion results
                detection.distance = final_distance
                detection.angle = float(pixel_angle_deg)
                detection.lidar_distance = best_lidar_distance if best_lidar_distance else 0.0
                detection.depth_distance = depth_distance if depth_distance else 0.0
                detection.fusion_confidence = confidence_score
                detection.lidar_points_count = len(detection_distances)

                fused_objects.append(detection)

                if verbose:
                    print(f"Vehicle {self.vehicle_id}: Early_Fusion - Detection {i+1}")
                    print(f"  - Filtering method: {filtering_method}")
                    print(f"  - LiDAR points found: {len(detection_distances)}")
                    print(f"  - LiDAR distance: {best_lidar_distance:.2f}m" if best_lidar_distance else "  - LiDAR: None")
                    print(f"  - Depth distance: {depth_distance:.2f}m" if depth_distance else "  - Depth: None")
                    print(f"  - Final distance: {final_distance:.2f}m (confidence: {confidence_score:.2f})")

        return fused_objects, filtered_lidar_points

    def _rect_contains_point(self, detection, point, image_width, image_height, shrink_factor=0.0):
        """Check if a point is inside a detection's bounding box with optional shrink factor."""
        try:
            # Import the utility function if available
            if DEPENDENCIES_AVAILABLE:
                from src.Cam_Lidar_fusion.qcar_utils_HUY import rect_contains_point
                return rect_contains_point(detection, point, image_width, image_height, shrink_factor=shrink_factor)
            else:
                # Fallback implementation
                u, v = point
                if hasattr(detection, 'bbox'):
                    bbox = detection.bbox
                    x1, y1, x2, y2 = bbox

                    # Apply shrink factor
                    width = x2 - x1
                    height = y2 - y1
                    shrink_x = width * shrink_factor
                    shrink_y = height * shrink_factor

                    x1 += shrink_x
                    y1 += shrink_y
                    x2 -= shrink_x
                    y2 -= shrink_y

                    return x1 <= u <= x2 and y1 <= v <= y2
                return False
        except Exception as e:
            if self.verbose:
                print(f"Vehicle {self.vehicle_id}: Error in rect_contains_point: {e}")
            return False

    def _extract_depth_from_bbox(self, detection, depth_image, verbose=False):
        """Extract depth distance from bounding box."""
        try:
            # Import the utility function if available
            if DEPENDENCIES_AVAILABLE:
                # from src.Cam_Lidar_fusion.qcar_utils_HUY import extract_depth_from_bbox
                return extract_depth_from_bbox(detection, depth_image, verbose=verbose)
            else:
                # Fallback implementation
                if hasattr(detection, 'bbox') and depth_image is not None:
                    bbox = detection.bbox
                    x1, y1, x2, y2 = map(int, bbox)

                    # Ensure bounds are within image
                    x1 = max(0, min(x1, depth_image.shape[1]))
                    x2 = max(0, min(x2, depth_image.shape[1]))
                    y1 = max(0, min(y1, depth_image.shape[0]))
                    y2 = max(0, min(y2, depth_image.shape[0]))

                    if x2 > x1 and y2 > y1:
                        roi = depth_image[y1:y2, x1:x2]
                        valid_depths = roi[(roi > 0) & (roi < 10000)]  # Filter valid depths
                        if len(valid_depths) > 0:
                            return float(np.median(valid_depths)) / 1000.0  # Convert to meters
                return None
        except Exception as e:
            if verbose:
                print(f"Vehicle {self.vehicle_id}: Error extracting depth from bbox: {e}")
            return None

    def _get_best_distance(self, distances, technique="closest"):
        """Get the best distance from a list of distances."""
        try:
            if DEPENDENCIES_AVAILABLE:
                return get_best_distance(distances, technique=technique)
            else:
                # Fallback implementation
                if not distances:
                    return None
                if technique == "closest":
                    return min(distances)
                elif technique == "median":
                    return np.median(distances)
                else:
                    return np.mean(distances)
        except Exception as e:
            if self.verbose:
                print(f"Vehicle {self.vehicle_id}: Error getting best distance: {e}")
            return None

    def _fuse_distance_measurements_early_fusion_style(self, lidar_distance, depth_distance, lidar_point_count, verbose=False):
        """
        Enhanced Early_Fusion style distance fusion with point count weighting

        Args:
            lidar_distance: Best LiDAR distance estimate
            depth_distance: Depth camera distance
            lidar_point_count: Number of LiDAR points used (affects confidence)
            verbose: Debug output

        Returns:
            final_distance: Fused distance estimate
            confidence_score: Confidence (0-1)
        """
        if lidar_distance is not None and depth_distance is not None:
            # Both measurements available - Early_Fusion weighted combination

            # Weight based on point count (more LiDAR points = higher confidence)
            lidar_confidence = min(1.0, lidar_point_count / 4.0)  # Max confidence at 4+ points
            depth_confidence = 0.6  # Depth camera baseline confidence

            # Distance-based weighting (Early_Fusion principle)
            if lidar_distance < 3.0:
                # Close range: prefer depth camera
                lidar_weight = 0.3 * lidar_confidence
                depth_weight = 0.7
            elif lidar_distance < 10.0:
                # Medium range: balanced
                lidar_weight = 0.6 * lidar_confidence
                depth_weight = 0.4
            else:
                # Long range: prefer LiDAR
                lidar_weight = 0.8 * lidar_confidence
                depth_weight = 0.2

            # Normalize weights
            total_weight = lidar_weight + depth_weight
            lidar_weight /= total_weight
            depth_weight /= total_weight

            final_distance = lidar_weight * lidar_distance + depth_weight * depth_distance
            confidence_score = min(0.95, (lidar_confidence + depth_confidence) / 2.0)

            if verbose:
                print(f"Vehicle {self.vehicle_id}: Early_Fusion: L={lidar_distance:.2f}m({lidar_weight:.2f}) + D={depth_distance:.2f}m({depth_weight:.2f}) = {final_distance:.2f}m")

            return final_distance, confidence_score

        elif lidar_distance is not None:
            # Only LiDAR available
            confidence_score = min(0.9, lidar_point_count / 5.0)  # Max 0.9 for LiDAR-only
            return lidar_distance, confidence_score

        elif depth_distance is not None:
            # Only depth available
            return depth_distance, 0.6  # Lower confidence for depth-only

        return None, 0.0

    def get_relative_distances(self) -> Dict[str, Dict[str, float]]:
        """
        Get the latest relative distance measurements to the closest detected object.

        Returns:
            Dictionary containing the closest detection with relative distance information
        """
        # Return only the closest detection
        if self.last_relative_distances:
            # Find the closest detection from all stored detections
            closest_key = min(self.last_relative_distances.keys(),
                            key=lambda k: self.last_relative_distances[k].get('distance', float('inf')))
            return {closest_key: self.last_relative_distances[closest_key].copy()}
        return {}

    def get_closest_distance(self) -> Optional[float]:
        """
        Get the distance to the closest detected object.

        Returns:
            Distance in meters to closest detection, or None if no detections
        """
        if self.last_relative_distances:
            # Find the closest detection
            closest_key = min(self.last_relative_distances.keys(),
                            key=lambda k: self.last_relative_distances[k].get('distance', float('inf')))
            return self.last_relative_distances[closest_key].get('distance', None)
        return None

    def get_vehicle_distances(self) -> List[Dict[str, float]]:
        """
        Get distances specifically to detected vehicles.

        Returns:
            List of vehicle detections with distance information
        """
        vehicle_distances = []
        for detection_key, detection_data in self.last_relative_distances.items():
            if detection_data.get('class_name', '').lower() in ['car', 'truck', 'vehicle', 'bus']:
                vehicle_distances.append(detection_data)

        return vehicle_distances

    def get_status(self) -> Dict[str, Any]:
        """Get component status and statistics."""
        return {
            'initialized': self.initialized,
            'last_update_time': self.last_update_time,
            'detection_count': len(self.last_relative_distances),
            'update_count': self.update_count,
            'avg_processing_time': self.avg_processing_time,
            'dependencies_available': self._check_dependencies(),
            'visualization_enabled': self.enable_visualization,
            'lidar_plot_enabled': self.show_lidar_plot
        }

    def reset(self):
        """Reset component state."""
        self.last_relative_distances = {}
        self.update_count = 0
        self.avg_processing_time = 0.0

        # Clean up visualization
        if self.enable_visualization:
            cv2.destroyAllWindows()
            self.last_rgb_image = None
            self.last_projected_points = []

        print(f"Vehicle {self.vehicle_id}: CamLidarFusion reset")