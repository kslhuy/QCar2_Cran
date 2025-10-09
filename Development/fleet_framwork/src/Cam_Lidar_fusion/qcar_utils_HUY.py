"""
QCar Utilities Module

This module contains utility functions used across the QCar simulation system.
Functions are organized into categories:
- Distance and Statistical Utilities
- Geometric and Image Processing Utilities  
- Sensor Data Fusion Utilities
- Debug and Visualization Utilities
"""

import numpy as np
import cv2
import math
from typing import List, Tuple, Optional, Union, Any


# =============================================================================
# DISTANCE AND STATISTICAL UTILITIES
# =============================================================================

def filter_outlier_distances(distances: Union[List[float], np.ndarray], method: str = "one_sigma") -> np.ndarray:
    """
    Enhanced outlier filtering using Early_Fusion techniques with QCar adaptations
    
    Args:
        distances: Array of distance measurements
        method: "one_sigma", "three_sigma", "iqr", or "ransac"
        
    Returns:
        filtered_distances: Array with outliers removed
    """
    if len(distances) < 3:
        return np.array(distances)
        
    distances = np.array(distances)
    
    if method == "one_sigma":
        # Early_Fusion's one sigma method - most reliable for automotive applications
        mean_dist = np.mean(distances)
        std_dist = np.std(distances)
        
        # Use set intersection like Early_Fusion for precise filtering
        upper_mask = distances <= (mean_dist + std_dist)
        lower_mask = distances >= (mean_dist - std_dist)
        valid_indices = set(np.where(upper_mask)[0]) & set(np.where(lower_mask)[0])
        
        return distances[list(valid_indices)]
        
    elif method == "three_sigma": 
        # Three sigma rule for more permissive filtering
        mean_dist = np.mean(distances)
        std_dist = np.std(distances)
        mask = np.abs(distances - mean_dist) <= 3 * std_dist
        return distances[mask]
        
    elif method == "iqr":
        # Interquartile range method - robust against extreme outliers
        q1 = np.percentile(distances, 25)
        q3 = np.percentile(distances, 75)
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr
        mask = (distances >= lower_bound) & (distances <= upper_bound)
        return distances[mask]
    
    elif method == "ransac":
        # RANSAC-like approach for robust distance estimation
        if len(distances) < 5:
            return distances
            
        best_inliers = []
        max_inliers = 0
        
        # Try multiple random samples
        for _ in range(min(10, len(distances))):
            # Sample 3 random points
            sample_indices = np.random.choice(len(distances), min(3, len(distances)), replace=False)
            sample_mean = np.mean(distances[sample_indices])
            
            # Count inliers within threshold
            threshold = 0.5  # 50cm threshold for automotive distances
            inlier_mask = np.abs(distances - sample_mean) <= threshold
            inlier_count = np.sum(inlier_mask)
            
            if inlier_count > max_inliers:
                max_inliers = inlier_count
                best_inliers = distances[inlier_mask]
        
        return best_inliers if len(best_inliers) > 0 else distances
    
    return distances


def get_best_distance(distances: Union[List[float], np.ndarray], technique: str = "median") -> Optional[float]:
    """
    Enhanced distance selection using Early_Fusion's comprehensive approach
    
    Args:
        distances: Array of distance measurements
        technique: "closest", "farthest", "average", "median", "robust_mean", "early_fusion_best"
        
    Returns:
        best_distance: Single distance estimate optimized for automotive scenarios
    """
    if len(distances) == 0:
        return None
        
    distances = np.array(distances)
    
    if technique == "closest":
        return float(np.min(distances))
    elif technique == "farthest": 
        return float(np.max(distances))
    elif technique == "average":
        return float(np.mean(distances))
    elif technique == "median":
        return float(np.median(distances))
    elif technique == "robust_mean":
        # Robust mean excluding outliers
        q1, q3 = np.percentile(distances, [25, 75])
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr
        filtered = distances[(distances >= lower_bound) & (distances <= upper_bound)]
        return float(np.mean(filtered)) if len(filtered) > 0 else float(np.mean(distances))
    elif technique == "early_fusion_best":
        # Early_Fusion's sophisticated technique
        if len(distances) == 1:
            return float(distances[0])
        elif len(distances) == 2:
            return float(np.mean(distances))  # Average of two
        else:
            # For 3+ measurements, use median for robustness
            median_val = np.median(distances)
            # But also consider the closest reading if it's within reasonable bounds
            closest_val = np.min(distances)
            
            # Early_Fusion weighting: prefer median but bias towards closer readings
            if abs(closest_val - median_val) / median_val < 0.2:  # Within 20%
                return float(0.7 * median_val + 0.3 * closest_val)
            else:
                return float(median_val)  # Use median if closest is too different
    
    # Default to median
    return float(np.median(distances))


# =============================================================================
# GEOMETRIC AND IMAGE PROCESSING UTILITIES
# =============================================================================

def rect_contains_point(detected_object , 
                       point, 
                       image_width , image_height, 
                       shrink_factor = 0.0,
                       only_height: bool = True) -> bool:
    """
    Enhanced bounding box containment check based on Early_Fusion's rectContains
    Implements sophisticated shrinking strategy for outlier reduction

    Note: shrink_factor=-0.2 means expand the bbox by 20% (negative shrink = expansion for more permissive matching).
    
    Args:
        bbox: Bounding box [x1, y1, x2, y2] or [center_x, center_y, width, height]
        point: 2D point [x, y]
        image_width: Image width for normalization
        image_height: Image height for normalization  
        shrink_factor: Early_Fusion shrink factor (0.0-0.3 recommended for automotive)
        
    Returns:
        bool: True if point is inside the (potentially shrunk) bounding box
    """
    # Handle different bbox formats
    
    # Handle object attributes if it's not a simple array
    if hasattr(detected_object, 'center_x') and hasattr(detected_object, 'center_y'):
        center_x, center_y = detected_object.center_x, detected_object.center_y
        width, height = detected_object.width, detected_object.height
    else:
        center_x, center_y = detected_object["center_x"], detected_object["center_y"]
        width, height = detected_object["width"], detected_object["height"]



    if only_height:
        # Only shrink height (vertical), width remains unchanged
        max_x = center_x + width / 2.0
        min_x = center_x - width / 2.0
        # Apply shrink_factor only to height
        new_height = height * (1 - shrink_factor)
        max_y = center_y + new_height / 2.0
        min_y = center_y - new_height / 2.0
    else:
        # Early_Fusion shrinking strategy - reduces outliers by tightening bbox
        # Automotive research shows 10-20% shrink improves LiDAR-camera association
        max_x = center_x + (width * (1 - shrink_factor)) / 2.0
        min_x = center_x - (width * (1 - shrink_factor)) / 2.0
        max_y = center_y + (height * (1 - shrink_factor)) / 2.0
        min_y = center_y - (height * (1 - shrink_factor)) / 2.0
    
    # Early_Fusion containment check with proper bounds
    if min_x <= point[0] <= max_x:
        if min_y <= point[1] <= max_y:
            return True
    
    return False


def extract_depth_from_bbox(detection, depth_image, verbose=False):
    """
    Extract depth information from detection bounding box
    
    Args:
        detection: YOLO detection object with bbox
        depth_image: Depth image (uint16, values in mm)
        verbose: Enable debug output
        
    Returns:
        mean_depth: Average depth in meters, or None if invalid
    """
    try:
        bbox = detection.bbox
        center_x, center_y = int(detection.center_x), int(detection.center_y)
        
        # Method 1: Sample depth at bbox center
        if (0 <= center_y < depth_image.shape[0] and 
            0 <= center_x < depth_image.shape[1]):
            center_depth = depth_image[center_y, center_x] / 1000.0  # Convert mm to meters
        else:
            center_depth = 0.0
        
        # Method 2: Average depth over bbox area (more robust)
        x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        
        # Ensure bbox is within image bounds
        x1 = max(0, min(x1, depth_image.shape[1] - 1))
        x2 = max(0, min(x2, depth_image.shape[1] - 1))
        y1 = max(0, min(y1, depth_image.shape[0] - 1))
        y2 = max(0, min(y2, depth_image.shape[0] - 1))
        
        if x2 > x1 and y2 > y1:
            # Extract bbox region
            bbox_depths = depth_image[y1:y2, x1:x2]
            
            # Filter out invalid depths (too close/far or zero)
            valid_depths = bbox_depths[(bbox_depths > 500) & (bbox_depths < 50000)]  # 0.5m to 50m in mm
            
            if len(valid_depths) > 0:
                mean_depth = float(np.mean(valid_depths)) / 1000.0  # Convert to meters
                
                if verbose:
                    print(f"    Depth extraction: center={center_depth:.2f}m, bbox_mean={mean_depth:.2f}m, valid_pixels={len(valid_depths)}")
                
                # Use bbox average if we have enough valid pixels, otherwise use center
                if len(valid_depths) > 10:  # At least 10 valid pixels for reliable average
                    return mean_depth
                elif center_depth > 0.5 and center_depth < 50.0:  # Use center if valid
                    return center_depth
                    
        # Fallback to center depth if available
        if center_depth > 0.5 and center_depth < 50.0:
            return center_depth
            
    except Exception as e:
        if verbose:
            print(f"    Error extracting depth: {e}")
    
    return None


def erode_mask(mask: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    """
    Apply morphological erosion to a binary mask
    
    Args:
        mask: Binary mask to erode
        kernel_size: Size of erosion kernel
        
    Returns:
        Eroded mask
    """
    if mask is None or mask.size == 0:
        return mask
        
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    eroded_mask = cv2.erode(mask, kernel, iterations=1)
    return eroded_mask


# =============================================================================
# SENSOR DATA FUSION UTILITIES
# =============================================================================

def find_lidar_distance_angular_match(angles: np.ndarray, distances: np.ndarray, 
                                     target_angle: float, 
                                     search_degrees: List[int] = [3, 6, 10], 
                                     verbose: bool = False) -> Optional[float]:
    """
    Find LiDAR distance using angular matching with progressive search
    
    Args:
        angles: Array of LiDAR angles
        distances: Array of LiDAR distances
        target_angle: Target angle to search for
        search_degrees: Progressive search radii in degrees
        verbose: Enable verbose output
        
    Returns:
        Best matching distance or None
    """
    matched_distances = []
    
    for search_deg in search_degrees:
        search_rad = math.radians(search_deg)
        angle_diff = np.abs(angles - target_angle)
        close_indices = np.where(angle_diff < search_rad)[0]
        
        if len(close_indices) > 0:
            close_distances = distances[close_indices]
            valid_distances = close_distances[(close_distances > 0.1) & (close_distances < 50)]
            
            if len(valid_distances) >= 2:
                matched_distances.extend(valid_distances)
                break  # Use tightest match that has enough points
    
    return matched_distances


def fuse_distance_measurements_early_fusion_style(lidar_distance: Optional[float], 
                                                 depth_distance: Optional[float], 
                                                 lidar_point_count: int = 0, 
                                                 verbose: bool = False) -> Optional[float]:
    """
    Fuse distance measurements using Early_Fusion methodology
    
    Args:
        lidar_distance: Distance from LiDAR sensor
        depth_distance: Distance from depth camera
        lidar_point_count: Number of LiDAR points used
        verbose: Enable verbose output
        
    Returns:
        Fused distance estimate
    """
    if lidar_distance is None and depth_distance is None:
        return None
    elif lidar_distance is None:
        return depth_distance
    elif depth_distance is None:
        return lidar_distance
    
    # Early_Fusion weighting strategy
    # More LiDAR points = higher confidence
    lidar_weight = min(0.8, 0.4 + (lidar_point_count * 0.1))  # 0.4 to 0.8 based on point count
    depth_weight = 1.0 - lidar_weight
    
    # Check for sensor disagreement
    distance_diff = abs(lidar_distance - depth_distance)
    relative_diff = distance_diff / min(lidar_distance, depth_distance)
    
    if relative_diff > 0.3:  # 30% disagreement threshold
        # High disagreement - favor sensor with more data points
        if lidar_point_count > 5:
            fused_distance = lidar_distance  # Trust LiDAR with many points
        else:
            fused_distance = depth_distance  # Trust depth camera
        
        if verbose:
            print(f"High sensor disagreement ({relative_diff:.2%}), using {'LiDAR' if lidar_point_count > 5 else 'depth'}")
    else:
        # Low disagreement - weighted fusion
        fused_distance = lidar_weight * lidar_distance + depth_weight * depth_distance
        
        if verbose:
            print(f"Fused distance: {fused_distance:.3f}m (LiDAR: {lidar_distance:.3f}, Depth: {depth_distance:.3f})")
    
    return float(fused_distance)


def fuse_distance_measurements(lidar_distance: Optional[float], 
                              depth_distance: Optional[float], 
                              verbose: bool = False) -> Optional[float]:
    """
    Simple distance fusion with basic conflict resolution
    
    Args:
        lidar_distance: Distance from LiDAR
        depth_distance: Distance from depth camera
        verbose: Enable verbose output
        
    Returns:
        Fused distance
    """
    if lidar_distance is None and depth_distance is None:
        return None
    elif lidar_distance is None:
        return depth_distance
    elif depth_distance is None:
        return lidar_distance
    
    # Simple weighted average favoring LiDAR
    fused_distance = 0.6 * lidar_distance + 0.4 * depth_distance
    
    if verbose:
        diff = abs(lidar_distance - depth_distance)
        print(f"Fused: {fused_distance:.3f}m (LiDAR: {lidar_distance:.3f}m, Depth: {depth_distance:.3f}m, Diff: {diff:.3f}m)")
    
    return float(fused_distance)


# =============================================================================
# DEBUG AND VISUALIZATION UTILITIES
# =============================================================================

def debug_detection_object(detection: Any, name: str = "Detection") -> None:
    """
    Debug utility to print detection object information
    
    Args:
        detection: Detection object to debug
        name: Name for the debug output
    """
    print(f"\n=== {name} Debug Info ===")
    
    if hasattr(detection, '__len__') and not isinstance(detection, str):
        # Array-like detection
        print(f"Type: {type(detection)}")
        print(f"Length: {len(detection)}")
        if len(detection) >= 4:
            print(f"Bbox: [{detection[0]:.1f}, {detection[1]:.1f}, {detection[2]:.1f}, {detection[3]:.1f}]")
        if len(detection) > 4:
            print(f"Additional: {detection[4:]}")
    else:
        # Object detection
        print(f"Type: {type(detection)}")
        attrs = ['name', 'distance', 'conf', 'x', 'y', 'bbox', 'center_x', 'center_y', 'width', 'height', 'area']
        for attr in attrs:
            if hasattr(detection, attr):
                value = getattr(detection, attr)
                print(f"{attr}: {value}")
    print("=" * 30)


def create_debug_image(rgb_image: np.ndarray, 
                      detection_objects: List[Any], 
                      title: str = "Debug View") -> np.ndarray:
    """
    Create a debug visualization image with detection overlays
    
    Args:
        rgb_image: Base RGB image
        detection_objects: List of detection objects
        title: Title for the debug view
        
    Returns:
        Debug image with overlays
    """
    debug_img = rgb_image.copy()
    
    # Draw detection boxes
    for i, detection in enumerate(detection_objects):
        if hasattr(detection, 'bbox') and detection.bbox is not None:
            x1, y1, x2, y2 = detection.bbox[:4]
            color = (0, 255, 0) if i % 2 == 0 else (255, 0, 0)  # Alternate colors
            cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            
            # Add detection info
            label = f"{detection.name if hasattr(detection, 'name') else 'Unknown'}"
            if hasattr(detection, 'distance') and detection.distance is not None:
                label += f" {detection.distance:.2f}m"
            
            cv2.putText(debug_img, label, (int(x1), int(y1) - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    
    # Add title
    cv2.putText(debug_img, title, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    return debug_img


def build_probability_colormap(prob_map: np.ndarray) -> np.ndarray:
    """
    Convert probability map (0..1) to a colored image for display
    
    Args:
        prob_map: Probability map with values 0-1
        
    Returns:
        Colored visualization image
    """
    prob_norm = (np.clip(prob_map, 0.0, 1.0) * 255).astype(np.uint8)
    colored = cv2.applyColorMap(prob_norm, cv2.COLORMAP_INFERNO)
    return colored


# =============================================================================
# ANGLE AND TRIGONOMETRIC UTILITIES
# =============================================================================

def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi] range
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def angle_difference(angle1: float, angle2: float) -> float:
    """
    Calculate the smallest difference between two angles
    
    Args:
        angle1: First angle in radians
        angle2: Second angle in radians
        
    Returns:
        Smallest angle difference
    """
    diff = normalize_angle(angle1 - angle2)
    return abs(diff)


def pixel_to_angle(pixel_x: int, image_width: int, fov_horizontal: float) -> float:
    """
    Convert pixel coordinate to camera angle
    
    Args:
        pixel_x: Pixel x coordinate
        image_width: Image width in pixels
        fov_horizontal: Horizontal field of view in radians
        
    Returns:
        Angle in radians relative to camera center
    """
    # Convert pixel to normalized coordinate [-1, 1]
    normalized_x = (2 * pixel_x / image_width) - 1
    
    # Convert to angle
    angle = normalized_x * (fov_horizontal / 2)
    
    return angle


def combine_segmentation_masks(segmentation_masks: List[np.ndarray], 
                              image_height: int, image_width: int,
                              erosion_kernel_size: int = 3) -> np.ndarray:
    """
    Combine multiple segmentation masks into a single binary mask for LiDAR filtering
    OPTIMIZED: Specifically designed for the enhanced LiDAR projection pipeline
    
    Args:
        segmentation_masks: List of individual segmentation masks from YOLO
        image_height: Target image height
        image_width: Target image width
        erosion_kernel_size: Size of erosion kernel to refine mask boundaries
        
    Returns:
        combined_mask: Single binary mask combining all input masks (dtype=bool)
    """
    if not segmentation_masks or len(segmentation_masks) == 0:
        return np.zeros((image_height, image_width), dtype=bool)
    
    # Initialize combined mask
    combined_mask = np.zeros((image_height, image_width), dtype=bool)
    erosion_kernel = np.ones((erosion_kernel_size, erosion_kernel_size), np.uint8)
    
    for i, mask in enumerate(segmentation_masks):
        try:
            # Convert mask to numpy array if needed
            if hasattr(mask, 'cpu'):
                mask_np = mask.cpu().numpy()
            elif hasattr(mask, 'numpy'):
                mask_np = mask.numpy()
            else:
                mask_np = np.array(mask)
            
            # Ensure mask is 2D
            if len(mask_np.shape) > 2:
                mask_np = mask_np.squeeze()
            elif len(mask_np.shape) < 2:
                print(f"Warning: Segmentation mask {i} has invalid shape {mask_np.shape}, skipping")
                continue
            
            # Resize mask to target dimensions if needed
            if mask_np.shape != (image_height, image_width):
                mask_np = cv2.resize(mask_np.astype(np.float32), 
                                   (image_width, image_height), 
                                   interpolation=cv2.INTER_NEAREST)
            
            # Convert to binary mask
            binary_mask = (mask_np > 0.5).astype(np.uint8)
            
            # Apply erosion to refine boundaries (reduces noise, tightens object boundaries)
            if erosion_kernel_size > 0:
                eroded_mask = cv2.erode(binary_mask, erosion_kernel, iterations=1)
            else:
                eroded_mask = binary_mask
            
            # Combine with existing mask (logical OR)
            combined_mask |= (eroded_mask > 0)
            
        except Exception as e:
            print(f"Error processing segmentation mask {i}: {e}")
            continue
    
    return combined_mask


def filter_lidar_points_by_masks(lidar_points: np.ndarray, 
                                 segmentation_masks: List[np.ndarray],
                                 image_height: int, image_width: int) -> Tuple[np.ndarray, np.ndarray]:
    """
    Filter LiDAR points based on segmentation masks for efficient processing
    OPTIMIZED: Core utility for the enhanced LiDAR-YOLO fusion pipeline
    
    Args:
        lidar_points: Nx2 array of projected LiDAR points [[u, v], ...]
        segmentation_masks: List of segmentation masks from YOLO
        image_height: Image height
        image_width: Image width
        
    Returns:
        filtered_points: LiDAR points that fall within segmentation masks
        filter_mask: Boolean array indicating which original points were kept
    """
    if len(lidar_points) == 0:
        return np.array([]), np.array([], dtype=bool)
    
    # Combine all segmentation masks
    combined_mask = combine_segmentation_masks(segmentation_masks, image_height, image_width)
    
    # Check which LiDAR points fall within masks
    filter_mask = np.zeros(len(lidar_points), dtype=bool)
    
    for i, point in enumerate(lidar_points):
        u, v = int(point[0]), int(point[1])
        
        # Check bounds and mask value
        if (0 <= u < image_width and 0 <= v < image_height):
            filter_mask[i] = combined_mask[v, u]
    
    filtered_points = lidar_points[filter_mask]
    
    return filtered_points, filter_mask
