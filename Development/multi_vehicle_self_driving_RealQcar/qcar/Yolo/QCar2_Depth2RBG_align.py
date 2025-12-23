import numpy as np
import cv2
import time
import json
import os
from pit.LaneNet.nets import LaneNet
from pal.utilities.vision import Camera3D
from pal.products.qcar import QCarRealSense


## Timing Parameters and methods 
def elapsed_time():
    return time.time() - startTime

def save_alignment_settings():
    """Save current alignment settings to mode-specific JSON file"""
    # Use different filenames for different alignment modes
    filename = "alignment_settings_intrinsics.json" if use_intrinsics else "alignment_settings_simple.json"
    
    if use_intrinsics:
        # Intrinsics mode settings
        settings = {
            "mode": "intrinsics",
            "focal_scale_rgb": focal_scale_rgb,
            "focal_scale_depth": focal_scale_depth,
            "crop_offset_x": crop_offset_x,
            "crop_offset_y": crop_offset_y,
            "crop_scale_x": crop_scale_x,
            "crop_scale_y": crop_scale_y,
            "edge_threshold1": edge_threshold1,
            "edge_threshold2": edge_threshold2,
            "visualization_mode": visualization_mode
        }
    else:
        # Simple mode settings
        settings = {
            "mode": "simple",
            "crop_top": crop_top,
            "crop_bottom": crop_bottom,
            "crop_left": crop_left,
            "crop_right": crop_right,
            "edge_threshold1": edge_threshold1,
            "edge_threshold2": edge_threshold2,
            "visualization_mode": visualization_mode
        }
    
    filepath = os.path.join(os.path.dirname(__file__), filename)
    with open(filepath, 'w') as f:
        json.dump(settings, f, indent=4)
    mode_name = "Intrinsics" if use_intrinsics else "Simple"
    print(f"\n💾 {mode_name} mode settings saved to: {filepath}")
    return filepath

def load_alignment_settings():
    """Load alignment settings from mode-specific JSON file"""
    global focal_scale_rgb, focal_scale_depth, crop_offset_x, crop_offset_y
    global crop_scale_x, crop_scale_y, edge_threshold1, edge_threshold2, visualization_mode
    global fx_rgb, fy_rgb, fx_depth, fy_depth, K_rgb, K_depth
    global crop_top, crop_bottom, crop_left, crop_right
    
    # Use different filenames for different alignment modes
    filename = "alignment_settings_intrinsics.json" if use_intrinsics else "alignment_settings_simple.json"
    filepath = os.path.join(os.path.dirname(__file__), filename)
    
    if not os.path.exists(filepath):
        print(f"\n⚠ Settings file not found: {filepath}")
        return False
    
    try:
        with open(filepath, 'r') as f:
            settings = json.load(f)
        
        mode_name = settings.get("mode", "unknown")
        
        if use_intrinsics and mode_name == "intrinsics":
            # Load intrinsics mode settings
            focal_scale_rgb = settings.get("focal_scale_rgb", 1.0)
            focal_scale_depth = settings.get("focal_scale_depth", 1.0)
            crop_offset_x = settings.get("crop_offset_x", 0)
            crop_offset_y = settings.get("crop_offset_y", 0)
            crop_scale_x = settings.get("crop_scale_x", 1.0)
            crop_scale_y = settings.get("crop_scale_y", 1.0)
            edge_threshold1 = settings.get("edge_threshold1", 50)
            edge_threshold2 = settings.get("edge_threshold2", 150)
            visualization_mode = settings.get("visualization_mode", 0)
            
            # Recalculate focal lengths
            fx_rgb = (imageWidth / 2.0) / np.tan(fov_rgb_h / 2.0) * focal_scale_rgb
            fy_rgb = (imageHeight / 2.0) / np.tan(fov_rgb_v / 2.0) * focal_scale_rgb
            fx_depth = (imageWidth / 2.0) / np.tan(fov_depth_h / 2.0) * focal_scale_depth
            fy_depth = (imageHeight / 2.0) / np.tan(fov_depth_v / 2.0) * focal_scale_depth
            
            # Update intrinsic matrices
            K_rgb[0, 0] = fx_rgb
            K_rgb[1, 1] = fy_rgb
            K_depth[0, 0] = fx_depth
            K_depth[1, 1] = fy_depth
            
            print(f"\n📂 Intrinsics mode settings loaded from: {filepath}")
            print(f"   Focal scales: RGB={focal_scale_rgb:.3f}, Depth={focal_scale_depth:.3f}")
            print(f"   Crop offset: X={crop_offset_x}, Y={crop_offset_y}")
            print(f"   Crop scale: X={crop_scale_x:.3f}, Y={crop_scale_y:.3f}")
            print(f"   Edge thresholds: {edge_threshold1}/{edge_threshold2}")
            
        elif not use_intrinsics and mode_name == "simple":
            # Load simple mode settings
            crop_top = settings.get("crop_top", 65)
            crop_bottom = settings.get("crop_bottom", 415)
            crop_left = settings.get("crop_left", 105)
            crop_right = settings.get("crop_right", 510)
            edge_threshold1 = settings.get("edge_threshold1", 50)
            edge_threshold2 = settings.get("edge_threshold2", 150)
            visualization_mode = settings.get("visualization_mode", 0)
            
            print(f"\n📂 Simple mode settings loaded from: {filepath}")
            print(f"   Crop boundaries: Top={crop_top}, Bottom={crop_bottom}, Left={crop_left}, Right={crop_right}")
            print(f"   Edge thresholds: {edge_threshold1}/{edge_threshold2}")
        else:
            expected_mode = "intrinsics" if use_intrinsics else "simple"
            print(f"\n⚠ Warning: Settings file mode mismatch. Expected '{expected_mode}', got '{mode_name}'")
            return False
        
        return True
    except Exception as e:
        print(f"\n❌ Error loading settings: {e}")
        return False

def align_depth_to_rgb_using_intrinsics(depth_image, rgb_shape, K_rgb, K_depth, depth_to_rgb_transform):
    """
    Align depth image to RGB using camera intrinsics and extrinsics.
    
    Args:
        depth_image: Depth image in meters (H_d, W_d)
        rgb_shape: Target RGB shape (H_rgb, W_rgb)
        K_rgb: RGB camera intrinsic matrix (3x3)
        K_depth: Depth camera intrinsic matrix (3x3)
        depth_to_rgb_transform: Transformation matrix from depth to RGB camera frame (4x4)
    
    Returns:
        aligned_depth: Depth image aligned to RGB frame
    """
    h_d, w_d = depth_image.shape[:2]
    h_rgb, w_rgb = rgb_shape[:2]
    
    # Create aligned depth image
    aligned_depth = np.zeros((h_rgb, w_rgb), dtype=np.float32)
    
    # Generate depth pixel coordinates
    u_d, v_d = np.meshgrid(np.arange(w_d), np.arange(h_d))
    
    # Get depth values
    z_d = depth_image.reshape(-1)
    valid_mask = z_d > 0
    
    # Pixel coordinates
    u_d_flat = u_d.reshape(-1)[valid_mask]
    v_d_flat = v_d.reshape(-1)[valid_mask]
    z_d_flat = z_d[valid_mask]
    
    # Back-project depth pixels to 3D points in depth camera frame
    K_depth_inv = np.linalg.inv(K_depth)
    pixels_depth = np.vstack([u_d_flat, v_d_flat, np.ones_like(u_d_flat)])
    points_3d_depth = K_depth_inv @ pixels_depth * z_d_flat
    
    # Transform to RGB camera frame
    points_3d_depth_homo = np.vstack([points_3d_depth, np.ones((1, points_3d_depth.shape[1]))])
    points_3d_rgb_homo = depth_to_rgb_transform @ points_3d_depth_homo
    points_3d_rgb = points_3d_rgb_homo[:3, :]
    
    # Project to RGB image plane
    pixels_rgb = K_rgb @ points_3d_rgb
    pixels_rgb = pixels_rgb[:2, :] / pixels_rgb[2, :]
    
    # Round to nearest pixel
    u_rgb = np.round(pixels_rgb[0, :]).astype(np.int32)
    v_rgb = np.round(pixels_rgb[1, :]).astype(np.int32)
    
    # Filter valid projections
    valid_proj = (u_rgb >= 0) & (u_rgb < w_rgb) & (v_rgb >= 0) & (v_rgb < h_rgb)
    u_rgb = u_rgb[valid_proj]
    v_rgb = v_rgb[valid_proj]
    z_rgb = points_3d_rgb[2, valid_proj]
    
    # Fill aligned depth (keep closest depth for overlapping pixels)
    for i in range(len(u_rgb)):
        if aligned_depth[v_rgb[i], u_rgb[i]] == 0 or z_rgb[i] < aligned_depth[v_rgb[i], u_rgb[i]]:
            aligned_depth[v_rgb[i], u_rgb[i]] = z_rgb[i]
    
    # Optional: Fill holes with interpolation
    mask = aligned_depth > 0
    if np.sum(mask) > 0:
        from scipy.interpolate import griddata
        points = np.column_stack(np.where(mask))
        values = aligned_depth[mask]
        grid_y, grid_x = np.mgrid[0:h_rgb, 0:w_rgb]
        try:
            aligned_depth = griddata(points, values, (grid_y, grid_x), method='nearest', fill_value=0)
        except:
            pass  # Keep original if interpolation fails
    
    return aligned_depth

def simple_align_depth_to_rgb(depth_image, target_shape, crop_bounds=None):
    """
    Simple alignment using crop and resize.
    
    Args:
        depth_image: Input depth image
        target_shape: Target (height, width)
        crop_bounds: Dict with 'top', 'bottom', 'left', 'right' pixel coordinates
    """
    if crop_bounds is not None:
        # Crop the depth image first
        top = crop_bounds['top']
        bottom = crop_bounds['bottom']
        left = crop_bounds['left']
        right = crop_bounds['right']
        
        # Ensure bounds are valid
        h, w = depth_image.shape[:2]
        top = max(0, min(top, h))
        bottom = max(top + 1, min(bottom, h))
        left = max(0, min(left, w))
        right = max(left + 1, min(right, w))
        
        depth_cropped = depth_image[top:bottom, left:right]
    else:
        depth_cropped = depth_image
    
    # Resize cropped depth to match RGB dimensions
    depth_resized = cv2.resize(depth_cropped, (target_shape[1], target_shape[0]), interpolation=cv2.INTER_NEAREST)
    return depth_resized

sampleRate     = 30.0
sampleTime     = 1/sampleRate
simulationTime = 1000.0
print('Sample Time: ', sampleTime)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Additional parameters
imageWidth  = 640
imageHeight = 480

# Depth visualization parameters
clipping_distance_in_meters = 3.0  # Remove background beyond this distance

# Alignment visualization parameters
# Visualization modes: 0=Edges, 1=Checkerboard, 2=Contours, 3=Overlay, 4=All, 5=RGB+Depth side-by-side
visualization_mode = 0  # Start with edge detection
edge_threshold1 = 50
edge_threshold2 = 150

# Dynamic tuning parameters
tune_step = 0.01  # Step size for focal length adjustments

# Crop parameters - to match RGB FOV to Depth FOV
# Since Depth FOV (87°x58°) > RGB FOV (69.4°x42.5°), we crop depth to RGB
crop_offset_x = 0  # Horizontal crop offset
crop_offset_y = 0  # Vertical crop offset
crop_tune_step = 2  # Pixels to adjust per keypress

# Crop scale adjustments (fine-tune the FOV-based crop size)
crop_scale_x = 1.0  # Horizontal crop scale multiplier
crop_scale_y = 1.0  # Vertical crop scale multiplier (adjust if depth is bigger/smaller than RGB)
crop_scale_step = 0.01  # Scale adjustment step

# Simple mode crop boundaries (pixel coordinates)
crop_top = 65
crop_bottom = 415
crop_left = 105
crop_right = 510

# Calculate focal lengths from Field of View (FOV)
# FOV data from camera specifications:
# RGB: H=69.4°, V=42.5°, D=77°
# Depth: H=87°, V=58°, D=95°

# Formula: focal_length = (image_dimension / 2) / tan(FOV / 2)
fov_rgb_h = np.deg2rad(69.4)  # Horizontal FOV in radians
fov_rgb_v = np.deg2rad(42.5)  # Vertical FOV in radians
fov_depth_h = np.deg2rad(87.0)  # Horizontal FOV in radians
fov_depth_v = np.deg2rad(58.0)  # Vertical FOV in radians

# Calculate focal lengths in pixels
# TUNING PARAMETERS: Adjust these multipliers if alignment is off
focal_scale_rgb = 1.0  # Try 0.9-1.1 if RGB is misaligned
focal_scale_depth = 1.0  # Try 0.9-1.1 if depth is misaligned

fx_rgb = (imageWidth / 2.0) / np.tan(fov_rgb_h / 2.0) * focal_scale_rgb
fy_rgb = (imageHeight / 2.0) / np.tan(fov_rgb_v / 2.0) * focal_scale_rgb
fx_depth = (imageWidth / 2.0) / np.tan(fov_depth_h / 2.0) * focal_scale_depth
fy_depth = (imageHeight / 2.0) / np.tan(fov_depth_v / 2.0) * focal_scale_depth

# Principal points (typically at image center)
cx_rgb = imageWidth / 2.0
cy_rgb = imageHeight / 2.0
cx_depth = imageWidth / 2.0
cy_depth = imageHeight / 2.0

# Build intrinsic matrices manually
K_rgb_manual = np.array([
    [fx_rgb, 0, cx_rgb],
    [0, fy_rgb, cy_rgb],
    [0, 0, 1]
], dtype=np.float64)

K_depth_manual = np.array([
    [fx_depth, 0, cx_depth],
    [0, fy_depth, cy_depth],
    [0, 0, 1]
], dtype=np.float64)

print("\n=== Calculated Intrinsics from FOV ===")
print(f"RGB Focal Lengths: fx={fx_rgb:.2f}, fy={fy_rgb:.2f} pixels")
print(f"RGB Principal Point: cx={cx_rgb:.2f}, cy={cy_rgb:.2f} pixels")
print(f"\nDepth Focal Lengths: fx={fx_depth:.2f}, fy={fy_depth:.2f} pixels")
print(f"Depth Principal Point: cx={cx_depth:.2f}, cy={cy_depth:.2f} pixels")
print("\nRGB Intrinsic Matrix:\n", K_rgb_manual)
print("\nDepth Intrinsic Matrix:\n", K_depth_manual)

# Initialize the RealSense camera for RGB and DEPTH (BOTH must be enabled!)
# myCamRGB = QCarRealSense(
#     mode='RGB, Depth',
#     # frameWidthRGB=imageWidth,
#     # frameHeightRGB=imageHeight,
#     # frameWidthDepth=imageWidth,
#     # frameHeightDepth=imageHeight,
#     # frameRateRGB=30,
#     # frameRateDepth=30,
#     # video3dPort=18805,
#     # Pass calculated focal lengths and principal points
#     focalLengthRGB=np.array([[fx_rgb/2], [fy_rgb/2]], dtype=np.float64),  # Divided by 2 because Camera3D doubles it
#     principlePointRGB=np.array([[cx_rgb], [cy_rgb]], dtype=np.float64),
#     focalLengthDepth=np.array([[fx_depth/2], [fy_depth/2]], dtype=np.float64),
#     principlePointDepth=np.array([[cx_depth], [cy_depth]], dtype=np.float64),
# )

myCamRGB = QCarRealSense(
    mode='RGB, Depth',
    # frameWidthRGB=imageWidth,
    # frameHeightRGB=imageHeight,
    # frameWidthDepth=imageWidth,
    # frameHeightDepth=imageHeight,
    # frameRateRGB=30,
    # frameRateDepth=30,
    # video3dPort=18805,
    # Pass calculated focal lengths and principal points
    focalLengthRGB=np.array([[fx_rgb/2], [fy_rgb/2]], dtype=np.float64),  # Divided by 2 because Camera3D doubles it
    principlePointRGB=np.array([[cx_rgb], [cy_rgb]], dtype=np.float64),
    focalLengthDepth=np.array([[fx_depth/2], [fy_depth/2]], dtype=np.float64),
    principlePointDepth=np.array([[cx_depth], [cy_depth]], dtype=np.float64),
    skewDepth=0.0,
    skewRGB=0.0,
    video3dPort=18805
)
# Try to get camera intrinsics and extrinsics
print("\n=== Camera Calibration Information ===")
try:
    K_rgb = myCamRGB.intrinsics_rgb()
    K_depth = myCamRGB.intrinsics_depth()
    E_rgb = myCamRGB.extrinsics_rgb()
    E_depth = myCamRGB.extrinsics_depth()
    
    print("\nRGB Intrinsics (from QCarRealSense):\n", K_rgb)
    print("\nDepth Intrinsics (from QCarRealSense):\n", K_depth)
    
    # Check if intrinsics are valid (not None values)
    use_camera_intrinsics = not (np.any(np.isnan(K_rgb)) or np.any(np.isnan(K_depth)))
    # use_camera_intrinsics = False  # Force to False for testing purposes

    if use_camera_intrinsics:
        print("\n✓ Using QCarRealSense intrinsics")
        # Extrinsics might still be None, so check
        if np.any(np.isnan(E_rgb)) or np.any(np.isnan(E_depth)):
            print("⚠ Extrinsics contain None/NaN - assuming cameras are co-located")
            depth_to_rgb_transform = np.eye(4)  # Identity transform (cameras at same position)
            use_intrinsics = True
        else:
            print("\nRGB Extrinsics:\n", E_rgb)
            print("\nDepth Extrinsics:\n", E_depth)
            depth_to_rgb_transform = E_rgb @ np.linalg.inv(E_depth)
            print("\nDepth-to-RGB Transformation:\n", depth_to_rgb_transform)
            use_intrinsics = True
    else:
        print("\n⚠ QCarRealSense intrinsics contain None/NaN. Using calculated intrinsics from FOV.")
        K_rgb = K_rgb_manual
        K_depth = K_depth_manual
        # Assume cameras are co-located (typical for RealSense D435i)
        depth_to_rgb_transform = np.eye(4)
        use_intrinsics = False
        
except Exception as e:
    print(f"\n⚠ Could not retrieve intrinsics/extrinsics: {e}")
    print("Using calculated intrinsics from FOV.")
    K_rgb = K_rgb_manual
    K_depth = K_depth_manual
    depth_to_rgb_transform = np.eye(4)
    use_intrinsics = False

print("=" * 50 + "\n")

# Print keyboard controls
print("=" * 70)
print("🎮 KEYBOARD CONTROLS:")
print("=" * 70)
print("VISUALIZATION MODES:")
print("  [0] - Edge Detection (RED=depth, GREEN=RGB, YELLOW=aligned)")
print("  [1] - Checkerboard (RGB/Depth alternating)")
print("  [2] - Contours (Cyan depth contours on RGB)")
print("  [3] - Overlay (Blended RGB+Depth)")
print("  [4] - All modes (side-by-side comparison)")
print("  [5] - RGB + Depth (side-by-side original)")
print("\nDYNAMIC TUNING:")
print("  [W] - Increase RGB focal scale (+0.01)")
print("  [S] - Decrease RGB focal scale (-0.01)")
print("  [A] - Decrease Depth focal scale (-0.01)")
print("  [D] - Increase Depth focal scale (+0.01)")
print("  [R] - Reset focal scales to 1.0")
print("  [M] - Toggle alignment mode (Intrinsics/Simple)")
print("\nCROP ALIGNMENT (Move depth relative to RGB):")
print("  [J] - Shift depth LEFT")
print("  [L] - Shift depth RIGHT")
print("  [I] - Shift depth UP")
print("  [K] - Shift depth DOWN")
print("\nCROP SIZE (Fix vertical/horizontal mismatch):")
print("  [U] - Increase vertical crop (make depth smaller vertically)")
print("  [O] - Decrease vertical crop (make depth bigger vertically)")
print("  [Y] - Increase horizontal crop (make depth smaller horizontally)")
print("  [P] - Decrease horizontal crop (make depth bigger horizontally)")
print("  [C] - Reset crop offset & scale to default")
print("\nEDGE DETECTION TUNING:")
print("  [+/=] - Increase edge threshold (+10)")
print("  [-/_] - Decrease edge threshold (-10)")
print("\nSAVE/LOAD SETTINGS:")
print("  [V] - Save current settings to JSON")
print("  [B] - Load settings from JSON")
print("\nOTHER:")
print("  [H] - Show help")
print("  [Q/ESC] - Quit")
print("=" * 70)
print()

# Calculate crop region based on FOV difference
# Depth FOV is larger, so we need to crop it to match RGB FOV
fov_ratio_h = np.tan(fov_rgb_h / 2.0) / np.tan(fov_depth_h / 2.0)
fov_ratio_v = np.tan(fov_rgb_v / 2.0) / np.tan(fov_depth_v / 2.0)

# Calculate the crop dimensions (with scale adjustments)
crop_width = int(imageWidth * fov_ratio_h * crop_scale_x)
crop_height = int(imageHeight * fov_ratio_v * crop_scale_y)

# Calculate initial crop offsets (centered)
crop_start_x = (imageWidth - crop_width) // 2
crop_start_y = (imageHeight - crop_height) // 2

print(f"📐 FOV-based Crop Calculation:")
print(f"   RGB FOV: {np.rad2deg(fov_rgb_h):.1f}° x {np.rad2deg(fov_rgb_v):.1f}°")
print(f"   Depth FOV: {np.rad2deg(fov_depth_h):.1f}° x {np.rad2deg(fov_depth_v):.1f}°")
print(f"   FOV Ratio: {fov_ratio_h:.3f} x {fov_ratio_v:.3f}")
print(f"   Crop scale: X={crop_scale_x:.3f}, Y={crop_scale_y:.3f}")
print(f"   Crop size: {crop_width}x{crop_height} from {imageWidth}x{imageHeight}")
print(f"   Initial crop region: x=[{crop_start_x}:{crop_start_x+crop_width}], y=[{crop_start_y}:{crop_start_y+crop_height}]")
print(f"   💡 If depth is vertically bigger than RGB, press [U] to increase vertical crop")
print(f"   💡 If depth is vertically smaller than RGB, press [O] to decrease vertical crop")
print()


def print_help():
    """Print keyboard controls help"""
    print("\n" + "=" * 70)
    print("🎮 KEYBOARD CONTROLS:")
    print("=" * 70)
    print("VISUALIZATION MODES:")
    print("  [0] - Edge Detection (RED=depth, GREEN=RGB, YELLOW=aligned)")
    print("  [1] - Checkerboard (RGB/Depth alternating)")
    print("  [2] - Contours (Cyan depth contours on RGB)")
    print("  [3] - Overlay (Blended RGB+Depth)")
    print("  [4] - All modes (side-by-side comparison)")
    print("  [5] - RGB + Depth (side-by-side original)")
    print("\nDYNAMIC TUNING:")
    print("  [W] - Increase RGB focal scale (+0.01)")
    print("  [S] - Decrease RGB focal scale (-0.01)")
    print("  [A] - Decrease Depth focal scale (-0.01)")
    print("  [D] - Increase Depth focal scale (+0.01)")
    print("  [R] - Reset focal scales to 1.0")    
    print("  [M] - Toggle alignment mode (Intrinsics/Simple)")    
    print("\nCROP ALIGNMENT (Move depth relative to RGB):")
    print("  [J] - Shift depth LEFT")
    print("  [L] - Shift depth RIGHT")
    print("  [I] - Shift depth UP")
    print("  [K] - Shift depth DOWN")
    print("\nCROP SIZE (Fix vertical/horizontal mismatch):")
    print("  [U] - Increase vertical crop (make depth smaller vertically)")
    print("  [O] - Decrease vertical crop (make depth bigger vertically)")
    print("  [Y] - Increase horizontal crop (make depth smaller horizontally)")
    print("  [P] - Decrease horizontal crop (make depth bigger horizontally)")
    print("  [C] - Reset crop offset & scale to default")
    print("\nEDGE DETECTION TUNING:")
    print("  [+/=] - Increase edge threshold (+10)")
    print("  [-/_] - Decrease edge threshold (-10)")
    print("\nSAVE/LOAD SETTINGS:")
    print("  [V] - Save current settings to JSON")
    print("  [B] - Load settings from JSON")
    print("\nOTHER:")
    print("  [H] - Show this help")
    print("  [Q/ESC] - Quit")
    print("=" * 70 + "\n")

try:
    startTime = time.time()
    iteration = 0
    mode_names = ["Edge Detection", "Checkerboard", "Contours", "Overlay", "All Modes", "RGB+Depth"]
    
    # FPS tracking
    fps_start_time = time.time()
    fps_frame_count = 0
    current_fps = 0.0
    
    while elapsed_time()<simulationTime:
        start = time.time()
        iteration += 1
        
        # Read the RGB and Depth frames
        myCamRGB.read_RGB()
        myCamRGB.read_depth(dataMode='PX')  # 'PX' for pixel values (uint8 0-255)
        
        # Get the images from the camera buffers
        color_image = myCamRGB.imageBufferRGB.copy()
        depth_image_px = myCamRGB.imageBufferDepthPX.copy().squeeze()  # Use pixel buffer
        
        # Convert pixel depth to approximate meters (this is device-specific scaling)
        # Typical RealSense: pixel value represents depth, needs scaling
        # You may need to adjust this scale factor based on your camera
        depth_scale = 0.001  # Common scale: 1 pixel unit = 1mm = 0.001m
        depth_image_m = depth_image_px.astype(np.float32) * depth_scale
        
        # ===== CROP DEPTH TO MATCH RGB FOV =====
        # Recalculate crop dimensions (in case scale was adjusted)
        crop_width = int(imageWidth * fov_ratio_h * crop_scale_x)
        crop_height = int(imageHeight * fov_ratio_v * crop_scale_y)
        crop_start_x = (imageWidth - crop_width) // 2
        crop_start_y = (imageHeight - crop_height) // 2
        
        # Apply user-adjustable crop offset
        actual_crop_x = crop_start_x + crop_offset_x
        actual_crop_y = crop_start_y + crop_offset_y
        
        # Ensure crop region is within bounds
        actual_crop_x = max(0, min(actual_crop_x, imageWidth - crop_width))
        actual_crop_y = max(0, min(actual_crop_y, imageHeight - crop_height))
        
        # Crop depth image to match RGB FOV
        depth_image_m_cropped = depth_image_m[
            actual_crop_y:actual_crop_y+crop_height,
            actual_crop_x:actual_crop_x+crop_width
        ]
        
        # Resize cropped depth to full resolution for alignment
        depth_image_m_cropped_resized = cv2.resize(
            depth_image_m_cropped, 
            (imageWidth, imageHeight), 
            interpolation=cv2.INTER_LINEAR
        )
        
        # Debug: Print depth statistics every 100 frames
        if iteration % 100 == 1:
            print(f"\n=== Frame {iteration} Debug Info ===")
            print(f"RGB shape: {color_image.shape}")
            print(f"Depth (pixels) - shape: {depth_image_px.shape}, dtype: {depth_image_px.dtype}")
            print(f"Depth CROPPED - shape: {depth_image_m_cropped.shape} (from crop region x=[{actual_crop_x}:{actual_crop_x+crop_width}], y=[{actual_crop_y}:{actual_crop_y+crop_height}])")
            print(f"Crop offset: x={crop_offset_x}, y={crop_offset_y}")
            print(f"Depth pixel min: {depth_image_px.min()}, max: {depth_image_px.max()}")
            print(f"Depth pixel mean: {depth_image_px.mean():.1f}")
            print(f"Depth meters min: {depth_image_m_cropped_resized.min():.3f}m, max: {depth_image_m_cropped_resized.max():.3f}m")
            print(f"Depth meters mean: {depth_image_m_cropped_resized.mean():.3f}m")
            print(f"Non-zero depth pixels: {np.count_nonzero(depth_image_m_cropped_resized)} / {depth_image_m_cropped_resized.size}")
            
            # Suggest better clipping distance based on actual data
            if depth_image_m_cropped_resized.max() > 0:
                suggested_clip = max(0.5, depth_image_m_cropped_resized.max() * 1.2)  # 20% above max
                print(f"💡 Suggested clipping distance: {suggested_clip:.2f}m (current: {clipping_distance_in_meters:.2f}m)")

        
        # Align depth to RGB using appropriate method (use CROPPED depth)
        if use_intrinsics:
            # Use intrinsic/extrinsic-based alignment
            depth_image_m_aligned = align_depth_to_rgb_using_intrinsics(
                depth_image_m_cropped_resized, 
                (imageHeight, imageWidth),
                K_rgb, 
                K_depth, 
                depth_to_rgb_transform
            )
        else:
            # Fallback to simple crop and resize alignment
            crop_bounds = {
                'top': crop_top,
                'bottom': crop_bottom,
                'left': crop_left,
                'right': crop_right
            }
            depth_image_m_aligned = simple_align_depth_to_rgb(
                depth_image_m,  # Use original depth, not cropped
                (imageHeight, imageWidth),
                crop_bounds
            )
        
        # Display alignment info every 100 frames (reduced frequency for performance)
        if iteration % 100 == 1:
            print(f"Aligned shape: {depth_image_m_aligned.shape}")
            print(f"Aligned min: {depth_image_m_aligned.min():.3f}m, max: {depth_image_m_aligned.max():.3f}m")
            print(f"Non-zero aligned pixels: {np.count_nonzero(depth_image_m_aligned)} / {depth_image_m_aligned.size}")


        
        # Convert depth to uint8 for visualization using the aligned depth
        # Use adaptive scaling based on actual depth range for better visualization
        valid_depth = depth_image_m_aligned[depth_image_m_aligned > 0]
        if len(valid_depth) > 0:
            depth_min = valid_depth.min()
            depth_max = valid_depth.max()
            # Normalize to full 0-255 range for better color distribution
            depth_image_normalized = np.zeros_like(depth_image_m_aligned, dtype=np.uint8)
            mask = depth_image_m_aligned > 0
            depth_image_normalized[mask] = np.clip(
                (depth_image_m_aligned[mask] - depth_min) * 255.0 / (depth_max - depth_min + 1e-6), 
                0, 255
            ).astype(np.uint8)
        else:
            # Fallback if no valid depth
            depth_image_normalized = np.clip(depth_image_m_aligned * 255.0 / clipping_distance_in_meters, 0, 255).astype(np.uint8)
            depth_min = 0
            depth_max = 0
        
        # Apply colormap to depth for better visualization (JET: blue=close, red=far)
        depth_colormap = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)
        
        # Create RGB-Depth overlay (blend RGB with depth colormap)
        alpha = 0.6  # RGB weight (0.6 = 60% RGB, 40% depth colormap)
        overlay = cv2.addWeighted(color_image, alpha, depth_colormap, 1-alpha, 0)
        
        # Create depth overlay on RGB (show depth only where valid)
        depth_overlay = color_image.copy()
        valid_mask = depth_image_m_aligned > 0
        if valid_mask.any():
            depth_overlay[valid_mask] = cv2.addWeighted(
                color_image[valid_mask], 
                0.5, 
                depth_colormap[valid_mask], 
                0.5, 
                0
            )
        
        # ===== ALIGNMENT VISUALIZATION =====
        # Only compute the visualization mode that's actually being displayed for better performance
        
        if visualization_mode == 0:
            # 1. Edge Detection mode
            depth_edges = cv2.Canny(depth_image_normalized, edge_threshold1, edge_threshold2)
            rgb_gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            rgb_edges = cv2.Canny(rgb_gray, edge_threshold1, edge_threshold2)
            
            edge_overlay = color_image.copy()
            edge_overlay[depth_edges > 0] = [0, 0, 255]  # Red for depth edges
            edge_overlay[rgb_edges > 0] = [0, 255, 0]  # Green for RGB edges
            display_base = edge_overlay
            
        elif visualization_mode == 1:
            # 2. Checkerboard pattern
            checkerboard = color_image.copy()
            block_size = 40
            for i in range(0, imageHeight, block_size):
                for j in range(0, imageWidth, block_size):
                    if (i // block_size + j // block_size) % 2 == 0:
                        checkerboard[i:i+block_size, j:j+block_size] = depth_colormap[i:i+block_size, j:j+block_size]
            display_base = checkerboard
            
        elif visualization_mode == 2:
            # 3. Contour overlay - IMPROVED
            contour_overlay = color_image.copy()
            # Use bilateral filter to smooth while preserving edges
            depth_smooth = cv2.bilateralFilter(depth_image_normalized, 5, 50, 50)
            # Use multiple thresholds for better contour detection
            _, thresh1 = cv2.threshold(depth_smooth, 50, 255, cv2.THRESH_BINARY)
            _, thresh2 = cv2.threshold(depth_smooth, 150, 255, cv2.THRESH_BINARY)
            
            # Find contours at different depth levels
            contours1, _ = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours2, _ = cv2.findContours(thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw contours with different colors for different depths
            cv2.drawContours(contour_overlay, contours1, -1, (0, 255, 255), 1)  # Yellow for far
            cv2.drawContours(contour_overlay, contours2, -1, (255, 0, 255), 2)  # Magenta for near
            display_base = contour_overlay
            
        elif visualization_mode == 3:
            # 4. Overlay mode
            alpha = 0.6
            overlay = cv2.addWeighted(color_image, alpha, depth_colormap, 1-alpha, 0)
            display_base = overlay
            
        elif visualization_mode == 4:
            # 5. All modes - compute all
            depth_edges = cv2.Canny(depth_image_normalized, edge_threshold1, edge_threshold2)
            rgb_gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            rgb_edges = cv2.Canny(rgb_gray, edge_threshold1, edge_threshold2)
            edge_overlay = color_image.copy()
            edge_overlay[depth_edges > 0] = [0, 0, 255]
            edge_overlay[rgb_edges > 0] = [0, 255, 0]
            
            checkerboard = color_image.copy()
            block_size = 40
            for i in range(0, imageHeight, block_size):
                for j in range(0, imageWidth, block_size):
                    if (i // block_size + j // block_size) % 2 == 0:
                        checkerboard[i:i+block_size, j:j+block_size] = depth_colormap[i:i+block_size, j:j+block_size]
            
            contour_overlay = color_image.copy()
            depth_smooth = cv2.bilateralFilter(depth_image_normalized, 5, 50, 50)
            _, thresh = cv2.threshold(depth_smooth, 100, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(contour_overlay, contours, -1, (255, 255, 0), 1)
            
            alpha = 0.6
            overlay = cv2.addWeighted(color_image, alpha, depth_colormap, 1-alpha, 0)
            
            # Create 2x2 grid
            top_row = np.hstack((
                cv2.resize(edge_overlay, (320, 240)),
                cv2.resize(checkerboard, (320, 240))
            ))
            bottom_row = np.hstack((
                cv2.resize(contour_overlay, (320, 240)),
                cv2.resize(overlay, (320, 240))
            ))
            display_base = np.vstack((top_row, bottom_row))
            
        else:  # visualization_mode == 5
            # 6. RGB + Depth side-by-side
            display_base = np.hstack((color_image, depth_colormap))
        
        # Add info text overlay function with FPS
        def add_info_text(img, mode_name):
            img_with_text = img.copy()
            # Compact overlay - top right corner
            overlay_width = 240
            overlay_height = 120  # Increased for alignment mode line
            margin = 5
            x_start = img.shape[1] - overlay_width - margin
            y_start = margin
            
            # Semi-transparent background
            overlay_bg = img_with_text[y_start:y_start+overlay_height, x_start:x_start+overlay_width].copy()
            cv2.rectangle(img_with_text, (x_start, y_start), (x_start+overlay_width, y_start+overlay_height), (0, 0, 0), -1)
            cv2.addWeighted(overlay_bg, 0.3, img_with_text[y_start:y_start+overlay_height, x_start:x_start+overlay_width], 0.7, 0, img_with_text[y_start:y_start+overlay_height, x_start:x_start+overlay_width])
            
            # Border
            cv2.rectangle(img_with_text, (x_start, y_start), (x_start+overlay_width, y_start+overlay_height), (0, 255, 0), 1)
            
            # Compact text info
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.35
            thickness = 1
            line_height = 15
            
            y_offset = y_start + 15
            # FPS display with color coding
            fps_color = (0, 255, 0) if current_fps >= 30 else (0, 165, 255) if current_fps >= 20 else (0, 0, 255)
            cv2.putText(img_with_text, f"FPS:{current_fps:.1f} Mode:{mode_name[:8]}", (x_start+5, y_offset), font, font_scale, fps_color, thickness)
            y_offset += line_height
            cv2.putText(img_with_text, f"Focal:RGB={focal_scale_rgb:.2f} D={focal_scale_depth:.2f}", (x_start+5, y_offset), font, font_scale, (255, 255, 255), thickness)
            y_offset += line_height
            cv2.putText(img_with_text, f"Crop:X={crop_offset_x} Y={crop_offset_y}", (x_start+5, y_offset), font, font_scale, (255, 255, 255), thickness)
            y_offset += line_height
            cv2.putText(img_with_text, f"Scale:X={crop_scale_x:.2f} Y={crop_scale_y:.2f}", (x_start+5, y_offset), font, font_scale, (255, 255, 255), thickness)
            y_offset += line_height
            cv2.putText(img_with_text, f"Edge:{edge_threshold1}/{edge_threshold2}", (x_start+5, y_offset), font, font_scale, (255, 255, 255), thickness)
            y_offset += line_height
            align_mode_text = "Intrinsics" if use_intrinsics else "Simple"
            align_mode_color = (0, 255, 0) if use_intrinsics else (0, 165, 255)
            cv2.putText(img_with_text, f"Align:{align_mode_text} [M]:Toggle", (x_start+5, y_offset), font, font_scale, align_mode_color, thickness)
            y_offset += line_height
            cv2.putText(img_with_text, f"[H]:Help [V]:Save [B]:Load", (x_start+5, y_offset), font, font_scale, (255, 200, 0), thickness)
            
            return img_with_text
        
        # Display the selected mode
        display_img = add_info_text(display_base, mode_names[visualization_mode])
        cv2.namedWindow('Alignment Viewer', cv2.WINDOW_NORMAL)
        cv2.imshow('Alignment Viewer', display_img)
        
        # Calculate FPS
        fps_frame_count += 1
        if fps_frame_count >= 10:  # Update FPS every 10 frames
            fps_end_time = time.time()
            current_fps = fps_frame_count / (fps_end_time - fps_start_time)
            fps_start_time = fps_end_time
            fps_frame_count = 0
        
        # Print status every 50 frames
        if iteration % 50 == 1:
            print(f"\n{'='*70}")
            print(f"📊 Frame {iteration} | Mode: {mode_names[visualization_mode]}")
            print(f"{'='*70}")
            if len(valid_depth) > 0:
                print(f"Depth range: {depth_min:.2f}m - {depth_max:.2f}m")
                print(f"RGB focal scale: {focal_scale_rgb:.3f} | Depth focal scale: {focal_scale_depth:.3f}")
                print(f"RGB focal: ({fx_rgb:.1f}, {fy_rgb:.1f}) | Depth focal: ({fx_depth:.1f}, {fy_depth:.1f})")
                print(f"Crop offset: X={crop_offset_x}, Y={crop_offset_y}")
                print(f"Crop scale: X={crop_scale_x:.3f}, Y={crop_scale_y:.3f}")
                print(f"Crop size: {crop_width}x{crop_height}")
                print(f"Crop region: x=[{actual_crop_x}:{actual_crop_x+crop_width}], y=[{actual_crop_y}:{actual_crop_y+crop_height}]")
                print(f"Edge thresholds: {edge_threshold1}/{edge_threshold2}")
                print(f"💡 TIP: If depth vertically bigger than RGB, press [U]. If smaller, press [O]")
                print(f"💡 Use [I/K/J/L] to shift depth position until edges align (Yellow in mode 0)")

        
        # Create background removed image using the ALIGNED depth
        grey_color = 153
        depth_image_3d = np.dstack((depth_image_m_aligned, depth_image_m_aligned, depth_image_m_aligned))
        bg_removed = np.where(
            (depth_image_3d > clipping_distance_in_meters) | (depth_image_3d <= 0), 
            grey_color, 
            color_image
        )
        
        # Stack images horizontally: background removed (left) and depth colormap (right)
        images = np.hstack((bg_removed.astype(np.uint8), depth_colormap))

        # End timing this iteration
        end = time.time()

        # Calculate the computation time, and the time that the thread should pause/sleep for
        computationTime = end - start
        sleepTime = sampleTime - ( computationTime % sampleTime )

        # Pause/sleep for sleepTime in milliseconds
        msSleepTime = int(1000*sleepTime)
        if msSleepTime <= 0:
            msSleepTime = 1
        
        # Check for key press - MODE SELECTION AND DYNAMIC TUNING
        key = cv2.waitKey(msSleepTime)
        
        # # Debug: Print key code when any key is pressed (comment out after testing)
        # if key != -1:
        #     print(f"\n[DEBUG] Key pressed: {key} | Masked (key & 0xFF): {key & 0xFF}")
        
        # Quit keys
        if key & 0xFF == ord('q') or key == 27:
            print("Exiting...")
            break
        
        # Help key
        elif key & 0xFF == ord('h') or key & 0xFF == ord('H'):
            print_help()
        
        # Mode selection (0-5)
        elif key & 0xFF == ord('0'):
            visualization_mode = 0
            print(f"\n✓ Switched to mode: {mode_names[0]}")
        elif key & 0xFF == ord('1'):
            visualization_mode = 1
            print(f"\n✓ Switched to mode: {mode_names[1]}")
        elif key & 0xFF == ord('2'):
            visualization_mode = 2
            print(f"\n✓ Switched to mode: {mode_names[2]}")
        elif key & 0xFF == ord('3'):
            visualization_mode = 3
            print(f"\n✓ Switched to mode: {mode_names[3]}")
        elif key & 0xFF == ord('4'):
            visualization_mode = 4
            print(f"\n✓ Switched to mode: {mode_names[4]}")
        elif key & 0xFF == ord('5'):
            visualization_mode = 5
            print(f"\n✓ Switched to mode: {mode_names[5]}")
        
        # Dynamic tuning - RGB focal scale
        elif key & 0xFF == ord('w') or key & 0xFF == ord('W'):
            focal_scale_rgb += tune_step
            fx_rgb = (imageWidth / 2.0) / np.tan(fov_rgb_h / 2.0) * focal_scale_rgb
            fy_rgb = (imageHeight / 2.0) / np.tan(fov_rgb_v / 2.0) * focal_scale_rgb
            K_rgb[0, 0] = fx_rgb
            K_rgb[1, 1] = fy_rgb
            print(f"\n⬆ RGB focal scale: {focal_scale_rgb:.3f} (fx={fx_rgb:.1f}, fy={fy_rgb:.1f})")
        elif key & 0xFF == ord('s') or key & 0xFF == ord('S'):
            focal_scale_rgb -= tune_step
            fx_rgb = (imageWidth / 2.0) / np.tan(fov_rgb_h / 2.0) * focal_scale_rgb
            fy_rgb = (imageHeight / 2.0) / np.tan(fov_rgb_v / 2.0) * focal_scale_rgb
            K_rgb[0, 0] = fx_rgb
            K_rgb[1, 1] = fy_rgb
            print(f"\n⬇ RGB focal scale: {focal_scale_rgb:.3f} (fx={fx_rgb:.1f}, fy={fy_rgb:.1f})")
        
        # Dynamic tuning - Depth focal scale
        elif key & 0xFF == ord('d') or key & 0xFF == ord('D'):
            focal_scale_depth += tune_step
            fx_depth = (imageWidth / 2.0) / np.tan(fov_depth_h / 2.0) * focal_scale_depth
            fy_depth = (imageHeight / 2.0) / np.tan(fov_depth_v / 2.0) * focal_scale_depth
            K_depth[0, 0] = fx_depth
            K_depth[1, 1] = fy_depth
            print(f"\n⬆ Depth focal scale: {focal_scale_depth:.3f} (fx={fx_depth:.1f}, fy={fy_depth:.1f})")
        elif key & 0xFF == ord('a') or key & 0xFF == ord('A'):
            focal_scale_depth -= tune_step
            fx_depth = (imageWidth / 2.0) / np.tan(fov_depth_h / 2.0) * focal_scale_depth
            fy_depth = (imageHeight / 2.0) / np.tan(fov_depth_v / 2.0) * focal_scale_depth
            K_depth[0, 0] = fx_depth
            K_depth[1, 1] = fy_depth
            print(f"\n⬇ Depth focal scale: {focal_scale_depth:.3f} (fx={fx_depth:.1f}, fy={fy_depth:.1f})")
        
        # Reset focal scales
        elif key & 0xFF == ord('r') or key & 0xFF == ord('R'):
            focal_scale_rgb = 1.0
            focal_scale_depth = 1.0
            fx_rgb = (imageWidth / 2.0) / np.tan(fov_rgb_h / 2.0)
            fy_rgb = (imageHeight / 2.0) / np.tan(fov_rgb_v / 2.0)
            fx_depth = (imageWidth / 2.0) / np.tan(fov_depth_h / 2.0)
            fy_depth = (imageHeight / 2.0) / np.tan(fov_depth_v / 2.0)
            K_rgb[0, 0] = fx_rgb
            K_rgb[1, 1] = fy_rgb
            K_depth[0, 0] = fx_depth
            K_depth[1, 1] = fy_depth
            print(f"\n🔄 RESET focal scales to 1.0")
        
        # Toggle alignment mode
        elif key & 0xFF == ord('m') or key & 0xFF == ord('M'):
            use_intrinsics = not use_intrinsics
            mode_name = "Camera Intrinsics" if use_intrinsics else "Simple Resize"
            print(f"\n🔀 Alignment mode: {mode_name}")
        
        # Reset crop offset
        elif key & 0xFF == ord('c') or key & 0xFF == ord('C'):
            crop_offset_x = 0
            crop_offset_y = 0
            crop_scale_x = 1.0
            crop_scale_y = 1.0
            print(f"\n🔄 RESET crop offset & scale to default")
        
        # Crop position adjustment (IJKL keys only - arrow keys removed)
        elif key & 0xFF == ord('j') or key & 0xFF == ord('J'):
            crop_offset_x -= crop_tune_step
            print(f"\n← Crop shifted LEFT | Offset: X={crop_offset_x}, Y={crop_offset_y}")
        elif key & 0xFF == ord('l') or key & 0xFF == ord('L'):
            crop_offset_x += crop_tune_step
            print(f"\n→ Crop shifted RIGHT | Offset: X={crop_offset_x}, Y={crop_offset_y}")
        elif key & 0xFF == ord('i') or key & 0xFF == ord('I'):
            crop_offset_y -= crop_tune_step
            print(f"\n↑ Crop shifted UP | Offset: X={crop_offset_x}, Y={crop_offset_y}")
        elif key & 0xFF == ord('k') or key & 0xFF == ord('K'):
            crop_offset_y += crop_tune_step
            print(f"\n↓ Crop shifted DOWN | Offset: X={crop_offset_x}, Y={crop_offset_y}")
        
        # Crop SCALE adjustment (fix vertical/horizontal size mismatch)
        elif key & 0xFF == ord('u') or key & 0xFF == ord('U'):
            crop_scale_y += crop_scale_step
            print(f"\n📏 Vertical crop INCREASED (depth smaller) | Crop scale: X={crop_scale_x:.3f}, Y={crop_scale_y:.3f}")
        elif key & 0xFF == ord('o') or key & 0xFF == ord('O'):
            crop_scale_y = max(0.5, crop_scale_y - crop_scale_step)
            print(f"\n📏 Vertical crop DECREASED (depth bigger) | Crop scale: X={crop_scale_x:.3f}, Y={crop_scale_y:.3f}")
        elif key & 0xFF == ord('y') or key & 0xFF == ord('Y'):
            crop_scale_x += crop_scale_step
            print(f"\n📏 Horizontal crop INCREASED (depth smaller) | Crop scale: X={crop_scale_x:.3f}, Y={crop_scale_y:.3f}")
        elif key & 0xFF == ord('p') or key & 0xFF == ord('P'):
            crop_scale_x = max(0.5, crop_scale_x - crop_scale_step)
            print(f"\n📏 Horizontal crop DECREASED (depth bigger) | Crop scale: X={crop_scale_x:.3f}, Y={crop_scale_y:.3f}")
        
        # Edge threshold tuning with +/- keys
        elif key & 0xFF == ord('+') or key & 0xFF == ord('='):
            edge_threshold1 += 10
            edge_threshold2 += 10
            print(f"\n⬆ Edge thresholds: {edge_threshold1}/{edge_threshold2}")
        elif key & 0xFF == ord('-') or key & 0xFF == ord('_'):
            edge_threshold1 = max(10, edge_threshold1 - 10)
            edge_threshold2 = max(20, edge_threshold2 - 10)
            print(f"\n⬇ Edge thresholds: {edge_threshold1}/{edge_threshold2}")
        
        # Save/Load settings
        elif key & 0xFF == ord('v') or key & 0xFF == ord('V'):
            save_alignment_settings()
        elif key & 0xFF == ord('b') or key & 0xFF == ord('B'):
            load_alignment_settings()


except KeyboardInterrupt:
    print("User interrupted! (Ctrl+C)")

finally:
    myCamRGB.terminate()


