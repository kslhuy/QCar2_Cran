import numpy as np
import cv2
import time
from pit.LaneNet.nets import LaneNet
from pal.utilities.vision import Camera3D
from pal.products.qcar import QCarRealSense


## Timing Parameters and methods 
def elapsed_time():
    return time.time() - startTime

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

def simple_align_depth_to_rgb(depth_image, target_shape):
    """
    Simple alignment using resize with aspect ratio considerations.
    """
    # Resize depth to match RGB dimensions
    depth_resized = cv2.resize(depth_image, (target_shape[1], target_shape[0]), interpolation=cv2.INTER_NEAREST)
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
fx_rgb = (imageWidth / 2.0) / np.tan(fov_rgb_h / 2.0)
fy_rgb = (imageHeight / 2.0) / np.tan(fov_rgb_v / 2.0)
fx_depth = (imageWidth / 2.0) / np.tan(fov_depth_h / 2.0)
fy_depth = (imageHeight / 2.0) / np.tan(fov_depth_v / 2.0)

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
myCamRGB = QCarRealSense(
    mode='RGB, Depth',
    frameWidthRGB=imageWidth,
    frameHeightRGB=imageHeight,
    frameWidthDepth=imageWidth,
    frameHeightDepth=imageHeight,
    # frameRateRGB=30,
    # frameRateDepth=30,
    # video3dPort=18805,
    # Pass calculated focal lengths and principal points
    focalLengthRGB=np.array([[fx_rgb/2], [fy_rgb/2]], dtype=np.float64),  # Divided by 2 because Camera3D doubles it
    principlePointRGB=np.array([[cx_rgb], [cy_rgb]], dtype=np.float64),
    focalLengthDepth=np.array([[fx_depth/2], [fy_depth/2]], dtype=np.float64),
    principlePointDepth=np.array([[cx_depth], [cy_depth]], dtype=np.float64),
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
        use_intrinsics = True
        
except Exception as e:
    print(f"\n⚠ Could not retrieve intrinsics/extrinsics: {e}")
    print("Using calculated intrinsics from FOV.")
    K_rgb = K_rgb_manual
    K_depth = K_depth_manual
    depth_to_rgb_transform = np.eye(4)
    use_intrinsics = True

print("=" * 50 + "\n")


try:
    startTime = time.time()
    iteration = 0
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
        
        # Debug: Print depth statistics every 30 frames
        if iteration % 30 == 1:
            print(f"\n=== Frame {iteration} Debug Info ===")
            print(f"RGB shape: {color_image.shape}")
            print(f"Depth (pixels) - shape: {depth_image_px.shape}, dtype: {depth_image_px.dtype}")
            print(f"Depth pixel min: {depth_image_px.min()}, max: {depth_image_px.max()}")
            print(f"Depth pixel mean: {depth_image_px.mean():.1f}")
            print(f"Depth meters min: {depth_image_m.min():.3f}m, max: {depth_image_m.max():.3f}m")
            print(f"Depth meters mean: {depth_image_m.mean():.3f}m")
            print(f"Non-zero depth pixels: {np.count_nonzero(depth_image_m)} / {depth_image_m.size}")
            
            # Suggest better clipping distance based on actual data
            if depth_image_m.max() > 0:
                suggested_clip = max(0.5, depth_image_m.max() * 1.2)  # 20% above max
                print(f"💡 Suggested clipping distance: {suggested_clip:.2f}m (current: {clipping_distance_in_meters:.2f}m)")

        
        # Align depth to RGB using appropriate method
        if use_intrinsics:
            # Use intrinsic/extrinsic-based alignment
            depth_image_m_aligned = align_depth_to_rgb_using_intrinsics(
                depth_image_m, 
                (imageHeight, imageWidth),
                K_rgb, 
                K_depth, 
                depth_to_rgb_transform
            )
        else:
            # Fallback to simple resize-based alignment
            depth_image_m_aligned = simple_align_depth_to_rgb(
                depth_image_m,
                (imageHeight, imageWidth)
            )
        
        # Display alignment info every 30 frames
        if iteration % 30 == 1:
            print(f"Aligned shape: {depth_image_m_aligned.shape}")
            print(f"Aligned min: {depth_image_m_aligned.min():.3f}m, max: {depth_image_m_aligned.max():.3f}m")
            print(f"Aligned mean: {depth_image_m_aligned.mean():.3f}m")
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
        
        # Display multiple visualizations
        # 1. RGB + Depth Overlay (blended)
        cv2.namedWindow('RGB-Depth Overlay (Blended)', cv2.WINDOW_NORMAL)
        cv2.imshow('RGB-Depth Overlay (Blended)', overlay)
        
        # 2. Side by side comparison
        cv2.namedWindow('Comparison: BG Removed | Depth Colormap', cv2.WINDOW_NORMAL)
        cv2.imshow('Comparison: BG Removed | Depth Colormap', images)
        
        # 3. RGB with depth colormap only on valid depth regions
        cv2.namedWindow('Depth Overlay on RGB', cv2.WINDOW_NORMAL)
        cv2.imshow('Depth Overlay on RGB', depth_overlay)
        
        # 4. Original streams
        cv2.namedWindow('RGB Stream', cv2.WINDOW_NORMAL)
        cv2.imshow('RGB Stream', color_image)
        
        cv2.namedWindow('Depth Colormap', cv2.WINDOW_NORMAL)
        cv2.imshow('Depth Colormap', depth_colormap)

        # Add info text to overlay window
        if iteration % 30 == 1 and len(valid_depth) > 0:
            info_text = f"Depth range: {depth_min:.2f}m - {depth_max:.2f}m | JET: Blue=Near, Red=Far"
            print(f"📊 {info_text}")

        # End timing this iteration
        end = time.time()

        # Calculate the computation time, and the time that the thread should pause/sleep for
        computationTime = end - start
        sleepTime = sampleTime - ( computationTime % sampleTime )

        # Pause/sleep for sleepTime in milliseconds
        msSleepTime = int(1000*sleepTime)
        if msSleepTime <= 0:
            msSleepTime = 1
        
        # Check for key press to exit (q, ESC, or Ctrl+C)
        key = cv2.waitKey(msSleepTime)
        if key & 0xFF == ord('q') or key == 27:
            print("Exiting...")
            break

except KeyboardInterrupt:
    print("User interrupted! (Ctrl+C)")

finally:
    myCamRGB.terminate()


