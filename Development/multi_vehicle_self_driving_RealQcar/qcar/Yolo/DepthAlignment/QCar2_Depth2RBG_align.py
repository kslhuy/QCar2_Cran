import numpy as np
import cv2
import time
import json
import os
from QCar2DepthAlignedCamera import QCar2DepthAlignedCamera


## Timing Parameters and methods 
def elapsed_time():
    return time.time() - startTime

# Note: Alignment functions now handled by QCar2DepthAlignedCamera class

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

tune_step = 0.01  # Step size for focal length adjustments
principal_point_step = 1.0  # Pixels to shift principal point (cx, cy)
crop_tune_step = 2  # Pixels to move crop offset (simple mode only)
crop_scale_step = 0.01  # Step size for crop scale adjustments
# Alignment mode: True=intrinsics, False=simple
use_intrinsics = True

# Initialize the depth-aligned camera using the new class
print("\n=== Initializing QCar2 Depth Aligned Camera ===")
camera = QCar2DepthAlignedCamera(
    imageWidth=imageWidth,
    imageHeight=imageHeight,
    use_intrinsics=use_intrinsics,
    clipping_distance=clipping_distance_in_meters,
    video3dPort=18805,
    load_settings=True
)

# Note: Camera initialization and intrinsics are now handled by QCar2DepthAlignedCamera class


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
    print("  [6] - Depth with Crop Region (calibration view)")
    print("\nDYNAMIC TUNING:")
    print("  [W] - Increase RGB focal scale (+0.01)")
    print("  [S] - Decrease RGB focal scale (-0.01)")
    print("  [A] - Decrease Depth focal scale (-0.01)")
    print("  [D] - Increase Depth focal scale (+0.01)")
    print("  [R] - Reset focal scales to 1.0")    
    print("  [M] - Toggle alignment mode (Intrinsics/Simple)")    
    print("\nALIGNMENT SHIFT (Intrinsics mode uses principal point):")
    print("  [J] - Shift depth LEFT")
    print("  [L] - Shift depth RIGHT")
    print("  [I] - Shift depth UP")
    print("  [K] - Shift depth DOWN")
    print("\nCROP SIZE (Adjust depth FOV/crop region):")
    print("  [U] - Increase vertical crop (make depth smaller vertically)")
    print("  [O] - Decrease vertical crop (make depth bigger vertically)")
    print("  [Y] - Increase horizontal crop (make depth smaller horizontally)")
    print("  [P] - Decrease horizontal crop (make depth bigger horizontally)")
    print("  [C] - Reset alignment parameters to default")
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
    mode_names = ["Edge Detection", "Checkerboard", "Contours", "Overlay", "All Modes", "RGB+Depth", "Depth Crop"]
    
    # FPS tracking
    fps_start_time = time.time()
    fps_frame_count = 0
    current_fps = 0.0
    
    while elapsed_time()<simulationTime:
        start = time.time()
        iteration += 1
        
        # Read RGB and aligned Depth using the camera class
        camera.read()
        
        # Get the aligned images from camera
        color_image = camera.rgb
        depth_image_m_aligned = camera.depth  # Already aligned in meters
        depth_colormap = camera.depth_colormap  # Pre-computed colormap
        
        # # Debug: Print depth statistics every 100 frames
        # if iteration % 100 == 1:
        #     print(f"\n=== Frame {iteration} Debug Info ===")
        #     print(f"RGB shape: {color_image.shape}")
        #     print(f"Aligned Depth shape: {depth_image_m_aligned.shape}")
        #     print(f"Depth min: {depth_image_m_aligned.min():.3f}m, max: {depth_image_m_aligned.max():.3f}m")
        #     print(f"Depth mean: {depth_image_m_aligned.mean():.3f}m")
        #     print(f"Non-zero depth pixels: {np.count_nonzero(depth_image_m_aligned)} / {depth_image_m_aligned.size}")
        #     print(f"Camera mode: {'Intrinsics' if camera.use_intrinsics else 'Simple'}")
        #     if camera.use_intrinsics:
        #         print(f"Focal scales: RGB={camera.focal_scale_rgb:.3f}, Depth={camera.focal_scale_depth:.3f}")
        #         print(f"Crop offset: X={camera.crop_offset_x}, Y={camera.crop_offset_y}")
        #         print(f"Crop scale: X={camera.crop_scale_x:.3f}, Y={camera.crop_scale_y:.3f}")
        #     else:
        #         print(f"Crop boundaries: Top={camera.crop_top}, Bottom={camera.crop_bottom}, Left={camera.crop_left}, Right={camera.crop_right}")
            
        #     # Suggest better clipping distance based on actual data
        #     if depth_image_m_aligned.max() > 0:
        #         suggested_clip = max(0.5, depth_image_m_aligned.max() * 1.2)
        #         print(f"💡 Suggested clipping distance: {suggested_clip:.2f}m (current: {clipping_distance_in_meters:.2f}m)")
            

        # ===== ALIGNMENT VISUALIZATION =====
        # Only compute the visualization mode that's actually being displayed for better performance
        # Depth is already aligned and colormap is already created by the camera class
        
        # Convert depth to uint8 for edge detection if needed
        valid_depth = depth_image_m_aligned[depth_image_m_aligned > 0]
        if len(valid_depth) > 0:
            depth_min = valid_depth.min()
            depth_max = valid_depth.max()
            depth_image_normalized = np.zeros_like(depth_image_m_aligned, dtype=np.uint8)
            mask = depth_image_m_aligned > 0
            depth_image_normalized[mask] = np.clip(
                (depth_image_m_aligned[mask] - depth_min) * 255.0 / (depth_max - depth_min + 1e-6), 
                0, 255
            ).astype(np.uint8)
        else:
            depth_image_normalized = np.zeros_like(depth_image_m_aligned, dtype=np.uint8)
        
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
            
        elif visualization_mode == 5:
            # 6. RGB + Depth side-by-side
            display_base = np.hstack((color_image, depth_colormap))
            
        else:  # visualization_mode == 6
            # 7. Depth with Crop Region (calibration mode)
            depth_vis = depth_colormap.copy()
            
            if camera.use_intrinsics:
                # For intrinsics mode, show FOV-based crop region
                fov_ratio_h = np.tan(camera.fov_rgb_h / 2.0) / np.tan(camera.fov_depth_h / 2.0)
                fov_ratio_v = np.tan(camera.fov_rgb_v / 2.0) / np.tan(camera.fov_depth_v / 2.0)
                
                crop_width = int(imageWidth * fov_ratio_h * camera.crop_scale_x)
                crop_height = int(imageHeight * fov_ratio_v * camera.crop_scale_y)
                crop_start_x = (imageWidth - crop_width) // 2
                crop_start_y = (imageHeight - crop_height) // 2
                
                # Draw FOV-based crop region
                cv2.rectangle(depth_vis, 
                            (crop_start_x, crop_start_y), 
                            (crop_start_x + crop_width, crop_start_y + crop_height), 
                            (0, 255, 0), 2)
                
                # Draw principal point offset visualization
                center_x = imageWidth // 2
                center_y = imageHeight // 2
                offset_x = int(camera.principal_point_offset_x)
                offset_y = int(camera.principal_point_offset_y)
                
                # Draw optical center
                cv2.circle(depth_vis, (center_x, center_y), 5, (255, 0, 0), -1)  # Blue dot
                # Draw shifted principal point
                cv2.circle(depth_vis, (center_x + offset_x, center_y + offset_y), 5, (0, 0, 255), -1)  # Red dot
                # Draw arrow showing shift
                cv2.arrowedLine(depth_vis, (center_x, center_y), 
                              (center_x + offset_x, center_y + offset_y), 
                              (255, 255, 0), 2, tipLength=0.3)  # Cyan arrow
                
                cv2.putText(depth_vis, "INTRINSICS CALIBRATION MODE", (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(depth_vis, f"FOV Crop: [{crop_start_y}:{crop_start_y+crop_height}, {crop_start_x}:{crop_start_x+crop_width}]", 
                          (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(depth_vis, f"PP Offset: cx={camera.principal_point_offset_x:.1f}, cy={camera.principal_point_offset_y:.1f}", 
                          (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(depth_vis, f"Crop Scale: X={camera.crop_scale_x:.3f}, Y={camera.crop_scale_y:.3f}", 
                          (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            else:
                # For simple mode, show actual crop region
                crop_top = camera.crop_top
                crop_bottom = camera.crop_bottom
                crop_left = camera.crop_left
                crop_right = camera.crop_right
                
                cv2.rectangle(depth_vis, (crop_left, crop_top), (crop_right, crop_bottom), (0, 255, 0), 2)
                cv2.putText(depth_vis, "SIMPLE CALIBRATION MODE", (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(depth_vis, f"Crop: [{crop_top}:{crop_bottom}, {crop_left}:{crop_right}]", 
                          (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            display_base = depth_vis
        
        # Add info text overlay function with FPS
        def add_info_text(img, mode_name, depth_min_val, depth_max_val):
            img_with_text = img.copy()
            # Compact overlay - top right corner
            overlay_width = 280
            overlay_height = 145  # Increased for depth range line
            margin = 5
            x_start = img.shape[1] - overlay_width - margin
            y_start = margin
            
            # Semi-transparent background
            overlay_bg = img_with_text[y_start:y_start+overlay_height, x_start:x_start+overlay_width].copy()
            cv2.rectangle(img_with_text, (x_start, y_start), (x_start+overlay_width, y_start+overlay_height), (0, 0, 0), -1)
            cv2.addWeighted(overlay_bg, 0.3, img_with_text[y_start:y_start+overlay_height, x_start:x_start+overlay_width], 0.7, 0, img_with_text[y_start:y_start+overlay_height, x_start:x_start+overlay_width])
            
            # Border
            cv2.rectangle(img_with_text, (x_start, y_start), (x_start+overlay_width, y_start+overlay_height), (0, 255, 0), 1)
            
            # Compact text info - HIGHER RESOLUTION
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.45  # Increased from 0.35 for better readability
            thickness = 1
            line_height = 17  # Slightly increased for better spacing
            
            y_offset = y_start + 18
            # FPS display with color coding
            fps_color = (0, 255, 0) if current_fps >= 30 else (0, 165, 255) if current_fps >= 20 else (0, 0, 255)
            cv2.putText(img_with_text, f"FPS:{current_fps:.1f} Mode:{mode_name[:8]}", (x_start+5, y_offset), font, font_scale, fps_color, thickness)
            y_offset += line_height
            # Depth range with color coding
            if depth_max_val > 0:
                depth_color = (0, 255, 255)  # Cyan for depth info
                cv2.putText(img_with_text, f"Depth:{depth_min_val:.2f}-{depth_max_val:.2f}m", (x_start+5, y_offset), font, font_scale, depth_color, thickness)
            else:
                cv2.putText(img_with_text, f"Depth: No data", (x_start+5, y_offset), font, font_scale, (0, 0, 255), thickness)
            y_offset += line_height
            cv2.putText(img_with_text, f"Focal:RGB={camera.focal_scale_rgb:.2f} D={camera.focal_scale_depth:.2f}", (x_start+5, y_offset), font, font_scale, (255, 255, 255), thickness)
            y_offset += line_height
            if camera.use_intrinsics:
                cv2.putText(img_with_text, f"PP:cx={camera.principal_point_offset_x:.1f} cy={camera.principal_point_offset_y:.1f}", (x_start+5, y_offset), font, font_scale, (255, 255, 255), thickness)
            else:
                cv2.putText(img_with_text, f"Crop:X={camera.crop_offset_x} Y={camera.crop_offset_y}", (x_start+5, y_offset), font, font_scale, (255, 255, 255), thickness)
            y_offset += line_height
            cv2.putText(img_with_text, f"Scale:X={camera.crop_scale_x:.2f} Y={camera.crop_scale_y:.2f}", (x_start+5, y_offset), font, font_scale, (255, 255, 255), thickness)
            y_offset += line_height
            cv2.putText(img_with_text, f"Edge:{edge_threshold1}/{edge_threshold2}", (x_start+5, y_offset), font, font_scale, (255, 255, 255), thickness)
            y_offset += line_height
            align_mode_text = "Intrinsics" if camera.use_intrinsics else "Simple"
            align_mode_color = (0, 255, 0) if camera.use_intrinsics else (0, 165, 255)
            cv2.putText(img_with_text, f"Align:{align_mode_text} [M]:Toggle", (x_start+5, y_offset), font, font_scale, align_mode_color, thickness)
            y_offset += line_height
            cv2.putText(img_with_text, f"[H]:Help [V]:Save [B]:Load", (x_start+5, y_offset), font, font_scale, (255, 200, 0), thickness)
            
            return img_with_text
        
        # Display the selected mode
        display_img = add_info_text(display_base, mode_names[visualization_mode], depth_min if len(valid_depth) > 0 else 0, depth_max if len(valid_depth) > 0 else 0)
        cv2.namedWindow('Alignment Viewer', cv2.WINDOW_NORMAL)
        cv2.imshow('Alignment Viewer', display_img)
        
        # Calculate FPS
        fps_frame_count += 1
        if fps_frame_count >= 10:  # Update FPS every 10 frames
            fps_end_time = time.time()
            current_fps = fps_frame_count / (fps_end_time - fps_start_time)
            fps_start_time = fps_end_time
            fps_frame_count = 0
        
        # # Print status every 50 frames
        # if iteration % 50 == 1:
        #     print(f"\n{'='*70}")
        #     print(f"📊 Frame {iteration} | Mode: {mode_names[visualization_mode]}")
        #     print(f"{'='*70}")
        #     if len(valid_depth) > 0:
        #         print(f"Depth range: {depth_min:.2f}m - {depth_max:.2f}m")
        #         print(f"RGB focal scale: {camera.focal_scale_rgb:.3f} | Depth focal scale: {camera.focal_scale_depth:.3f}")
        #         print(f"RGB focal: ({camera.fx_rgb:.1f}, {camera.fy_rgb:.1f}) | Depth focal: ({camera.fx_depth:.1f}, {camera.fy_depth:.1f})")
        #         print(f"Crop offset: X={camera.crop_offset_x}, Y={camera.crop_offset_y}")
        #         print(f"Crop scale: X={camera.crop_scale_x:.3f}, Y={camera.crop_scale_y:.3f}")
        #         print(f"Edge thresholds: {edge_threshold1}/{edge_threshold2}")
        #         print(f"💡 TIP: If depth vertically bigger than RGB, press [U]. If smaller, press [O]")
        #         print(f"💡 Use [I/K/J/L] to shift depth position until edges align (Yellow in mode 0)")

        
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
        elif key & 0xFF == ord('6'):
            visualization_mode = 6
            print(f"\n✓ Switched to mode: {mode_names[6]}")
        
        # Dynamic tuning - RGB focal scale
        elif key & 0xFF == ord('w') or key & 0xFF == ord('W'):
            camera.focal_scale_rgb += tune_step
            camera._calculate_focal_lengths()
            print(f"\n⬆ RGB focal scale: {camera.focal_scale_rgb:.3f} (fx={camera.fx_rgb:.1f}, fy={camera.fy_rgb:.1f})")
        elif key & 0xFF == ord('s') or key & 0xFF == ord('S'):
            camera.focal_scale_rgb -= tune_step
            camera._calculate_focal_lengths()
            print(f"\n⬇ RGB focal scale: {camera.focal_scale_rgb:.3f} (fx={camera.fx_rgb:.1f}, fy={camera.fy_rgb:.1f})")
        
        # Dynamic tuning - Depth focal scale
        elif key & 0xFF == ord('d') or key & 0xFF == ord('D'):
            camera.focal_scale_depth += tune_step
            camera._calculate_focal_lengths()
            print(f"\n⬆ Depth focal scale: {camera.focal_scale_depth:.3f} (fx={camera.fx_depth:.1f}, fy={camera.fy_depth:.1f})")
        elif key & 0xFF == ord('a') or key & 0xFF == ord('A'):
            camera.focal_scale_depth -= tune_step
            camera._calculate_focal_lengths()
            print(f"\n⬇ Depth focal scale: {camera.focal_scale_depth:.3f} (fx={camera.fx_depth:.1f}, fy={camera.fy_depth:.1f})")
        
        # Reset focal scales
        elif key & 0xFF == ord('r') or key & 0xFF == ord('R'):
            camera.focal_scale_rgb = 1.0
            camera.focal_scale_depth = 1.0
            camera._calculate_focal_lengths()
            print(f"\n🔄 RESET focal scales to 1.0")
        
        # Toggle alignment mode
        elif key & 0xFF == ord('m') or key & 0xFF == ord('M'):
            camera.use_intrinsics = not camera.use_intrinsics
            mode_name = "Camera Intrinsics" if camera.use_intrinsics else "Simple Resize"
            print(f"\n🔀 Alignment mode: {mode_name}")
        
        # Reset alignment parameters
        elif key & 0xFF == ord('c') or key & 0xFF == ord('C'):
            if camera.use_intrinsics:
                camera.principal_point_offset_x = 0.0
                camera.principal_point_offset_y = 0.0
                camera.crop_scale_x = 1.0
                camera.crop_scale_y = 1.0
                camera._calculate_focal_lengths()
                print(f"\n🔄 RESET principal point offsets & scale to default")
            else:
                camera.crop_offset_x = 0
                camera.crop_offset_y = 0
                # Reset to default crop values
                camera.crop_top = 65
                camera.crop_bottom = 415
                camera.crop_left = 105
                camera.crop_right = 510
                print(f"\n🔄 RESET crop boundaries to default")
        
        # Alignment shift adjustment (IJKL keys)
        elif key & 0xFF == ord('j') or key & 0xFF == ord('J'):
            if camera.use_intrinsics:
                camera.principal_point_offset_x -= principal_point_step
                camera._calculate_focal_lengths()
                print(f"\n← Principal point shifted LEFT | cx_offset={camera.principal_point_offset_x:.1f}, cy_offset={camera.principal_point_offset_y:.1f}")
            else:
                camera.crop_offset_x -= crop_tune_step
                print(f"\n← Crop shifted LEFT | Offset: X={camera.crop_offset_x}, Y={camera.crop_offset_y}")
        elif key & 0xFF == ord('l') or key & 0xFF == ord('L'):
            if camera.use_intrinsics:
                camera.principal_point_offset_x += principal_point_step
                camera._calculate_focal_lengths()
                print(f"\n→ Principal point shifted RIGHT | cx_offset={camera.principal_point_offset_x:.1f}, cy_offset={camera.principal_point_offset_y:.1f}")
            else:
                camera.crop_offset_x += crop_tune_step
                print(f"\n→ Crop shifted RIGHT | Offset: X={camera.crop_offset_x}, Y={camera.crop_offset_y}")
        elif key & 0xFF == ord('i') or key & 0xFF == ord('I'):
            if camera.use_intrinsics:
                camera.principal_point_offset_y -= principal_point_step
                camera._calculate_focal_lengths()
                print(f"\n↑ Principal point shifted UP | cx_offset={camera.principal_point_offset_x:.1f}, cy_offset={camera.principal_point_offset_y:.1f}")
            else:
                camera.crop_offset_y -= crop_tune_step
                print(f"\n↑ Crop shifted UP | Offset: X={camera.crop_offset_x}, Y={camera.crop_offset_y}")
        elif key & 0xFF == ord('k') or key & 0xFF == ord('K'):
            if camera.use_intrinsics:
                camera.principal_point_offset_y += principal_point_step
                camera._calculate_focal_lengths()
                print(f"\n↓ Principal point shifted DOWN | cx_offset={camera.principal_point_offset_x:.1f}, cy_offset={camera.principal_point_offset_y:.1f}")
            else:
                camera.crop_offset_y += crop_tune_step
                print(f"\n↓ Crop shifted DOWN | Offset: X={camera.crop_offset_x}, Y={camera.crop_offset_y}")
        
        # Crop SCALE adjustment (fix vertical/horizontal size mismatch)
        elif key & 0xFF == ord('u') or key & 0xFF == ord('U'):
            if camera.use_intrinsics:
                camera.crop_scale_y += crop_scale_step
                print(f"\n📏 Vertical crop INCREASED (depth smaller) | Crop scale: X={camera.crop_scale_x:.3f}, Y={camera.crop_scale_y:.3f}")
            else:
                # In simple mode, shrink crop region vertically (increase top/bottom inset)
                camera.crop_top += crop_tune_step
                camera.crop_bottom -= crop_tune_step
                print(f"\n📏 Vertical crop INCREASED | Crop: [{camera.crop_top}:{camera.crop_bottom}, {camera.crop_left}:{camera.crop_right}]")
        elif key & 0xFF == ord('o') or key & 0xFF == ord('O'):
            if camera.use_intrinsics:
                camera.crop_scale_y = max(0.5, camera.crop_scale_y - crop_scale_step)
                print(f"\n📏 Vertical crop DECREASED (depth bigger) | Crop scale: X={camera.crop_scale_x:.3f}, Y={camera.crop_scale_y:.3f}")
            else:
                # In simple mode, expand crop region vertically (decrease top/bottom inset)
                camera.crop_top = max(0, camera.crop_top - crop_tune_step)
                camera.crop_bottom = min(imageHeight, camera.crop_bottom + crop_tune_step)
                print(f"\n📏 Vertical crop DECREASED | Crop: [{camera.crop_top}:{camera.crop_bottom}, {camera.crop_left}:{camera.crop_right}]")
        elif key & 0xFF == ord('y') or key & 0xFF == ord('Y'):
            if camera.use_intrinsics:
                camera.crop_scale_x += crop_scale_step
                print(f"\n📏 Horizontal crop INCREASED (depth smaller) | Crop scale: X={camera.crop_scale_x:.3f}, Y={camera.crop_scale_y:.3f}")
            else:
                # In simple mode, shrink crop region horizontally (increase left/right inset)
                camera.crop_left += crop_tune_step
                camera.crop_right -= crop_tune_step
                print(f"\n📏 Horizontal crop INCREASED | Crop: [{camera.crop_top}:{camera.crop_bottom}, {camera.crop_left}:{camera.crop_right}]")
        elif key & 0xFF == ord('p') or key & 0xFF == ord('P'):
            if camera.use_intrinsics:
                camera.crop_scale_x = max(0.5, camera.crop_scale_x - crop_scale_step)
                print(f"\n📏 Horizontal crop DECREASED (depth bigger) | Crop scale: X={camera.crop_scale_x:.3f}, Y={camera.crop_scale_y:.3f}")
            else:
                # In simple mode, expand crop region horizontally (decrease left/right inset)
                camera.crop_left = max(0, camera.crop_left - crop_tune_step)
                camera.crop_right = min(imageWidth, camera.crop_right + crop_tune_step)
                print(f"\n📏 Horizontal crop DECREASED | Crop: [{camera.crop_top}:{camera.crop_bottom}, {camera.crop_left}:{camera.crop_right}]")
        
        # Edge detection tuning
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
            camera.save_alignment_settings()
        elif key & 0xFF == ord('b') or key & 0xFF == ord('B'):
            camera.load_alignment_settings()


except KeyboardInterrupt:
    print("User interrupted! (Ctrl+C)")

finally:
    camera.terminate()


