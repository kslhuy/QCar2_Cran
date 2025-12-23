## rgbd_imaging.py - Enhanced Version with Improved Depth Alignment Control
# This example combines the depth and RGB sensors from the Intel Realsense D435 to display objects 
# within a specified distance with configurable alignment parameters.
# 
# KEYBOARD CONTROLS:
#   'q' or ESC: Quit
#   'w/s': Adjust crop top/bottom
#   'a/d': Adjust crop left/right
#   'r': Reset to default alignment
#   'c': Toggle calibration mode
#   'v': Toggle visualization mode (normal/debug/overlay)
#   '1/2': Adjust min threshold distance
#   '3/4': Adjust max threshold distance
#   'h': Print help

from hal.utilities.image_processing import ImageProcessing
from pal.products.qcar import QCarRealSense
import time
import struct
import numpy as np 
import cv2
import json
import os

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Alignment Configuration Class
class DepthAlignmentConfig:
    """Manages depth-to-RGB alignment parameters with save/load capabilities"""
    
    def __init__(self, config_file='depth_alignment_config.json'):
        # Use absolute path to ensure config is saved in the same directory as the script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(script_dir, config_file)
        
        # Default alignment parameters
        self.crop_top = 20
        self.crop_bottom = 420
        self.crop_left = 60
        self.crop_right = 600
        
        # Morphological filtering parameters
        self.dilate_iterations = 3
        self.erode_iterations = 1
        self.morph_total = 1
        
        # Temporal filtering
        self.use_temporal_filter = True
        self.temporal_weight = 1.0
        
        # Visualization mode: 'normal', 'debug', 'overlay', 'edges', 'blend', 'checkerboard'
        self.viz_mode = 'normal'
        
        # Calibration mode
        self.calibration_mode = False
        
        # Blend transparency for alignment verification
        self.blend_alpha = 0.5
        
        # Distance thresholds (in scaled units - will be divided by QLabsScaling for display)
        self.minThreshold = 2.0  # Default 0.2m * 10 scaling
        self.maxThreshold = 5.0  # Default 0.5m * 10 scaling
        
        self.load()
    
    def load(self):
        """Load configuration from file if it exists"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    data = json.load(f)
                    self.__dict__.update(data)
                print(f"✓ Loaded alignment config from {self.config_file}")
                print(f"  Crop: [{self.crop_top}:{self.crop_bottom}, {self.crop_left}:{self.crop_right}]")
                print(f"  Thresholds: min={self.minThreshold:.1f}, max={self.maxThreshold:.1f}")
            except Exception as e:
                print(f"✗ Error loading config: {e}, using defaults")
        else:
            print(f"No config file found at {self.config_file}, using defaults")
    
    def save(self):
        """Save current configuration to file"""
        try:
            # Don't save config_file path itself
            save_data = {k: v for k, v in self.__dict__.items() if k != 'config_file'}
            with open(self.config_file, 'w') as f:
                json.dump(save_data, f, indent=4)
            print(f"✓ Saved alignment config to {self.config_file}")
            print(f"  Crop: [{self.crop_top}:{self.crop_bottom}, {self.crop_left}:{self.crop_right}]")
            print(f"  Thresholds: min={self.minThreshold:.1f}, max={self.maxThreshold:.1f}")
        except Exception as e:
            print(f"✗ Error saving config: {e}")
    
    def get_crop_region(self):
        """Return crop coordinates as tuple"""
        return (self.crop_top, self.crop_bottom, self.crop_left, self.crop_right)
    
    def adjust_crop(self, direction, amount=5):
        """Adjust crop region dynamically"""
        if direction == 'top_up':
            self.crop_top = max(0, self.crop_top - amount)
        elif direction == 'top_down':
            self.crop_top = min(self.crop_bottom - 10, self.crop_top + amount)
        elif direction == 'bottom_up':
            self.crop_bottom = max(self.crop_top + 10, self.crop_bottom - amount)
        elif direction == 'bottom_down':
            self.crop_bottom = min(480, self.crop_bottom + amount)
        elif direction == 'left_left':
            self.crop_left = max(0, self.crop_left - amount)
        elif direction == 'left_right':
            self.crop_left = min(self.crop_right - 10, self.crop_left + amount)
        elif direction == 'right_left':
            self.crop_right = max(self.crop_left + 10, self.crop_right - amount)
        elif direction == 'right_right':
            self.crop_right = min(640, self.crop_right + amount)
    
    def reset_defaults(self):
        """Reset to default alignment parameters"""
        self.crop_top = 20
        self.crop_bottom = 420
        self.crop_left = 60
        self.crop_right = 600
        self.dilate_iterations = 3
        self.erode_iterations = 1
        print("Reset to default alignment parameters")

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Timing Parameters and methods 
startTime = time.time()
def elapsed_time():
    return time.time() - startTime

sampleRate     = 30.0
sampleTime     = 1/sampleRate
simulationTime = 300.0  # Extended to 5 minutes for calibration
print('Sample Time: ', sampleTime)


# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Initialize Alignment Configuration
align_config = DepthAlignmentConfig()

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Initialize the RealSense camera for RGB and Depth data

# Multiagent port assignment (ensure unique port if multiple QCars are used)
myCam1 = QCarRealSense(mode='RGB, Depth', video3dPort=18805) 
# myCam1 = QCarRealSense(mode='RGB, Depth')

# Scaling constants
QLabsScaling = 10
MaxDistance = 10  # m

# Distance adjustment step
distance_step = 0.5

# Use threshold values from config (loaded values or defaults)
# These are already in scaled units (real_meters * QLabsScaling)
minPixel = int((align_config.minThreshold / MaxDistance) * 255)
maxPixel = int((align_config.maxThreshold / MaxDistance) * 255)

print("\n=== Enhanced Depth Alignment Control ===")
print("KEYBOARD CONTROLS:")
print("  'q' or ESC: Quit")
print("\nCROP REGION CONTROL (Hold Shift for fine adjustment):")
print("  'w'/'s': Adjust TOP edge (up/down)")
print("  'a'/'d': Adjust LEFT edge (left/right)")
print("  'i'/'k': Adjust BOTTOM edge (up/down)")
print("  'j'/'l': Adjust RIGHT edge (left/right)")
print("  Arrow Keys: Up=top, Down=bottom, Left=left edge, Right=right edge")
print("  Hold SHIFT: Fine adjustment (1 pixel instead of 5)")
print("\nMODES & SETTINGS:")
print("  'r': Reset to default alignment")
print("  'c': Toggle calibration mode (shows crop overlay)")
print("  'v': Cycle visualization modes (normal/debug/overlay/edges/blend/checkerboard)")
print("  '+'/'-': Adjust blend transparency (in blend mode)")
print("  '1/2': Adjust min threshold distance")
print("  '3/4': Adjust max threshold distance")
print("  'p': Save current alignment configuration")
print("  'h': Print this help")
print("\nALIGNMENT VERIFICATION MODES:")
print("  edges: Green=RGB edges, Red=Depth edges, Yellow=Aligned (BEST for verification!)")
print("  blend: Transparent overlay - adjust with +/-")
print("  checkerboard: Alternating blocks - look for seamless transitions\n")
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

## Main Loop
flag = True
fps_counter = 0
fps_start_time = time.time()
fps_display = 0.0

try:
    while elapsed_time() < simulationTime:
        # Start timing this iteration
        start = time.time()

        # Read the RGB and Depth data (latter in meters)
        myCam1.read_RGB()
        myCam1.read_depth(dataMode='PX')
        
        # Threshold the depth image based on min and max distance set above, and cast it to uint8 (to be used as a mask later)
        binaryNow = ImageProcessing.binary_thresholding(
            myCam1.imageBufferDepthPX,
            minPixel, 
            maxPixel
        ).astype(np.uint8)

        # Initialize binaryBefore to keep a 1 step time history of the binary to do a temporal difference filter later. 
        # At the first time step, flag = True. Initialize binaryBefore and then set flag = False to not do this again.
        if flag:
            binaryBefore = binaryNow
            flag = False
        
        # Apply temporal filtering if enabled
        if align_config.use_temporal_filter:
            # Clean = closing filter applied ON (binaryNow BITWISE AND (BITWISE NOT of (the ABSOLUTE of (difference between binary now and before))))
            binaryClean = ImageProcessing.image_filtering_close(
                cv2.bitwise_and(
                    cv2.bitwise_not(np.abs(binaryNow - binaryBefore) / 255), 
                    binaryNow / 255
                ), 
                dilate=align_config.dilate_iterations, 
                erode=align_config.erode_iterations, 
                total=align_config.morph_total
            )
        else:
            binaryClean = binaryNow / 255
        
        # Get crop region from configuration
        crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
        
        # Grab the cropped chunk of the depth data and scale it back to full resolution
        # to account for field-of-view differences and physical distance between the RGB/Depth cameras.
        binaryClean = cv2.resize(
            binaryClean[crop_top:crop_bottom, crop_left:crop_right],
            (640, 480)
        ).astype(np.uint8) * 255
        
        # Apply the binaryClean mask to the RGB image captured
        maskedRGB = cv2.bitwise_and(myCam1.imageBufferRGB, myCam1.imageBufferRGB, mask=binaryClean)
        
        # --- Visualization based on mode ---
        if align_config.viz_mode == 'normal':
            # Normal mode: just show the masked RGB
            display_image = maskedRGB.copy()
            
        elif align_config.viz_mode == 'debug':
            # Debug mode: show original RGB, depth, and masked side by side
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(myCam1.imageBufferDepthPX, alpha=1.0), 
                cv2.COLORMAP_JET
            )
            # Resize for display
            rgb_small = cv2.resize(myCam1.imageBufferRGB, (320, 240))
            depth_small = cv2.resize(depth_colormap, (320, 240))
            masked_small = cv2.resize(maskedRGB, (320, 240))
            
            # Convert binary mask to 3-channel for stacking
            binary_small = cv2.resize(binaryClean, (320, 240))
            binary_small_3ch = cv2.cvtColor(binary_small, cv2.COLOR_GRAY2BGR)
            
            top_row = np.hstack((rgb_small, depth_small))
            bottom_row = np.hstack((masked_small, binary_small_3ch))
            display_image = np.vstack((top_row, bottom_row))
            
        elif align_config.viz_mode == 'overlay':
            # Overlay mode: show mask as semi-transparent overlay on RGB
            display_image = myCam1.imageBufferRGB.copy()
            # Create green-colored mask manually (OpenCV doesn't have COLORMAP_GREEN)
            mask_colored = np.zeros((480, 640, 3), dtype=np.uint8)
            mask_colored[:, :, 1] = binaryClean  # Green channel only
            display_image = cv2.addWeighted(display_image, 0.7, mask_colored, 0.3, 0)
            
        elif align_config.viz_mode == 'edges':
            # Edge detection mode: Show RGB edges and depth edges overlaid to verify alignment
            # Convert RGB to grayscale and detect edges
            rgb_gray = cv2.cvtColor(myCam1.imageBufferRGB, cv2.COLOR_BGR2GRAY)
            rgb_edges = cv2.Canny(rgb_gray, 50, 150)
            
            # Detect edges in depth
            depth_edges = cv2.Canny(myCam1.imageBufferDepthPX.astype(np.uint8), 30, 100)
            
            # Crop and resize depth edges to match RGB
            crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
            depth_edges_cropped = depth_edges[crop_top:crop_bottom, crop_left:crop_right]
            depth_edges_aligned = cv2.resize(depth_edges_cropped, (640, 480))
            
            # Create color-coded edge overlay: RGB edges in GREEN, Depth edges in RED
            display_image = np.zeros((480, 640, 3), dtype=np.uint8)
            display_image[:, :, 1] = rgb_edges  # Green channel for RGB edges
            display_image[:, :, 2] = depth_edges_aligned  # Red channel for depth edges
            # Where both edges align, you'll see YELLOW (red + green)
            
            # Add original RGB as background for context
            rgb_dimmed = (myCam1.imageBufferRGB * 0.3).astype(np.uint8)
            display_image = cv2.add(display_image, rgb_dimmed)
            
            cv2.putText(display_image, "GREEN=RGB edges, RED=Depth edges, YELLOW=Aligned", 
                       (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
        elif align_config.viz_mode == 'blend':
            # Blend mode: Blend RGB with colorized depth for direct alignment verification
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(myCam1.imageBufferDepthPX, alpha=1.0), 
                cv2.COLORMAP_JET
            )
            
            # Crop and resize depth colormap to match RGB
            crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
            depth_cropped = depth_colormap[crop_top:crop_bottom, crop_left:crop_right]
            depth_aligned = cv2.resize(depth_cropped, (640, 480))
            
            # Blend RGB and depth - adjust alpha with '+'/'-' keys
            display_image = cv2.addWeighted(
                myCam1.imageBufferRGB, 
                align_config.blend_alpha, 
                depth_aligned, 
                1.0 - align_config.blend_alpha, 
                0
            )
            
            cv2.putText(display_image, f"Blend: RGB {align_config.blend_alpha:.1f} / Depth {1.0-align_config.blend_alpha:.1f}", 
                       (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(display_image, "Press '+'/'-' to adjust blend", 
                       (10, 475), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
        elif align_config.viz_mode == 'checkerboard':
            # Checkerboard mode: Alternate blocks of RGB and depth for precise alignment check
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(myCam1.imageBufferDepthPX, alpha=1.0), 
                cv2.COLORMAP_JET
            )
            
            # Crop and resize depth to match RGB
            crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
            depth_cropped = depth_colormap[crop_top:crop_bottom, crop_left:crop_right]
            depth_aligned = cv2.resize(depth_cropped, (640, 480))
            
            # Create checkerboard mask (40x40 pixel blocks)
            block_size = 40
            checkerboard = np.zeros((480, 640), dtype=np.uint8)
            for i in range(0, 480, block_size):
                for j in range(0, 640, block_size):
                    if ((i // block_size) + (j // block_size)) % 2 == 0:
                        checkerboard[i:i+block_size, j:j+block_size] = 255
            
            # Apply checkerboard pattern
            checkerboard_3ch = cv2.cvtColor(checkerboard, cv2.COLOR_GRAY2BGR)
            display_image = np.where(checkerboard_3ch == 255, myCam1.imageBufferRGB, depth_aligned)
            
            cv2.putText(display_image, "Checkerboard: Look for seamless transitions at block edges", 
                       (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Show calibration overlay if in calibration mode
        if align_config.calibration_mode:
            # Draw crop region on depth image for visualization
            depth_vis = cv2.applyColorMap(
                cv2.convertScaleAbs(myCam1.imageBufferDepthPX, alpha=1.0), 
                cv2.COLORMAP_JET
            ).copy()
            cv2.rectangle(depth_vis, (crop_left, crop_top), (crop_right, crop_bottom), (0, 255, 0), 2)
            cv2.putText(depth_vis, "CALIBRATION MODE", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(depth_vis, f"Crop: [{crop_top}:{crop_bottom}, {crop_left}:{crop_right}]", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            cv2.imshow('Depth with Crop Region', depth_vis)
        
        # Add info overlay to main display
        info_y = 30
        cv2.putText(display_image, f"FPS: {fps_display:.1f}", (10, info_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        info_y += 25
        cv2.putText(display_image, f"Min Dist: {align_config.minThreshold/QLabsScaling:.2f}m", (10, info_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        info_y += 20
        cv2.putText(display_image, f"Max Dist: {align_config.maxThreshold/QLabsScaling:.2f}m", (10, info_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        info_y += 20
        cv2.putText(display_image, f"Mode: {align_config.viz_mode}", (10, info_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Display the result
        cv2.namedWindow('Enhanced RGBD Alignment', cv2.WINDOW_NORMAL)
        cv2.imshow('Enhanced RGBD Alignment', display_image)
        
        # Calculate FPS
        fps_counter += 1
        if fps_counter >= 10:
            fps_display = fps_counter / (time.time() - fps_start_time)
            fps_counter = 0
            fps_start_time = time.time()
        
        # End timing this iteration
        end = time.time()

        # Calculate the computation time, and the time that the thread should pause/sleep for
        computationTime = end - start
        sleepTime = sampleTime - (computationTime % sampleTime)

        # Pause/sleep for sleepTime in milliseconds
        msSleepTime = int(1000 * sleepTime)
        if msSleepTime <= 0:
            msSleepTime = 1
        
        # Handle keyboard input
        key = cv2.waitKey(msSleepTime)
        
        if key != -1:  # Key was pressed
            key_char = chr(key & 0xFF)
            
            # Check for shift modifier (fine adjustment)
            shift_pressed = (key & 0x10000) != 0  # Shift key modifier
            adjust_amount = 1 if shift_pressed else 5
            
            # Quit
            if key_char == 'q' or key == 27:
                print("Quitting...")
                break
            
            # Crop adjustments - TOP edge (w/s)
            elif key_char == 'w' or key_char == 'W':
                align_config.adjust_crop('top_up', adjust_amount)
                crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
                print(f"Crop region: [{crop_top}:{crop_bottom}, {crop_left}:{crop_right}]")
            elif key_char == 's' or key_char == 'S':
                align_config.adjust_crop('top_down', adjust_amount)
                crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
                print(f"Crop region: [{crop_top}:{crop_bottom}, {crop_left}:{crop_right}]")
            
            # Crop adjustments - LEFT edge (a/d)
            elif key_char == 'a' or key_char == 'A':
                align_config.adjust_crop('left_left', adjust_amount)
                crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
                print(f"Crop region: [{crop_top}:{crop_bottom}, {crop_left}:{crop_right}]")
            elif key_char == 'd' or key_char == 'D':
                align_config.adjust_crop('left_right', adjust_amount)
                crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
                print(f"Crop region: [{crop_top}:{crop_bottom}, {crop_left}:{crop_right}]")
            
            # Crop adjustments - BOTTOM edge (i/k)
            elif key_char == 'i' or key_char == 'I':
                align_config.adjust_crop('bottom_up', adjust_amount)
                crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
                print(f"Crop region: [{crop_top}:{crop_bottom}, {crop_left}:{crop_right}]")
            elif key_char == 'k' or key_char == 'K':
                align_config.adjust_crop('bottom_down', adjust_amount)
                crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
                print(f"Crop region: [{crop_top}:{crop_bottom}, {crop_left}:{crop_right}]")
            
            # Crop adjustments - RIGHT edge (j/l)
            elif key_char == 'j' or key_char == 'J':
                align_config.adjust_crop('right_left', adjust_amount)
                crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
                print(f"Crop region: [{crop_top}:{crop_bottom}, {crop_left}:{crop_right}]")
            elif key_char == 'l' or key_char == 'L':
                align_config.adjust_crop('right_right', adjust_amount)
                crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
                print(f"Crop region: [{crop_top}:{crop_bottom}, {crop_left}:{crop_right}]")
            
            # Arrow keys for crop adjustment (alternative controls)
            elif key == 82 or key == 2490368:  # Up arrow
                align_config.adjust_crop('top_up', adjust_amount)
                crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
                print(f"TOP edge: {crop_top}")
            elif key == 84 or key == 2621440:  # Down arrow
                align_config.adjust_crop('bottom_down', adjust_amount)
                crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
                print(f"BOTTOM edge: {crop_bottom}")
            elif key == 81 or key == 2424832:  # Left arrow
                align_config.adjust_crop('left_left', adjust_amount)
                crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
                print(f"LEFT edge: {crop_left}")
            elif key == 83 or key == 2555904:  # Right arrow
                align_config.adjust_crop('right_right', adjust_amount)
                crop_top, crop_bottom, crop_left, crop_right = align_config.get_crop_region()
                print(f"RIGHT edge: {crop_right}")
            
            # Reset alignment
            elif key_char == 'r':
                align_config.reset_defaults()
            
            # Toggle calibration mode
            elif key_char == 'c':
                align_config.calibration_mode = not align_config.calibration_mode
                if not align_config.calibration_mode:
                    cv2.destroyWindow('Depth with Crop Region')
                print(f"Calibration mode: {'ON' if align_config.calibration_mode else 'OFF'}")
            
            # Toggle visualization mode
            elif key_char == 'v':
                modes = ['normal', 'debug', 'overlay', 'edges', 'blend', 'checkerboard']
                current_idx = modes.index(align_config.viz_mode)
                align_config.viz_mode = modes[(current_idx + 1) % len(modes)]
                print(f"Visualization mode: {align_config.viz_mode}")
            
            # Adjust blend transparency
            elif key_char == '+' or key_char == '=':
                align_config.blend_alpha = min(1.0, align_config.blend_alpha + 0.1)
                print(f"Blend alpha: {align_config.blend_alpha:.1f}")
            elif key_char == '-' or key_char == '_':
                align_config.blend_alpha = max(0.0, align_config.blend_alpha - 0.1)
                print(f"Blend alpha: {align_config.blend_alpha:.1f}")
            
            # Distance threshold adjustments
            elif key_char == '1':
                align_config.minThreshold = max(0.1 * QLabsScaling, align_config.minThreshold - distance_step)
                minPixel = int((align_config.minThreshold / MaxDistance) * 255)
                print(f"Min threshold: {align_config.minThreshold/QLabsScaling:.2f}m")
            elif key_char == '2':
                align_config.minThreshold = min(align_config.maxThreshold - 0.1 * QLabsScaling, align_config.minThreshold + distance_step)
                minPixel = int((align_config.minThreshold / MaxDistance) * 255)
                print(f"Min threshold: {align_config.minThreshold/QLabsScaling:.2f}m")
            elif key_char == '3':
                align_config.maxThreshold = max(align_config.minThreshold + 0.1 * QLabsScaling, align_config.maxThreshold - distance_step)
                maxPixel = int((align_config.maxThreshold / MaxDistance) * 255)
                print(f"Max threshold: {align_config.maxThreshold/QLabsScaling:.2f}m")
            elif key_char == '4':
                align_config.maxThreshold = min(MaxDistance, align_config.maxThreshold + distance_step)
                maxPixel = int((align_config.maxThreshold / MaxDistance) * 255)
                print(f"Max threshold: {align_config.maxThreshold/QLabsScaling:.2f}m")
            
            # Save configuration
            elif key_char == 'p':
                align_config.save()
            
            # Help
            elif key_char == 'h':
                print("\n=== KEYBOARD CONTROLS ===")
                print("  'q' or ESC: Quit")
                print("\nINDEPENDENT EDGE CONTROL:")
                print("  'w'/'s': TOP edge (up/down)")
                print("  'a'/'d': LEFT edge (left/right)")
                print("  'i'/'k': BOTTOM edge (up/down)")
                print("  'j'/'l': RIGHT edge (left/right)")
                print("  Arrow Keys: Quick edge adjustment")
                print("  Hold SHIFT with any key: Fine adjustment (1 pixel vs 5)")
                print("\nOTHER CONTROLS:")
                print("  'r': Reset to default alignment")
                print("  'c': Toggle calibration mode (shows crop overlay)")
                print("  'v': Cycle visualization mode (normal/debug/overlay/edges/blend/checkerboard)")
                print("  '+'/'-': Adjust blend transparency (in blend mode)")
                print("  '1/2': Adjust min threshold distance")
                print("  '3/4': Adjust max threshold distance")
                print("  'p': Save current alignment configuration")
                print("\nALIGNMENT TIPS:")
                print("  1. Press 'c' to see crop region overlay on depth")
                print("  2. Use 'edges' mode - look for YELLOW overlap of red/green edges")
                print("  3. Adjust each edge independently: w/s/a/d for one side, i/k/j/l for opposite")
                print("  4. Use 'blend' mode to verify - objects should not appear doubled")
                print("  5. Use 'checkerboard' mode - look for seamless transitions\n")
        
        binaryBefore = binaryNow

except KeyboardInterrupt:
    print("User interrupted!")

finally:
    # Save configuration before exiting
    align_config.save()
    
    # Terminate RealSense camera object
    myCam1.terminate()
    cv2.destroyAllWindows()
    print("\nProgram terminated. Configuration saved.")
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 