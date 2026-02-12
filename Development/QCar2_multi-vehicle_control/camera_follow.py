"""
Camera Follow Script - Follows vehicle 0 with fixed relative offset

This script creates a free camera that follows vehicle 0 (leader) with a 
fixed relative position offset. The camera does not rotate with the vehicle.

Relative offset configuration:
- Forward: 15m (ahead of the vehicle)
- Right: 15m (to the right side of the vehicle)
- Up: 10m (above the vehicle)
"""
import time
import numpy as np
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.free_camera import QLabsFreeCamera

# Configuration
VEHICLE_ACTOR_NUMBER = 0  # Follow vehicle 0
CAMERA_ACTOR_NUMBER = 10  # Use actor number 10 for camera

# Relative offset from vehicle (in vehicle's local coordinate frame)
OFFSET_FORWARD = 15.0  # meters ahead
OFFSET_RIGHT = 15.0    # meters to the right
OFFSET_UP = 10.0       # meters above

# Fixed camera rotation (does not follow vehicle rotation)
CAMERA_ROTATION_DEGREES = [0, 30, 135]  # [roll, pitch, yaw] looking down at vehicle

# Update frequency
UPDATE_RATE_HZ = 60  # 60Hz update rate
UPDATE_INTERVAL = 1.0 / UPDATE_RATE_HZ


def calculate_world_position(car_location, car_rotation, forward_vec):
    """
    Calculate camera's world position based on vehicle position and orientation.
    
    Args:
        car_location: [x, y, z] position of the vehicle
        car_rotation: [roll, pitch, yaw] rotation of the vehicle in radians
        forward_vec: Forward direction vector from vehicle state
        
    Returns:
        [x, y, z] world position for the camera
    """
    # Normalize forward vector (just in case)
    forward = np.array(forward_vec[:2])  # Only x, y components
    if np.linalg.norm(forward) > 0:
        forward = forward / np.linalg.norm(forward)
    else:
        # Fallback: use yaw angle
        yaw = car_rotation[2]
        forward = np.array([np.cos(yaw), np.sin(yaw)])
    
    # Calculate right vector (perpendicular to forward, in 2D plane)
    # Right = rotate forward by -90 degrees (clockwise when looking down)
    right = np.array([forward[1], -forward[0]])
    
    # Calculate position offset in world coordinates
    offset_x = OFFSET_FORWARD * forward[0] + OFFSET_RIGHT * right[0]
    offset_y = OFFSET_FORWARD * forward[1] + OFFSET_RIGHT * right[1]
    offset_z = OFFSET_UP
    
    # Calculate camera world position
    camera_x = car_location[0] + offset_x
    camera_y = car_location[1] + offset_y
    camera_z = car_location[2] + offset_z
    
    return [camera_x, camera_y, camera_z]


def main():
    print("=" * 60)
    print("Camera Follow Script")
    print("=" * 60)
    print(f"Following vehicle ID: {VEHICLE_ACTOR_NUMBER}")
    print(f"Relative offset: Forward={OFFSET_FORWARD}m, Right={OFFSET_RIGHT}m, Up={OFFSET_UP}m")
    print(f"Camera rotation: {CAMERA_ROTATION_DEGREES} degrees (fixed)")
    print(f"Update rate: {UPDATE_RATE_HZ} Hz")
    print("=" * 60)
    
    # Connect to QLabs
    qlabs = QuanserInteractiveLabs()
    
    print("\nConnecting to QLabs...")
    try:
        qlabs.open("localhost")
        print("Connected to QLabs successfully!")
    except Exception as e:
        print(f"Failed to connect to QLabs: {e}")
        print("Please ensure QLabs is running.")
        return
    
    # Create vehicle and camera instances
    vehicle = QLabsQCar2(qlabs)
    vehicle.actorNumber = VEHICLE_ACTOR_NUMBER
    
    camera = QLabsFreeCamera(qlabs)
    camera.actorNumber = CAMERA_ACTOR_NUMBER
    
    # Spawn camera (initial position will be updated immediately)
    print(f"\nSpawning camera (actor #{CAMERA_ACTOR_NUMBER})...")
    status = camera.spawn_degrees(
        location=[0, 0, 10],  # Temporary initial position
        rotation=CAMERA_ROTATION_DEGREES
    )
    
    if not status:
        print("Failed to spawn camera!")
        qlabs.close()
        return
    
    print("Camera spawned successfully!")
    
    # Possess the camera to view from it
    print("Switching to camera view...")
    if camera.possess():
        print("Camera view activated!")
    else:
        print("Warning: Failed to possess camera, but continuing anyway...")
    
    print("\nStarting camera follow loop...")
    print("Press Ctrl+C to stop following.\n")
    
    try:
        loop_count = 0
        last_print_time = time.time()
        
        while True:
            loop_start_time = time.time()
            
            # Get current vehicle state (read-only, does not modify vehicle)
            success, car_location, car_rotation_deg, scale = vehicle.get_world_transform_degrees()
            
            if not success:
                print("Warning: Failed to get vehicle state!")
                time.sleep(UPDATE_INTERVAL)
                continue
            
            # Convert rotation to radians for calculation
            car_rotation_rad = [r * np.pi / 180.0 for r in car_rotation_deg]
            
            # Calculate forward vector from yaw angle
            yaw = car_rotation_rad[2]
            forward_vec = [np.cos(yaw), np.sin(yaw), 0]
            
            # Calculate camera world position
            camera_position = calculate_world_position(car_location, car_rotation_rad, forward_vec)
            
            # Update camera position
            camera.set_transform_degrees(
                location=camera_position,
                rotation=CAMERA_ROTATION_DEGREES
            )
            
            # Print status every second
            current_time = time.time()
            if current_time - last_print_time >= 1.0:
                print(f"[{loop_count:6d}] Vehicle: ({car_location[0]:7.2f}, {car_location[1]:7.2f}, {car_location[2]:6.2f}) | "
                      f"Camera: ({camera_position[0]:7.2f}, {camera_position[1]:7.2f}, {camera_position[2]:6.2f})")
                last_print_time = current_time
            
            loop_count += 1
            
            # Maintain update rate
            elapsed = time.time() - loop_start_time
            sleep_time = max(0, UPDATE_INTERVAL - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n\nCamera follow stopped by user.")
    except Exception as e:
        print(f"\n\nError occurred: {e}")
    finally:
        print("\nCleaning up...")
        qlabs.close()
        print("Disconnected from QLabs.")
        print("Camera follow script terminated.")


if __name__ == "__main__":
    main()
