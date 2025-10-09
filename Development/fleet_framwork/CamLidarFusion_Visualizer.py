"""
Standalone CamLidarFusion Visualizer

This script provides visualization capabilities for CamLidarFusion in a single-process environment.
It creates GUI windows for LiDAR point cloud visualization and camera feed with overlays.

Usage:
    python CamLidarFusion_Visualizer.py --vehicle-id 1 --following-target 0

This allows you to visualize the CamLidarFusion output for a specific vehicle
without running the full multi-process fleet simulation.
"""

import os
import sys
import time
import argparse
import threading
import signal

# Add the current directory to Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

# Import QLabs and QCar libraries
try:
    from qvl.qlabs import QuanserInteractiveLabs
    from qvl.qcar2 import QLabsQCar2
    QLABS_AVAILABLE = True
except ImportError:
    print("Warning: QLabs libraries not available")
    QLABS_AVAILABLE = False

# Import CamLidarFusion
from CamLidarFusion import CamLidarFusion

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='CamLidarFusion Visualizer')
    parser.add_argument('--vehicle-id', type=int, default=1,
                       help='Vehicle ID to visualize (default: 1)')
    parser.add_argument('--following-target', type=int, default=0,
                       help='Vehicle ID this one is following (default: 0)')
    parser.add_argument('--image-width', type=int, default=640,
                       help='Camera image width (default: 640)')
    parser.add_argument('--image-height', type=int, default=480,
                       help='Camera image height (default: 480)')
    parser.add_argument('--scale-factor', type=float, default=0.1,
                       help='Coordinate transformation scale factor (default: 0.1)')
    parser.add_argument('--verbose', action='store_true',
                       help='Enable verbose output')
    parser.add_argument('--max-frames', type=int, default=1000,
                       help='Maximum number of frames to process (default: 1000)')

    return parser.parse_args()

def setup_qlabs_environment(vehicle_id, following_target):
    """Setup QLabs environment for visualization."""
    if not QLABS_AVAILABLE:
        print("Error: QLabs libraries not available")
        return None, None

    try:
        # Initialize QLabs
        qlabs = QuanserInteractiveLabs()
        if not qlabs.open("localhost"):
            print("Error: Could not connect to QLabs")
            return None, None

        # Create QCar for the vehicle
        qcar = QLabsQCar2(qlabs)
        qcar.spawn_id(actorNumber=vehicle_id, location=[-8.700 + vehicle_id * 2, 14.643, 0.005],
                     rotation=[0, 0, 0], waitForConfirmation=True)

        # Create target vehicle if following someone
        if following_target >= 0 and following_target != vehicle_id:
            target_qcar = QLabsQCar2(qlabs)
            target_qcar.spawn_id(actorNumber=following_target,
                               location=[-8.700 + following_target * 2, 24.643, 0.005],
                               rotation=[0, 0, 0], waitForConfirmation=True)
            target_qcar.set_velocity_and_request_state(forward=0.3, turn=0, headlights=True,
                                                     leftTurnSignal=False, rightTurnSignal=False,
                                                     brakeSignal=False, reverseSignal=False)

        return qlabs, qcar

    except Exception as e:
        print(f"Error setting up QLabs environment: {e}")
        return None, None

def main():
    """Main visualization function."""
    args = parse_arguments()

    print("=== CamLidarFusion Visualizer ===")
    print(f"Vehicle ID: {args.vehicle_id}")
    print(f"Following Target: {args.following_target}")
    print(f"Image Size: {args.image_width}x{args.image_height}")
    print(f"Scale Factor: {args.scale_factor}")
    print(f"Verbose: {args.verbose}")
    print(f"Max Frames: {args.max_frames}")

    # Setup QLabs environment
    qlabs, qcar = setup_qlabs_environment(args.vehicle_id, args.following_target)
    if qcar is None:
        print("Failed to setup QLabs environment")
        return

    try:
        # Initialize CamLidarFusion with visualization enabled
        fusion = CamLidarFusion(
            qcar=qcar,
            vehicle_id=args.vehicle_id,
            image_width=args.image_width,
            image_height=args.image_height,
            scale_factor=args.scale_factor,
            verbose=args.verbose,
            enable_visualization=True,  # Enable visualization for standalone mode
            show_lidar_overlay=True
        )

        if not fusion.initialized:
            print("Failed to initialize CamLidarFusion")
            return

        print("\n=== Starting Visualization ===")
        print("Controls:")
        print("  'q' - Quit visualization")
        print("  'l' - Toggle LiDAR overlay on camera")
        print("  'p' - Toggle LiDAR plot window")
        print("Close the windows to exit")

        # Main visualization loop
        frame_count = 0
        running = True

        def signal_handler(signum, frame):
            nonlocal running
            print("\nShutdown signal received")
            running = False

        signal.signal(signal.SIGINT, signal_handler)

        while running and frame_count < args.max_frames:
            try:
                # Update sensor data and visualization
                success = fusion.update_sensor_data()

                if success:
                    frame_count += 1
                    if args.verbose and frame_count % 10 == 0:
                        print(f"Processed frame {frame_count}")

                    # Get current distances for display
                    distances = fusion.get_relative_distances()
                    if distances:
                        print(f"Frame {frame_count}: Detected {len(distances)} objects")
                        for key, data in distances.items():
                            print(".1f")

                # Process Qt events to keep GUI responsive
                if hasattr(fusion, 'lidar_app') and fusion.lidar_app is not None:
                    try:
                        fusion.lidar_app.processEvents()
                    except:
                        pass

                # Small delay to prevent excessive CPU usage
                time.sleep(0.1)

            except KeyboardInterrupt:
                print("\nVisualization interrupted by user")
                running = False
            except Exception as e:
                print(f"Error in visualization loop: {e}")
                running = False

        print(f"\nVisualization complete. Processed {frame_count} frames.")

    except Exception as e:
        print(f"Error during visualization: {e}")
    finally:
        # Cleanup
        if fusion:
            fusion.reset()

        if qlabs:
            qlabs.close()

        print("Visualization shutdown complete.")

if __name__ == "__main__":
    main()