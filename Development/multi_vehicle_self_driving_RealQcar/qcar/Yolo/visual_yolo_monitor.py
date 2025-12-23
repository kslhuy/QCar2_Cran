"""
Visual YOLO Monitor with Image Display
========================================
Complete visual test for YOLO server - shows both camera feed and detection data.

Usage:
    python visual_yolo_monitor.py --car-id 0
    python visual_yolo_monitor.py --car-id 0 --show-probe

    --- Example with camera feed ---
    python yolo_server_virtual.py -idx 0 -s
"""

import numpy as np
import time
import argparse
import sys
import cv2
from collections import deque

try:
    from YoLo import YOLOReceiver
    from QCar2DepthAlignedCamera import QCar2DepthAlignedCamera
except ImportError:
    print("Error: Could not import required modules")
    print("Make sure you're in the Yolo directory")
    sys.exit(1)

try:
    from pal.utilities.probe import Observer
    PROBE_AVAILABLE = True
except ImportError:
    PROBE_AVAILABLE = False
    print("Warning: Probe utilities not available. Will only show detection visualization.")


class VisualYOLOMonitor:
    """Visual monitor for YOLO with image display"""
    
    def __init__(self, car_id=0, show_probe=False):
        self.car_id = car_id
        self.yolo_port = f'1866{car_id}'
        self.show_probe = show_probe and PROBE_AVAILABLE
        
        self.receiver = None
        self.probe_observer = None
        self.camera = None
        
        self.packet_count = 0
        self.start_time = time.time()
        self.fps_history = deque(maxlen=30)
        self.last_frame_time = time.time()
        
        # Colors (BGR format for OpenCV)
        self.COLORS = {
            'stop_sign': (0, 0, 255),      # Red
            'traffic': (0, 140, 255),       # Orange
            'car': (255, 0, 0),             # Blue
            'yield': (0, 255, 255),         # Yellow
            'person': (0, 255, 0),          # Green
            'text': (255, 255, 255),        # White
            'bg': (40, 40, 40),             # Dark gray
            'header': (200, 200, 0),        # Cyan
        }
        
    def connect(self):
        """Connect to YOLO receiver and optionally probe"""
        print(f"\n{'='*60}")
        print(f"  VISUAL YOLO MONITOR - CAR {self.car_id}")
        print(f"{'='*60}")
        
        # Connect to YOLO receiver
        print(f"Connecting to YOLO port {self.yolo_port}...")
        try:
            self.receiver = YOLOReceiver(port=self.yolo_port)
            print("✓ YOLO Receiver connected!")
        except Exception as e:
            print(f"✗ YOLO connection failed: {e}")
            return False
        
        # Setup probe observer if requested
        if self.show_probe:
            print(f"Setting up Probe Observer for Car {self.car_id}...")
            try:
                self.setup_probe_observer()
                print("✓ Probe Observer setup complete!")
            except Exception as e:
                print(f"⚠ Probe Observer setup failed: {e}")
                print("  Continuing without probe images...")
                self.show_probe = False
        

        
        print(f"{'='*60}\n")
        return True
    
    def setup_probe_observer(self):
        """Setup Observer for probe images (multi_probing approach)"""
        if not PROBE_AVAILABLE:
            return
            
        self.probe_observer = Observer()
        self.probe_observer.ip = '192.168.2.10'
        self.probe_observer.numDisplays = self.car_id
        self.probe_observer.add_display(
            imageSize=[200, 320, 3],
            name=f'QCar {self.car_id} YOLO',
            scalingFactor=1
        )
    
    def create_detection_visualization(self, width=800, height=600):
        """Create a visualization canvas for detections"""
        canvas = np.ones((height, width, 3), dtype=np.uint8) * 40
        
        # Calculate FPS
        current_time = time.time()
        fps = 1.0 / (current_time - self.last_frame_time) if self.last_frame_time else 0
        self.fps_history.append(fps)
        avg_fps = np.mean(self.fps_history) if self.fps_history else 0
        self.last_frame_time = current_time
        
        # Header
        elapsed = current_time - self.start_time
        cv2.putText(canvas, f"YOLO Monitor - Car {self.car_id}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, self.COLORS['header'], 2)
        cv2.putText(canvas, f"Packets: {self.packet_count} | FPS: {avg_fps:.1f} | Time: {elapsed:.1f}s",
                    (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.COLORS['text'], 1)
        
        cv2.line(canvas, (20, 85), (width-20, 85), self.COLORS['text'], 1)
        
        # Get detection counts
        stop_count = int(self.receiver.stopSign[0])
        traffic_count = int(self.receiver.trafficlight[0])
        car_count = int(self.receiver.cars[0])
        yield_count = int(self.receiver.yieldSign[0])
        person_count = int(self.receiver.person[0])
        
        y_offset = 120
        x_left = 40
        x_right = width // 2 + 20
        line_height = 35
        
        # Draw detections in two columns
        # Left column
        self.draw_detection_item(canvas, "STOP SIGNS", stop_count, 
                                 self.receiver.stopSign, x_left, y_offset, 
                                 self.COLORS['stop_sign'], "🛑")
        y_offset += line_height * (1 + max(1, stop_count))
        
        self.draw_detection_item(canvas, "TRAFFIC LIGHTS", traffic_count,
                                 self.receiver.trafficlight, x_left, y_offset,
                                 self.COLORS['traffic'], "🚦")
        y_offset += line_height * (1 + max(1, traffic_count))
        
        self.draw_detection_item(canvas, "CARS", car_count,
                                 self.receiver.cars, x_left, y_offset,
                                 self.COLORS['car'], "🚗")
        
        # Right column
        y_offset = 120
        self.draw_detection_item(canvas, "YIELD SIGNS", yield_count,
                                 self.receiver.yieldSign, x_right, y_offset,
                                 self.COLORS['yield'], "⚠")
        y_offset += line_height * (1 + max(1, yield_count))
        
        self.draw_detection_item(canvas, "PERSONS", person_count,
                                 self.receiver.person, x_right, y_offset,
                                 self.COLORS['person'], "🚶")
        
        # Total summary at bottom
        total = stop_count + traffic_count + car_count + yield_count + person_count
        cv2.rectangle(canvas, (20, height-80), (width-20, height-20),
                     self.COLORS['header'], 2)
        cv2.putText(canvas, f"TOTAL OBJECTS DETECTED: {total}",
                   (width//2 - 150, height-45), cv2.FONT_HERSHEY_SIMPLEX,
                   0.8, self.COLORS['text'], 2)
        
        return canvas
    
    def draw_detection_item(self, canvas, label, count, data_array, x, y, color, icon):
        """Draw a detection item with distances"""
        # Label and count
        cv2.rectangle(canvas, (x-10, y-25), (x+300, y+5), color, -1)
        cv2.putText(canvas, f"{label}: {count}",
                   (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Bar chart
        bar_width = 200
        bar_height = 15
        filled = int((count / 6) * bar_width) if count > 0 else 0
        cv2.rectangle(canvas, (x+320, y-20), (x+320+bar_width, y-5),
                     (80, 80, 80), -1)
        if filled > 0:
            cv2.rectangle(canvas, (x+320, y-20), (x+320+filled, y-5),
                         color, -1)
        
        # Distances
        if count > 0:
            distances_text = []
            for i in range(1, min(count + 1, 7)):
                dist = data_array[i]
                distances_text.append(f"#{i}: {dist:.2f}m")
            
            # Display distances
            for idx, dist_text in enumerate(distances_text):
                cv2.putText(canvas, dist_text, (x+20, y+25+idx*20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS['text'], 1)
    
    def create_combined_view(self, probe_img=None, camera_img=None):
        """Create combined view with images and detection data"""
        detection_viz = self.create_detection_visualization(800, 600)
        
        if probe_img is not None or camera_img is not None:
            # Determine which image to use
            img_to_show = probe_img if probe_img is not None else camera_img
            
            # Resize image to fit
            img_height = 480
            aspect_ratio = img_to_show.shape[1] / img_to_show.shape[0]
            img_width = int(img_height * aspect_ratio)
            img_resized = cv2.resize(img_to_show, (img_width, img_height))
            
            # Create combined canvas
            total_width = 800 + img_width + 20
            combined = np.ones((600, total_width, 3), dtype=np.uint8) * 40
            
            # Place detection visualization
            combined[0:600, 0:800] = detection_viz
            
            # Place image
            y_offset = (600 - img_height) // 2
            combined[y_offset:y_offset+img_height, 820:820+img_width] = img_resized
            
            # Label the image
            cv2.putText(combined, "YOLO Camera Feed", (820, y_offset-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.COLORS['header'], 2)
            
            return combined
        else:
            return detection_viz
    
    def read_probe_image(self):
        """Read image from probe observer
        
        Note: With Observer approach, images are displayed in separate windows.
        This method returns None as Observer handles display independently.
        """
        # Observer displays images in its own window, so we don't read them here
        return None
    
    def read_camera_image(self):
        """Read image from camera"""
        if not self.use_camera or not self.camera:
            return None
        
        try:
            self.camera.read()
            return self.camera.rgb.copy()
        except:
            pass
        return None
    
    def run(self, update_hz=10.0):
        """Run the visual monitor"""
        print("\n" + "="*60)
        print("VISUAL MONITOR STARTED")
        print("="*60)
        print(f"Update Rate: {update_hz} Hz")
        if self.show_probe:
            print(f"Probe Observer: ENABLED (Car {self.car_id})")
            print("  Note: YOLO feed will display in separate Observer window")
        if self.use_camera:
            print("Camera: ENABLED")
        print("\nPress 'q' to quit, 's' to save screenshot")
        print("="*60 + "\n")
        
        cv2.namedWindow('YOLO Visual Monitor', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('YOLO Visual Monitor', 1200, 600)
        
        update_interval = 1.0 / update_hz
        no_data_count = 0
        screenshot_count = 0
        
        try:
            while True:
                loop_start = time.time()
                
                # Read YOLO data
                new_data = self.receiver.read()
                if new_data:
                    self.packet_count += 1
                    no_data_count = 0
                else:
                    no_data_count += 1
                
                # Read images (probe_img will be None when using Observer)
                probe_img = self.read_probe_image()
                camera_img = self.read_camera_image()
                
                # Launch probe observer if not already launched
                if self.show_probe and self.probe_observer and not hasattr(self.probe_observer, '_launched'):
                    try:
                        import threading
                        thread = threading.Thread(target=self.probe_observer.launch, daemon=True)
                        thread.start()
                        self.probe_observer._launched = True
                        print("✓ Probe Observer window launched")
                    except Exception as e:
                        print(f"⚠ Failed to launch Observer window: {e}")
                
                # Create visualization
                display = self.create_combined_view(probe_img, camera_img)
                
                # Add warning if no data
                if no_data_count > 20:
                    cv2.putText(display, "WARNING: NO DATA FROM YOLO SERVER",
                               (display.shape[1]//2 - 250, display.shape[0]//2),
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                
                # Show
                cv2.imshow('YOLO Visual Monitor', display)
                
                # Handle key press
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\nQuitting...")
                    break
                elif key == ord('s'):
                    filename = f"yolo_monitor_screenshot_{screenshot_count:03d}.png"
                    cv2.imwrite(filename, display)
                    print(f"Screenshot saved: {filename}")
                    screenshot_count += 1
                
                # Maintain update rate
                elapsed = time.time() - loop_start
                sleep_time = max(0, update_interval - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n\nMonitor interrupted by user")
        finally:
            cv2.destroyAllWindows()
    
    def cleanup(self):
        """Cleanup resources"""
        print("\nCleaning up...")
        if self.receiver:
            self.receiver.terminate()
            print("✓ YOLO receiver terminated")
        if self.probe_observer and PROBE_AVAILABLE:
            try:
                self.probe_observer.terminate()
                print("✓ Probe Observer terminated")
            except:
                pass
        if self.camera:
            try:
                self.camera.terminate()
                print("✓ Camera terminated")
            except:
                pass


def main():
    parser = argparse.ArgumentParser(
        description='Visual YOLO monitor with image display',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python visual_yolo_monitor.py --car-id 0
  python visual_yolo_monitor.py --car-id 0 --show-probe
  python visual_yolo_monitor.py --car-id 0 --use-camera
  python visual_yolo_monitor.py --car-id 0 --show-probe --rate 15
        """
    )
    parser.add_argument('--car-id', type=int, default=0,
                        help='Car ID (affects ports)')
    parser.add_argument('--rate', type=float, default=10.0,
                        help='Update rate in Hz (default: 10.0)')
    parser.add_argument('--show-probe', action='store_true',
                        help='Show images from probe stream')
    parser.add_argument('--use-camera', action='store_true',
                        help='Use direct camera feed (alternative to probe)')
    
    args = parser.parse_args()
    
    monitor = VisualYOLOMonitor(
        car_id=args.car_id,
        show_probe=args.show_probe,
        use_camera=args.use_camera
    )
    
    try:
        if monitor.connect():
            monitor.run(update_hz=args.rate)
    finally:
        monitor.cleanup()
        print("\nVisual monitor closed.")


if __name__ == "__main__":
    main()
