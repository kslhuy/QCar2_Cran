"""
Complete YOLO Test with Real-Time Image Display
================================================
This script creates a complete testing environment showing:
- Camera feed with YOLO annotations
- Detection statistics
- Real-time performance metrics

Usage:
    python test_yolo_with_images.py --car-id 0
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
    from pit.YOLO.nets import YOLOv8
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Make sure you're in the correct directory")
    sys.exit(1)


class CompleteYOLOTester:
    """Complete YOLO testing with visualization"""
    
    def __init__(self, car_id=0):
        self.car_id = car_id
        self.yolo_port = f'1866{car_id}'
        
        # Components
        self.receiver = None
        self.camera = None
        self.yolo_model = None
        
        # Statistics
        self.frame_count = 0
        self.packet_count = 0
        self.start_time = time.time()
        self.fps_deque = deque(maxlen=30)
        self.last_time = time.time()
        self.server_available = False
        self.server_checked = False
        
    def initialize(self):
        """Initialize all components"""
        print("\n" + "="*70)
        print(" "*20 + "COMPLETE YOLO TESTER")
        print("="*70)
        
        # Initialize camera
        print("\n[1/3] Initializing Camera...")
        try:
            self.camera = QCar2DepthAlignedCamera(
                imageWidth=640,
                imageHeight=480,
                use_intrinsics=True,
                clipping_distance=10.0,
                video3dPort=18805,
                load_settings=True
            )
            print("      ✓ Camera initialized")
        except Exception as e:
            print(f"      ✗ Camera failed: {e}")
            return False
        
        # Initialize YOLO model
        print("\n[2/3] Loading YOLO Model...")
        try:
            self.yolo_model = YOLOv8(
                imageHeight=480,
                imageWidth=640
            )
            print("      ✓ YOLO model loaded")
        except Exception as e:
            print(f"      ✗ YOLO model failed: {e}")
            return False
        
        # Initialize receiver (optional - for comparison)
        print(f"\n[3/3] Connecting to YOLO Server (port {self.yolo_port})...")
        try:
            self.receiver = YOLOReceiver(port=self.yolo_port)
            # Try a test read to verify server is actually working
            test_data = self.receiver.read()
            if test_data:
                self.server_available = True
                print("      ✓ Connected to server")
                print("      (Will compare with local YOLO processing)")
            else:
                print("      ⚠ Server connected but not sending data")
                print("      (Will run local YOLO only)")
                self.receiver = None
        except Exception as e:
            print(f"      ⚠ Server connection failed: {e}")
            print("      (Will run local YOLO only)")
            self.receiver = None
        
        print("\n" + "="*70)
        print("✓ ALL SYSTEMS READY")
        print("="*70 + "\n")
        return True
    
    def calculate_fps(self):
        """Calculate current FPS"""
        current = time.time()
        fps = 1.0 / (current - self.last_time) if self.last_time else 0
        self.fps_deque.append(fps)
        self.last_time = current
        return np.mean(self.fps_deque) if self.fps_deque else 0
    
    def process_frame(self):
        """Process one frame with YOLO"""
        # Read camera
        self.camera.read()
        rgb = self.camera.rgb.copy()
        depth = self.camera.depth.copy()
        
        # Run YOLO detection
        rgb_processed = self.yolo_model.pre_process(rgb)
        prediction = self.yolo_model.predict(
            inputImg=rgb_processed,
            classes=[0, 2, 9, 11, 33],  # person, car, traffic light, stop sign, yield
            confidence=0.4,
            half=True,
            verbose=False
        )
        processed_results = self.yolo_model.post_processing(
            alignedDepth=depth,
            clippingDistance=10
        )
        annotated = self.yolo_model.post_process_render(showFPS=False)
        
        # Count detections
        detections = {
            'stop_sign': 0,
            'traffic': 0,
            'car': 0,
            'yield': 0,
            'person': 0
        }
        
        detection_details = {key: [] for key in detections.keys()}
        
        for result in processed_results:
            if 'stop sign' in result.name:
                detections['stop_sign'] += 1
                detection_details['stop_sign'].append(result.distance)
            elif 'red' in result.name or 'traffic' in result.name.lower():
                detections['traffic'] += 1
                detection_details['traffic'].append(result.distance)
            elif 'car' in result.name:
                detections['car'] += 1
                detection_details['car'].append(result.distance)
            elif 'yield' in result.name:
                detections['yield'] += 1
                detection_details['yield'].append(result.distance)
            elif 'person' in result.name:
                detections['person'] += 1
                detection_details['person'].append(result.distance)
        
        self.frame_count += 1
        return annotated, detections, detection_details
    
    def read_server_data(self):
        """Read data from YOLO server if connected"""
        if not self.receiver or not self.server_available:
            return None
        
        try:
            new_data = self.receiver.read()
            if new_data:
                self.packet_count += 1
                self.server_checked = True
                return {
                    'stop_sign': int(self.receiver.stopSign[0]),
                    'traffic': int(self.receiver.trafficlight[0]),
                    'car': int(self.receiver.cars[0]),
                    'yield': int(self.receiver.yieldSign[0]),
                    'person': int(self.receiver.person[0])
                }
        except Exception as e:
            # Server failed during operation - disable it
            if not self.server_checked:
                print(f"\n⚠ Server read failed: {e}")
                print("  Disabling server comparison (will use local YOLO only)\n")
                self.server_checked = True
            self.server_available = False
            self.receiver = None
        
        return None
    
    def create_info_panel(self, width=400, height=480):
        """Create information panel"""
        panel = np.ones((height, width, 3), dtype=np.uint8) * 50
        
        # Header
        cv2.putText(panel, "YOLO TEST INFO", (20, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Runtime info
        fps = self.calculate_fps()
        elapsed = time.time() - self.start_time
        
        y = 80
        info_lines = [
            f"FPS: {fps:.1f}",
            f"Frames: {self.frame_count}",
            f"Runtime: {elapsed:.1f}s",
            f"Car ID: {self.car_id}",
        ]
        
        for line in info_lines:
            cv2.putText(panel, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX,
                       0.6, (255, 255, 255), 1)
            y += 30
        
        return panel
    
    def create_detection_panel(self, local_detections, local_details, 
                               server_detections=None, width=400, height=480):
        """Create detection comparison panel"""
        panel = np.ones((height, width, 3), dtype=np.uint8) * 50
        
        # Header
        cv2.putText(panel, "DETECTIONS", (20, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        y = 80
        colors = {
            'stop_sign': (0, 0, 255),
            'traffic': (0, 140, 255),
            'car': (255, 0, 0),
            'yield': (0, 255, 255),
            'person': (0, 255, 0)
        }
        
        labels = {
            'stop_sign': 'Stop Signs',
            'traffic': 'Traffic Lights',
            'car': 'Cars',
            'yield': 'Yield Signs',
            'person': 'Persons'
        }
        
        for key, label in labels.items():
            color = colors[key]
            count = local_detections[key]
            
            # Draw label
            cv2.putText(panel, f"{label}:", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # Draw count with colored box
            cv2.rectangle(panel, (200, y-15), (240, y+5), color, -1)
            cv2.putText(panel, f"{count}", (210, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Show comparison with server if available
            if server_detections:
                server_count = server_detections[key]
                match = "✓" if count == server_count else "✗"
                match_color = (0, 255, 0) if count == server_count else (0, 0, 255)
                cv2.putText(panel, f"Srv:{server_count} {match}", (260, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, match_color, 1)
            
            y += 30
            
            # Show distances if any
            if count > 0 and local_details[key]:
                distances = local_details[key][:3]  # Show first 3
                dist_text = ", ".join([f"{d:.1f}m" for d in distances])
                cv2.putText(panel, f"  {dist_text}", (30, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
                y += 25
        
        # Server connection status
        y += 20
        if server_detections:
            cv2.putText(panel, "Server: CONNECTED", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        else:
            cv2.putText(panel, "Server: NOT CONNECTED", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # Instructions
        y = height - 60
        cv2.putText(panel, "Press 'q' to quit", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(panel, "Press 's' for screenshot", (20, y+25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return panel
    
    def run(self, duration=None):
        """Run the test"""
        print("Starting visual test...")
        print("Press 'q' to quit, 's' to save screenshot\n")
        
        cv2.namedWindow('YOLO Test', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('YOLO Test', 1400, 600)
        
        screenshot_count = 0
        
        try:
            while True:
                if duration and (time.time() - self.start_time) > duration:
                    print("\nTest duration completed")
                    break
                
                # Process frame
                annotated, detections, details = self.process_frame()
                
                # Read server data
                server_data = self.read_server_data()
                
                # Create panels
                info_panel = self.create_info_panel(400, 480)
                detection_panel = self.create_detection_panel(
                    detections, details, server_data, 400, 480
                )
                
                # Resize annotated image
                annotated_resized = cv2.resize(annotated, (640, 480))
                
                # Combine all views
                top_row = np.hstack([annotated_resized, info_panel])
                bottom_row = np.hstack([
                    np.ones((120, 640, 3), dtype=np.uint8) * 50,
                    detection_panel[:120, :]
                ])
                full_view = np.vstack([top_row, bottom_row])
                
                # Add main title
                cv2.putText(full_view, f"YOLO COMPLETE TEST - Car {self.car_id}",
                           (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                
                # Display
                cv2.imshow('YOLO Test', full_view)
                
                # Handle keys
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    filename = f"yolo_test_{screenshot_count:03d}.png"
                    cv2.imwrite(filename, full_view)
                    print(f"Screenshot saved: {filename}")
                    screenshot_count += 1
                
                time.sleep(0.01)  # Small delay
                
        except KeyboardInterrupt:
            print("\n\nTest interrupted")
        finally:
            cv2.destroyAllWindows()
    
    def cleanup(self):
        """Cleanup resources"""
        print("\nCleaning up...")
        if self.camera:
            self.camera.terminate()
            print("✓ Camera terminated")
        if self.receiver:
            self.receiver.terminate()
            print("✓ Receiver terminated")
        
        # Print statistics
        print("\n" + "="*70)
        print("TEST STATISTICS")
        print("="*70)
        print(f"Total Frames: {self.frame_count}")
        print(f"Total Packets: {self.packet_count}")
        elapsed = time.time() - self.start_time
        print(f"Total Time: {elapsed:.1f}s")
        print(f"Average FPS: {self.frame_count/elapsed:.1f}")
        print("="*70)


def main():
    parser = argparse.ArgumentParser(description='Complete YOLO test with images')
    parser.add_argument('--car-id', type=int, default=0,
                        help='Car ID')
    parser.add_argument('--duration', type=int, default=None,
                        help='Test duration in seconds (default: run until quit)')
    
    args = parser.parse_args()
    
    tester = CompleteYOLOTester(car_id=args.car_id)
    
    try:
        if tester.initialize():
            tester.run(duration=args.duration)
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()
