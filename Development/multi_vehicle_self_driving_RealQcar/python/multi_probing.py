#!/usr/bin/env python3
"""
multi_probing for Multiple QCar YOLO Streams using ZMQ

Receives ZMQ video streams from yolo_server_virtual.py and displays them using OpenCV.
Completely independent of Quanser 'pal' utilities.
"""

import cv2
import time
import argparse
import threading
import sys
import os
import numpy as np

# Ensure we can import from local directory
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

try:
    from qcar.Yolo.YoLo import YOLOVideoReceiver
except ImportError:
    # Fallback if running from a different directory structure
    sys.path.append(os.path.join(parent_dir, 'qcar', 'Yolo'))
    from YoLo import YOLOVideoReceiver


class VideoStreamViewer:
    """
    Handles receiving and displaying video for a single car.
    """
    def __init__(self, car_id, ip='localhost', window_name=None, scale=1.0, port=None):
        self.car_id = car_id
        self.window_name = window_name if window_name else f"QCar {car_id} - YOLO Stream"
        self.scale = scale
        
        # ZMQ Video Receiver (port 1876x)
        self.ip = ip
        self.port = str(port if port is not None else (18760 + car_id))
        self.receiver = None
        self.frame = None
        self.running = False
        self.connected = False
        self.last_frame_time = 0
        self.start_time = 0
        self.fps = 0.0
        
        # Threading
        self.lock = threading.Lock()
        self.thread = None

    def start(self):
        """Start the receiver thread."""
        self.start_time = time.time()
        self.running = True
        self.thread = threading.Thread(target=self._update_loop, daemon=True)
        self.thread.start()
        print(f"[Viewer {self.car_id}] API started on port {self.port}")

    def stop(self):
        """Stop the receiver thread."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.receiver:
            self.receiver.terminate()

    def _update_loop(self):
        """Background loop to fetch frames from ZMQ."""
        # Initialize receiver in the thread (ZMQ context is thread-safe mostly, but safer this way)
        self.receiver = YOLOVideoReceiver(ip=self.ip, port=self.port)
        
        while self.running:
            frame = self.receiver.read()
            if frame is not None:
                with self.lock:
                    self.frame = frame
                    self.connected = True
                    
                    # Calculate FPS
                    now = time.time()
                    if self.last_frame_time > 0:
                        dt = now - self.last_frame_time
                        if dt > 0:
                            current_fps = 1.0 / dt
                            self.fps = 0.9 * self.fps + 0.1 * current_fps
                    self.last_frame_time = now
            else:
                # No frame received
                time.sleep(0.005) # Yield slightly

    def get_frame(self):
        """Get the latest frame (thread-safe)."""
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

def main():
    parser = argparse.ArgumentParser(description="Single-QCar ZMQ Video Viewer")
    parser.add_argument('--car', type=int, default=0,
                        help='Car ID to observe (default: 0)')
    # Backward compatibility: older scripts/docs may still pass --cars for single car ID.
    parser.add_argument('--cars', type=int, dest='car',
                        help='Deprecated alias for --car')
    parser.add_argument('--ip', type=str, default='localhost',
                        help='IP address of the car (default: localhost)')
    parser.add_argument('--scale', type=float, default=1.0, help='Image scaling factor')
    parser.add_argument('--port', type=int, default=None,
                        help='Video ZMQ port override (default: 18760 + car_id)')
    
    args = parser.parse_args()
    
    print("="*60)
    print(f" ZMQ Video Probe for QCar {args.car}")
    print("="*60)
    
    # Start single viewer
    viewer = VideoStreamViewer(args.car, ip=args.ip, scale=args.scale, port=args.port)
    viewer.start()
    cv2.namedWindow(viewer.window_name, cv2.WINDOW_NORMAL)
    
    print("\nPress 'q' or 'Esc' to exit.\n")
    last_wait_log = 0.0
    
    try:
        while True:
            # Main GUI loop - OpenCV imshow must run in main thread
            frame = viewer.get_frame()
            
            if frame is not None:
                # Optional scaling
                if viewer.scale != 1.0:
                    frame = cv2.resize(frame, None, fx=viewer.scale, fy=viewer.scale)
                
                # Overlay info
                status_color = (0, 255, 0) # Green
                cv2.putText(frame, f"Car {args.car} | FPS: {viewer.fps:.1f}", (10, 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
                
                cv2.imshow(viewer.window_name, frame)
            else:
                # Show explicit waiting screen so users can see probing is running.
                waiting = np.zeros((240, 480, 3), dtype=np.uint8)
                elapsed = time.time() - viewer.start_time if viewer.start_time > 0 else 0.0
                cv2.putText(waiting, f"QCar {args.car} - waiting for video stream", (15, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
                cv2.putText(waiting, f"Target: tcp://{args.ip}:{viewer.port}", (15, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
                cv2.putText(waiting, f"Elapsed: {elapsed:.1f}s", (15, 135),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
                cv2.putText(waiting, "Check: perception active, network reachable, yolo_<id>.log", (15, 175),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
                cv2.imshow(viewer.window_name, waiting)

                # Print a periodic terminal hint if no frames arrive.
                if elapsed >= 5.0 and (time.time() - last_wait_log) >= 5.0:
                    print(f"[Viewer {args.car}] Still waiting for frames from tcp://{args.ip}:{viewer.port} ({elapsed:.1f}s)")
                    last_wait_log = time.time()
        
            # Check for quit key
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q') or key == 27:
                break
                
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("Stopping viewer...")
        viewer.stop()
        cv2.destroyAllWindows()
        print("Done.")

if __name__ == "__main__":
    main()
