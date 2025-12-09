"""
Test script to check if YOLO camera is working on the QCar
Run this directly on the QCar via SSH or upload and execute
"""
import cv2
import numpy as np
from pit.YOLO.utils import QCar2DepthAligned

print("=" * 60)
print("YOLO Camera Test Script")
print("=" * 60)
print("\nInitializing camera...")

try:
    # Initialize the camera
    QCarImg = QCar2DepthAligned(port='18777')
    print("✓ Camera initialized successfully")
    
    print("\nTesting camera capture...")
    print("Press Ctrl+C to exit\n")
    
    frame_count = 0
    success_count = 0
    
    for i in range(10):  # Test 10 frames
        try:
            QCarImg.read()
            
            if QCarImg.rgb is not None and QCarImg.depth is not None:
                rgb_shape = QCarImg.rgb.shape
                depth_shape = QCarImg.depth.shape
                
                success_count += 1
                print(f"Frame {i+1}: ✓ RGB {rgb_shape}, Depth {depth_shape}")
            else:
                print(f"Frame {i+1}: ✗ Failed to capture")
                
            frame_count += 1
            
        except Exception as e:
            print(f"Frame {i+1}: ✗ Error - {e}")
    
    print("\n" + "=" * 60)
    print(f"Results: {success_count}/{frame_count} frames captured successfully")
    
    if success_count == frame_count:
        print("✓ Camera is working properly!")
    elif success_count > 0:
        print("⚠ Camera is working but with some issues")
    else:
        print("✗ Camera is NOT working")
    
    print("=" * 60)

except Exception as e:
    print(f"✗ Failed to initialize camera: {e}")
    print("\nPossible issues:")
    print("1. Camera not connected")
    print("2. Camera port already in use")
    print("3. Missing dependencies")

finally:
    try:
        QCarImg.terminate()
        print("\nCamera closed successfully")
    except:
        pass
