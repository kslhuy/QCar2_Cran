#!/usr/bin/env python3
"""
multi-probing for Multiple QCar YOLO Streams

Creates separate observer windows for each QCar that is probing.
Each observer listens on a different port to avoid conflicts.
"""

from pal.utilities.probe import Observer
import time
import threading
import argparse

def create_observer(car_id, width=320, height=200):
    """Create an observer for a specific car
    
    Each observer has a unique display counter (numDisplays) set to car_id.
    This allows multiple vehicles to send their YOLO streams to different windows.
    """
    observer = Observer()
    # + 50 conflict with other ports already used
    observer.numDisplays = car_id + 50 # Unique counter for each car's YOLO stream
    observer.add_display(
        imageSize=[height, width, 3],
        name=f'YOLO Car {car_id}',  # Must match name in yolo_server_virtual.py
        scalingFactor=1
    )
    print(f"  [✓] Observer for QCar {car_id} created (Display ID: {car_id})")
    return observer

def launch_observer(observer, car_id):
    """Launch observer in a separate thread
    
    Each observer runs in its own thread to handle YOLO streams independently.
    """
    print(f"  [→] Launching observer window for QCar {car_id}...")
    try:
        observer.launch()
        print(f"  [✓] Observer for QCar {car_id} started successfully")
    except Exception as e:
        print(f"  [✗] Error in QCar {car_id} observer: {e}")
        import traceback
        traceback.print_exc()

def main():
    parser = argparse.ArgumentParser(description="Multi-QCar Observer")
    parser.add_argument('--cars', type=int, nargs='+', default=[0 , 1],
                        help='List of car IDs to observe (e.g., --cars 0 1 2)')
    parser.add_argument('--width', type=int, default=320, help='Image width')
    parser.add_argument('--height', type=int, default=200, help='Image height')
    
    args = parser.parse_args()
    
    print("="*70)
    print(" Multi-Vehicle YOLO Observer System")
    print("="*70)
    print(f"Number of vehicles: {len(args.cars)}")
    print(f"Car IDs: {args.cars}")
    print(f"Image size: {args.width}x{args.height}")
    print("="*70)
    print()
    
    observers = []
    threads = []
    
    # Create observers for each car
    for car_id in args.cars:
        observer = create_observer(car_id, args.width, args.height)
        observers.append(observer)
        
        # Launch each observer in a separate thread
        thread = threading.Thread(
            target=launch_observer, 
            args=(observer, car_id),
            daemon=True,
            name=f"Observer-Car{car_id}"
        )
        thread.start()
        threads.append(thread)
        
        time.sleep(0.5)  # Stagger the launches to avoid conflicts
    
    # Wait for observers to establish connections
    print("\nWaiting for YOLO servers to connect...")
    
    # Track which observers are already connected
    connected_status = {idx: False for idx in range(len(observers))}
    first_frame_received = {idx: False for idx in range(len(observers))}
    
    while True:
        time.sleep(1.0)
        
        # Check each observer
        for idx, observer in enumerate(observers):
            car_id = args.cars[idx]
            
            if not connected_status[idx]:
                # Check for initial connection
                if len(observer.agentList) > 0:
                    agent = observer.agentList[0]
                    if hasattr(agent, 'connected') and agent.connected:
                        print(f"  [✓] QCar {car_id}: Connected to YOLO stream")
                        connected_status[idx] = True
            elif not first_frame_received[idx]:
                # Check if receiving data (counter > 0 means data received)
                if len(observer.agentList) > 0:
                    agent = observer.agentList[0]
                    if hasattr(agent, 'counter') and agent.counter > 0:
                        print(f"  [✓] QCar {car_id}: Receiving video frames - window should be visible")
                        first_frame_received[idx] = True
        
        # Check if all connected
        if all(connected_status.values()):
            break
    
    print()
    print("="*70)
    print(f" All {len(observers)} observer(s) connected and receiving data!")
    print("="*70)
    print("\nPress Ctrl+C to stop all observers...")
    print()
    
    try:
        # Keep main thread alive
        while True:
            time.sleep(5)
    except KeyboardInterrupt:
        print("\nShutting down observers...")
        for observer in observers:
            observer.terminate()
        print("All observers terminated.")

if __name__ == "__main__":
    main()