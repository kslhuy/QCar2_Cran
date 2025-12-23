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
        name=f'QCar {car_id} YOLO',
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
    
    print()
    print("="*70)
    print(f" All {len(observers)} observer window(s) launched successfully!")
    print("="*70)
    print("\nEach vehicle will send its YOLO detection stream to its own window.")
    print("Press Ctrl+C to stop all observers...")
    print()
    
    try:
        # Keep main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down observers...")
        for observer in observers:
            observer.terminate()
        print("All observers terminated.")

if __name__ == "__main__":
    main()