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
    """Create an observer for a specific car"""
    observer = Observer()
    observer.numDisplays = car_id  # Set the counter to car_id
    observer.add_display(
        imageSize=[height, width, 3],
        name=f'QCar {car_id} YOLO',
        scalingFactor=1
    )
    print(f"Observer for QCar {car_id} created")
    return observer

def launch_observer(observer, car_id):
    """Launch observer in a separate thread"""
    print(f"Launching observer for QCar {car_id}...")
    try:
        observer.launch()
        print(f"Observer for QCar {car_id} started successfully")
    except Exception as e:
        print(f"Error in QCar {car_id} observer: {e}")
        import traceback
        traceback.print_exc()

def main():
    parser = argparse.ArgumentParser(description="Multi-QCar Observer")
    parser.add_argument('--cars', type=int, nargs='+', default=[0 , 1],
                        help='List of car IDs to observe (e.g., --cars 0 1 2)')
    parser.add_argument('--width', type=int, default=320, help='Image width')
    parser.add_argument('--height', type=int, default=200, help='Image height')
    
    args = parser.parse_args()
    
    print("="*60)
    print("Multi-QCar Observer System")
    print("="*60)
    print(f"Observing QCars: {args.cars}")
    print(f"Image size: {args.width}x{args.height}")
    print("="*60)
    
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
            daemon=True
        )
        thread.start()
        threads.append(thread)
        
        time.sleep(1)  # Stagger the launches
    
    print(f"\nAll {len(observers)} observers launched!")
    print("Press Ctrl+C to stop all observers...")
    
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