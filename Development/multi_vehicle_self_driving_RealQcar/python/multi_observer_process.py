#!/usr/bin/env python3
"""
Process-Based Multi-Observer System
Creates completely separate Python processes for each vehicle observer
"""

import subprocess
import time
import os
import signal
import sys
import argparse

def main():
    """Main entry point - spawns separate observer processes"""
    parser = argparse.ArgumentParser(description="Multi-Vehicle Observer System")
    parser.add_argument('--cars', type=int, nargs='+', default=[0, 1],
                        help='List of car IDs to observe')
    parser.add_argument('--width', type=int, default=320, help='Image width')
    parser.add_argument('--height', type=int, default=200, help='Image height')
    args = parser.parse_args()
    
    print("="*60)
    print(f" Multi-Vehicle Observer System (Process-Based)")
    print("="*60)
    print(f"  Vehicles: {args.cars}")
    print(f"  Image size: {args.width}x{args.height}")
    print("="*60)
    
    processes = []
    temp_scripts = []
    
    def cleanup():
        """Cleanup function"""
        print("\n[→] Stopping all observers...")
        for proc, car_id in processes:
            try:
                if proc.poll() is None:
                    proc.terminate()
                    proc.wait(timeout=5)
                    print(f"  [✓] Observer {car_id} stopped")
            except subprocess.TimeoutExpired:
                proc.kill()
                print(f"  [!] Observer {car_id} force killed")
            except Exception as e:
                print(f"  [✗] Error stopping observer {car_id}: {e}")
        
        # Remove temp scripts
        for script in temp_scripts:
            try:
                if os.path.exists(script):
                    os.remove(script)
            except:
                pass
        print("[✓] Cleanup complete")
    
    def signal_handler(signum, frame):
        cleanup()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        for car_id in args.cars:
            print(f"\n[→] Creating Observer for Vehicle {car_id}...")
            
            # Create standalone observer script
            script_content = f'''
from pal.utilities.probe import Observer
import sys

print("Starting Observer for Vehicle {car_id}")
print("Port: {18801 + car_id}")

observer = Observer()
observer.add_display(
    imageSize=[{args.height}, {args.width}, 3],
    scalingFactor=1,
    name="Vehicle {car_id} YOLO"
)

print("Observer {car_id} ready - waiting for connection...")
try:
    observer.launch()
except KeyboardInterrupt:
    print("Observer {car_id} interrupted")
    observer.terminate()
    sys.exit(0)
except Exception as e:
    print(f"Observer {car_id} error: {{e}}")
    sys.exit(1)
'''
            
            script_name = f"temp_observer_{car_id}.py"
            with open(script_name, 'w') as f:
                f.write(script_content)
            temp_scripts.append(script_name)
            
            # Start process
            print(f"[→] Starting Observer {car_id} process...")
            proc = subprocess.Popen([sys.executable, script_name])
            processes.append((proc, car_id))
            print(f"[✓] Observer {car_id} started (PID: {proc.pid})")
            
            time.sleep(2)  # Give time to initialize
        
        print(f"\n[✓] All {len(processes)} observers started!")
        print("\nExpected OpenCV windows:")
        for car_id in args.cars:
            print(f"  • 'Vehicle {car_id} YOLO' (Port: {18801 + car_id})")
        
        print(f"\nPress Ctrl+C to stop all observers...")
        
        # Wait for processes or interrupt
        while True:
            alive_count = 0
            for proc, car_id in processes:
                if proc.poll() is None:
                    alive_count += 1
                else:
                    print(f"\n[!] Observer {car_id} ended")
            
            if alive_count == 0:
                print("\n[!] All observers ended")
                break
            
            time.sleep(1)
    
    except Exception as e:
        print(f"\n[✗] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cleanup()

if __name__ == '__main__':
    main()