#!/usr/bin/env python3
"""
Monitor real fleet communication and performance data.
Run this while your main fleet simulation is running.
"""

import sys
import os
import time

# Add the current directory to path to import local modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def monitor_fleet_performance():
    """Monitor real-time performance data from running fleet system."""
    print("FLEET PERFORMANCE MONITOR")
    print("=" * 50)
    print("This will monitor your running fleet system for performance data.")
    print("Make sure your main fleet simulation is running!")
    print("Press Ctrl+C to stop monitoring.\n")
    
    try:
        from performance_monitor import perf_monitor
        
        last_count = 0
        start_time = time.time()
        
        while True:
            try:
                stats = perf_monitor.get_performance_stats()
                current_count = stats['processing']['count']
                
                # Check if new data is coming in
                if current_count > last_count:
                    print(f"[{time.strftime('%H:%M:%S')}] NEW DATA! Total: {current_count}, "
                          f"Avg: {stats['processing']['avg']:.1f}ms, "
                          f"Health: {stats.get('health', {}).get('status', 'UNKNOWN')}")
                    last_count = current_count
                elif time.time() - start_time > 10:  # Show status every 10 seconds
                    print(f"[{time.strftime('%H:%M:%S')}] Status: {current_count} total measurements, "
                          f"Health: {stats.get('health', {}).get('status', 'WAITING_FOR_DATA')}")
                    start_time = time.time()
                
                time.sleep(1)
                
            except KeyboardInterrupt:
                print("\nStopping monitor...")
                break
            except Exception as e:
                print(f"Error: {e}")
                time.sleep(1)
        
        # Final report
        print("\n" + "="*50)
        print("FINAL PERFORMANCE REPORT")
        print("="*50)
        perf_monitor.print_performance_report()
        
    except ImportError:
        print("Error: Could not import performance_monitor")
    except Exception as e:
        print(f"Error: {e}")

def check_communication_logs():
    """Check if vehicles are actually communicating by looking at log files."""
    print("\n=== CHECKING COMMUNICATION LOGS ===")
    
    log_files = [
        "fleet_operations.log",
        "logs/communication_vehicle_0.log",
        "logs/communication_vehicle_1.log"
    ]
    
    for log_file in log_files:
        if os.path.exists(log_file):
            print(f"Found log file: {log_file}")
            try:
                with open(log_file, 'r') as f:
                    lines = f.readlines()
                    if lines:
                        print(f"  Last 3 lines:")
                        for line in lines[-3:]:
                            print(f"    {line.strip()}")
                    else:
                        print(f"  File is empty")
            except Exception as e:
                print(f"  Error reading file: {e}")
        else:
            print(f"Log file not found: {log_file}")

def main():
    """Main monitoring function."""
    print("FLEET COMMUNICATION & PERFORMANCE MONITOR")
    print("=" * 60)
    
    # Check if performance monitoring is available
    try:
        from performance_monitor import perf_monitor
        stats = perf_monitor.get_performance_stats()
        print(f"Performance monitor status: Available")
        print(f"Current measurements: {stats['processing']['count']}")
        print(f"Current health: {stats.get('health', {}).get('status', 'UNKNOWN')}")
    except Exception as e:
        print(f"Performance monitor error: {e}")
        return
    
    # Check for communication logs
    check_communication_logs()
    
    print(f"\nInstructions:")
    print(f"1. Start your main fleet simulation (main.py or QcarFleet.py)")
    print(f"2. This monitor will show when performance data starts flowing")
    print(f"3. If no data appears, check that vehicles are communicating")
    
    # Start monitoring
    monitor_fleet_performance()

if __name__ == "__main__":
    main()
