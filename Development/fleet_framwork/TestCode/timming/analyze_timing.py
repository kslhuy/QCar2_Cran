#!/usr/bin/env python3
"""
Analyze actual log timestamps to understand the delay issue.
"""

import re
from datetime import datetime

def parse_log_timestamp(timestamp_str):
    """Parse log timestamp string to datetime object."""
    try:
        return datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S,%f")
    except:
        return None

def analyze_communication_timing():
    """Analyze the timing in actual log files."""
    
    print("=== Communication Timing Analysis ===\n")
    
    # Read recent communication logs
    try:
        with open("logs/communication_vehicle_0.log", "r") as f:
            v0_lines = f.readlines()[-10:]  # Last 10 lines
            
        with open("logs/communication_vehicle_1.log", "r") as f:
            v1_lines = f.readlines()[-10:]  # Last 10 lines
            
        print("Vehicle 0 (Leader) - Last few messages:")
        for line in v0_lines:
            if "SENT:" in line and "Seq:" in line:
                print(f"  {line.strip()}")
                
        print("\nVehicle 1 (Follower) - Last few messages:")
        for line in v1_lines:
            if "RECEIVED STATE:" in line:
                print(f"  {line.strip()}")
                
        # Look for timing patterns
        print("\n=== Timing Pattern Analysis ===")
        
        # Extract send times from vehicle 0
        send_times = []
        for line in v0_lines:
            if "SENT:" in line and "Seq:" in line:
                # Extract timestamp
                timestamp_match = re.match(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3})', line)
                if timestamp_match:
                    dt = parse_log_timestamp(timestamp_match.group(1))
                    if dt:
                        send_times.append(dt)
                        
        # Extract receive times from vehicle 1
        receive_times = []
        for line in v1_lines:
            if "RECEIVED STATE:" in line:
                timestamp_match = re.match(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3})', line)
                if timestamp_match:
                    dt = parse_log_timestamp(timestamp_match.group(1))
                    if dt:
                        receive_times.append(dt)
                        
        print(f"Found {len(send_times)} send timestamps")
        print(f"Found {len(receive_times)} receive timestamps")
        
        if send_times and receive_times:
            # Calculate transmission delays
            min_pairs = min(len(send_times), len(receive_times))
            print(f"\nTransmission delays (last {min_pairs} messages):")
            
            for i in range(min_pairs):
                if i < len(send_times) and i < len(receive_times):
                    delay = (receive_times[i] - send_times[i]).total_seconds()
                    print(f"  Message {i+1}: {delay:.3f}s delay")
                    if delay > 0.5:
                        print(f"    ⚠️  This delay exceeds the 0.5s threshold!")
                        
    except FileNotFoundError as e:
        print(f"Log file not found: {e}")
        print("Make sure the fleet is running and generating logs.")

if __name__ == "__main__":
    analyze_communication_timing()
