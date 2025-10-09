#!/usr/bin/env python3
"""
Debug script to analyze StateQueue validation issues.
This script will help identify why states are being rejected.
"""

import time
import logging
import sys
import os

# Add the current directory to path to import local modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from StateQueue import StateQueue
from md_logging_config import get_individual_vehicle_logger

class MockGPSSync:
    """Mock GPS sync for testing."""
    
    def __init__(self, offset=0.0):
        self.offset = offset
        
    def get_synced_time(self):
        return time.time() + self.offset

def analyze_state_validation():
    """Analyze why states might be rejected by StateQueue."""
    
    print("=== StateQueue Validation Analysis ===\n")
    
    # Setup logging with DEBUG level
    logger = logging.getLogger("DebugStateQueue")
    logger.setLevel(logging.DEBUG)
    handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    
    # Create StateQueue with debug logging
    state_queue = StateQueue(
        max_queue_size=50,
        max_age_seconds=1.0,  # Default: states older than 1 second are rejected
        max_delay_threshold=0.5,  # Default: states with >0.5s delay are rejected
        logger=logger
    )
    
    # Create mock GPS sync objects for both vehicles
    gps_vehicle_0 = MockGPSSync(offset=0.0)    # Leader
    gps_vehicle_1 = MockGPSSync(offset=0.0)    # Follower
    
    current_time = time.time()
    
    print(f"Current system time: {current_time:.3f}")
    print(f"Vehicle 0 GPS time: {gps_vehicle_0.get_synced_time():.3f}")
    print(f"Vehicle 1 GPS time: {gps_vehicle_1.get_synced_time():.3f}")
    print()
    
    # Test scenarios that might cause rejection
    test_scenarios = [
        {
            "name": "Current timestamp (should accept)",
            "timestamp": current_time,
            "expected": True
        },
        {
            "name": "0.1s old timestamp (should accept)",
            "timestamp": current_time - 0.1,
            "expected": True
        },
        {
            "name": "0.5s old timestamp (boundary case)",
            "timestamp": current_time - 0.5,
            "expected": True
        },
        {
            "name": "0.6s old timestamp (should reject - exceeds delay threshold)",
            "timestamp": current_time - 0.6,
            "expected": False
        },
        {
            "name": "1.0s old timestamp (should reject - exceeds age limit)",
            "timestamp": current_time - 1.0,
            "expected": False
        },
        {
            "name": "1.5s old timestamp (should reject - exceeds age limit)",
            "timestamp": current_time - 1.5,
            "expected": False
        },
        {
            "name": "Future timestamp +0.05s (should accept - within tolerance)",
            "timestamp": current_time + 0.05,
            "expected": True
        },
        {
            "name": "Future timestamp +0.15s (should reject - beyond tolerance)",
            "timestamp": current_time + 0.15,
            "expected": False
        }
    ]
    
    print("Testing validation scenarios:")
    print("-" * 60)
    
    for i, scenario in enumerate(test_scenarios):
        # Create test state
        test_state = {
            'type': 'state',
            'seq': i,
            'id': 0,  # From vehicle 0
            'pos': [0.0, 0.0, 0.0],
            'rot': [0.0, 0.0, 0.0],
            'v': 1.0,
            'timestamp': scenario['timestamp'],
            'ack_port': 6002
        }
        
        print(f"\nScenario: {scenario['name']}")
        print(f"  Timestamp: {scenario['timestamp']:.3f}")
        print(f"  Age: {current_time - scenario['timestamp']:.3f}s")
        print(f"  Expected: {'ACCEPT' if scenario['expected'] else 'REJECT'}")
        
        # Test validation using vehicle 1's GPS sync (as receiver)
        result = state_queue.add_state(test_state, gps_vehicle_1)
        
        print(f"  Result: {'ACCEPTED' if result else 'REJECTED'}")
        
        if result != scenario['expected']:
            print(f"  ⚠️  UNEXPECTED RESULT! Expected {scenario['expected']}, got {result}")
        else:
            print(f"  ✅ Validation result as expected")
    
    # Show statistics
    print("\n" + "="*60)
    print("StateQueue Statistics:")
    stats = state_queue.get_queue_stats()
    for key, value in stats.items():
        print(f"  {key}: {value}")

def analyze_gps_sync_differences():
    """Analyze potential GPS synchronization differences between vehicles."""
    
    print("\n=== GPS Synchronization Analysis ===\n")
    
    # Simulate different GPS sync scenarios
    scenarios = [
        {"name": "Perfect sync", "offset_0": 0.0, "offset_1": 0.0},
        {"name": "Small offset", "offset_0": 0.0, "offset_1": 0.01},
        {"name": "Medium offset", "offset_0": 0.0, "offset_1": 0.1},
        {"name": "Large offset", "offset_0": 0.0, "offset_1": 0.5},
        {"name": "Very large offset", "offset_0": 0.0, "offset_1": 1.0},
    ]
    
    for scenario in scenarios:
        print(f"Scenario: {scenario['name']}")
        
        gps_0 = MockGPSSync(offset=scenario['offset_0'])
        gps_1 = MockGPSSync(offset=scenario['offset_1'])
        
        time_0 = gps_0.get_synced_time()
        time_1 = gps_1.get_synced_time()
        
        print(f"  Vehicle 0 GPS time: {time_0:.3f}")
        print(f"  Vehicle 1 GPS time: {time_1:.3f}")
        print(f"  Time difference: {time_1 - time_0:.3f}s")
        
        # Simulate vehicle 0 sending state to vehicle 1
        state_from_0 = {
            'timestamp': time_0,
            'id': 0,
            'seq': 1,
            'pos': [0, 0, 0],
            'rot': [0, 0, 0],
            'v': 1.0
        }
        
        # Calculate delay as vehicle 1 would see it
        delay = time_1 - time_0
        print(f"  Delay as seen by vehicle 1: {delay:.3f}s")
        
        # Check if it would be accepted (assuming default thresholds)
        max_delay = 0.5
        max_age = 1.0
        would_accept = abs(delay) <= max_delay and delay <= max_age and delay >= -0.1
        
        print(f"  Would be accepted: {'YES' if would_accept else 'NO'}")
        if not would_accept:
            if abs(delay) > max_delay:
                print(f"    Reason: Excessive delay ({delay:.3f}s > {max_delay}s)")
            elif delay > max_age:
                print(f"    Reason: Too old ({delay:.3f}s > {max_age}s)")
            elif delay < -0.1:
                print(f"    Reason: Too far in future ({delay:.3f}s < -0.1s)")
        print()

if __name__ == "__main__":
    analyze_state_validation()
    analyze_gps_sync_differences()
