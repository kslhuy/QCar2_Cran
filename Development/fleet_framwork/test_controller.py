#!/usr/bin/env python3
"""
Quick test to see if VehicleFollowerController can be initialized.
"""

import sys
import os

# Add current directory to path
sys.path.insert(0, os.path.dirname(__file__))

# Test 1: Check if controller_config.yaml exists
config_file = os.path.join(os.path.dirname(__file__), 'controller_config.yaml')
print(f"✓ Checking for controller_config.yaml at: {config_file}")
print(f"  File exists: {os.path.exists(config_file)}")

if os.path.exists(config_file):
    print(f"  File size: {os.path.getsize(config_file)} bytes")

# Test 2: Try to import VehicleFollowerController
print("\n✓ Attempting to import VehicleFollowerController...")
try:
    from VehicleFollowerController import VehicleFollowerController
    print("  ✅ Import successful!")
except Exception as e:
    print(f"  ❌ Import failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test 3: Try to instantiate a follower controller
print("\n✓ Attempting to instantiate VehicleFollowerController...")
try:
    controller = VehicleFollowerController(
        vehicle_id=1,
        controller_type="CACC",
        config={'road_type': 'Studio', 'node_sequence': [10, 4, 20, 13, 10]},
        logger=None
    )
    print("  ✅ Instantiation successful!")
    print(f"  Vehicle ID: {controller.vehicle_id}")
    print(f"  Controller type: {controller.controller_type}")
    print(f"  Follower mode: {controller.follower_mode}")
    print(f"  Initialized: {controller.initialized}")
except Exception as e:
    print(f"  ❌ Instantiation failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n✅ All tests passed!")
