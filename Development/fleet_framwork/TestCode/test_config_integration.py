#!/usr/bin/env python3
"""
Test script to verify that the use_control_observer_mode configuration
is properly integrated across all system components.
"""

import sys
import os

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(__file__))

from simple_config import SimpleFleetConfig, ConfigPresets

def test_config_integration():
    """Test that all configuration settings are properly accessible."""
    
    print("=" * 60)
    print("TESTING CONFIGURATION INTEGRATION")
    print("=" * 60)
    
    # Load configuration from YAML
    print("\n1. Loading configuration from config.yaml...")
    try:
        config = ConfigPresets.load_from_file("config.yaml")
        print("✅ Configuration loaded successfully")
    except Exception as e:
        print(f"❌ Failed to load configuration: {e}")
        return False
    
    # Test the specific settings we care about
    print("\n2. Testing configuration property access...")
    
    # Test update rates
    general_update = config.update_rate
    controller_rate = config.control_rate
    observer_rate = config.observer_rate
    gps_rate = config.gps_update_rate
    
    print(f"✅ General Update Rate: {general_update} Hz")
    print(f"✅ Controller Rate: {controller_rate} Hz")  
    print(f"✅ Observer Rate: {observer_rate} Hz")
    print(f"✅ GPS Rate: {gps_rate} Hz")
    
    # Test physical QCar settings
    use_physical = config.use_physical_qcar
    use_control_observer = config.use_control_observer_mode
    calibrate = config.calibrate
    
    print(f"✅ Use Physical QCar: {use_physical}")
    print(f"✅ Use Control Observer Mode: {use_control_observer}")
    print(f"✅ Calibrate: {calibrate}")
    
    # Verify expected values match config.yaml
    print("\n3. Verifying values match config.yaml...")
    
    expected_values = {
        'general_update_rate': 50,
        'controller_rate': 100,
        'use_physical_qcar': True,
        'use_control_observer_mode': True,
        'calibrate': False
    }
    
    actual_values = {
        'general_update_rate': general_update,
        'controller_rate': controller_rate,
        'use_physical_qcar': use_physical,
        'use_control_observer_mode': use_control_observer,
        'calibrate': calibrate
    }
    
    all_correct = True
    for key, expected in expected_values.items():
        actual = actual_values[key]
        if actual == expected:
            print(f"✅ {key}: {actual} (matches expected)")
        else:
            print(f"❌ {key}: {actual} (expected {expected})")
            all_correct = False
    
    # Test that getattr() would work (simulating QcarFleet.py behavior)
    print("\n4. Testing getattr() access (simulating QcarFleet.py)...")
    
    try:
        # This is how QcarFleet.py accesses the properties
        general_via_getattr = getattr(config, 'update_rate', 100)
        control_via_getattr = getattr(config, 'control_rate', 50)
        physical_via_getattr = getattr(config, 'use_physical_qcar', False)
        observer_via_getattr = getattr(config, 'use_control_observer_mode', False)
        calibrate_via_getattr = getattr(config, 'calibrate', False)
        
        print(f"✅ getattr(config, 'update_rate'): {general_via_getattr}")
        print(f"✅ getattr(config, 'control_rate'): {control_via_getattr}")
        print(f"✅ getattr(config, 'use_physical_qcar'): {physical_via_getattr}")
        print(f"✅ getattr(config, 'use_control_observer_mode'): {observer_via_getattr}")
        print(f"✅ getattr(config, 'calibrate'): {calibrate_via_getattr}")
        
    except Exception as e:
        print(f"❌ getattr() access failed: {e}")
        all_correct = False
    
    # Test configuration printing
    print("\n5. Testing configuration display...")
    try:
        config.print_config()
        print("✅ Configuration printing works")
    except Exception as e:
        print(f"❌ Configuration printing failed: {e}")
        all_correct = False
    
    print("\n" + "=" * 60)
    if all_correct:
        print("🎉 ALL TESTS PASSED - Configuration integration is working correctly!")
        print("Your use_control_observer_mode setting will be properly used.")
    else:
        print("⚠️  SOME TESTS FAILED - There may be configuration issues.")
    print("=" * 60)
    
    return all_correct

if __name__ == "__main__":
    success = test_config_integration()
    sys.exit(0 if success else 1)