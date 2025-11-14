#!/usr/bin/env python3
"""
Test script to check state machine behavior
"""
import sys
import time
from unittest.mock import Mock

# Add the qcar directory to the path
sys.path.append('.')

try:
    from StateMachine import VehicleState, VehicleStateMachine
    from logging_utils import VehicleLogger
    print("✅ Successfully imported state machine components")
except ImportError as e:
    print(f"❌ Import error: {e}")
    sys.exit(1)

def test_state_machine():
    """Test the state machine initialization and basic functionality"""
    print("\n" + "="*60)
    print("TESTING STATE MACHINE")
    print("="*60)
    
    # Create mock vehicle logic
    mock_vehicle_logic = Mock()
    mock_vehicle_logic.config = Mock()
    mock_vehicle_logic.config.steering.enable_steering_control = True
    mock_vehicle_logic.config.path.calibration_pose = [0, 0, 0]
    mock_vehicle_logic.config.logging.enable_telemetry_logging = False
    mock_vehicle_logic.loop_counter = 0
    
    # Create logger
    logger = VehicleLogger(
        car_id=0,
        log_dir="./logs",
        log_level="INFO"
    )
    
    print(f"📝 Created logger for Car ID: 0")
    
    # Create state machine
    try:
        state_machine = VehicleStateMachine(mock_vehicle_logic, logger)
        print(f"✅ State machine created successfully")
        print(f"📍 Initial state: {state_machine.state.name}")
        
        # Get state info
        state_info = state_machine.get_current_state_info()
        print(f"📊 State details:")
        for key, value in state_info.items():
            print(f"   {key}: {value}")
        
        # Test state history
        history = state_machine.get_state_history(5)
        print(f"📜 State history ({len(history)} entries):")
        for state, timestamp in history:
            print(f"   {state.name} at {timestamp}")
        
        # Test state machine statistics
        stats = state_machine.get_state_statistics()
        print(f"📈 State statistics:")
        for key, value in stats.items():
            print(f"   {key}: {value}")
        
        # Test a few update cycles
        print(f"\n🔄 Testing state machine updates...")
        
        # Mock sensor data
        sensor_data = {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,
            'velocity': 0.0,
            'motor_tach': 0.0,
            'gyro_z': 0.0,
            'yolo_data': {
                'stop_sign': [0]*7,
                'traffic_light': [0]*7,
                'cars': [0]*7,
                'yield_sign': [0]*7,
                'person': [0]*7,
                'car_dist': 0.0,
                'person_dist': 0.0
            },
            'state_valid': True
        }
        
        # Run several update cycles
        for i in range(5):
            mock_vehicle_logic.loop_counter = i
            throttle, steering = state_machine.update(0.005, sensor_data)
            print(f"   Cycle {i}: State={state_machine.state.name}, Throttle={throttle:.3f}, Steering={steering:.3f}")
            time.sleep(0.1)  # Short delay to see progression
        
        print(f"✅ State machine test completed successfully")
        
    except Exception as e:
        print(f"❌ State machine creation failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        # Clean up logger
        logger.close()
    
    return True

if __name__ == "__main__":
    success = test_state_machine()
    if success:
        print("\n🎉 All tests passed!")
    else:
        print("\n💥 Tests failed!")
        sys.exit(1)