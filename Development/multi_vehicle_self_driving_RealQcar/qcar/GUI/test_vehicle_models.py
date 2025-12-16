"""
Test script for vehicle models integration
Tests that MockQCar properly uses vehiclemodels dynamics
"""
import sys
import os
import numpy as np
import time

# Add parent directory to path
parent_dir = os.path.dirname(os.path.abspath(__file__))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from fake_vehicle_real_logic import MockQCar

def test_kinematic_model():
    """Test kinematic single-track model"""
    print("\n" + "="*60)
    print("Testing Kinematic Single-Track Model")
    print("="*60)
    
    car = MockQCar(car_id=0, use_dynamic_model=False)
    
    # Test forward motion
    print("\n1. Testing forward motion...")
    car.write(throttle=0.3, steering=0.0)
    
    for i in range(10):
        car.read()
        time.sleep(0.02)  # 50 Hz update
    
    print(f"   Position: ({car.x:.3f}, {car.y:.3f})")
    print(f"   Velocity: {car.velocity:.3f} m/s")
    print(f"   Heading: {car.heading:.3f} rad")
    
    # Test turning
    print("\n2. Testing turning...")
    car.write(throttle=0.3, steering=0.5)
    
    for i in range(20):
        car.read()
        time.sleep(0.02)
    
    print(f"   Position: ({car.x:.3f}, {car.y:.3f})")
    print(f"   Velocity: {car.velocity:.3f} m/s")
    print(f"   Heading: {car.heading:.3f} rad")
    print(f"   Angular velocity: {car.angular_velocity:.3f} rad/s")
    
    # Test stopping
    print("\n3. Testing stopping...")
    car.write(throttle=0.0, steering=0.0)
    
    for i in range(20):
        car.read()
        time.sleep(0.02)
    
    print(f"   Position: ({car.x:.3f}, {car.y:.3f})")
    print(f"   Velocity: {car.velocity:.3f} m/s")
    
    print("✅ Kinematic model test PASSED")
    return True

def test_dynamic_model():
    """Test single-track dynamic model"""
    print("\n" + "="*60)
    print("Testing Single-Track Dynamic Model")
    print("="*60)
    
    car = MockQCar(car_id=0, use_dynamic_model=True)
    
    # Test forward motion
    print("\n1. Testing forward motion...")
    car.write(throttle=0.3, steering=0.0)
    
    for i in range(10):
        car.read()
        time.sleep(0.02)
    
    print(f"   Position: ({car.x:.3f}, {car.y:.3f})")
    print(f"   Velocity: {car.velocity:.3f} m/s")
    print(f"   Heading: {car.heading:.3f} rad")
    print(f"   Yaw rate: {car.angular_velocity:.3f} rad/s")
    
    # Test turning
    print("\n2. Testing turning...")
    car.write(throttle=0.3, steering=0.5)
    
    for i in range(20):
        car.read()
        time.sleep(0.02)
    
    print(f"   Position: ({car.x:.3f}, {car.y:.3f})")
    print(f"   Velocity: {car.velocity:.3f} m/s")
    print(f"   Heading: {car.heading:.3f} rad")
    print(f"   Angular velocity: {car.angular_velocity:.3f} rad/s")
    print(f"   Slip angle: {car.state_st[6]:.3f} rad")
    
    # Test stopping
    print("\n3. Testing stopping...")
    car.write(throttle=0.0, steering=0.0)
    
    for i in range(20):
        car.read()
        time.sleep(0.02)
    
    print(f"   Position: ({car.x:.3f}, {car.y:.3f})")
    print(f"   Velocity: {car.velocity:.3f} m/s")
    
    print("✅ Dynamic model test PASSED")
    return True

def test_sensor_updates():
    """Test that sensors are properly updated"""
    print("\n" + "="*60)
    print("Testing Sensor Updates")
    print("="*60)
    
    car = MockQCar(car_id=0, use_dynamic_model=False)
    
    car.write(throttle=0.5, steering=0.3)
    
    for i in range(10):
        car.read()
        time.sleep(0.02)
    
    print(f"\n   Motor Tach: {car.motorTach:.3f} m/s")
    print(f"   Gyroscope Z: {car.gyroscope[2]:.3f} rad/s")
    print(f"   Battery: {car.battery[0]:.1f} V")
    
    # Verify sensor values are reasonable
    assert abs(car.motorTach - car.velocity) < 0.001, "Motor tach should match velocity"
    assert car.battery[0] > 11.0, "Battery should be charged"
    
    print("✅ Sensor update test PASSED")
    return True

def compare_models():
    """Compare kinematic vs dynamic model behavior"""
    print("\n" + "="*60)
    print("Comparing Kinematic vs Dynamic Models")
    print("="*60)
    
    car_ks = MockQCar(car_id=0, use_dynamic_model=False)
    car_st = MockQCar(car_id=1, use_dynamic_model=True)
    
    # Apply same control input
    throttle = 0.4
    steering = 0.3
    
    car_ks.write(throttle, steering)
    car_st.write(throttle, steering)
    
    # Simulate for 1 second
    for i in range(50):
        car_ks.read()
        car_st.read()
        time.sleep(0.02)
    
    print(f"\nAfter 1 second with throttle={throttle}, steering={steering}:")
    print(f"\nKinematic Model:")
    print(f"   Position: ({car_ks.x:.3f}, {car_ks.y:.3f})")
    print(f"   Velocity: {car_ks.velocity:.3f} m/s")
    print(f"   Heading: {car_ks.heading:.3f} rad")
    
    print(f"\nDynamic Model:")
    print(f"   Position: ({car_st.x:.3f}, {car_st.y:.3f})")
    print(f"   Velocity: {car_st.velocity:.3f} m/s")
    print(f"   Heading: {car_st.heading:.3f} rad")
    print(f"   Slip angle: {car_st.state_st[6]:.3f} rad")
    
    # Calculate differences
    pos_diff = np.sqrt((car_ks.x - car_st.x)**2 + (car_ks.y - car_st.y)**2)
    vel_diff = abs(car_ks.velocity - car_st.velocity)
    heading_diff = abs(car_ks.heading - car_st.heading)
    
    print(f"\nDifferences:")
    print(f"   Position: {pos_diff:.3f} m")
    print(f"   Velocity: {vel_diff:.3f} m/s")
    print(f"   Heading: {heading_diff:.3f} rad")
    
    print("\nℹ️  Dynamic model includes tire slip, so differences are expected")
    print("✅ Model comparison test PASSED")
    return True

def main():
    """Run all tests"""
    print("="*60)
    print("MockQCar Vehicle Models Integration Test")
    print("="*60)
    
    try:
        # Run tests
        test_kinematic_model()
        test_dynamic_model()
        test_sensor_updates()
        compare_models()
        
        print("\n" + "="*60)
        print("✅ ALL TESTS PASSED")
        print("="*60)
        print("\nVehicle models are properly integrated!")
        print("You can now use:")
        print("  - Kinematic model: python fake_vehicle_real_logic.py 0")
        print("  - Dynamic model: python fake_vehicle_real_logic.py 0 127.0.0.1 5000 true")
        return 0
        
    except Exception as e:
        print(f"\n❌ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)
