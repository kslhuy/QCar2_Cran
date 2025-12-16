"""
Test script to verify controller configuration system

Run this to ensure your controller_config.yaml is working correctly.
"""
import sys
import os

# Add parent directory to path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

def test_config_loading():
    """Test that config file can be loaded"""
    print("=" * 60)
    print("Testing Controller Configuration System")
    print("=" * 60)
    
    try:
        from Controller.config_controller_loader import get_controller_config
        print("✓ Config loader imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import config_loader: {e}")
        return False
    
    try:
        config = get_controller_config()
        print(f"✓ Config loaded successfully")
        print(f"  Config file: {config.config_path}")
    except FileNotFoundError as e:
        print(f"✗ Config file not found: {e}")
        print("\nPlease copy controller_config_vehicle_main.yaml to controller_config.yaml")
        return False
    except Exception as e:
        print(f"✗ Failed to load config: {e}")
        return False
    
    # Test getting controller types
    try:
        long_type = config.get_longitudinal_controller_type()
        lat_type = config.get_lateral_controller_type()
        print(f"✓ Controller types loaded:")
        print(f"  Longitudinal: {long_type}")
        print(f"  Lateral: {lat_type}")
    except Exception as e:
        print(f"✗ Failed to get controller types: {e}")
        return False
    
    # Test getting longitudinal parameters
    try:
        long_params = config.get_longitudinal_params()
        print(f"✓ Longitudinal parameters loaded:")
        for key, value in long_params.items():
            print(f"  {key}: {value}")
    except Exception as e:
        print(f"✗ Failed to get longitudinal params: {e}")
        return False
    
    # Test getting lateral parameters
    try:
        lat_params = config.get_lateral_params()
        print(f"✓ Lateral parameters loaded:")
        for key, value in lat_params.items():
            if hasattr(value, '__class__'):
                print(f"  {key}: {value.__class__.__name__}")
            else:
                print(f"  {key}: {value}")
    except Exception as e:
        print(f"✗ Failed to get lateral params: {e}")
        return False
    
    # Test creating controllers
    print("\n" + "-" * 60)
    print("Testing Controller Creation")
    print("-" * 60)
    
    try:
        from Controller.longitudinal_controllers import ControllerFactory
        long_controller = ControllerFactory.create(
            long_type,
            long_params
        )
        print(f"✓ Created {long_type} longitudinal controller: {long_controller.__class__.__name__}")
    except Exception as e:
        print(f"✗ Failed to create longitudinal controller: {e}")
        return False
    
    try:
        from Controller.lateral_controllers import LateralControllerFactory
        if lat_type != 'hybrid':  # Hybrid params already contain controller instances
            lat_controller = LateralControllerFactory.create(
                lat_type,
                lat_params
            )
            print(f"✓ Created {lat_type} lateral controller: {lat_controller.__class__.__name__}")
        else:
            print(f"✓ Hybrid lateral controller configured (contains sub-controllers)")
    except Exception as e:
        print(f"✗ Failed to create lateral controller: {e}")
        return False
    
    print("\n" + "=" * 60)
    print("✓ ALL TESTS PASSED!")
    print("=" * 60)
    print("\nYour controller configuration system is working correctly.")
    print("You can now edit controller_config.yaml to change parameters.")
    return True


if __name__ == "__main__":
    success = test_config_loading()
    sys.exit(0 if success else 1)
