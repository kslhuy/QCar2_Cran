"""
Quick test script to verify config_example.yaml loads correctly
"""
import os
import sys
from config import VehicleControlConfig

def main():
    print("=" * 70)
    print("Testing Configuration Loading")
    print("=" * 70)
    
    # Get path to config_example.yaml
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, 'config_example.yaml')
    
    print(f"\nLooking for config at: {config_path}")
    print(f"File exists: {os.path.exists(config_path)}")
    
    if not os.path.exists(config_path):
        print("\n❌ ERROR: config_example.yaml not found!")
        return 1
    
    # Try to load config
    try:
        print("\nLoading configuration...")
        config = VehicleControlConfig.from_yaml(config_path)
        print("✅ Configuration loaded successfully!")
        
        print("\n" + "-" * 70)
        print("Configuration Summary:")
        print("-" * 70)
        print(f"Speed:")
        print(f"  v_ref: {config.speed.v_ref} m/s")
        print(f"  K_p: {config.speed.K_p}")
        print(f"  K_i: {config.speed.K_i}")
        
        print(f"\nSteering:")
        print(f"  K_stanley: {config.steering.K_stanley}")
        print(f"  Enabled: {config.steering.enable_steering_control}")
        
        print(f"\nNetwork:")
        print(f"  Host IP: {config.network.host_ip}")
        print(f"  Base Port: {config.network.base_port}")
        print(f"  Car ID: {config.network.car_id}")
        print(f"  Port (calculated): {config.network.port}")
        print(f"  Remote Enabled: {config.network.is_remote_enabled}")
        
        print(f"\nPath Planning:")
        print(f"  Node Configuration: {config.path.node_configuration}")
        print(f"  Calibration Pose: {config.path.calibration_pose}")
        print(f"  Valid Nodes: {config.path.valid_nodes}")
        
        print(f"\nTiming:")
        print(f"  Controller Rate: {config.timing.controller_update_rate} Hz")
        print(f"  Telemetry Rate: {config.timing.telemetry_send_rate} Hz")
        
        print(f"\nLogging:")
        print(f"  Log Directory: {config.logging.log_dir}")
        print(f"  Data Log Directory: {config.logging.data_log_dir}")
        print(f"  Telemetry Logging: {config.logging.enable_telemetry_logging}")
        
        print("-" * 70)
        print("\n✅ All configuration parameters loaded correctly!")
        return 0
        
    except Exception as e:
        print(f"\n❌ ERROR loading configuration: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    sys.exit(main())
