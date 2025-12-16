"""
Refactored Vehicle Control - Main Entry Point

This is a complete refactoring of the vehicle control system with:
- Configuration management
- State machine
- Enhanced logging and monitoring
- Robust error handling
- Safety systems
- Thread-safe components
"""

import sys
import argparse
import signal
import time
from threading import Event

from config_main import VehicleMainConfig
from vehicle_logic import VehicleLogic
from pal.products.qcar import  IS_PHYSICAL_QCAR



# Global kill event for clean shutdown
kill_event = Event()


def signal_handler(*args):
    """Handle Ctrl+C and other signals"""
    print("\n[SIGNAL] Shutdown signal received")
    kill_event.set()

def stop_quarc_models():
    """Stop QUARC models as a fallback"""
    try:
        import subprocess
        import socket
        
        print("\n[CLEANUP] Stopping QUARC models...")
        
        # Try to stop QUARC models using quarc_run command
        quarc_target = "tcpip://localhost:17000"
        cmd = ["quarc_run", "-q", "-Q", "-t", quarc_target, "*.rt-linux_qcar2"]
        
        try:
            result = subprocess.run(cmd, 
                                  capture_output=True, 
                                  text=True, 
                                  timeout=5)
            
            if result.returncode == 0:
                print("  [✓] QUARC models stopped successfully")
            else:
                print(f"  [⚠] QUARC stop returned code {result.returncode}")
                
        except subprocess.TimeoutExpired:
            print("  [⚠] QUARC stop command timed out")
        except FileNotFoundError:
            print("  [⚠] quarc_run not found - hardware may still be active")
        except Exception as e:
            print(f"  [⚠] Error running quarc_run: {e}")
            
        # Test if QUARC is still running
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex(("localhost", 17000))
            sock.close()
            
            if result == 0:
                print("  [⚠] QUARC service still accessible - hardware may still be active")
            else:
                print("  [✓] QUARC service stopped - hardware should be inactive")
                
        except Exception as e:
            print(f"  [i] Cannot test QUARC connection: {e}")
            
    except Exception as e:
        print(f"  [✗] Error stopping QUARC models: {e}")


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        prog='Vehicle Control (Refactored)',
        description='QCar autonomous vehicle control system'
    )
    
    parser.add_argument(
        '-c', '--calibrate',
        action='store_true',
        default=False,
        help='Recalibrate vehicle before starting'
    )
    
    parser.add_argument(
        '-n', '--path_number',
        type=int,
        default=0,
        choices=[0, 1, 2],
        help='Node configuration (0, 1, or 2 for different traffic patterns)'
    )
    # Auto compute port based on car_id (just keep base port here)
    parser.add_argument(
        '--host',
        type=str,
        default="127.0.0.1",
        help='Host PC IP address for remote control (optional)'
    )
    
    parser.add_argument(
        '--port',
        type=int,
        default=5000,
        help='Base port number (default: 5000)'
    )
    
    parser.add_argument(
        '--car-id',
        type=int,
        default=0,
        help='Car ID (0, 1, 2, ...) (default: 0)'
    )
    
    parser.add_argument(
        '--config',
        type=str,
        default=None,
        help='Path to configuration file (JSON or YAML). Default: config_vehicle_main.yaml in script directory'
    )
    
    parser.add_argument(
        '--v-ref',
        type=float,
        default=None,
        help='Initial velocity reference in m/s (overrides config file)'
    )
    
    parser.add_argument(
        '--left-hand-traffic',
        action='store_true',
        default=False,
        help='Enable left-hand traffic mode'
    )
    
    parser.add_argument(
        '--no-steering',
        action='store_true',
        default=False,
        help='Disable steering control'
    )
    
    parser.add_argument(
        '--vehicle-type',
        type=str,
        default='Qcar',
        choices=['Qcar', 'Limo'],
        help='Vehicle type: Qcar or Limo (default: Qcar)'
    )
    
    return parser.parse_args()


def load_configuration(args) -> VehicleMainConfig:
    """Load configuration from file or defaults"""
    
    # Determine config file path
    import os
    
    if args.config:
        config_path = args.config
    else:
        # Use config_vehicle_main.yaml from same directory as this script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, 'config_vehicle_main.yaml')
    
    # Load from file if it exists
    if os.path.exists(config_path):
        print(f"Loading configuration from: {config_path}")
        if config_path.endswith('.json'):
            config = VehicleMainConfig.from_json(config_path)
        elif config_path.endswith('.yaml') or config_path.endswith('.yml'):
            config = VehicleMainConfig.from_yaml(config_path)
        else:
            print("Error: Config file must be .json or .yaml")
            sys.exit(1)
    else:
        if args.config:
            print(f"Error: Config file not found: {config_path}")
            sys.exit(1)
        else:
            print(f"Warning: config_vehicle_main.yaml not found at {config_path}")
            print("Using default configuration values")
            config = VehicleMainConfig()
    
    # Update from command line arguments
    config.update_from_args(args)
    

    return config





def main():
    """Main entry point"""
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    print("="*70)
    print(" QCar Autonomous Vehicle Control System (Refactored)")
    print("="*70)
    
    # Parse arguments
    args = parse_arguments()
    
    # Load configuration
    print("\n[INIT] Loading configuration...")
    config = load_configuration(args)
    
    Control_rate = config.timing.controller_update_rate if IS_PHYSICAL_QCAR else 100
    print("\n[CONFIG] Vehicle Configuration:")
    print(f"  Car ID: {config.network.car_id}")
    print(f"  Vehicle Type: {config.vehicle.vehicle_type}")
    print(f"  Controller rate: {Control_rate} Hz")
    print(f"  Remote control: {'Enabled' if config.network.is_remote_enabled else 'Disabled'}")
    
    if config.network.is_remote_enabled:
        print(f"    Host: {config.network.host_ip}:{config.network.port}")
    
    
    # Create vehicle logic controller
    print("\n[INIT] Creating vehicle controller...")
    vehicle_logic = VehicleLogic(config, kill_event)
    
    # Override v_ref if specified in command line
    if args.v_ref is not None:
        vehicle_logic.v_ref = args.v_ref
        print(f"[CONFIG] Velocity reference overridden to: {args.v_ref} m/s")
    
    print("\n[READY] Vehicle controller created and ready")
    print("="*70)
    print("Starting control loop... (Press Ctrl+C to stop)")
    print("="*70)
    print()
    
    # Start control loop directly
    try:
        vehicle_logic.run()
    except KeyboardInterrupt:
        print("\n[INTERRUPT] Keyboard interrupt received")
        kill_event.set()
    except Exception as e:
        print(f"\n[ERROR] Control loop exception: {e}")
        import traceback
        traceback.print_exc()
        kill_event.set()
    
    # Ensure QUARC models are stopped even if vehicle_logic shutdown fails
    try:
        if IS_PHYSICAL_QCAR:
            # stop_quarc_models()
            pass
        else:
            # CamLidarFusion thread cleanup removed (moved to main loop)
            from qvl.real_time import QLabsRealTime

            cmd = QLabsRealTime().terminate_all_real_time_models()
            time.sleep(1)
    except Exception as e:
        print(f"[WARNING] Error during QUARC cleanup: {e}")
    
    print("\n" + "="*70)
    print(" Shutdown complete")
    print("="*70)
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
