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
from threading import Event, Thread

from config import VehicleControlConfig
from vehicle_controller import VehicleController


# Global kill event for clean shutdown
kill_event = Event()
KILL_THREAD = False


def signal_handler(*args):
    """Handle Ctrl+C and other signals"""
    global KILL_THREAD
    print("\n[SIGNAL] Shutdown signal received")
    KILL_THREAD = True
    kill_event.set()


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
        '-n', '--node_configuration',
        type=int,
        default=0,
        choices=[0, 1],
        help='Node configuration (0 or 1 for different traffic patterns)'
    )
    
    parser.add_argument(
        '--host',
        type=str,
        default=None,
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
        help='Path to configuration file (JSON or YAML). Default: config_example.yaml in script directory'
    )
    
    parser.add_argument(
        '--v-ref',
        type=float,
        default=None,
        help='Reference velocity in m/s'
    )
    
    parser.add_argument(
        '--no-steering',
        action='store_true',
        default=False,
        help='Disable steering control'
    )
    
    return parser.parse_args()


def load_configuration(args) -> VehicleControlConfig:
    """Load configuration from file or defaults"""
    
    # Determine config file path
    import os
    
    if args.config:
        config_path = args.config
    else:
        # Use config_example.yaml from same directory as this script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, 'config_example.yaml')
    
    # Load from file if it exists
    if os.path.exists(config_path):
        print(f"Loading configuration from: {config_path}")
        if config_path.endswith('.json'):
            config = VehicleControlConfig.from_json(config_path)
        elif config_path.endswith('.yaml') or config_path.endswith('.yml'):
            config = VehicleControlConfig.from_yaml(config_path)
        else:
            print("Error: Config file must be .json or .yaml")
            sys.exit(1)
    else:
        if args.config:
            print(f"Error: Config file not found: {config_path}")
            sys.exit(1)
        else:
            print(f"Warning: config_example.yaml not found at {config_path}")
            print("Using default configuration values")
            config = VehicleControlConfig()
    
    # Update from command line arguments
    config.update_from_args(args)
    
    # Override with specific arguments
    if args.v_ref is not None:
        config.speed.v_ref = args.v_ref
    
    if args.no_steering:
        config.steering.enable_steering_control = False
    
    return config


def wait_for_yolo_server(timeout: float = 15.0):
    """Wait for YOLO server to start"""
    print("Waiting for YOLO server to start...")
    print(f"  (Timeout: {timeout}s)")
    
    start_time = time.time()
    dots = 0
    
    while time.time() - start_time < timeout:
        if kill_event.is_set():
            return False
        
        # Simple progress indicator
        dots = (dots + 1) % 4
        print(f"\r  Waiting{'.' * dots}{' ' * (3 - dots)}", end='', flush=True)
        time.sleep(0.5)
    
    print("\n  YOLO server ready (timeout reached, proceeding)")
    return True


def run_control_thread(controller: VehicleController):
    """Run the control loop in a thread"""
    try:
        controller.run()
    except Exception as e:
        print(f"\n[ERROR] Control thread exception: {e}")
        import traceback
        traceback.print_exc()
        kill_event.set()


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
    
    print(f"  Car ID: {config.network.car_id}")
    print(f"  Reference velocity: {config.speed.v_ref} m/s")
    print(f"  Controller rate: {config.timing.controller_update_rate} Hz")
    print(f"  Steering control: {'Enabled' if config.steering.enable_steering_control else 'Disabled'}")
    print(f"  Remote control: {'Enabled' if config.network.is_remote_enabled else 'Disabled'}")
    
    if config.network.is_remote_enabled:
        print(f"    Host: {config.network.host_ip}:{config.network.port}")
    
    # Wait for YOLO server
    if not wait_for_yolo_server():
        print("\n[SHUTDOWN] Cancelled by user")
        return 0
    
    # Create controller
    print("\n[INIT] Creating vehicle controller...")
    controller = VehicleController(config, kill_event)
    
    # Initialize systems
    print("[INIT] Initializing vehicle systems...")
    if not controller.initialize():
        print("\n[ERROR] Initialization failed!")
        return 1
    
    print("\n[READY] All systems initialized")
    print("="*70)
    print("Starting control loop... (Press Ctrl+C to stop)")
    print("="*70)
    print()
    
    # Start control thread
    control_thread = Thread(target=run_control_thread, args=(controller,), daemon=False)
    control_thread.start()
    
    # Wait for completion or interrupt
    try:
        while control_thread.is_alive() and not kill_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[INTERRUPT] Keyboard interrupt received")
        kill_event.set()
    
    # Wait for thread to finish
    print("\n[SHUTDOWN] Waiting for control thread to finish...")
    control_thread.join(timeout=5.0)
    
    if control_thread.is_alive():
        print("[WARNING] Control thread did not terminate cleanly")
    else:
        print("[SHUTDOWN] Control thread terminated cleanly")
    
    print("\n" + "="*70)
    print(" Shutdown complete")
    print("="*70)
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
