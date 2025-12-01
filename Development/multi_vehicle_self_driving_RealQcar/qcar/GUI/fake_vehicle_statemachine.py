"""
StateMachine-based Fake Vehicle 
Uses the real vehicle StateMachine system for proper command handling
and state transitions, making it behave exactly like a real vehicle.

Usage:
    python fake_vehicle_statemachine.py [car_id] [host_ip] [base_port]
    
Examples:
    python fake_vehicle_statemachine.py                    # Car 0 connecting to localhost:5000
    python fake_vehicle_statemachine.py 1                 # Car 1 connecting to localhost:5001  
    python fake_vehicle_statemachine.py 0 192.168.1.100   # Car 0 connecting to remote IP
"""
import sys
import os
import time
import socket
import json
import threading
import math
import random
from typing import Dict, Any, Optional
from threading import Event

# Add parent directory to path to import qcar modules
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Import real vehicle modules
from command_handler import CommandHandler
from logging_utils import VehicleLogger
from StateMachine.vehicle_state_machine import VehicleStateMachine
from StateMachine.vehicle_state import VehicleState, StateTransitionReason
from StateMachine.waiting_for_start_state import WaitingForStartState
from StateMachine.following_path_state import FollowingPathState
from StateMachine.following_leader_state import FollowingLeaderState
from StateMachine.stopped_state import StoppedState
from fake_initializing_state import FakeInitializingState
from config import VehicleControlConfig

class MockSpeedController:
    """Mock speed controller with minimal interface for StateMachine compatibility"""
    def __init__(self):
        self.ei = 0  # Integral error term
    
    def update(self, velocity, v_ref, dt):
        """Simple mock speed control - just return a throttle value"""
        error = v_ref - velocity
        throttle = max(-1.0, min(1.0, error * 0.5))  # Simple P controller
        return throttle

class MockSteeringController:
    """Mock steering controller with minimal interface for StateMachine compatibility"""
    def __init__(self):
        self.wp = [[0], [0]]  # Mock waypoint data [x, y]
        self._waypoint_index = 0
        self._errors = {'cross_track_error': 0.0, 'heading_error': 0.0}
    
    def update(self, position, theta, velocity):
        """Simple mock steering control"""
        # Simple mock steering output
        return 0.0  # No steering for fake vehicle
    
    def get_waypoint_index(self):
        """Return current waypoint index"""
        return self._waypoint_index
    
    def get_errors(self):
        """Return tracking errors"""
        return self._errors

class MockGPS:
    """Mock GPS with minimal interface for StateMachine compatibility"""
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.position = [x, y, 0.0]  # [x, y, z]
        self.orientation = [0.0, 0.0, theta]  # [roll, pitch, yaw]
    
    def readGPS(self):
        """Mock GPS read - always return success"""
        return True
    
    def update_position(self, x, y, theta):
        """Update GPS position (for simulation)"""
        self.position[0] = x
        self.position[1] = y
        self.orientation[2] = theta

class MockRoadmap:
    """Mock roadmap with minimal interface"""
    def __init__(self):
        self.nodes = [10, 4, 6, 8, 1]  # Simple node list
        
class MockQCar:
    """Mock QCar hardware interface"""
    def __init__(self):
        self.throttle = 0.0
        self.steering = 0.0
    
    def set_throttle(self, throttle):
        self.throttle = throttle
    
    def set_steering(self, steering):
        self.steering = steering

class FakeVehicleStateMachine(VehicleStateMachine):
    """Custom StateMachine for fake vehicle with simplified initialization"""
    
    def __init__(self, vehicle_logic, logger=None):
        # Initialize parent but override the state handlers
        super().__init__(vehicle_logic, logger)
        
        # Replace the real initialization state with fake one
        self.state_handlers[VehicleState.INITIALIZING] = FakeInitializingState(vehicle_logic)
        
        # Make sure the fake initialization state enters properly
        # Since the parent constructor already called enter() on the original handler,
        # we need to call enter() on our replacement handler
        if self.state == VehicleState.INITIALIZING:
            print(f"DEBUG StateMachine: Calling enter() on fake initialization state")
            try:
                success = self.state_handlers[self.state].enter()
                print(f"DEBUG StateMachine: Fake state enter() returned: {success}")
            except Exception as e:
                print(f"DEBUG StateMachine: Error calling enter() on fake state: {e}")
                import traceback
                traceback.print_exc()

class FakeVehicleLogic:
    def __init__(self, car_id: int, host_ip: str, base_port: int):
        self.car_id = car_id
        self.host_ip = host_ip
        self.port = base_port + car_id
        
        # Create logger
        self.logger = VehicleLogger(
            car_id=car_id,
            log_dir=f"fake_logs/car_{car_id}", 
            log_level="INFO"
        )
        
        # Load real config
        self.config = self._load_real_config()
        
        # Create kill event
        self.kill_event = Event()
        
        # Create command handler
        self.command_handler = CommandHandler(
            logger=self.logger,
            config=self.config
        )
        
        # Vehicle state for physics simulation
        self.x = car_id * 2.0  # Start positions spread out
        self.y = 0.0
        self.heading = 0.0
        self.velocity = 0.0
        self.target_velocity = 0.0
        self.throttle = 0.0
        self.steering = 0.0
        
        # Path and platoon settings  
        self.platoon_enabled = False
        self.platoon_role = "none"
        self.platoon_leader_id = None
        self.following_distance = 2.0
        self.path_nodes = [10, 4, 6, 8, 1]
        self.current_node = 0
        
        # Create mock hardware components with proper interfaces
        self.qcar = MockQCar()
        self.gps = MockGPS(x=self.x, y=self.y, theta=self.heading)
        self.roadmap = MockRoadmap()
        self.waypoint_sequence = [[self.x, self.y]]  # Simple waypoint sequence
        self.node_sequence = [10, 4, 6, 8, 1]
        self.speed_controller = MockSpeedController()
        self.steering_controller = MockSteeringController()
        
        print(f"[MOCK] Created mock hardware components for car {self.car_id}")
        print(f"   - MockSpeedController with ei attribute")
        print(f"   - MockSteeringController with waypoint interface")
        print(f"   - MockGPS at position ({self.x:.1f}, {self.y:.1f})")
        print(f"   - MockQCar hardware interface")
        print(f"   - MockRoadmap with nodes {self.node_sequence}")
        
        # Network
        self.socket = None
        self.connected = False
        self.running = False
        
        # Threading
        self.telemetry_thread = None
        self.command_thread = None
        self.physics_thread = None
        self.state_thread = None
        
        # Statistics
        self.telemetry_sent = 0
        self.commands_received = 0
        self.start_time = time.time()
        
        print(f"[*] StateMachine Fake Car {self.car_id} initialized")
        print(f"   Target: {self.host_ip}:{self.port}")
        print(f"   Initial position: ({self.x:.1f}, {self.y:.1f})")
        print(f"   Using full StateMachine + CommandHandler architecture")
    
    def _load_real_config(self):
        """Load real configuration from config_example.yaml"""
        # Find config file in the parent qcar directory
        qcar_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(qcar_dir, 'config_example.yaml')
        
        if os.path.exists(config_path):
            print(f"[CONFIG] Loading configuration from: {config_path}")
            config = VehicleControlConfig.from_yaml(config_path)
            
            # Override some settings for fake vehicle
            config.network.car_id = self.car_id  # Set correct car ID
            config.network.host_ip = self.host_ip  # Set correct host IP
            config.network.base_port = 5000  # Use base port
            config.logging.data_log_dir = f"fake_data_logs/car_{self.car_id}"  # Separate log dir
            config.steering.enable_steering_control = False  # Simplified physics for fake vehicle
            
            print(f"[CONFIG] Real config loaded and customized for fake vehicle {self.car_id}")
            return config
        else:
            print(f"[CONFIG] Config file not found at {config_path}, using default config")
            config = VehicleControlConfig()
            
            # Set basic overrides for fake vehicle
            config.network.car_id = self.car_id
            config.network.host_ip = self.host_ip
            config.network.base_port = 5000
            config.logging.data_log_dir = f"fake_data_logs/car_{self.car_id}"
            config.steering.enable_steering_control = False
            
            return config
    
    def connect_to_ground_station(self) -> bool:
        """Connect to the Ground Station using basic socket"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host_ip, self.port))
            self.connected = True
            print(f"[+] Car {self.car_id} connected to Ground Station at {self.host_ip}:{self.port}")
            return True
        except ConnectionRefusedError:
            print(f"[-] Car {self.car_id}: Connection refused. Is Ground Station running on {self.host_ip}:{self.port}?")
            return False
        except Exception as e:
            print(f"[-] Car {self.car_id}: Connection failed - {e}")
            return False
    
    def start_simulation(self):
        """Start the vehicle simulation with StateMachine"""
        if not self.connected:
            print(f"[-] Car {self.car_id}: Not connected to Ground Station")
            return
        
        self.running = True
        
        # Create custom StateMachine with fake initialization
        self.state_machine = FakeVehicleStateMachine(vehicle_logic=self, logger=self.logger)
        
        # Start telemetry sender thread
        self.telemetry_thread = threading.Thread(target=self._telemetry_worker, daemon=True)
        self.telemetry_thread.start()
        
        # Start command receiver thread
        self.command_thread = threading.Thread(target=self._command_worker, daemon=True)
        self.command_thread.start()
        
        # Start physics simulation
        self.physics_thread = threading.Thread(target=self._physics_worker, daemon=True)
        self.physics_thread.start()
        
        # Start state machine update thread
        self.state_thread = threading.Thread(target=self._state_machine_worker, daemon=True)
        self.state_thread.start()
        
        print(f"[*] Car {self.car_id}: StateMachine simulation started")
    
    def _telemetry_worker(self):
        """Send telemetry data to Ground Station"""
        print(f"[T] Car {self.car_id}: Telemetry thread started")
        
        while self.running and self.connected:
            try:
                # Get current state from StateMachine
                current_state = self.state_machine.state if hasattr(self, 'state_machine') else VehicleState.INITIALIZING
                
                # Create telemetry packet (same format as real vehicle)
                telemetry = {
                    'timestamp': time.time(),
                    'car_id': self.car_id,
                    'x': round(self.x, 3),
                    'y': round(self.y, 3),
                    'theta': round(self.heading, 3),
                    'v': round(self.velocity, 3),
                    'v_ref': round(self.target_velocity, 3),
                    'throttle': round(self.throttle, 3),
                    'steering': round(self.steering, 3),
                    'vehicle_state': current_state.name,  # Send StateMachine state
                    'is_running': current_state in [VehicleState.FOLLOWING_PATH, VehicleState.FOLLOWING_LEADER],
                    'path_nodes': self.path_nodes,
                    'current_node': self.current_node,
                    'platoon_enabled': self.platoon_enabled,
                    'platoon_role': self.platoon_role,
                    'platoon_leader_id': self.platoon_leader_id,
                    'following_distance': self.following_distance,
                    'state_machine_active': hasattr(self, 'state_machine')
                }
                
                # Send telemetry 
                message = json.dumps(telemetry) + '\n'
                self.socket.sendall(message.encode('utf-8'))
                self.telemetry_sent += 1
                
                if self.telemetry_sent % 50 == 0:  # Log every 50 sends
                    print(f"[S] Car {self.car_id}: Sent {self.telemetry_sent} telemetry packets - State: {current_state.name}")
                
                time.sleep(0.1)  # 10 Hz telemetry rate
                
            except Exception as e:
                print(f"[-] Car {self.car_id}: Telemetry error - {e}")
                self.connected = False
                break
        
        print(f"[T] Car {self.car_id}: Telemetry thread stopped")
    
    def _command_worker(self):
        """Receive and process commands using CommandHandler"""
        print(f"[C] Car {self.car_id}: Command thread started (with CommandHandler + StateMachine)")
        buffer = ""
        
        while self.running and self.connected:
            try:
                # Set socket timeout for graceful handling
                self.socket.settimeout(1.0)
                data = self.socket.recv(1024).decode('utf-8')
                
                if not data:
                    print(f"[!] Car {self.car_id}: Connection closed by Ground Station")
                    self.connected = False
                    break
                
                buffer += data
                
                # Process complete messages (newline-delimited JSON)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            command = json.loads(line)
                            print(f"[R] Car {self.car_id}: Received command: {command}")
                            
                            # Process using CommandHandler 
                            current_state = self.state_machine.state if hasattr(self, 'state_machine') else VehicleState.INITIALIZING
                            print(f"[?] Car {self.car_id}: Current StateMachine state: {current_state.name}")
                            
                            success = self.command_handler.process_command(command)
                            
                            if success:
                                self.commands_received += 1
                                print(f"[+] Car {self.car_id}: Command processed by CommandHandler")
                                # Note: StateMachine will pick up the command flags automatically
                            else:
                                print(f"[-] Car {self.car_id}: Command processing failed")
                                
                        except json.JSONDecodeError as e:
                            print(f"[-] Car {self.car_id}: Invalid JSON received: {e}")
                
            except socket.timeout:
                # Timeout is normal - just continue
                continue
            except Exception as e:
                print(f"[-] Car {self.car_id}: Command receive error - {e}")
                self.connected = False
                break
        
        print(f"[C] Car {self.car_id}: Command thread stopped")
    
    def _state_machine_worker(self):
        """Run the StateMachine update loop"""
        print(f"StateM Car {self.car_id}: StateMachine thread started")
        last_time = time.time()
        
        while self.running and hasattr(self, 'state_machine'):
            try:
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time
                
                if dt > 0.1:  # Skip large time steps
                    dt = 0.02  # Use smaller fixed timestep
                
                # Create sensor data for StateMachine
                sensor_data = {
                    'x': self.x,
                    'y': self.y,
                    'theta': self.heading,
                    'velocity': self.velocity,
                    'yolo_data': {}  # Empty YOLO data for fake vehicle
                }
                
                # Update StateMachine - this processes command flags
                current_state_before = self.state_machine.state
                throttle_cmd, steering_cmd = self.state_machine.update(dt, sensor_data)
                current_state_after = self.state_machine.state
                
                # Log state transitions for debugging
                if current_state_before != current_state_after:
                    print(f"StateM Car {self.car_id}: State transition {current_state_before.name} -> {current_state_after.name}")
                
                # Apply StateMachine commands to physics
                if throttle_cmd != 0:
                    self.target_velocity = max(0.0, min(2.0, abs(throttle_cmd) * 2.0))
                
                time.sleep(0.02)  # 50 Hz StateMachine update
                
            except Exception as e:
                print(f"ERROR Car {self.car_id}: StateMachine error - {e}")
                import traceback
                traceback.print_exc()
                time.sleep(1.0)
        
        print(f"StateM Car {self.car_id}: StateMachine thread stopped")
    
    def _physics_worker(self):
        """Simulate vehicle physics"""
        print(f"[P] Car {self.car_id}: Physics simulation started")
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            if dt > 0.1:  # Skip large time steps
                dt = 0.1
            
            # Update vehicle physics
            self._update_physics(dt)
            
            # Sleep to maintain reasonable update rate
            time.sleep(0.05)  # 20 Hz physics
        
        print(f"[P] Car {self.car_id}: Physics simulation stopped")
    
    def _update_physics(self, dt: float):
        """Update vehicle physics simulation"""
        # Check if StateMachine says we should be moving
        current_state = self.state_machine.state if hasattr(self, 'state_machine') else VehicleState.INITIALIZING
        should_move = current_state in [VehicleState.FOLLOWING_PATH, VehicleState.FOLLOWING_LEADER]
        
        if not should_move:
            # Apply braking when not in moving state
            self.velocity *= 0.9  # Friction/braking
            if self.velocity < 0.01:
                self.velocity = 0.0
            return
        
        # Simple velocity control (approach target velocity)
        velocity_error = self.target_velocity - self.velocity
        velocity_gain = 2.0  # How quickly we reach target velocity
        
        self.velocity += velocity_gain * velocity_error * dt
        self.velocity = max(0.0, min(2.0, self.velocity))  # Clamp to realistic range
        
        # Update throttle (for display)
        self.throttle = self.target_velocity / 2.0  # Normalized throttle
        
        # Simple motion - drive in a circle for now
        if self.velocity > 0.1:
            # Add some movement pattern
            time_factor = time.time() - self.start_time
            
            # Drive in a figure-8 pattern
            radius = 5.0 + self.car_id * 2.0  # Different radius for each car
            angular_speed = self.velocity / radius
            
            self.x += self.velocity * math.cos(self.heading) * dt
            self.y += self.velocity * math.sin(self.heading) * dt
            
            # Gentle turning
            self.heading += angular_speed * 0.1 * math.sin(time_factor * 0.5) * dt
            
            # Update steering for display
            self.steering = 0.1 * math.sin(time_factor * 0.5)
            
            # Keep heading in reasonable range
            while self.heading > math.pi:
                self.heading -= 2 * math.pi
            while self.heading < -math.pi:
                self.heading += 2 * math.pi
            
            # Update mock GPS position
            if self.gps:
                self.gps.update_position(self.x, self.y, self.heading)
    
    def stop(self):
        """Stop the simulation"""
        print(f"[!] Car {self.car_id}: Stopping StateMachine simulation...")
        self.running = False
        
        # Set kill event
        self.kill_event.set()
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        
        self.connected = False
        
        # Print final stats
        uptime = time.time() - self.start_time
        print(f"\n[S] Car {self.car_id} StateMachine Final Stats:")
        print(f"   Uptime: {uptime:.1f}s")
        print(f"   Telemetry sent: {self.telemetry_sent}")
        print(f"   Commands received: {self.commands_received}")
        print(f"   Final position: ({self.x:.2f}, {self.y:.2f})")
        print(f"   Final velocity: {self.velocity:.2f} m/s")
        if hasattr(self, 'state_machine'):
            print(f"   Final state: {self.state_machine.state.name}")
        if self.command_handler:
            stats = self.command_handler.get_statistics()
            print(f"   Commands processed: {stats.get('commands_processed', 0)}")
            print(f"   Commands rejected: {stats.get('commands_rejected', 0)}")


def main():
    """Main entry point for StateMachine fake vehicle"""
    
    # Parse command line arguments
    car_id = 0
    host_ip = '127.0.0.1'
    base_port = 5000
    
    if len(sys.argv) > 1:
        car_id = int(sys.argv[1])
    if len(sys.argv) > 2:
        host_ip = sys.argv[2]
    if len(sys.argv) > 3:
        base_port = int(sys.argv[3])
    
    print("="*60)
    print("[*] StateMachine QCar Fake Vehicle Simulator")
    print("   Using FULL StateMachine + CommandHandler architecture")
    print("   Behaves exactly like real vehicles")
    print("="*60)
    print(f"Car ID: {car_id}")
    print(f"Ground Station: {host_ip}:{base_port + car_id}")
    print("")
    
    # Create StateMachine fake vehicle
    vehicle = FakeVehicleLogic(car_id, host_ip, base_port)
    
    # Connect to Ground Station
    if not vehicle.connect_to_ground_station():
        print("Failed to connect. Make sure Ground Station is running.")
        return
    
    # Start simulation
    vehicle.start_simulation()
    
    print("\n" + "="*60)
    print("[*] StateMachine Vehicle is now running")
    print("   Uses REAL StateMachine for state management")
    print("   Uses REAL CommandHandler for command processing")
    print("   Processes cmd_ flags exactly like real vehicles")
    print("   Vehicle will appear in the Ground Station GUI")
    print("   Commands will be processed through proper state transitions")
    print("   Press Ctrl+C to stop")
    print("="*60)
    
    try:
        # Keep running until interrupted
        while vehicle.running and vehicle.connected:
            time.sleep(1)
            
            # Show periodic status
            if vehicle.telemetry_sent > 0 and vehicle.telemetry_sent % 100 == 0:
                current_state = vehicle.state_machine.state.name if hasattr(vehicle, 'state_machine') else "Unknown"
                print(f"[S] Car {car_id}: {vehicle.telemetry_sent} telemetry sent, "
                      f"{vehicle.commands_received} commands processed, "
                      f"State: {current_state}")
    
    except KeyboardInterrupt:
        print(f"\n[!] Car {car_id}: Shutting down...")
    
    except Exception as e:
        print(f"[-] Car {car_id}: Error - {e}")
    
    finally:
        vehicle.stop()
        print(f"[*] Car {car_id}: StateMachine simulation ended")


if __name__ == '__main__':
    main()