"""
Fake Vehicle with Real GroundStationClient
Uses the actual GroundStationClient for proper network communication,
telemetry transmission, and command reception exactly like real vehicles.

Usage:
    python fake_vehicle_groundstation.py [car_id] [host_ip] [base_port]
    
Examples:
    python fake_vehicle_groundstation.py                    # Car 0 connecting to localhost:5000
    python fake_vehicle_groundstation.py 1                 # Car 1 connecting to localhost:5001  
    python fake_vehicle_groundstation.py 0 192.168.1.100   # Car 0 connecting to remote IP
"""

import os
import sys
import time
import socket
import json
import threading
import traceback
from typing import Dict, Any, Optional
from threading import Event

# Add parent directory to path to import qcar modules
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Import real vehicle modules
from ground_station_client import GroundStationClient
from command_handler import CommandHandler
from logging_utils import VehicleLogger
from config import VehicleControlConfig
from StateMachine.vehicle_state_machine import VehicleStateMachine
from StateMachine.vehicle_state import VehicleState, StateTransitionReason
from fake_initializing_state import FakeInitializingState


class FakeVehicleConfig:
    """Minimal config class that mimics VehicleControlConfig structure"""
    
    def __init__(self, car_id: int, host_ip: str, port: int):
        # Network configuration
        self.network = type('Network', (), {})()
        self.network.is_remote_enabled = True
        self.network.host_ip = host_ip
        self.network.port = port
        self.network.car_id = car_id
        self.network.connection_timeout = 5
        self.network.max_reconnect_attempts = 10
        self.network.reconnect_delay = 2.0
        
        # Logging configuration
        self.logging = type('Logging', (), {})()
        self.logging.log_dir = f"fake_logs/car_{car_id}"
        self.logging.log_level = "INFO"
        self.logging.enable_telemetry_logging = False
        self.logging.data_log_dir = "fake_data_logs"
        
        # Vehicle configuration
        self.steering = type('Steering', (), {})()
        self.steering.enable_steering_control = False
        
        self.timing = type('Timing', (), {})()
        self.timing.controller_update_rate = 50  # 50 Hz
        
        self.path = type('Path', (), {})()
        self.path.valid_nodes = [1, 2, 3, 4]
        
        self.yolo = type('YOLO', (), {})()
        self.yolo.pulse_length_multiplier = 3


class FakeVehicleStateMachine(VehicleStateMachine):
    """Custom StateMachine for fake vehicle with simplified initialization"""
    
    def __init__(self, vehicle_logic, logger=None):
        # Initialize parent but override the state handlers
        super().__init__(vehicle_logic, logger)
        
        # Replace the real initialization state with fake one
        self.state_handlers[VehicleState.INITIALIZING] = FakeInitializingState(vehicle_logic)
        
        # Make sure the fake initialization state enters properly
        if self.state == VehicleState.INITIALIZING:
            print(f"DEBUG StateMachine: Calling enter() on fake initialization state")
            try:
                success = self.state_handlers[self.state].enter()
                print(f"DEBUG StateMachine: Fake state enter() returned: {success}")
            except Exception as e:
                print(f"DEBUG StateMachine: Error calling enter() on fake state: {e}")
                import traceback
                traceback.print_exc()


class FakeVehicleWithGroundStation:
    """Fake vehicle that uses real GroundStationClient for network communication"""
    
    def __init__(self, car_id: int, host_ip: str, base_port: int):
        self.car_id = car_id
        self.host_ip = host_ip
        self.port = base_port + car_id
        
        print(f"🚗 Fake Vehicle {car_id} initializing with GroundStationClient")
        print(f"   Target Ground Station: {host_ip}:{self.port}")
        
        # Create fake config that matches real vehicle expectations
        self.config = FakeVehicleConfig(car_id, host_ip, self.port)
        
        # Create logger
        self.logger = VehicleLogger(
            car_id=car_id,
            log_dir=self.config.logging.log_dir,
            log_level=self.config.logging.log_level
        )
        
        # Create kill event
        self.kill_event = Event()
        
        # Create real GroundStationClient
        self.ground_station = GroundStationClient(
            config=self.config,
            logger=self.logger,
            kill_event=self.kill_event
        )
        
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
        self.path_nodes = [1, 2, 3, 4]
        self.current_node = 0
        
        # Fake hardware components (minimal interface for StateMachine)
        self.qcar = None
        self.gps = None
        self.roadmap = None
        self.waypoint_sequence = None
        self.node_sequence = [1, 2, 3, 4]
        self.speed_controller = None
        self.steering_controller = None
        
        # Simulation state
        self.running = False
        self.state_machine = None
        
        # Threading
        self.physics_thread = None
        self.state_thread = None
        self.command_thread = None
        self.telemetry_thread = None
        
        # Statistics
        self.telemetry_sent = 0
        self.commands_received = 0
        self.start_time = time.time()
        
        print(f"✅ Fake Vehicle {car_id} initialized with GroundStationClient")
    
    def connect_to_ground_station(self) -> bool:
        """Connect to Ground Station using real GroundStationClient"""
        try:
            print(f"🔌 Car {self.car_id}: Connecting to Ground Station using GroundStationClient...")
            
            # Initialize network connection
            if not self.ground_station.initialize_network():
                print(f"❌ Car {self.car_id}: Failed to initialize network connection")
                return False
            
            # Start GroundStationClient threads for telemetry and commands
            if not self.ground_station.start_threads():
                print(f"❌ Car {self.car_id}: Failed to start GroundStationClient threads")
                return False
            
            print(f"✅ Car {self.car_id}: Connected to Ground Station via GroundStationClient")
            print(f"   Network stats: {self.ground_station.get_statistics()}")
            
            return True
            
        except Exception as e:
            print(f"❌ Car {self.car_id}: GroundStationClient connection failed - {e}")
            traceback.print_exc()
            return False
    
    def start_simulation(self):
        """Start the vehicle simulation with StateMachine and GroundStationClient"""
        if not self.ground_station.is_connected():
            print(f"❌ Car {self.car_id}: Not connected to Ground Station")
            return
        
        self.running = True
        
        # Create StateMachine with fake initialization
        self.state_machine = FakeVehicleStateMachine(vehicle_logic=self, logger=self.logger)
        
        # Start physics simulation
        self.physics_thread = threading.Thread(target=self._physics_worker, daemon=True)
        self.physics_thread.start()
        
        # Start state machine update thread
        self.state_thread = threading.Thread(target=self._state_machine_worker, daemon=True)
        self.state_thread.start()
        
        # Start command processing thread (processes commands from GroundStationClient)
        self.command_thread = threading.Thread(target=self._command_processor, daemon=True)
        self.command_thread.start()
        
        # Start telemetry thread (sends telemetry via GroundStationClient)
        self.telemetry_thread = threading.Thread(target=self._telemetry_sender, daemon=True)
        self.telemetry_thread.start()
        
        print(f"🚀 Car {self.car_id}: Simulation started with GroundStationClient integration")
        print(f"   StateMachine state: {self.state_machine.state.name}")
        print(f"   GroundStation connected: {self.ground_station.is_connected()}")
    
    def _telemetry_sender(self):
        """Send telemetry data via GroundStationClient"""
        print(f"📡 Car {self.car_id}: Telemetry sender started (GroundStationClient)")
        
        while self.running and self.ground_station.is_connected():
            try:
                # Get current state from StateMachine
                current_state = self.state_machine.state if self.state_machine else VehicleState.INITIALIZING
                
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
                    'vehicle_state': current_state.name,
                    'is_running': current_state in [VehicleState.FOLLOWING_PATH, VehicleState.FOLLOWING_LEADER],
                    'path_nodes': self.path_nodes,
                    'current_node': self.current_node,
                    'platoon_enabled': self.platoon_enabled,
                    'platoon_role': self.platoon_role,
                    'platoon_leader_id': self.platoon_leader_id,
                    'following_distance': self.following_distance,
                    'state_machine_active': self.state_machine is not None,
                    'ground_station_client': True  # Flag to identify GroundStationClient usage
                }
                
                # Send telemetry via GroundStationClient
                if self.ground_station.queue_telemetry(telemetry):
                    self.telemetry_sent += 1
                    
                    if self.telemetry_sent % 100 == 0:  # Log every 100 sends
                        stats = self.ground_station.get_statistics()
                        print(f"📊 Car {self.car_id}: Sent {self.telemetry_sent} telemetry packets")
                        print(f"   State: {current_state.name}, GSC stats: {stats['telemetry_sent']} sent")
                
                time.sleep(0.1)  # 10 Hz telemetry rate
                
            except Exception as e:
                print(f"❌ Car {self.car_id}: Telemetry error - {e}")
                time.sleep(1.0)  # Backoff on error
        
        print(f"📡 Car {self.car_id}: Telemetry sender stopped")
    
    def _command_processor(self):
        """Process commands received via GroundStationClient"""
        print(f"🎮 Car {self.car_id}: Command processor started (GroundStationClient + CommandHandler)")
        
        while self.running and self.ground_station.is_connected():
            try:
                # Get commands from GroundStationClient
                commands = self.ground_station.get_latest_commands()
                
                if commands:
                    print(f"📨 Car {self.car_id}: Received commands via GroundStationClient: {commands}")
                    
                    # Get current state for command processing
                    current_state = self.state_machine.state if self.state_machine else VehicleState.INITIALIZING
                    print(f"🔍 Car {self.car_id}: Processing command in state: {current_state.name}")
                    
                    # Process using CommandHandler
                    success = self.command_handler.process_command(commands, current_state)
                    
                    if success:
                        self.commands_received += 1
                        print(f"✅ Car {self.car_id}: Command processed successfully")
                    else:
                        print(f"❌ Car {self.car_id}: Command processing failed")
                
                time.sleep(0.05)  # 20 Hz command processing rate
                
            except Exception as e:
                print(f"❌ Car {self.car_id}: Command processing error - {e}")
                time.sleep(1.0)  # Backoff on error
        
        print(f"🎮 Car {self.car_id}: Command processor stopped")
    
    def _state_machine_worker(self):
        """Run the StateMachine update loop"""
        print(f"🔄 Car {self.car_id}: StateMachine worker started")
        last_time = time.time()
        
        while self.running and self.state_machine:
            try:
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time
                
                if dt > 0.1:  # Skip large time steps
                    dt = 0.02
                
                # Create sensor data for StateMachine
                sensor_data = {
                    'x': self.x,
                    'y': self.y,
                    'theta': self.heading,
                    'velocity': self.velocity,
                    'yolo_data': {}  # Empty YOLO data for fake vehicle
                }
                
                # Update StateMachine - processes command flags
                current_state_before = self.state_machine.state
                throttle_cmd, steering_cmd = self.state_machine.update(dt, sensor_data)
                current_state_after = self.state_machine.state
                
                # Log state transitions
                if current_state_before != current_state_after:
                    print(f"🔄 Car {self.car_id}: State transition: {current_state_before.name} -> {current_state_after.name}")
                
                # Apply StateMachine commands to physics
                if throttle_cmd != 0:
                    self.target_velocity = throttle_cmd * 2.0  # Scale throttle to velocity
                
                time.sleep(0.02)  # 50 Hz StateMachine update
                
            except Exception as e:
                print(f"❌ Car {self.car_id}: StateMachine error - {e}")
                traceback.print_exc()
                time.sleep(0.1)
        
        print(f"🔄 Car {self.car_id}: StateMachine worker stopped")
    
    def _physics_worker(self):
        """Simulate vehicle physics"""
        print(f"⚙️  Car {self.car_id}: Physics simulation started")
        last_time = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time
                
                if dt > 0.1:  # Skip large time steps
                    dt = 0.02
                
                # Update vehicle physics
                self._update_physics(dt)
                
                time.sleep(0.02)  # 50 Hz physics update
                
            except Exception as e:
                print(f"❌ Car {self.car_id}: Physics error - {e}")
                time.sleep(0.1)
        
        print(f"⚙️  Car {self.car_id}: Physics simulation stopped")
    
    def _update_physics(self, dt: float):
        """Update vehicle physics simulation"""
        # Check if StateMachine says we should be moving
        current_state = self.state_machine.state if self.state_machine else VehicleState.INITIALIZING
        should_move = current_state in [VehicleState.FOLLOWING_PATH, VehicleState.FOLLOWING_LEADER]
        
        if not should_move:
            # Vehicle should be stopped - reduce velocity
            self.target_velocity = 0.0
            self.velocity *= 0.9  # Gradual stop
            return
        
        # Simple velocity control (approach target velocity)
        velocity_error = self.target_velocity - self.velocity
        velocity_gain = 2.0  # How quickly we reach target velocity
        
        self.velocity += velocity_gain * velocity_error * dt
        self.velocity = max(0.0, min(2.0, self.velocity))  # Clamp to realistic range
        
        # Update throttle (for display)
        self.throttle = self.target_velocity / 2.0  # Normalized throttle
        
        # Simple motion - drive in a circle for demonstration
        if self.velocity > 0.1:
            self.x += self.velocity * dt * 0.8  # Move mostly forward
            self.y += self.velocity * dt * 0.2  # Slight curve
            self.heading += dt * 0.1  # Slow turn
    
    def stop(self):
        """Stop the simulation and cleanup"""
        print(f"🛑 Car {self.car_id}: Stopping simulation...")
        self.running = False
        
        # Set kill event
        self.kill_event.set()
        
        # Stop GroundStationClient
        if self.ground_station:
            self.ground_station.stop_threads()
            self.ground_station.close()
        
        # Print final stats
        uptime = time.time() - self.start_time
        print(f"\n📊 Car {self.car_id} GroundStationClient Final Stats:")
        print(f"   Uptime: {uptime:.1f}s")
        print(f"   Telemetry sent: {self.telemetry_sent}")
        print(f"   Commands received: {self.commands_received}")
        print(f"   Final position: ({self.x:.2f}, {self.y:.2f})")
        print(f"   Final velocity: {self.velocity:.2f} m/s")
        
        if self.state_machine:
            print(f"   Final state: {self.state_machine.state.name}")
        
        if self.ground_station:
            gs_stats = self.ground_station.get_statistics()
            print(f"   GroundStationClient stats: {gs_stats}")


def main():
    """Main entry point for GroundStationClient fake vehicle"""
    
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
    
    print("="*70)
    print("🚗 QCar Fake Vehicle with GroundStationClient")
    print("   Uses REAL GroundStationClient for network communication")
    print("   Uses REAL StateMachine + CommandHandler architecture")
    print("   Identical behavior to real vehicles")
    print("="*70)
    print(f"Car ID: {car_id}")
    print(f"Ground Station: {host_ip}:{base_port + car_id}")
    print("")
    
    # Create fake vehicle with GroundStationClient
    vehicle = FakeVehicleWithGroundStation(car_id, host_ip, base_port)
    
    # Connect to Ground Station
    if not vehicle.connect_to_ground_station():
        print("❌ Failed to connect via GroundStationClient. Make sure Ground Station is running.")
        return
    
    # Start simulation
    vehicle.start_simulation()
    
    print("\n" + "="*70)
    print("🎮 GroundStationClient Fake Vehicle is now running")
    print("   ✅ Uses REAL GroundStationClient for network communication")
    print("   ✅ Uses REAL StateMachine for state management")  
    print("   ✅ Uses REAL CommandHandler for command processing")
    print("   ✅ Telemetry sent via GroundStationClient.queue_telemetry()")
    print("   ✅ Commands received via GroundStationClient.get_latest_commands()")
    print("   ✅ No port conflicts - proper network handling")
    print("   Vehicle will appear in the Ground Station GUI")
    print("   Press Ctrl+C to stop")
    print("="*70)
    
    try:
        # Keep running until interrupted
        while vehicle.running and vehicle.ground_station.is_connected():
            time.sleep(1.0)
            
            # Periodically show status
            if int(time.time()) % 30 == 0:  # Every 30 seconds
                stats = vehicle.ground_station.get_statistics()
                print(f"🔄 Car {car_id}: Running - State: {vehicle.state_machine.state.name if vehicle.state_machine else 'None'}")
                print(f"   GroundStationClient: Connected={stats['connected']}, Telemetry={stats['telemetry_sent']}, Commands={stats['commands_received']}")
    
    except KeyboardInterrupt:
        print(f"\n🛑 Car {car_id}: Shutting down...")
    
    except Exception as e:
        print(f"❌ Car {car_id}: Error - {e}")
        traceback.print_exc()
    
    finally:
        vehicle.stop()
        print(f"🚗 Car {car_id}: GroundStationClient simulation ended")


if __name__ == '__main__':
    main()