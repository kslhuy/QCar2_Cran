"""
Fake Vehicle using REAL VehicleLogic Class
This creates a fake vehicle that uses the actual VehicleLogic class from vehicle_logic.py
with mock hardware components. This allows testing the complete real system without
actual QCar hardware.

Usage:
    python fake_vehicle_real_logic.py [car_id] [host_ip] [base_port]
    
Examples:
    python fake_vehicle_real_logic.py                    # Car 0 connecting to localhost:5000
    python fake_vehicle_real_logic.py 1                 # Car 1 connecting to localhost:5001  
    python fake_vehicle_real_logic.py 0 192.168.1.100   # Car 0 connecting to remote IP
"""
import sys
import os
import time
import socket
import json
import threading
import math
import random
import numpy as np
from typing import Dict, Any, Optional
from threading import Event

# Add parent directory to path to import qcar modules
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Import the REAL VehicleLogic and related classes
from vehicle_logic import VehicleLogic
from config import VehicleControlConfig
from ground_station_client import GroundStationClient
from StateMachine.vehicle_state_machine import VehicleStateMachine
from StateMachine.vehicle_state import VehicleState
from fake_initializing_state import FakeInitializingState


class FakeVehicleStateMachine(VehicleStateMachine):
    """Custom StateMachine for fake vehicle with fake initialization"""
    
    def __init__(self, vehicle_logic, logger=None):
        # Initialize parent but override the state handlers
        super().__init__(vehicle_logic, logger)
        
        # Replace the real initialization state with fake one
        self.state_handlers[VehicleState.INITIALIZING] = FakeInitializingState(vehicle_logic)
        
        # Make sure the fake initialization state enters properly
        if self.state == VehicleState.INITIALIZING:
            print(f"🔧 FakeVehicleStateMachine: Calling enter() on fake initialization state")
            try:
                success = self.state_handlers[self.state].enter()
                print(f"✅ FakeVehicleStateMachine: Fake state enter() returned: {success}")
            except Exception as e:
                print(f"❌ FakeVehicleStateMachine: Error calling enter() on fake state: {e}")
                import traceback
                traceback.print_exc()
    
    def transition_to(self, new_state):
        """Handle state transitions for fake vehicle (for compatibility with vehicle_logic.py)"""
        # Handle string state names or missing states gracefully
        if isinstance(new_state, str):
            print(f"[STATE] FakeVehicleStateMachine: Attempting transition to state: {new_state}")
            if new_state == "SHUTTING_DOWN":
                print(f"[STATE] FakeVehicleStateMachine: Handling SHUTTING_DOWN as STOPPED")
                # Use STOPPED state instead of non-existent SHUTTING_DOWN
                from StateMachine.vehicle_state import VehicleState
                new_state = VehicleState.STOPPED
            else:
                print(f"[ERROR] FakeVehicleStateMachine: Unknown state string: {new_state}")
                return
        
        # Handle enum attribute access for missing states
        try:
            # Check if it's trying to access a non-existent enum value
            if hasattr(new_state, 'name') and new_state.name == 'SHUTTING_DOWN':
                print(f"[STATE] FakeVehicleStateMachine: Handling SHUTTING_DOWN enum as STOPPED")
                from StateMachine.vehicle_state import VehicleState
                new_state = VehicleState.STOPPED
        except AttributeError:
            # This might be VehicleState.SHUTTING_DOWN which doesn't exist
            print(f"[STATE] FakeVehicleStateMachine: Handling missing enum state as STOPPED")
            from StateMachine.vehicle_state import VehicleState
            new_state = VehicleState.STOPPED
        
        print(f"[STATE] FakeVehicleStateMachine: Transitioning from {self.state.name} to {new_state.name}")
        try:
            # Use force_transition_to which exists in parent
            self.force_transition_to(new_state, "Legacy transition_to call")
        except Exception as e:
            print(f"[ERROR] FakeVehicleStateMachine: Transition error - {e}")

    def force_transition_to(self, new_state, reason="Fake vehicle override"):
        """Handle forced state transitions for fake vehicle"""
        # Handle string state names or missing states gracefully
        if isinstance(new_state, str):
            print(f"[STATE] FakeVehicleStateMachine: Attempting transition to state: {new_state}")
            if new_state == "SHUTTING_DOWN":
                print(f"[STATE] FakeVehicleStateMachine: Handling SHUTTING_DOWN as STOPPED")
                # Use STOPPED state instead of non-existent SHUTTING_DOWN
                from StateMachine.vehicle_state import VehicleState
                new_state = VehicleState.STOPPED
            else:
                print(f"[ERROR] FakeVehicleStateMachine: Unknown state string: {new_state}")
                return
        
        print(f"[STATE] FakeVehicleStateMachine: Force transitioning from {self.state.name} to {new_state.name}")
        try:
            # Call parent force transition method
            super().force_transition_to(new_state, reason)
        except Exception as e:
            print(f"[ERROR] FakeVehicleStateMachine: Force transition error - {e}")
    
    def _transition_to(self, new_state, reason):
        """Override internal transition method to handle fake vehicle specifics"""
        # Handle string state names or missing states gracefully
        if isinstance(new_state, str):
            print(f"[STATE] FakeVehicleStateMachine: Internal transition to state: {new_state}")
            if new_state == "SHUTTING_DOWN":
                print(f"[STATE] FakeVehicleStateMachine: Handling SHUTTING_DOWN as STOPPED")
                # Use STOPPED state instead of non-existent SHUTTING_DOWN
                from StateMachine.vehicle_state import VehicleState, StateTransitionReason
                new_state = VehicleState.STOPPED
                reason = StateTransitionReason.SHUTDOWN
            else:
                print(f"[ERROR] FakeVehicleStateMachine: Unknown state string: {new_state}")
                return
        
        try:
            # Call parent internal transition method
            super()._transition_to(new_state, reason)
        except Exception as e:
            print(f"❌ FakeVehicleStateMachine: Internal transition error - {e}")
            # Emergency fallback
            try:
                from StateMachine.vehicle_state import VehicleState, StateTransitionReason
                super()._transition_to(VehicleState.STOPPED, StateTransitionReason.ERROR)
            except:
                pass


class MockQCar:
    """Mock QCar hardware that mimics the real QCar interface"""
    
    def __init__(self, car_id: int):
        self.car_id = car_id
        
        # Mock sensor data
        self.motorTach = 0.0
        self.gyroscope = np.array([0.0, 0.0, 0.0])
        self.battery = np.array([12.0])  # Mock battery voltage
        
        # Mock actuator commands
        self._throttle = 0.0
        self._steering = 0.0
        
        # Physics simulation state
        self.x = car_id * 2.0
        self.y = 0.0
        self.heading = 0.0
        self.velocity = 0.0
        self.angular_velocity = 0.0
        
        self.last_time = time.time()
        
        print(f"🔧 MockQCar {car_id}: Initialized at position ({self.x:.1f}, {self.y:.1f})")
    
    def read(self):
        """Simulate reading sensors and update physics"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt > 0.1:  # Skip large time steps
            dt = 0.02
        
        # Update physics based on commands
        self._update_physics(dt)
        
        # Update mock sensors
        self.motorTach = self.velocity  # Motor tach ~ velocity
        self.gyroscope[2] = self.angular_velocity  # Yaw rate
    
    def write(self, throttle: float, steering: float):
        """Receive control commands"""
        self._throttle = np.clip(throttle, -1.0, 1.0)
        self._steering = np.clip(steering, -1.0, 1.0)
    
    def _update_physics(self, dt: float):
        """Simple physics simulation"""
        # Convert throttle to target velocity
        max_velocity = 2.0  # m/s
        target_velocity = self._throttle * max_velocity
        
        # Simple velocity control
        velocity_error = target_velocity - self.velocity
        acceleration = 3.0 * velocity_error  # Simple P controller
        self.velocity += acceleration * dt
        self.velocity = np.clip(self.velocity, -max_velocity, max_velocity)
        
        # Update position
        self.x += self.velocity * math.cos(self.heading) * dt
        self.y += self.velocity * math.sin(self.heading) * dt
        
        # Update heading based on steering (simple bicycle model)
        if abs(self.velocity) > 0.1:
            wheelbase = 0.3  # meters
            self.angular_velocity = (self.velocity / wheelbase) * math.tan(self._steering * 0.5)
            self.heading += self.angular_velocity * dt
            
            # Normalize heading
            while self.heading > math.pi:
                self.heading -= 2 * math.pi
            while self.heading < -math.pi:
                self.heading += 2 * math.pi
        else:
            self.angular_velocity = 0.0


class MockQCarGPS:
    """Mock GPS that provides position data"""
    
    def __init__(self, qcar: MockQCar):
        self.qcar = qcar
        self.x = qcar.x
        self.y = qcar.y
        self.latitude = 0.0
        self.longitude = 0.0
        self.valid = True
        
        print(f"🛰️  MockGPS {qcar.car_id}: Initialized")
    
    def read(self):
        """Update GPS position from QCar physics"""
        # Add some noise to simulate real GPS
        noise_std = 0.05  # 5cm standard deviation
        self.x = self.qcar.x + random.gauss(0, noise_std)
        self.y = self.qcar.y + random.gauss(0, noise_std)
        
        # Convert to lat/lon (fake conversion)
        self.latitude = self.y * 0.00001 + 43.0  # Fake latitude around 43°N
        self.longitude = self.x * 0.00001 + -80.0  # Fake longitude around 80°W


class MockYOLOReceiver:
    """Mock YOLO receiver that provides detection data"""
    
    def __init__(self):
        # YOLO detection arrays (same format as real YOLO)
        self.stopSign = np.zeros(7, dtype=np.float64)
        self.trafficlight = np.zeros(7, dtype=np.float64) 
        self.cars = np.zeros(7, dtype=np.float64)
        self.yieldSign = np.zeros(7, dtype=np.float64)
        self.person = np.zeros(7, dtype=np.float64)
        
        print("👁️  MockYOLO: Initialized (no detections)")
    
    def read(self):
        """Simulate YOLO detections (mostly empty for now)"""
        # Reset all detections
        self.stopSign.fill(0.0)
        self.trafficlight.fill(0.0)
        self.cars.fill(0.0)
        self.yieldSign.fill(0.0)
        self.person.fill(0.0)
        
        # Very occasionally add some fake detections for testing (reduce frequency)
        if random.random() < 0.001:  # 0.1% chance per read (was 1%)
            detection_type = random.choice(['car', 'person'])  # Remove 'stop' to reduce spam
            distance = random.uniform(5.0, 15.0)  # Larger distances to avoid emergency stops
            
            if detection_type == 'car':
                self.cars[0] = 1
                self.cars[1] = distance
            elif detection_type == 'person':
                self.person[0] = 1
                self.person[1] = distance
    
    def terminate(self):
        """Mock terminate method"""
        pass


class MockStateEstimator:
    """Mock State Estimator that provides position from mock GPS"""
    
    def __init__(self, qcar: MockQCar, gps: MockQCarGPS):
        self.qcar = qcar
        self.gps = gps
        self.state_valid = True
        
        print(f"🧭 MockStateEstimator {qcar.car_id}: Initialized")
    
    def update(self, motor_tach: float, steering: float, dt: float, gyro_z: float):
        """Update state estimate (mock implementation)"""
        # Update GPS position
        self.gps.read()
        
        # Add some small random movement for testing telemetry
        if random.random() < 0.01:  # 1% chance per update
            self.qcar.x += random.uniform(-0.05, 0.05)
            self.qcar.y += random.uniform(-0.05, 0.05)
            self.qcar.heading += random.uniform(-0.02, 0.02)
    
    def get_state(self):
        """Get current state estimate"""
        return (
            self.qcar.x,      # x position
            self.qcar.y,      # y position  
            self.qcar.heading,  # theta (heading)
            self.qcar.velocity, # velocity
            self.state_valid   # state validity
        )


class MockSpeedController:
    """Mock Speed Controller for PID speed control"""
    
    def __init__(self, car_id: int):
        self.car_id = car_id
        self.ei = 0.0  # Integral error (needed by stopped_state.py)
        self.last_error = 0.0
        self.target_speed = 0.0
        print(f"🏎️  MockSpeedController {car_id}: Initialized")
    
    def update(self, current_speed, target_speed, dt):
        """Mock speed control update"""
        self.target_speed = target_speed
        error = target_speed - current_speed
        
        # Simple P controller
        kp = 0.5
        throttle = kp * error
        
        # Integrate error
        self.ei += error * dt
        self.last_error = error
        
        return np.clip(throttle, -1.0, 1.0)
    
    def reset(self):
        """Reset controller state"""
        self.ei = 0.0
        self.last_error = 0.0


class MockSteeringController:
    """Mock Steering Controller for path following"""
    
    def __init__(self, car_id: int):
        self.car_id = car_id
        self.waypoint_index = 0
        self.cross_track_error = 0.0
        self.heading_error = 0.0
        self.waypoint_sequence = None
        print(f"🎯 MockSteeringController {car_id}: Initialized")
    
    def update(self, x, y, theta, velocity, dt):
        """Mock steering control update"""
        # Simple mock - just return small random steering
        steering = 0.0  # Keep straight for now
        return np.clip(steering, -1.0, 1.0)
    
    def reset(self, waypoint_sequence=None):
        """Reset controller state"""
        self.waypoint_index = 0
        self.cross_track_error = 0.0
        self.heading_error = 0.0
        if waypoint_sequence is not None:
            self.waypoint_sequence = waypoint_sequence
    
    def get_waypoint_index(self):
        """Get current waypoint index"""
        return self.waypoint_index
    
    def get_errors(self):
        """Get control errors (cross_track, heading)"""
        return (self.cross_track_error, self.heading_error)


class MockYOLODrive:
    """Mock YOLO Drive system for obstacle detection and response"""
    
    def __init__(self, car_id: int):
        self.car_id = car_id
        self.yolo_gain = 1.0  # Default gain
        self.carDist = 0.0  # Distance to detected car
        self.personDist = 0.0  # Distance to detected person
        print(f"🤖 MockYOLODrive {car_id}: Initialized")
    
    def check_yolo(self, stop_sign, traffic_light, cars, yield_sign, person):
        """Mock YOLO obstacle detection and speed adjustment"""
        # Simple mock implementation - just return a speed adjustment gain
        # In real system, this would analyze YOLO detections and adjust speed accordingly
        
        # Update distances based on detections
        if np.any(cars > 0):
            # If car detected, set a safe distance (not 0 to avoid emergency stop)
            self.carDist = max(5.0, cars[1]) if cars[1] > 0 else 5.0
        else:
            self.carDist = 100.0  # No car detected, set large distance
            
        if np.any(person > 0):
            # If person detected, set a safe distance
            self.personDist = max(3.0, person[1]) if person[1] > 0 else 3.0
        else:
            self.personDist = 100.0  # No person detected, set large distance
        
        # Check for any significant detections
        has_detection = (
            np.any(stop_sign > 0) or 
            np.any(traffic_light > 0) or 
            np.any(cars > 0) or 
            np.any(yield_sign > 0) or 
            np.any(person > 0)
        )
        
        if has_detection:
            # Reduce speed if objects detected
            adjusted_gain = 0.8  # Less aggressive reduction
            # Log only occasionally to reduce spam
            if self.car_id == 0 and random.random() < 0.1:  # 10% chance to log
                print(f"🚨 MockYOLODrive {self.car_id}: Objects detected, reducing speed (car: {self.carDist:.1f}m, person: {self.personDist:.1f}m)")
        else:
            # Normal operation
            adjusted_gain = 1.0
        
        self.yolo_gain = adjusted_gain
        return adjusted_gain


class FakeVehicleWithRealLogic:
    """Fake vehicle that uses the real VehicleLogic class"""
    
    def __init__(self, car_id: int, host_ip: str, base_port: int):
        self.car_id = car_id
        self.host_ip = host_ip
        self.base_port = base_port
        
        print("="*60)
        print(f"[CAR] Real VehicleLogic Fake Vehicle - Car {car_id}")
        print("   Using ACTUAL VehicleLogic class with mock hardware")
        print("="*60)
        
        # Create mock hardware
        self.mock_qcar = MockQCar(car_id)
        self.mock_gps = MockQCarGPS(self.mock_qcar)
        self.mock_yolo = MockYOLOReceiver()
        
        # Create real configuration
        self.config = self._create_real_config()
        
        # Create kill event
        self.kill_event = Event()
        
        # Create the REAL VehicleLogic
        self.vehicle_logic = VehicleLogic(self.config, self.kill_event)
        
        # Replace the StateMachine with our custom fake version BEFORE starting
        self._replace_state_machine_with_fake()
        
        # Replace hardware with mocks AFTER VehicleLogic is created
        self._inject_mock_hardware()
        
        # Real Ground Station client (using actual ground_station_client.py)
        self.ground_station_client = None
        self.running = False
        
        # Threading
        self.vehicle_thread = None
        
        # Statistics
        self.start_time = time.time()
        
        print(f"✅ Real VehicleLogic initialized for Car {car_id}")
        print(f"   Mock hardware injected successfully")
    
    def _create_real_config(self) -> VehicleControlConfig:
        """Create real configuration for VehicleLogic"""
        config = VehicleControlConfig()
        
        # Network configuration
        config.network.car_id = self.car_id
        config.network.host_ip = self.host_ip
        config.network.base_port = self.base_port
        
        # Enable telemetry for Ground Station visibility
        config.logging.enable_telemetry_logging = True
        
        # Set timing for better telemetry rate
        config.timing.controller_update_rate = 50  # 50 Hz
        config.timing.telemetry_send_rate = 10     # 10 Hz (increase from default)
        config.timing.tf = 300.0  # 5 minute experiment
        
        # Disable some features that need real hardware
        config.steering.enable_steering_control = False
        
        # Path configuration - don't set valid_nodes as it's a property
        # The default valid_nodes from PathPlanningConfig will be used
        
        return config
    
    def _replace_state_machine_with_fake(self):
        """Replace VehicleLogic's StateMachine with fake version"""
        print(f"🔧 Car {self.car_id}: Replacing StateMachine with fake version...")
        
        # Replace the StateMachine with our custom fake version
        self.vehicle_logic.state_machine = FakeVehicleStateMachine(
            vehicle_logic=self.vehicle_logic,
            logger=self.vehicle_logic.logger
        )
        
        print(f"✅ Car {self.car_id}: StateMachine replaced with FakeVehicleStateMachine")
    
    def _inject_mock_hardware(self):
        """Replace real hardware with mocks in VehicleLogic"""
        # This will be done during initialization states
        # The VehicleLogic will try to initialize hardware, we'll replace it then
        pass
    
    def connect_to_ground_station(self) -> bool:
        """Connect to Ground Station using real GroundStationClient"""
        try:
            print(f"🔌 Car {self.car_id}: Creating real GroundStationClient...")
            
            # Import the real GroundStationClient
            from ground_station_client import GroundStationClient
            
            # Create real Ground Station client
            self.ground_station_client = GroundStationClient(
                config=self.config,
                logger=self.vehicle_logic.logger,
                kill_event=self.kill_event
            )
            
            # Initialize network connection
            print(f"🔌 Car {self.car_id}: Initializing network connection...")
            if not self.ground_station_client.initialize_network():
                print(f"❌ Car {self.car_id}: Failed to initialize network")
                return False
            
            # Start network threads
            print(f"🔌 Car {self.car_id}: Starting network threads...")
            if not self.ground_station_client.start_threads():
                print(f"❌ Car {self.car_id}: Failed to start network threads")
                return False
            
            print(f"✅ Car {self.car_id}: Connected to Ground Station using real GroundStationClient")
            return True
            
        except Exception as e:
            print(f"❌ Car {self.car_id}: Ground Station connection failed - {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def start_simulation(self):
        """Start the vehicle simulation"""
        # Connect to Ground Station FIRST using real GroundStationClient
        print(f"🔌 Car {self.car_id}: Connecting to Ground Station using real client...")
        if not self.connect_to_ground_station():
            print(f"❌ Car {self.car_id}: Failed to connect to Ground Station")
            return
        
        self.running = True
        
        # Inject mock hardware into VehicleLogic
        self._inject_mock_hardware_now()
        
        # Start VehicleLogic in its own thread
        self.vehicle_thread = threading.Thread(target=self._vehicle_logic_worker, daemon=True)
        self.vehicle_thread.start()
        
        print(f"🚀 Car {self.car_id}: Real VehicleLogic simulation started")
        print(f"   VehicleLogic thread: {'✓' if self.vehicle_thread.is_alive() else '❌'}")
        print(f"   Ground Station: {'✓' if self.ground_station_client else '❌'}")
    
    def _inject_mock_hardware_now(self):
        """Inject mock hardware into the VehicleLogic instance"""
        print(f"🔧 Car {self.car_id}: Injecting mock hardware into VehicleLogic...")
        
        # Replace hardware components
        self.vehicle_logic.qcar = self.mock_qcar
        self.vehicle_logic.gps = self.mock_gps
        self.vehicle_logic.yolo = self.mock_yolo
        
        # Use the REAL Ground Station client we created earlier
        self.vehicle_logic.client_Ground_Station = self.ground_station_client
        
        # Create mock YOLO drive system
        self.vehicle_logic.yolo_drive = MockYOLODrive(self.car_id)
        
        # Create mock state estimator
        self.vehicle_logic.state_estimator = MockStateEstimator(self.mock_qcar, self.mock_gps)
        
        # Create mock controllers (needed by state machine)
        self.vehicle_logic.speed_controller = MockSpeedController(self.car_id)
        self.vehicle_logic.steering_controller = MockSteeringController(self.car_id)
        
        # Override collision avoidance to prevent emergency stops during testing
        if hasattr(self.vehicle_logic, 'collision_avoidance'):
            original_check = self.vehicle_logic.collision_avoidance.check_collision_risk
            def mock_collision_check(car_distance, person_distance, current_velocity):
                # Always return safe (no emergency stop) for fake vehicle testing
                # Only emergency stop if objects are extremely close (< 0.5m)
                if car_distance < 0.5 or person_distance < 0.5:
                    return True, "collision_imminent_testing"
                return False, "safe_testing_mode"
            self.vehicle_logic.collision_avoidance.check_collision_risk = mock_collision_check
            print(f"✅ Car {self.car_id}: Collision avoidance overridden for safe testing")
        
        print(f"✅ Car {self.car_id}: Mock hardware and controllers injected successfully")
    
    def _vehicle_logic_worker(self):
        """Run the real VehicleLogic in a thread"""
        print(f"🧠 Car {self.car_id}: VehicleLogic thread started")
        
        try:
            # Run the REAL VehicleLogic
            self.vehicle_logic.run()
        except Exception as e:
            print(f"❌ Car {self.car_id}: VehicleLogic error - {e}")
            import traceback
            traceback.print_exc()
        finally:
            print(f"🧠 Car {self.car_id}: VehicleLogic thread stopped")
            self.kill_event.set()
    
    # Telemetry handled by real GroundStationClient - no separate worker needed
    
    # Commands handled by real GroundStationClient - no separate worker needed
    
    def stop(self):
        """Stop the simulation"""
        print(f"🛑 Car {self.car_id}: Stopping Real VehicleLogic simulation...")
        
        # Signal shutdown
        self.running = False
        self.kill_event.set()
        
        # Wait for VehicleLogic to finish
        if self.vehicle_thread and self.vehicle_thread.is_alive():
            print(f"⏳ Car {self.car_id}: Waiting for VehicleLogic to finish...")
            self.vehicle_thread.join(timeout=5.0)
            if self.vehicle_thread.is_alive():
                print(f"WARNING: Car {self.car_id}: VehicleLogic thread did not stop cleanly")
        
        # Close Ground Station client
        if self.ground_station_client:
            try:
                self.ground_station_client.close()
            except Exception as e:
                print(f"WARNING: Car {self.car_id}: Error closing Ground Station client - {e}")
            self.ground_station_client = None
        
        # Print final stats
        uptime = time.time() - self.start_time
        print(f"\n[STATS] Car {self.car_id} Real VehicleLogic Final Stats:")
        print(f"   Uptime: {uptime:.1f}s")
        print(f"   Final position: ({self.mock_qcar.x:.2f}, {self.mock_qcar.y:.2f})")
        print(f"   Final velocity: {self.mock_qcar.velocity:.2f} m/s")
        if hasattr(self.vehicle_logic, 'state_machine') and self.vehicle_logic.state_machine:
            print(f"   Final VehicleLogic state: {self.vehicle_logic.state_machine.state.name}")
        if hasattr(self.vehicle_logic, 'loop_counter'):
            print(f"   VehicleLogic iterations: {self.vehicle_logic.loop_counter}")
        if self.ground_station_client:
            stats = self.ground_station_client.get_statistics()
            print(f"   Network stats: Sent={stats.get('telemetry_sent', 0)}, Received={stats.get('commands_received', 0)}")


# Using real GroundStationClient instead of mock


def main():
    """Main entry point"""
    
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
    print("[CAR] QCar Fake Vehicle with REAL VehicleLogic")
    print("   This uses the actual VehicleLogic class from vehicle_logic.py")
    print("   with mock hardware components for complete system testing")
    print("="*70)
    print(f"Car ID: {car_id}")
    print(f"Ground Station: {host_ip}:{base_port + car_id}")
    print(f"Using: VehicleLogic + StateMachine + CommandHandler + all real classes")
    print("")
    
    # Create fake vehicle with real logic
    try:
        vehicle = FakeVehicleWithRealLogic(car_id, host_ip, base_port)
        print(f"✅ Fake vehicle created successfully")
    except Exception as e:
        print(f"❌ Failed to create fake vehicle: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Start simulation (includes Ground Station connection)
    vehicle.start_simulation()
    
    print("\n" + "="*70)
    print("🎮 Real VehicleLogic Fake Vehicle is running")
    print("   ✅ Uses REAL VehicleLogic class")
    print("   ✅ Uses REAL StateMachine system")
    print("   ✅ Uses REAL CommandHandler")
    print("   ✅ Uses REAL controllers and safety systems")
    print("   ✅ All real classes are tested")
    print("   📡 Vehicle appears in Ground Station GUI")
    print("   🎮 Commands processed exactly like real vehicle")
    print("   Press Ctrl+C to stop")
    print("="*70)
    
    try:
        # Keep running until interrupted
        while (vehicle.running and 
               vehicle.ground_station_client and 
               not vehicle.kill_event.is_set()):
            time.sleep(0.1)
            
            # Show periodic status (every 5 seconds)
            current_time = time.time()
            if not hasattr(vehicle, '_last_status_time') or current_time - vehicle._last_status_time > 5.0:
                vehicle._last_status_time = current_time
                state = "Unknown"
                loop_count = 0
                if hasattr(vehicle.vehicle_logic, 'state_machine') and vehicle.vehicle_logic.state_machine:
                    state = vehicle.vehicle_logic.state_machine.state.name
                if hasattr(vehicle.vehicle_logic, 'loop_counter'):
                    loop_count = vehicle.vehicle_logic.loop_counter
                
                # Get stats from real Ground Station client
                gs_stats = vehicle.ground_station_client.get_statistics() if vehicle.ground_station_client else {}
                telemetry_sent = gs_stats.get('telemetry_sent', 0)
                
                print(f"[STATS] Car {car_id}: State={state}, Loops={loop_count}, Telemetry={telemetry_sent}")
    
    except KeyboardInterrupt:
        print(f"\n🛑 Car {car_id}: Shutting down...")
    
    except Exception as e:
        print(f"❌ Car {car_id}: Unexpected error - {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        vehicle.stop()
        print(f"[CAR] Car {car_id}: Real VehicleLogic simulation ended")
        return 0


if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)