"""
Fake Vehicle using REAL VehicleLogic Class - SIMPLIFIED
This creates a fake vehicle that uses the actual VehicleLogic class from vehicle_logic.py
with mock hardware components. Only the INITIALIZING state is replaced with a fake version.

Key simplifications:
- No custom FakeVehicleStateMachine (uses real VehicleStateMachine)
- No redundant Ground Station client creation (VehicleLogic handles it)
- Only INITIALIZING state is fake, all other states are real
- Mock hardware is injected during the fake initialization

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


# Removed FakeVehicleStateMachine class - using real VehicleStateMachine instead
# We only need to replace the INITIALIZING state with fake version


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
    
    def readGPS(self):
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
        """Simulate YOLO detections - always empty for safe testing"""
        # Always reset all detections to zero (no objects detected)
        self.stopSign.fill(0.0)
        self.trafficlight.fill(0.0)
        self.cars.fill(0.0)
        self.yieldSign.fill(0.0)
        self.person.fill(0.0)
        
        # No fake detections to avoid triggering emergency stops
    
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
        self.gps.readGPS()
        
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
        self.carDist = 100.0  # Safe distance - no cars detected
        self.personDist = 100.0  # Safe distance - no persons detected
        print(f"🤖 MockYOLODrive {car_id}: Initialized (safe distances)")
    
    def check_yolo(self, stop_sign, traffic_light, cars, yield_sign, person):
        """Mock YOLO obstacle detection - always returns safe values for testing"""
        # For testing, always report safe distances to avoid emergency stops
        self.carDist = 100.0  # Always safe distance
        self.personDist = 100.0  # Always safe distance
        
        # Always return normal speed gain (no obstacles)
        self.yolo_gain = 1.0
        return 1.0


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
        
        # Set a reference so the fake initialization state can access our mock hardware
        self.vehicle_logic._parent_fake_vehicle = self
        
        # Replace ONLY the INITIALIZING state with fake version AFTER starting
        # All other states remain real for complete system testing
        
        # Replace hardware with mocks AFTER VehicleLogic is created
        self._inject_mock_hardware()
        
        # Initialize state for main loop
        self.running = True  # Start in running state
        self.ground_station_client = None  # Will be set by VehicleLogic
        
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
        
        # Set timing for better telemetry rate and V2V performance
        config.timing.controller_update_rate = 200  # 200 Hz (match vehicle_logic.py)
        config.timing.telemetry_send_rate = 20      # 20 Hz for V2V compatibility
        config.timing.tf = 300.0  # 5 minute experiment
        
        # Disable some features that need real hardware
        config.steering.enable_steering_control = True
        
        # Path configuration - don't set valid_nodes as it's a property
        # The default valid_nodes from PathPlanningConfig will be used
        
        return config
    
    def _replace_initialization_state_only(self):
        """Replace only the INITIALIZING state with fake version, keep everything else real"""
        try:
            # Wait for VehicleLogic to create the state machine
            import time
            start_time = time.time()
            while not hasattr(self.vehicle_logic, 'state_machine') and (time.time() - start_time) < 5.0:
                time.sleep(0.1)
            
            if not hasattr(self.vehicle_logic, 'state_machine'):
                print("🔧 State machine not found, VehicleLogic may not be fully initialized")
                return
            
            # Replace only the INITIALIZING state handler with our fake one
            from fake_initializing_state import FakeInitializingState
            fake_init_state = FakeInitializingState(self.vehicle_logic)
            self.vehicle_logic.state_machine.state_handlers[VehicleState.INITIALIZING] = fake_init_state
            
            print(f"✅ Car {self.car_id}: Replaced INITIALIZING state with fake version")
            print(f"          All other states remain real for complete system testing")
            
        except Exception as e:
            print(f"❌ Car {self.car_id}: Failed to replace initialization state: {e}")
    
    def _inject_mock_hardware(self):
        """Mock hardware injection will be done during state machine initialization"""
        # The fake initialization state will handle mock hardware injection
        print(f"🔧 Car {self.car_id}: Mock hardware injection deferred to initialization state")
    
    def start_simulation(self):
        """Start the fake vehicle simulation using real VehicleLogic"""
        print("\\n" + "="*60)
        print("[SIM] Starting Real VehicleLogic with Mock Hardware")
        print("      VehicleLogic handles ALL initialization including Ground Station")
        print("="*60)
        
        # Replace only the initialization state (keep everything else real)
        self._replace_initialization_state_only()
        
        # Start VehicleLogic in a separate thread
        import threading
        self.vehicle_thread = threading.Thread(target=self._vehicle_logic_worker, daemon=True)
        self.vehicle_thread.start()
        
        # Give it time to start
        time.sleep(1.0)
        
        # Verify thread is running
        if self.vehicle_thread.is_alive():
            print(f"✅ Car {self.car_id}: VehicleLogic thread started successfully")
        else:
            print(f"❌ Car {self.car_id}: VehicleLogic thread failed to start")
        
        print(f"✅ Car {self.car_id}: Real VehicleLogic started with fake initialization")
        print(f"          ✅ Ground Station connection: Handled by VehicleLogic")
        print(f"          ✅ State machine: Real (except INITIALIZING state)")
        print(f"          ✅ All controllers: Real")
        print(f"          ✅ Hardware: Mock (injected during initialization)")
    
    
    def _vehicle_logic_worker(self):
        """Run the real VehicleLogic in a thread"""
        print(f"🧠 Car {self.car_id}: VehicleLogic thread started")
        
        try:
            # Run the REAL VehicleLogic
            print(f"[THREAD] Car {self.car_id}: Starting VehicleLogic.run()...")
            self.vehicle_logic.run()
            print(f"[THREAD] Car {self.car_id}: VehicleLogic.run() completed normally")
        except Exception as e:
            print(f"❌ Car {self.car_id}: VehicleLogic error - {e}")
            import traceback
            traceback.print_exc()
        finally:
            print(f"🧠 Car {self.car_id}: VehicleLogic thread stopped")
            # Signal that we're shutting down
            self.running = False
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
    print("[CAR] QCar Fake Vehicle with REAL VehicleLogic - SIMPLIFIED")
    print("   Uses actual VehicleLogic + real StateMachine + real GroundStationClient")
    print("   Only INITIALIZING state is fake for quick mock hardware injection")
    print("="*70)
    print(f"Car ID: {car_id}")
    print(f"Ground Station: {host_ip}:{base_port + car_id}")
    print(f"Approach: Real VehicleLogic + Fake initialization + Mock hardware")
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
    print("🎮 Simplified Real VehicleLogic Fake Vehicle is running")
    print("   ✅ Uses REAL VehicleLogic class")
    print("   ✅ Uses REAL VehicleStateMachine (except INITIALIZING state)")
    print("   ✅ Uses REAL GroundStationClient (created by VehicleLogic)")
    print("   ✅ Uses REAL controllers and safety systems")
    print("   ✅ Mock hardware injected during fake initialization")
    print("   📡 Vehicle appears in Ground Station GUI")
    print("   🎮 Commands processed exactly like real vehicle")
    print("   Press Ctrl+C to stop")
    print("="*70)
    
    try:
        # Keep running until interrupted
        print(f"[MAIN] Car {car_id}: Entering main loop...")
        
        while vehicle.running and not vehicle.kill_event.is_set():
            time.sleep(0.1)
            
            # Check if VehicleLogic thread is still alive
            if vehicle.vehicle_thread and not vehicle.vehicle_thread.is_alive():
                print(f"[MAIN] Car {car_id}: VehicleLogic thread has stopped, exiting main loop")
                break
            
            # Show periodic status (every 10 seconds)
            current_time = time.time()
            if not hasattr(vehicle, '_last_status_time') or current_time - vehicle._last_status_time > 10.0:
                vehicle._last_status_time = current_time
                state = "Unknown"
                loop_count = 0
                if hasattr(vehicle.vehicle_logic, 'state_machine') and vehicle.vehicle_logic.state_machine:
                    state = vehicle.vehicle_logic.state_machine.state.name
                if hasattr(vehicle.vehicle_logic, 'loop_counter'):
                    loop_count = vehicle.vehicle_logic.loop_counter
                
                # Get stats from VehicleLogic's Ground Station client
                gs_client = getattr(vehicle.vehicle_logic, 'client_Ground_Station', None)
                gs_stats = gs_client.get_statistics() if gs_client else {}
                telemetry_sent = gs_stats.get('telemetry_sent', 0)
                
                print(f"[STATS] Car {car_id}: State={state}, Loops={loop_count}, Telemetry={telemetry_sent}")
        
        print(f"[MAIN] Car {car_id}: Main loop ended")
    
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