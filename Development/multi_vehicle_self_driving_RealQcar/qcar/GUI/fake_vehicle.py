"""
Fake Vehicle Simulator for Testing Ground Station Connection
This script simulates a QCar vehicle connecting to the Ground Station
and sending realistic telemetry data while receiving commands.

Usage:
    python fake_vehicle.py [car_id] [host_ip] [base_port]
    
Examples:
    python fake_vehicle.py                    # Car 0 connecting to localhost:5000
    python fake_vehicle.py 1                 # Car 1 connecting to localhost:5001  
    python fake_vehicle.py 0 192.168.1.100   # Car 0 connecting to remote IP
"""

import socket
import json
import time
import threading
import math
import sys
from typing import Dict, Any, Optional


class FakeVehicle:
    """Simulates a QCar vehicle for Ground Station testing"""
    
    def __init__(self, car_id: int = 0, host_ip: str = '127.0.0.1', base_port: int = 5000):
        self.car_id = car_id
        self.host_ip = host_ip
        self.port = base_port + car_id
        
        # Vehicle state
        self.x = car_id * 2.0  # Start positions spread out
        self.y = 0.0
        self.heading = 0.0
        self.velocity = 0.0
        self.target_velocity = 0.0
        self.throttle = 0.0
        self.steering = 0.0
        
        # Vehicle status
        self.vehicle_state = "IDLE"  # IDLE, RUNNING, STOPPED, EMERGENCY
        self.is_running = False
        self.path_nodes = [1, 2, 3, 4]  # Default path
        self.current_node = 0
        
        # Platoon settings
        self.platoon_enabled = False
        self.platoon_role = "none"  # "none", "leader", "follower"
        self.platoon_leader_id = None
        self.following_distance = 2.0
        
        # Network
        self.socket = None
        self.connected = False
        self.running = False
        
        # Threading
        self.telemetry_thread = None
        self.command_thread = None
        
        # Statistics
        self.telemetry_sent = 0
        self.commands_received = 0
        self.start_time = time.time()
        
        print(f"🚗 Fake Car {self.car_id} initialized")
        print(f"   Target: {self.host_ip}:{self.port}")
        print(f"   Initial position: ({self.x:.1f}, {self.y:.1f})")
    
    def connect_to_ground_station(self) -> bool:
        """Connect to the Ground Station"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host_ip, self.port))
            self.connected = True
            print(f"✅ Car {self.car_id} connected to Ground Station at {self.host_ip}:{self.port}")
            return True
        except ConnectionRefusedError:
            print(f"❌ Car {self.car_id}: Connection refused. Is Ground Station running on {self.host_ip}:{self.port}?")
            return False
        except Exception as e:
            print(f"❌ Car {self.car_id}: Connection failed - {e}")
            return False
    
    def start_simulation(self):
        """Start the vehicle simulation"""
        if not self.connected:
            print(f"❌ Car {self.car_id}: Not connected to Ground Station")
            return
        
        self.running = True
        
        # Start telemetry sender thread
        self.telemetry_thread = threading.Thread(target=self._telemetry_worker, daemon=True)
        self.telemetry_thread.start()
        
        # Start command receiver thread
        self.command_thread = threading.Thread(target=self._command_worker, daemon=True)
        self.command_thread.start()
        
        # Start physics simulation
        physics_thread = threading.Thread(target=self._physics_worker, daemon=True)
        physics_thread.start()
        
        print(f"🚀 Car {self.car_id}: Simulation started")
    
    def _telemetry_worker(self):
        """Send telemetry data to Ground Station"""
        print(f"📡 Car {self.car_id}: Telemetry thread started")
        
        while self.running and self.connected:
            try:
                # Create telemetry packet
                telemetry = {
                    'car_id': self.car_id,
                    'timestamp': time.time(),
                    'x': round(self.x, 3),
                    'y': round(self.y, 3),
                    'th': round(self.heading, 3),  # heading in radians
                    'v': round(self.velocity, 3),  # velocity in m/s
                    'u': round(self.throttle, 3),  # throttle input
                    'delta': round(self.steering, 3),  # steering angle
                    'state': self.vehicle_state,
                    'target_velocity': round(self.target_velocity, 3),
                    'path_nodes': self.path_nodes,
                    'current_node': self.current_node,
                    'platoon_enabled': self.platoon_enabled,
                    'platoon_role': self.platoon_role,
                    'platoon_leader_id': self.platoon_leader_id
                }
                
                # Send telemetry
                message = json.dumps(telemetry) + '\n'
                self.socket.sendall(message.encode('utf-8'))
                self.telemetry_sent += 1
                
                # Send at 10 Hz
                time.sleep(0.1)
                
            except Exception as e:
                print(f"❌ Car {self.car_id}: Error sending telemetry - {e}")
                self.connected = False
                break
        
        print(f"📡 Car {self.car_id}: Telemetry thread stopped")
    
    def _command_worker(self):
        """Receive commands from Ground Station"""
        print(f"🎮 Car {self.car_id}: Command thread started")
        buffer = ""
        
        while self.running and self.connected:
            try:
                # Receive data
                data = self.socket.recv(1024).decode('utf-8')
                if not data:
                    print(f"🔌 Car {self.car_id}: Ground Station disconnected")
                    self.connected = False
                    break
                
                buffer += data
                
                # Process complete messages (newline-delimited)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line:
                        try:
                            command = json.loads(line)
                            self._process_command(command)
                            self.commands_received += 1
                        except json.JSONDecodeError as e:
                            print(f"❌ Car {self.car_id}: JSON decode error - {e}")
                
            except Exception as e:
                print(f"❌ Car {self.car_id}: Error receiving commands - {e}")
                self.connected = False
                break
        
        print(f"🎮 Car {self.car_id}: Command thread stopped")
    
    def _process_command(self, command: Dict[str, Any]):
        """Process received command"""
        cmd_type = command.get('type', command.get('command'))  # Support both formats
        
        print(f"📨 Car {self.car_id}: Received command - {cmd_type}")
        
        if cmd_type == 'start':
            self.vehicle_state = "RUNNING"
            self.is_running = True
            print(f"▶️  Car {self.car_id}: Started")
            
        elif cmd_type == 'stop':
            self.vehicle_state = "STOPPED" 
            self.is_running = False
            self.target_velocity = 0.0
            print(f"⬛ Car {self.car_id}: Stopped")
            
        elif cmd_type == 'emergency_stop':
            self.vehicle_state = "EMERGENCY"
            self.is_running = False
            self.target_velocity = 0.0
            self.velocity = 0.0  # Immediate stop
            print(f"🚨 Car {self.car_id}: EMERGENCY STOP")
            
        elif cmd_type == 'set_velocity':
            velocity = command.get('v_ref', 0.0)
            self.target_velocity = max(0.0, min(2.0, velocity))  # Clamp to 0-2 m/s
            print(f"🎯 Car {self.car_id}: Target velocity set to {self.target_velocity:.2f} m/s")
            
        elif cmd_type == 'set_path':
            nodes = command.get('node_sequence', [])
            if nodes:
                self.path_nodes = nodes
                self.current_node = 0
                print(f"🛤️  Car {self.car_id}: Path updated to {nodes}")
            
        elif cmd_type == 'enable_platoon':
            role = command.get('role', 'follower')
            self.platoon_enabled = True
            self.platoon_role = role
            
            if role == 'leader':
                print(f"👑 Car {self.car_id}: Platoon LEADER enabled")
            elif role == 'follower':
                leader_id = command.get('leader_id')
                self.platoon_leader_id = leader_id
                distance = command.get('following_distance', 2.0)
                self.following_distance = distance
                print(f"🚗 Car {self.car_id}: Platoon FOLLOWER enabled (following Car {leader_id})")
                
        elif cmd_type == 'disable_platoon':
            self.platoon_enabled = False
            self.platoon_role = "none"
            self.platoon_leader_id = None
            print(f"🚗 Car {self.car_id}: Platoon disabled")
            
        elif cmd_type == 'shutdown':
            print(f"🔌 Car {self.car_id}: Shutdown command received")
            self.stop()
            
        else:
            print(f"❓ Car {self.car_id}: Unknown command - {cmd_type}")
    
    def _physics_worker(self):
        """Simulate vehicle physics"""
        print(f"⚙️  Car {self.car_id}: Physics simulation started")
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            if dt > 0.1:  # Skip if too much time has passed
                dt = 0.1
            
            # Update vehicle physics
            self._update_physics(dt)
            
            # Sleep to maintain reasonable update rate
            time.sleep(0.05)  # 20 Hz physics
        
        print(f"⚙️  Car {self.car_id}: Physics simulation stopped")
    
    def _update_physics(self, dt: float):
        """Update vehicle physics simulation"""
        if not self.is_running:
            # Apply braking when not running
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
    
    def stop(self):
        """Stop the simulation"""
        print(f"🛑 Car {self.car_id}: Stopping simulation...")
        self.running = False
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        
        self.connected = False
        
        # Print final stats
        uptime = time.time() - self.start_time
        print(f"\n📊 Car {self.car_id} Final Stats:")
        print(f"   Uptime: {uptime:.1f}s")
        print(f"   Telemetry sent: {self.telemetry_sent}")
        print(f"   Commands received: {self.commands_received}")
        print(f"   Final position: ({self.x:.2f}, {self.y:.2f})")
        print(f"   Final velocity: {self.velocity:.2f} m/s")
    
    def run_interactive(self):
        """Run with interactive commands for testing"""
        print(f"\n🎮 Car {self.car_id} Interactive Mode")
        print("Commands: status, pos, vel <speed>, start, stop, emergency, quit")
        
        while self.running and self.connected:
            try:
                cmd = input(f"Car{self.car_id}> ").strip().lower().split()
                
                if not cmd:
                    continue
                    
                if cmd[0] == 'quit':
                    break
                elif cmd[0] == 'status':
                    print(f"State: {self.vehicle_state}")
                    print(f"Position: ({self.x:.2f}, {self.y:.2f})")
                    print(f"Velocity: {self.velocity:.2f} m/s (target: {self.target_velocity:.2f})")
                    print(f"Heading: {self.heading:.2f} rad")
                    print(f"Platoon: {self.platoon_role} (enabled: {self.platoon_enabled})")
                elif cmd[0] == 'pos':
                    print(f"Position: ({self.x:.3f}, {self.y:.3f})")
                elif cmd[0] == 'vel' and len(cmd) == 2:
                    try:
                        new_vel = float(cmd[1])
                        self.target_velocity = max(0.0, min(2.0, new_vel))
                        print(f"Target velocity set to {self.target_velocity:.2f} m/s")
                    except ValueError:
                        print("Invalid velocity value")
                elif cmd[0] == 'start':
                    self.vehicle_state = "RUNNING"
                    self.is_running = True
                    print("Started")
                elif cmd[0] == 'stop':
                    self.vehicle_state = "STOPPED"
                    self.is_running = False
                    self.target_velocity = 0.0
                    print("Stopped")
                elif cmd[0] == 'emergency':
                    self.vehicle_state = "EMERGENCY"
                    self.is_running = False
                    self.target_velocity = 0.0
                    self.velocity = 0.0
                    print("EMERGENCY STOP")
                else:
                    print("Unknown command")
                    
                time.sleep(0.1)  # Brief pause
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")


def main():
    """Main entry point for fake vehicle"""
    
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
    print("🚗 QCar Fake Vehicle Simulator")
    print("="*60)
    print(f"Car ID: {car_id}")
    print(f"Ground Station: {host_ip}:{base_port + car_id}")
    print("")
    
    # Create and start fake vehicle
    vehicle = FakeVehicle(car_id, host_ip, base_port)
    
    # Connect to Ground Station
    if not vehicle.connect_to_ground_station():
        print("Failed to connect. Make sure Ground Station is running.")
        return
    
    # Start simulation
    vehicle.start_simulation()
    
    print("\n" + "="*60)
    print("🎮 Vehicle is now running and sending telemetry")
    print("   The vehicle will appear in the Ground Station GUI")
    print("   You can send commands from the Ground Station")
    print("   Press Ctrl+C to stop")
    print("="*60)
    
    try:
        # Keep running until interrupted
        while vehicle.running and vehicle.connected:
            time.sleep(1)
            
            # Show periodic status
            if vehicle.telemetry_sent > 0 and vehicle.telemetry_sent % 100 == 0:
                print(f"📡 Car {car_id}: Sent {vehicle.telemetry_sent} telemetry packets, "
                      f"received {vehicle.commands_received} commands")
    
    except KeyboardInterrupt:
        print(f"\n🛑 Car {car_id}: Shutting down...")
    
    except Exception as e:
        print(f"❌ Car {car_id}: Error - {e}")
    
    finally:
        vehicle.stop()
        print(f"🚗 Car {car_id}: Simulation ended")


if __name__ == '__main__':
    main()