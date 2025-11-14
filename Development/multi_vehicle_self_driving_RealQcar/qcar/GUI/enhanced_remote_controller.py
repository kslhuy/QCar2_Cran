"""
Updated Remote Controller for Ground Station
Compatible with the new CommandHandler system in vehicle_logic.py

Features:
- Uses proper command structure with validation
- Supports all new command types including platoon commands
- Provides better error handling and feedback
- Compatible with both legacy and new command formats
"""

import socket
import json
import time
import threading
from typing import Dict, List, Tuple, Optional
from enum import Enum


class CommandType(Enum):
    """Mirror of CommandType enum from command_handler.py"""
    STOP = "stop"
    START = "start" 
    EMERGENCY_STOP = "emergency_stop"
    SET_VELOCITY = "set_velocity"
    SET_PATH = "set_path"
    SET_PARAMS = "set_params"
    ENABLE_PLATOON_LEADER = "enable_platoon_leader"
    ENABLE_PLATOON_FOLLOWER = "enable_platoon_follower"
    DISABLE_PLATOON = "disable_platoon"
    SHUTDOWN = "shutdown"
    RESET = "reset"


class QCarRemoteController:
    """Enhanced controller class to manage multiple QCars remotely"""
    
    def __init__(self, host_ip: str = '0.0.0.0', base_port: int = 5000):
        """
        Initialize the remote controller
        
        Args:
            host_ip: IP address of the host PC (0.0.0.0 to listen on all interfaces)
            base_port: Base port number (each car will use base_port + car_id)
        """
        self.host_ip = host_ip
        self.base_port = base_port
        self.cars: Dict[int, Dict] = {}  # car_id -> {socket, address, status, last_command_time}
        self.server_sockets: Dict[int, socket.socket] = {}
        self.running = False
        
        # Statistics
        self.commands_sent = 0
        self.commands_failed = 0
        self.start_time = time.time()
        
    def start_server(self, num_cars: int = 2):
        """Start server to accept connections from QCars"""
        self.running = True
        
        for car_id in range(num_cars):
            port = self.base_port + car_id
            server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_sock.bind((self.host_ip, port))
            server_sock.listen(1)
            self.server_sockets[car_id] = server_sock
            
            # Start listening thread for each car
            thread = threading.Thread(target=self._accept_connection, args=(car_id, server_sock))
            thread.daemon = True
            thread.start()
            
            print(f"[Ground Station] Listening for Car {car_id} on port {port}")
    
    def _accept_connection(self, car_id: int, server_sock: socket.socket):
        """Accept connection from a specific car"""
        try:
            while self.running:
                conn, addr = server_sock.accept()
                print(f"[Ground Station] Car {car_id} connected from {addr}")
                
                self.cars[car_id] = {
                    'socket': conn,
                    'address': addr,
                    'status': 'connected',
                    'last_data': None,
                    'last_command_time': 0.0,
                    'commands_sent': 0,
                    'connection_time': time.time()
                }
                
                # Start receiving thread for this car
                thread = threading.Thread(target=self._receive_data, args=(car_id, conn))
                thread.daemon = True
                thread.start()
        except Exception as e:
            print(f"[Ground Station] Error accepting connection for Car {car_id}: {e}")
    
    def _receive_data(self, car_id: int, conn: socket.socket):
        """Receive telemetry data from a car"""
        buffer = ""
        try:
            while self.running and car_id in self.cars:
                data = conn.recv(4096).decode('utf-8')
                if not data:
                    print(f"[Ground Station] Car {car_id} disconnected")
                    break
                
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line:
                        try:
                            telemetry = json.loads(line)
                            self.cars[car_id]['last_data'] = telemetry
                            # Optional: Log telemetry for debugging
                            # print(f"[Car {car_id}] Telemetry: x={telemetry.get('x', 0):.2f}, y={telemetry.get('y', 0):.2f}")
                        except json.JSONDecodeError as e:
                            print(f"[Car {car_id}] JSON decode error: {e}")
                        except Exception as e:
                            print(f"[Car {car_id}] Error processing telemetry: {e}")
        except Exception as e:
            print(f"[Ground Station] Error receiving from Car {car_id}: {e}")
        finally:
            if car_id in self.cars:
                self.cars[car_id]['status'] = 'disconnected'
    
    def send_command(self, car_id: int, command: Dict, validate: bool = True) -> bool:
        """
        Send command to a specific car with validation
        
        Args:
            car_id: ID of the car (0, 1, 2, ...)
            command: Dictionary with command data
            validate: Whether to validate command format before sending
            
        Returns:
            True if command was sent successfully
        """
        if car_id not in self.cars or self.cars[car_id]['status'] != 'connected':
            print(f"[Ground Station] ❌ Car {car_id} is not connected")
            return False
        
        # Add timestamp and source to command
        command_with_metadata = command.copy()
        command_with_metadata['timestamp'] = time.time()
        command_with_metadata['source'] = 'ground_station'
        
        # Validate command format if requested
        if validate and not self._validate_command(command_with_metadata):
            print(f"[Ground Station] ❌ Invalid command format for Car {car_id}: {command}")
            self.commands_failed += 1
            return False
        
        try:
            cmd_str = json.dumps(command_with_metadata) + '\n'
            self.cars[car_id]['socket'].sendall(cmd_str.encode('utf-8'))
            
            # Update statistics
            self.commands_sent += 1
            self.cars[car_id]['commands_sent'] += 1
            self.cars[car_id]['last_command_time'] = time.time()
            
            print(f"[Ground Station] ✅ Sent to Car {car_id}: {command}")
            return True
            
        except Exception as e:
            print(f"[Ground Station] ❌ Error sending to Car {car_id}: {e}")
            self.cars[car_id]['status'] = 'disconnected'
            self.commands_failed += 1
            return False
    
    def _validate_command(self, command: Dict) -> bool:
        """Validate command format"""
        # Check if command has required 'type' field
        if 'type' not in command:
            # Check for legacy formats
            if 'command' in command or 'v_ref' in command:
                return True  # Legacy format is acceptable
            return False
        
        # Validate command type
        cmd_type = command['type']
        valid_types = [cmd.value for cmd in CommandType]
        
        if cmd_type not in valid_types:
            return False
        
        # Type-specific validation
        if cmd_type == 'set_velocity':
            v_ref = command.get('v_ref')
            if v_ref is None or not (0 <= v_ref <= 2.0):
                return False
        
        elif cmd_type == 'enable_platoon':
            role = command.get('role')
            if role not in ['leader', 'follower']:
                return False
            if role == 'follower' and 'leader_id' not in command:
                return False
        
        elif cmd_type == 'set_path':
            node_sequence = command.get('node_sequence')
            if not node_sequence or not isinstance(node_sequence, list):
                return False
        
        return True
    
    # ===== BASIC MOVEMENT COMMANDS =====
    
    def stop_car(self, car_id: int) -> bool:
        """Send stop command to a car"""
        return self.send_command(car_id, {'type': 'stop'})
    
    def start_car(self, car_id: int) -> bool:
        """Send start command to a car"""
        return self.send_command(car_id, {'type': 'start'})
    
    def emergency_stop_car(self, car_id: int) -> bool:
        """Send emergency stop command to a car"""
        return self.send_command(car_id, {'type': 'emergency_stop'})
    
    def shutdown_car(self, car_id: int) -> bool:
        """Send shutdown command to a car"""
        return self.send_command(car_id, {'type': 'shutdown'})
    
    # ===== PARAMETER COMMANDS =====
    
    def set_velocity(self, car_id: int, velocity: float) -> bool:
        """Set velocity for a specific car"""
        return self.send_command(car_id, {
            'type': 'set_velocity',
            'v_ref': velocity
        })
    
    def set_path(self, car_id: int, node_sequence: List[int]) -> bool:
        """Set path for a specific car"""
        return self.send_command(car_id, {
            'type': 'set_path',
            'node_sequence': node_sequence
        })
    
    def set_params(self, car_id: int, **params) -> bool:
        """Set multiple parameters for a car"""
        command = {'type': 'set_params'}
        command.update(params)
        return self.send_command(car_id, command)
    
    # ===== PLATOON COMMANDS =====
    
    def enable_platoon_leader(self, car_id: int) -> bool:
        """Enable platoon leader mode for a car"""
        return self.send_command(car_id, {
            'type': 'enable_platoon',
            'role': 'leader'
        })
    
    def enable_platoon_follower(self, car_id: int, leader_id: int, following_distance: float = 2.0) -> bool:
        """Enable platoon follower mode for a car"""
        return self.send_command(car_id, {
            'type': 'enable_platoon',
            'role': 'follower',
            'leader_id': leader_id,
            'following_distance': following_distance
        })
    
    def disable_platoon(self, car_id: int) -> bool:
        """Disable platoon mode for a car"""
        return self.send_command(car_id, {'type': 'disable_platoon'})
    
    # ===== FLEET OPERATIONS =====
    
    def stop_all_cars(self) -> Dict[int, bool]:
        """Stop all cars"""
        results = {}
        print("[Ground Station] 🛑 STOPPING ALL CARS")
        for car_id in self.cars.keys():
            results[car_id] = self.stop_car(car_id)
        return results
    
    def start_all_cars(self) -> Dict[int, bool]:
        """Start all cars"""
        results = {}
        print("[Ground Station] ▶️  STARTING ALL CARS")
        for car_id in self.cars.keys():
            results[car_id] = self.start_car(car_id)
        return results
    
    def emergency_stop_all(self) -> Dict[int, bool]:
        """Emergency stop for all cars"""
        results = {}
        print("[Ground Station] 🚨 EMERGENCY STOP ALL CARS")
        for car_id in self.cars.keys():
            results[car_id] = self.emergency_stop_car(car_id)
        return results
    
    def setup_convoy(self, leader_id: int, follower_ids: List[int], following_distance: float = 2.0) -> bool:
        """Setup a convoy with one leader and multiple followers"""
        print(f"[Ground Station] 🚗 Setting up convoy: Leader={leader_id}, Followers={follower_ids}")
        
        # Enable leader
        if not self.enable_platoon_leader(leader_id):
            return False
        
        # Enable followers
        success = True
        for follower_id in follower_ids:
            if not self.enable_platoon_follower(follower_id, leader_id, following_distance):
                success = False
        
        return success
    
    def disable_all_platoons(self) -> Dict[int, bool]:
        """Disable platoon mode for all cars"""
        results = {}
        print("[Ground Station] 🚗 Disabling all platoons")
        for car_id in self.cars.keys():
            results[car_id] = self.disable_platoon(car_id)
        return results
    
    # ===== STATUS AND TELEMETRY =====
    
    def get_telemetry(self, car_id: int) -> Optional[Dict]:
        """Get latest telemetry data from a car"""
        if car_id in self.cars:
            return self.cars[car_id].get('last_data')
        return None
    
    def get_all_telemetry(self) -> Dict[int, Dict]:
        """Get telemetry from all cars"""
        return {car_id: data.get('last_data') for car_id, data in self.cars.items() 
                if data.get('last_data') is not None}
    
    def get_car_status(self, car_id: int) -> Dict:
        """Get detailed status of a car"""
        if car_id not in self.cars:
            return {'status': 'not_configured'}
        
        car = self.cars[car_id]
        return {
            'status': car['status'],
            'address': car.get('address'),
            'connection_time': car.get('connection_time', 0),
            'last_command_time': car.get('last_command_time', 0),
            'commands_sent': car.get('commands_sent', 0),
            'has_telemetry': car.get('last_data') is not None,
            'last_telemetry_time': car.get('last_data', {}).get('timestamp', 0) if car.get('last_data') else 0
        }
    
    def get_fleet_status(self) -> Dict:
        """Get overall fleet status"""
        connected_cars = [car_id for car_id, data in self.cars.items() if data['status'] == 'connected']
        
        return {
            'total_cars': len(self.cars),
            'connected_cars': len(connected_cars),
            'connected_car_ids': connected_cars,
            'commands_sent_total': self.commands_sent,
            'commands_failed_total': self.commands_failed,
            'uptime_seconds': time.time() - self.start_time,
            'success_rate': self.commands_sent / max(1, self.commands_sent + self.commands_failed) * 100
        }
    
    def is_car_connected(self, car_id: int) -> bool:
        """Check if a specific car is connected"""
        return car_id in self.cars and self.cars[car_id]['status'] == 'connected'
    
    # ===== LEGACY SUPPORT =====
    
    def send_legacy_command(self, car_id: int, command_str: str, **params) -> bool:
        """Send command in legacy format for backward compatibility"""
        legacy_command = {'command': command_str}
        legacy_command.update(params)
        return self.send_command(car_id, legacy_command, validate=False)
    
    # ===== CLEANUP =====
    
    def close(self):
        """Close all connections and cleanup"""
        self.running = False
        
        # Close car connections
        for car_id, car_data in self.cars.items():
            try:
                car_data['socket'].close()
            except:
                pass
        
        # Close server sockets
        for sock in self.server_sockets.values():
            try:
                sock.close()
            except:
                pass
        
        # Log final statistics
        stats = self.get_fleet_status()
        print(f"\n[Ground Station] 📊 Final Statistics:")
        print(f"  Commands sent: {stats['commands_sent_total']}")
        print(f"  Commands failed: {stats['commands_failed_total']}")
        print(f"  Success rate: {stats['success_rate']:.1f}%")
        print(f"  Uptime: {stats['uptime_seconds']:.1f}s")
        print("[Ground Station] Controller closed")


def main():
    """Enhanced example usage with new command features"""
    
    # Configuration
    HOST_IP = '0.0.0.0'  # Listen on all network interfaces
    BASE_PORT = 5000  # Car 0 uses 5000, Car 1 uses 5001, etc.
    NUM_CARS = 2
    
    # Create enhanced controller
    controller = QCarRemoteController(HOST_IP, BASE_PORT)
    
    # Start server
    controller.start_server(NUM_CARS)
    
    print("\n" + "="*70)
    print("🚗 Enhanced QCar Ground Station Controller")
    print("="*70)
    print(f"Waiting for {NUM_CARS} cars to connect...")
    print(f"Car 0 should connect to: {HOST_IP}:{BASE_PORT}")
    print(f"Car 1 should connect to: {HOST_IP}:{BASE_PORT + 1}")
    print("\n📋 Available Commands:")
    print("  start <car_id>                    - Start a car")
    print("  stop <car_id>                     - Stop a car")
    print("  emergency <car_id>                - Emergency stop a car")
    print("  velocity <car_id> <speed>         - Set velocity (0.0-2.0 m/s)")
    print("  path <car_id> <n1> <n2> ...       - Set path nodes")
    print("  leader <car_id>                   - Enable platoon leader")
    print("  follower <car_id> <leader_id>     - Enable platoon follower")
    print("  convoy <leader_id> <f1> <f2> ...  - Setup convoy")
    print("  disable_platoon <car_id>          - Disable platoon")
    print("  start_all                         - Start all cars")
    print("  stop_all                          - Stop all cars")
    print("  emergency_all                     - Emergency stop all")
    print("  status                            - Show detailed status")
    print("  fleet_status                      - Show fleet overview")
    print("  shutdown <car_id>                 - Shutdown car system")
    print("  quit                              - Exit")
    print("="*70 + "\n")
    
    try:
        while True:
            cmd = input("Ground Station > ").strip().split()
            
            if not cmd:
                continue
            
            command = cmd[0].lower()
            
            if command == 'quit':
                break
            
            elif command == 'start' and len(cmd) == 2:
                car_id = int(cmd[1])
                controller.start_car(car_id)
            
            elif command == 'stop' and len(cmd) == 2:
                car_id = int(cmd[1])
                controller.stop_car(car_id)
            
            elif command == 'emergency' and len(cmd) == 2:
                car_id = int(cmd[1])
                controller.emergency_stop_car(car_id)
            
            elif command == 'velocity' and len(cmd) == 3:
                car_id = int(cmd[1])
                velocity = float(cmd[2])
                if 0 <= velocity <= 2.0:
                    controller.set_velocity(car_id, velocity)
                else:
                    print("❌ Velocity must be between 0.0 and 2.0 m/s")
            
            elif command == 'path' and len(cmd) >= 3:
                car_id = int(cmd[1])
                nodes = [int(n) for n in cmd[2:]]
                controller.set_path(car_id, nodes)
            
            elif command == 'leader' and len(cmd) == 2:
                car_id = int(cmd[1])
                controller.enable_platoon_leader(car_id)
            
            elif command == 'follower' and len(cmd) == 3:
                car_id = int(cmd[1])
                leader_id = int(cmd[2])
                controller.enable_platoon_follower(car_id, leader_id)
            
            elif command == 'convoy' and len(cmd) >= 3:
                leader_id = int(cmd[1])
                follower_ids = [int(f) for f in cmd[2:]]
                controller.setup_convoy(leader_id, follower_ids)
            
            elif command == 'disable_platoon' and len(cmd) == 2:
                car_id = int(cmd[1])
                controller.disable_platoon(car_id)
            
            elif command == 'start_all':
                controller.start_all_cars()
            
            elif command == 'stop_all':
                controller.stop_all_cars()
            
            elif command == 'emergency_all':
                controller.emergency_stop_all()
            
            elif command == 'shutdown' and len(cmd) == 2:
                car_id = int(cmd[1])
                controller.shutdown_car(car_id)
            
            elif command == 'status':
                print("\n" + "-"*60)
                for car_id in range(NUM_CARS):
                    status = controller.get_car_status(car_id)
                    print(f"🚗 Car {car_id}: {status['status'].upper()}")
                    
                    if status['status'] == 'connected':
                        telemetry = controller.get_telemetry(car_id)
                        if telemetry:
                            print(f"   📍 Position: ({telemetry.get('x', 0):.2f}, {telemetry.get('y', 0):.2f})")
                            print(f"   🏃 Velocity: {telemetry.get('v', 0):.2f} m/s")
                            print(f"   🧭 Heading: {telemetry.get('th', 0):.2f} rad")
                            print(f"   🎯 State: {telemetry.get('state', 'unknown')}")
                        
                        uptime = time.time() - status['connection_time']
                        print(f"   🔗 Connected for: {uptime:.1f}s")
                        print(f"   📤 Commands sent: {status['commands_sent']}")
                
                print("-"*60 + "\n")
            
            elif command == 'fleet_status':
                stats = controller.get_fleet_status()
                print("\n" + "-"*60)
                print("🚁 Fleet Overview:")
                print(f"   Total cars: {stats['total_cars']}")
                print(f"   Connected: {stats['connected_cars']}")
                print(f"   Connected IDs: {stats['connected_car_ids']}")
                print(f"   Commands sent: {stats['commands_sent_total']}")
                print(f"   Commands failed: {stats['commands_failed_total']}")
                print(f"   Success rate: {stats['success_rate']:.1f}%")
                print(f"   Uptime: {stats['uptime_seconds']:.1f}s")
                print("-"*60 + "\n")
            
            else:
                print("❌ Unknown command or incorrect parameters")
    
    except KeyboardInterrupt:
        print("\n\n🛑 Shutting down Ground Station...")
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
    
    finally:
        controller.close()


if __name__ == '__main__':
    main()