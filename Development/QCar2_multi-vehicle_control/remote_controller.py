"""
Remote Controller for Multiple Physical QCars
This script runs on your Host PC to control multiple physical QCars over the network.
"""

import socket
import json
import time
import threading
from typing import Dict, List, Tuple

class QCarRemoteController:
    """Controller class to manage multiple QCars remotely"""
    
    def __init__(self, host_ip: str = '0.0.0.0', base_port: int = 5000):
        """
        Initialize the remote controller
        
        Args:
            host_ip: IP address of the host PC (0.0.0.0 to listen on all interfaces)
            base_port: Base port number (each car will use base_port + car_id)
        """
        self.host_ip = host_ip
        self.base_port = base_port
        self.cars: Dict[int, Dict] = {}  # car_id -> {socket, address, status}
        self.server_sockets: Dict[int, socket.socket] = {}
        self.running = False
        
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
            
            print(f"[Host PC] Listening for Car {car_id} on port {port}")
    
    def _accept_connection(self, car_id: int, server_sock: socket.socket):
        """Accept connection from a specific car"""
        try:
            while self.running:
                conn, addr = server_sock.accept()
                print(f"[Host PC] Car {car_id} connected from {addr}")
                
                self.cars[car_id] = {
                    'socket': conn,
                    'address': addr,
                    'status': 'connected',
                    'last_data': None
                }
                
                # Start receiving thread for this car
                thread = threading.Thread(target=self._receive_data, args=(car_id, conn))
                thread.daemon = True
                thread.start()
        except Exception as e:
            print(f"[Host PC] Error accepting connection for Car {car_id}: {e}")
    
    def _receive_data(self, car_id: int, conn: socket.socket):
        """Receive telemetry data from a car"""
        buffer = ""
        try:
            while self.running and car_id in self.cars:
                data = conn.recv(4096).decode('utf-8')
                if not data:
                    print(f"[Host PC] Car {car_id} disconnected")
                    break
                
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line:
                        try:
                            telemetry = json.loads(line)
                            self.cars[car_id]['last_data'] = telemetry
                            # print(f"[Car {car_id}] Telemetry received: x={telemetry.get('x', 'N/A'):.2f}, y={telemetry.get('y', 'N/A'):.2f}, v={telemetry.get('v', 'N/A'):.2f}")
                        except json.JSONDecodeError as e:
                            print(f"[Car {car_id}] JSON decode error: {e}, line: {line[:50]}")
                        except Exception as e:
                            print(f"[Car {car_id}] Error processing telemetry: {e}")
        except Exception as e:
            print(f"[Host PC] Error receiving from Car {car_id}: {e}")
        finally:
            if car_id in self.cars:
                self.cars[car_id]['status'] = 'disconnected'
    
    def send_command(self, car_id: int, command: Dict):
        """
        Send command to a specific car
        
        Args:
            car_id: ID of the car (0, 1, 2, ...)
            command: Dictionary with command data, e.g.:
                {
                    'type': 'control',  # 'control', 'stop', 'start', 'set_params'
                    'v_ref': 0.9,       # desired velocity
                    'enable_steering': True,
                    'node_sequence': [10, 4, 20, 10]
                }
        """
        if car_id not in self.cars or self.cars[car_id]['status'] != 'connected':
            print(f"[Host PC] Car {car_id} is not connected")
            return False
        
        try:
            cmd_str = json.dumps(command) + '\n'
            self.cars[car_id]['socket'].sendall(cmd_str.encode('utf-8'))
            print(f"[Host PC] Sent to Car {car_id}: {command}")
            return True
        except Exception as e:
            print(f"[Host PC] Error sending to Car {car_id}: {e}")
            self.cars[car_id]['status'] = 'disconnected'
            return False
    
    def stop_car(self, car_id: int):
        """Send stop command to a car"""
        return self.send_command(car_id, {'type': 'stop'})
    
    def start_car(self, car_id: int):
        """Send start command to a car"""
        return self.send_command(car_id, {'type': 'start'})
    
    def set_velocity(self, car_id: int, velocity: float):
        """Set velocity for a specific car"""
        return self.send_command(car_id, {
            'type': 'set_params',
            'v_ref': velocity
        })
    
    def set_path(self, car_id: int, node_sequence: List[int]):
        """Set path for a specific car"""
        return self.send_command(car_id, {
            'type': 'set_params',
            'node_sequence': node_sequence
        })
    
    def emergency_stop_all(self):
        """Emergency stop for all cars"""
        print("[Host PC] EMERGENCY STOP ALL CARS")
        for car_id in self.cars.keys():
            self.stop_car(car_id)
    
    def get_telemetry(self, car_id: int) -> Dict:
        """Get latest telemetry data from a car"""
        if car_id in self.cars:
            return self.cars[car_id].get('last_data')
        return None
    
    def get_all_telemetry(self) -> Dict[int, Dict]:
        """Get telemetry from all cars"""
        return {car_id: data.get('last_data') for car_id, data in self.cars.items()}
    
    def close(self):
        """Close all connections"""
        self.running = False
        for car_id, car_data in self.cars.items():
            try:
                car_data['socket'].close()
            except:
                pass
        for sock in self.server_sockets.values():
            try:
                sock.close()
            except:
                pass
        print("[Host PC] Controller closed")


def main():
    """Example usage of the remote controller"""
    
    # Configuration
    HOST_IP = '0.0.0.0'  # Listen on all network interfaces
    BASE_PORT = 5000  # Car 0 uses 5000, Car 1 uses 5001, etc.
    NUM_CARS = 2
    
    # Create controller
    controller = QCarRemoteController(HOST_IP, BASE_PORT)
    
    # Start server
    controller.start_server(NUM_CARS)
    
    print("\n" + "="*60)
    print("QCar Remote Controller - Host PC")
    print("="*60)
    print(f"Waiting for {NUM_CARS} cars to connect...")
    print(f"Car 0 should connect to: {HOST_IP}:{BASE_PORT}")
    print(f"Car 1 should connect to: {HOST_IP}:{BASE_PORT + 1}")
    print("\nCommands:")
    print("  start <car_id>     - Start a car")
    print("  stop <car_id>      - Stop a car")
    print("  velocity <car_id> <speed>  - Set velocity")
    print("  path <car_id> <n1> <n2> ... - Set path nodes")
    print("  status             - Show all car status")
    print("  emergency          - Emergency stop all")
    print("  quit               - Exit")
    print("="*60 + "\n")
    
    try:
        while True:
            cmd = input("Command> ").strip().lower().split()
            
            if not cmd:
                continue
            
            if cmd[0] == 'quit':
                break
            
            elif cmd[0] == 'start' and len(cmd) == 2:
                car_id = int(cmd[1])
                controller.start_car(car_id)
            
            elif cmd[0] == 'stop' and len(cmd) == 2:
                car_id = int(cmd[1])
                controller.stop_car(car_id)
            
            elif cmd[0] == 'velocity' and len(cmd) == 3:
                car_id = int(cmd[1])
                velocity = float(cmd[2])
                controller.set_velocity(car_id, velocity)
            
            elif cmd[0] == 'path' and len(cmd) >= 3:
                car_id = int(cmd[1])
                nodes = [int(n) for n in cmd[2:]]
                controller.set_path(car_id, nodes)
            
            elif cmd[0] == 'status':
                print("\n" + "-"*60)
                for car_id, data in controller.cars.items():
                    print(f"Car {car_id}: {data['status']}")
                    if data['last_data']:
                        print(f"  Position: ({data['last_data'].get('x', 'N/A'):.2f}, "
                              f"{data['last_data'].get('y', 'N/A'):.2f})")
                        print(f"  Velocity: {data['last_data'].get('v', 'N/A'):.2f} m/s")
                        print(f"  Heading: {data['last_data'].get('th', 'N/A'):.2f} rad")
                print("-"*60 + "\n")
            
            elif cmd[0] == 'emergency':
                controller.emergency_stop_all()
            
            else:
                print("Unknown command")
    
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    
    finally:
        controller.close()


if __name__ == '__main__':
    main()
