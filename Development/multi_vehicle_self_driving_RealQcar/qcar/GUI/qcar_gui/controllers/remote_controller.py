"""
Remote Controller module for QCar Fleet Controller.

This module provides a clean, well-structured interface for communicating
with remote QCar vehicles over TCP/IP sockets. It handles connection management,
command sending, telemetry reception, and status tracking.
"""

import socket
import json
import time
import threading
from typing import Dict, List, Optional, Any, Callable, Set
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
import sys
import os
import sys
import asyncio
import websockets
from websockets.server import WebSocketServerProtocol

# Add qcar directory to path for command_types import
# Path: controllers -> qcar_gui -> GUI -> qcar (where command_types.py is located)
_current_dir = os.path.dirname(os.path.abspath(__file__))
_qcar_dir = os.path.dirname(os.path.dirname(os.path.dirname(_current_dir)))
if _qcar_dir not in sys.path:
    sys.path.insert(0, _qcar_dir)

from command_types import CommandType

# Import scope manager for remote plotting
try:
    from .remote_scope_manager import RemoteScopeManager
    SCOPE_MANAGER_AVAILABLE = True
except ImportError:
    SCOPE_MANAGER_AVAILABLE = False



@dataclass
class CarConnection:
    """Data class representing a car connection."""
    car_id: int
    sock: Optional[socket.socket] = None  # Renamed from 'socket' to avoid module shadowing
    address: Optional[tuple] = None
    status: str = 'disconnected'
    last_data: Optional[Dict] = None
    last_command_time: float = 0.0
    commands_sent: int = 0
    connection_time: float = 0.0


@dataclass
class TelemetryStats:
    """Statistics for telemetry data."""
    msg_count: int = 0
    last_print_time: float = field(default_factory=time.time)
    msg_rate: float = 0.0


@dataclass
class ControllerStats:
    """Statistics for the controller."""
    commands_sent: int = 0
    commands_failed: int = 0
    start_time: float = field(default_factory=time.time)
    
    @property
    def uptime(self) -> float:
        return time.time() - self.start_time
    
    @property
    def success_rate(self) -> float:
        total = self.commands_sent + self.commands_failed
        return (self.commands_sent / max(1, total)) * 100


class CommandValidator:
    """Validates command structure and parameters."""
    
    VALID_COMMAND_TYPES = {cmd.value for cmd in CommandType}
    
    CONTROL_TYPES = {'keyboard', 'wheel', 'joystick'}
    PLATOON_ROLES = {'leader', 'follower'}
    
    @classmethod
    def validate(cls, command: Dict) -> bool:
        """Validate a command dictionary."""
        if 'type' not in command:
            # Check for legacy formats
            return 'command' in command or 'v_ref' in command
        
        cmd_type = command['type']
        if cmd_type not in cls.VALID_COMMAND_TYPES:
            return False
        
        return cls._validate_specific(cmd_type, command)
    
    @classmethod
    def _validate_specific(cls, cmd_type: str, command: Dict) -> bool:
        """Validate type-specific command parameters."""
        validators = {
            'set_velocity': cls._validate_velocity,
            'enable_platoon': cls._validate_platoon,
            'setup_platoon_formation': cls._validate_formation,
            'start_platoon': cls._validate_start_platoon,
            'set_path': cls._validate_path,
            'manual_control': cls._validate_manual_control,
            'enable_manual_mode': cls._validate_manual_mode,
        }
        
        validator = validators.get(cmd_type)
        return validator(command) if validator else True
    
    @classmethod
    def _validate_velocity(cls, command: Dict) -> bool:
        v_ref = command.get('v_ref')
        return v_ref is not None and 0 <= v_ref <= 2.0
    
    @classmethod
    def _validate_platoon(cls, command: Dict) -> bool:
        role = command.get('role')
        if role not in cls.PLATOON_ROLES:
            return False
        if role == 'follower' and 'leader_id' not in command:
            return False
        return True
    
    @classmethod
    def _validate_formation(cls, command: Dict) -> bool:
        formation = command.get('formation')
        if not formation or not isinstance(formation, dict):
            return False
        return all(isinstance(pos, int) and pos >= 1 for pos in formation.values())
    
    @classmethod
    def _validate_start_platoon(cls, command: Dict) -> bool:
        leader_id = command.get('leader_id')
        return leader_id is not None and isinstance(leader_id, int)
    
    @classmethod
    def _validate_path(cls, command: Dict) -> bool:
        node_sequence = command.get('node_sequence')
        return node_sequence and isinstance(node_sequence, list)
    
    @classmethod
    def _validate_manual_control(cls, command: Dict) -> bool:
        throttle = command.get('throttle')
        steering = command.get('steering')
        if throttle is None or steering is None:
            return False
        return -1.0 <= throttle <= 1.0 and -1.0 <= steering <= 1.0
    
    @classmethod
    def _validate_manual_mode(cls, command: Dict) -> bool:
        control_type = command.get('control_type', 'keyboard')
        return control_type in cls.CONTROL_TYPES


class QCarRemoteController:
    """
    Controller for managing multiple QCar connections.
    
    This class handles TCP/IP communication with remote QCar vehicles,
    including connection management, command sending, and telemetry reception.
    """
    
    def __init__(self, host_ip: str = '0.0.0.0', base_port: int = 5000,
                 telemetry_buffer_size: int = 100):
        """
        Initialize the remote controller.
        
        Args:
            host_ip: IP address to listen on (0.0.0.0 for all interfaces)
            base_port: Base port number (each car uses base_port + car_id)
            telemetry_buffer_size: Size of telemetry buffer per car
        """
        self.host_ip = host_ip
        self.base_port = base_port
        self.telemetry_buffer_size = telemetry_buffer_size
        
        # WebSocket state
        self.websocket_port = 8080 # Default, will be configurable
        self.websocket_clients: Set[WebSocketServerProtocol] = set()
        self.websocket_loop: Optional[asyncio.AbstractEventLoop] = None
        self.websocket_thread: Optional[threading.Thread] = None
        
        # Connection state
        self.cars: Dict[int, CarConnection] = {}
        self.server_sockets: Dict[int, socket.socket] = {}
        self.running = False
        
        # Telemetry buffering
        self.telemetry_buffers: Dict[int, deque] = {}
        self.telemetry_stats: Dict[int, TelemetryStats] = {}
        
        # Statistics
        self.stats = ControllerStats()
        
        # Callbacks for special messages
        self._platoon_callback: Optional[Callable[[int, Dict], None]] = None
        self._v2v_callback: Optional[Callable[[int, Dict], None]] = None
        
        # GUI reference for backward compatibility
        self.gui_controller = None
        
        # Scope manager for remote plotting (if available)
        self.scope_manager = None
        if SCOPE_MANAGER_AVAILABLE:
            self.scope_manager = RemoteScopeManager()
    
    # ========== Server Management ==========
    
    def start_server(self, num_cars: int = 2) -> None:
        """
        Start server to accept connections from QCars.
        
        Args:
            num_cars: Number of cars to listen for
        """
        self.running = True
        
        for car_id in range(num_cars):
            port = self.base_port + car_id
            
            try:
                server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server_sock.bind((self.host_ip, port))
                server_sock.listen(1)
                self.server_sockets[car_id] = server_sock
                
                # Start listener thread
                thread = threading.Thread(
                    target=self._accept_connection,
                    args=(car_id, server_sock),
                    daemon=True
                )
                thread.start()
                
                print(f"[Ground Station] Listening for Car {car_id} on port {port}")
                
            except Exception as e:
                print(f"[Ground Station] Failed to start server for Car {car_id}: {e}")
        
        # Start WebSocket server
        self._start_websocket_server()

    def _start_websocket_server(self):
        """Start WebSocket server in a separate thread."""
        def run_loop():
            self.websocket_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.websocket_loop)
            
            async def start_server():
                print(f"[Ground Station] WebSocket server starting on port {self.websocket_port}")
                async with websockets.serve(
                    self._handle_websocket_client,
                    "0.0.0.0",
                    self.websocket_port
                ):
                    await asyncio.Future()  # Run forever

            try:
                self.websocket_loop.run_until_complete(start_server())
            except RuntimeError:
                # Loop stopped before future completed - normal during shutdown
                pass

        self.websocket_thread = threading.Thread(target=run_loop, daemon=True)
        self.websocket_thread.start()

    async def _handle_websocket_client(self, websocket: WebSocketServerProtocol):
        """Handle a WebSocket connection from browser."""
        self.websocket_clients.add(websocket)
        print(f"[WS] Client connected from {websocket.remote_address}")
        
        try:
            # Send initial status for all connected cars
            for car_id, car in self.cars.items():
                if car.status == 'connected':
                    await websocket.send(json.dumps({
                        "type": "vehicle_status",
                        "vehicle_id": f"qcar-{car_id}", # Match bridge format
                        "status": "connected",
                        "ip": car.address[0] if car.address else "unknown",
                        "port": self.base_port + car_id
                    }))

            async for message in websocket:
                await self._handle_websocket_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.websocket_clients.discard(websocket)
            print("[WS] Client disconnected")

    async def _handle_websocket_message(self, websocket, message):
        """Handle incoming WebSocket message."""
        try:
            data = json.loads(message)
            msg_type = data.get('type')
            
            if msg_type == 'ping':
                await websocket.send(json.dumps({"type": "pong"}))
                return
                
            elif msg_type == 'get_status':
                # Send current vehicle status for all connected cars
                for car_id, car in self.cars.items():
                    if car.status == 'connected':
                        await websocket.send(json.dumps({
                            "type": "vehicle_status",
                            "vehicle_id": f"qcar-{car_id}",
                            "status": "connected",
                            "ip": car.address[0] if car.address else "unknown",
                            "port": self.base_port + car_id
                        }))
                        
                        # Also send latest telemetry if available
                        if car.last_data:
                            telemetry = car.last_data.copy()
                            telemetry['vehicle_id'] = f"qcar-{car_id}"
                            await websocket.send(json.dumps(telemetry))
                return
                
            # Handle commands from browser
            target = data.get('target', 'all')
            # Extract actual command payload (remove protocol wrapper if needed)
            # The bridge passed 'action' or 'msg_type' as command
            
            if target == 'all':
                for car_id in self.cars:
                    self.send_command(car_id, data)
            else:
                # Target format might be 'qcar-1' or just '1'
                try:
                    target_id = int(str(target).replace('qcar-', ''))
                    self.send_command(target_id, data)
                except ValueError:
                    pass

        except Exception as e:
            print(f"[WS] Error handling message: {e}")

    def _broadcast_to_websockets(self, data: dict):
        """Broadcast data to all connected WebSocket clients."""
        if not self.websocket_clients or not self.websocket_loop:
            return
            
        async def send():
            disconnected = set()
            message = json.dumps(data)
            for client in self.websocket_clients:
                try:
                    await client.send(message)
                except:
                    disconnected.add(client)
            self.websocket_clients -= disconnected
            
        asyncio.run_coroutine_threadsafe(send(), self.websocket_loop)
    
    def stop(self) -> None:
        """Stop the controller and servers."""
        self.running = False
        
        # Shutdown scope manager
        if self.scope_manager:
            self.scope_manager.shutdown()
            
        # Close all server sockets
        for sock in self.server_sockets.values():
            try:
                sock.close()
            except:
                pass
        self.server_sockets.clear()
        
        # Stop WebSocket server
        if self.websocket_loop:
            self.websocket_loop.call_soon_threadsafe(self.websocket_loop.stop)
            if self.websocket_thread:
                self.websocket_thread.join(timeout=1.0)
        
        # Close all car connections
        for car in self.cars.values():
            if car.sock:
                try:
                    car.sock.close()
                except:
                    pass
        self.cars.clear()
        
        print("[Ground Station] Controller stopped")
    
    def _accept_connection(self, car_id: int, server_sock: socket.socket) -> None:
        """Accept and handle connection from a specific car."""
        try:
            while self.running:
                conn, addr = server_sock.accept()
                print(f"[Ground Station] Car {car_id} connected from {addr}")
                
                # Initialize car connection
                self.cars[car_id] = CarConnection(
                    car_id=car_id,
                    sock=conn,
                    address=addr,
                    status='connected',
                    connection_time=time.time()
                )
                
                # Initialize telemetry buffer
                self.telemetry_buffers[car_id] = deque(maxlen=self.telemetry_buffer_size)
                self.telemetry_stats[car_id] = TelemetryStats()
                
                # Start receiver thread
                thread = threading.Thread(
                    target=self._receive_data,
                    args=(car_id, conn),
                    daemon=True
                )
                thread.start()
                
                # Broadcast vehicle connection to WebSocket clients
                self._broadcast_to_websockets({
                    "type": "vehicle_status",
                    "vehicle_id": f"qcar-{car_id}",
                    "status": "connected",
                    "ip": addr[0],
                    "port": self.base_port + car_id
                })
                
        except Exception as e:
            if self.running:
                print(f"[Ground Station] Error accepting connection for Car {car_id}: {e}")
    
    def _receive_data(self, car_id: int, conn: socket.socket) -> None:
        """Receive and process telemetry data from a car."""
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
                        self._process_message(car_id, line)
                        
        except Exception as e:
            print(f"[Ground Station] Error receiving from Car {car_id}: {e}")
        finally:
            # Clean up car data on disconnection
            self._cleanup_car(car_id)
    
    def _cleanup_car(self, car_id: int) -> None:
        """Clean up all local data for a disconnected car.
        
        This removes the car from tracking so no further commands are attempted.
        The server will continue listening for reconnection.
        """
        # Close socket if still open
        if car_id in self.cars and self.cars[car_id].sock:
            try:
                self.cars[car_id].sock.close()
            except Exception:
                pass
        
        # Remove car from tracking
        if car_id in self.cars:
            del self.cars[car_id]
        
        # Clear telemetry data
        if car_id in self.telemetry_buffers:
            del self.telemetry_buffers[car_id]
        if car_id in self.telemetry_stats:
            del self.telemetry_stats[car_id]
        
        # Broadcast disconnection to WebSocket clients
        self._broadcast_to_websockets({
            "type": "vehicle_status",
            "vehicle_id": f"qcar-{car_id}",
            "status": "disconnected"
        })
        
        # Notify GUI controller to clean up manual mode state
        if self.gui_controller and hasattr(self.gui_controller, '_on_car_disconnected'):
            self.gui_controller._on_car_disconnected(car_id)
        
        print(f"[Ground Station] Car {car_id} data cleaned up, waiting for reconnection...")
    
    def _process_message(self, car_id: int, message: str) -> None:
        """Process a received message."""
        try:
            data = json.loads(message)
            
            # Update last data (MERGE instead of overwrite to support partial updates)
            if self.cars[car_id].last_data is None:
                self.cars[car_id].last_data = {}
            
            self.cars[car_id].last_data.update(data)
            self.cars[car_id].last_data['timestamp_recv'] = time.time()
            
            # Add to buffer
            if car_id in self.telemetry_buffers:
                self.telemetry_buffers[car_id].append({
                    'data': data,
                    'timestamp': time.time()
                })
            
            # Update statistics
            self._update_telemetry_stats(car_id)
            
            # Process special messages
            msg_type = data.get('type', 'telemetry')
            
            # Broadcast to WebSockets
            # Add vehicle_id for the web app and ensure type is set
            ws_data = data.copy()
            ws_data['vehicle_id'] = f"qcar-{car_id}"
            # Ensure 'type' field is always present for the web handler
            if 'type' not in ws_data:
                ws_data['type'] = 'telemetry'
            self._broadcast_to_websockets(ws_data)

            # Feed regular telemetry to observer plot buffer.
            if msg_type != 'scope_data' and self.scope_manager:
                self.scope_manager.receive_observer_telemetry(car_id, data)

            self._handle_special_message(car_id, msg_type, data)
            
        except json.JSONDecodeError as e:
            print(f"[Car {car_id}] JSON decode error: {e}")
        except Exception as e:
            print(f"[Car {car_id}] Error processing message: {e}")
    
    def _update_telemetry_stats(self, car_id: int) -> None:
        """Update telemetry statistics."""
        if car_id not in self.telemetry_stats:
            return
        
        stats = self.telemetry_stats[car_id]
        stats.msg_count += 1
        
        current_time = time.time()
        time_elapsed = current_time - stats.last_print_time
        
        if time_elapsed >= 1.0:
            stats.msg_rate = stats.msg_count / time_elapsed
            stats.msg_count = 0
            stats.last_print_time = current_time
    
    def _handle_special_message(self, car_id: int, msg_type: str, data: Dict) -> None:
        """Handle special message types."""
        if msg_type == 'scope_data':
            # High-frequency scope data for remote plotting
            if self.scope_manager:
                payload = data.get('payload', '')
                self.scope_manager.receive_scope_data(car_id, payload)
            return  # Don't process as telemetry
        
        elif msg_type == 'v2v_status':
            v2v_data = data.get('data', {})
            if self._v2v_callback:
                self._v2v_callback(car_id, v2v_data)
            elif self.gui_controller and hasattr(self.gui_controller, 'process_v2v_status'):
                self.gui_controller.process_v2v_status(car_id, v2v_data)
                
        elif msg_type == 'platoon_setup_confirm':
            platoon_data = data.get('data', {})
            if self._platoon_callback:
                self._platoon_callback(car_id, platoon_data)
            elif self.gui_controller and hasattr(self.gui_controller, 'process_platoon_setup_confirmation'):
                self.gui_controller.process_platoon_setup_confirmation(car_id, platoon_data)
    
    # ========== Command Sending ==========
    
    def send_command(self, car_id: int, command: Dict, validate: bool = True) -> bool:
        """
        Send a command to a specific car.
        
        Args:
            car_id: ID of the target car
            command: Command dictionary
            validate: Whether to validate the command
            
        Returns:
            True if command was sent successfully
        """
        if not self.is_car_connected(car_id):
            # Don't log here - disconnection is already logged once when it happens
            return False
        
        # Add metadata
        command_with_metadata = command.copy()
        command_with_metadata['timestamp'] = time.time()
        command_with_metadata['source'] = 'ground_station'
        
        # Validate
        if validate and not CommandValidator.validate(command_with_metadata):
            print(f"[Ground Station] ❌ Invalid command format: {command}")
            self.stats.commands_failed += 1
            return False
        
        try:
            cmd_str = json.dumps(command_with_metadata) + '\n'
            self.cars[car_id].sock.sendall(cmd_str.encode('utf-8'))
            
            # Update statistics
            self.stats.commands_sent += 1
            self.cars[car_id].commands_sent += 1
            self.cars[car_id].last_command_time = time.time()
            
            return True
            
        except Exception as e:
            print(f"[Ground Station] ❌ Error sending to Car {car_id}: {e}")
            self.cars[car_id].status = 'disconnected'
            self.stats.commands_failed += 1
            return False
    
    # ========== Basic Movement Commands ==========
    
    def start_car(self, car_id: int) -> bool:
        """Send start command."""
        return self.send_command(car_id, {'type': CommandType.START.value})
    
    def stop_car(self, car_id: int) -> bool:
        """Send stop command."""
        return self.send_command(car_id, {'type': CommandType.STOP.value, 'source': 'Ground Station'})
    
    def emergency_stop_car(self, car_id: int) -> bool:
        """Send emergency stop command."""
        return self.send_command(car_id, {
            'type': CommandType.EMERGENCY_STOP.value,
            'source': 'Ground Station',
            'reason': 'Emergency'
        })
    
    def shutdown_car(self, car_id: int) -> bool:
        """Send shutdown command."""
        return self.send_command(car_id, {'type': CommandType.SHUTDOWN.value})
    
    # ========== Parameter Commands ==========
    
    def set_velocity(self, car_id: int, velocity: float) -> bool:
        """Set target velocity."""
        return self.send_command(car_id, {'type': CommandType.SET_VELOCITY.value, 'v_ref': velocity})
    
    def set_path(self, car_id: int, node_sequence: List[int]) -> bool:
        """Set path nodes."""
        return self.send_command(car_id, {'type': CommandType.SET_PATH.value, 'node_sequence': node_sequence})
    
    def set_initial_position(self, car_id: int, x: float, y: float,
                             theta: float = 0.0, calibrate: bool = True) -> bool:
        """Set initial position."""
        return self.send_command(car_id, {
            'type': CommandType.SET_INITIAL_POSITION.value,
            'x': x,
            'y': y,
            'theta': theta,
            'calibrate': calibrate
        })
    
    def set_params(self, car_id: int, **params) -> bool:
        """Set multiple parameters."""
        command = {'type': CommandType.SET_PARAMS.value}
        command.update(params)
        return self.send_command(car_id, command)
    
    # ========== Platoon Commands ==========
    
    def enable_platoon_leader(self, car_id: int) -> bool:
        """Enable platoon leader mode."""
        return self.send_command(car_id, {'type': CommandType.ENABLE_PLATOON_LEADER.value, 'role': 'leader'})
    
    def enable_platoon_follower(self, car_id: int, leader_id: int,
                                 following_distance: float = 2.0) -> bool:
        """Enable platoon follower mode."""
        return self.send_command(car_id, {
            'type': CommandType.ENABLE_PLATOON_FOLLOWER.value,
            'role': 'follower',
            'leader_id': leader_id,
            'following_distance': following_distance
        })
    
    def disable_platoon(self, car_id: int) -> bool:
        """Disable platoon mode."""
        return self.send_command(car_id, {'type': CommandType.DISABLE_PLATOON.value})
    
    def setup_global_platoon_formation(self, formation: Dict[int, int]) -> Dict[int, bool]:
        """
        Send global platoon formation to all vehicles.
        
        Args:
            formation: Dict mapping car_id -> position (1=Leader, 2+=Followers)
            
        Returns:
            Dict of car_id -> success results
        """
        # Find leader
        leader_id = next((cid for cid, pos in formation.items() if pos == 1), None)
        
        formation_command = {
            'type': CommandType.SETUP_PLATOON_FORMATION.value,
            'formation': formation,
            'leader_id': leader_id,
            'timestamp': time.time()
        }
        
        print(f"[Ground Station] 🚗🚗 Setting up global platoon formation:")
        for car_id, position in formation.items():
            role = "LEADER" if position == 1 else "FOLLOWER"
            print(f"  Car {car_id}: Position {position} ({role})")
        
        results = {}
        for car_id in formation.keys():
            success = self.send_command(car_id, formation_command)
            results[car_id] = success
            status = "✅" if success else "❌"
            print(f"{status} Car {car_id}: Formation data {'sent' if success else 'failed'}")
        
        return results
    
    def start_platoon_mode(self, car_id: int, leader_id: int) -> Dict[str, Any]:
        """Start platoon mode after formation setup."""
        if not self.is_car_connected(car_id):
            return {'status': 'error', 'message': f'Car {car_id} not connected'}
        
        command = {
            'type': CommandType.START_PLATOON.value,
            'leader_id': leader_id,
            'timestamp': time.time()
        }
        
        print(f"[Ground Station] 🚀 Starting platoon mode for Car {car_id} (Leader: {leader_id})")
        success = self.send_command(car_id, command)
        
        if success:
            return {'status': 'success', 'message': 'Platoon start command sent'}
        return {'status': 'error', 'message': 'Failed to send start platoon command'}
    
    # ========== Fleet Operations ==========
    
    def start_all_cars(self) -> Dict[int, bool]:
        """Start all connected cars."""
        print("[Ground Station] ▶️  STARTING ALL CARS")
        return {car_id: self.start_car(car_id) for car_id in self.cars.keys()}
    
    def stop_all_cars(self) -> Dict[int, bool]:
        """Stop all connected cars."""
        print("[Ground Station] 🛑 STOPPING ALL CARS")
        return {car_id: self.stop_car(car_id) for car_id in self.cars.keys()}
    
    def emergency_stop_all(self) -> Dict[int, bool]:
        """Emergency stop all cars."""
        print("[Ground Station] 🚨 EMERGENCY STOP ALL CARS")
        return {car_id: self.emergency_stop_car(car_id) for car_id in self.cars.keys()}
    
    def disable_all_platoons(self) -> Dict[int, bool]:
        """Disable platoon mode for all cars."""
        print("[Ground Station] 🚗 Disabling all platoons")
        return {car_id: self.disable_platoon(car_id) for car_id in self.cars.keys()}
    
    def setup_convoy(self, leader_id: int, follower_ids: List[int],
                     following_distance: float = 2.0) -> bool:
        """Setup convoy with leader and followers."""
        print(f"[Ground Station] 🚗 Setting up convoy: Leader={leader_id}, Followers={follower_ids}")
        
        if not self.enable_platoon_leader(leader_id):
            return False
        
        return all(
            self.enable_platoon_follower(fid, leader_id, following_distance)
            for fid in follower_ids
        )
    
    # ========== Manual Control Commands ==========
    
    def enable_manual_mode(self, car_id: int, control_type: str = 'keyboard') -> bool:
        """Enable manual control mode."""
        return self.send_command(car_id, {
            'type': CommandType.ENABLE_MANUAL_MODE.value,
            'control_type': control_type
        })
    
    def send_manual_control(self, car_id: int, throttle: float, steering: float) -> bool:
        """Send manual control inputs (high frequency, skip validation)."""
        throttle = max(-1.0, min(1.0, throttle))
        steering = max(-1.0, min(1.0, steering))
        
        return self.send_command(car_id, {
            'type': CommandType.MANUAL_CONTROL.value,
            'throttle': throttle,
            'steering': steering
        }, validate=False)
    
    def disable_manual_mode(self, car_id: int) -> bool:
        """Disable manual control mode."""
        return self.send_command(car_id, {'type': CommandType.DISABLE_MANUAL_MODE.value})
    
    # ========== Perception Commands ==========
    
    def activate_perception(self, car_id: int) -> bool:
        """
        Activate perception system (YOLO) for a vehicle.
        
        Args:
            car_id: Vehicle ID
            
        Returns:
            bool: True if command sent successfully
        """
        return self.send_command(car_id, {'type': CommandType.ACTIVATE_PERCEPTION.value})
    
    def disable_perception(self, car_id: int) -> bool:
        """
        Disable perception system for a vehicle.
        
        Args:
            car_id: Vehicle ID
            
        Returns:
            bool: True if command sent successfully
        """
        return self.send_command(car_id, {'type': CommandType.DISABLE_PERCEPTION.value})
    
    def activate_perception_fleet(self, car_ids: List[int] = None) -> Dict[int, bool]:
        """
        Activate perception for multiple vehicles.
        
        Args:
            car_ids: List of vehicle IDs. If None, activates for all connected vehicles.
            
        Returns:
            Dict mapping car_id to success status
        """
        if car_ids is None:
            car_ids = list(self.cars.keys())
        
        results = {}
        for car_id in car_ids:
            results[car_id] = self.activate_perception(car_id)
        return results
    
    # ========== Scope Streaming Commands ==========
    
    def enable_scope_streaming(self, car_id: int, preset_names: List[str] = None,
                                stream_rate: float = 50.0) -> bool:
        """
        Enable scope data streaming from a vehicle for remote plotting.
        
        Args:
            car_id: Vehicle ID
            preset_names: List of preset names to stream (default: local_state, local_control)
            stream_rate: Streaming rate in Hz (default 50)
            
        Returns:
            bool: True if command sent successfully
        """
        if preset_names is None:
            preset_names = ['local_state', 'local_control']
        
        # Start receiving on scope manager
        if self.scope_manager:
            self.scope_manager.start_stream(car_id, preset_names)
        
        return self.send_command(car_id, {
            'type': CommandType.ENABLE_SCOPE_STREAMING.value,
            'preset_names': preset_names,
            'stream_rate': stream_rate
        })
    
    def disable_scope_streaming(self, car_id: int) -> bool:
        """
        Disable scope data streaming from a vehicle.
        
        Args:
            car_id: Vehicle ID
            
        Returns:
            bool: True if command sent successfully
        """
        # Stop receiving on scope manager
        if self.scope_manager:
            self.scope_manager.stop_stream(car_id)
        
        return self.send_command(car_id, {'type': CommandType.DISABLE_SCOPE_STREAMING.value})
    
    def open_scope_viewer(self, car_id: int, preset_names: List[str] = None) -> bool:
        """
        Open a scope viewer window for a vehicle.
        
        Args:
            car_id: Vehicle ID
            preset_names: Presets to visualize
            
        Returns:
            bool: True if viewer opened
        """
        if self.scope_manager:
            self.scope_manager.open_viewer(car_id, preset_names)
            return True
        return False
    
    def close_scope_viewer(self, car_id: int) -> bool:
        """
        Close the scope viewer window for a vehicle.
        
        Args:
            car_id: Vehicle ID
            
        Returns:
            bool: True if viewer closed
        """
        if self.scope_manager:
            self.scope_manager.close_viewer(car_id)
            return True
        return False
    
    def is_scope_streaming(self, car_id: int) -> bool:
        """
        Check if scope streaming is active for a vehicle.
        
        Args:
            car_id: Vehicle ID
            
        Returns:
            bool: True if streaming is active
        """
        if self.scope_manager:
            return self.scope_manager.is_streaming(car_id)
        return False

    def open_plot_all_observer_viewer(self, refresh_ms: int = 150, time_window: float = 20.0) -> bool:
        """Open integrated Plot-All observer viewer backed by scope manager."""
        if not self.scope_manager:
            return False
        return self.scope_manager.open_observer_viewer(refresh_ms=refresh_ms, time_window=time_window)

    def close_plot_all_observer_viewer(self) -> bool:
        """Close integrated Plot-All observer viewer if running."""
        if not self.scope_manager:
            return False
        self.scope_manager.close_observer_viewer()
        return True

    def is_plot_all_observer_viewer_running(self) -> bool:
        """Return whether integrated Plot-All observer viewer is running."""
        if not self.scope_manager:
            return False
        return self.scope_manager.is_observer_viewer_running()
    
    # ========== Status and Telemetry ==========
    
    def get_telemetry(self, car_id: int) -> Optional[Dict]:
        """Get latest telemetry data."""
        if car_id in self.cars:
            return self.cars[car_id].last_data
        return None
    
    def get_telemetry_buffer(self, car_id: int, max_count: int = None) -> List[Dict]:
        """Get buffered telemetry data."""
        if car_id not in self.telemetry_buffers:
            return []
        
        buffer = list(self.telemetry_buffers[car_id])
        if max_count:
            buffer = buffer[-max_count:]
        
        return list(reversed(buffer))
    
    def get_telemetry_rate(self, car_id: int) -> float:
        """Get current telemetry message rate (Hz)."""
        if car_id in self.telemetry_stats:
            return self.telemetry_stats[car_id].msg_rate
        return 0.0
    
    def get_all_telemetry(self) -> Dict[int, Dict]:
        """Get telemetry from all cars."""
        return {
            car_id: car.last_data
            for car_id, car in self.cars.items()
            if car.last_data is not None
        }
    
    def get_car_status(self, car_id: int) -> Dict[str, Any]:
        """Get detailed status of a car."""
        if car_id not in self.cars:
            return {'status': 'not_configured'}
        
        car = self.cars[car_id]
        return {
            'status': car.status,
            'address': car.address,
            'connection_time': car.connection_time,
            'last_command_time': car.last_command_time,
            'commands_sent': car.commands_sent,
            'has_telemetry': car.last_data is not None,
            'last_telemetry_time': car.last_data.get('timestamp', 0) if car.last_data else 0
        }
    
    def get_fleet_status(self) -> Dict[str, Any]:
        """Get overall fleet status."""
        connected_cars = [cid for cid, car in self.cars.items() if car.status == 'connected']
        
        avg_rate = 0.0
        if connected_cars:
            total_rate = sum(self.get_telemetry_rate(cid) for cid in connected_cars)
            avg_rate = total_rate / len(connected_cars)
        
        return {
            'total_cars': len(self.cars),
            'connected_cars': len(connected_cars),
            'connected_car_ids': connected_cars,
            'commands_sent_total': self.stats.commands_sent,
            'commands_failed_total': self.stats.commands_failed,
            'uptime_seconds': self.stats.uptime,
            'success_rate': self.stats.success_rate,
            'avg_telemetry_rate_hz': avg_rate
        }
    
    def is_car_connected(self, car_id: int) -> bool:
        """Check if a car is connected."""
        return car_id in self.cars and self.cars[car_id].status == 'connected'
    
    # ========== Callbacks ==========
    
    def set_platoon_callback(self, callback: Callable[[int, Dict], None]) -> None:
        """Set callback for platoon status updates."""
        self._platoon_callback = callback
    
    def set_v2v_callback(self, callback: Callable[[int, Dict], None]) -> None:
        """Set callback for V2V status updates."""
        self._v2v_callback = callback
    
    
    # ========== Cleanup ==========
    
    def close(self) -> None:
        """Close all connections and cleanup."""
        self.running = False
        
        # Close car connections
        for car in self.cars.values():
            try:
                if car.sock:
                    car.sock.close()
            except Exception:
                pass
        
        # Close server sockets
        for sock in self.server_sockets.values():
            try:
                sock.close()
            except Exception:
                pass
        
        # Print final stats
        print(f"\n[Ground Station] 📊 Final Statistics:")
        print(f"  Commands sent: {self.stats.commands_sent}")
        print(f"  Commands failed: {self.stats.commands_failed}")
        print(f"  Success rate: {self.stats.success_rate:.1f}%")
        print(f"  Uptime: {self.stats.uptime:.1f}s")
        print("[Ground Station] Controller closed")
