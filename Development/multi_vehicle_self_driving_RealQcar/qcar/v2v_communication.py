"""
Vehicle-to-Vehicle (V2V) Communication Module
Handles P2P communication between vehicles for cooperative driving
"""
import socket
import threading
import json
import time
import logging
from typing import Dict, List, Optional, Callable
from queue import Queue, Empty
from dataclasses import dataclass, asdict


@dataclass
class V2VMessage:
    """Structure for V2V messages"""
    sender_id: int
    message_type: str
    timestamp: float
    data: dict
    
    def to_dict(self):
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: dict):
        return cls(**data)


class V2VCommunication:
    """Vehicle-to-Vehicle communication handler"""
    
    def __init__(self, vehicle_id: int, logger: logging.Logger, base_port: int = 6000):
        self.vehicle_id = vehicle_id
        self.logger = logger
        self.base_port = base_port
        self.my_port = base_port + vehicle_id
        
        # Communication state
        self.is_active = False
        self.peer_vehicles = []
        self.peer_ips = {}  # {vehicle_id: ip_address}
        self.connections = {}  # {vehicle_id: socket}
        
        # Server socket for incoming connections
        self.server_socket = None
        self.server_thread = None
        
        # Message handling
        self.message_queue = Queue(maxsize=100)
        self.message_handlers = {}
        self.stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'connection_attempts': 0,
            'active_connections': 0
        }
        
        # Threading
        self.running = False
        self.connection_threads = []
        
    def activate(self, peer_vehicles: List[int], peer_ips: List[str]) -> bool:
        """Activate V2V communication with specified peers"""
        try:
            self.logger.info(f"V2V: Activating communication for vehicle {self.vehicle_id}")
            
            # Store peer information
            self.peer_vehicles = peer_vehicles.copy()
            self.peer_ips = {vid: ip for vid, ip in zip(peer_vehicles, peer_ips)}
            
            # Start server for incoming connections
            if not self._start_server():
                self.logger.error("V2V: Failed to start server")
                return False
            
            # Give server time to start
            time.sleep(0.5)
            
            # Connect to peer vehicles
            self._connect_to_peers()
            
            self.is_active = True
            self.running = True
            
            self.logger.info(f"V2V: Communication activated with peers: {peer_vehicles}")
            return True
            
        except Exception as e:
            self.logger.error(f"V2V: Activation failed - {e}")
            return False
    
    def deactivate(self):
        """Deactivate V2V communication"""
        try:
            self.logger.info("V2V: Deactivating communication")
            self.is_active = False
            self.running = False
            
            # Close all peer connections
            for vehicle_id, sock in self.connections.items():
                try:
                    sock.close()
                except:
                    pass
            self.connections.clear()
            
            # Close server socket
            if self.server_socket:
                try:
                    self.server_socket.close()
                except:
                    pass
            
            # Wait for threads to finish
            for thread in self.connection_threads:
                if thread.is_alive():
                    thread.join(timeout=1.0)
            
            if self.server_thread and self.server_thread.is_alive():
                self.server_thread.join(timeout=1.0)
            
            self.connection_threads.clear()
            self.peer_vehicles.clear()
            self.peer_ips.clear()
            
            self.logger.info("V2V: Communication deactivated")
            
        except Exception as e:
            self.logger.error(f"V2V: Deactivation error - {e}")
    
    def _start_server(self) -> bool:
        """Start server to listen for incoming connections"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.my_port))
            self.server_socket.listen(10)
            
            self.server_thread = threading.Thread(target=self._server_loop, daemon=True)
            self.server_thread.start()
            
            self.logger.info(f"V2V: Server listening on port {self.my_port}")
            return True
            
        except Exception as e:
            self.logger.error(f"V2V: Server start failed - {e}")
            return False
    
    def _server_loop(self):
        """Server loop to handle incoming connections"""
        while self.running and self.server_socket:
            try:
                client_socket, addr = self.server_socket.accept()
                self.logger.info(f"V2V: Incoming connection from {addr}")
                
                # Handle connection in separate thread
                thread = threading.Thread(
                    target=self._handle_incoming_connection,
                    args=(client_socket, addr),
                    daemon=True
                )
                thread.start()
                self.connection_threads.append(thread)
                
            except socket.error:
                if self.running:
                    self.logger.error("V2V: Server socket error")
                break
            except Exception as e:
                self.logger.error(f"V2V: Server loop error - {e}")
                break
    
    def _handle_incoming_connection(self, client_socket: socket.socket, addr):
        """Handle incoming connection from peer vehicle"""
        try:
            # Receive identification message
            data = client_socket.recv(1024).decode('utf-8')
            if data:
                msg_data = json.loads(data)
                peer_id = msg_data.get('sender_id')
                
                if peer_id in self.peer_vehicles:
                    self.connections[peer_id] = client_socket
                    self.stats['active_connections'] += 1
                    self.logger.info(f"V2V: Connected to vehicle {peer_id}")
                    
                    # Start message receiving for this connection
                    recv_thread = threading.Thread(
                        target=self._receive_messages,
                        args=(client_socket, peer_id),
                        daemon=True
                    )
                    recv_thread.start()
                    self.connection_threads.append(recv_thread)
                    
                else:
                    self.logger.warning(f"V2V: Unknown vehicle ID {peer_id}")
                    client_socket.close()
            else:
                client_socket.close()
                
        except Exception as e:
            self.logger.error(f"V2V: Incoming connection error - {e}")
            try:
                client_socket.close()
            except:
                pass
    
    def _connect_to_peers(self):
        """Connect to all peer vehicles"""
        for vehicle_id in self.peer_vehicles:
            if vehicle_id != self.vehicle_id:
                thread = threading.Thread(
                    target=self._connect_to_peer,
                    args=(vehicle_id,),
                    daemon=True
                )
                thread.start()
                self.connection_threads.append(thread)
    
    def _connect_to_peer(self, peer_id: int):
        """Connect to a specific peer vehicle"""
        if peer_id not in self.peer_ips:
            return
        
        peer_ip = self.peer_ips[peer_id]
        peer_port = self.base_port + peer_id
        
        max_attempts = 5
        attempt = 0
        
        while attempt < max_attempts and self.running:
            try:
                self.stats['connection_attempts'] += 1
                
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(3.0)
                sock.connect((peer_ip, peer_port))
                
                # Send identification
                ident_msg = V2VMessage(
                    sender_id=self.vehicle_id,
                    message_type='identification',
                    timestamp=time.time(),
                    data={}
                )
                
                sock.send(json.dumps(ident_msg.to_dict()).encode('utf-8'))
                
                self.connections[peer_id] = sock
                self.stats['active_connections'] += 1
                self.logger.info(f"V2V: Connected to vehicle {peer_id} at {peer_ip}:{peer_port}")
                
                # Start message receiving
                recv_thread = threading.Thread(
                    target=self._receive_messages,
                    args=(sock, peer_id),
                    daemon=True
                )
                recv_thread.start()
                self.connection_threads.append(recv_thread)
                break
                
            except Exception as e:
                attempt += 1
                if attempt < max_attempts:
                    self.logger.warning(f"V2V: Connection attempt {attempt} to vehicle {peer_id} failed, retrying...")
                    time.sleep(1.0 * attempt)  # Exponential backoff
                else:
                    self.logger.error(f"V2V: Failed to connect to vehicle {peer_id} after {max_attempts} attempts")
    
    def _receive_messages(self, sock: socket.socket, peer_id: int):
        """Receive messages from a peer vehicle"""
        try:
            while self.running:
                data = sock.recv(4096)
                if not data:
                    break
                
                try:
                    msg_data = json.loads(data.decode('utf-8'))
                    message = V2VMessage.from_dict(msg_data)
                    
                    self.stats['messages_received'] += 1
                    
                    # Queue message for processing
                    if not self.message_queue.full():
                        self.message_queue.put(message)
                    
                    # Call registered handlers
                    handler = self.message_handlers.get(message.message_type)
                    if handler:
                        handler(message)
                        
                except json.JSONDecodeError:
                    self.logger.warning(f"V2V: Invalid JSON from vehicle {peer_id}")
                except Exception as e:
                    self.logger.error(f"V2V: Message processing error - {e}")
                    
        except Exception as e:
            self.logger.error(f"V2V: Receive error from vehicle {peer_id} - {e}")
        finally:
            # Clean up connection
            if peer_id in self.connections:
                try:
                    self.connections[peer_id].close()
                except:
                    pass
                del self.connections[peer_id]
                self.stats['active_connections'] -= 1
    
    def send_message(self, message_type: str, data: dict, target_vehicles: Optional[List[int]] = None) -> bool:
        """Send message to peer vehicles"""
        if not self.is_active:
            return False
        
        message = V2VMessage(
            sender_id=self.vehicle_id,
            message_type=message_type,
            timestamp=time.time(),
            data=data
        )
        
        targets = target_vehicles if target_vehicles else list(self.connections.keys())
        success_count = 0
        
        for target_id in targets:
            if target_id in self.connections:
                try:
                    sock = self.connections[target_id]
                    msg_json = json.dumps(message.to_dict()).encode('utf-8')
                    sock.send(msg_json)
                    success_count += 1
                    self.stats['messages_sent'] += 1
                    
                except Exception as e:
                    self.logger.error(f"V2V: Send error to vehicle {target_id} - {e}")
        
        return success_count > 0
    
    def register_message_handler(self, message_type: str, handler: Callable[[V2VMessage], None]):
        """Register a handler for specific message types"""
        self.message_handlers[message_type] = handler
    
    def get_messages(self, max_count: int = 10) -> List[V2VMessage]:
        """Get received messages from queue"""
        messages = []
        for _ in range(max_count):
            try:
                msg = self.message_queue.get_nowait()
                messages.append(msg)
            except Empty:
                break
        return messages
    
    def get_connected_peers(self) -> List[int]:
        """Get list of currently connected peer vehicles"""
        return list(self.connections.keys())
    
    def get_statistics(self) -> dict:
        """Get communication statistics"""
        stats = self.stats.copy()
        stats['is_active'] = self.is_active
        stats['connected_peers'] = len(self.connections)
        stats['peer_vehicles'] = self.peer_vehicles.copy()
        return stats
    
    def send_telemetry(self, vehicle_state: dict):
        """Send vehicle telemetry to all connected peers"""
        if self.is_active:
            self.send_message('telemetry', {
                'position': [vehicle_state.get('x', 0), vehicle_state.get('y', 0)],
                'heading': vehicle_state.get('theta', 0),
                'velocity': vehicle_state.get('velocity', 0),
                'timestamp': time.time()
            })
    
    def send_intent(self, intention: str, parameters: dict):
        """Send driving intent to other vehicles"""
        if self.is_active:
            self.send_message('intent', {
                'intention': intention,
                'parameters': parameters,
                'timestamp': time.time()
            })
    
    def send_warning(self, warning_type: str, urgency: str, data: dict):
        """Send warning message to other vehicles"""
        if self.is_active:
            self.send_message('warning', {
                'warning_type': warning_type,
                'urgency': urgency,
                'data': data,
                'timestamp': time.time()
            })