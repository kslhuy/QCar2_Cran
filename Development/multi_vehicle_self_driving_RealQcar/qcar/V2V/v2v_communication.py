"""
Vehicle-to-Vehicle (V2V) Communication Module
High-performance UDP-based communication system for cooperative driving
"""
import socket
import threading
import time
import logging
from typing import Dict, List, Optional, Callable, Set
from queue import Queue, Empty
from dataclasses import dataclass, asdict
from contextlib import contextmanager
from enum import Enum

import json


class MessageType(Enum):
    """Predefined message types for V2V communication"""
    IDENTIFICATION = "identification"
    TELEMETRY = "telemetry"
    INTENT = "intent"
    WARNING = "warning"
    HEARTBEAT = "heartbeat"


@dataclass
class V2VMessage:
    """Structure for V2V messages"""
    sender_id: int
    message_type: str
    data: dict
    seq_id: int = 0  # Sequence ID for packet tracking
    send_time_ns: int = 0  # Send time in nanoseconds for precise latency measurement
    
    def to_dict(self) -> dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: dict) -> 'V2VMessage':
        # Include seq_id and send_time_ns if present
        valid_fields = {'sender_id', 'message_type', 'data', 'seq_id', 'send_time_ns'}
        filtered_data = {k: v for k, v in data.items() if k in valid_fields}
        return cls(**filtered_data)


class ConnectionState(Enum):
    """Connection states for peer vehicles"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    FAILED = "failed"


class V2VCommunication:
    """High-performance UDP-based Vehicle-to-Vehicle communication handler"""
    
    # Performance constants
    RECV_TIMEOUT = 0.01   # 10ms timeout for non-blocking receive
    MAX_PACKET_SIZE = 1024
    
    # Default send intervals per message type (in nanoseconds)
    DEFAULT_SEND_INTERVALS = {
        'local_state': 50_000_000,    # 50ms = 20 Hz - high frequency for position updates
        'fleet_state': 200_000_000,   # 200ms = 5 Hz - medium frequency for fleet consensus
        'heartbeat': 1_000_000_000,   # 1000ms = 1 Hz - low frequency for health checks
        'telemetry': 50_000_000,      # 50ms = 20 Hz - high frequency (legacy)
        'intent': 100_000_000,        # 100ms = 10 Hz - medium frequency for intentions
        'warning': 50_000_000,        # 50ms = 20 Hz - high frequency for safety
    }
    
    def __init__(self, vehicle_id: int, logger: logging.Logger, 
                 base_port: int = 7000, status_callback: Optional[Callable] = None,
                 send_intervals: Optional[Dict[str, float]] = None):
        self.vehicle_id = vehicle_id
        self.logger = logger
        self.base_port = base_port
        self.my_port = base_port + vehicle_id
        self.status_callback = status_callback
        
        # Configure send intervals per message type
        self.send_intervals = self.DEFAULT_SEND_INTERVALS.copy()
        if send_intervals:
            self.send_intervals.update(send_intervals)
        
        # Core state
        self._running = False
        self._is_active = False
        self._lock = threading.RLock()
        
        # Peer management
        self.peer_vehicles: List[int] = []
        self.peer_ips: Dict[int, str] = {}
        self.peer_ports: Dict[int, int] = {}  # UDP ports for each peer
        self.connection_states: Dict[int, ConnectionState] = {}
        
        # UDP Networking (simplified)
        self.send_socket: Optional[socket.socket] = None
        self.recv_socket: Optional[socket.socket] = None
        self.recv_thread: Optional[threading.Thread] = None
        
        # Performance tracking (in nanoseconds)
        self.last_send_time_ns = 0
        self.sequence_number = 0
        
        # Per-message-type rate limiting - initialize with all possible types (in nanoseconds)
        self.message_send_times_ns = {msg_type: 0 for msg_type in self.send_intervals.keys()}
        
        
        # Message handling
        self.message_queue = Queue(maxsize=200)  # Larger queue for high frequency
        self.message_handlers: Dict[str, Callable[[V2VMessage], None]] = {}
        self.stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'packets_dropped': 0,
            'send_rate': 0.0
        }
        
        # Simplified thread management (only 1 receive thread)
        self._recv_thread_active = False
        
    @property
    def is_active(self) -> bool:
        """Check if V2V communication is active"""
        return self._is_active
    
    def activate(self, peer_vehicles: List[int], peer_ips: List[str]) -> bool:
        """Activate high-performance UDP V2V communication"""
        with self._lock:
            if self._is_active:
                self.logger.warning(f"V2V already active for vehicle {self.vehicle_id}")
                return True
            
            try:
                # Validate input parameters
                if len(peer_vehicles) != len(peer_ips):
                    self.logger.error(f"Mismatch: {len(peer_vehicles)} peer vehicles but {len(peer_ips)} IP addresses")
                    return False
                
                fleet_size = len(peer_vehicles) + 1  # peers + self
                self.logger.info(f"Activating UDP V2V for vehicle {self.vehicle_id} with {len(peer_vehicles)} peers (fleet size: {fleet_size})")
                self.logger.info(f"Peer vehicles: {peer_vehicles}")
                self.logger.info(f"Peer IPs: {peer_ips}")
                
                # Initialize peer information
                self.peer_vehicles = [pid for pid in peer_vehicles if pid != self.vehicle_id]
                self.peer_ips = {pid: ip for pid, ip in zip(peer_vehicles, peer_ips) if pid != self.vehicle_id}
                self.peer_ports = {pid: self.base_port + pid for pid in self.peer_vehicles}
                
                # Initialize connection states (all start as connected for UDP)
                for peer_id in self.peer_vehicles:
                    self.connection_states[peer_id] = ConnectionState.CONNECTED
                
                # Setup UDP sockets
                if not self._setup_udp_sockets():
                    return False
                
                self._running = True
                self._is_active = True
                
                # Start single receive thread
                self._start_receive_thread()
                
                self.logger.info(f"UDP V2V activated successfully for fleet of {fleet_size} vehicles at 20Hz")
                return True
                
            except Exception as e:
                self.logger.error(f"UDP V2V activation failed: {e}")
                self._cleanup()
                return False
    
    def deactivate(self):
        """Cleanly deactivate V2V communication"""
        with self._lock:
            if not self._is_active:
                return
            
            self.logger.info(f"Deactivating V2V for vehicle {self.vehicle_id}")
            
            self._running = False
            self._is_active = False
            
            self._cleanup()
            self.logger.info("V2V deactivated successfully")
    
    def _setup_udp_sockets(self) -> bool:
        """Setup optimized UDP sockets for high-performance communication"""
        try:
            # Send socket with broadcast capability
            self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 16384)  # 16KB send buffer
            
            # Receive socket with optimized buffer
            self.recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32768)  # 32KB receive buffer
            self.recv_socket.bind(('0.0.0.0', self.my_port))
            self.recv_socket.settimeout(self.RECV_TIMEOUT)
            
            self.logger.info(f"UDP sockets setup - port {self.my_port}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to setup UDP sockets: {e}")
            return False
    
    def _start_receive_thread(self):
        """Start single optimized UDP receive thread"""
        self.recv_thread = threading.Thread(
            target=self._udp_receive_loop,
            name=f"V2V-UDP-Recv-{self.vehicle_id}",
            daemon=True
        )
        self._recv_thread_active = True
        self.recv_thread.start()
    
    def _udp_receive_loop(self):
        """High-performance UDP receive loop"""
        while self._running and self._recv_thread_active:
            try:
                data, addr = self.recv_socket.recvfrom(self.MAX_PACKET_SIZE)
                if data and self._running:
                    self._process_udp_message(data, addr)
                    
            except socket.timeout:
                continue  # Normal timeout, keep listening
            except socket.error as e:
                if self._running:
                    self.logger.debug(f"UDP receive error: {e}")
                continue
            except Exception as e:
                if self._running:
                    self.logger.error(f"UDP receive loop error: {e}")
                break
    
    def _process_udp_message(self, data: bytes, addr):
        """Process incoming UDP message with fast JSON parsing"""
        try:
            # Fast JSON decode
            msg_data = json.loads(data.decode('utf-8'))
            sender_id = msg_data.get('sender_id')
            
            # Quick validation
            if sender_id == self.vehicle_id or sender_id not in self.peer_vehicles:
                return
            
            # Create message object
            message = V2VMessage.from_dict(msg_data)
            self.stats['messages_received'] += 1
            
            # Queue for external processing (non-blocking)
            try:
                self.message_queue.put_nowait(message)
            except:
                # Queue full, drop oldest and add new
                try:
                    self.message_queue.get_nowait()
                    self.message_queue.put_nowait(message)
                    self.stats['packets_dropped'] += 1
                except:
                    pass
            
            # Call registered handler if available
            handler = self.message_handlers.get(message.message_type)
            if handler:
                try:
                    handler(message)
                except Exception as e:
                    self.logger.error(f"Handler error for {message.message_type}: {e}")
                    
        except json.JSONDecodeError:
            self.logger.debug(f"Invalid JSON from {addr}")
        except Exception as e:
            self.logger.error(f"UDP message processing error: {e}")
    
    # UDP-specific helper methods
    def send_high_frequency_telemetry(self, vehicle_state: dict) -> bool:
        """Optimized method for high-frequency telemetry sending"""
        if not self._is_active:
            return False
        
        # Only send if enough time has passed (rate limiting using nanoseconds)
        current_time_ns = time.time_ns()
        telemetry_interval_ns = self.send_intervals.get('telemetry', 50_000_000)  # 50ms default
        last_telemetry_time_ns = self.message_send_times_ns.get('telemetry', 0)
        if current_time_ns - last_telemetry_time_ns < telemetry_interval_ns:
            return False
        
        # Create comprehensive telemetry message with all vehicle state data
        telemetry_data = {
            'position': [vehicle_state.get('x', 0), vehicle_state.get('y', 0)],
            'heading': vehicle_state.get('theta', 0),
            'velocity': vehicle_state.get('velocity', 0),
            'state': vehicle_state.get('state', 'UNKNOWN'),
        }
        
        return self.send_message(MessageType.TELEMETRY.value, telemetry_data)
    
    def _cleanup(self):
        """Clean up UDP resources"""
        # Stop receive thread
        self._recv_thread_active = False
        
        # Close UDP sockets
        if self.send_socket:
            try:
                self.send_socket.close()
            except:
                pass
            self.send_socket = None
        
        if self.recv_socket:
            try:
                self.recv_socket.close()
            except:
                pass
            self.recv_socket = None
        
        # Wait for receive thread to finish
        if self.recv_thread and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=0.5)
        
        # Reset state
        self.peer_vehicles.clear()
        self.peer_ips.clear()
        self.peer_ports.clear()
        self.connection_states.clear()
        self.sequence_number = 0
        self.last_send_time_ns = 0
        # Reset per-message-type tracking
        for msg_type in self.message_send_times_ns:
            self.message_send_times_ns[msg_type] = 0
    
    def _notify_status_change(self, event: str, peer_id: int):
        """Notify status callback of connection changes"""
        if self.status_callback:
            try:
                self.status_callback(event, peer_id)
            except Exception as e:
                self.logger.error(f"Status callback error: {e}")
    
    def send_message(self, message_type: str, data: dict, 
                    target_vehicles: Optional[List[int]] = None) -> bool:
        """Send message via optimized UDP broadcast"""
        if not self._is_active or not self.send_socket:
            return False
        
        # Per-message-type rate limiting using nanoseconds for better performance
        send_time_ns = time.time_ns()
        
        # Get interval for this message type in nanoseconds (default to 50ms if not configured)
        message_interval_ns = self.send_intervals.get(message_type, 50_000_000)
        
        # Check rate limit for this specific message type
        last_send_for_type_ns = self.message_send_times_ns.get(message_type, 0)
        if send_time_ns - last_send_for_type_ns < message_interval_ns:
            return False  # Skip this send to maintain per-type rate limit
        
        message = V2VMessage(
            sender_id=self.vehicle_id,
            message_type=message_type,
            data=data,
            seq_id=self.sequence_number,
            send_time_ns=send_time_ns
        )
        
        # Convert to dict (already includes seq_id and send_time_ns)
        msg_dict = message.to_dict()
        
        try:
            # Fast JSON encode
            msg_bytes = json.dumps(msg_dict).encode('utf-8')
            
            if len(msg_bytes) > self.MAX_PACKET_SIZE:
                self.logger.warning(f"Message too large: {len(msg_bytes)} bytes")
                return False
            
            # Send to specific targets or broadcast
            targets = target_vehicles or self.peer_vehicles
            success_count = 0
            
            for target_id in targets:
                if target_id in self.peer_ips:
                    target_ip = self.peer_ips[target_id]
                    target_port = self.peer_ports[target_id]
                    try:
                        self.send_socket.sendto(msg_bytes, (target_ip, target_port))
                        success_count += 1
                    except Exception as e:
                        self.logger.debug(f"Send failed to {target_id}: {e}")
            
            if success_count > 0:
                self.stats['messages_sent'] += 1
                self.sequence_number += 1
                self.last_send_time_ns = send_time_ns
                # Update per-message-type send time
                self.message_send_times_ns[message_type] = send_time_ns
                
                # Update send rate statistics (convert to Hz)
                if self.stats['messages_sent'] % 20 == 0:  # Every 20 messages
                    time_diff_s = (send_time_ns - (self.last_send_time_ns - int(1e9))) / 1e9
                    self.stats['send_rate'] = 20.0 / max(0.001, time_diff_s)
            
            return success_count > 0
            
        except Exception as e:
            self.logger.error(f"UDP send error: {e}")
            return False
    
    def register_message_handler(self, message_type: str, handler: Callable[[V2VMessage], None]):
        """Register a handler for specific message types"""
        self.message_handlers[message_type] = handler
    
    def get_messages(self, max_count: int = 10) -> List[V2VMessage]:
        """Get received messages from queue"""
        messages = []
        for _ in range(max_count):
            try:
                messages.append(self.message_queue.get_nowait())
            except Empty:
                break
        return messages
    
    def get_connected_peers(self) -> List[int]:
        """Get list of peer vehicles (UDP assumes all are reachable)"""
        with self._lock:
            return self.peer_vehicles.copy()
    
    def is_fully_connected(self) -> bool:
        """Check if UDP communication is active (no connection state needed)"""
        with self._lock:
            return self._is_active and len(self.peer_vehicles) > 0
    
    def get_statistics(self) -> dict:
        """Get comprehensive UDP communication statistics"""
        with self._lock:
            stats = self.stats.copy()
            stats.update({
                'is_active': self._is_active,
                'peer_count': len(self.peer_vehicles),
                'expected_peers': len(self.peer_vehicles),
                'is_fully_connected': self.is_fully_connected(),
                'sequence_number': self.sequence_number,
                'send_intervals': self.send_intervals.copy(),
                'actual_rate_hz': self.stats.get('send_rate', 0.0),
            })
        return stats
    
    def get_connection_summary(self) -> str:
        """Get human-readable UDP communication summary"""
        if not self._is_active:
            return "V2V-UDP: Inactive"
        
        peer_count = len(self.peer_vehicles)
        rate = self.stats.get('send_rate', 0.0)
        
        if peer_count > 0:
            return f"V2V-UDP: Active ({peer_count} peers) @ {rate:.1f}Hz"
        else:
            return "V2V-UDP: Active (no peers)"
    
    # High-performance convenience methods for common message types
    def send_telemetry(self, vehicle_state: dict) -> bool:
        """Send optimized vehicle telemetry via UDP"""
        # Use the high-frequency optimized method
        return self.send_high_frequency_telemetry(vehicle_state)
    
    def send_intent(self, intention: str, parameters: dict) -> bool:
        """Send driving intent to other vehicles via UDP"""
        return self.send_message(MessageType.INTENT.value, {
            'intention': intention,
            'parameters': parameters
        })
    
    def send_warning(self, warning_type: str, urgency: str, data: dict) -> bool:
        """Send warning message to other vehicles via UDP"""
        return self.send_message(MessageType.WARNING.value, {
            'warning_type': warning_type,
            'urgency': urgency,
            'data': data
        })