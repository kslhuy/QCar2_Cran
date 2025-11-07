"""
Simplified and refactored CommHandler for vehicle fleet communication.

This version removes unnecessary complexity and focuses on core functionality:
- Fast UDP communication without ACKs
- Single socket setup method
- Streamlined message processing
- Removed unused features (batching, topology, performance monitoring)
"""

import socket
import json as ujson
import time
import threading
import random
from typing import Dict, Any, Tuple, Optional

# Import logging configuration
from md_logging_config import get_fleet_observer_logger


class CommHandler:
    """
    Simplified UDP communication handler for vehicle fleet coordination.
    
    Features:
    - Fast UDP communication without ACK overhead
    - Non-blocking message processing
    - Simple state and fleet estimate broadcasting
    - Clean message routing
    """
    
    # Communication timing constants - OPTIMIZED
    SEND_INTERVAL = 0.015           # ~67Hz send rate
    RECEIVE_TIMEOUT = 0.005         # 5ms socket timeout for fast polling
    
    def __init__(self, vehicle_id: int, target_ip: str, send_port: int, recv_port: int, 
                 logger, vehicle=None):
        """
        Initialize simplified communication handler.
        
        Args:
            vehicle_id: Unique identifier for this vehicle
            target_ip: IP address for broadcasting messages
            send_port: Port for sending messages
            recv_port: Port for receiving messages
            logger: Logger instance
            vehicle: Reference to Vehicle instance
        """
        self.vehicle_id = vehicle_id
        self.target_ip = target_ip
        self.send_port = send_port
        self.recv_port = recv_port
        self.logger = logger
        self.vehicle = vehicle
        
        # Fleet observer logger for fleet estimates
        self.fleet_comm_logger = get_fleet_observer_logger(vehicle_id)
        
        # Communication state
        self.sequence_number = 0
        self.last_send_time = 0
        self.send_jitter = random.uniform(0.0005, 0.002)  # Small jitter to prevent sync
        self.actual_send_interval = self.SEND_INTERVAL + self.send_jitter
        
        # Thread safety
        self.lock = threading.Lock()
        self.running = threading.Event()
        
        # Statistics
        self.send_count = 0
        self.recv_count = 0
        
        # Setup sockets
        self._setup_sockets()
        
        self.logger.info(f"CommHandler initialized for Vehicle {self.vehicle_id}, "
                        f"send_interval={self.actual_send_interval:.4f}s")
    
    def _setup_sockets(self):
        """Initialize UDP sockets for communication."""
        try:
            # Send socket - OPTIMIZED for fast, small messages
            self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)  # 8KB - smaller, faster
            self.send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcast
            
            # Receive socket - OPTIMIZED for fast reception
            self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16384)  # 16KB - optimized size
            self.recv_sock.bind(('0.0.0.0', self.recv_port))
            self.recv_sock.settimeout(self.RECEIVE_TIMEOUT)
            
            self.logger.info(f"Sockets configured - recv_port: {self.recv_port}")
            
        except Exception as e:
            self.logger.error(f"Socket setup failed: {e}")
            raise
    
    def start_communication(self):
        """Start communication threads."""
        self.running.set()
        
        # Start receive thread
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
        
        self.logger.info(f"Communication started for Vehicle {self.vehicle_id}")
    
    def stop_communication(self):
        """Stop communication threads."""
        self.running.clear()
        if hasattr(self, 'receive_thread'):
            self.receive_thread.join(timeout=1.0)
        
        self.logger.info(f"Communication stopped for Vehicle {self.vehicle_id}")
    
    def send_state_broadcast(self, state_data: dict) -> bool:
        """
        Send vehicle state to other vehicles.
        
        Args:
            state_data: Vehicle state dictionary
            
        Returns:
            True if sent successfully
        """
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_send_time < self.actual_send_interval:
            return False
        
        try:
            # Create message
            message = {
                'type': 'state',
                'id': self.vehicle_id,
                'seq': self.sequence_number,
                'timestamp': current_time,
                'pos': state_data.get('pos', [0, 0, 0]),
                'rot': state_data.get('rot', [0, 0, 0]),
                'v': state_data.get('vel', 0.0),
                'ctrl_u': state_data.get('ctrl_u', [0.0, 0.0])
            }
            
            # Send message
            message_json = ujson.dumps(message).encode()
            self.send_sock.sendto(message_json, (self.target_ip, self.send_port))
            
            # Update state
            with self.lock:
                self.sequence_number += 1
                self.send_count += 1
            
            self.last_send_time = current_time
            
            self.logger.debug(f"State sent: Seq={self.sequence_number-1}, "
                            f"Pos={message['pos'][:2]}, V={message['v']:.2f}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send state: {e}")
            return False
    
    def send_fleet_estimates_broadcast(self, fleet_message: dict) -> bool:
        """
        Send fleet estimates to other vehicles.
        
        Args:
            fleet_message: Fleet estimates dictionary
            
        Returns:
            True if sent successfully
        """
        try:
            # Ensure message has correct structure
            fleet_message['type'] = 'fleet_estimates'
            fleet_message['sender_id'] = self.vehicle_id
            fleet_message['seq'] = self.sequence_number
            
            # Send message
            message_json = ujson.dumps(fleet_message).encode()
            self.send_sock.sendto(message_json, (self.target_ip, self.send_port))
            
            # Update sequence
            with self.lock:
                self.sequence_number += 1
            
            # Log fleet estimates
            estimates = fleet_message.get('estimates', {})
            est_summary = {}
            for vid, est in estimates.items():
                pos = est.get('pos', [0, 0])
                vel = est.get('vel', 0.0)
                est_summary[f'V{vid}'] = f'({pos[0]:.1f},{pos[1]:.1f},v={vel:.2f})'
            
            self.fleet_comm_logger.info(f"SEND Fleet Est: {est_summary}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send fleet estimates: {e}")
            return False
    
    def _receive_loop(self):
        """Main message reception loop."""
        self.logger.info(f"Starting receive loop for Vehicle {self.vehicle_id}")
        
        while self.running.is_set():
            try:
                # Receive message with timeout
                data, sender_addr = self.recv_sock.recvfrom(1024)
                self._process_message(data, sender_addr)
                
            except socket.timeout:
                # Expected for responsive shutdown
                continue
            except Exception as e:
                self.logger.error(f"Receive error: {e}")
                time.sleep(0.01)  # Brief pause on error
        
        self.logger.info(f"Receive loop terminated for Vehicle {self.vehicle_id}")
    
    def _process_message(self, data: bytes, sender_addr: Tuple[str, int]):
        """
        Process received message.
        
        Args:
            data: Raw message bytes
            sender_addr: Sender address tuple (IP, port)
        """
        try:
            # Decode message
            message = ujson.loads(data.decode())
            
            # Get message info
            msg_type = message.get('type', '')
            sender_id = message.get('id')
            
            # Ignore our own messages
            if sender_id == self.vehicle_id:
                return
            
            # Route to appropriate handler
            if msg_type == 'state':
                self._handle_state_message(message, sender_addr)
            elif msg_type == 'fleet_estimates':
                self._handle_fleet_estimates(message, sender_addr)
            elif msg_type == 'heartbeat':
                self._handle_heartbeat(message)
            else:
                self.logger.warning(f"Unknown message type: {msg_type} from {sender_addr}")
                
        except Exception as e:
            self.logger.error(f"Message processing error: {e}")
    
    def _handle_state_message(self, message: Dict[str, Any], sender_addr: Tuple[str, int]):
        """Handle received state message."""
        sender_id = message.get('id')
        seq = message.get('seq', -1)
        
        # Log detailed data for vehicle 1
        if self.vehicle_id == 1:
            receive_time = time.time()
            pos = message.get('pos', [0, 0, 0])
            vel = message.get('v', 0.0)
            msg_timestamp = message.get('timestamp', 0.0)
            delay = receive_time - msg_timestamp if msg_timestamp > 0 else 0.0
            
            self.logger.info(f"STATE_RECV: From=V{sender_id} Seq={seq} "
                           f"Pos=({pos[0]:.3f},{pos[1]:.3f}) Vel={vel:.3f} "
                           f"Delay={delay*1000:.1f}ms")
        
        # Process through vehicle
        try:
            self.vehicle.process_received_state(message)
            self.recv_count += 1
        except Exception as e:
            self.logger.error(f"Error processing state from V{sender_id}: {e}")
    
    def _handle_fleet_estimates(self, message: Dict[str, Any], sender_addr: Tuple[str, int]):
        """Handle received fleet estimates."""
        sender_id = message.get('sender_id', message.get('id'))
        seq = message.get('seq', -1)
        timestamp = message.get('timestamp', 0.0)
        
        # Log fleet estimates
        estimates = message.get('estimates', {})
        est_summary = {}
        for vid, est in estimates.items():
            pos = est.get('pos', [0, 0])
            vel = est.get('vel', 0.0)
            est_summary[f'V{vid}'] = f'({pos[0]:.1f},{pos[1]:.1f},v={vel:.2f})'
        
        self.fleet_comm_logger.info(f"RECV Fleet Est: From=V{sender_id} {est_summary}")
        
        # Process through vehicle
        try:
            if hasattr(self.vehicle, 'process_received_fleet_estimates'):
                self.vehicle.process_received_fleet_estimates(message)
        except Exception as e:
            self.logger.error(f"Error processing fleet estimates from V{sender_id}: {e}")
    
    def _handle_heartbeat(self, message: Dict[str, Any]):
        """Handle heartbeat message."""
        sender_id = message.get('id')
        self.logger.debug(f"Heartbeat from V{sender_id}")
    
    def get_stats(self) -> Dict[str, Any]:
        """Get communication statistics."""
        return {
            'vehicle_id': self.vehicle_id,
            'send_count': self.send_count,
            'recv_count': self.recv_count,
            'sequence_number': self.sequence_number,
            'send_rate': self.send_count / max(time.time() - getattr(self, 'start_time', time.time()), 1),
            'is_running': self.running.is_set()
        }
    
    def cleanup(self):
        """Clean up resources."""
        self.stop_communication()
        
        try:
            if hasattr(self, 'send_sock'):
                self.send_sock.close()
            if hasattr(self, 'recv_sock'):
                self.recv_sock.close()
        except Exception as e:
            self.logger.error(f"Socket cleanup error: {e}")
        
        self.logger.info(f"CommHandler cleaned up for Vehicle {self.vehicle_id}")