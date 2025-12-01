"""
Ground Station Client - Simplified high-performance TCP communication
Clean and efficient implementation for Ground Station communication
"""
import socket
import threading
import time
import queue
import json
from typing import Optional, Dict, Any
from threading import Event

from logging_utils import VehicleLogger
from config import VehicleControlConfig


class GroundStationClient:
    """
    Simplified Ground Station communication handler for high-performance operation.
    Uses single thread for both send and receive operations.
    """
    
    # Performance constants
    SEND_INTERVAL = 0.05  # 50ms = 20Hz telemetry
    RECV_TIMEOUT = 0.1    # 100ms socket timeout
    MAX_QUEUE_SIZE = 50   # Smaller queue for better performance
    
    def __init__(self, config: VehicleControlConfig, logger: VehicleLogger, kill_event: Event):
        self.config = config
        self.logger = logger
        self.kill_event = kill_event
        
        # Network configuration
        self.host_ip = config.network.host_ip
        self.port = config.network.port
        self.car_id = config.network.car_id
        self.connection_timeout = getattr(config.network, 'connection_timeout', 10)
        
        # Core state
        self.socket = None
        self.connected = False
        self._running = False
        self.last_send_time = 0.0
        
        # Single thread for communication
        self.comm_thread = None
        
        # Simplified queues
        self.telemetry_queue = queue.Queue(maxsize=self.MAX_QUEUE_SIZE)
        self.command_queue = queue.Queue(maxsize=10)
        
        # Simple statistics
        self.stats = {
            'telemetry_sent': 0,
            'commands_received': 0,
            'connection_errors': 0,
            'queue_overflows': 0
        }
        
        # Receive buffer for message parsing
        self._recv_buffer = ""
    
    def initialize_network(self) -> bool:
        """Initialize network connection to Ground Station"""
        if not self.config.network.is_remote_enabled:
            self.logger.logger.info("Remote control disabled - no network needed")
            return True
        
        self.logger.logger.info(f"Connecting to Ground Station at {self.host_ip}:{self.port}")
        return self._connect()
    
    def _connect(self) -> bool:
        """Connect to Ground Station with simplified logic"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.connection_timeout)
            self.socket.connect((self.host_ip, self.port))
            self.socket.settimeout(self.RECV_TIMEOUT)  # Set receive timeout
            
            self.connected = True
            self.logger.logger.info(f"Connected to Ground Station successfully")
            return True
            
        except Exception as e:
            self.logger.log_error("Ground Station connection failed", e)
            self.stats['connection_errors'] += 1
            self._cleanup_socket()
            return False
    
    def _cleanup_socket(self):
        """Clean up socket connection"""
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
    
    def start_threads(self) -> bool:
        """Start the communication thread"""
        if not self.connected:
            self.logger.logger.info("Not connected - skipping thread start")
            return True
        
        try:
            self._running = True
            self.comm_thread = threading.Thread(
                target=self._communication_loop,
                name=f"GroundStation-{self.car_id}",
                daemon=True
            )
            self.comm_thread.start()
            self.logger.logger.info("Ground Station communication thread started")
            return True
            
        except Exception as e:
            self.logger.log_error("Failed to start communication thread", e)
            return False
    
    def _communication_loop(self):
        """Main communication loop - handles both send and receive"""
        self.logger.logger.info("Ground Station communication loop started")
        
        last_telemetry_time = 0.0
        
        try:
            while self._running and not self.kill_event.is_set():
                current_time = time.time()
                
                # Send telemetry at controlled rate
                if current_time - last_telemetry_time >= self.SEND_INTERVAL:
                    self._send_queued_telemetry()
                    last_telemetry_time = current_time
                
                # Receive commands
                self._receive_commands()
                
                # Brief sleep to prevent CPU spinning
                time.sleep(0.01)  # 10ms
                
        except Exception as e:
            self.logger.log_error("Communication loop error", e)
        finally:
            self.logger.logger.info("Ground Station communication loop stopped")
    
    def _send_queued_telemetry(self):
        """Send all queued telemetry data"""
        if not self.connected:
            return
        
        # Process all queued telemetry
        sent_count = 0
        try:
            while not self.telemetry_queue.empty() and sent_count < 5:  # Limit per cycle
                telemetry_data = self.telemetry_queue.get_nowait()
                
                if self._send_json_message(telemetry_data):
                    self.stats['telemetry_sent'] += 1
                    sent_count += 1
                else:
                    # Connection lost
                    return
                
                self.telemetry_queue.task_done()
                
        except queue.Empty:
            pass
        except Exception as e:
            self.logger.log_error("Error sending telemetry", e)
    
    def _receive_commands(self):
        """Receive and queue commands from Ground Station"""
        if not self.connected:
            return
        
        try:
            # Try to receive data
            data = self.socket.recv(1024)
            if not data:
                # Connection closed
                self._handle_disconnect()
                return
            
            # Add to buffer and parse messages
            self._recv_buffer += data.decode('utf-8')
            self._parse_received_messages()
            
        except socket.timeout:
            # Normal timeout - no data available
            pass
        except Exception as e:
            self.logger.log_error("Error receiving commands", e)
            self._handle_disconnect()
    
    def _parse_received_messages(self):
        """Parse newline-delimited JSON messages from buffer"""
        while '\n' in self._recv_buffer:
            try:
                line, self._recv_buffer = self._recv_buffer.split('\n', 1)
                if line.strip():
                    command = json.loads(line)
                    
                    # Queue command (drop oldest if full)
                    try:
                        self.command_queue.put_nowait(command)
                        self.stats['commands_received'] += 1
                    except queue.Full:
                        self.command_queue.get_nowait()  # Remove oldest
                        self.command_queue.put_nowait(command)
                        self.stats['queue_overflows'] += 1
                        
            except json.JSONDecodeError:
                self.logger.log_warning("Invalid JSON received from Ground Station")
            except Exception as e:
                self.logger.log_error("Error parsing command", e)
    
    def _send_json_message(self, data: dict) -> bool:
        """Send JSON message to Ground Station"""
        try:
            message = json.dumps(data) + '\n'
            self.socket.sendall(message.encode('utf-8'))
            return True
        except Exception as e:
            self.logger.log_warning(f"Send failed: {e}")
            self._handle_disconnect()
            return False
    
    def _handle_disconnect(self):
        """Handle connection loss"""
        self.logger.log_warning("Ground Station connection lost")
        self._cleanup_socket()
        self.stats['connection_errors'] += 1
    
    # Public interface methods
    def queue_telemetry(self, telemetry_data: Dict[str, Any]) -> bool:
        """Queue telemetry data for sending (non-blocking)"""
        if not self._running:
            return False
        
        try:
            self.telemetry_queue.put_nowait(telemetry_data)
            return True
        except queue.Full:
            # Drop oldest and add new
            try:
                self.telemetry_queue.get_nowait()
                self.telemetry_queue.put_nowait(telemetry_data)
                self.stats['queue_overflows'] += 1
                return True
            except queue.Empty:
                return False
    
    def get_latest_commands(self) -> Optional[Dict[str, Any]]:
        """Get latest commands from queue"""
        if not self._running:
            return None
        
        latest_command = None
        try:
            # Get the most recent command(s)
            while True:
                latest_command = self.command_queue.get_nowait()
        except queue.Empty:
            pass
        
        return latest_command
    
    def stop_threads(self):
        """Stop communication thread gracefully"""
        if not self._running:
            return
        
        self.logger.logger.info("Stopping Ground Station communication...")
        self._running = False
        
        # Wait for thread to finish
        if self.comm_thread and self.comm_thread.is_alive():
            self.comm_thread.join(timeout=1.0)
        
        # Clear queues
        while not self.telemetry_queue.empty():
            try:
                self.telemetry_queue.get_nowait()
            except queue.Empty:
                break
        
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
            except queue.Empty:
                break
        
        self.logger.logger.info(f"Ground Station stopped. Stats: {self.get_statistics()}")
    
    def close(self):
        """Close Ground Station connection"""
        self.stop_threads()
        self._cleanup_socket()
        self.logger.logger.info("Ground Station client closed")
    
    def is_connected(self) -> bool:
        """Check if connected to Ground Station"""
        return self.connected
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get simple statistics"""
        return {
            'connected': self.connected,
            'telemetry_sent': self.stats['telemetry_sent'],
            'commands_received': self.stats['commands_received'],
            'connection_errors': self.stats['connection_errors'],
            'queue_overflows': self.stats['queue_overflows'],
            'telemetry_queue_size': self.telemetry_queue.qsize(),
            'command_queue_size': self.command_queue.qsize(),
            'host_ip': self.host_ip,
            'port': self.port,
            'car_id': self.car_id
        }
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()