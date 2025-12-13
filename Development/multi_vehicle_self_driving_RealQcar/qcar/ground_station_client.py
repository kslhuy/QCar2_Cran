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
from config_main import VehicleMainConfig

class GroundStationClient:
    """
    Simplified Ground Station communication handler for high-performance operation.
    Uses single thread for both send and receive operations.
    """
    
    # Performance constants
    SEND_INTERVAL = 0.05  # 50ms = 20Hz telemetry
    RECV_TIMEOUT = 0.1    # 100ms socket timeout
    MAX_QUEUE_SIZE = 50   # Smaller queue for better performance
    RECONNECT_TIMEOUT = 10.0  # 10 seconds to wait for reconnection before shutdown
    RECONNECT_INTERVAL = 1.0  # Try to reconnect every 1 second
    
    def __init__(self, config: VehicleMainConfig, logger: VehicleLogger, kill_event: Event):
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
        
        # Reconnection state
        self._reconnecting = False
        self._reconnect_start_time = 0.0
        
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
        """Initialize network connection to Ground Station with 8-second timeout"""
        #TODO : Handle disabled remote control
        if not self.config.network.is_remote_enabled:
            self.logger.logger.info("Remote control disabled - Ground Station not used")
            return True
        
        self.logger.logger.info(f"Connecting to Ground Station at {self.host_ip}:{self.port}")
        
        # Try to connect with total 8-second timeout
        max_connection_time = 8.0
        connection_start = time.time()
        attempt = 0
        
        while time.time() - connection_start < max_connection_time:
            attempt += 1
            self.logger.logger.info(f"Connection attempt {attempt} to Ground Station...")
            
            if self._connect():
                elapsed = time.time() - connection_start
                self.logger.logger.info(f"Connected to Ground Station successfully in {elapsed:.2f}s")
                return True
            
            # Check if we have time for another attempt
            if time.time() - connection_start < max_connection_time - 1.0:
                time.sleep(0.5)  # Wait before retry
            else:
                break
        
        # Timeout reached
        elapsed = time.time() - connection_start
        self.logger.logger.warning(
            f"Failed to connect to Ground Station after {elapsed:.1f}s ({attempt} attempts) - continuing without connection"
        )
        return True  # Return True to allow vehicle to continue without Ground Station
    
    def _connect(self) -> bool:
        """Connect to Ground Station with simplified logic"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(2.0)  # 2-second timeout per attempt
            self.socket.connect((self.host_ip, self.port))
            self.socket.settimeout(self.RECV_TIMEOUT)  # Set receive timeout
            
            self.connected = True
            return True
            
        except Exception as e:
            self.logger.logger.debug(f"Connection attempt failed: {e}")
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
                
                # Skip communication if reconnecting
                if self._reconnecting:
                    time.sleep(0.1)
                    continue
                
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
        if not self.connected or self._reconnecting:
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
                    # Connection lost - _send_json_message already triggered reconnection
                    return
                
                self.telemetry_queue.task_done()
                
        except queue.Empty:
            pass
        except Exception as e:
            self.logger.log_error("Error sending telemetry", e)
    
    def _receive_commands(self):
        """Receive and queue commands from Ground Station"""
        if not self.connected or self._reconnecting:
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
    
    def _attempt_reconnection(self) -> bool:
        """Attempt to reconnect to Ground Station with timeout"""
        self.logger.logger.info(f"Attempting to reconnect to Ground Station (timeout: {self.RECONNECT_TIMEOUT}s)...")
        
        reconnect_attempts = 0
        while time.time() - self._reconnect_start_time < self.RECONNECT_TIMEOUT:
            if self.kill_event.is_set():
                self.logger.logger.info("Reconnection cancelled - shutdown requested")
                return False
            
            reconnect_attempts += 1
            self.logger.logger.info(f"Reconnection attempt {reconnect_attempts}...")
            
            try:
                # Try to reconnect
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(2.0)  # Short timeout for reconnection attempts
                self.socket.connect((self.host_ip, self.port))
                self.socket.settimeout(self.RECV_TIMEOUT)  # Restore normal timeout
                
                self.connected = True
                self._reconnecting = False
                self._recv_buffer = ""  # Clear receive buffer
                
                self.logger.logger.info(f"Successfully reconnected to Ground Station after {reconnect_attempts} attempts")
                return True
                
            except Exception as e:
                self.logger.logger.debug(f"Reconnection attempt {reconnect_attempts} failed: {e}")
                self._cleanup_socket()
                
                # Wait before next attempt
                time.sleep(self.RECONNECT_INTERVAL)
        
        # Timeout reached
        elapsed = time.time() - self._reconnect_start_time
        self.logger.logger.error(f"Failed to reconnect after {elapsed:.1f}s ({reconnect_attempts} attempts)")
        self._reconnecting = False
        return False
    
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
        """Handle connection loss and attempt reconnection"""
        if self._reconnecting:
            return  # Already handling reconnection
        
        self.logger.log_warning("Ground Station connection lost - attempting reconnection...")
        self._cleanup_socket()
        self.stats['connection_errors'] += 1
        
        # Start reconnection attempt
        self._reconnecting = True
        self._reconnect_start_time = time.time()
        
        # Try to reconnect
        if not self._attempt_reconnection():
            # Reconnection failed - trigger shutdown
            self.logger.logger.error("Failed to reconnect to Ground Station within 8 seconds - shutting down")
            self.kill_event.set()  # Trigger system shutdown
    
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