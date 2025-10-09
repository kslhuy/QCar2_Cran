"""
Enhanced Network Client with error handling and retry logic
"""
import socket
import json
import time
from typing import Optional, Dict, Any
from threading import Lock


class NetworkClient:
    """Handles communication with the host PC with robust error handling"""
    
    def __init__(self, host_ip: str, port: int, car_id: int, logger=None, config=None):
        self.host_ip = host_ip
        self.port = port
        self.car_id = car_id
        self.logger = logger
        
        # Configuration
        self.connection_timeout = config.network.connection_timeout if config else 5
        self.max_reconnect_attempts = config.network.max_reconnect_attempts if config else 10
        self.reconnect_delay = config.network.reconnect_delay if config else 2.0
        
        # Connection state
        self.socket = None
        self.connected = False
        self.reconnect_attempts = 0
        self.last_send_time = 0
        self.last_receive_time = 0
        
        # Thread safety
        self._send_lock = Lock()
        self._receive_lock = Lock()
        
        # Statistics
        self.bytes_sent = 0
        self.bytes_received = 0
        self.messages_sent = 0
        self.messages_received = 0
        self.connection_errors = 0
    
    def connect(self, timeout: Optional[int] = None) -> bool:
        """
        Connect to the host PC with retry logic
        
        Args:
            timeout: Connection timeout in seconds (uses config default if None)
            
        Returns:
            True if connection successful
        """
        if timeout is None:
            timeout = self.connection_timeout
        
        if self.logger:
            self.logger.log_network_event(
                "connection_attempt",
                {"host": self.host_ip, "port": self.port}
            )
        
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(timeout)
            self.socket.connect((self.host_ip, self.port))
            
            # Set socket to non-blocking for receive operations
            self.socket.setblocking(False)
            
            self.connected = True
            self.reconnect_attempts = 0
            
            if self.logger:
                self.logger.log_network_event("connected", {"port": self.port})
            
            return True
            
        except socket.timeout:
            if self.logger:
                self.logger.log_warning(f"Connection timeout to {self.host_ip}:{self.port}")
            self._handle_connection_failure()
            return False
            
        except ConnectionRefusedError:
            if self.logger:
                self.logger.log_warning(f"Connection refused by {self.host_ip}:{self.port}")
            self._handle_connection_failure()
            return False
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Connection failed", e)
            self._handle_connection_failure()
            return False
    
    def _handle_connection_failure(self):
        """Handle connection failure"""
        self.connected = False
        self.connection_errors += 1
        self.reconnect_attempts += 1
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
    
    def reconnect(self) -> bool:
        """Attempt to reconnect to host PC"""
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            if self.logger:
                self.logger.log_error(
                    f"Max reconnection attempts ({self.max_reconnect_attempts}) reached"
                )
            return False
        
        if self.logger:
            self.logger.log_network_event(
                "reconnecting",
                {"attempt": self.reconnect_attempts + 1, "max": self.max_reconnect_attempts}
            )
        
        time.sleep(self.reconnect_delay)
        return self.connect()
    
    def send_telemetry(self, data: Dict[str, Any]) -> bool:
        """
        Send telemetry data to host PC with retry logic
        
        Args:
            data: Dictionary containing telemetry data
            
        Returns:
            True if send was successful
        """
        if not self.connected:
            return False
        
        max_retries = 3
        
        for attempt in range(max_retries):
            try:
                with self._send_lock:
                    # Serialize to JSON with newline (matches remote_controller.py protocol)
                    message_json = json.dumps(data)
                    message_str = message_json + '\n'
                    message_bytes = message_str.encode('utf-8')
                    
                    # Send message (newline-delimited JSON)
                    self.socket.sendall(message_bytes)
                    
                    # Update statistics
                    self.bytes_sent += len(message_bytes)
                    self.messages_sent += 1
                    self.last_send_time = time.time()
                    
                    return True
                    
            except socket.timeout:
                if attempt < max_retries - 1:
                    time.sleep(0.01)
                    continue
                if self.logger:
                    self.logger.log_warning("Send timeout")
                self._handle_connection_loss()
                return False
                
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                if self.logger:
                    self.logger.log_warning(f"Connection lost during send: {e}")
                self._handle_connection_loss()
                return False
                
            except Exception as e:
                if self.logger:
                    self.logger.log_error("Send failed", e)
                return False
        
        return False
    
    def receive_commands(self) -> Optional[Dict[str, Any]]:
        """
        Receive commands from host PC (non-blocking)
        
        Returns:
            Dictionary containing command data, or None if no data available
        """
        if not self.connected:
            return None
        
        # Keep a buffer for partial messages
        if not hasattr(self, '_recv_buffer'):
            self._recv_buffer = ""
        
        try:
            with self._receive_lock:
                # Try to receive data (non-blocking)
                chunk = self.socket.recv(1024).decode('utf-8')
                
                if not chunk:
                    # Connection closed by remote
                    self._handle_connection_loss()
                    return None
                
                # Add to buffer
                self._recv_buffer += chunk
                
                # Check if we have a complete message (newline-delimited)
                if '\n' not in self._recv_buffer:
                    # Incomplete message
                    return None
                
                # Extract first complete message
                line, self._recv_buffer = self._recv_buffer.split('\n', 1)
                
                if not line:
                    return None
                
                # Parse JSON
                command = json.loads(line)
                
                # Update statistics
                self.bytes_received += len(line) + 1
                self.messages_received += 1
                self.last_receive_time = time.time()
                
                return command
                
        except BlockingIOError:
            # No data available (non-blocking socket)
            return None
            
        except (ConnectionResetError, BrokenPipeError, OSError) as e:
            if self.logger:
                self.logger.log_warning(f"Connection lost during receive: {e}")
            self._handle_connection_loss()
            return None
            
        except json.JSONDecodeError as e:
            if self.logger:
                self.logger.log_error("Failed to decode message", e)
            return None
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Receive failed", e)
            return None
    
    def _handle_connection_loss(self):
        """Handle loss of connection"""
        if self.logger:
            self.logger.log_network_event("connection_lost", {})
        
        self.connected = False
        self.connection_errors += 1
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get network statistics"""
        return {
            'connected': self.connected,
            'bytes_sent': self.bytes_sent,
            'bytes_received': self.bytes_received,
            'messages_sent': self.messages_sent,
            'messages_received': self.messages_received,
            'connection_errors': self.connection_errors,
            'reconnect_attempts': self.reconnect_attempts,
            'last_send_time': self.last_send_time,
            'last_receive_time': self.last_receive_time
        }
    
    def close(self):
        """Close connection"""
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except:
                pass
            
            try:
                self.socket.close()
            except:
                pass
            
            if self.logger:
                self.logger.log_network_event("disconnected", self.get_statistics())
        
        self.socket = None
        self.connected = False
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
