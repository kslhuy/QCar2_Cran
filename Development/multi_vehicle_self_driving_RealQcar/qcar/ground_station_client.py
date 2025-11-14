"""
Ground Station Client - Unified threaded network handler for vehicle communication
Combines socket operations and threading for robust Ground Station communication
"""
import threading
import queue
import time
import socket
import json
from typing import Optional, Dict, Any
from threading import Event, Lock

from logging_utils import VehicleLogger
from config import VehicleControlConfig


class GroundStationClient:
    """
    Unified Ground Station communication handler with integrated socket operations and threading.
    Handles telemetry transmission and command reception in separate threads for non-blocking operation.
    """
    
    def __init__(self, config: VehicleControlConfig, logger: VehicleLogger, kill_event: Event):
        self.config = config
        self.logger = logger
        self.kill_event = kill_event
        
        # Network configuration
        self.host_ip = config.network.host_ip
        self.port = config.network.port
        self.car_id = config.network.car_id
        self.connection_timeout = config.network.connection_timeout if hasattr(config.network, 'connection_timeout') else 15
        self.max_reconnect_attempts = config.network.max_reconnect_attempts if hasattr(config.network, 'max_reconnect_attempts') else 10
        self.reconnect_delay = config.network.reconnect_delay if hasattr(config.network, 'reconnect_delay') else 2.0
        
        # Socket and connection state
        self.socket = None
        self.connected = False
        self.reconnect_attempts = 0
        self.last_send_time = 0
        self.last_receive_time = 0
        
        # Thread-safe queues
        self.telemetry_queue = queue.Queue(maxsize=100)  # Bounded to prevent memory growth
        self.command_queue = queue.Queue(maxsize=20)
        
        # Worker threads
        self.telemetry_thread = None
        self.command_thread = None
        
        # Thread control
        self.threads_running = False
        self.shutdown_event = Event()
        
        # Thread safety for socket operations
        self._send_lock = Lock()
        self._receive_lock = Lock()
        
        # Statistics
        self.telemetry_sent_count = 0
        self.commands_received_count = 0
        self.queue_overflows = 0
        self.bytes_sent = 0
        self.bytes_received = 0
        self.messages_sent = 0
        self.messages_received = 0
        self.connection_errors = 0
        
        # Circuit breaker for network resilience (tuned for local connections)
        self.circuit_breaker_errors = 0
        self.circuit_breaker_threshold = 100  # Open circuit after 100 consecutive errors
        self.circuit_breaker_cooldown = 10.0  # 10 seconds cooldown
        self.circuit_breaker_open_time = 0.0
        self.circuit_breaker_state = "closed"  # "closed", "open", "half_open"
        
        # Receive buffer for partial messages
        self._recv_buffer = ""
    
    def initialize_network(self) -> bool:
        """Initialize network connection to Ground Station"""
        if not self.config.network.is_remote_enabled:
            self.logger.logger.info("Remote control disabled - no network connection needed")
            return True
        
        if self.logger:
            self.logger.log_network_event(
                "connection_attempt",
                {"host": self.host_ip, "port": self.port, "car_id": self.car_id}
            )
        
        return self._connect()
    
    def _connect(self, timeout: Optional[int] = None) -> bool:
        """
        Connect to the Ground Station with retry logic
        
        Args:
            timeout: Connection timeout in seconds (uses config default if None)
            
        Returns:
            True if connection successful
        """
        if timeout is None:
            timeout = self.connection_timeout
        
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(timeout)
            self.socket.connect((self.host_ip, self.port))
            
            # Keep socket blocking with reasonable timeouts for stability
            self.socket.settimeout(1.0)  # 1 second default timeout
            
            self.connected = True
            self.reconnect_attempts = 0
            
            if self.logger:
                self.logger.log_network_event("connected", {
                    "host": self.host_ip, 
                    "port": self.port, 
                    "car_id": self.car_id
                })
            
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
    
    def _reconnect(self) -> bool:
        """Attempt to reconnect to Ground Station"""
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
        return self._connect()
    
    def start_threads(self) -> bool:
        """Start the network worker threads"""
        if not self.connected:
            self.logger.logger.info("Network not connected - skipping thread startup")
            return True
        
        try:
            self.threads_running = True
            self.shutdown_event.clear()
            
            # Start telemetry sender thread
            self.telemetry_thread = threading.Thread(
                target=self._telemetry_worker,
                name=f"TelemetryThread-Car{self.car_id}",
                daemon=True
            )
            self.telemetry_thread.start()
            
            # Start command receiver thread
            self.command_thread = threading.Thread(
                target=self._command_worker,
                name=f"CommandThread-Car{self.car_id}",
                daemon=True
            )
            self.command_thread.start()
            
            self.logger.logger.info("Ground Station network worker threads started")
            return True
            
        except Exception as e:
            self.logger.log_error("Failed to start network threads", e)
            return False
    
    def queue_telemetry(self, telemetry_data: Dict[str, Any]) -> bool:
        """Queue telemetry data for sending (non-blocking)"""
        if not self.threads_running:
            return False
        
        try:
            # Try to put data in queue without blocking
            self.telemetry_queue.put_nowait(telemetry_data)
            return True
        except queue.Full:
            # Queue is full - drop oldest data to make room
            try:
                self.telemetry_queue.get_nowait()  # Remove one item
                self.telemetry_queue.put_nowait(telemetry_data)  # Add new data
                self.queue_overflows += 1
                if self.queue_overflows % 100 == 1:  # Log every 100 overflows
                    self.logger.log_warning(f"Telemetry queue overflow (count: {self.queue_overflows})")
                return True
            except queue.Empty:
                return False
    
    def get_latest_commands(self) -> Optional[Dict[str, Any]]:
        """Get latest commands from queue (non-blocking)"""
        if not self.threads_running:
            return None
        
        latest_commands = None
        
        # Drain the queue to get the latest commands only
        try:
            while True:
                latest_commands = self.command_queue.get_nowait()
        except queue.Empty:
            pass
        
        return latest_commands
    
    def _circuit_breaker_check(self) -> bool:
        """Check if circuit breaker allows network operations"""
        current_time = time.time()
        
        if self.circuit_breaker_state == "open":
            # Check if cooldown period has passed
            if current_time - self.circuit_breaker_open_time > self.circuit_breaker_cooldown:
                self.circuit_breaker_state = "half_open"
                self.logger.logger.info("[STATE] Circuit breaker: Half-open (testing connection)")
                return True
            return False  # Still in cooldown
        
        return True  # Closed or half-open allows operations
    
    def _circuit_breaker_success(self):
        """Record successful network operation"""
        if self.circuit_breaker_state == "half_open":
            # Test successful, close circuit
            self.circuit_breaker_state = "closed"
            self.circuit_breaker_errors = 0
            self.logger.logger.info("✅ Circuit breaker: Closed (connection restored)")
        elif self.circuit_breaker_state == "closed":
            # Reset error counter on success
            self.circuit_breaker_errors = max(0, self.circuit_breaker_errors - 1)
    
    def _circuit_breaker_failure(self):
        """Record failed network operation"""
        self.circuit_breaker_errors += 1
        
        if (self.circuit_breaker_state in ["closed", "half_open"] and 
            self.circuit_breaker_errors >= self.circuit_breaker_threshold):
            # Open circuit
            self.circuit_breaker_state = "open"
            self.circuit_breaker_open_time = time.time()
            self.logger.log_warning(
                f"🚫 Circuit breaker: Opened ({self.circuit_breaker_errors} errors) "
                f"- Network operations suspended for {self.circuit_breaker_cooldown}s"
            )
    
    def _send_telemetry_raw(self, data: Dict[str, Any]) -> bool:
        """
        Send telemetry data to Ground Station with retry logic
        
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
                    # Serialize to JSON with newline (matches Ground Station protocol)
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
    
    def _receive_commands_raw(self) -> Optional[Dict[str, Any]]:
        """
        Receive commands from Ground Station (non-blocking)
        
        Returns:
            Dictionary containing command data, or None if no data available
        """
        if not self.connected:
            return None
        
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
                
        except socket.timeout:
            # No data available within timeout period (blocking socket)
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
    
    def _telemetry_worker(self):
        """Worker thread for sending telemetry data"""
        self.logger.logger.info("Ground Station telemetry worker thread started")
        
        consecutive_errors = 0
        max_consecutive_errors = 10
        
        try:
            while not self.shutdown_event.is_set() and not self.kill_event.is_set():
                try:
                    # Wait for telemetry data with timeout
                    telemetry_data = self.telemetry_queue.get(timeout=0.1)
                    
                    # Send telemetry with circuit breaker protection
                    if self.connected:
                        # Check circuit breaker
                        if not self._circuit_breaker_check():
                            # Circuit is open, skip this operation
                            self.telemetry_queue.task_done()
                            continue
                        
                        try:
                            # Use existing socket timeout (no aggressive changes)
                            if self._send_telemetry_raw(telemetry_data):
                                self.telemetry_sent_count += 1
                                consecutive_errors = 0  # Reset error counter on success
                                self._circuit_breaker_success()
                            else:
                                consecutive_errors += 1
                                self._circuit_breaker_failure()
                                
                        except socket.timeout:
                            # Timeout is normal for blocking sockets - don't count as failure
                            pass
                        except (ConnectionResetError, BrokenPipeError, OSError) as net_error:
                            consecutive_errors += 1
                            self._circuit_breaker_failure()
                            
                            if consecutive_errors <= max_consecutive_errors:
                                # Don't log every network error to avoid spam
                                if consecutive_errors % 10 == 1:
                                    self.logger.log_warning(f"Network connection error (#{consecutive_errors}): {net_error}")
                            else:
                                # Too many consecutive errors - connection likely lost
                                self.logger.log_warning(f"Network connection severely degraded ({consecutive_errors} errors)")
                                self._handle_connection_loss()
                                time.sleep(0.5)  # Longer backoff for connection issues
                    
                    # Mark task as done
                    self.telemetry_queue.task_done()
                    
                except queue.Empty:
                    continue  # Timeout - check for shutdown
                except Exception as e:
                    self.logger.log_error("Error in telemetry worker", e)
                    consecutive_errors += 1
                    time.sleep(0.1)  # Brief pause before retrying
        
        except Exception as e:
            self.logger.log_error("Telemetry worker thread crashed", e)
        finally:
            self.logger.logger.info("Ground Station telemetry worker thread stopped")
    
    def _command_worker(self):
        """Worker thread for receiving commands"""
        self.logger.logger.info("Ground Station command worker thread started")
        
        consecutive_errors = 0
        max_consecutive_errors = 10
        
        try:
            while not self.shutdown_event.is_set() and not self.kill_event.is_set():
                try:
                    # Check for commands with circuit breaker protection
                    if self.connected:
                        # Check circuit breaker
                        if not self._circuit_breaker_check():
                            # Circuit is open, skip this operation
                            time.sleep(0.05)
                            continue
                        
                        try:
                            # Use existing socket timeout (no aggressive changes)
                            commands = self._receive_commands_raw()
                            if commands is not None:
                                consecutive_errors = 0  # Reset error counter on success
                                self._circuit_breaker_success()
                            
                            if commands:
                                # Queue the commands for main thread processing
                                try:
                                    self.command_queue.put_nowait(commands)
                                    self.commands_received_count += 1
                                except queue.Full:
                                    # Command queue is full - replace with latest
                                    try:
                                        self.command_queue.get_nowait()  # Remove old command
                                        self.command_queue.put_nowait(commands)  # Add new command
                                    except queue.Empty:
                                        pass
                                        
                        except socket.timeout:
                            # Timeout is normal for blocking sockets - don't count as failure
                            pass
                        except (ConnectionResetError, BrokenPipeError, OSError) as net_error:
                            consecutive_errors += 1
                            self._circuit_breaker_failure()
                            
                            if consecutive_errors <= max_consecutive_errors:
                                # Don't log every network error to avoid spam
                                if consecutive_errors % 10 == 1:
                                    self.logger.log_warning(f"Network receive error (#{consecutive_errors}): {net_error}")
                            else:
                                # Too many consecutive errors - connection likely lost
                                self._handle_connection_loss()
                                time.sleep(0.5)  # Longer backoff for connection issues
                    
                    # Control receive rate - check every 50ms (20Hz)
                    time.sleep(0.05)
                    
                except Exception as e:
                    consecutive_errors += 1
                    self.logger.log_error("Error in command worker", e)
                    time.sleep(0.1)  # Brief pause before retrying
        
        except Exception as e:
            self.logger.log_error("Command worker thread crashed", e)
        finally:
            self.logger.logger.info("Ground Station command worker thread stopped")
    
    def stop_threads(self):
        """Stop the worker threads gracefully"""
        if not self.threads_running:
            return
        
        self.logger.logger.info("Stopping Ground Station network worker threads...")
        
        # Signal threads to stop
        self.shutdown_event.set()
        self.threads_running = False
        
        # Wait for threads to finish with timeout
        if self.telemetry_thread and self.telemetry_thread.is_alive():
            self.telemetry_thread.join(timeout=1.0)
            if self.telemetry_thread.is_alive():
                self.logger.log_warning("Telemetry thread did not stop gracefully")
        
        if self.command_thread and self.command_thread.is_alive():
            self.command_thread.join(timeout=1.0)
            if self.command_thread.is_alive():
                self.logger.log_warning("Command thread did not stop gracefully")
        
        # Clear queues
        try:
            while not self.telemetry_queue.empty():
                self.telemetry_queue.get_nowait()
        except queue.Empty:
            pass
        
        try:
            while not self.command_queue.empty():
                self.command_queue.get_nowait()
        except queue.Empty:
            pass
        
        self.logger.logger.info(
            f"Ground Station network threads stopped. "
            f"Stats - Telemetry sent: {self.telemetry_sent_count}, "
            f"Commands received: {self.commands_received_count}, "
            f"Queue overflows: {self.queue_overflows}"
        )
    
    def close(self):
        """Close network connection"""
        self.stop_threads()
        
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
    
    def is_connected(self) -> bool:
        """Check if network is connected"""
        return self.connected
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get comprehensive network statistics"""
        return {
            # Threading stats
            'telemetry_sent': self.telemetry_sent_count,
            'commands_received': self.commands_received_count,
            'queue_overflows': self.queue_overflows,
            'telemetry_queue_size': self.telemetry_queue.qsize() if self.telemetry_queue else 0,
            'command_queue_size': self.command_queue.qsize() if self.command_queue else 0,
            'circuit_breaker_state': self.circuit_breaker_state,
            'circuit_breaker_errors': self.circuit_breaker_errors,
            
            # Socket stats
            'connected': self.connected,
            'bytes_sent': self.bytes_sent,
            'bytes_received': self.bytes_received,
            'messages_sent': self.messages_sent,
            'messages_received': self.messages_received,
            'connection_errors': self.connection_errors,
            'reconnect_attempts': self.reconnect_attempts,
            'last_send_time': self.last_send_time,
            'last_receive_time': self.last_receive_time,
            
            # Configuration
            'host_ip': self.host_ip,
            'port': self.port,
            'car_id': self.car_id
        }
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()