"""
Threaded Network Handler - Non-blocking network communication
"""
import threading
import queue
import time
import socket
from typing import Optional, Dict, Any
from threading import Event

from network_client import NetworkClient2GS
from logging_utils import VehicleLogger
from config import VehicleControlConfig


class ThreadedNetworkHandler:
    """Handles network communication in separate threads to avoid blocking main control loop"""
    
    def __init__(self, config: VehicleControlConfig, logger: VehicleLogger, kill_event: Event):
        self.config = config
        self.logger = logger
        self.kill_event = kill_event
        
        # Network client
        self.network = None
        
        # Thread-safe queues
        self.telemetry_queue = queue.Queue(maxsize=100)  # Bounded to prevent memory growth
        self.command_queue = queue.Queue(maxsize=50)
        
        # Worker threads
        self.telemetry_thread = None
        self.command_thread = None
        
        # Thread control
        self.threads_running = False
        self.shutdown_event = Event()
        
        # Statistics and circuit breaker
        self.telemetry_sent_count = 0
        self.commands_received_count = 0
        self.queue_overflows = 0
        
        # Circuit breaker for network resilience
        self.circuit_breaker_errors = 0
        self.circuit_breaker_threshold = 50  # Open circuit after 50 consecutive errors
        self.circuit_breaker_cooldown = 30.0  # 30 seconds cooldown
        self.circuit_breaker_open_time = 0.0
        self.circuit_breaker_state = "closed"  # "closed", "open", "half_open"
    
    def initialize_network(self) -> bool:
        """Initialize network connection"""
        if not self.config.network.is_remote_enabled:
            self.logger.logger.info("Remote control disabled - no network threads needed")
            return True
        
        try:
            self.network = NetworkClient2GS(
                host_ip=self.config.network.host_ip,
                port=self.config.network.port,
                car_id=self.config.network.car_id,
                logger=self.logger,
                config=self.config
            )
            
            # Try to connect
            if not self.network.connect():
                self.logger.log_warning("Failed to connect to host PC")
                return False
            
            self.logger.logger.info("Network connection established")
            return True
            
        except Exception as e:
            self.logger.log_error("Network initialization failed", e)
            return False
    
    def start_threads(self) -> bool:
        """Start the network worker threads"""
        if not self.network or not self.network.connected:
            self.logger.logger.info("Network not connected - skipping thread startup")
            return True
        
        try:
            self.threads_running = True
            self.shutdown_event.clear()
            
            # Start telemetry sender thread
            self.telemetry_thread = threading.Thread(
                target=self._telemetry_worker,
                name=f"TelemetryThread-Car{self.config.network.car_id}",
                daemon=True
            )
            self.telemetry_thread.start()
            
            # Start command receiver thread
            self.command_thread = threading.Thread(
                target=self._command_worker,
                name=f"CommandThread-Car{self.config.network.car_id}",
                daemon=True
            )
            self.command_thread.start()
            
            self.logger.logger.info("Network worker threads started")
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
                self.logger.logger.info("🔄 Circuit breaker: Half-open (testing connection)")
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
    
    def _telemetry_worker(self):
        """Worker thread for sending telemetry data"""
        self.logger.logger.info("Telemetry worker thread started")
        
        consecutive_errors = 0
        max_consecutive_errors = 10
        
        try:
            while not self.shutdown_event.is_set() and not self.kill_event.is_set():
                try:
                    # Wait for telemetry data with timeout
                    telemetry_data = self.telemetry_queue.get(timeout=0.1)
                    
                    # Send telemetry with circuit breaker protection
                    if self.network and self.network.connected:
                        # Check circuit breaker
                        if not self._circuit_breaker_check():
                            # Circuit is open, skip this operation
                            self.telemetry_queue.task_done()
                            continue
                        
                        try:
                            # Set socket timeout for send operation (if supported)
                            if hasattr(self.network, 'socket') and self.network.socket:
                                old_timeout = self.network.socket.gettimeout()
                                self.network.socket.settimeout(0.05)  # 50ms timeout
                            
                            self.network.send_telemetry(telemetry_data)
                            self.telemetry_sent_count += 1
                            consecutive_errors = 0  # Reset error counter on success
                            self._circuit_breaker_success()
                            
                            # Restore original timeout
                            if hasattr(self.network, 'socket') and self.network.socket:
                                self.network.socket.settimeout(old_timeout)
                                
                        except (socket.timeout, socket.error, OSError, IOError) as net_error:
                            consecutive_errors += 1
                            self._circuit_breaker_failure()
                            
                            if consecutive_errors <= max_consecutive_errors:
                                # Don't log every network error to avoid spam
                                if consecutive_errors % 5 == 1:
                                    self.logger.log_warning(f"Network send timeout/error (#{consecutive_errors}): {net_error}")
                            else:
                                # Too many consecutive errors - might be connection lost
                                self.logger.log_warning(f"Network severely degraded ({consecutive_errors} errors), continuing anyway")
                                time.sleep(0.1)  # Brief backoff
                    
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
            self.logger.logger.info("Telemetry worker thread stopped")
    
    def _command_worker(self):
        """Worker thread for receiving commands"""
        self.logger.logger.info("Command worker thread started")
        
        consecutive_errors = 0
        max_consecutive_errors = 10
        
        try:
            while not self.shutdown_event.is_set() and not self.kill_event.is_set():
                try:
                    # Check for commands with circuit breaker protection
                    if self.network and self.network.connected:
                        # Check circuit breaker
                        if not self._circuit_breaker_check():
                            # Circuit is open, skip this operation
                            time.sleep(0.05)
                            continue
                        
                        try:
                            # Set socket timeout for receive operation (if supported)
                            if hasattr(self.network, 'socket') and self.network.socket:
                                old_timeout = self.network.socket.gettimeout()
                                self.network.socket.settimeout(0.02)  # 20ms timeout for non-blocking behavior
                            
                            commands = self.network.receive_commands()
                            consecutive_errors = 0  # Reset error counter on success
                            self._circuit_breaker_success()
                            
                            # Restore original timeout
                            if hasattr(self.network, 'socket') and self.network.socket:
                                self.network.socket.settimeout(old_timeout)
                            
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
                                        
                        except (socket.timeout, socket.error, OSError, IOError) as net_error:
                            consecutive_errors += 1
                            self._circuit_breaker_failure()
                            
                            if consecutive_errors <= max_consecutive_errors:
                                # Don't log every network error to avoid spam
                                if consecutive_errors % 5 == 1:
                                    self.logger.log_warning(f"Network receive timeout/error (#{consecutive_errors}): {net_error}")
                            else:
                                # Too many consecutive errors
                                time.sleep(0.1)  # Brief backoff
                    
                    # Control receive rate - check every 50ms (20Hz)
                    time.sleep(0.05)
                    
                except Exception as e:
                    consecutive_errors += 1
                    self.logger.log_error("Error in command worker", e)
                    time.sleep(0.1)  # Brief pause before retrying
        
        except Exception as e:
            self.logger.log_error("Command worker thread crashed", e)
        finally:
            self.logger.logger.info("Command worker thread stopped")
    
    def stop_threads(self):
        """Stop the worker threads gracefully"""
        if not self.threads_running:
            return
        
        self.logger.logger.info("Stopping network worker threads...")
        
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
        
        self.logger.logger.info(f"Network threads stopped. Stats - Telemetry sent: {self.telemetry_sent_count}, Commands received: {self.commands_received_count}, Queue overflows: {self.queue_overflows}")
    
    def close(self):
        """Close network connection"""
        self.stop_threads()
        
        if self.network:
            self.network.close()
            self.network = None
    
    def is_connected(self) -> bool:
        """Check if network is connected"""
        return self.network and self.network.connected if self.network else False
    
    def get_statistics(self) -> Dict[str, int]:
        """Get network statistics"""
        return {
            'telemetry_sent': self.telemetry_sent_count,
            'commands_received': self.commands_received_count,
            'queue_overflows': self.queue_overflows,
            'telemetry_queue_size': self.telemetry_queue.qsize(),
            'command_queue_size': self.command_queue.qsize(),
            'circuit_breaker_state': self.circuit_breaker_state,
            'circuit_breaker_errors': self.circuit_breaker_errors
        }