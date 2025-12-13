"""
Logging utilities for QCar Vehicle Control System
"""
import logging
import logging.handlers
import os
import time
from datetime import datetime
from typing import Optional
import csv
import json
import queue
import threading


class VehicleLogger:
    """Enhanced logging system for vehicle control with non-blocking async writes"""
    
    def __init__(self, car_id: int, log_dir: str = "logs", log_level: str = "INFO"):
        self.car_id = car_id
        self.log_dir = log_dir
        
        # Create log directory if it doesn't exist
        os.makedirs(log_dir, exist_ok=True)
        
        # Setup logger
        self.logger = self._setup_logger(log_level)
        
        # Reference start time for relative timestamps (set by vehicle_logic)
        self.start_time = None
        
        # Telemetry logging
        self.telemetry_file = None
        self.telemetry_writer = None
        
        # Fleet estimation logging (received from other vehicles)
        self.fleet_estimation_file = None
        self.fleet_estimation_writer = None
        self.fleet_estimation_queue = queue.Queue(maxsize=1000)
        self.fleet_estimation_thread = None
        self.fleet_estimation_active = False
        
        # Local estimation logging (received from other vehicles)
        self.local_estimation_file = None
        self.local_estimation_writer = None
        self.local_estimation_queue = queue.Queue(maxsize=1000)
        self.local_estimation_thread = None
        self.local_estimation_active = False
        
        # Following leader state logging
        self.following_leader_file = None
        self.following_leader_writer = None
        self.following_leader_queue = queue.Queue(maxsize=1000)
        self.following_leader_thread = None
        self.following_leader_active = False
        
        # Non-blocking logging queue and thread
        self.log_queue = queue.Queue(maxsize=1000)  # Buffer up to 1000 log entries
        self.logging_thread = None
        self.logging_active = False
        
    def _setup_logger(self, log_level: str) -> logging.Logger:
        """Setup logging configuration"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = os.path.join(self.log_dir, f"vehicle_{self.car_id}_{timestamp}.log")
        
        # Create logger
        logger = logging.getLogger(f"Car_{self.car_id}")
        logger.setLevel(getattr(logging, log_level.upper()))
        
        # Remove existing handlers
        logger.handlers.clear()
        
        # File handler with larger buffer to reduce I/O blocking
        file_handler = logging.FileHandler(log_file, mode='a', encoding='utf-8', delay=False)
        file_handler.setLevel(logging.DEBUG)
        
        # Configure handler to use memory buffering (reduces disk I/O)
        # Wrap with MemoryHandler to buffer log records in memory
        memory_handler = logging.handlers.MemoryHandler(
            capacity=100,  # Buffer 100 log records before flushing
            flushLevel=logging.ERROR,  # Auto-flush on ERROR or higher
            target=file_handler
        )
        
        file_formatter = logging.Formatter(
            '%(asctime)s - [Car %(name)s] - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(file_formatter)
        
        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_formatter = logging.Formatter(
            '[Car %(name)s] %(levelname)s - %(message)s'
        )
        console_handler.setFormatter(console_formatter)
        
        # Use memory handler instead of direct file handler to reduce blocking
        logger.addHandler(memory_handler)
        logger.addHandler(console_handler)
        
        # Store reference to force flush on close
        self._memory_handler = memory_handler
        
        return logger
    
    def set_start_time(self, start_time: float):
        """Set the reference start time for relative timestamps"""
        self.start_time = start_time
    
    def get_relative_time(self) -> float:
        """Get time in seconds since start (0.0 if not set)"""
        if self.start_time is None:
            return 0.0
        return time.time() - self.start_time
    
    def setup_telemetry_logging(self, data_log_dir: str = "data_logs"):
        """Setup CSV telemetry logging with async thread"""
        os.makedirs(data_log_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.join(data_log_dir, f"run_{timestamp}")
        os.makedirs(run_dir, exist_ok=True)
        
        telemetry_file = os.path.join(run_dir, f"telemetry_vehicle_{self.car_id}.csv")
        self.telemetry_file = open(telemetry_file, 'w', newline='', buffering=8192)  # 8KB buffer for better performance
        
        fieldnames = [
            # Core telemetry
            'timestamp', 'time', 'x', 'y', 'th', 'v', 
            'u', 'delta', 'v_ref', 'yolo_gain',
            # 'waypoint_index', 'cross_track_error', 'heading_error',
            'state', 'gps_valid',
            # Platoon status (only fields actually returned by _get_platoon_status)
            'platoon_enabled', 'platoon_is_leader', 'platoon_position', 
            'platoon_leader_id', 'platoon_setup_complete',
            # V2V communication status
            'v2v_active', 'v2v_peers', 'v2v_protocol', 
            'v2v_local_rate', 'v2v_fleet_rate'
        ]
        
        self.telemetry_writer = csv.DictWriter(self.telemetry_file, fieldnames=fieldnames, extrasaction='ignore')
        self.telemetry_writer.writeheader()
        self.telemetry_file.flush()
        
        # Start async logging thread
        self.logging_active = True
        self.logging_thread = threading.Thread(target=self._logging_worker, daemon=True)
        self.logging_thread.start()
        
        self.logger.info(f"Telemetry logging initialized (async): {telemetry_file}")
        
        # Setup fleet estimation logging
        self._setup_fleet_estimation_logging(run_dir)
        
        # Setup local estimation logging
        self._setup_local_estimation_logging(run_dir)
        
        # Setup following leader state logging
        self._setup_following_leader_logging(run_dir)
        
        return run_dir
    
    def _logging_worker(self):
        """Background thread worker for non-blocking logging"""
        flush_counter = 0
        while self.logging_active:
            try:
                # Get log entry from queue (timeout to check if we should stop)
                log_entry = self.log_queue.get(timeout=0.1)
                
                if log_entry is None:  # Poison pill to stop thread
                    break
                
                # Write to CSV (buffered)
                if self.telemetry_writer:
                    self.telemetry_writer.writerow(log_entry)
                    flush_counter += 1
                    
                    # Periodic flush (every 100 entries for better performance, or when queue is empty)
                    # Increased from 50 to 100 to reduce flush frequency
                    if flush_counter >= 100 or (self.log_queue.qsize() == 0 and flush_counter > 0):
                        try:
                            # Use os.fsync for guaranteed write (prevents data loss)
                            self.telemetry_file.flush()
                            os.fsync(self.telemetry_file.fileno())
                            flush_counter = 0
                        except (OSError, IOError) as e:
                            # Non-blocking: if flush fails (disk full, etc), just continue
                            print(f"[Logging Worker] Flush failed: {e}")
                
            except queue.Empty:
                continue
            except Exception as e:
                # Use basic print to avoid recursion
                print(f"[Logging Worker] Error: {e}")
                # Brief pause on error to prevent tight error loop
                try:
                    time.sleep(0.01)
                except:
                    pass
    
    def _setup_fleet_estimation_logging(self, run_dir: str):
        """Setup CSV logging for received fleet estimations from other vehicles"""
        fleet_est_file = os.path.join(run_dir, f"received_fleet_estimations_vehicle_{self.car_id}.csv")
        self.fleet_estimation_file = open(fleet_est_file, 'w', newline='', buffering=8192)
        
        fieldnames = [
            'timestamp', 'sender_id', 'source',  # timestamp=relative_time
            'seq_id', 'latency_ns',  # Sequence ID and latency in nanoseconds
            'vehicle_id', 'x', 'y', 'theta', 'v', 'acceleration', 'confidence'
        ]
        
        self.fleet_estimation_writer = csv.DictWriter(self.fleet_estimation_file, fieldnames=fieldnames)
        self.fleet_estimation_writer.writeheader()
        self.fleet_estimation_file.flush()
        
        # Start async logging thread
        self.fleet_estimation_active = True
        self.fleet_estimation_thread = threading.Thread(target=self._fleet_estimation_worker, daemon=True)
        self.fleet_estimation_thread.start()
        
        self.logger.info(f"Fleet estimation logging initialized: {fleet_est_file}")
    
    def _setup_local_estimation_logging(self, run_dir: str):
        """Setup CSV logging for received local estimations from other vehicles"""
        local_est_file = os.path.join(run_dir, f"received_local_estimations_vehicle_{self.car_id}.csv")
        self.local_estimation_file = open(local_est_file, 'w', newline='', buffering=8192)
        
        fieldnames = [
            'timestamp', 'data_age', 'sender_id', 'source',  # timestamp=relative_time, data_age=latency_in_seconds
            'seq_id',  # Sequence ID
            'x', 'y', 'theta', 'v', 'acceleration', 
            'steering', 'throttle','confidence'  # New fields for dynamics and control
        ]
        
        self.local_estimation_writer = csv.DictWriter(self.local_estimation_file, fieldnames=fieldnames)
        self.local_estimation_writer.writeheader()
        self.local_estimation_file.flush()
        
        # Start async logging thread
        self.local_estimation_active = True
        self.local_estimation_thread = threading.Thread(target=self._local_estimation_worker, daemon=True)
        self.local_estimation_thread.start()
        
        self.logger.info(f"Local estimation logging initialized: {local_est_file}")
    
    def _fleet_estimation_worker(self):
        """Background thread worker for fleet estimation logging"""
        flush_counter = 0
        while self.fleet_estimation_active:
            try:
                log_entry = self.fleet_estimation_queue.get(timeout=0.1)
                
                if log_entry is None:  # Poison pill
                    break
                
                if self.fleet_estimation_writer:
                    self.fleet_estimation_writer.writerow(log_entry)
                    flush_counter += 1
                    
                    if flush_counter >= 100 or (self.fleet_estimation_queue.qsize() == 0 and flush_counter > 0):
                        try:
                            self.fleet_estimation_file.flush()
                            os.fsync(self.fleet_estimation_file.fileno())
                            flush_counter = 0
                        except (OSError, IOError) as e:
                            print(f"[Fleet Estimation Logger] Flush failed: {e}")
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[Fleet Estimation Logger] Error: {e}")
                try:
                    time.sleep(0.01)
                except:
                    pass
    
    def _local_estimation_worker(self):
        """Background thread worker for local estimation logging"""
        flush_counter = 0
        while self.local_estimation_active:
            try:
                log_entry = self.local_estimation_queue.get(timeout=0.1)
                
                if log_entry is None:  # Poison pill
                    break
                
                if self.local_estimation_writer:
                    self.local_estimation_writer.writerow(log_entry)
                    flush_counter += 1
                    
                    if flush_counter >= 100 or (self.local_estimation_queue.qsize() == 0 and flush_counter > 0):
                        try:
                            self.local_estimation_file.flush()
                            os.fsync(self.local_estimation_file.fileno())
                            flush_counter = 0
                        except (OSError, IOError) as e:
                            print(f"[Local Estimation Logger] Flush failed: {e}")
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[Local Estimation Logger] Error: {e}")
                try:
                    time.sleep(0.01)
                except:
                    pass
    
    def _setup_following_leader_logging(self, run_dir: str):
        """Setup CSV logging for following leader state control data"""
        following_leader_file = os.path.join(run_dir, f"following_leader_control_vehicle_{self.car_id}.csv")
        self.following_leader_file = open(following_leader_file, 'w', newline='', buffering=8192)
        
        fieldnames = [
            'timestamp',  # Relative time in seconds
            # Follower state
            'follower_x', 'follower_y', 'follower_theta', 'follower_velocity', 'follower_target_velocity',
            # Leader state
            'leader_x', 'leader_y', 'leader_theta', 'leader_velocity',
            # Control commands
            'throttle_u', 'steering_delta',
            # Derived metrics
            'distance_to_leader', 'velocity_difference'
        ]
        
        self.following_leader_writer = csv.DictWriter(self.following_leader_file, fieldnames=fieldnames)
        self.following_leader_writer.writeheader()
        self.following_leader_file.flush()
        
        # Start async logging thread
        self.following_leader_active = True
        self.following_leader_thread = threading.Thread(target=self._following_leader_worker, daemon=True)
        self.following_leader_thread.start()
        
        self.logger.info(f"Following leader state logging initialized: {following_leader_file}")
    
    def _following_leader_worker(self):
        """Background thread worker for following leader state logging"""
        flush_counter = 0
        while self.following_leader_active:
            try:
                log_entry = self.following_leader_queue.get(timeout=0.1)
                
                if log_entry is None:  # Poison pill
                    break
                
                if self.following_leader_writer:
                    self.following_leader_writer.writerow(log_entry)
                    flush_counter += 1
                    
                    if flush_counter >= 100 or (self.following_leader_queue.qsize() == 0 and flush_counter > 0):
                        try:
                            self.following_leader_file.flush()
                            os.fsync(self.following_leader_file.fileno())
                            flush_counter = 0
                        except (OSError, IOError) as e:
                            print(f"[Following Leader Logger] Flush failed: {e}")
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[Following Leader Logger] Error: {e}")
                try:
                    time.sleep(0.01)
                except:
                    pass
    
    def log_telemetry(self, data: dict):
        """Log telemetry data to CSV (non-blocking via queue)"""
        if self.telemetry_writer and self.logging_active:
            try:
                # Non-blocking put - drop data if queue is full (shouldn't happen)
                self.log_queue.put_nowait(data)
            except queue.Full:
                # Queue is full, drop this data point (very rare)
                pass
            except Exception as e:
                # Use basic print to avoid recursion
                print(f"[Telemetry Logger] Error queuing data: {e}")
    
    def log_fleet_estimation(self, sender_id: int, fleet_states: dict, source: str, seq_id: int = 0, send_time_ns: int = 0):
        """Log received fleet estimation from another vehicle (non-blocking via queue)
        
        Args:
            sender_id: ID of the vehicle that sent the fleet estimation
            fleet_states: Dictionary of fleet states {vehicle_id: {x, y, theta, v, confidence}}
            source: Source of estimation (e.g., 'fleet_consensus', 'local_observer')
            seq_id: Sequence ID of the message
            send_time_ns: Send time in nanoseconds for precise latency measurement
        """
        if self.fleet_estimation_writer and self.fleet_estimation_active:
            receive_time_ns = time.time_ns()
            
            # Calculate precise latency in nanoseconds
            latency_ns = receive_time_ns - send_time_ns if send_time_ns > 0 else 0
            
            # Log each vehicle in the fleet estimation as a separate row
            for vehicle_key, state in fleet_states.items():
                # Extract vehicle ID from key (e.g., 'vehicle_0' -> 0)
                if isinstance(vehicle_key, str) and vehicle_key.startswith('vehicle_'):
                    vehicle_id = int(vehicle_key.split('_')[1])
                else:
                    vehicle_id = vehicle_key
                
                try:
                    log_entry = {
                        'timestamp': self.get_relative_time(),  # Relative time in seconds
                        # 'receive_time_ns': receive_time_ns,  # When we received it (relative time in ns)
                        'sender_id': sender_id,
                        'source': source,
                        'seq_id': seq_id,
                        'latency_ns': latency_ns,
                        'vehicle_id': vehicle_id,
                        'x': state.get('x', 0.0),
                        'y': state.get('y', 0.0),
                        'theta': state.get('theta', 0.0),
                        'v': state.get('v', 0.0),
                        'acceleration': state.get('acceleration', 0.0),
                        'confidence': state.get('confidence', 0.0)
                    }
                    self.fleet_estimation_queue.put_nowait(log_entry)
                except queue.Full:
                    pass
                except Exception as e:
                    print(f"[Fleet Estimation Logger] Error queuing data: {e}")
    
    def log_local_estimation(self, sender_id: int, state: dict, source: str, seq_id: int = 0, send_time_ns: int = 0):
        """Log received local estimation from another vehicle (non-blocking via queue)
        
        Args:
            sender_id: ID of the vehicle that sent the local estimation
            state: State dictionary {x, y, theta, v, confidence}
            source: Source of estimation (e.g., 'gps', 'ekf', 'observer')
            timestamp: Timestamp from sender (Unix timestamp - will calculate age)
            seq_id: Sequence ID of the message
            send_time_ns: Send time in nanoseconds for precise latency measurement
        """
        if self.local_estimation_writer and self.local_estimation_active:
            # receive_time = self.get_relative_time()
            receive_time_ns = time.time_ns()
            
            
            # Calculate precise latency in nanoseconds # Age in seconds
            latency_ns = receive_time_ns - send_time_ns if send_time_ns > 0 else 0
            data_age = latency_ns / 1e9  # Convert to seconds

            try:
                # Extract control inputs if available
                control_input = state.get('control_input', {})
                
                log_entry = {
                    'timestamp': self.get_relative_time(),  # Relative time in seconds
                    # 'receive_time_ns': receive_time_ns,  # When we received it (relative time in ns)
                    'data_age': data_age,
                    'sender_id': sender_id,
                    'source': source,
                    'seq_id': seq_id,
                    'x': state.get('x', 0.0),
                    'y': state.get('y', 0.0),
                    'theta': state.get('theta', 0.0),
                    'v': state.get('v', 0.0),
                    'confidence': state.get('confidence', 0.0),
                    'acceleration': state.get('acceleration', 0.0),
                    'steering': control_input.get('steering', 0.0) if isinstance(control_input, dict) else 0.0,
                    'throttle': control_input.get('throttle', 0.0) if isinstance(control_input, dict) else 0.0
                }
                self.local_estimation_queue.put_nowait(log_entry)
            except queue.Full:
                pass
            except Exception as e:
                print(f"[Local Estimation Logger] Error queuing data: {e}")
    
    def log_following_leader_control(self, follower_state: dict, leader_state: dict, u: float, delta: float):
        """Log following leader state control data (non-blocking via queue)
        
        Args:
            follower_state: Dictionary with follower state {x, y, theta, velocity, target_velocity}
            leader_state: Dictionary with leader state {x, y, theta, velocity}
            u: Throttle control command
            delta: Steering control command
            # relative_time: Time in seconds since vehicle logic started (optional, uses 0.0 if None)
        """
        if self.following_leader_writer and self.following_leader_active:
            current_time = time.time()
            
            # Calculate derived metrics
            import numpy as np
            distance_to_leader = np.sqrt(
                (leader_state.get('x', 0.0) - follower_state.get('x', 0.0))**2 +
                (leader_state.get('y', 0.0) - follower_state.get('y', 0.0))**2
            )
            velocity_difference = leader_state.get('velocity', 0.0) - follower_state.get('velocity', 0.0)
            
            try:
                entry = {
                    # 'timestamp': current_time,  # Unix timestamp
                    'timestamp': self.get_relative_time(),  # Unix timestamp
                    
                    # Follower state
                    'follower_x': follower_state.get('x', 0.0),
                    'follower_y': follower_state.get('y', 0.0),
                    'follower_theta': follower_state.get('theta', 0.0),
                    'follower_velocity': follower_state.get('velocity', 0.0),
                    'follower_target_velocity': follower_state.get('target_velocity', 0.0),
                    # Leader state
                    'leader_x': leader_state.get('x', 0.0),
                    'leader_y': leader_state.get('y', 0.0),
                    'leader_theta': leader_state.get('theta', 0.0),
                    'leader_velocity': leader_state.get('velocity', 0.0),
                    # Control commands
                    'throttle_u': u,
                    'steering_delta': delta,
                    # Derived metrics
                    'distance_to_leader': distance_to_leader,
                    'velocity_difference': velocity_difference
                }
                self.following_leader_queue.put_nowait(entry)
            except queue.Full:
                pass  # Drop data if queue is full
            except Exception as e:
                print(f"[Following Leader Logger] Error queuing data: {e}")
    
    def log_network_event(self, event: str, details: Optional[dict] = None):
        """Log network-related events"""
        msg = f"Network: {event}"
        if details:
            msg += f" - {details}"
        self.logger.info(msg)
    
    def log_control_event(self, event: str, values: Optional[dict] = None):
        """Log control-related events"""
        msg = f"Control: {event}"
        if values:
            msg += f" - {values}"
        self.logger.debug(msg)
    
    def log_state_transition(self, old_state: str, new_state: str):
        """Log state machine transitions"""
        self.logger.info(f"State transition: {old_state} -> {new_state}")
    
    def log_error(self, error: str, exception: Optional[Exception] = None):
        """Log errors"""
        if exception:
            self.logger.error(f"{error}: {exception}", exc_info=True)
        else:
            self.logger.error(error)
    
    def log_warning(self, warning: str):
        """Log warnings"""
        self.logger.warning(warning)
    
    def log_performance(self, metric: str, value: float, threshold: Optional[float] = None):
        """Log performance metrics"""
        msg = f"Performance: {metric} = {value:.6f}"
        if threshold and value > threshold:
            msg += f" (EXCEEDS THRESHOLD: {threshold})"
            self.logger.warning(msg)
        else:
            self.logger.debug(msg)
    
    def close(self):
        """Close all logging handlers and stop async threads"""
        # Stop telemetry logging thread
        if self.logging_active:
            self.logging_active = False
            # Send poison pill to stop thread
            try:
                self.log_queue.put(None, timeout=1.0)
            except:
                pass
            
            # Wait for thread to finish
            if self.logging_thread and self.logging_thread.is_alive():
                self.logging_thread.join(timeout=2.0)
        
        # Stop fleet estimation logging thread
        if self.fleet_estimation_active:
            self.fleet_estimation_active = False
            try:
                self.fleet_estimation_queue.put(None, timeout=1.0)
            except:
                pass
            
            if self.fleet_estimation_thread and self.fleet_estimation_thread.is_alive():
                self.fleet_estimation_thread.join(timeout=2.0)
        
        # Stop local estimation logging thread
        if self.local_estimation_active:
            self.local_estimation_active = False
            try:
                self.local_estimation_queue.put(None, timeout=1.0)
            except:
                pass
            
            if self.local_estimation_thread and self.local_estimation_thread.is_alive():
                self.local_estimation_thread.join(timeout=2.0)
        
        # Stop following leader logging thread
        if self.following_leader_active:
            self.following_leader_active = False
            try:
                self.following_leader_queue.put(None, timeout=1.0)
            except:
                pass
            
            if self.following_leader_thread and self.following_leader_thread.is_alive():
                self.following_leader_thread.join(timeout=2.0)
        
        # Flush memory handler before closing
        if hasattr(self, '_memory_handler'):
            try:
                self._memory_handler.flush()
            except:
                pass
        
        # Close telemetry file
        if self.telemetry_file:
            try:
                self.telemetry_file.flush()
                self.telemetry_file.close()
            except:
                pass
        
        # Close fleet estimation file
        if self.fleet_estimation_file:
            try:
                self.fleet_estimation_file.flush()
                self.fleet_estimation_file.close()
            except:
                pass
        
        # Close local estimation file
        if self.local_estimation_file:
            try:
                self.local_estimation_file.flush()
                self.local_estimation_file.close()
            except:
                pass
        
        # Close following leader file
        if self.following_leader_file:
            try:
                self.following_leader_file.flush()
                self.following_leader_file.close()
            except:
                pass
        
        # Close logger handlers
        for handler in self.logger.handlers:
            handler.close()
            self.logger.removeHandler(handler)
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


class PerformanceMonitor:
    """Monitor and log performance metrics"""
    
    def __init__(self, logger: VehicleLogger, window_size: int = 1000):
        self.logger = logger
        self.window_size = window_size
        
        self.loop_times = []
        self.network_latencies = []
        self.control_computation_times = []
        
        self._iteration_count = 0
        self._last_report_time = datetime.now()
        self.report_interval = 10.0  # Report every 10 seconds
        
        # Blocking detection
        self.blocking_threshold = 0.010  # 10ms threshold for potential blocking
        self.severe_blocking_threshold = 0.025  # 25ms threshold for severe blocking
        self.blocking_incidents = 0
        self.severe_blocking_incidents = 0
    
    def log_loop_time(self, dt: float):
        """Log control loop execution time"""
        self.loop_times.append(dt)
        if len(self.loop_times) > self.window_size:
            self.loop_times.pop(0)
        
        self._iteration_count += 1
        
        # Blocking detection
        if dt > self.severe_blocking_threshold:
            self.severe_blocking_incidents += 1
            self.logger.log_warning(
                f"WARNING: SEVERE BLOCKING detected: Loop time {dt*1000:.2f}ms "
                f"(threshold: {self.severe_blocking_threshold*1000:.2f}ms)"
            )
        elif dt > self.blocking_threshold:
            self.blocking_incidents += 1
            if self.blocking_incidents % 50 == 1:  # Log every 50th incident
                self.logger.log_warning(
                    f"WARNING: Potential blocking: Loop time {dt*1000:.2f}ms "
                    f"(incident #{self.blocking_incidents})"
                )
        
        # Periodic reporting
        now = datetime.now()
        if (now - self._last_report_time).total_seconds() >= self.report_interval:
            self.report_statistics()
            self._last_report_time = now
    
    def log_network_latency(self, latency: float):
        """Log network communication latency"""
        self.network_latencies.append(latency)
        if len(self.network_latencies) > self.window_size:
            self.network_latencies.pop(0)
    
    def log_control_computation_time(self, computation_time: float):
        """Log control computation time"""
        self.control_computation_times.append(computation_time)
        if len(self.control_computation_times) > self.window_size:
            self.control_computation_times.pop(0)
    
    def get_statistics(self) -> dict:
        """Get current performance statistics"""
        import numpy as np
        
        stats = {
            'iteration_count': self._iteration_count,
            'blocking_incidents': self.blocking_incidents,
            'severe_blocking_incidents': self.severe_blocking_incidents
        }
        
        if self.loop_times:
            stats['loop_time'] = {
                'mean': np.mean(self.loop_times),
                'max': np.max(self.loop_times),
                'min': np.min(self.loop_times),
                'std': np.std(self.loop_times),
                'frequency': 1.0 / np.mean(self.loop_times) if np.mean(self.loop_times) > 0 else 0,
                'blocking_percentage': (self.blocking_incidents / self._iteration_count * 100) if self._iteration_count > 0 else 0
            }
        
        if self.network_latencies:
            stats['network_latency'] = {
                'mean': np.mean(self.network_latencies),
                'max': np.max(self.network_latencies),
                'min': np.min(self.network_latencies)
            }
        
        if self.control_computation_times:
            stats['control_computation'] = {
                'mean': np.mean(self.control_computation_times),
                'max': np.max(self.control_computation_times)
            }
        
        return stats
    
    def report_statistics(self):
        """Report current statistics to logger"""
        stats = self.get_statistics()
        
        if 'loop_time' in stats:
            lt = stats['loop_time']
            self.logger.logger.info(
                f"Performance: Loop time avg={lt['mean']*1000:.2f}ms, "
                f"max={lt['max']*1000:.2f}ms, freq={lt['frequency']:.1f}Hz, "
                f"blocking={lt['blocking_percentage']:.1f}%"
            )
            
            # Alert on concerning blocking levels
            if lt['blocking_percentage'] > 5.0:
                self.logger.log_warning(
                    f"WARNING: High blocking percentage: {lt['blocking_percentage']:.1f}% "
                    f"({stats['blocking_incidents']} incidents)"
                )
        
        if 'network_latency' in stats:
            nl = stats['network_latency']
            self.logger.logger.info(
                f"Performance: Network latency avg={nl['mean']*1000:.2f}ms, "
                f"max={nl['max']*1000:.2f}ms"
            )
        
        # Reset counters periodically to prevent overflow
        if self._iteration_count > 10000:
            self.blocking_incidents = max(0, self.blocking_incidents - 100)
            self.severe_blocking_incidents = max(0, self.severe_blocking_incidents - 10)
