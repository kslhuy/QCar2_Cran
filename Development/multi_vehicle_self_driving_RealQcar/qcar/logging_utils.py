"""
Logging utilities for QCar Vehicle Control System
"""
import logging
import os
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
        
        # Telemetry logging
        self.telemetry_file = None
        self.telemetry_writer = None
        
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
        
        # File handler
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)
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
        
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
        
        return logger
    
    def setup_telemetry_logging(self, data_log_dir: str = "data_logs"):
        """Setup CSV telemetry logging with async thread"""
        os.makedirs(data_log_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.join(data_log_dir, f"run_{timestamp}")
        os.makedirs(run_dir, exist_ok=True)
        
        telemetry_file = os.path.join(run_dir, f"telemetry_vehicle_{self.car_id}.csv")
        self.telemetry_file = open(telemetry_file, 'w', newline='', buffering=1)  # Line buffering
        
        fieldnames = [
            'timestamp', 'time', 'x', 'y', 'th', 'v', 
            'u', 'delta', 'v_ref', 'yolo_gain',
            'waypoint_index', 'cross_track_error', 'heading_error',
            'state', 'gps_valid',
            # Platoon telemetry fields
            'platoon_enabled', 'platoon_id', 'platoon_role', 'platoon_active',
            'leader_id', 'leader_detected', 'leader_distance', 
            'spacing_stable', 'followers_ready', 'all_ready',
            'formation_ready', 'desired_speed', 'spacing_error'
        ]
        
        self.telemetry_writer = csv.DictWriter(self.telemetry_file, fieldnames=fieldnames)
        self.telemetry_writer.writeheader()
        self.telemetry_file.flush()
        
        # Start async logging thread
        self.logging_active = True
        self.logging_thread = threading.Thread(target=self._logging_worker, daemon=True)
        self.logging_thread.start()
        
        self.logger.info(f"Telemetry logging initialized (async): {telemetry_file}")
        
        return run_dir
    
    def _logging_worker(self):
        """Background thread worker for non-blocking logging"""
        while self.logging_active:
            try:
                # Get log entry from queue (timeout to check if we should stop)
                log_entry = self.log_queue.get(timeout=0.1)
                
                if log_entry is None:  # Poison pill to stop thread
                    break
                
                # Write to CSV
                if self.telemetry_writer:
                    self.telemetry_writer.writerow(log_entry)
                    # Flush every 10 entries to balance performance and data safety
                    if self.log_queue.qsize() == 0:
                        self.telemetry_file.flush()
                
            except queue.Empty:
                continue
            except Exception as e:
                # Use basic print to avoid recursion
                print(f"[Logging Worker] Error: {e}")
    
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
        """Close all logging handlers and stop async thread"""
        # Stop logging thread
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
        
        # Close file
        if self.telemetry_file:
            self.telemetry_file.close()
        
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
    
    def log_loop_time(self, dt: float):
        """Log control loop execution time"""
        self.loop_times.append(dt)
        if len(self.loop_times) > self.window_size:
            self.loop_times.pop(0)
        
        self._iteration_count += 1
        
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
            'iteration_count': self._iteration_count
        }
        
        if self.loop_times:
            stats['loop_time'] = {
                'mean': np.mean(self.loop_times),
                'max': np.max(self.loop_times),
                'min': np.min(self.loop_times),
                'std': np.std(self.loop_times),
                'frequency': 1.0 / np.mean(self.loop_times) if np.mean(self.loop_times) > 0 else 0
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
                f"max={lt['max']*1000:.2f}ms, freq={lt['frequency']:.1f}Hz"
            )
        
        if 'network_latency' in stats:
            nl = stats['network_latency']
            self.logger.logger.info(
                f"Performance: Network latency avg={nl['mean']*1000:.2f}ms, "
                f"max={nl['max']*1000:.2f}ms"
            )
