"""
Simple Vehicle Logger - Fusion of logging_utils.py and md_logging_config.py
Lightweight, scalable logging system for vehicle control with modular design
"""
import logging
import os
import time
import csv
import queue
import threading
from datetime import datetime
from typing import Optional, Dict
from logging.handlers import RotatingFileHandler


class AsyncFileHandler(logging.Handler):
    """Lightweight async logging handler for non-blocking writes"""
    
    def __init__(self, file_path: str, max_bytes: int = 5*1024*1024, backup_count: int = 2):
        super().__init__()
        self.file_handler = RotatingFileHandler(file_path, maxBytes=max_bytes, backupCount=backup_count)
        self.log_queue = queue.Queue(maxsize=500)  # Smaller queue for lower memory
        self.worker_thread = None
        self.active = False
        self._start_worker()
    
    def _start_worker(self):
        """Start background logging worker"""
        if not self.active:
            self.active = True
            self.worker_thread = threading.Thread(target=self._log_worker, daemon=True)
            self.worker_thread.start()
    
    def _log_worker(self):
        """Background worker for non-blocking file writes"""
        while self.active:
            try:
                record = self.log_queue.get(timeout=0.1)
                if record is None:  # Poison pill
                    break
                self.file_handler.emit(record)
                self.log_queue.task_done()
            except queue.Empty:
                continue
            except Exception:
                continue  # Silent failure for performance
    
    def emit(self, record):
        """Queue log record for async writing"""
        try:
            self.log_queue.put_nowait(record)
        except queue.Full:
            pass  # Drop logs if queue full - prevents blocking
    
    def close(self):
        """Close handler and stop worker"""
        if self.active:
            self.active = False
            try:
                self.log_queue.put(None, timeout=0.5)  # Poison pill
            except:
                pass
            if self.worker_thread and self.worker_thread.is_alive():
                self.worker_thread.join(timeout=1.0)
        if self.file_handler:
            self.file_handler.close()
        super().close()


class SimpleVehicleLogger:
    """
    Simple, scalable vehicle logging system with modular design
    Features:
    - Separate log files for different subsystems
    - Lightweight with minimal configuration
    - Non-blocking async writes
    - CSV telemetry logging
    - Automatic file rotation
    """
    
    def __init__(self, vehicle_id: int, log_dir: str = "logs"):
        self.vehicle_id = vehicle_id
        self.log_dir = log_dir
        self.loggers = {}
        self.telemetry_writer = None
        self.telemetry_file = None
        
        # Create log directory
        os.makedirs(log_dir, exist_ok=True)
        
        # Setup modular loggers
        self._setup_loggers()
        
        # Telemetry async logging
        self.telemetry_queue = queue.Queue(maxsize=1000)
        self.telemetry_active = False
        self.telemetry_thread = None
    
    def _setup_loggers(self):
        """Setup separate loggers for different subsystems"""
        
        # Define subsystem loggers with simple configuration
        subsystems = {
            'main': 'Main vehicle operations and control',
            'network': 'Network communication and V2V',
            'observer': 'State estimation and observer',
            'control': 'Control logic and commands'
        }
        
        # Standard formatter for all subsystems
        formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] V{} [%(name)s]: %(message)s'.format(self.vehicle_id),
            datefmt='%H:%M:%S'
        )
        
        for subsystem, description in subsystems.items():
            # Create logger
            logger = logging.getLogger(f"vehicle_{self.vehicle_id}_{subsystem}")
            logger.setLevel(logging.INFO)
            logger.handlers.clear()
            logger.propagate = False
            
            # Create async file handler
            log_file = os.path.join(self.log_dir, f"vehicle_{self.vehicle_id}_{subsystem}.log")
            async_handler = AsyncFileHandler(log_file)
            async_handler.setFormatter(formatter)
            logger.addHandler(async_handler)
            
            # Store logger
            self.loggers[subsystem] = logger
            
            # Log initialization
            logger.info(f"Logger initialized - {description}")
    
    def setup_telemetry_logging(self, data_log_dir: str = "data_logs") -> str:
        """Setup CSV telemetry logging with async writes"""
        os.makedirs(data_log_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.join(data_log_dir, f"run_{timestamp}")
        os.makedirs(run_dir, exist_ok=True)
        
        # Create telemetry CSV file
        telemetry_file = os.path.join(run_dir, f"telemetry_vehicle_{self.vehicle_id}.csv")
        self.telemetry_file = open(telemetry_file, 'w', newline='', buffering=4096)
        
        # Define telemetry fields
        fieldnames = [
            'timestamp', 'time', 'x', 'y', 'th', 'v', 'u', 'delta', 
            'v_ref', 'yolo_gain', 'waypoint_index', 'cross_track_error', 
            'heading_error', 'state', 'gps_valid',
            # V2V and network status
            'v2v_active', 'v2v_peers', 'v2v_protocol'
        ]
        
        self.telemetry_writer = csv.DictWriter(self.telemetry_file, fieldnames=fieldnames)
        self.telemetry_writer.writeheader()
        self.telemetry_file.flush()
        
        # Start async telemetry logging
        self.telemetry_active = True
        self.telemetry_thread = threading.Thread(target=self._telemetry_worker, daemon=True)
        self.telemetry_thread.start()
        
        self.get_logger('main').info(f"Telemetry logging started: {telemetry_file}")
        return run_dir
    
    def _telemetry_worker(self):
        """Background worker for telemetry CSV writing"""
        flush_counter = 0
        while self.telemetry_active:
            try:
                entry = self.telemetry_queue.get(timeout=0.1)
                if entry is None:  # Poison pill
                    break
                
                if self.telemetry_writer:
                    self.telemetry_writer.writerow(entry)
                    flush_counter += 1
                    
                    # Periodic flush for data safety
                    if flush_counter >= 50 or self.telemetry_queue.qsize() == 0:
                        try:
                            self.telemetry_file.flush()
                            flush_counter = 0
                        except:
                            pass  # Silent failure
                
            except queue.Empty:
                continue
            except Exception:
                continue  # Silent failure for performance
    
    def get_logger(self, subsystem: str = 'main') -> logging.Logger:
        """Get logger for specific subsystem"""
        return self.loggers.get(subsystem, self.loggers['main'])
    
    def get_subsystem_logger(self, subsystem: str, component_name: str = None) -> logging.Logger:
        """
        Get a logger for external components with optional component name prefix
        
        Args:
            subsystem: Subsystem type ('main', 'network', 'observer', 'control')
            component_name: Optional component name to add to logger name
            
        Returns:
            Logger instance for the component
        """
        base_logger = self.get_logger(subsystem)
        
        # If component name is provided, create a child logger
        if component_name:
            return base_logger.getChild(component_name)
        
        return base_logger
    
    def log_telemetry(self, data: Dict):
        """Log telemetry data to CSV (non-blocking)"""
        if self.telemetry_active:
            try:
                self.telemetry_queue.put_nowait(data)
            except queue.Full:
                pass  # Drop data if queue full
    
    # Convenience methods for different subsystems
    def log_main(self, level: str, message: str):
        """Log main vehicle operations"""
        getattr(self.get_logger('main'), level.lower())(message)
    
    def log_network(self, level: str, message: str):
        """Log network and communication events"""
        getattr(self.get_logger('network'), level.lower())(message)
    
    def log_observer(self, level: str, message: str):
        """Log observer and state estimation"""
        getattr(self.get_logger('observer'), level.lower())(message)
    
    def log_control(self, level: str, message: str):
        """Log control logic and commands"""
        getattr(self.get_logger('control'), level.lower())(message)
    
    # High-level logging methods
    def info(self, message: str, subsystem: str = 'main'):
        """Log info message"""
        self.get_logger(subsystem).info(message)
    
    def warning(self, message: str, subsystem: str = 'main'):
        """Log warning message"""
        self.get_logger(subsystem).warning(message)
    
    def error(self, message: str, exception: Optional[Exception] = None, subsystem: str = 'main'):
        """Log error message with optional exception"""
        if exception:
            self.get_logger(subsystem).error(f"{message}: {exception}", exc_info=True)
        else:
            self.get_logger(subsystem).error(message)
    
    def debug(self, message: str, subsystem: str = 'main'):
        """Log debug message"""
        self.get_logger(subsystem).debug(message)
    
    # Specialized logging methods for vehicle operations
    def log_state_transition(self, old_state: str, new_state: str):
        """Log state machine transitions"""
        self.info(f"State transition: {old_state} -> {new_state}", 'main')
    
    def log_network_event(self, event: str, details: Optional[Dict] = None):
        """Log network events"""
        msg = f"Network: {event}"
        if details:
            msg += f" - {details}"
        self.info(msg, 'network')
    
    def log_control_event(self, event: str, values: Optional[Dict] = None):
        """Log control events"""
        msg = f"Control: {event}"
        if values:
            msg += f" - {values}"
        self.info(msg, 'control')
    
    def log_v2v_status(self, status: str, details: Optional[Dict] = None):
        """Log V2V communication status"""
        msg = f"V2V: {status}"
        if details:
            peer_count = details.get('peer_count', 0)
            protocol = details.get('protocol', 'unknown')
            msg += f" - {peer_count} peers via {protocol}"
        self.info(msg, 'network')
    
    def log_observer_update(self, state_info: Dict):
        """Log observer state updates"""
        self.debug(
            f"State: x={state_info.get('x', 0):.2f}, y={state_info.get('y', 0):.2f}, "
            f"theta={state_info.get('theta', 0):.2f}, v={state_info.get('v', 0):.2f}",
            'observer'
        )
    
    def close(self):
        """Close all loggers and stop async threads"""
        # Stop telemetry logging
        if self.telemetry_active:
            self.telemetry_active = False
            try:
                self.telemetry_queue.put(None, timeout=0.5)
            except:
                pass
            
            if self.telemetry_thread and self.telemetry_thread.is_alive():
                self.telemetry_thread.join(timeout=1.0)
        
        # Close telemetry file
        if self.telemetry_file:
            try:
                self.telemetry_file.close()
            except:
                pass
        
        # Close all async handlers
        for logger in self.loggers.values():
            for handler in logger.handlers:
                if isinstance(handler, AsyncFileHandler):
                    handler.close()
                logger.removeHandler(handler)
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# Factory function for easy integration
def create_vehicle_logger(vehicle_id: int, log_dir: str = "logs", enable_telemetry: bool = True) -> SimpleVehicleLogger:
    """
    Factory function to create a simple vehicle logger
    
    Args:
        vehicle_id: Vehicle identifier
        log_dir: Directory for log files
        enable_telemetry: Whether to enable CSV telemetry logging
    
    Returns:
        Configured SimpleVehicleLogger instance
    """
    logger = SimpleVehicleLogger(vehicle_id, log_dir)
    
    if enable_telemetry:
        logger.setup_telemetry_logging()
    
    return logger


# Example usage for vehicle_logic.py integration:
"""
Usage in vehicle_logic.py:

# Replace existing logger initialization with:
from simple_vehicle_logger import create_vehicle_logger

# In __init__ method:
self.logger = create_vehicle_logger(
    vehicle_id=self.config.network.car_id,
    log_dir="logs",
    enable_telemetry=True
)

# Pass subsystem-specific loggers to components:
self.v2v_communication = V2VCommunication(
    vehicle_id=config.network.car_id,
    logger=self.logger.get_logger('network'),  # Network subsystem
    base_port=8000
)

self.vehicle_observer = VehicleObserver(
    vehicle_id=config.network.car_id,
    logger=self.logger.get_logger('observer'),  # Observer subsystem
    config=observer_config
)

self.state_machine = VehicleStateMachine(
    self, 
    self.logger.get_logger('control')  # Control subsystem
)

self.command_handler = CommandHandler(
    self.logger.get_logger('main'),  # Main subsystem
    config
)

# Usage examples:
self.logger.info("Vehicle initialized successfully")
self.logger.log_state_transition("INIT", "READY")
self.logger.log_network_event("Ground Station connected", {"ip": "192.168.1.100"})
self.logger.log_v2v_status("activated", {"peer_count": 3, "protocol": "UDP"})
self.logger.error("Control error occurred", exception, 'control')
self.logger.log_telemetry(telemetry_data)

# Subsystem-specific logging:
self.logger.info("Main vehicle operation", 'main')
self.logger.warning("Network issue detected", 'network')
self.logger.debug("Observer state updated", 'observer')
self.logger.error("Control command failed", exception, 'control')

# Get subsystem loggers for external components:
network_logger = self.logger.get_logger('network')
observer_logger = self.logger.get_logger('observer')
control_logger = self.logger.get_logger('control')

# Component-specific loggers with child naming:
v2v_logger = self.logger.get_subsystem_logger('network', 'V2VManager')
ekf_logger = self.logger.get_subsystem_logger('observer', 'EKF')

# Close on shutdown:
self.logger.close()

# Subsystem Log File Organization:
logs/
├── vehicle_1_main.log       # Main vehicle operations, commands, state transitions
├── vehicle_1_network.log    # V2V, Ground Station, communication events  
├── vehicle_1_observer.log   # State estimation, sensor fusion, EKF updates
└── vehicle_1_control.log    # Control logic, actuators, steering, throttle

data_logs/run_*/
└── telemetry_vehicle_1.csv  # CSV telemetry data with all subsystem metrics
"""