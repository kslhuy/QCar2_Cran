import os
import logging
from logging.handlers import RotatingFileHandler
from typing import Dict, Optional
import threading
import queue
import time
import sys

class AsyncLoggingHandler(logging.Handler):
    """
    Asynchronous logging handler that uses a queue and background thread
    for improved performance by not blocking the main thread on I/O operations.
    """
    
    def __init__(self, target_handler, queue_size=1000):
        """
        Initialize async logging handler.
        
        Args:
            target_handler: The actual handler (RotatingFileHandler, etc.) to write to
            queue_size: Maximum queue size before blocking
        """
        super().__init__()
        self.target_handler = target_handler
        self.log_queue = queue.Queue(maxsize=queue_size)
        self.worker_thread = None
        self.stop_event = threading.Event()
        self._start_worker()
    
    def _start_worker(self):
        """Start the background logging worker thread."""
        if self.worker_thread is None or not self.worker_thread.is_alive():
            self.worker_thread = threading.Thread(target=self._log_worker, daemon=True, name="AsyncLogWorker")
            self.worker_thread.start()
    
    def _log_worker(self):
        """Background worker that processes log records from the queue."""
        while not self.stop_event.is_set():
            try:
                # Get log record with timeout to allow shutdown checking
                record = self.log_queue.get(timeout=0.1)
                self.target_handler.emit(record)
                self.log_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                # Log to stderr if we can't log normally
                print(f"Async logging error: {e}", file=sys.stderr)
    
    def emit(self, record):
        """Emit log record to queue instead of directly to handler."""
        try:
            self.log_queue.put_nowait(record)
        except queue.Full:
            # If queue is full, drop the log record to prevent blocking
            pass
    
    def flush(self):
        """Flush the target handler."""
        if self.target_handler:
            self.target_handler.flush()
    
    def close(self):
        """Close the handler and stop the worker thread."""
        self.stop_event.set()
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        
        if self.target_handler:
            self.target_handler.close()
        super().close()

class FilteredLoggerAdapter(logging.LoggerAdapter):
    """Logger adapter that respects module-level logging controls."""
    
    def __init__(self, logger, extra, fleet_config, module_type):
        super().__init__(logger, extra)
        self.fleet_config = fleet_config
        self.module_type = module_type
    
    def _should_log(self, level, msg):
        """Check if we should log based on module settings and message content."""
        # Check if module logging is enabled
        if not self.fleet_config.module_logging_enabled.get(self.module_type, True):
            return False
        
        # Check specific message type controls
        msg_str = str(msg)
        
        # Timing logs
        if 'TIMING:' in msg_str or 'EKF_TIMING:' in msg_str or 'DIST_TIMING:' in msg_str or 'OBS_TIMING:' in msg_str:
            return self.fleet_config.module_logging_enabled.get('timing', True)
        
        # State logs
        if 'STATE:' in msg_str or 'OBS_STATE:' in msg_str or 'EKF_STATE:' in msg_str:
            return self.fleet_config.module_logging_enabled.get('state', True)
        
        # Trust logs
        if 'TRUST:' in msg_str or 'TRUST_SCORE:' in msg_str or 'TRUST_EVAL:' in msg_str or 'GRAPH_TRUST:' in msg_str:
            return self.fleet_config.module_logging_enabled.get('trust', True)
        
        # For DEBUG level, allow if the module is enabled (don't require separate 'debug' module)
        # The 'debug' module setting is now only for general debug filtering if needed
        if level == logging.DEBUG:
            return True  # Allow DEBUG if module is enabled
        
        return True
    
    def log(self, level, msg, *args, **kwargs):
        """Override log method to apply filtering."""
        if self._should_log(level, msg):
            super().log(level, msg, *args, **kwargs)
    
    def debug(self, msg, *args, **kwargs):
        if self._should_log(logging.DEBUG, msg):
            super().debug(msg, *args, **kwargs)
    
    def info(self, msg, *args, **kwargs):
        if self._should_log(logging.INFO, msg):
            super().info(msg, *args, **kwargs)
    
    def warning(self, msg, *args, **kwargs):
        if self._should_log(logging.WARNING, msg):
            super().warning(msg, *args, **kwargs)
    
    def error(self, msg, *args, **kwargs):
        if self._should_log(logging.ERROR, msg):
            super().error(msg, *args, **kwargs)

class FleetLoggingConfig:
    """
    Advanced logging configuration for fleet vehicle simulation.
    
    Creates separate log files for:
    - Communication data (send/receive messages)
    - GPS synchronization events
    - Individual vehicle operations
    - Observer timing and operations
    - Control operations
    - General fleet operations
    """
    
    def __init__(self, log_dir: str = "logs", max_bytes: int = 20*1024*1024, backup_count: int = 3):
        """
        Initialize fleet logging configuration.
        
        Args:
            log_dir: Directory to store log files
            max_bytes: Maximum size per log file before rotation (default: 5MB)
            backup_count: Number of backup files to keep (default: 3)
        """
        self.log_dir = log_dir
        self.max_bytes = max_bytes
        self.backup_count = backup_count
        self.show_console = False  # Console output control
        
        # Individual module logging control
        self.module_logging_enabled = {
            'fleet': True,          # General fleet operations
            'communication': True,  # Communication send/receive
            'gps': True,           # GPS synchronization
            'control': True,       # Vehicle control operations
            'observer': True,      # Observer timing and state estimation
            'fleet_observer': True, # Fleet estimation data only
            'vehicle': True,       # Individual vehicle operations
            'trust': True,         # Trust evaluation and scores
            'weight': True,        # Weight calculation and distribution
            'timing': True,        # Performance timing logs
            'state': True,         # State update logs
            'debug': False         # Debug level logs (deprecated - now controlled by individual logger levels)
        }
        
        # Create logs directory if it doesn't exist
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Initialize different loggers
        self.loggers = {}
        self._setup_loggers()
        
    def _setup_loggers(self):
        """Setup all specialized loggers with their respective handlers."""
        
        # Define logger configurations
        logger_configs = {
            'fleet': {
                'filename': 'fleet_operations.log',
                'level': logging.INFO,
                'description': 'General fleet operations and coordination'
            },
            'communication': {
                'filename': 'communication_data.log', 
                'level': logging.INFO,
                'description': 'Data send/receive messages and ACK operations'
            },
            'gps': {
                'filename': 'gps_sync.log',
                'level': logging.INFO, 
                'description': 'GPS synchronization events and time coordination'
            },
            'control': {
                'filename': 'vehicle_control.log',
                'level': logging.INFO,
                'description': 'Vehicle control logic and command execution'
            }
        }
        
        # Create formatters
        self.formatters = {
            'standard': logging.Formatter(
                '%(asctime)s [%(levelname)s] %(name)s: %(message)s'
            ),
            'vehicle': logging.Formatter(
                '%(asctime)s [%(levelname)s] V%(vehicle_id)s: %(message)s', 
                defaults={'vehicle_id': 'N/A'}
            ),
            'communication': logging.Formatter(
                '%(asctime)s [%(levelname)s] V%(vehicle_id)s [COMM]: %(message)s',
                defaults={'vehicle_id': 'N/A'}
            ),
            'gps': logging.Formatter(
                '%(asctime)s [%(levelname)s] V%(vehicle_id)s [GPS]: %(message)s',
                defaults={'vehicle_id': 'N/A'}
            ),
            'fleet_observer': logging.Formatter(
                '%(asctime)s [%(levelname)s] V%(vehicle_id)s [FLEET_OBS]: %(message)s',
                defaults={'vehicle_id': 'N/A'}
            ),
            'trust': logging.Formatter(
                '%(asctime)s [%(levelname)s] V%(vehicle_id)s [TRUST]: %(message)s',
                defaults={'vehicle_id': 'N/A'}
            ),
            'weight': logging.Formatter(
                '%(asctime)s [%(levelname)s] V%(vehicle_id)s [WEIGHT]: %(message)s',
                defaults={'vehicle_id': 'N/A'}
            ),
            'concise': logging.Formatter(
                '%(message)s'  # Only the message, no timestamp for concise logs
            )
        }
        
        # Setup each logger
        for logger_name, config in logger_configs.items():
            self._create_logger(logger_name, config)
            
    def _create_logger(self, logger_name: str, config: dict):
        """Create and configure a specific logger."""
        
        # Create logger
        logger = logging.getLogger(f"fleet.{logger_name}")
        logger.setLevel(logging.WARNING)  # Reduced from INFO to WARNING for performance
        
        # Clear existing handlers to avoid duplicates
        logger.handlers.clear()
        logger.propagate = False
        
        # Create log file path
        log_file_path = os.path.join(self.log_dir, config['filename'])
        
        # Truncate existing log file for fresh start
        if os.path.exists(log_file_path):
            try:
                with open(log_file_path, 'w') as f:
                    f.truncate(0)
                print(f"Truncated existing {config['filename']} for new run")
            except Exception as e:
                print(f"Failed to truncate {config['filename']}: {e}")
        
        # Create rotating file handler
        file_handler = RotatingFileHandler(
            log_file_path, 
            maxBytes=self.max_bytes, 
            backupCount=self.backup_count
        )
        
        # Wrap with async handler for better performance
        async_handler = AsyncLoggingHandler(file_handler, queue_size=2000)
        
        # Choose appropriate formatter
        if logger_name == 'communication':
            async_handler.setFormatter(self.formatters['communication'])
        elif logger_name == 'gps':
            async_handler.setFormatter(self.formatters['gps'])
        elif logger_name == 'trust':
            async_handler.setFormatter(self.formatters['trust'])
        elif logger_name == 'weight':
            async_handler.setFormatter(self.formatters['weight'])
        else:
            async_handler.setFormatter(self.formatters['vehicle'])
        
        logger.addHandler(async_handler)
        
        # Add console handler if enabled
        if self.show_console:
            console_handler = logging.StreamHandler()
            console_handler.setFormatter(async_handler.formatter)
            logger.addHandler(console_handler)
        
        # Store logger reference
        self.loggers[logger_name] = logger
        
        # Log initialization
        logger.info(f"Logger '{logger_name}' initialized - {config['description']}")
    
    def get_vehicle_logger(self, vehicle_id: int, subsystem: str = 'fleet') -> FilteredLoggerAdapter:
        """
        Get a logger adapter for a specific vehicle and subsystem.
        
        Args:
            vehicle_id: Unique vehicle identifier
            subsystem: Subsystem type ('fleet', 'communication', 'gps', 'control')
            
        Returns:
            FilteredLoggerAdapter with vehicle_id context and module filtering
        """
        if subsystem not in self.loggers:
            subsystem = 'fleet'  # Fallback to fleet logger
            
        base_logger = self.loggers[subsystem]
        return FilteredLoggerAdapter(base_logger, {'vehicle_id': vehicle_id}, self, subsystem)
    
    def get_individual_vehicle_logger(self, vehicle_id: int) -> FilteredLoggerAdapter:
        """
        Create a dedicated logger for an individual vehicle with its own log file.
        
        Args:
            vehicle_id: Unique vehicle identifier
            
        Returns:
            FilteredLoggerAdapter for the specific vehicle
        """
        logger_name = f"vehicle_{vehicle_id}"
        
        # Create individual vehicle logger if it doesn't exist
        if logger_name not in self.loggers:
            vehicle_logger = logging.getLogger(f"fleet.{logger_name}")
            vehicle_logger.setLevel(logging.INFO)
            vehicle_logger.handlers.clear()
            vehicle_logger.propagate = False
            
            # Create vehicle-specific log file
            log_file_path = os.path.join(self.log_dir, f"vehicle_{vehicle_id}.log")
            
            # Truncate existing log file
            if os.path.exists(log_file_path):
                try:
                    with open(log_file_path, 'w') as f:
                        f.truncate(0)
                except Exception as e:
                    print(f"Failed to truncate vehicle_{vehicle_id}.log: {e}")
            
            # Setup file handler
            file_handler = RotatingFileHandler(
                log_file_path,
                maxBytes=self.max_bytes,
                backupCount=self.backup_count
            )
            file_handler.setFormatter(self.formatters['vehicle'])
            vehicle_logger.addHandler(file_handler)
            
            # Add console handler if enabled
            if self.show_console:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(self.formatters['vehicle'])
                vehicle_logger.addHandler(console_handler)
            
            # Store logger
            self.loggers[logger_name] = vehicle_logger
            vehicle_logger.info(f"Individual vehicle logger created for Vehicle {vehicle_id}")
        
        return FilteredLoggerAdapter(self.loggers[logger_name], {'vehicle_id': vehicle_id}, self, 'vehicle')
    
    def enable_console_output(self, enabled: bool = True):
        """Enable or disable console output for all loggers."""
        self.show_console = enabled
        
        for logger in self.loggers.values():
            # Remove existing console handlers
            console_handlers = [h for h in logger.handlers if isinstance(h, logging.StreamHandler) and not isinstance(h, RotatingFileHandler)]
            for handler in console_handlers:
                logger.removeHandler(handler)
            
            # Add console handler if enabled
            if enabled:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(logger.handlers[0].formatter if logger.handlers else self.formatters['standard'])
                logger.addHandler(console_handler)
    
    def set_module_logging(self, module: str, enabled: bool):
        """
        Enable or disable logging for a specific module.
        
        Args:
            module: Module name ('fleet', 'communication', 'gps', 'control', 'observer', 'vehicle', 'timing', 'state', 'debug')
            enabled: Whether to enable logging for this module
        """
        if module in self.module_logging_enabled:
            self.module_logging_enabled[module] = enabled
            
            # Update logger levels for all loggers of this type
            if enabled:
                level = logging.INFO if module != 'debug' else logging.DEBUG
            else:
                level = logging.CRITICAL + 1  # Effectively disable logging
            
            # Update relevant loggers
            for logger_name, logger in self.loggers.items():
                if module in logger_name or (module == 'vehicle' and 'vehicle_' in logger_name):
                    logger.setLevel(level)
            
            # print(f"Module '{module}' logging {'enabled' if enabled else 'disabled'}")
        else:
            print(f"Unknown module: {module}. Available modules: {list(self.module_logging_enabled.keys())}")
    
    def set_multiple_modules(self, modules_config: dict):
        """
        Set logging state for multiple modules at once.
        
        Args:
            modules_config: Dictionary of {module_name: enabled_bool}
        """
        for module, enabled in modules_config.items():
            self.set_module_logging(module, enabled)
    
    def disable_all_logging(self):
        """Disable all logging for maximum performance."""
        for module in self.module_logging_enabled.keys():
            self.set_module_logging(module, False)
        # print("All logging disabled for maximum performance")
    
    def enable_all_logging(self):
        """Enable all logging."""
        for module in self.module_logging_enabled.keys():
            self.set_module_logging(module, True)
        print("All logging enabled")
    
    def enable_essential_only(self):
        """Enable only essential logging (errors and warnings)."""
        essential_config = {
            'fleet': False,
            'communication': False,
            'gps': False,
            'control': False,
            'observer': False,
            'vehicle': False,
            'trust': False,
            'timing': False,
            'state': False,
            'debug': False
        }
        self.set_multiple_modules(essential_config)
        
        # Set all loggers to WARNING level for essential-only mode
        for logger in self.loggers.values():
            logger.setLevel(logging.WARNING)
        
        print("Essential logging only (warnings and errors)")
    
    def enable_performance_monitoring(self):
        """Enable only timing and performance related logging."""
        perf_config = {
            'fleet': False,
            'communication': False,
            'gps': False,
            'control': False,
            'observer': True,        # Keep observer timing
            'fleet_observer': True,  # Keep fleet estimation data
            'vehicle': False,
            'trust': True,          # Keep trust evaluation logs
            'timing': True,          # Keep timing logs
            'state': False,
            'debug': False
        }
        self.set_multiple_modules(perf_config)
        print("Performance monitoring logging enabled")
    
    def get_logging_status(self) -> dict:
        """Get current logging status for all modules."""
        return self.module_logging_enabled.copy()
    
    def print_logging_status(self):
        """Print current logging status in a readable format."""
        print("\n=== Fleet Logging Configuration ===")
        for module, enabled in self.module_logging_enabled.items():
            status = "✅ ENABLED " if enabled else "❌ DISABLED"
            print(f"{module.upper():15} | {status}")
        print(f"Console Output:   {'✅ ENABLED' if self.show_console else '❌ DISABLED'}")
        print("=" * 40)
    
    def get_communication_logger(self, vehicle_id: int) -> FilteredLoggerAdapter:
        """
        Get logger specifically for communication events with individual file per vehicle.
        Creates separate communication log files: communication_vehicle_N.log
        """
        logger_name = f"communication_vehicle_{vehicle_id}"
        
        # Create individual communication logger if it doesn't exist
        if logger_name not in self.loggers:
            comm_logger = logging.getLogger(f"fleet.{logger_name}")
            comm_logger.setLevel(logging.INFO)
            comm_logger.handlers.clear()
            comm_logger.propagate = False
            
            # Create vehicle-specific communication log file
            log_file_path = os.path.join(self.log_dir, f"communication_vehicle_{vehicle_id}.log")
            
            # Truncate existing log file
            if os.path.exists(log_file_path):
                try:
                    with open(log_file_path, 'w') as f:
                        f.truncate(0)
                except Exception as e:
                    print(f"Failed to truncate communication_vehicle_{vehicle_id}.log: {e}")
            
            # Setup file handler
            file_handler = RotatingFileHandler(
                log_file_path,
                maxBytes=self.max_bytes,
                backupCount=self.backup_count
            )
            file_handler.setFormatter(self.formatters['communication'])
            comm_logger.addHandler(file_handler)
            
            # Add console handler if enabled
            if self.show_console:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(self.formatters['communication'])
                comm_logger.addHandler(console_handler)
            
            # Store logger
            self.loggers[logger_name] = comm_logger
            comm_logger.info(f"Individual communication logger created for Vehicle {vehicle_id}")
        
        return FilteredLoggerAdapter(self.loggers[logger_name], {'vehicle_id': vehicle_id}, self, 'communication')
    
    def get_gps_logger(self, vehicle_id: int) -> FilteredLoggerAdapter:
        """
        Get logger specifically for GPS synchronization events with individual file per vehicle.
        Creates separate GPS log files: gps_vehicle_N.log
        """
        logger_name = f"gps_vehicle_{vehicle_id}"
        
        # Create individual GPS logger if it doesn't exist
        if logger_name not in self.loggers:
            gps_logger = logging.getLogger(f"fleet.{logger_name}")
            gps_logger.setLevel(logging.INFO)
            gps_logger.handlers.clear()
            gps_logger.propagate = False
            
            # Create vehicle-specific GPS log file
            log_file_path = os.path.join(self.log_dir, f"gps_vehicle_{vehicle_id}.log")
            
            # Truncate existing log file
            if os.path.exists(log_file_path):
                try:
                    with open(log_file_path, 'w') as f:
                        f.truncate(0)
                except Exception as e:
                    print(f"Failed to truncate gps_vehicle_{vehicle_id}.log: {e}")
            
            # Setup file handler
            file_handler = RotatingFileHandler(
                log_file_path,
                maxBytes=self.max_bytes,
                backupCount=self.backup_count
            )
            file_handler.setFormatter(self.formatters['gps'])
            gps_logger.addHandler(file_handler)
            
            # Add console handler if enabled
            if self.show_console:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(self.formatters['gps'])
                gps_logger.addHandler(console_handler)
            
            # Store logger
            self.loggers[logger_name] = gps_logger
            gps_logger.info(f"Individual GPS logger created for Vehicle {vehicle_id}")
        
        return FilteredLoggerAdapter(self.loggers[logger_name], {'vehicle_id': vehicle_id}, self, 'gps')
    
    def get_control_logger(self, vehicle_id: int) -> FilteredLoggerAdapter:
        """
        Get logger specifically for control operations with individual file per vehicle.
        Creates separate control log files: control_vehicle_N.log
        """
        logger_name = f"control_vehicle_{vehicle_id}"
        
        # Create individual control logger if it doesn't exist
        if logger_name not in self.loggers:
            control_logger = logging.getLogger(f"fleet.{logger_name}")
            control_logger.setLevel(logging.INFO)
            control_logger.handlers.clear()
            control_logger.propagate = False
            
            # Create vehicle-specific control log file
            log_file_path = os.path.join(self.log_dir, f"control_vehicle_{vehicle_id}.log")
            
            # Truncate existing log file
            if os.path.exists(log_file_path):
                try:
                    with open(log_file_path, 'w') as f:
                        f.truncate(0)
                except Exception as e:
                    print(f"Failed to truncate control_vehicle_{vehicle_id}.log: {e}")
            
            # Setup file handler
            file_handler = RotatingFileHandler(
                log_file_path,
                maxBytes=self.max_bytes,
                backupCount=self.backup_count
            )
            file_handler.setFormatter(self.formatters['vehicle'])  # Use vehicle formatter for control
            control_logger.addHandler(file_handler)
            
            # Add console handler if enabled
            if self.show_console:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(self.formatters['vehicle'])
                control_logger.addHandler(console_handler)
            
            # Store logger
            self.loggers[logger_name] = control_logger
            control_logger.info(f"Individual control logger created for Vehicle {vehicle_id}")
        
        return FilteredLoggerAdapter(self.loggers[logger_name], {'vehicle_id': vehicle_id}, self, 'control')
    
    def get_observer_logger(self, vehicle_id: int) -> FilteredLoggerAdapter:
        """
        Get logger specifically for observer operations with individual file per vehicle.
        Creates separate observer log files: observer_vehicle_N.log
        """
        logger_name = f"observer_vehicle_{vehicle_id}"
        
        # Create individual observer logger if it doesn't exist
        if logger_name not in self.loggers:
            observer_logger = logging.getLogger(f"fleet.{logger_name}")
            observer_logger.setLevel(logging.INFO)
            observer_logger.handlers.clear()
            observer_logger.propagate = False
            
            # Create vehicle-specific observer log file
            log_file_path = os.path.join(self.log_dir, f"observer_vehicle_{vehicle_id}.log")
            
            # Truncate existing log file
            if os.path.exists(log_file_path):
                try:
                    with open(log_file_path, 'w') as f:
                        f.truncate(0)
                except Exception as e:
                    print(f"Failed to truncate observer_vehicle_{vehicle_id}.log: {e}")
            
            # Setup file handler with detailed formatter for timing analysis
            file_handler = RotatingFileHandler(
                log_file_path,
                maxBytes=self.max_bytes,
                backupCount=self.backup_count
            )
            
            # Create specialized formatter for observer timing
            observer_formatter = logging.Formatter(
                '%(asctime)s.%(msecs)03d [%(levelname)s] V%(vehicle_id)s [OBS]: %(message)s',
                datefmt='%H:%M:%S',
                defaults={'vehicle_id': 'N/A'}
            )
            file_handler.setFormatter(observer_formatter)
            observer_logger.addHandler(file_handler)
            
            # Add console handler if enabled
            if self.show_console:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(observer_formatter)
                observer_logger.addHandler(console_handler)
            
            # Store logger
            self.loggers[logger_name] = observer_logger
            observer_logger.info(f"Individual observer logger created for Vehicle {vehicle_id}")
        
        return FilteredLoggerAdapter(self.loggers[logger_name], {'vehicle_id': vehicle_id}, self, 'observer')
    
    def get_fleet_observer_logger(self, vehicle_id: int) -> FilteredLoggerAdapter:
        """
        Get logger specifically for fleet estimation data with individual file per vehicle.
        Creates separate fleet observer log files: fleet_observer_vehicle_N.log
        This logger is dedicated to distributed observer fleet estimation data only.
        """
        logger_name = f"fleet_observer_vehicle_{vehicle_id}"
        
        # Create individual fleet observer logger if it doesn't exist
        if logger_name not in self.loggers:
            fleet_obs_logger = logging.getLogger(f"fleet.{logger_name}")
            fleet_obs_logger.setLevel(logging.INFO)
            fleet_obs_logger.handlers.clear()
            fleet_obs_logger.propagate = False
            
            # Create vehicle-specific fleet observer log file
            log_file_path = os.path.join(self.log_dir, f"fleet_observer_vehicle_{vehicle_id}.log")
            
            # Truncate existing log file
            if os.path.exists(log_file_path):
                try:
                    with open(log_file_path, 'w') as f:
                        f.truncate(0)
                except Exception as e:
                    print(f"Failed to truncate fleet_observer_vehicle_{vehicle_id}.log: {e}")
            
            # Setup file handler with specialized formatter for fleet estimation
            file_handler = RotatingFileHandler(
                log_file_path,
                maxBytes=self.max_bytes,
                backupCount=self.backup_count
            )
            
            # Create specialized formatter for fleet observer data (time only, no date)
            fleet_observer_formatter = logging.Formatter(
                '%(asctime)s.%(msecs)03d [%(levelname)s] V%(vehicle_id)s [FLEET]: %(message)s',
                datefmt='%H:%M:%S',
                defaults={'vehicle_id': 'N/A'}
            )
            file_handler.setFormatter(fleet_observer_formatter)
            fleet_obs_logger.addHandler(file_handler)
            
            # Add console handler if enabled
            if self.show_console:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(fleet_observer_formatter)
                fleet_obs_logger.addHandler(console_handler)
            
            # Store logger
            self.loggers[logger_name] = fleet_obs_logger
            fleet_obs_logger.info(f"Individual fleet observer logger created for Vehicle {vehicle_id}")
        
        return FilteredLoggerAdapter(self.loggers[logger_name], {'vehicle_id': vehicle_id}, self, 'fleet_observer')
    
    def get_trust_logger(self, vehicle_id: int) -> FilteredLoggerAdapter:
        """
        Get logger specifically for trust evaluation and scoring with individual file per vehicle.
        Creates separate trust log files: trust_vehicle_N.log
        This logger is dedicated to trust model operations and trust scores.
        """
        logger_name = f"trust_vehicle_{vehicle_id}"
        
        # Create individual trust logger if it doesn't exist
        if logger_name not in self.loggers:
            trust_logger = logging.getLogger(f"fleet.{logger_name}")
            trust_logger.setLevel(logging.INFO)
            trust_logger.handlers.clear()
            trust_logger.propagate = False
            
            # Create trust-specific log file
            log_file_path = os.path.join(self.log_dir, f"trust_vehicle_{vehicle_id}.log")
            
            # Truncate existing log file
            if os.path.exists(log_file_path):
                try:
                    with open(log_file_path, 'w') as f:
                        f.truncate(0)
                except Exception as e:
                    print(f"Failed to truncate trust_vehicle_{vehicle_id}.log: {e}")
            
            # Setup file handler with specialized formatter for trust data
            file_handler = RotatingFileHandler(
                log_file_path,
                maxBytes=self.max_bytes,
                backupCount=self.backup_count
            )
            
            # Create specialized formatter for trust data
            trust_formatter = logging.Formatter(
                '%(asctime)s.%(msecs)03d [%(levelname)s] V%(vehicle_id)s [TRUST]: %(message)s',
                datefmt='%H:%M:%S',
                defaults={'vehicle_id': 'N/A'}
            )
            file_handler.setFormatter(trust_formatter)
            trust_logger.addHandler(file_handler)
            
            # Add console handler if enabled
            if self.show_console:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(trust_formatter)
                trust_logger.addHandler(console_handler)
            
            # Store logger
            self.loggers[logger_name] = trust_logger
            trust_logger.info(f"Individual trust logger created for Vehicle {vehicle_id}")
        
        return FilteredLoggerAdapter(self.loggers[logger_name], {'vehicle_id': vehicle_id}, self, 'trust')
    
    def get_weight_logger(self, vehicle_id: int) -> FilteredLoggerAdapter:
        """
        Get logger specifically for weight calculation and distribution with individual file per vehicle.
        Creates separate weight log files: weight_vehicle_N.log
        This logger is dedicated to adaptive weight calculations and consensus weight updates.
        """
        logger_name = f"weight_vehicle_{vehicle_id}"
        
        # Create individual weight logger if it doesn't exist
        if logger_name not in self.loggers:
            weight_logger = logging.getLogger(f"fleet.{logger_name}")
            weight_logger.setLevel(logging.INFO)
            weight_logger.handlers.clear()
            weight_logger.propagate = False
            
            # Create weight-specific log file
            log_file_path = os.path.join(self.log_dir, f"weight_vehicle_{vehicle_id}.log")
            
            # Truncate existing log file
            if os.path.exists(log_file_path):
                try:
                    with open(log_file_path, 'w') as f:
                        f.truncate(0)
                except Exception as e:
                    print(f"Failed to truncate weight_vehicle_{vehicle_id}.log: {e}")
            
            # Setup file handler with specialized formatter for weight data
            file_handler = RotatingFileHandler(
                log_file_path,
                maxBytes=self.max_bytes,
                backupCount=self.backup_count
            )
            
            # Create specialized formatter for weight data
            weight_formatter = logging.Formatter(
                '%(asctime)s.%(msecs)03d [%(levelname)s] V%(vehicle_id)s [WEIGHT]: %(message)s',
                datefmt='%H:%M:%S',
                defaults={'vehicle_id': 'N/A'}
            )
            file_handler.setFormatter(weight_formatter)
            weight_logger.addHandler(file_handler)
            
            # Add console handler if enabled
            if self.show_console:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(weight_formatter)
                weight_logger.addHandler(console_handler)
            
            # Store logger
            self.loggers[logger_name] = weight_logger
            weight_logger.info(f"Individual weight logger created for Vehicle {vehicle_id}")
        
        return FilteredLoggerAdapter(self.loggers[logger_name], {'vehicle_id': vehicle_id}, self, 'weight')
    
    def cleanup_all_loggers(self):
        """Clean up all loggers and their handlers. Call this when shutting down the application."""
        print("Cleaning up all logging handlers...")
        
        for logger_name, logger in self.loggers.items():
            try:
                # Close all handlers for this logger
                for handler in logger.handlers:
                    try:
                        handler.close()
                    except Exception as e:
                        print(f"Error closing handler for {logger_name}: {e}")
                
                # Clear handlers list
                logger.handlers.clear()
                
            except Exception as e:
                print(f"Error cleaning up logger {logger_name}: {e}")
        
        # Clear the loggers dictionary
        self.loggers.clear()
        
        print("All logging handlers cleaned up")
    
    def __del__(self):
        """Destructor to ensure cleanup on garbage collection."""
        try:
            self.cleanup_all_loggers()
        except:
            pass  # Ignore errors during cleanup

# Global fleet logging configuration instance
fleet_logging = FleetLoggingConfig()

# Convenience functions for backward compatibility
def get_logger(vehicle_id: int) -> FilteredLoggerAdapter:
    """Get general fleet logger for a vehicle (backward compatibility)."""
    return fleet_logging.get_fleet_logger(vehicle_id)

def get_communication_logger(vehicle_id: int) -> FilteredLoggerAdapter:
    """Get communication-specific logger for a vehicle."""
    return fleet_logging.get_communication_logger(vehicle_id)

def get_gps_logger(vehicle_id: int) -> FilteredLoggerAdapter:
    """Get GPS-specific logger for a vehicle."""
    return fleet_logging.get_gps_logger(vehicle_id)

def get_individual_vehicle_logger(vehicle_id: int) -> FilteredLoggerAdapter:
    """Get individual vehicle logger with dedicated log file."""
    return fleet_logging.get_individual_vehicle_logger(vehicle_id)

def get_control_logger(vehicle_id: int) -> FilteredLoggerAdapter:
    """Get control-specific logger for a vehicle."""
    return fleet_logging.get_control_logger(vehicle_id)

def get_observer_logger(vehicle_id: int) -> FilteredLoggerAdapter:
    """Get observer-specific logger for a vehicle."""
    return fleet_logging.get_observer_logger(vehicle_id)

def get_fleet_observer_logger(vehicle_id: int) -> FilteredLoggerAdapter:
    """Get fleet observer-specific logger for a vehicle."""
    return fleet_logging.get_fleet_observer_logger(vehicle_id)

def get_trust_logger(vehicle_id: int) -> FilteredLoggerAdapter:
    """Get trust evaluation-specific logger for a vehicle."""
    return fleet_logging.get_trust_logger(vehicle_id)

def get_weight_logger(vehicle_id: int) -> FilteredLoggerAdapter:
    """Get weight calculation-specific logger for a vehicle."""
    return fleet_logging.get_weight_logger(vehicle_id)

def enable_console_logging(enabled: bool = True):
    """Enable or disable console output for all loggers."""
    fleet_logging.enable_console_output(enabled)

# Module control functions for easy access
def set_module_logging(module: str, enabled: bool):
    """Enable or disable logging for a specific module."""
    fleet_logging.set_module_logging(module, enabled)

def disable_all_logging():
    """Disable all logging for maximum performance."""
    fleet_logging.disable_all_logging()

def enable_all_logging():
    """Enable all logging."""
    fleet_logging.enable_all_logging()

def enable_essential_only():
    """Enable only essential logging (errors and warnings)."""
    fleet_logging.enable_essential_only()

def enable_performance_monitoring():
    """Enable only timing and performance related logging."""
    fleet_logging.enable_performance_monitoring()

def print_logging_status():
    """Print current logging status in a readable format."""
    fleet_logging.print_logging_status()

def cleanup_all_logging():
    """Clean up all logging handlers. Call this when shutting down the application."""
    fleet_logging.cleanup_all_loggers()

# Backward compatibility - default logger
logger = fleet_logging.loggers.get('fleet', logging.getLogger(__name__))