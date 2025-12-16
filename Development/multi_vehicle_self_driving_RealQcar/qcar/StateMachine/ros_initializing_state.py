"""
ROS-Specific Initializing State

This is a simplified initialization state for ROS 2 integration that:
1. Skips QCar/GPS hardware initialization (ROS topics provide sensor data)
2. Only initializes path planning and perception
3. Waits for ROS topic readiness signals
4. Transitions faster since hardware is already "connected" via ROS adapters

Use this as a drop-in replacement for InitializingState when running under ROS.
"""

import time
import numpy as np
from typing import Dict, Any, Tuple, Optional

from StateMachine.state_base import StateBase
from StateMachine.vehicle_state import VehicleState, StateTransitionReason
from hal.products.mats import SDCSRoadMap
from Yolo.YoLo import YOLOReceiver, YOLODriveLogic
from pal.products.qcar import IS_PHYSICAL_QCAR


class ROSInitializingState(StateBase):
    """ROS-specific initialization state - skips hardware init"""
    
    # Faster timing for ROS (hardware already connected)
    INITIAL_DELAY = 0.5  # Reduced from 1.0
    STEP_DELAY = 0.2     # Reduced from 0.5
    TIMEOUT = 15.0       # Reduced from 30.0
    
    def enter(self) -> bool:
        """Initialize system components for ROS environment"""
        super().enter()
        self.logger.logger.info("=" * 70)
        self.logger.logger.info("Entering ROS INITIALIZING state")
        self.logger.logger.info("(Hardware init skipped - using ROS adapters)")
        self.logger.logger.info("=" * 70)
        
        self.state_data = {
            'initialization_start': time.time(),
            'components_initialized': False,
            'ready_to_start': False,
            'last_step_time': time.time(),
            'ros_topics_ready': False
        }
        
        return True
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Handle ROS initialization process"""
        throttle, steering = 0.0, 0.0
        current_time = time.time()
        elapsed_time = current_time - self.state_data['initialization_start']
        
        # Check for timeout
        if elapsed_time > self.TIMEOUT:
            self.logger.log_warning(f"[!] Initialization timeout ({self.TIMEOUT}s), proceeding anyway")
            return throttle, steering, (VehicleState.WAITING_FOR_START, StateTransitionReason.INITIALIZATION_COMPLETE)
        
        # Wait for initial system settle time
        if elapsed_time < self.INITIAL_DELAY:
            return throttle, steering, None
        
        # Check if ROS topics are ready (set by VehicleControlFullSystem node)
        if not self.state_data['ros_topics_ready']:
            # Check if running under ROS mode
            if hasattr(self.vehicle_logic, '_ros_mode') and self.vehicle_logic._ros_mode:
                # Wait for ROS node to signal readiness
                # The ROS node will set this flag when topics are connected
                if hasattr(self.vehicle_logic, '_ros_topics_ready'):
                    if self.vehicle_logic._ros_topics_ready:
                        self.state_data['ros_topics_ready'] = True
                        self.logger.logger.info("✓ ROS topics connected")
                else:
                    # Assume ready after initial delay if flag doesn't exist
                    self.state_data['ros_topics_ready'] = True
                    self.logger.logger.info("✓ ROS mode active (assuming topics ready)")
            else:
                # Not in ROS mode, proceed normally
                self.state_data['ros_topics_ready'] = True
            
            if not self.state_data['ros_topics_ready']:
                # Still waiting for ROS topics
                if elapsed_time % 2.0 < 0.1:
                    self.logger.logger.info("Waiting for ROS topics to connect...")
                return throttle, steering, None
        
        # Initialize components (skip hardware, only path planning and perception)
        if not self.state_data['components_initialized']:
            if self._initialize_ros_components():
                self.state_data['components_initialized'] = True
                self.state_data['last_step_time'] = current_time
                self.logger.logger.info("[STEP 1/2] ROS components initialized")
            else:
                self.logger.log_error("Component initialization failed")
                return throttle, steering, None
        
        # Finalize and transition
        if self.state_data['components_initialized'] and not self.state_data['ready_to_start']:
            if current_time - self.state_data['last_step_time'] < self.STEP_DELAY:
                return throttle, steering, None
            
            self.state_data['ready_to_start'] = True
            self.logger.logger.info("[STEP 2/2] System ready to start")
            time.sleep(0.2)
        
        # Transition to WAITING_FOR_START
        if self.state_data['ready_to_start']:
            self.logger.logger.info("=" * 70)
            self.logger.logger.info(f"✓ ROS Initialization complete in {elapsed_time:.1f}s")
            self.logger.logger.info("=" * 70)
            return throttle, steering, (VehicleState.WAITING_FOR_START, StateTransitionReason.INITIALIZATION_COMPLETE)
        
        return throttle, steering, None
    
    def _initialize_ros_components(self) -> bool:
        """Initialize only non-hardware components for ROS"""
        initialization_steps = [
            ("Path planning", self._initialize_path_planning, 0.1),
            ("Perception (optional)", self._initialize_perception, 0.1),
            ("Observer readiness", self._check_observer_ready, 0.1)
        ]
        
        try:
            self.logger.logger.info("Starting ROS component initialization...")
            
            for idx, (component_name, init_func, settle_time) in enumerate(initialization_steps, 1):
                self.logger.logger.info(f"[{idx}/{len(initialization_steps)}] Initializing {component_name}...")
                if not init_func():
                    self.logger.log_error(f"Failed to initialize {component_name}")
                    return False
                time.sleep(settle_time)
            
            self.logger.logger.info(f"All {len(initialization_steps)} components initialized!")
            return True
            
        except Exception as e:
            self.logger.log_error("ROS component initialization failed", e)
            return False
    
    def _initialize_path_planning(self) -> bool:
        """Initialize path planning system"""
        if not self.vehicle_logic.controller_config.enable_steering_control:
            self.logger.logger.info("Steering control disabled, skipping path planning")
            return True
        
        try:
            # Create roadmap
            self.vehicle_logic.roadmap = SDCSRoadMap(
                leftHandTraffic=self.config.path.left_hand_traffic,
                useSmallMap=True
            )
            
            # Set node sequence
            self.vehicle_logic.node_sequence = self.config.path.valid_nodes
            self.logger.logger.info(
                f"Node sequence ({len(self.vehicle_logic.node_sequence)} nodes): "
                f"{self.vehicle_logic.node_sequence}"
            )
            
            # Generate waypoints
            waypoints = self.vehicle_logic.roadmap.generate_path(self.vehicle_logic.node_sequence)
            if waypoints is None or waypoints.shape[0] < 2 or waypoints.shape[1] < 2:
                self.logger.log_error(f"Invalid waypoint generation")
                return False
            
            self.vehicle_logic.waypoint_sequence = waypoints
            self.logger.logger.info(f"Generated path with {waypoints.shape[1]} waypoints")
            return True
            
        except Exception as e:
            self.logger.log_error("Path planning initialization failed", e)
            return False
    
    def _initialize_perception(self) -> bool:
        """Initialize perception systems (YOLO) - optional"""
        try:
            # YOLO is optional for ROS
            yolo_receiver = None  # Can be None
            
            pulse_length = (
                self.config.timing.controller_update_rate *
                self.config.yolo.pulse_length_multiplier
            )
            yolo_drive = YOLODriveLogic(pulseLength=pulse_length)
            
            self.vehicle_logic.yolo_manager.initialize(yolo_receiver, yolo_drive)
            self.logger.logger.info("Perception initialized (YOLO optional)")
            return True
            
        except Exception as e:
            # YOLO is optional - log warning but continue
            self.logger.log_warning(f"YOLO initialization skipped: {e}")
            return True  # Not critical
    
    def _check_observer_ready(self) -> bool:
        """Check if VehicleObserver is ready (should be initialized by ROS adapters)"""
        try:
            if not hasattr(self.vehicle_logic, 'vehicle_observer'):
                self.logger.log_error("VehicleObserver not found")
                return False
            
            # In ROS mode, observer should be initialized with GPS adapter
            # Check if it has received initial pose data
            if hasattr(self.vehicle_logic, 'gps') and self.vehicle_logic.gps:
                gps_data = self.vehicle_logic.gps.read()
                if gps_data and len(gps_data) >= 2:
                    x, y = gps_data[0], gps_data[1]
                    
                    # Initialize observer with GPS data if not already done
                    if not hasattr(self.vehicle_logic.vehicle_observer, '_initialized'):
                        self.logger.logger.info(f"Initializing observer with GPS: ({x:.2f}, {y:.2f})")
                        
                        # Create initial pose
                        self.init_pose = np.array([x, y, 0.0])  # [x, y, theta]
                        
                        # Initialize observer
                        success = self.vehicle_logic.vehicle_observer.initialize_local_estimator(
                            gps=self.vehicle_logic.gps,
                            initial_pose=self.init_pose,
                            estimator_params={'use_qcar_ekf': False}  # Use simple estimator for ROS
                        )
                        
                        if success:
                            self.vehicle_logic.vehicle_observer._initialized = True
                            self.logger.logger.info("Observer initialized with ROS GPS adapter")
                        else:
                            self.logger.log_warning("Observer initialization returned False, but continuing")
            
            self.logger.logger.info("Observer ready for ROS operation")
            return True
            
        except Exception as e:
            self.logger.log_warning(f"Observer readiness check issue: {e}")
            return True  # Continue anyway, observer will initialize on first update
    
    def handle_event(self, command_type, data: Dict[str, Any] = None) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """Handle commands during initialization"""
        try:
            from command_handler import CommandType
            
            if command_type == CommandType.EMERGENCY_STOP:
                self.logger.logger.warning("[!] Emergency stop during initialization")
                return (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
            
            self.logger.logger.info(f"Ignoring '{command_type}' command during initialization")
            return None
            
        except ImportError:
            return super().handle_event(command_type, data)
    
    def exit(self):
        """Clean up initialization state"""
        init_time = self.get_time_in_state()
        self.logger.logger.info(f"[ROS INIT] Exiting - completed in {init_time:.1f}s")
        super().exit()
