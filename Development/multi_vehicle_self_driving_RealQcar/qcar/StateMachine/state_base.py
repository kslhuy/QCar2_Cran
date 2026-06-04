"""
Base State Class for Event-Driven State Machine

All states inherit from this base class and implement:
- enter, update, exit methods for state lifecycle
- handle_event method to respond to commands and trigger direct transitions

This is much simpler than the previous approach - events trigger immediate
transitions without needing pending_transition mechanisms.
"""
from typing import Dict, Any, Tuple, Optional
from .vehicle_state import VehicleState, StateTransitionReason
import time
import sys
import os

from pal.products.qcar import QCarGPS, IS_PHYSICAL_QCAR
import numpy as np

# Add parent directory to sys.path to import command_types
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from command_types import CommandType


class StateBase:
    """Base class for all vehicle states with direct event-driven transitions"""
    
    def __init__(self, vehicle_logic):
        """
        Initialize state with reference to vehicle controller
        
        Args:
            vehicle_logic: Reference to main VehicleLogic instance
        """
        self.vehicle_logic = vehicle_logic
        self.logger = vehicle_logic.vehicle_logger # Use vehicle's logger
        self.config = vehicle_logic.config
        
        # State-specific data
        self.state_entry_time = None
        self.state_data = {}
    
    def enter(self) -> bool:
        """
        Called when entering this state
        
        Returns:
            bool: True if successful, False to abort transition
        """
        self.state_entry_time = time.time()
        self.state_data = {}
        return True
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """
        Called every control loop iteration while in this state
        
        Args:
            dt: Time delta since last update
            sensor_data: Dictionary containing all sensor readings
        
        Returns:
            Tuple containing:
            - float: throttle_command
            - float: steering_command  
            - Optional[Tuple[VehicleState, StateTransitionReason]]: Next state transition if needed
        """

        
        # Base implementation - no movement, no transition
        return 0.0, 0.0, None
    
    def exit(self):
        """Called when exiting this state"""
        pass
    
    def _init_controllers(self):
        """
        Initialize controllers specific to this state.
        Override this in subclasses to initialize state-specific controllers.
        Called once during state initialization.
        """
        pass
    
    def get_time_in_state(self) -> float:
        """Get time spent in current state"""
        import time
        if self.state_entry_time:
            return time.time() - self.state_entry_time
        return 0.0
    
    # === Single Event Handler Method ===
    
    def handle_event(self, command_type, data: Dict[str, Any] = None) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """
        Handle a command type and optionally trigger a state transition
        
        Args:
            command_type: CommandType enum (e.g., CommandType.START, CommandType.STOP)
            data: Optional event data
            
        Returns:
            Optional[Tuple[VehicleState, StateTransitionReason]]: 
                - None if event not handled or no transition needed
                - (new_state, reason) if transition should occur
        """
        data = data or {}
        
        # Log the event
        if self.logger:
            self.logger.logger.info(f"[CMD] State {self.__class__.__name__} received command: {command_type}")
        
        # Handle common events that most states should support
        if command_type == CommandType.STOP:
            # Get the source/reason from the data if available
            source = data.get('source', 'Ground Station')
            if self.logger:
                self.logger.logger.info(f"[STOP] Stop command from {source} accepted in {self.__class__.__name__}")
            return (VehicleState.STOPPED, StateTransitionReason.STOP_COMMAND)
        
        elif command_type == CommandType.EMERGENCY_STOP:
            reason = data.get('reason', 'Emergency command')
            if self.logger:
                self.logger.logger.warning(f"[!] Emergency stop ({reason}) accepted in {self.__class__.__name__}")
            return (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
        
        elif command_type == CommandType.SET_VELOCITY:
            # Handle velocity updates without transitioning
            v_ref = data.get('v_ref')
            if v_ref is not None and 0 <= v_ref <= 2.0:
                if hasattr(self.vehicle_logic, 'v_ref'):
                    self.vehicle_logic.v_ref = v_ref
                    if self.logger:
                        self.logger.logger.info(f"[OK] Velocity updated to {v_ref} in {self.__class__.__name__}")
                    return None  # No state transition
            else:
                if self.logger:
                    self.logger.logger.warning(f"[!] Invalid velocity: {v_ref}")
            return None
        
        elif command_type == CommandType.SET_PATH:
            # Handle path updates without transitioning
            node_sequence = data.get('node_sequence')
            if node_sequence and isinstance(node_sequence, list):
                # Generate waypoints from node sequence using roadmap
                if hasattr(self.vehicle_logic, 'roadmap') and self.vehicle_logic.roadmap:
                    try:
                        new_waypoints = self.vehicle_logic.roadmap.generate_path(node_sequence)
                        self.vehicle_logic.waypoint_sequence = new_waypoints
                        
                        # Update steering controller if it exists
                        if hasattr(self.vehicle_logic, 'steering_controller') and self.vehicle_logic.steering_controller:
                            self.vehicle_logic.steering_controller.reset(new_waypoints)
                        
                        if self.logger:
                            self.logger.logger.info(f"[OK] Path updated with {len(node_sequence)} nodes in {self.__class__.__name__}")
                        return None
                    except Exception as e:
                        if self.logger:
                            self.logger.log_error("Failed to generate path from nodes", e)
                else:
                    if self.logger:
                        self.logger.logger.warning("[!] No roadmap available for path generation")
            else:
                if self.logger:
                    self.logger.logger.warning(f"[!] Invalid path update data")
            return None
        
        elif command_type == CommandType.ACTIVATE_V2V:
            # Handle V2V activation
            peer_vehicles = data.get('peer_vehicles', [])
            peer_ips = data.get('peer_ips', [])
            
            if peer_vehicles and peer_ips:
                if hasattr(self.vehicle_logic, 'v2v_manager'):
                    success = self.vehicle_logic.v2v_manager.activate_v2v(peer_vehicles, peer_ips)
                    if success and self.logger:
                        self.logger.logger.info(f"[OK] V2V activated in {self.__class__.__name__}")
                    elif self.logger:
                        self.logger.logger.warning(f"[!] V2V activation failed in {self.__class__.__name__}")
                    return None  # No state transition
            else:
                if self.logger:
                    self.logger.logger.warning(f"[!] Invalid V2V activation data")
            return None
        
        elif command_type == CommandType.SETUP_PLATOON_FORMATION:
            # Handle global platoon formation setup (available in any state)
            print(f"\n[DEBUG] ===== SETUP_PLATOON_FORMATION RECEIVED =====\n")
            print(f"[DEBUG] Formation setup command received in {self.__class__.__name__}")
            print(f"[DEBUG] Data: {data}")
            
            if not self.validate_event_data(data, ['formation', 'leader_id']):
                print(f"[DEBUG] Formation setup failed validation")
                return None
            
            formation = data.get('formation')  # Dict[vehicle_id, position]
            leader_id = data.get('leader_id')
            my_vehicle_id = getattr(self.vehicle_logic, 'vehicle_id', 0)
            
            if self.logger:
                self.logger.logger.info(f"Global platoon formation received:")
                for vehicle_id, position in formation.items():
                    role = "LEADER" if position == 1 else f"FOLLOWER-{position}"
                    marker = " ← ME" if vehicle_id == my_vehicle_id else ""
                    self.logger.logger.info(f"  Vehicle ID {vehicle_id}: Position {position} ({role}){marker}")
            
            # Update vehicle_logic with new position (handle JSON string keys)
            vehicle_found = False
            new_position = None
            
            # Try both integer and string keys due to JSON conversion
            if my_vehicle_id in formation:
                new_position = formation[my_vehicle_id]
                vehicle_found = True
            elif str(my_vehicle_id) in formation:
                new_position = formation[str(my_vehicle_id)]
                vehicle_found = True
            
            if vehicle_found and new_position is not None:
                if hasattr(self.vehicle_logic, 'vehicle_position'):
                    self.vehicle_logic.vehicle_position = new_position
                    if self.logger:
                        self.logger.logger.info(f"Updated vehicle position to {new_position}")
                else:
                    if self.logger:
                        self.logger.logger.info(f"No vehicle_position attribute - using formation position {new_position}")
            else:
                if self.logger:
                    self.logger.logger.warning(f"Vehicle ID {my_vehicle_id} not found in formation: {formation}")
            
            # Update V2V manager with formation mapping
            if hasattr(self.vehicle_logic, 'v2v_manager') and self.vehicle_logic.v2v_manager:
                self.vehicle_logic.v2v_manager.update_platoon_formation(formation)
            
            # Configure platoon controller from global formation
            if hasattr(self.vehicle_logic, 'platoon_controller'):
                print(f"[DEBUG] Calling setup_from_global_formation: car_id={my_vehicle_id}, formation={formation}, leader_id={leader_id}")
                
                # Use the proper setup_from_global_formation method
                setup_success = self.vehicle_logic.platoon_controller.setup_from_global_formation(
                    my_vehicle_id, formation, leader_id
                )
                
                if setup_success:
                    if self.logger:
                        self.logger.logger.info(f"Formation configured successfully in {self.__class__.__name__}")
                        self.logger.logger.info(f"Vehicle {my_vehicle_id} position: {getattr(self.vehicle_logic.platoon_controller, 'my_position', 'unknown')}")
                        self.logger.logger.info(f"Is leader: {self.vehicle_logic.platoon_controller.is_leader}")
                        self.logger.logger.info(f"Leader car ID: {self.vehicle_logic.platoon_controller.leader_car_id}")
                    
                    # Send platoon setup confirmation to Ground Station
                    self._send_platoon_setup_confirmation(my_vehicle_id, formation, leader_id)
                else:
                    if self.logger:
                        self.logger.logger.error(f"Failed to configure formation for vehicle {my_vehicle_id}")
            else:
                if self.logger:
                    self.logger.logger.warning("No platoon controller available")
            
            return None  # No transition, just configuration
        
        elif command_type == CommandType.DISABLE_PLATOON:
            # Handle platoon disable - NOTE: This only pauses platoon operation
            # Formation/position/role data is PRESERVED so platoon can restart
            if hasattr(self.vehicle_logic, 'platoon_controller'):
                self.vehicle_logic.platoon_controller.disable()
                if self.logger:
                    self.logger.logger.info(f"[OK] Platoon paused (configuration preserved) in {self.__class__.__name__}")
            return None
        
        elif command_type == CommandType.DISABLE_V2V:
            # Handle V2V deactivation
            if hasattr(self.vehicle_logic, 'v2v_manager'):
                success = self.vehicle_logic.v2v_manager.disable_v2v()
                
                # Also reset fleet estimation to clean up state
                if hasattr(self.vehicle_logic, 'vehicle_observer') and self.vehicle_logic.vehicle_observer:
                    self.vehicle_logic.vehicle_observer.reset_fleet_estimation()
                
                if success and self.logger:
                    self.logger.logger.info(f"[OK] V2V disabled in {self.__class__.__name__}")
                elif self.logger:
                    self.logger.logger.warning(f"[!] V2V disable failed in {self.__class__.__name__}")
                return None  # No state transition
            return None
        
        elif command_type == CommandType.ACTIVATE_PERCEPTION:
            self.logger.logger.info(f"[CMD] Activating perception system (YOLO)")
            try:
                success = self._activate_perception_system()
                if success:
                    self.logger.logger.info(f"[CMD] Perception system activated successfully")
                else:
                    self.logger.log_error("[CMD] Failed to activate perception system")
            except Exception as e:
                self.logger.log_error("[CMD] Error activating perception", e)
            return None
        
        elif command_type == CommandType.DISABLE_PERCEPTION:
            self.logger.logger.info(f"[CMD] Disabling perception system")
            try:
                self._disable_perception_system()
                self.logger.logger.info(f"[CMD] Perception system disabled")
            except Exception as e:
                self.logger.log_error("[CMD] Error disabling perception", e)
            return None
        
        elif command_type == CommandType.ACTIVATE_SCOPES:
            self.logger.logger.info(f"[CMD] Activating estimation scopes")
            try:
                preset_names = data.get('preset_names', ['local_state', 'local_control'])
                success = self._activate_estimation_scopes(preset_names)
                if success:
                    self.logger.logger.info(f"[CMD] Estimation scopes activated: {preset_names}")
                else:
                    self.logger.log_error("[CMD] Failed to activate estimation scopes")
            except Exception as e:
                self.logger.log_error("[CMD] Error activating scopes", e)
            return None
        
        elif command_type == CommandType.DISABLE_SCOPES:
            self.logger.logger.info(f"[CMD] Disabling estimation scopes")
            try:
                self._disable_estimation_scopes()
                self.logger.logger.info(f"[CMD] Estimation scopes disabled")
            except Exception as e:
                self.logger.log_error("[CMD] Error disabling scopes", e)
            return None
        
        elif command_type == CommandType.ENABLE_SCOPE_STREAMING:
            self.logger.logger.info(f"[CMD] Enabling scope data streaming for remote plot")
            try:
                preset_names = data.get('preset_names', ['local_state', 'local_control'])
                stream_rate = data.get('stream_rate', 50.0)
                success = self._enable_scope_streaming(preset_names, stream_rate)
                if success:
                    self.logger.logger.info(f"[CMD] Scope streaming enabled at {stream_rate}Hz: {preset_names}")
                else:
                    self.logger.log_error("[CMD] Failed to enable scope streaming")
            except Exception as e:
                self.logger.log_error("[CMD] Error enabling scope streaming", e)
            return None
        
        elif command_type == CommandType.DISABLE_SCOPE_STREAMING:
            self.logger.logger.info(f"[CMD] Disabling scope data streaming")
            try:
                self._disable_scope_streaming()
                self.logger.logger.info(f"[CMD] Scope streaming disabled")
            except Exception as e:
                self.logger.log_error("[CMD] Error disabling scope streaming", e)
            return None
        
        elif command_type == CommandType.SET_LOCAL_OBSERVER:
            observer_type = data.get('observer_type')
            if observer_type:
                self.logger.logger.info(f"[CMD] Switching local observer to: {observer_type}")
                try:
                    success = self._switch_local_observer(observer_type)
                    if success:
                        self.logger.logger.info(f"[CMD] Local observer switched to {observer_type}")
                    else:
                        self.logger.log_error(f"[CMD] Failed to switch local observer to {observer_type}")
                except Exception as e:
                    self.logger.log_error("[CMD] Error switching local observer", e)
            else:
                self.logger.log_warning("[CMD] SET_LOCAL_OBSERVER missing observer_type")
            return None
        
        elif command_type == CommandType.SET_FLEET_OBSERVER:
            observer_type = data.get('observer_type')
            if observer_type:
                self.logger.logger.info(f"[CMD] Switching fleet observer to: {observer_type}")
                try:
                    success = self._switch_fleet_observer(observer_type)
                    if success:
                        self.logger.logger.info(f"[CMD] Fleet observer switched to {observer_type}")
                    else:
                        self.logger.log_error(f"[CMD] Failed to switch fleet observer to {observer_type}")
                except Exception as e:
                    self.logger.log_error("[CMD] Error switching fleet observer", e)
            else:
                self.logger.log_warning("[CMD] SET_FLEET_OBSERVER missing observer_type")
            return None
        
        elif command_type == CommandType.SET_CONTROLLER:
            category = data.get('category')  # 'longitudinal' or 'lateral'
            controller_type = data.get('controller_type')
            if category and controller_type:
                self.logger.logger.info(f"[CMD] Switching {category} controller to: {controller_type}")
                try:
                    success = self._switch_controller(category, controller_type)
                    if success:
                        self.logger.logger.info(f"[CMD] {category.capitalize()} controller switched to {controller_type}")
                    else:
                        self.logger.log_error(f"[CMD] Failed to switch {category} controller to {controller_type}")
                except Exception as e:
                    self.logger.log_error("[CMD] Error switching controller", e)
            else:
                self.logger.log_warning("[CMD] SET_CONTROLLER missing category or controller_type")
            return None
        
        return None
    
    def _init_controllers(self, force: bool = False):
        """
        Initialize controllers specific to this state.
        Override this in subclasses to initialize state-specific controllers.
        Called once during state initialization or when forced.
        
        Args:
            force: If True, force re-initialization even if already initialized
        """
        pass
    
    # === Helper Methods ===
    
    def _switch_controller(self, category: str, controller_type: str) -> bool:
        """
        Switch controller type in real-time.
        validates type against ConfigControllerLoader values.
        """
        if not hasattr(self.vehicle_logic, 'controller_manager'):
            self.logger.logger.error("Controller Manager not available")
            return False
            
        cm = self.vehicle_logic.controller_manager
        
        if category == 'longitudinal':
            valid_types = cm.config.get_available_longitudinal_types()
            if controller_type not in valid_types:
                self.logger.logger.error(f"Invalid longitudinal controller type: {controller_type}. Valid: {valid_types}")
                return False
            success = cm.switch_longitudinal(controller_type)
            
        elif category == 'lateral':
            valid_types = cm.config.get_available_lateral_types()
            if controller_type not in valid_types:
                self.logger.logger.error(f"Invalid lateral controller type: {controller_type}. Valid: {valid_types}")
                return False
            success = cm.switch_lateral(controller_type)
            
        else:
            self.logger.logger.error(f"Invalid controller category: {category}")
            return False
            
        if success:
             self._init_controllers(force=True) # Force refresh controllers in current state
        # else:
        #     self.logger.logger.error(f"Failed to switch {category} controller to {controller_type}")
        
        return success
    

    
    def _send_platoon_setup_confirmation(self, my_vehicle_id: int, formation: Dict, leader_id: int):
        """Send platoon setup confirmation to Ground Station"""
        try:
            if hasattr(self.vehicle_logic, 'client_Ground_Station') and self.vehicle_logic.client_Ground_Station:
                platoon_ctrl = self.vehicle_logic.platoon_controller
                confirmation = {
                    'type': 'platoon_setup_confirm',
                    'car_id': my_vehicle_id,
                    'data': {
                        'position': getattr(platoon_ctrl, 'my_position', None),
                        'is_leader': platoon_ctrl.is_leader,
                        'leader_id': platoon_ctrl.leader_car_id,
                        'setup_complete': getattr(platoon_ctrl, 'setup_complete', False),
                        'formation': formation
                    }
                }
                
                self.vehicle_logic.client_Ground_Station.queue_telemetry(confirmation)
                
                if self.logger:
                    role = "LEADER" if platoon_ctrl.is_leader else f"FOLLOWER-{getattr(platoon_ctrl, 'my_position', '?')}"
                    self.logger.logger.info(f"Sent platoon setup confirmation to GS: Role={role}, Leader ID={platoon_ctrl.leader_car_id}")
            else:
                if self.logger:
                    self.logger.logger.warning("Cannot send platoon confirmation - no Ground Station connection")
        except Exception as e:
            if self.logger:
                self.logger.logger.error(f"Failed to send platoon setup confirmation: {e}")
    
    def validate_event_data(self, data: Dict[str, Any], required_fields: list) -> bool:
        """
        Validate that event data contains required fields
        
        Args:
            data: Event data dictionary
            required_fields: List of required field names
            
        Returns:
            bool: True if all required fields are present
        """
        for field in required_fields:
            if field not in data:
                if self.logger:
                    self.logger.logger.warning(f"Event data missing required field: {field}")
                return False
        return True
    
    def _recalibrate_gps(self, initial_pose=None , calibrate=True) -> bool:
        """
        Recalibrate GPS without reinitializing other components
        
        Args:
            initial_pose: Optional numpy array [x, y, theta]. If None, uses config calibration_pose
            
        Returns:
            bool: True if recalibration successful
        """
        try:

            
            # Use provided initial_pose or fall back to config calibration_pose
            calibration_pose = initial_pose if initial_pose is not None else self.config.path.calibration_pose
            
            if initial_pose is not None:
                self.logger.logger.info(f"Starting GPS recalibration with custom pose: ({calibration_pose[0]:.2f}, {calibration_pose[1]:.2f}, {np.rad2deg(calibration_pose[2]):.1f}°)")
            else:
                self.logger.logger.info("Starting GPS recalibration with config calibration_pose")
            
            # Close existing GPS if it exists
            if hasattr(self.vehicle_logic, 'gps') and self.vehicle_logic.gps:
                try:
                    self.vehicle_logic.gps.terminate()
                    self.logger.logger.info("Existing GPS terminated")
                except:
                    pass
            
            # Reinitialize GPS with calibration
            if not IS_PHYSICAL_QCAR:
                # For fake vehicles: Update mock hardware positions
                self._update_fake_vehicle_position(calibration_pose)
                
                # We dont need to recalibrate simulated GPS in Qlabs (since the Position will always be perfect)
                self.logger.logger.info("GPS recalibrated (simulated/Fake mode)")
            else:
                # Physical QCar
                self.vehicle_logic.gps = QCarGPS(
                    initialPose=calibration_pose,
                    calibrate=calibrate
                )
                self.logger.logger.info("GPS recalibrated (physical mode)")
            

            
            time.sleep(0.5)
            
            # Wait for new GPS reading
            self.logger.logger.info("Waiting for GPS reading after calibration...")
            start = time.time()
            timeout = 5.0
            
            while (time.time() - start) < timeout:
                if self.vehicle_logic.gps.readGPS():
                    new_pose = np.array([
                        self.vehicle_logic.gps.position[0],
                        self.vehicle_logic.gps.position[1],
                        self.vehicle_logic.gps.orientation[2]
                    ])
                    self.logger.logger.info(
                        f"GPS recalibrated - New pose: x={new_pose[0]:.2f}, "
                        f"y={new_pose[1]:.2f}, theta={np.rad2deg(new_pose[2]):.1f}°"
                    )
                    
                    # Reset vehicle observer with calibrated GPS position
                    if hasattr(self.vehicle_logic, 'vehicle_observer') and self.vehicle_logic.vehicle_observer:
                        self.vehicle_logic.vehicle_observer.reset_observer(new_pose)
                        self.logger.logger.info("Vehicle observer reset with calibrated GPS position")
                    
                    return True
                time.sleep(0.1)
            
            self.logger.log_error("GPS recalibration timeout - no reading received")
            return False
            
        except Exception as e:
            self.logger.log_error("GPS recalibration failed", e)
            return False
    
    def _update_fake_vehicle_position(self, pose):
        """
        Update fake vehicle mock hardware position after GPS recalibration.
        Only applies to fake vehicles with mock hardware.
        
        Args:
            pose: numpy array [x, y, theta]
        """
        try:
            
            # Check if this is a fake vehicle with mock hardware
            if hasattr(self.vehicle_logic, '_parent_fake_vehicle'):
                fake_vehicle = self.vehicle_logic._parent_fake_vehicle
                
                # Update MockQCar position
                if hasattr(fake_vehicle, 'mock_qcar'):
                    fake_vehicle.mock_qcar.reset(pose)
                    # fake_vehicle.mock_qcar.x = float(pose[0])
                    # fake_vehicle.mock_qcar.y = float(pose[1])
                    # fake_vehicle.mock_qcar.heading = float(pose[2])
                    # self.logger.logger.info(
                    #     f"MockQCar position updated: ({pose[0]:.2f}, {pose[1]:.2f}, {np.rad2deg(pose[2]):.1f}°)"
                    # )
                
                # # Update MockQCarGPS position
                # if hasattr(fake_vehicle, 'mock_gps'):
                #     fake_vehicle.mock_gps.x = float(pose[0])
                #     fake_vehicle.mock_gps.y = float(pose[1])
                #     self.logger.logger.info(
                #         f"MockGPS position updated: ({pose[0]:.2f}, {pose[1]:.2f})"
                #     )
        except Exception as e:
            self.logger.logger.error(f"Failed to update fake vehicle position: {e}")
            pass
    
    def _activate_perception_system(self) -> bool:
        """
        Activate perception system (YOLO) based on fleet configuration.
        Works for both physical and simulated vehicles.
        Fake vehicles gracefully ignore this command.
        
        Returns:
            bool: True if activation successful
        """
        try:
            import subprocess
            import os
            import time
            from Yolo.YoLo import YOLOReceiver, YOLODriveLogic
            
            # Check if this is a fake vehicle - if so, just log and return success
            if hasattr(self.vehicle_logic, '_parent_fake_vehicle'):
                self.logger.logger.info("[PERCEPTION] Fake vehicle detected - ignoring perception activation")
                return True
            
            # Check if perception should be enabled based on config
            vehicle_id = self.vehicle_logic.vehicle_id
            
            # Load fleet_config to check probing setting
            config_enabled = self._check_perception_config()
            if not config_enabled:
                self.logger.logger.warning(f"[PERCEPTION] Vehicle {vehicle_id} not configured for perception in fleet_config.yaml")
                return False
            
            self.logger.logger.info(f"[PERCEPTION] Starting YOLO system for vehicle {vehicle_id}...")
            
            # Get probing flag from config
            probing_enabled = self.config.vehicle.probing
            probing_flag = "True" if probing_enabled else "False"
            
            # For simulated vehicles, launch YOLO server subprocess
            if not IS_PHYSICAL_QCAR:
                yolo_port = f'1866{vehicle_id}'
                
                yolo_script = os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), 
                    '..', 'Yolo', 'yolo_server_virtual.py'
                )
                
                # Verify script exists
                if not os.path.exists(yolo_script):
                    self.logger.log_error(f"[PERCEPTION] YOLO script not found: {yolo_script}")
                    return False
                
                self.logger.logger.info(f"[PERCEPTION] [→] Starting yolo_server_virtual.py...")
                self.logger.logger.info(f"[PERCEPTION] Script path: {yolo_script}")
                self.logger.logger.info(f"[PERCEPTION] Launching YOLO server on port {yolo_port}...")
                self.logger.logger.info(f"[PERCEPTION] Probing enabled: {probing_enabled}")
                
                # Build command with probing flag
                # Note: yolo_server_virtual.py uses -p/--probing with value "True" or "False"
                # and -s/--show-image to display the image directly in a window
                # For simulated vehicles, we dont use probing use directly cv2 window (reduce latency)
                cmd = ['python', yolo_script, '-idx', str(vehicle_id), '-p', "False"]
                
                # If probing is enabled but no Observer is running, also show image directly
                # This provides a fallback visualization
                if probing_enabled:
                    cmd.append('-s')  # Show image in cv2 window as backup
                
                self.logger.logger.info(f"[PERCEPTION] Command: {' '.join(cmd)}")
                
                # Launch YOLO server as subprocess
                # On Windows, use CREATE_NEW_CONSOLE so cv2 windows can display properly
                import sys
                if sys.platform == 'win32':
                    # Create new console window for the YOLO server
                    yolo_process = subprocess.Popen(
                        cmd,
                        creationflags=subprocess.CREATE_NEW_CONSOLE
                    )
                else:
                    yolo_process = subprocess.Popen(
                        cmd,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE
                    )
                
                # Store process handle
                self.vehicle_logic.yolo_process = yolo_process
                
                # Wait for server startup - just check that process is running
                # Note: We don't use a test socket connection here because it can interfere
                # with BasicStream's connection handling (connects then immediately disconnects)
                self.logger.logger.info("[PERCEPTION] Waiting for YOLO server to initialize...")
                max_wait = 6.0  # Maximum wait time for process to stabilize
                check_interval = 0.5
                waited = 0.0
                
                while waited < max_wait:
                    time.sleep(check_interval)
                    waited += check_interval
                    
                    # Check if process crashed
                    if yolo_process.poll() is not None:
                        # Process terminated unexpectedly
                        self.logger.log_error("[PERCEPTION] YOLO server process terminated unexpectedly")
                        self.logger.logger.error(f"[PERCEPTION] Exit code: {yolo_process.returncode}")
                        # On Windows with CREATE_NEW_CONSOLE, we don't have stdout/stderr pipes
                        if sys.platform != 'win32':
                            try:
                                stdout, stderr = yolo_process.communicate(timeout=1)
                                if stderr:
                                    self.logger.logger.error(f"[PERCEPTION] STDERR: {stderr.decode('utf-8', errors='ignore')}")
                                if stdout:
                                    self.logger.logger.error(f"[PERCEPTION] STDOUT: {stdout.decode('utf-8', errors='ignore')}")
                            except:
                                pass
                        return False
                    
                    # Log progress every second
                    if waited % 1.0 == 0:
                        self.logger.logger.info(f"[PERCEPTION] Server starting... ({waited:.0f}s)")
                
                self.logger.logger.info(f"[PERCEPTION] [✓] YOLO server process running (waited {waited:.1f}s)")
                self.logger.logger.info(f"[PERCEPTION] [✓] YOLO server started (Probing: {probing_enabled})")
                
                # Create YOLOReceiver to connect to the server
                yolo_receiver = YOLOReceiver(
                    ip='localhost',
                    nonBlocking=True,
                    port=yolo_port
                )
            else:
                # For physical vehicles, launch yolo_server.py with probing flag
                yolo_script = os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), 
                    '..', 'Yolo', 'yolo_server.py'
                )
                
                self.logger.logger.info(f"[PERCEPTION] [→] Starting yolo_server.py...")
                self.logger.logger.info(f"[PERCEPTION] Probing: {probing_enabled}, Car ID: {vehicle_id}")
                
                # Build command with probing flag
                yolo_cmd = [
                    'python', yolo_script,
                    '--probing', probing_flag,
                    '--car-id', str(vehicle_id)
                ]
                
                # Set up logging - redirect to file for physical vehicles
                # This allows checking logs later via: tail -f logs/yolo_0.log
                log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'logs')
                os.makedirs(log_dir, exist_ok=True)
                log_file = os.path.join(log_dir, f'yolo_{vehicle_id}.log')
                
                self.logger.logger.info(f"[PERCEPTION] YOLO log file: {log_file}")
                self.logger.logger.info(f"[PERCEPTION] Command: {' '.join(yolo_cmd)}")
                
                # Open log file and keep handle for subprocess lifetime
                # Store the file handle so it doesn't get garbage collected
                yolo_log_handle = open(log_file, 'w')
                self.vehicle_logic.yolo_log_handle = yolo_log_handle
                
                # Launch YOLO server as subprocess with log file
                yolo_process = subprocess.Popen(
                    yolo_cmd,
                    stdout=yolo_log_handle,
                    stderr=subprocess.STDOUT
                )
                
                # Store process handle
                self.vehicle_logic.yolo_process = yolo_process
                
                # Wait for server startup
                self.logger.logger.info("[PERCEPTION] Waiting for YOLO server to initialize...")
                time.sleep(2.5)
                
                # Check if process is still running
                if yolo_process.poll() is not None:
                    self.logger.log_error("[PERCEPTION] YOLO server failed to start")
                    return False
                
                self.logger.logger.info(f"[PERCEPTION] [✓] YOLO server started (Probing: {probing_enabled})")
                
                # Create YOLOReceiver to connect to the server
                yolo_receiver = YOLOReceiver(
                    ip='localhost',
                    nonBlocking=True,
                )
            
            # Create YOLO drive logic
            pulse_length = (
                self.config.timing.controller_update_rate *
                self.config.yolo.pulse_length_multiplier
            )
            yolo_drive_logic = YOLODriveLogic(
                stopSignThreshold=self.config.yolo.stop_sign_threshold,
                trafficThreshold=self.config.yolo.traffic_threshold,
                carThreshold=self.config.yolo.car_threshold,
                yieldThreshold=self.config.yolo.yield_threshold,
                personThreshold=self.config.yolo.person_threshold,
                pulseLength=pulse_length
            )
            
            # Initialize YOLOManager
            self.vehicle_logic.yolo_manager.initialize(yolo_receiver, yolo_drive_logic)
            
            self.logger.logger.info("[PERCEPTION] YOLO system activated successfully")
            return True
            
        except Exception as e:
            self.logger.log_error("[PERCEPTION] Failed to activate perception system", e)
            # Clean up process if it was started
            if hasattr(self.vehicle_logic, 'yolo_process'):
                try:
                    self.vehicle_logic.yolo_process.terminate()
                except:
                    pass
            return False
    
    def _disable_perception_system(self) -> bool:
        """
        Disable perception system (YOLO).
        Fake vehicles gracefully ignore this command.
        
        Process termination is done asynchronously in a background thread 
        to avoid blocking the main control loop.
        
        Returns:
            bool: True if deactivation successful
        """
        try:
            import threading
            
            # Check if this is a fake vehicle - if so, just log and return success
            if hasattr(self.vehicle_logic, '_parent_fake_vehicle'):
                self.logger.logger.info("[PERCEPTION] Fake vehicle detected - ignoring perception deactivation")
                return True
            
            self.logger.logger.info("[PERCEPTION] Disabling YOLO system...")
            
            # Disable YOLO manager (non-blocking - just clears references and terminates receiver)
            if hasattr(self.vehicle_logic, 'yolo_manager'):
                self.vehicle_logic.yolo_manager.disable()
            
            # Terminate YOLO process asynchronously to avoid blocking the main loop
            yolo_process = getattr(self.vehicle_logic, 'yolo_process', None)
            yolo_log_handle = getattr(self.vehicle_logic, 'yolo_log_handle', None)
            logger = self.logger
            
            def cleanup_yolo_process():
                """Background thread to clean up YOLO process without blocking main loop."""
                if yolo_process is not None:
                    try:
                        yolo_process.terminate()
                        yolo_process.wait(timeout=5)  # Wait up to 5 seconds
                        if logger:
                            logger.logger.info("[PERCEPTION] YOLO process terminated (async)")
                    except Exception:
                        try:
                            yolo_process.kill()
                            if logger:
                                logger.logger.warning("[PERCEPTION] YOLO process killed (async)")
                        except Exception:
                            pass
                
                # Close YOLO log file handle if it exists (physical vehicles)
                if yolo_log_handle is not None:
                    try:
                        yolo_log_handle.close()
                        if logger:
                            logger.logger.info("[PERCEPTION] YOLO log file closed (async)")
                    except Exception:
                        pass
            
            # Start cleanup in background thread (daemon so it won't block program exit)
            cleanup_thread = threading.Thread(target=cleanup_yolo_process, daemon=True)
            cleanup_thread.start()
            
            # Clear references immediately
            self.vehicle_logic.yolo_process = None
            self.vehicle_logic.yolo_log_handle = None
            
            self.logger.logger.info("[PERCEPTION] YOLO cleanup started in background thread")
            return True
            
        except Exception as e:
            self.logger.log_error("[PERCEPTION] Error disabling perception", e)
            return False
    
    def _check_perception_config(self) -> bool:
        """
        Check if perception is enabled for this vehicle in configuration.
        
        Returns:
            bool: True if perception should be enabled
        """
        # Simply check the probing flag in vehicle config
        probing_enabled = self.config.vehicle.probing
        
        if probing_enabled:
            self.logger.logger.info(f"[PERCEPTION] Vehicle {self.vehicle_logic.vehicle_id} configured with probing=True")
        else:
            self.logger.logger.info(f"[PERCEPTION] Vehicle {self.vehicle_logic.vehicle_id} configured with probing=False")
        
        return probing_enabled
    
    def _activate_estimation_scopes(self, preset_names: list = None) -> bool:
        """
        Activate estimation scopes visualization.
        
        Args:
            preset_names: List of preset names to enable. Options:
                - 'local_state', 'local_error', 'local_control'
                - 'fleet_position', 'fleet_state', 'fleet_consensus'
                
        Returns:
            bool: True if activation successful
        """
        try:
            if preset_names is None:
                preset_names = ['local_state', 'local_control']
            
            # Check if vehicle_observer exists
            if not hasattr(self.vehicle_logic, 'vehicle_observer') or self.vehicle_logic.vehicle_observer is None:
                self.logger.log_error("[SCOPES] Vehicle observer not available")
                return False
            
            # Enable scopes on observer
            success = self.vehicle_logic.vehicle_observer.enable_scopes(
                preset_names=preset_names,
                fps=30,
                time_window=60.0
            )
            
            if success:
                self.logger.logger.info(f"[SCOPES] Estimation scopes activated: {preset_names}")
            else:
                self.logger.log_error("[SCOPES] Failed to enable scopes")
            
            return success
            
        except Exception as e:
            self.logger.log_error("[SCOPES] Error activating estimation scopes", e)
            return False
    
    def _disable_estimation_scopes(self) -> bool:
        """
        Disable estimation scopes visualization.
        
        Returns:
            bool: True if deactivation successful
        """
        try:
            # Check if vehicle_observer exists
            if not hasattr(self.vehicle_logic, 'vehicle_observer') or self.vehicle_logic.vehicle_observer is None:
                self.logger.log_error("[SCOPES] Vehicle observer not available")
                return False
            
            # Stop scopes on observer
            self.vehicle_logic.vehicle_observer.stop_scopes()
            self.logger.logger.info("[SCOPES] Estimation scopes disabled")
            
            return True
            
        except Exception as e:
            self.logger.log_error("[SCOPES] Error disabling estimation scopes", e)
            return False
    
    def _enable_scope_streaming(self, preset_names: list = None, stream_rate: float = 50.0) -> bool:
        """
        Enable scope data streaming to Ground Station for remote plotting.
        
        This creates a ScopeDataStreamer that packages data and sends it
        at high frequency (default 50Hz) to the Ground Station.
        
        Args:
            preset_names: List of preset names to stream
            stream_rate: Streaming rate in Hz
            
        Returns:
            bool: True if streaming enabled successfully
        """
        try:
            if preset_names is None:
                preset_names = ['local_state', 'local_control']
            
            # Check if Ground Station client exists
            if not hasattr(self.vehicle_logic, 'client_Ground_Station') or self.vehicle_logic.client_Ground_Station is None:
                self.logger.log_error("[STREAMING] Ground Station client not available")
                return False
            
            # Import and create streamer
            from scope_data_streamer import ScopeDataStreamer
            
            # Create or update streamer
            if not hasattr(self.vehicle_logic, 'scope_streamer') or self.vehicle_logic.scope_streamer is None:
                self.vehicle_logic.scope_streamer = ScopeDataStreamer(
                    gs_client=self.vehicle_logic.client_Ground_Station,
                    vehicle_id=self.vehicle_logic.vehicle_id,
                    stream_rate=stream_rate
                )
            else:
                self.vehicle_logic.scope_streamer.set_stream_rate(stream_rate)
            
            # Enable streaming on client
            self.vehicle_logic.client_Ground_Station.enable_scope_streaming()
            
            # Enable streamer
            success = self.vehicle_logic.scope_streamer.enable(preset_names)
            
            if success:
                self.logger.logger.info(f"[STREAMING] Scope streaming enabled: {preset_names} at {stream_rate}Hz")
            
            return success
            
        except Exception as e:
            self.logger.log_error("[STREAMING] Error enabling scope streaming", e)
            return False
    
    def _disable_scope_streaming(self) -> bool:
        """
        Disable scope data streaming to Ground Station.
        
        Returns:
            bool: True if streaming disabled successfully
        """
        try:
            # Disable streamer
            if hasattr(self.vehicle_logic, 'scope_streamer') and self.vehicle_logic.scope_streamer is not None:
                self.vehicle_logic.scope_streamer.disable()
            
            # Disable streaming on client
            if hasattr(self.vehicle_logic, 'client_Ground_Station') and self.vehicle_logic.client_Ground_Station is not None:
                self.vehicle_logic.client_Ground_Station.disable_scope_streaming()
            
            self.logger.logger.info("[STREAMING] Scope streaming disabled")
            return True
            
        except Exception as e:
            self.logger.log_error("[STREAMING] Error disabling scope streaming", e)
            return False
    
    # === Runtime Switching Methods ===
    
    def _switch_local_observer(self, observer_type: str) -> bool:
        """
        Switch the local state estimator at runtime.
        
        Args:
            observer_type: Type of local estimator ('ekf', 'luenberger', 'dead_reckoning', 'neural_luenberger')
            
        Returns:
            bool: True if successful
        """
        try:
            from Observer.local_state_estimators import LocalEstimatorFactory
            
            valid_types = ['ekf', 'luenberger', 'dead_reckoning', 'neural_luenberger']
            if observer_type not in valid_types:
                self.logger.log_error(f"Invalid local observer type: {observer_type}. Valid: {valid_types}")
                return False
            
            if not hasattr(self.vehicle_logic, 'vehicle_observer') or self.vehicle_logic.vehicle_observer is None:
                self.logger.log_error("Vehicle observer not initialized")
                return False
            
            vehicle_observer = self.vehicle_logic.vehicle_observer
            
            # Get current state for continuity
            current_pose = None
            if hasattr(vehicle_observer, 'local_estimator') and vehicle_observer.local_estimator is not None:
                try:
                    state = vehicle_observer.local_estimator.get_state()
                    if state is not None:
                        current_pose = state[:3]  # [x, y, theta]
                except:
                    pass
            
            # Get config defaults for the new estimator type
            config_defaults = vehicle_observer.local_config_defaults.get(observer_type, {})
            
            # Create new estimator using factory
            new_estimator = LocalEstimatorFactory.create(
                estimator_type=observer_type,
                config=config_defaults,
                logger=self.logger
            )
            
            # Initialize the new estimator
            if hasattr(new_estimator, 'initialize'):
                gps = getattr(self.vehicle_logic, 'gps', None)
                new_estimator.initialize(gps=gps, initial_pose=current_pose)
            
            # Swap the estimator
            vehicle_observer.set_local_estimator(new_estimator)
            vehicle_observer.local_estimator_type = observer_type
            
            return True
            
        except Exception as e:
            self.logger.log_error(f"Failed to switch local observer to {observer_type}", e)
            return False
    
    def _switch_fleet_observer(self, observer_type: str) -> bool:
        """
        Switch the fleet state estimator at runtime.
        
        Args:
            observer_type: Type of fleet estimator ('consensus', 'distributed_kalman', etc.)
            
        Returns:
            bool: True if successful
        """
        try:
            from Observer.fleet_state_estimators import FleetEstimatorFactory
            
            valid_types = ['consensus', 'distributed_kalman', 'distributed_luenberger',
                          'leadering_observer', 'classical_luenberger_observer',
                          'classical_luenberge_observer', 'parallel_observers',
                          'trust_consensus', 'trust_kalman']
            if observer_type not in valid_types:
                self.logger.log_error(f"Invalid fleet observer type: {observer_type}. Valid: {valid_types}")
                return False
            
            if not hasattr(self.vehicle_logic, 'vehicle_observer') or self.vehicle_logic.vehicle_observer is None:
                self.logger.log_error("Vehicle observer not initialized")
                return False
            
            vehicle_observer = self.vehicle_logic.vehicle_observer
            
            # Get observer config
            obs_config = vehicle_observer._get_observer_config()
            
            # Create new fleet estimator using factory
            new_estimator = FleetEstimatorFactory.create(
                estimator_type=observer_type,
                vehicle_id=self.vehicle_logic.vehicle_id,
                config=obs_config,
                logger=self.logger
            )
            
            # Swap the estimator
            vehicle_observer.set_fleet_estimator(new_estimator)
            vehicle_observer.fleet_estimator_type = observer_type
            
            return True
            
        except Exception as e:
            self.logger.log_error(f"Failed to switch fleet observer to {observer_type}", e)
            return False
    
