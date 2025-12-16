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
        # Check for emergency stop conditions first
        if self.should_transition_to_stopped(sensor_data):
            return 0.0, 0.0, (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
        
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
                if success and self.logger:
                    self.logger.logger.info(f"[OK] V2V disabled in {self.__class__.__name__}")
                elif self.logger:
                    self.logger.logger.warning(f"[!] V2V disable failed in {self.__class__.__name__}")
                return None  # No state transition
            return None
        
        # Default: Event not handled by this state
        if self.logger:
            self.logger.logger.info(f"Command '{command_type}' ignored in {self.__class__.__name__}")
        return None
    
    # === Helper Methods ===
    
    def should_transition_to_stopped(self, sensor_data: Dict[str, Any]) -> bool:
        """
        Common check for emergency stop conditions
        All states can use this to check for safety stops
        Works with or without camera/YOLO system
        """
        # Check collision avoidance system (YOLO/Camera-based) - only if enabled
        yolo_enabled = getattr(self.vehicle_logic.yolo_manager, 'yolo_enabled', False)
        if yolo_enabled and hasattr(self.vehicle_logic, 'collision_avoidance'):
            try:
                yolo_data = sensor_data.get('yolo_data', {})
                emergency_stop, _ = self.vehicle_logic.collision_avoidance.check_collision_risk(
                    car_distance=yolo_data.get('car_dist'),
                    person_distance=yolo_data.get('person_dist'),
                    current_velocity=sensor_data.get('velocity', 0.0)
                )
                if emergency_stop:
                    if self.logger:
                        self.logger.log_warning("Emergency stop triggered by collision avoidance system")
                    return True
            except Exception as e:
                # Log but continue if collision check fails
                if self.logger:
                    self.logger.logger.debug(f"Collision check failed (ignored): {e}")
        
        # If YOLO not enabled, skip collision checks
        if not yolo_enabled:
            return False
        
       
        
        return False
    
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
                # TODO :  maybe we can teleport the Qcar to the calibration_pose?
                
                # # Simulated QCar - need car config
                # from qvl.multi_agent import readRobots
                # robotsDir = readRobots()
                # name = f"QC2_{self.vehicle_logic.vehicle_id}"
                # car_config = robotsDir[name]
                
                # self.vehicle_logic.gps = QCarGPS(
                #     initialPose=calibration_pose,
                #     calibrate=calibrate,
                #     gpsPort=car_config["gpsPort"],
                #     lidarIdealPort=car_config["lidarIdealPort"]
                # )
                self.logger.logger.info("GPS recalibrated (simulated mode)")
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
                    fake_vehicle.mock_qcar.x = float(pose[0])
                    fake_vehicle.mock_qcar.y = float(pose[1])
                    fake_vehicle.mock_qcar.heading = float(pose[2])
                    self.logger.logger.info(
                        f"MockQCar position updated: ({pose[0]:.2f}, {pose[1]:.2f}, {np.rad2deg(pose[2]):.1f}°)"
                    )
                
                # Update MockQCarGPS position
                if hasattr(fake_vehicle, 'mock_gps'):
                    fake_vehicle.mock_gps.x = float(pose[0])
                    fake_vehicle.mock_gps.y = float(pose[1])
                    self.logger.logger.info(
                        f"MockGPS position updated: ({pose[0]:.2f}, {pose[1]:.2f})"
                    )
        except Exception as e:
            # Silently ignore errors for non-fake vehicles
            pass