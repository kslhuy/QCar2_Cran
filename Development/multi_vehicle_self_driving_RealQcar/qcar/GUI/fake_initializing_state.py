"""
Simplified Initializing State for Fake Vehicle

This state quickly initializes the fake vehicle components 
and transitions to WAITING_FOR_START for testing.
"""
import time
import sys
import os
import numpy as np
from typing import Dict, Any, Tuple, Optional
from StateMachine.state_base import StateBase
from StateMachine.vehicle_state import VehicleState, StateTransitionReason

# Add parent directory to fix import issues
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Import CommandType with proper error handling - import from parent qcar directory
try:
    # command_handler.py is in the parent qcar directory
    import sys
    import os
    qcar_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # Go up to qcar directory
    if qcar_dir not in sys.path:
        sys.path.insert(0, qcar_dir)
    
    from command_handler import CommandType
    COMMAND_TYPE_AVAILABLE = True
    print(f"[+] CommandType imported successfully from {qcar_dir}")
except ImportError as e:
    print(f"INFO: CommandType not available in fake vehicle: {e}")
    COMMAND_TYPE_AVAILABLE = False
    CommandType = None


class FakeInitializingState(StateBase):
    """Handler for INITIALIZING state - simplified for fake vehicle"""
    
    def enter(self) -> bool:
        """Initialize fake vehicle components quickly"""
        print(f"[DEBUG] FakeInitializingState.enter() called - START")
        super().enter()
        print(f"[DEBUG] FakeInitializingState.enter() - after super().enter()")
        print(f"DEBUG Car init: enter() method called")
        self.logger.logger.info("Entering FAKE INITIALIZING state")
        
        # Reset any previous state
        self.state_data = {
            'initialization_start': time.time(),
            'quick_init_done': False,
            'telemetry_initialized': False
        }
        
        print(f"[DEBUG] Checking telemetry logging config: {self.config.logging.enable_telemetry_logging}")
        
        # Initialize telemetry logging immediately (if enabled)
        if self.config.logging.enable_telemetry_logging:
            print(f"[DEBUG] Telemetry logging is ENABLED - calling setup_telemetry_logging()")
            try:
                print(f"[DEBUG] About to call self.logger.setup_telemetry_logging()")
                run_dir = self.logger.setup_telemetry_logging(self.config.logging.data_log_dir)
                print(f"[DEBUG] setup_telemetry_logging() returned: {run_dir}")
                self.state_data['telemetry_initialized'] = True
                print(f"[+] FakeInitializingState: Telemetry logging initialized: {run_dir}")
            except Exception as e:
                print(f"[!] FakeInitializingState: Telemetry initialization failed: {e}")
                import traceback
                traceback.print_exc()
        else:
            print(f"[DEBUG] Telemetry logging is DISABLED")
        
        # # AUTO-ACTIVATE V2V for fake vehicles (simulate Ground Station command)
        # print(f"[DEBUG] Auto-activating V2V for fake vehicle testing...")
        # try:
        #     # Activate V2V with all possible peers (vehicles 0-3 on localhost)
        #     vehicle_id = self.config.network.car_id
        #     peer_vehicles = [i for i in range(4) if i != vehicle_id]  # All vehicles except self
        #     peer_ips = ["127.0.0.1"] * len(peer_vehicles)  # All on localhost
            
        #     if hasattr(self.vehicle_logic, 'v2v_manager'):
        #         success = self.vehicle_logic.v2v_manager.activate_v2v(peer_vehicles, peer_ips)
        #         if success:
        #             print(f"[+] FakeInitializingState: V2V auto-activated with peers {peer_vehicles}")
        #             if self.logger:
        #                 self.logger.logger.info(f"V2V auto-activated for fake vehicle {vehicle_id} with peers {peer_vehicles}")
        #         else:
        #             print(f"[!] FakeInitializingState: V2V auto-activation failed")
        #     else:
        #         print(f"[!] FakeInitializingState: No v2v_manager found")
        # except Exception as e:
        #     print(f"[ERROR] V2V auto-activation failed: {e}")
        #     import traceback
        #     traceback.print_exc()
        
        print(f"DEBUG Car init: state_data initialized in enter()")
        print(f"[DEBUG] FakeInitializingState.enter() - END")
        return True
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Handle fake initialization process - fast for testing"""
        
        # Ensure state_data is initialized (safety check)
        if not hasattr(self, 'state_data') or 'initialization_start' not in self.state_data:
            self.state_data = {
                'initialization_start': time.time(),
                'quick_init_done': False
            }
            print(f"DEBUG Car init: State data initialized in update()")
        
        # Debug print to show this method is being called
        init_time = time.time() - self.state_data['initialization_start']
        if int(init_time) % 2 == 0 and init_time > 0 and not self.state_data.get('debug_printed', False):
            print(f"DEBUG Car init: Running fake initialization for {init_time:.1f}s")
            self.state_data['debug_printed'] = True
        elif int(init_time) % 2 == 1:
            self.state_data['debug_printed'] = False
        
        # No control commands during initialization
        throttle, steering = 0.0, 0.0
        
        # Check if emergency stop requested - but use our overridden version
        if self.should_transition_to_stopped(sensor_data):
            return throttle, steering, (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
        
        # Quick fake initialization
        if not self.state_data['quick_init_done']:
            # Simulate quick component initialization by injecting mock hardware
            if init_time > 0.5:  # 0.5 second fake initialization (allow time for system to settle)
                # Inject mock hardware now
                if self._check_components_ready():
                    self.state_data['quick_init_done'] = True
                    self.logger.logger.info("Fake components initialized quickly for testing")
                    print(f"DEBUG Car init: Fake initialization complete after {init_time:.1f}s")
                    time.sleep(0.2)  # Small delay to let everything settle
                else:
                    print(f"DEBUG Car init: Component initialization failed, retrying...")
        
        # Transition to WAITING_FOR_START when ready (with small delay)
        if self.state_data['quick_init_done']:
            # Add small delay before transition for stability
            if init_time > 1.0:  # Total minimum initialization time
                print(f"DEBUG Car init: Transitioning to WAITING_FOR_START")
                return throttle, steering, (VehicleState.WAITING_FOR_START, StateTransitionReason.INITIALIZATION_COMPLETE)
        
        return throttle, steering, None
    
    def handle_event(self, command_type, data: Dict[str, Any] = None):
        """Handle commands during fake initialization"""
        if COMMAND_TYPE_AVAILABLE and CommandType:
            if command_type == CommandType.EMERGENCY_STOP:
                print(f"[!] FakeInitializingState: Emergency stop during initialization")
                return (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
        
        # Ignore other commands during initialization
        print(f"[-] FakeInitializingState: Ignoring command during initialization")
        return None
    
    def _check_components_ready(self) -> bool:
        """Override real hardware initialization - inject fake vehicle components"""
        print(f"[*] FakeInitializingState: Injecting mock hardware components...")
        
        try:
            # Get the parent fake vehicle instance to access mock components
            parent_fake_vehicle = getattr(self.vehicle_logic, '_parent_fake_vehicle', None)
            
            if not parent_fake_vehicle:
                print(f"   [!] Parent fake vehicle not found!")
                return False
            
            # Inject mock hardware from the fake vehicle
            self.vehicle_logic.qcar = parent_fake_vehicle.mock_qcar
            self.vehicle_logic.gps = parent_fake_vehicle.mock_gps  
            print(f"   [+] Mock QCar hardware injected")
            print(f"   [+] Mock GPS injected")
            
            # Initialize YOLOManager with parent's mock components
            from fake_vehicle_real_logic import MockSpeedController, MockSteeringController, MockYOLODrive
            mock_yolo_drive = MockYOLODrive(self.config.network.car_id)
            self.vehicle_logic.yolo_manager.initialize(parent_fake_vehicle.mock_yolo, mock_yolo_drive)
            print(f"   [+] Mock YOLO injected")
            
            # Initialize the existing local estimator with mock GPS
            # Use the normal state estimation scheme (EKF/Luenberger) with mock sensors
            initial_pose = np.array([0.0, 0.0, 0.0])  # Will be updated by mock GPS
            
            print(f"   [*] Initializing local estimator (type: {self.vehicle_logic.vehicle_observer.local_estimator_type})...")
            success = self.vehicle_logic.vehicle_observer.initialize_local_estimator(
                gps=parent_fake_vehicle.mock_gps,
                initial_pose=initial_pose,
                estimator_params={'use_qcar_ekf': False}  # Use fallback EKF for simulation
            )
            
            if not success:
                print(f"   [!] Local estimator initialization failed!")
                return False
            
            print(f"   [+] Local estimator initialized with mock GPS (using {self.vehicle_logic.vehicle_observer.local_estimator_type})")
            
            # Create mock controllers
            self.vehicle_logic.speed_controller = MockSpeedController(self.config.network.car_id)
            self.vehicle_logic.steering_controller = MockSteeringController(self.config.network.car_id)
            print(f"   [+] Mock controllers created")
           
            # Skip real hardware initialization completely
            print(f"   [+] Ground Station connection handled by VehicleLogic")
            print(f"   [+] Fake components ready")
            return True
            
        except Exception as e:
            print(f"   [!] Mock hardware injection failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def should_transition_to_stopped(self, sensor_data: Dict[str, Any]) -> bool:
        """Override emergency stop checks during fake initialization"""
        # During fake initialization, never trigger emergency stops
        # This allows the system to initialize properly with mock hardware
        return False
        
    def _check_initial_position(self, sensor_data: Dict[str, Any]) -> bool:
        """Override position check - fake vehicles start at valid positions"""
        print(f"[*] FakeInitializingState: Skipping position check for fake vehicle")
        return True
    
    def exit(self):
        """Clean up initialization state"""
        self.logger.logger.info("Exiting FAKE INITIALIZING state")
        
        # Log initialization summary
        init_time = self.get_time_in_state()
        self.logger.logger.info(f"Fake initialization completed in {init_time:.1f}s")
        
        super().exit()