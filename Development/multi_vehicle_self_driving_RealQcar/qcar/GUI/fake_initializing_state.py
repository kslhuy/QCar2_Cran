"""
Simplified Initializing State for Fake Vehicle

This state quickly initializes the fake vehicle components 
and transitions to WAITING_FOR_START for testing.
"""
import time
import sys
import os
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
        super().enter()
        print(f"DEBUG Car init: enter() method called")
        self.logger.logger.info("Entering FAKE INITIALIZING state")
        
        # Reset any previous state
        self.state_data = {
            'initialization_start': time.time(),
            'quick_init_done': False
        }
        
        print(f"DEBUG Car init: state_data initialized in enter()")
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
            if parent_fake_vehicle:
                # Inject mock hardware from the fake vehicle
                self.vehicle_logic.qcar = parent_fake_vehicle.mock_qcar
                self.vehicle_logic.gps = parent_fake_vehicle.mock_gps  
                # Initialize YOLOManager with parent's mock components
                from fake_vehicle_real_logic import MockStateEstimator, MockSpeedController, MockSteeringController, MockYOLODrive
                mock_yolo_drive = MockYOLODrive(self.config.network.car_id)
                self.vehicle_logic.yolo_manager.initialize(parent_fake_vehicle.mock_yolo, mock_yolo_drive)
                
                # Create mock controllers and state estimator
                mock_state_estimator = MockStateEstimator(parent_fake_vehicle.mock_qcar, parent_fake_vehicle.mock_gps)
                self.vehicle_logic.vehicle_observer.set_state_estimator(mock_state_estimator)
                self.vehicle_logic.speed_controller = MockSpeedController(self.config.network.car_id)
                self.vehicle_logic.steering_controller = MockSteeringController(self.config.network.car_id)
                
                print(f"   [+] Mock QCar hardware injected")
                print(f"   [+] Mock GPS injected")
                print(f"   [+] Mock YOLO injected")
                print(f"   [+] Mock controllers created")
            else:
                print(f"   [!] Parent fake vehicle not found, using basic mocks")
                # Fallback to creating basic mocks
                from fake_vehicle_real_logic import MockQCar, MockQCarGPS, MockYOLOReceiver
                from fake_vehicle_real_logic import MockStateEstimator, MockSpeedController, MockSteeringController, MockYOLODrive
                car_id = self.config.network.car_id
                
                self.vehicle_logic.qcar = MockQCar(car_id)
                self.vehicle_logic.gps = MockQCarGPS(self.vehicle_logic.qcar)
                
                # Initialize YOLOManager with mock components
                mock_yolo_receiver = MockYOLOReceiver()
                mock_yolo_drive = MockYOLODrive()
                self.vehicle_logic.yolo_manager.initialize(mock_yolo_receiver, mock_yolo_drive)
                
                mock_state_estimator = MockStateEstimator(self.vehicle_logic.qcar, self.vehicle_logic.gps)
                self.vehicle_logic.vehicle_observer.set_state_estimator(mock_state_estimator)
                self.vehicle_logic.speed_controller = MockSpeedController(car_id)
                self.vehicle_logic.steering_controller = MockSteeringController(car_id)
                
                print(f"   [+] Basic mock components created")
            
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