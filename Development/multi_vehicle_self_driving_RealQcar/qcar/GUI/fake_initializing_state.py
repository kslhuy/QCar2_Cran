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
        
        # Check if emergency stop requested
        if self.should_transition_to_stopped(sensor_data):
            return throttle, steering, (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
        
        # Quick fake initialization
        if not self.state_data['quick_init_done']:
            # Simulate quick component initialization
            
            if init_time > 0.5:  # 0.5 second fake initialization (faster for testing)
                self.state_data['quick_init_done'] = True
                self.logger.logger.info("Fake components initialized quickly for testing")
                print(f"DEBUG Car init: Fake initialization complete after {init_time:.1f}s")
        
        # Transition to WAITING_FOR_START when ready
        if self.state_data['quick_init_done']:
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
        """Override real hardware initialization - fake vehicle components are always ready"""
        print(f"[*] FakeInitializingState: Skipping real hardware initialization")
        print(f"   [+] Mock hardware already injected")
        print(f"   [+] Ground Station connection handled separately")
        print(f"   [+] Fake components ready")
        return True
    
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