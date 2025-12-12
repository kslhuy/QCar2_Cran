"""
Command Handler - Centralized command processing and validation

Provides a structured approach to handling Ground Station commands with:
- Clear command variable naming with 'cmd_' prefix
- Command validation and sanitization 
- State-based command filtering
- Thread-safe command flag management
"""
import time
import threading
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
from command_types import CommandType


@dataclass
class CommandInfo:
    """Information about a command"""
    command_type: CommandType
    timestamp: float
    data: Dict[str, Any]
    source: str = "ground_station"
    processed: bool = False
    valid: bool = True
    error_message: str = ""



class VehicleEvent:
    """Base class for vehicle events"""
    
    def __init__(self, event_type: str, data: Dict[str, Any] = None, source: str = "ground_station"):
        self.event_type = event_type
        self.data = data or {}
        self.source = source
        self.timestamp = time.time()
        self.handled = False
        self.result = None


class EventDispatcher:
    """Dispatches events to registered state handlers"""
    
    def __init__(self, logger=None):
        self.logger = logger
        self.event_listeners = {}  # Dict[str, List[callable]]
        self.event_history = []
        self.max_history = 50
    
    def register_listener(self, event_type: str, callback):
        """Register a callback function for an event type"""
        if event_type not in self.event_listeners:
            self.event_listeners[event_type] = []
        self.event_listeners[event_type].append(callback)
        if self.logger:
            self.logger.logger.info(f"Registered listener for event: {event_type}")
    
    def unregister_listener(self, event_type: str, callback):
        """Unregister a callback function for an event type"""
        if event_type in self.event_listeners:
            try:
                self.event_listeners[event_type].remove(callback)
            except ValueError:
                pass
    
    def dispatch_event(self, event: VehicleEvent) -> bool:
        """
        Dispatch an event to all registered listeners
        
        Args:
            event: VehicleEvent to dispatch
            
        Returns:
            bool: True if event was handled by at least one listener
        """
        if self.logger:
            self.logger.logger.info(f"[E] Dispatching event: {event.event_type} from {event.source}")
        
        handled = False
        listeners = self.event_listeners.get(event.event_type, [])
        
        for listener in listeners:
            try:
                result = listener(event)
                if result:
                    event.handled = True
                    event.result = result
                    handled = True
                    if self.logger:
                        self.logger.logger.info(f"[+] Event {event.event_type} handled by listener")
                    break  # Stop after first successful handler
            except Exception as e:
                if self.logger:
                    self.logger.log_error(f"Error in event listener for {event.event_type}", e)
        
        if not handled and self.logger:
            self.logger.logger.warning(f"⚠️ Event {event.event_type} was not handled by any listener")
        
        # Add to history
        self.event_history.append(event)
        if len(self.event_history) > self.max_history:
            self.event_history.pop(0)
        
        return handled
    
    def get_event_history(self, last_n: int = 10) -> list:
        """Get recent event history"""
        return self.event_history[-last_n:] if last_n > 0 else self.event_history


class CommandHandler:
    """
    Event-driven command handler for processing Ground Station commands.
    
    Features:
    - Event-based command processing 
    - Commands are converted to events and dispatched to states
    - States decide whether to handle events based on current context
    - Command history and logging
    """
    
    # Class-level mappings for better performance (avoid recreating dictionaries)
    TYPE_MAPPING = {
        'stop': CommandType.STOP,
        'start': CommandType.START,
        'emergency_stop': CommandType.EMERGENCY_STOP,
        'set_params': CommandType.SET_PARAMS,
        'set_velocity': CommandType.SET_VELOCITY,
        'set_path': CommandType.SET_PATH,
        'set_initial_position': CommandType.SET_INITIAL_POSITION,
        'enable_platoon': CommandType.ENABLE_PLATOON_FOLLOWER,  # Default to follower
        'enable_platoon_leader': CommandType.ENABLE_PLATOON_LEADER,
        'enable_platoon_follower': CommandType.ENABLE_PLATOON_FOLLOWER,
        'disable_platoon': CommandType.DISABLE_PLATOON,
        'setup_platoon_formation': CommandType.SETUP_PLATOON_FORMATION,  # New global formation
        'start_platoon': CommandType.START_PLATOON,  # New platoon trigger
        'activate_v2v': CommandType.ACTIVATE_V2V,
        'disable_v2v': CommandType.DISABLE_V2V,
        'enable_manual_mode': CommandType.ENABLE_MANUAL_MODE,
        'manual_control': CommandType.MANUAL_CONTROL,
        'disable_manual_mode': CommandType.DISABLE_MANUAL_MODE,
        'shutdown': CommandType.SHUTDOWN,
        'reset': CommandType.RESET,
        'calibrate': CommandType.CALIBRATE
    }
    
    def __init__(self, logger, config=None):
        self.logger = logger
        self.config = config
        
        # Event system
        self.event_dispatcher = EventDispatcher(logger)
        self.command_history: List[CommandInfo] = []
        self.max_history_size = 100
        
        # Statistics
        self.commands_processed = 0
        self.commands_rejected = 0
        self.last_command_time = 0.0
    
    def register_state_listeners(self, state_machine):
        """
        Register event listeners from the state machine
        Now much simpler - just register handle_event for all states
        
        Args:
            state_machine: VehicleStateMachine instance with state handlers
        """
        # Register the single event handler for the current state
        # We'll dispatch to current state in real-time
        self.state_machine = state_machine
        self.logger.logger.info("[+] Event system connected to state machine")
    
    def dispatch_to_current_state(self, command_type: CommandType, data: Dict[str, Any] = None) -> bool:
        """
        Dispatch command enum directly to current state
        
        Args:
            command_type: CommandType enum (e.g., CommandType.START, CommandType.STOP)
            data: Event data
            
        Returns:
            bool: True if event caused a state transition
        """
        if not hasattr(self, 'state_machine'):
            self.logger.logger.warning("⚠️ No state machine connected")
            return False
        
        # Get current state handler
        current_state_handler = self.state_machine.get_current_state_handler()
        
        # Send command enum to current state
        transition = current_state_handler.handle_event(command_type, data)
        
        # If state wants to transition, do it immediately
        if transition:
            new_state, reason = transition
            self.state_machine._transition_to(new_state, reason)
            return True
        
        # For commands that don't trigger transitions but are valid, return True to avoid "not handled" warning
        # This assumes the state logged any specific errors if the command failed
        NON_TRANSITION_COMMANDS = [
            CommandType.SET_VELOCITY, 
            CommandType.SET_PATH,
            CommandType.SET_INITIAL_POSITION,
            CommandType.ACTIVATE_V2V, 
            CommandType.DISABLE_V2V,
            CommandType.SET_PARAMS,
            CommandType.ENABLE_PLATOON_FOLLOWER, # In waiting state, this configures but doesn't transition
            CommandType.ENABLE_PLATOON_LEADER,
            CommandType.SETUP_PLATOON_FORMATION,  # New: global formation setup
            CommandType.DISABLE_PLATOON,  # Pause platoon without transition
            CommandType.MANUAL_CONTROL,  # Manual control commands update state but don't transition
            CommandType.CALIBRATE  # GPS recalibration without state transition
        ]
        
        if command_type in NON_TRANSITION_COMMANDS:
            return True
        
        return False
    
    def process_command(self, raw_command: Dict[str, Any]) -> bool:
        """
        Process a raw command from Ground Station by converting it to a CommandType
        and sending directly to current state
        
        Args:
            raw_command: Raw command dictionary from network
            
        Returns:
            True if command was processed successfully
        """
        try:
            # Parse command type and data
            command_info = self._parse_command(raw_command)
            if not command_info:
                self.commands_rejected += 1
                return False
            
            # Send CommandType enum directly to current state (no message conversion needed)
            success = self.dispatch_to_current_state(command_info.command_type, command_info.data)
            
            # Update statistics and history in one go
            current_time = time.time()
            if success:
                self.commands_processed += 1
                command_info.processed = True
            else:
                self.commands_rejected += 1
                self.logger.log_warning(f"Command type '{command_info.command_type.value}' was not handled")
            
            self._add_to_history(command_info)
            self.last_command_time = current_time
            
            return success
            
        except Exception as e:
            self.logger.log_error("Command processing error", e)
            self.commands_rejected += 1
            return False
    
    def _parse_command(self, raw_command: Dict[str, Any]) -> Optional[CommandInfo]:
        """Parse raw command into CommandInfo structure"""
        try:
            # Cache timestamp for better performance
            timestamp = time.time()
            
            # Handle command format (support both 'type' and 'command' keys)
            cmd_type_str = None
            if 'type' in raw_command:
                cmd_type_str = raw_command['type']
            elif 'command' in raw_command:
                cmd_type_str = raw_command['command']
            
            if cmd_type_str:
                # Use class-level mapping for better performance
                command_type = self.TYPE_MAPPING.get(cmd_type_str)
                if not command_type:
                    self.logger.log_warning(f"Unknown command type: {cmd_type_str}")
                    return None
                
                # Special handling for platoon commands
                if cmd_type_str == 'enable_platoon':
                    role = raw_command.get('role', 'follower')
                    if role == 'leader':
                        command_type = CommandType.ENABLE_PLATOON_LEADER
                    elif role == 'follower':
                        command_type = CommandType.ENABLE_PLATOON_FOLLOWER
                    else:
                        self.logger.log_warning(f"Invalid platoon role: {role}")
                        return None
                
                return CommandInfo(
                    command_type=command_type,
                    timestamp=timestamp,
                    data={**raw_command, 'source': raw_command.get('source', 'Ground Station')},  # Add source to data
                    source=raw_command.get('source', 'Ground Station')  # Also keep in CommandInfo source field
                )
            
            else:
                self.logger.log_warning(f"Unrecognized command format: {raw_command}")
                return None
                
        except Exception as e:
            self.logger.log_error("Command parsing error", e)
            return None
    
    def _add_to_history(self, command_info: CommandInfo):
        """Add command to history with size limit (optimized)"""
        self.command_history.append(command_info)
        # More efficient than checking length every time - only trim when we exceed
        if len(self.command_history) > self.max_history_size:
            # Remove multiple items at once for better performance
            self.command_history = self.command_history[-self.max_history_size:]
    
    def get_event_dispatcher(self) -> EventDispatcher:
        """Get reference to event dispatcher"""
        return self.event_dispatcher
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get command processing statistics"""
        return {
            'commands_processed': self.commands_processed,
            'commands_rejected': self.commands_rejected,
            'last_command_time': self.last_command_time,
            'history_size': len(self.command_history),
            'event_listeners': {k: len(v) for k, v in self.event_dispatcher.event_listeners.items()},
            'recent_events': len(self.event_dispatcher.event_history)
        }
    
    def dispatch_manual_event(self, command_type: CommandType, data: Dict[str, Any] = None, source: str = "manual"):
        """
        Manually dispatch a command (useful for testing or internal triggers)
        
        Args:
            command_type: CommandType enum to dispatch
            data: Event data
            source: Source of the event
        """
        return self.dispatch_to_current_state(command_type, data)