"""
Example Ground Station Commands - Improved Structure

This file demonstrates the improved command structure with clear naming
conventions and proper validation.
"""

# ===== NEW IMPROVED COMMAND FORMAT =====

# Basic Movement Commands
STOP_COMMAND = {
    "type": "stop",
    "timestamp": 1699876543.123,
    "source": "ground_station_gui"
}

START_COMMAND = {
    "type": "start", 
    "timestamp": 1699876543.123,
    "source": "ground_station_gui"
}

EMERGENCY_STOP_COMMAND = {
    "type": "emergency_stop",
    "timestamp": 1699876543.123,
    "source": "safety_system"
}

# Parameter Update Commands
SET_VELOCITY_COMMAND = {
    "type": "set_velocity",
    "v_ref": 1.2,
    "timestamp": 1699876543.123,
    "source": "ground_station_gui"
}

SET_PATH_COMMAND = {
    "type": "set_path",
    "node_sequence": [18, 19, 20, 21, 22],
    "timestamp": 1699876543.123,
    "source": "ground_station_gui"
}

SET_MULTIPLE_PARAMS_COMMAND = {
    "type": "set_params",
    "v_ref": 1.5,
    "node_sequence": [10, 11, 12, 13],
    "timestamp": 1699876543.123,
    "source": "ground_station_gui"
}

# Platoon Commands
ENABLE_PLATOON_LEADER_COMMAND = {
    "type": "enable_platoon",
    "role": "leader",
    "timestamp": 1699876543.123,
    "source": "fleet_coordinator"
}

ENABLE_PLATOON_FOLLOWER_COMMAND = {
    "type": "enable_platoon", 
    "role": "follower",
    "leader_id": 0,
    "following_distance": 2.0,
    "timestamp": 1699876543.123,
    "source": "fleet_coordinator"
}

DISABLE_PLATOON_COMMAND = {
    "type": "disable_platoon",
    "timestamp": 1699876543.123,
    "source": "fleet_coordinator"
}

# System Commands  
SHUTDOWN_COMMAND = {
    "type": "shutdown",
    "timestamp": 1699876543.123,
    "source": "ground_station_admin"
}

RESET_COMMAND = {
    "type": "reset",
    "timestamp": 1699876543.123,
    "source": "ground_station_admin"
}

# ===== LEGACY COMMAND FORMAT (STILL SUPPORTED) =====

LEGACY_STOP_COMMAND = {
    "command": "stop",
    "timestamp": 1699876543.123
}

LEGACY_START_COMMAND = {
    "command": "start", 
    "timestamp": 1699876543.123
}

LEGACY_VELOCITY_UPDATE = {
    "v_ref": 1.0,
    "timestamp": 1699876543.123
}

LEGACY_MIXED_COMMAND = {
    "command": "resume",
    "v_ref": 1.3,
    "timestamp": 1699876543.123
}


# ===== COMMAND VALIDATION EXAMPLES =====

def demonstrate_command_validation():
    """
    Example of how commands are validated based on current state
    """
    
    # Valid commands by state
    valid_commands_by_state = {
        "STOPPED": [
            "start",
            "set_velocity", 
            "set_path",
            "shutdown"
        ],
        
        "WAITING_FOR_START": [
            "start",
            "set_velocity",
            "set_path", 
            "stop",
            "shutdown"
        ],
        
        "FOLLOWING_PATH": [
            "stop",
            "emergency_stop",
            "set_velocity",
            "enable_platoon",
            "shutdown"
        ],
        
        "FOLLOWING_LEADER": [
            "stop", 
            "emergency_stop",
            "set_velocity",
            "disable_platoon",
            "shutdown"
        ]
    }
    
    # Invalid command examples
    invalid_commands = [
        {
            "description": "Cannot start from FOLLOWING_PATH state",
            "current_state": "FOLLOWING_PATH", 
            "command": {"type": "start"},
            "expected_error": "Cannot start from state FOLLOWING_PATH"
        },
        
        {
            "description": "Invalid velocity value",
            "current_state": "STOPPED",
            "command": {"type": "set_velocity", "v_ref": 5.0},
            "expected_error": "Invalid v_ref value: 5.0 (must be 0-2.0)"
        },
        
        {
            "description": "Enable platoon follower without leader_id",
            "current_state": "FOLLOWING_PATH",
            "command": {"type": "enable_platoon", "role": "follower"},
            "expected_error": "Leader ID required for follower mode"
        }
    ]


# ===== COMMAND FLAG USAGE EXAMPLES =====

def demonstrate_command_flag_usage():
    """
    Example of how to use command flags in state machine
    """
    
    # In state machine handlers, use these patterns:
    
    # Check for start command (preferred new way)
    '''
    if self.check_start_command():
        return 0.0, 0.0, (VehicleState.FOLLOWING_PATH, StateTransitionReason.START_COMMAND)
    '''
    
    # Check for stop command
    '''
    if self.check_stop_command():
        return 0.0, 0.0, (VehicleState.STOPPED, StateTransitionReason.STOP_COMMAND)
    '''
    
    # Check for velocity updates
    '''
    has_update, new_velocity = self.check_velocity_update_command()
    if has_update and new_velocity is not None:
        self.vehicle_logic.v_ref = new_velocity
        self.logger.logger.info(f"Updated v_ref to {new_velocity} m/s")
    '''
    
    # Check for platoon commands
    '''
    if self.check_platoon_leader_command():
        return 0.0, 0.0, (VehicleState.PLATOON_LEADER_FORMING, StateTransitionReason.PLATOON_COMMAND)
        
    if self.check_platoon_follower_command():
        return 0.0, 0.0, (VehicleState.PLATOON_FOLLOWER_SEARCHING, StateTransitionReason.PLATOON_COMMAND)
    '''


# ===== BENEFITS OF NEW SYSTEM =====

"""
IMPROVEMENTS OVER OLD SYSTEM:

1. CLEAR VARIABLE NAMING
   ❌ Old: start_commanded, platoon_follower_mode 
   ✅ New: cmd_start_requested, cmd_platoon_follower_mode_requested

2. CENTRALIZED COMMAND PROCESSING
   ❌ Old: Scattered command handling in vehicle_logic._process_commands()
   ✅ New: Centralized CommandHandler with validation

3. THREAD-SAFE FLAG MANAGEMENT
   ❌ Old: Direct attribute setting without synchronization
   ✅ New: Thread-safe CommandFlags with locks

4. STATE-AWARE COMMAND VALIDATION
   ❌ Old: No validation, commands processed regardless of state
   ✅ New: Commands validated based on current vehicle state

5. COMMAND HISTORY AND STATISTICS
   ❌ Old: No command tracking or statistics
   ✅ New: Command history, processing stats, error tracking

6. CONSISTENT API
   ❌ Old: Multiple command formats (legacy vs new)
   ✅ New: Unified API with legacy support for backward compatibility

7. BETTER ERROR HANDLING
   ❌ Old: Silent failures or basic logging
   ✅ New: Detailed error messages and validation feedback

8. EXTENSIBLE DESIGN
   ❌ Old: Hard to add new command types
   ✅ New: Easy to add new CommandType enum values and handlers
"""


if __name__ == "__main__":
    print("Ground Station Command Examples")
    print("=" * 50)
    
    print("\nNew Command Format Examples:")
    print("Stop:", STOP_COMMAND)
    print("Set Velocity:", SET_VELOCITY_COMMAND) 
    print("Enable Platoon Follower:", ENABLE_PLATOON_FOLLOWER_COMMAND)
    
    print("\nLegacy Command Format (still supported):")
    print("Legacy Stop:", LEGACY_STOP_COMMAND)
    print("Legacy Velocity:", LEGACY_VELOCITY_UPDATE)
    
    demonstrate_command_validation()
    demonstrate_command_flag_usage()