"""
Command Types - Centralized command type definitions

This module provides shared command type definitions that can be imported
by all components that need to work with vehicle commands.
"""

from enum import Enum


class CommandType(Enum):
    """Standard command types for vehicle control"""

    # Basic movement commands
    STOP = "stop"
    START = "start"
    EMERGENCY_STOP = "emergency_stop"

    # Parameter commands
    SET_VELOCITY = "set_velocity"
    SET_PATH = "set_path"
    SET_PARAMS = "set_params"
    SET_INITIAL_POSITION = "set_initial_position"
    SET_GEAR = "set_gear"

    # Platoon commands
    ENABLE_PLATOON_LEADER = "enable_platoon_leader"
    ENABLE_PLATOON_FOLLOWER = "enable_platoon_follower"
    DISABLE_PLATOON = "disable_platoon"
    SETUP_PLATOON_FORMATION = "setup_platoon_formation"  # Global formation setup
    START_PLATOON = "start_platoon"  # Trigger platoon start after setup

    # V2V commands
    ACTIVATE_V2V = "activate_v2v"
    DISABLE_V2V = "disable_v2v"

    # Perception commands
    ACTIVATE_PERCEPTION = "activate_perception"
    DISABLE_PERCEPTION = "disable_perception"

    # Visualization commands
    ACTIVATE_SCOPES = "activate_scopes"
    DISABLE_SCOPES = "disable_scopes"
    ENABLE_SCOPE_STREAMING = "enable_scope_streaming"
    DISABLE_SCOPE_STREAMING = "disable_scope_streaming"

    # Manual control commands
    ENABLE_MANUAL_MODE = "enable_manual_mode"
    MANUAL_CONTROL = "manual_control"
    DISABLE_MANUAL_MODE = "disable_manual_mode"

    # Taxi commands
    ENABLE_TAXI_MODE = "enable_taxi_mode"
    DISABLE_TAXI_MODE = "disable_taxi_mode"
    SET_TAXI_TRIP = "set_taxi_trip"

    # System commands
    SHUTDOWN = "shutdown"
    RESET = "reset"
    CALIBRATE = "calibrate"
    ENABLE_CALIBRATION_MODE = "enable_calibration_mode"
    ENABLE_ONLINE_CALIBRATION = "enable_online_calibration"
    DISABLE_ONLINE_CALIBRATION = "disable_online_calibration"

    # Runtime switching commands
    SET_LOCAL_OBSERVER = "set_local_observer"
    SET_FLEET_OBSERVER = "set_fleet_observer"
    SET_CONTROLLER = "set_controller"


# Convenience functions for command type checking
def is_movement_command(cmd_type: CommandType) -> bool:
    """Check if a command is a movement-related command"""
    return cmd_type in [CommandType.STOP, CommandType.START, CommandType.EMERGENCY_STOP]


def is_platoon_command(cmd_type: CommandType) -> bool:
    """Check if a command is platoon-related"""
    return cmd_type in [
        CommandType.ENABLE_PLATOON_LEADER,
        CommandType.ENABLE_PLATOON_FOLLOWER,
        CommandType.DISABLE_PLATOON,
        CommandType.SETUP_PLATOON_FORMATION,
        CommandType.START_PLATOON,
    ]


def is_v2v_command(cmd_type: CommandType) -> bool:
    """Check if a command is V2V-related"""
    return cmd_type in [CommandType.ACTIVATE_V2V, CommandType.DISABLE_V2V]


def is_manual_control_command(cmd_type: CommandType) -> bool:
    """Check if a command is manual control-related"""
    return cmd_type in [
        CommandType.ENABLE_MANUAL_MODE,
        CommandType.MANUAL_CONTROL,
        CommandType.DISABLE_MANUAL_MODE,
    ]


def is_system_command(cmd_type: CommandType) -> bool:
    """Check if a command is system-related"""
    return cmd_type in [CommandType.SHUTDOWN, CommandType.RESET]


def is_taxi_command(cmd_type: CommandType) -> bool:
    """Check if a command is taxi-related"""
    return cmd_type in [
        CommandType.ENABLE_TAXI_MODE,
        CommandType.DISABLE_TAXI_MODE,
        CommandType.SET_TAXI_TRIP,
    ]


def get_command_category(cmd_type: CommandType) -> str:
    """Get the category of a command"""
    if is_movement_command(cmd_type):
        return "movement"
    elif is_platoon_command(cmd_type):
        return "platoon"
    elif is_v2v_command(cmd_type):
        return "v2v"
    elif is_manual_control_command(cmd_type):
        return "manual_control"
    elif is_system_command(cmd_type):
        return "system"
    elif is_taxi_command(cmd_type):
        return "taxi"
    elif cmd_type in [
        CommandType.SET_VELOCITY,
        CommandType.SET_PATH,
        CommandType.SET_PARAMS,
    ]:
        return "parameter"
    else:
        return "unknown"
