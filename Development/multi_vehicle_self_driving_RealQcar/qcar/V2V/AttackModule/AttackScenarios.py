"""
Attack Scenarios Factory Module

Provides convenient factory functions for creating and managing attack scenarios,
designed for integration with the V2V communication system.

This module simplifies the creation of common attack scenarios and provides
utilities for loading scenarios from YAML configuration files.

Features:
    - Predefined attack scenario templates (Bogus, DoS, Position, Velocity, etc.)
    - YAML configuration file loading
    - MATLAB-compatible API (atk_scenarios function)
    - Programmatic scenario creation helpers

Author: Fleet Framework Security Research
Version: 2.0.0
Updated: January 2026
"""

import yaml
import logging
from typing import Dict, List, Optional, Union, Any
from pathlib import Path

from .AttackModule import (
    AttackModule, AttackScenario, 
    AttackType, DataType, ModificationType
)


def make_scenario(
    attacker_id: int, 
    victim_ids: Union[int, List[int]], 
    t_start: float, 
    t_end: float,
    modification_type: str, 
    intensity: Any,
    data_type: str, 
    target_fields: List[str],
    attack_type: str = "Bogus",
    scenario_name: str = "",
    description: str = ""
) -> AttackScenario:
    """
    Create an attack scenario with the given parameters.
    
    This is the primary factory function for creating attack scenarios.
    Similar to MATLAB's makeScenario function for compatibility.
    
    Args:
        attacker_id: Vehicle ID of the attacker
        victim_ids: Vehicle ID(s) of victims (-1 for all vehicles)
        t_start: Attack start time (seconds from simulation start)
        t_end: Attack end time (seconds from simulation start)
        modification_type: Type of modification:
            - 'scaling': value * factor
            - 'bias': value + offset
            - 'linear': value + rate * time
            - 'sinusoidal': value + A*sin(2πft + φ)
            - 'faulty': value + N(0, σ)
            - 'zero': set to 0
            - 'constant': set to fixed value
            - 'random': random value in range
        intensity: Modification intensity (float or dict for sinusoidal/random)
        data_type: Data type to attack:
            - 'local': Attack local state broadcasts
            - 'fleet': Attack fleet state broadcasts
            - 'both': Attack both local and fleet
        target_fields: List of fields to attack:
            - 'x', 'y': Position coordinates (meters)
            - 'theta': Heading angle (radians)
            - 'velocity': Velocity (m/s)
            - 'acceleration': Acceleration (m/s²)
        attack_type: Attack type ('Bogus', 'DoS', 'POS', 'VEL', 'ACC', 'Collusion')
        scenario_name: Human-readable name for the scenario
        description: Description of the attack
        
    Returns:
        AttackScenario object ready to be added to AttackModule
        
    Example:
        >>> scenario = make_scenario(
        ...     attacker_id=1, victim_ids=-1,
        ...     t_start=10.0, t_end=20.0,
        ...     modification_type='scaling', intensity=0.5,
        ...     data_type='local', target_fields=['velocity'],
        ...     scenario_name="VelocityHalving"
        ... )
    """
    # Convert single victim to list
    if isinstance(victim_ids, int):
        victim_ids = [victim_ids]
    
    # Auto-generate scenario name if not provided
    if not scenario_name:
        fields_str = '_'.join(target_fields)
        scenario_name = f"{attack_type}_{modification_type}_{fields_str}"
    
    # Auto-generate description if not provided
    if not description:
        description = f"{attack_type} attack: {modification_type} on {target_fields}"
    
    # Create and return scenario
    return AttackScenario(
        t_start=t_start,
        t_end=t_end,
        attacker_id=attacker_id,
        victim_ids=victim_ids,
        attack_type=AttackType(attack_type),
        modification_type=ModificationType(modification_type),
        data_type=DataType(data_type),
        intensity=intensity,
        target_fields=target_fields,
        scenario_name=scenario_name,
        description=description
    )


# =============================================================================
# Predefined Bogus Attack Scenarios (10 cases from MATLAB implementation)
# =============================================================================

def create_bogus_scenarios(
    attack_module: AttackModule, 
    case_number: int,
    attacker_id: int = 1, 
    victim_id: int = -1,
    t_start: float = 10.0, 
    t_end: float = 15.0,
    data_type: str = "local"
) -> AttackModule:
    """
    Create bogus message attack scenarios matching MATLAB implementation.
    
    Bogus attacks send falsified data to deceive other vehicles.
    
    Args:
        attack_module: AttackModule to add scenarios to
        case_number: Case number (1-10) defining the attack variant:
            1: Velocity scaling by -0.5 (reverse and reduce)
            2: Velocity scaling by 0.5 (reduce by half)
            3: Position X bias -15m
            4: Position X bias +33m
            5: Velocity linear drift 0.1 m/s²
            6: Position X sinusoidal (5m @ 0.5Hz)
            7: Position X noisy (σ=10m)
            8: Velocity noisy (σ=2.5 m/s)
            9: Acceleration noisy (σ=0.2 m/s²)
            10: Velocity bias +2 m/s
        attacker_id: Vehicle ID of the attacker
        victim_id: Vehicle ID of victim (-1 for all)
        t_start: Attack start time
        t_end: Attack end time
        data_type: Data type to attack ('local', 'fleet', 'both')
        
    Returns:
        AttackModule with scenario added
        
    Raises:
        ValueError: If case_number is not 1-10
    """
    scenarios = {
        1: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'scaling', -0.5, data_type, ['velocity'],
            scenario_name="Bogus_Case1_VelScalingNeg",
            description="Velocity scaled by -0.5 (reversed and reduced)"
        ),
        
        2: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'scaling', 0.5, data_type, ['velocity'],
            scenario_name="Bogus_Case2_VelScalingPos",
            description="Velocity scaled by 0.5 (halved)"
        ),
        
        3: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'bias', -15.0, data_type, ['x'],
            scenario_name="Bogus_Case3_PosBiasNeg",
            description="X position bias -15m"
        ),
        
        4: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'bias', 33.0, data_type, ['x'],
            scenario_name="Bogus_Case4_PosBiasPos",
            description="X position bias +33m"
        ),
        
        5: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'linear', 0.1, data_type, ['velocity'],
            scenario_name="Bogus_Case5_VelLinear",
            description="Velocity linear drift 0.1 m/s per second"
        ),
        
        6: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'sinusoidal', {'amplitude': 5.0, 'frequency': 0.5, 'phase': 0.0},
            data_type, ['x'],
            scenario_name="Bogus_Case6_PosSinusoidal",
            description="X position sinusoidal oscillation (5m @ 0.5Hz)"
        ),
        
        7: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'faulty', 10.0, data_type, ['x'],
            scenario_name="Bogus_Case7_PosFaultyHigh",
            description="X position noise injection (σ=10m)"
        ),
        
        8: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'faulty', 2.5, data_type, ['velocity'],
            scenario_name="Bogus_Case8_VelFaulty",
            description="Velocity noise injection (σ=2.5 m/s)"
        ),
        
        9: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'faulty', 0.2, data_type, ['acceleration'],
            scenario_name="Bogus_Case9_AccFaulty",
            description="Acceleration noise injection (σ=0.2 m/s²)"
        ),
        
        10: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'bias', 2.0, data_type, ['velocity'],
            scenario_name="Bogus_Case10_VelBiasSmall",
            description="Velocity bias +2 m/s"
        ),
    }
    
    if case_number not in scenarios:
        raise ValueError(f"Invalid Bogus case number: {case_number}. Must be 1-10.")
    
    attack_module.add_scenario(scenarios[case_number])
    return attack_module


# =============================================================================
# Denial of Service (DoS) Attack Scenarios
# =============================================================================

def create_dos_scenarios(
    attack_module: AttackModule, 
    case_number: int,
    attacker_id: int = 1, 
    victim_id: int = -1,
    t_start: float = 10.0, 
    t_end: float = 15.0,
    data_type: str = "local"
) -> AttackModule:
    """
    Create Denial of Service (DoS) attack scenarios.
    
    DoS attacks set data to zero, effectively denying service.
    
    Args:
        attack_module: AttackModule to add scenarios to
        case_number: Case number defining the attack variant:
            1: Set velocity to zero
            2: Set position (X, Y) to zero
            3: Set all state (X, Y, velocity) to zero
        attacker_id: Vehicle ID of the attacker
        victim_id: Vehicle ID of victim (-1 for all)
        t_start: Attack start time
        t_end: Attack end time
        data_type: Data type to attack
        
    Returns:
        AttackModule with scenario added
    """
    scenarios = {
        1: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'zero', 0, data_type, ['velocity'],
            attack_type="DoS",
            scenario_name="DoS_Case1_VelZero",
            description="Set velocity to zero"
        ),
        
        2: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'zero', 0, data_type, ['x', 'y'],
            attack_type="DoS",
            scenario_name="DoS_Case2_PosZero",
            description="Set position to zero"
        ),
        
        3: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'zero', 0, data_type, ['x', 'y', 'velocity'],
            attack_type="DoS",
            scenario_name="DoS_Case3_AllZero",
            description="Set all state to zero"
        ),
    }
    
    if case_number not in scenarios:
        raise ValueError(f"Invalid DoS case number: {case_number}. Must be 1-3.")
    
    attack_module.add_scenario(scenarios[case_number])
    return attack_module


# =============================================================================
# Position-Specific (POS) Attack Scenarios
# =============================================================================

def create_position_attack(
    attack_module: AttackModule, 
    case_number: int,
    attacker_id: int = 1, 
    victim_id: int = -1,
    t_start: float = 10.0, 
    t_end: float = 15.0,
    data_type: str = "local"
) -> AttackModule:
    """
    Create position-specific (POS) attack scenarios.
    
    These attacks specifically target position data (GPS spoofing simulation).
    
    Args:
        attack_module: AttackModule to add scenarios to
        case_number: Case number defining the attack variant:
            1: Position bias +10m on X and Y
            2: Position scaling by 1.5
            3: Position noise (σ=5m)
        attacker_id: Vehicle ID of the attacker
        victim_id: Vehicle ID of victim (-1 for all)
        t_start: Attack start time
        t_end: Attack end time
        data_type: Data type to attack
        
    Returns:
        AttackModule with scenario added
    """
    scenarios = {
        1: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'bias', 10.0, data_type, ['x', 'y'],
            attack_type="POS",
            scenario_name="POS_Case1_Bias",
            description="Position bias +10m on X and Y"
        ),
        
        2: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'scaling', 1.5, data_type, ['x', 'y'],
            attack_type="POS",
            scenario_name="POS_Case2_Scaling",
            description="Position scaling by 1.5"
        ),
        
        3: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'faulty', 5.0, data_type, ['x', 'y'],
            attack_type="POS",
            scenario_name="POS_Case3_Noisy",
            description="Position noise (σ=5m)"
        ),
    }
    
    if case_number not in scenarios:
        raise ValueError(f"Invalid POS case number: {case_number}. Must be 1-3.")
    
    attack_module.add_scenario(scenarios[case_number])
    return attack_module


# =============================================================================
# Velocity-Specific (VEL) Attack Scenarios
# =============================================================================

def create_velocity_attack(
    attack_module: AttackModule, 
    case_number: int,
    attacker_id: int = 1, 
    victim_id: int = -1,
    t_start: float = 10.0, 
    t_end: float = 15.0,
    data_type: str = "local"
) -> AttackModule:
    """
    Create velocity-specific (VEL) attack scenarios.
    
    These attacks specifically target velocity data (speedometer tampering).
    
    Args:
        attack_module: AttackModule to add scenarios to
        case_number: Case number defining the attack variant:
            1: Constant velocity 3 m/s
            2: Velocity bias +2 m/s
            3: Velocity doubled (scaling by 2)
        attacker_id: Vehicle ID of the attacker
        victim_id: Vehicle ID of victim (-1 for all)
        t_start: Attack start time
        t_end: Attack end time
        data_type: Data type to attack
        
    Returns:
        AttackModule with scenario added
    """
    scenarios = {
        1: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'constant', 3.0, data_type, ['velocity'],
            attack_type="VEL",
            scenario_name="VEL_Case1_Constant",
            description="Constant velocity 3 m/s"
        ),
        
        2: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'bias', 2.0, data_type, ['velocity'],
            attack_type="VEL",
            scenario_name="VEL_Case2_Bias",
            description="Velocity bias +2 m/s"
        ),
        
        3: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'scaling', 2.0, data_type, ['velocity'],
            attack_type="VEL",
            scenario_name="VEL_Case3_Exaggerate",
            description="Velocity doubled"
        ),
    }
    
    if case_number not in scenarios:
        raise ValueError(f"Invalid VEL case number: {case_number}. Must be 1-3.")
    
    attack_module.add_scenario(scenarios[case_number])
    return attack_module


# =============================================================================
# Acceleration-Specific (ACC) Attack Scenarios
# =============================================================================

def create_acceleration_attack(
    attack_module: AttackModule, 
    case_number: int,
    attacker_id: int = 1, 
    victim_id: int = -1,
    t_start: float = 10.0, 
    t_end: float = 15.0,
    data_type: str = "local"
) -> AttackModule:
    """
    Create acceleration-specific (ACC) attack scenarios.
    
    These attacks specifically target acceleration data (IMU manipulation).
    
    Args:
        attack_module: AttackModule to add scenarios to
        case_number: Case number defining the attack variant:
            1: Acceleration bias +1 m/s²
            2: Acceleration noise (σ=0.5 m/s²)
            3: Zero acceleration (constant velocity claim)
        attacker_id: Vehicle ID of the attacker
        victim_id: Vehicle ID of victim (-1 for all)
        t_start: Attack start time
        t_end: Attack end time
        data_type: Data type to attack
        
    Returns:
        AttackModule with scenario added
    """
    scenarios = {
        1: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'bias', 1.0, data_type, ['acceleration'],
            attack_type="ACC",
            scenario_name="ACC_Case1_Bias",
            description="Acceleration bias +1 m/s²"
        ),
        
        2: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'faulty', 0.5, data_type, ['acceleration'],
            attack_type="ACC",
            scenario_name="ACC_Case2_Noisy",
            description="Acceleration noise (σ=0.5 m/s²)"
        ),
        
        3: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'constant', 0.0, data_type, ['acceleration'],
            attack_type="ACC",
            scenario_name="ACC_Case3_Zero",
            description="Zero acceleration (constant velocity claim)"
        ),
    }
    
    if case_number not in scenarios:
        raise ValueError(f"Invalid ACC case number: {case_number}. Must be 1-3.")
    
    attack_module.add_scenario(scenarios[case_number])
    return attack_module


# =============================================================================
# Heading-Specific Attack Scenarios
# =============================================================================

def create_heading_attack(
    attack_module: AttackModule, 
    case_number: int,
    attacker_id: int = 1, 
    victim_id: int = -1,
    t_start: float = 10.0, 
    t_end: float = 15.0,
    data_type: str = "local"
) -> AttackModule:
    """
    Create heading-specific attack scenarios.
    
    These attacks specifically target heading/theta data (compass spoofing).
    
    Args:
        attack_module: AttackModule to add scenarios to
        case_number: Case number defining the attack variant:
            1: Heading bias +0.5 rad (~28°)
            2: Heading sinusoidal oscillation
            3: Heading noise (σ=0.1 rad)
        attacker_id: Vehicle ID of the attacker
        victim_id: Vehicle ID of victim (-1 for all)
        t_start: Attack start time
        t_end: Attack end time
        data_type: Data type to attack
        
    Returns:
        AttackModule with scenario added
    """
    scenarios = {
        1: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'bias', 0.5, data_type, ['theta'],
            attack_type="Bogus",
            scenario_name="Heading_Case1_Bias",
            description="Heading bias +0.5 rad (~28°)"
        ),
        
        2: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'sinusoidal', {'amplitude': 0.3, 'frequency': 0.2, 'phase': 0.0},
            data_type, ['theta'],
            attack_type="Bogus",
            scenario_name="Heading_Case2_Sinusoidal",
            description="Heading sinusoidal oscillation (0.3 rad @ 0.2Hz)"
        ),
        
        3: make_scenario(
            attacker_id, victim_id, t_start, t_end,
            'faulty', 0.1, data_type, ['theta'],
            attack_type="Bogus",
            scenario_name="Heading_Case3_Noisy",
            description="Heading noise (σ=0.1 rad)"
        ),
    }
    
    if case_number not in scenarios:
        raise ValueError(f"Invalid Heading case number: {case_number}. Must be 1-3.")
    
    attack_module.add_scenario(scenarios[case_number])
    return attack_module


# =============================================================================
# MATLAB-Compatible Main Dispatcher
# =============================================================================

def atk_scenarios(
    attack_module: AttackModule, 
    attack_type: str, 
    data_type: str,
    case_number: int, 
    t_start: float, 
    t_end: float,
    attacker_id: int, 
    victim_id: int = -1
) -> AttackModule:
    """
    Main function for creating attack scenarios based on attack type and case number.
    
    This function provides a MATLAB-compatible API similar to Atk_Scenarios.m
    
    Args:
        attack_module: AttackModule to configure
        attack_type: Type of attack:
            - 'Bogus': Falsified data attacks (10 cases)
            - 'DoS': Denial of service attacks (3 cases)
            - 'POS': Position-specific attacks (3 cases)
            - 'VEL': Velocity-specific attacks (3 cases)
            - 'ACC': Acceleration-specific attacks (3 cases)
            - 'Heading': Heading-specific attacks (3 cases)
            - 'None': No attack
        data_type: Data type to attack ('local', 'fleet', 'both')
        case_number: Case number for the specific attack variant
        t_start: Attack start time (seconds)
        t_end: Attack end time (seconds)
        attacker_id: Vehicle ID of the attacker
        victim_id: Vehicle ID of victim (-1 for all)
        
    Returns:
        AttackModule with configured scenario
        
    Example:
        >>> attack_module = AttackModule(vehicle_id=1)
        >>> atk_scenarios(attack_module, "Bogus", "local", 2, 10.0, 15.0, 1, -1)
    """
    attack_type_upper = attack_type.upper()
    
    dispatch = {
        "BOGUS": create_bogus_scenarios,
        "DOS": create_dos_scenarios,
        "POS": create_position_attack,
        "VEL": create_velocity_attack,
        "ACC": create_acceleration_attack,
        "HEADING": create_heading_attack,
    }
    
    if attack_type_upper == "NONE":
        return attack_module
    
    if attack_type_upper not in dispatch:
        raise ValueError(f"Unknown attack type: {attack_type}. "
                        f"Available: {list(dispatch.keys())}")
    
    return dispatch[attack_type_upper](
        attack_module, case_number, attacker_id, 
        victim_id, t_start, t_end, data_type
    )


# =============================================================================
# YAML Configuration Loading
# =============================================================================

def load_scenarios_from_config(
    attack_module: AttackModule,
    config_path: str = "attack_config.yaml",
    enabled_only: bool = True,
    logger: Optional[logging.Logger] = None
) -> AttackModule:
    """
    Load attack scenarios from YAML configuration file.
    
    Args:
        attack_module: AttackModule to add scenarios to
        config_path: Path to YAML configuration file
        enabled_only: If True, only load enabled scenarios
        logger: Logger for messages (uses attack_module's logger if None)
        
    Returns:
        AttackModule with loaded scenarios
        
    Configuration File Format:
        attack_settings:
          enabled: true
          dt: 0.01
          log_level: "WARNING"
        
        scenarios:
          - name: "Scenario_Name"
            enabled: true
            attack_type: "Bogus"
            modification_type: "scaling"
            data_type: "local"
            t_start: 10.0
            t_end: 15.0
            attacker_id: 1
            victim_ids: [-1]
            intensity: 0.5
            target_fields: ["velocity"]
            description: "Description"
    """
    logger = logger or attack_module.logger
    config_file = Path(config_path)
    
    if not config_file.exists():
        if logger:
            logger.warning(f"Attack config file not found: {config_path}")
        return attack_module
    
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
    except yaml.YAMLError as e:
        if logger:
            logger.error(f"Error parsing YAML config: {e}")
        return attack_module
    
    # Check global enable
    attack_settings = config.get('attack_settings', {})
    if not attack_settings.get('enabled', False) and enabled_only:
        if logger:
            logger.info("Attacks globally disabled in config")
        return attack_module
    
    # Load scenarios
    scenarios_config = config.get('scenarios', [])
    loaded_count = 0
    
    for scenario_data in scenarios_config:
        # Skip disabled scenarios if enabled_only
        if enabled_only and not scenario_data.get('enabled', True):
            continue
        
        try:
            # Parse intensity (handle dict for sinusoidal/random)
            intensity = scenario_data.get('intensity', 1.0)
            if isinstance(intensity, dict):
                pass  # Keep as dict
            else:
                intensity = float(intensity)
            
            # Parse victim_ids
            victim_ids = scenario_data.get('victim_ids', [-1])
            if isinstance(victim_ids, int):
                victim_ids = [victim_ids]
            else:
                victim_ids = [int(v) for v in victim_ids]
            
            # Create scenario
            scenario = AttackScenario(
                t_start=float(scenario_data['t_start']),
                t_end=float(scenario_data['t_end']),
                attacker_id=int(scenario_data['attacker_id']),
                victim_ids=victim_ids,
                attack_type=AttackType(scenario_data['attack_type']),
                modification_type=ModificationType(scenario_data['modification_type']),
                data_type=DataType(scenario_data['data_type']),
                intensity=intensity,
                target_fields=scenario_data['target_fields'],
                scenario_name=scenario_data.get('name', ''),
                description=scenario_data.get('description', '')
            )
            
            attack_module.add_scenario(scenario)
            loaded_count += 1
            
        except (KeyError, ValueError) as e:
            if logger:
                logger.error(
                    f"Error loading scenario '{scenario_data.get('name', 'unknown')}': {e}"
                )
            continue
    
    if logger:
        logger.info(f"Loaded {loaded_count} attack scenario(s) from {config_path}")
    
    return attack_module


def create_multi_vehicle_attack(
    attack_module: AttackModule,
    attacker_ids: List[int],
    victim_ids: List[int],
    t_start: float,
    t_end: float,
    attack_type: str,
    modification_type: str,
    intensity: Any,
    target_fields: List[str],
    data_type: str = "local",
    scenario_name: str = ""
) -> AttackModule:
    """
    Create a coordinated multi-vehicle attack (collusion).
    
    This creates identical attack scenarios for multiple attacker vehicles,
    simulating a coordinated attack.
    
    Args:
        attack_module: AttackModule to add scenarios to
        attacker_ids: List of attacker vehicle IDs
        victim_ids: List of victim vehicle IDs (-1 for all)
        t_start: Attack start time
        t_end: Attack end time
        attack_type: Type of attack
        modification_type: Modification type
        intensity: Attack intensity
        target_fields: Fields to attack
        data_type: Data type to attack
        scenario_name: Base name for scenarios
        
    Returns:
        AttackModule with collusion scenarios added
    """
    for i, attacker_id in enumerate(attacker_ids):
        name = f"{scenario_name or 'Collusion'}_Attacker{attacker_id}"
        
        scenario = make_scenario(
            attacker_id=attacker_id,
            victim_ids=victim_ids,
            t_start=t_start,
            t_end=t_end,
            modification_type=modification_type,
            intensity=intensity,
            data_type=data_type,
            target_fields=target_fields,
            attack_type=attack_type,
            scenario_name=name,
            description=f"Collusion attack from vehicle {attacker_id}"
        )
        
        attack_module.add_scenario(scenario)
    
    return attack_module


# =============================================================================
# Module Entry Point
# =============================================================================

if __name__ == "__main__":
    # Example usage
    print("Attack Scenarios Module - Example Usage")
    print("=" * 50)
    
    # Create attack module
    attack_module = AttackModule(vehicle_id=1)
    
    # Add Bogus case 2 (velocity scaling)
    atk_scenarios(attack_module, "Bogus", "local", 2, 10.0, 15.0, 1, -1)
    
    print(f"Configured {attack_module.get_scenario_count()} scenario(s)")
    
    # Example: Load from config
    # load_scenarios_from_config(attack_module, "attack_config.yaml")
