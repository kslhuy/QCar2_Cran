"""
Attack Scenarios Helper Module

Provides convenient functions for creating and managing attack scenarios,
similar to the MATLAB Atk_Scenarios function.

This module simplifies the creation of common attack scenarios and provides
utilities for loading scenarios from configuration files.
"""

import yaml
import numpy as np
from typing import Dict, List, Optional, Union, Any
from AttackModule import AttackModule, AttackScenario, AttackType, DataType, ModificationType


def make_scenario(attacker_id: int, victim_ids: Union[int, List[int]], 
                 t_start: float, t_end: float,
                 modification_type: str, intensity: Any,
                 data_type: str, target_fields: List[str],
                 attack_type: str = "Bogus",
                 scenario_name: str = "",
                 description: str = "") -> AttackScenario:
    """
    Create an attack scenario with the given parameters.
    Similar to MATLAB's makeScenario function.
    
    Args:
        attacker_id: Vehicle ID of the attacker
        victim_ids: Vehicle ID(s) of victims (-1 for all)
        t_start: Attack start time (seconds)
        t_end: Attack end time (seconds)
        modification_type: Type of modification ('scaling', 'bias', 'linear', 'sinusoidal', 'faulty')
        intensity: Modification intensity (float or dict for sinusoidal)
        data_type: Data type to attack ('local', 'fleet', 'both')
        target_fields: List of fields to attack (['X', 'Y', 'velocity', 'acceleration', 'theta'])
        attack_type: Attack type ('Bogus', 'DoS', 'POS', 'VEL', 'ACC')
        scenario_name: Human-readable name for the scenario
        description: Description of the attack
        
    Returns:
        AttackScenario object
    """
    # Convert single victim to list
    if isinstance(victim_ids, int):
        victim_ids = [victim_ids]
    
    # Auto-generate scenario name if not provided
    if not scenario_name:
        scenario_name = f"{attack_type}_{modification_type}_{','.join(target_fields)}"
    
    # Auto-generate description if not provided
    if not description:
        description = f"{attack_type} attack: {modification_type} on {target_fields}"
    
    # Create scenario
    scenario = AttackScenario(
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
    
    return scenario


def create_bogus_scenarios(attack_module: AttackModule, case_number: int,
                          attacker_id: int = 1, victim_id: int = -1,
                          t_star: float = 10.0, t_end: float = 15.0,
                          data_type: str = "local") -> AttackModule:
    """
    Create bogus message attack scenarios matching MATLAB implementation.
    
    Args:
        attack_module: AttackModule to add scenarios to
        case_number: Case number (1-10) defining the attack variant
        attacker_id: Vehicle ID of the attacker
        victim_id: Vehicle ID of victim (-1 for all)
        t_star: Attack start time
        t_end: Attack end time
        data_type: Data type to attack ('local', 'fleet', 'both')
        
    Returns:
        AttackModule with scenarios added
    """
    scenarios = {
        1: make_scenario(attacker_id, victim_id, t_star, t_end, 
                        'scaling', -0.5, data_type, ['velocity'],
                        scenario_name="Bogus_Case1_VelScalingNeg",
                        description="Velocity scaled by -0.5"),
        
        2: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'scaling', 0.5, data_type, ['velocity'],
                        scenario_name="Bogus_Case2_VelScalingPos",
                        description="Velocity scaled by 0.5"),
        
        3: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'bias', -15, data_type, ['X'],
                        scenario_name="Bogus_Case3_PosBiasNeg",
                        description="X position bias -15m"),
        
        4: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'bias', 33, data_type, ['X'],
                        scenario_name="Bogus_Case4_PosBiasPos",
                        description="X position bias +33m"),
        
        5: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'linear', 0.1, data_type, ['velocity'],
                        scenario_name="Bogus_Case5_VelLinear",
                        description="Velocity linear drift 0.1 m/s²"),
        
        6: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'sinusoidal', {'amplitude': 5, 'frequency': 0.5}, 
                        data_type, ['X'],
                        scenario_name="Bogus_Case6_PosSinusoidal",
                        description="X position sinusoidal (5m @ 0.5Hz)"),
        
        7: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'faulty', 10, data_type, ['X'],
                        scenario_name="Bogus_Case7_PosFaultyHigh",
                        description="X position faulty (σ=10m)"),
        
        8: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'faulty', 2.5, data_type, ['velocity'],
                        scenario_name="Bogus_Case8_VelFaulty",
                        description="Velocity faulty (σ=2.5 m/s)"),
        
        9: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'faulty', 0.2, data_type, ['acceleration'],
                        scenario_name="Bogus_Case9_AccFaulty",
                        description="Acceleration faulty (σ=0.2 m/s²)"),
        
        10: make_scenario(attacker_id, victim_id, t_star, t_end,
                         'bias', 2, data_type, ['velocity'],
                         scenario_name="Bogus_Case10_VelBiasSmall",
                         description="Velocity bias +2 m/s"),
    }
    
    if case_number in scenarios:
        attack_module.add_scenario(scenarios[case_number])
        return attack_module
    else:
        raise ValueError(f"Invalid Bogus case number: {case_number}. Must be 1-10.")


def create_dos_scenarios(attack_module: AttackModule, case_number: int,
                        attacker_id: int = 1, victim_id: int = -1,
                        t_star: float = 10.0, t_end: float = 15.0,
                        data_type: str = "local") -> AttackModule:
    """
    Create Denial of Service (DoS) attack scenarios.
    
    Args:
        attack_module: AttackModule to add scenarios to
        case_number: Case number defining the attack variant
        attacker_id: Vehicle ID of the attacker
        victim_id: Vehicle ID of victim (-1 for all)
        t_star: Attack start time
        t_end: Attack end time
        data_type: Data type to attack
        
    Returns:
        AttackModule with scenarios added
    """
    scenarios = {
        1: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'zero', 0, data_type, ['velocity'],
                        attack_type="DoS",
                        scenario_name="DoS_Case1_VelZero",
                        description="Set velocity to zero"),
        
        2: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'zero', 0, data_type, ['X', 'Y'],
                        attack_type="DoS",
                        scenario_name="DoS_Case2_PosZero",
                        description="Set position to zero"),
        
        3: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'zero', 0, data_type, ['X', 'Y', 'velocity'],
                        attack_type="DoS",
                        scenario_name="DoS_Case3_AllZero",
                        description="Set all state to zero"),
    }
    
    if case_number in scenarios:
        attack_module.add_scenario(scenarios[case_number])
        return attack_module
    else:
        raise ValueError(f"Invalid DoS case number: {case_number}")


def create_position_attack(attack_module: AttackModule, case_number: int,
                          attacker_id: int = 1, victim_id: int = -1,
                          t_star: float = 10.0, t_end: float = 15.0,
                          data_type: str = "local") -> AttackModule:
    """
    Create position-specific (POS) attack scenarios.
    
    Args:
        attack_module: AttackModule to add scenarios to
        case_number: Case number defining the attack variant
        attacker_id: Vehicle ID of the attacker
        victim_id: Vehicle ID of victim (-1 for all)
        t_star: Attack start time
        t_end: Attack end time
        data_type: Data type to attack
        
    Returns:
        AttackModule with scenarios added
    """
    scenarios = {
        1: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'bias', 10, data_type, ['X', 'Y'],
                        attack_type="POS",
                        scenario_name="POS_Case1_Bias",
                        description="Position bias +10m on X and Y"),
        
        2: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'scaling', 1.5, data_type, ['X', 'Y'],
                        attack_type="POS",
                        scenario_name="POS_Case2_Scaling",
                        description="Position scaling by 1.5"),
        
        3: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'faulty', 5, data_type, ['X', 'Y'],
                        attack_type="POS",
                        scenario_name="POS_Case3_Noisy",
                        description="Position noise (σ=5m)"),
    }
    
    if case_number in scenarios:
        attack_module.add_scenario(scenarios[case_number])
        return attack_module
    else:
        raise ValueError(f"Invalid POS case number: {case_number}")


def create_velocity_attack(attack_module: AttackModule, case_number: int,
                          attacker_id: int = 1, victim_id: int = -1,
                          t_star: float = 10.0, t_end: float = 15.0,
                          data_type: str = "local") -> AttackModule:
    """
    Create velocity-specific (VEL) attack scenarios.
    
    Args:
        attack_module: AttackModule to add scenarios to
        case_number: Case number defining the attack variant
        attacker_id: Vehicle ID of the attacker
        victim_id: Vehicle ID of victim (-1 for all)
        t_star: Attack start time
        t_end: Attack end time
        data_type: Data type to attack
        
    Returns:
        AttackModule with scenarios added
    """
    scenarios = {
        1: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'constant', 3.0, data_type, ['velocity'],
                        attack_type="VEL",
                        scenario_name="VEL_Case1_Constant",
                        description="Constant velocity 3 m/s"),
        
        2: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'bias', 2.0, data_type, ['velocity'],
                        attack_type="VEL",
                        scenario_name="VEL_Case2_Bias",
                        description="Velocity bias +2 m/s"),
        
        3: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'scaling', 2.0, data_type, ['velocity'],
                        attack_type="VEL",
                        scenario_name="VEL_Case3_Exaggerate",
                        description="Velocity doubled"),
    }
    
    if case_number in scenarios:
        attack_module.add_scenario(scenarios[case_number])
        return attack_module
    else:
        raise ValueError(f"Invalid VEL case number: {case_number}")


def create_acceleration_attack(attack_module: AttackModule, case_number: int,
                               attacker_id: int = 1, victim_id: int = -1,
                               t_star: float = 10.0, t_end: float = 15.0,
                               data_type: str = "local") -> AttackModule:
    """
    Create acceleration-specific (ACC) attack scenarios.
    
    Args:
        attack_module: AttackModule to add scenarios to
        case_number: Case number defining the attack variant
        attacker_id: Vehicle ID of the attacker
        victim_id: Vehicle ID of victim (-1 for all)
        t_star: Attack start time
        t_end: Attack end time
        data_type: Data type to attack
        
    Returns:
        AttackModule with scenarios added
    """
    scenarios = {
        1: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'bias', 1.0, data_type, ['acceleration'],
                        attack_type="ACC",
                        scenario_name="ACC_Case1_Bias",
                        description="Acceleration bias +1 m/s²"),
        
        2: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'faulty', 0.5, data_type, ['acceleration'],
                        attack_type="ACC",
                        scenario_name="ACC_Case2_Noisy",
                        description="Acceleration noise (σ=0.5 m/s²)"),
        
        3: make_scenario(attacker_id, victim_id, t_star, t_end,
                        'constant', 0.0, data_type, ['acceleration'],
                        attack_type="ACC",
                        scenario_name="ACC_Case3_Zero",
                        description="Zero acceleration (constant velocity claim)"),
    }
    
    if case_number in scenarios:
        attack_module.add_scenario(scenarios[case_number])
        return attack_module
    else:
        raise ValueError(f"Invalid ACC case number: {case_number}")


def atk_scenarios(attack_module: AttackModule, attack_type: str, data_type: str,
                 case_number: int, t_star: float, t_end: float,
                 attacker_id: int, victim_id: int = -1) -> AttackModule:
    """
    Main function for creating attack scenarios based on attack type and case number.
    Similar to MATLAB's Atk_Scenarios function.
    
    Args:
        attack_module: AttackModule to configure
        attack_type: Type of attack ('Bogus', 'DoS', 'POS', 'VEL', 'ACC', 'Collusion')
        data_type: Data type to attack ('local', 'fleet', 'both')
        case_number: Case number for the specific attack variant
        t_star: Attack start time
        t_end: Attack end time
        attacker_id: Vehicle ID of the attacker
        victim_id: Vehicle ID of victim (-1 for all)
        
    Returns:
        AttackModule with configured scenario
    """
    attack_type = attack_type.upper()
    
    if attack_type == "BOGUS":
        return create_bogus_scenarios(attack_module, case_number, attacker_id, 
                                     victim_id, t_star, t_end, data_type)
    
    elif attack_type == "DOS":
        return create_dos_scenarios(attack_module, case_number, attacker_id,
                                   victim_id, t_star, t_end, data_type)
    
    elif attack_type == "POS":
        return create_position_attack(attack_module, case_number, attacker_id,
                                      victim_id, t_star, t_end, data_type)
    
    elif attack_type == "VEL":
        return create_velocity_attack(attack_module, case_number, attacker_id,
                                      victim_id, t_star, t_end, data_type)
    
    elif attack_type == "ACC":
        return create_acceleration_attack(attack_module, case_number, attacker_id,
                                         victim_id, t_star, t_end, data_type)
    
    elif attack_type == "NONE":
        # No attack
        return attack_module
    
    else:
        raise ValueError(f"Unknown attack type: {attack_type}")


def load_scenarios_from_config(attack_module: AttackModule, 
                               config_path: str = "attack_config.yaml",
                               enabled_only: bool = True) -> AttackModule:
    """
    Load attack scenarios from YAML configuration file.
    
    Args:
        attack_module: AttackModule to add scenarios to
        config_path: Path to YAML configuration file
        enabled_only: If True, only load enabled scenarios
        
    Returns:
        AttackModule with loaded scenarios
    """
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Warning: Config file not found: {config_path}")
        return attack_module
    except yaml.YAMLError as e:
        print(f"Error parsing YAML config: {e}")
        return attack_module
    
    # Check if attacks are globally enabled
    attack_settings = config.get('attack_settings', {})
    if not attack_settings.get('enabled', False) and enabled_only:
        print("Attacks globally disabled in config")
        return attack_module
    
    # Load scenarios
    scenarios_config = config.get('scenarios', [])
    for scenario_data in scenarios_config:
        # Skip disabled scenarios if enabled_only is True
        if enabled_only and not scenario_data.get('enabled', True):
            continue
        
        try:
            # Parse intensity (handle dict for sinusoidal)
            intensity = scenario_data['intensity']
            if isinstance(intensity, dict):
                # Already a dict, use as-is
                pass
            else:
                # Convert to appropriate type
                intensity = float(intensity)
            
            # Create scenario
            scenario = AttackScenario(
                t_start=float(scenario_data['t_start']),
                t_end=float(scenario_data['t_end']),
                attacker_id=int(scenario_data['attacker_id']),
                victim_ids=[int(v) for v in scenario_data['victim_ids']],
                attack_type=AttackType(scenario_data['attack_type']),
                modification_type=ModificationType(scenario_data['modification_type']),
                data_type=DataType(scenario_data['data_type']),
                intensity=intensity,
                target_fields=scenario_data['target_fields'],
                scenario_name=scenario_data.get('name', ''),
                description=scenario_data.get('description', '')
            )
            
            attack_module.add_scenario(scenario)
            
        except (KeyError, ValueError) as e:
            print(f"Error loading scenario '{scenario_data.get('name', 'unknown')}': {e}")
            continue
    
    return attack_module


# Example usage
if __name__ == "__main__":
    # Example 1: Create attack module and add scenarios programmatically
    attack_module = AttackModule(dt=0.01, vehicle_id=1)
    
    # Add Bogus case 2 (velocity scaling)
    atk_scenarios(attack_module, "Bogus", "local", 2, 10.0, 15.0, 1, -1)
    
    print(f"Attack module configured with {len(attack_module.scenarios)} scenario(s)")
    
    # Example 2: Load from config file
    attack_module2 = AttackModule(dt=0.01, vehicle_id=1)
    load_scenarios_from_config(attack_module2, "attack_config.yaml", enabled_only=False)
    
    print(f"Loaded {len(attack_module2.scenarios)} scenario(s) from config")
