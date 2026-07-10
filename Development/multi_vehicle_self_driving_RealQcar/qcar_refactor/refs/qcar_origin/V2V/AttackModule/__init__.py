"""
V2V Attack Module Package

Provides fault injection capabilities for Vehicle-to-Vehicle (V2V) communication
in the Fleet Framework. Designed for security research and testing.

Main Components:
    - AttackModule: Core attack engine for managing and applying attack scenarios
    - AttackScenarios: Factory functions for creating predefined attack scenarios
    - V2VAttackInjector: Middleware that wraps V2VManager for transparent attack injection

Usage:
    from V2V.AttackModule import AttackModule, V2VAttackInjector
    from V2V.AttackModule import AttackScenarios

    # Create attack module
    attack_module = AttackModule(vehicle_id=1, logger=my_logger)
    
    # Add predefined scenario
    AttackScenarios.atk_scenarios(
        attack_module, "Bogus", "local", 2, 10.0, 20.0, 1, -1
    )
    
    # Or wrap V2VManager with attack injector
    attack_injector = V2VAttackInjector(v2v_manager, attack_config_path="attack_config.yaml")

Author: Fleet Framework Security Research
Version: 2.0.0
Updated: January 2026
"""

from .AttackModule import (
    AttackModule,
    AttackScenario,
    AttackType,
    DataType,
    ModificationType,
    create_attack_module,
)

from .V2VAttackInjector import (
    V2VAttackInjector,
    create_v2v_with_attack_injection,
)

from ....V2V.AttackModule import AttackScenarios

__version__ = "2.0.0"
__author__ = "Fleet Framework Security Research"

__all__ = [
    # Core classes
    "AttackModule",
    "AttackScenario",
    "AttackType",
    "DataType", 
    "ModificationType",
    
    # V2V Integration
    "V2VAttackInjector",
    
    # Factory functions
    "create_attack_module",
    "create_v2v_with_attack_injection",
    
    # Scenario module
    "AttackScenarios",
]
