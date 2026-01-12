"""
Attack Module for simulating communication attacks in vehicle fleet systems.

This module intercepts and modifies vehicle state data before broadcasting,
simulating various attack scenarios including:
- Bogus messages (scaling, bias, linear drift, sinusoidal, faulty)
- Denial of Service (DoS)
- Position/Velocity/Acceleration specific attacks
- Collusion attacks

Based on MATLAB Attack_module implementation for autonomous vehicle security research.
"""

import numpy as np
import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum


class AttackType(Enum):
    """Types of attacks that can be simulated."""
    NONE = "None"
    BOGUS = "Bogus"
    DOS = "DoS"
    COLLUSION = "Collusion"
    POS = "POS"  # Position-specific attack
    VEL = "VEL"  # Velocity-specific attack
    ACC = "ACC"  # Acceleration-specific attack


class DataType(Enum):
    """Types of data that can be attacked."""
    LOCAL = "local"      # Attack local state broadcasts
    FLEET = "fleet"      # Attack fleet estimate broadcasts
    BOTH = "both"        # Attack both local and fleet
    NONE = "none"        # No data attacked


class ModificationType(Enum):
    """Types of modifications for bogus messages."""
    SCALING = "scaling"           # Multiply by factor
    BIAS = "bias"                 # Add constant offset
    LINEAR = "linear"             # Linear drift over time
    SINUSOIDAL = "sinusoidal"     # Oscillating pattern
    FAULTY = "faulty"             # Random noise injection
    ZERO = "zero"                 # Set to zero (DoS variant)
    CONSTANT = "constant"         # Set to constant value


@dataclass
class AttackScenario:
    """
    Defines a single attack scenario with timing, targets, and modification parameters.
    """
    # Timing
    t_start: float                      # Attack start time (seconds)
    t_end: float                        # Attack end time (seconds)
    
    # Participants
    attacker_id: int                    # Vehicle performing the attack
    victim_ids: List[int]               # Target vehicles (-1 means all vehicles)
    
    # Attack type
    attack_type: AttackType             # Type of attack
    modification_type: ModificationType # How to modify data
    data_type: DataType                 # What data to attack
    
    # Modification parameters
    intensity: Any                      # Attack intensity (float, dict for sinusoidal, etc.)
    target_fields: List[str]            # State fields to attack: ['X', 'Y', 'velocity', 'acceleration', 'theta']
    
    # Metadata
    scenario_name: str = ""             # Human-readable scenario name
    description: str = ""               # Description of the attack
    
    def is_active(self, current_time: float) -> bool:
        """Check if attack is currently active based on time."""
        return self.t_start <= current_time <= self.t_end
    
    def is_victim(self, vehicle_id: int) -> bool:
        """Check if a vehicle is a victim of this attack."""
        if -1 in self.victim_ids:  # -1 means all vehicles
            return True
        return vehicle_id in self.victim_ids


class AttackModule:
    """
    Main attack module for simulating communication attacks on vehicle data.
    
    This module intercepts state data before broadcasting and applies
    various attack modifications based on configured scenarios.
    """
    
    def __init__(self, dt: float = 0.01, vehicle_id: Optional[int] = None, logger=None):
        """
        Initialize the Attack Module.
        
        Args:
            dt: Simulation time step (seconds)
            vehicle_id: ID of the vehicle this module is attached to
            logger: Logger instance for attack events
        """
        self.dt = dt
        self.vehicle_id = vehicle_id
        self.logger = logger
        
        # Attack scenarios
        self.scenarios: List[AttackScenario] = []
        
        # Attack state tracking
        self.attack_active = False
        self.current_scenarios: List[AttackScenario] = []
        self.attack_start_time: Optional[float] = None
        self.attack_elapsed_time: float = 0.0
        
        # Statistics
        self.total_attacks_applied = 0
        self.attacks_by_type: Dict[str, int] = {}
        self.last_attack_log_time = 0.0
        
        # State field mapping
        self.state_field_mapping = {
            'X': 0,           # Position X
            'Y': 1,           # Position Y
            'theta': 2,       # Orientation
            'velocity': 3,    # Velocity
            'acceleration': 4 # Acceleration (if available)
        }
        
        if self.logger:
            self.logger.info(f"AttackModule initialized for vehicle {vehicle_id}, dt={dt}")
    
    def add_scenario(self, scenario: AttackScenario) -> None:
        """
        Add an attack scenario to the module.
        
        Args:
            scenario: AttackScenario to add
        """
        self.scenarios.append(scenario)
        if self.logger:
            self.logger.info(f"Added attack scenario: {scenario.scenario_name} "
                           f"(Type: {scenario.attack_type.value}, "
                           f"Time: {scenario.t_start}-{scenario.t_end}s, "
                           f"Fields: {scenario.target_fields})")
    
    def clear_scenarios(self) -> None:
        """Clear all attack scenarios."""
        self.scenarios.clear()
        self.current_scenarios.clear()
        self.attack_active = False
        if self.logger:
            self.logger.info("All attack scenarios cleared")
    
    def update(self, current_time: float) -> None:
        """
        Update attack module state based on current time.
        
        Args:
            current_time: Current simulation time (seconds)
        """
        # Check which scenarios are active
        active_scenarios = [s for s in self.scenarios if s.is_active(current_time)]
        
        # Update attack state
        was_active = self.attack_active
        self.attack_active = len(active_scenarios) > 0
        self.current_scenarios = active_scenarios
        
        # Track attack timing
        if self.attack_active:
            if not was_active:
                self.attack_start_time = current_time
                if self.logger:
                    self.logger.warning(f"ATTACK STARTED at t={current_time:.3f}s - "
                                      f"{len(active_scenarios)} scenario(s) active")
            self.attack_elapsed_time = current_time - self.attack_start_time
        else:
            if was_active:
                if self.logger:
                    self.logger.warning(f"ATTACK ENDED at t={current_time:.3f}s")
            self.attack_elapsed_time = 0.0
    
    def apply_attack_to_local_state(self, state: np.ndarray, current_time: float) -> np.ndarray:
        """
        Apply active attacks to local state before broadcasting.
        
        Args:
            state: Local state array [x, y, theta, velocity, ...]
            current_time: Current simulation time
            
        Returns:
            Modified state array (or original if no attack active)
        """
        if not self.attack_active:
            return state
        
        # Check if this vehicle is an attacker for any active scenario
        attacker_scenarios = [s for s in self.current_scenarios 
                            if s.attacker_id == self.vehicle_id and 
                               s.data_type in [DataType.LOCAL, DataType.BOTH]]
        
        if not attacker_scenarios:
            return state
        
        # Apply each active attack scenario
        modified_state = state.copy()
        for scenario in attacker_scenarios:
            modified_state = self._apply_scenario_modification(
                modified_state, scenario, current_time, is_fleet=False
            )
        
        return modified_state
    
    def apply_attack_to_fleet_estimates(self, fleet_estimates: Dict, current_time: float) -> Dict:
        """
        Apply active attacks to fleet estimates before broadcasting.
        
        Args:
            fleet_estimates: Fleet estimates dictionary
            current_time: Current simulation time
            
        Returns:
            Modified fleet estimates dictionary (or original if no attack active)
        """
        if not self.attack_active:
            return fleet_estimates
        
        # Check if this vehicle is an attacker for any active scenario
        attacker_scenarios = [s for s in self.current_scenarios 
                            if s.attacker_id == self.vehicle_id and 
                               s.data_type in [DataType.FLEET, DataType.BOTH]]
        
        if not attacker_scenarios:
            return fleet_estimates
        
        # Apply each active attack scenario to fleet estimates
        modified_estimates = {}
        for vehicle_id, estimate in fleet_estimates.items():
            modified_estimate = estimate.copy() if isinstance(estimate, dict) else estimate
            
            # Convert vehicle_id to int for victim check
            try:
                vid = int(str(vehicle_id).replace('V', ''))
            except (ValueError, AttributeError):
                vid = vehicle_id
            
            # Check if this vehicle is a victim
            victim_scenarios = [s for s in attacker_scenarios if s.is_victim(vid)]
            
            if victim_scenarios:
                for scenario in victim_scenarios:
                    modified_estimate = self._apply_scenario_to_fleet_estimate(
                        modified_estimate, scenario, current_time
                    )
            
            modified_estimates[vehicle_id] = modified_estimate
        
        return modified_estimates
    
    def _apply_scenario_modification(self, state: np.ndarray, scenario: AttackScenario, 
                                    current_time: float, is_fleet: bool = False) -> np.ndarray:
        """
        Apply a single scenario's modification to a state array.
        
        Args:
            state: State array to modify
            scenario: Attack scenario to apply
            current_time: Current simulation time
            is_fleet: Whether this is a fleet estimate (affects logging)
            
        Returns:
            Modified state array
        """
        modified_state = state.copy()
        attack_time = current_time - scenario.t_start
        
        # PERFORMANCE OPTIMIZATION: Only log if attack type matches the data being modified
        should_log = False
        if is_fleet and scenario.data_type in [DataType.FLEET, DataType.BOTH]:
            should_log = True
        elif not is_fleet and scenario.data_type in [DataType.LOCAL, DataType.BOTH]:
            should_log = True
        
        for field in scenario.target_fields:
            if field not in self.state_field_mapping:
                continue
            
            idx = self.state_field_mapping[field]
            if idx >= len(modified_state):
                continue
            
            original_value = modified_state[idx]
            
            # Apply modification based on type
            if scenario.modification_type == ModificationType.SCALING:
                modified_state[idx] = original_value * scenario.intensity
                
            elif scenario.modification_type == ModificationType.BIAS:
                modified_state[idx] = original_value + scenario.intensity
                
            elif scenario.modification_type == ModificationType.LINEAR:
                # Linear drift: original_value + intensity * time_elapsed
                modified_state[idx] = original_value + scenario.intensity * attack_time
                
            elif scenario.modification_type == ModificationType.SINUSOIDAL:
                # Sinusoidal: original_value + amplitude * sin(2*pi*frequency*time)
                if isinstance(scenario.intensity, dict):
                    amplitude = scenario.intensity.get('amplitude', 1.0)
                    frequency = scenario.intensity.get('frequency', 1.0)
                    phase = scenario.intensity.get('phase', 0.0)
                    offset = amplitude * np.sin(2 * np.pi * frequency * attack_time + phase)
                    modified_state[idx] = original_value + offset
                else:
                    # Fallback: use intensity as amplitude with default frequency
                    modified_state[idx] = original_value + scenario.intensity * np.sin(2 * np.pi * attack_time)
                
            elif scenario.modification_type == ModificationType.FAULTY:
                # Random noise: original_value + random_noise * intensity
                noise = np.random.normal(0, scenario.intensity)
                modified_state[idx] = original_value + noise
                
            elif scenario.modification_type == ModificationType.ZERO:
                modified_state[idx] = 0.0
                
            elif scenario.modification_type == ModificationType.CONSTANT:
                modified_state[idx] = scenario.intensity
            
            # Log modification (throttled to avoid spam) - ONLY if relevant to this data type
            if should_log and self.logger and (current_time - self.last_attack_log_time) > 1.0:
                data_type_str = "FLEET" if is_fleet else "LOCAL"
                self.logger.debug(f"ATTACK [{data_type_str}]: {scenario.modification_type.value} on {field}: "
                                  f"{original_value:.3f} -> {modified_state[idx]:.3f}")
        
        # Update statistics
        self.total_attacks_applied += 1
        attack_key = f"{scenario.attack_type.value}_{scenario.modification_type.value}"
        self.attacks_by_type[attack_key] = self.attacks_by_type.get(attack_key, 0) + 1
        
        if (current_time - self.last_attack_log_time) > 1.0:
            self.last_attack_log_time = current_time
        
        return modified_state
    
    def _apply_scenario_to_fleet_estimate(self, estimate: Dict, scenario: AttackScenario, 
                                         current_time: float) -> Dict:
        """
        Apply attack scenario to a fleet estimate dictionary.
        
        Args:
            estimate: Fleet estimate dictionary with 'pos', 'rot', 'vel' keys
            scenario: Attack scenario to apply
            current_time: Current simulation time
            
        Returns:
            Modified fleet estimate dictionary
        """
        if not isinstance(estimate, dict):
            return estimate
        
        modified_estimate = estimate.copy()
        attack_time = current_time - scenario.t_start
        
        # PERFORMANCE OPTIMIZATION: Only log if attacking fleet data
        should_log = scenario.data_type in [DataType.FLEET, DataType.BOTH]
        
        for field in scenario.target_fields:
            original_value = None
            modified_value = None
            
            # Map field to fleet estimate structure
            if field == 'X' and 'pos' in modified_estimate:
                original_value = modified_estimate['pos'][0]
                modified_value = self._apply_modification_value(
                    original_value, scenario, attack_time
                )
                modified_estimate['pos'] = [modified_value, modified_estimate['pos'][1], 
                                          modified_estimate['pos'][2] if len(modified_estimate['pos']) > 2 else 0]
                
                # Log fleet estimate modification - ONLY if targeting fleet data
                if should_log and self.logger and (current_time - self.last_attack_log_time) > 1.0:
                    self.logger.debug(f"FLEET_ATTACK: {scenario.modification_type.value} on {field}: "
                                      f"{original_value:.3f} -> {modified_value:.3f}")
                
            elif field == 'Y' and 'pos' in modified_estimate:
                original_value = modified_estimate['pos'][1]
                modified_value = self._apply_modification_value(
                    original_value, scenario, attack_time
                )
                modified_estimate['pos'] = [modified_estimate['pos'][0], modified_value,
                                          modified_estimate['pos'][2] if len(modified_estimate['pos']) > 2 else 0]
                
                # Log fleet estimate modification - ONLY if targeting fleet data
                if should_log and self.logger and (current_time - self.last_attack_log_time) > 1.0:
                    self.logger.debug(f"FLEET_ATTACK: {scenario.modification_type.value} on {field}: "
                                      f"{original_value:.3f} -> {modified_value:.3f}")
                
            elif field == 'theta' and 'rot' in modified_estimate:
                original_value = modified_estimate['rot'][2]  # Yaw
                modified_value = self._apply_modification_value(
                    original_value, scenario, attack_time
                )
                modified_estimate['rot'] = [modified_estimate['rot'][0], modified_estimate['rot'][1], modified_value]
                
                # Log fleet estimate modification - ONLY if targeting fleet data
                if should_log and self.logger and (current_time - self.last_attack_log_time) > 1.0:
                    self.logger.debug(f"FLEET_ATTACK: {scenario.modification_type.value} on {field}: "
                                      f"{original_value:.3f} -> {modified_value:.3f}")
                
            elif field == 'velocity' and 'vel' in modified_estimate:
                original_value = modified_estimate['vel']
                modified_value = self._apply_modification_value(
                    original_value, scenario, attack_time
                )
                modified_estimate['vel'] = modified_value
                
                # Log fleet estimate modification - ONLY if targeting fleet data
                if should_log and self.logger and (current_time - self.last_attack_log_time) > 1.0:
                    self.logger.debug(f"FLEET_ATTACK: {scenario.modification_type.value} on {field}: "
                                      f"{original_value:.3f} -> {modified_value:.3f}")
        
        # Update statistics (same as for local state attacks)
        self.total_attacks_applied += 1
        attack_key = f"FLEET_{scenario.attack_type.value}_{scenario.modification_type.value}"
        self.attacks_by_type[attack_key] = self.attacks_by_type.get(attack_key, 0) + 1
        
        return modified_estimate
    
    def _apply_modification_value(self, original_value: float, scenario: AttackScenario, 
                                 attack_time: float) -> float:
        """
        Apply modification to a single value.
        
        Args:
            original_value: Original value to modify
            scenario: Attack scenario with modification parameters
            attack_time: Time elapsed since attack start
            
        Returns:
            Modified value
        """
        if scenario.modification_type == ModificationType.SCALING:
            return original_value * scenario.intensity
            
        elif scenario.modification_type == ModificationType.BIAS:
            return original_value + scenario.intensity
            
        elif scenario.modification_type == ModificationType.LINEAR:
            return original_value + scenario.intensity * attack_time
            
        elif scenario.modification_type == ModificationType.SINUSOIDAL:
            if isinstance(scenario.intensity, dict):
                amplitude = scenario.intensity.get('amplitude', 1.0)
                frequency = scenario.intensity.get('frequency', 1.0)
                phase = scenario.intensity.get('phase', 0.0)
                offset = amplitude * np.sin(2 * np.pi * frequency * attack_time + phase)
                return original_value + offset
            else:
                return original_value + scenario.intensity * np.sin(2 * np.pi * attack_time)
            
        elif scenario.modification_type == ModificationType.FAULTY:
            noise = np.random.normal(0, scenario.intensity)
            return original_value + noise
            
        elif scenario.modification_type == ModificationType.ZERO:
            return 0.0
            
        elif scenario.modification_type == ModificationType.CONSTANT:
            return scenario.intensity
        
        return original_value
    
    def get_attack_status(self) -> Dict:
        """
        Get current attack status and statistics.
        
        Returns:
            Dictionary with attack status information
        """
        return {
            'attack_active': self.attack_active,
            'active_scenarios': len(self.current_scenarios),
            'total_scenarios': len(self.scenarios),
            'elapsed_time': self.attack_elapsed_time,
            'total_attacks_applied': self.total_attacks_applied,
            'attacks_by_type': self.attacks_by_type.copy(),
            'current_scenario_names': [s.scenario_name for s in self.current_scenarios]
        }
    
    def log_status(self) -> None:
        """Log current attack status."""
        if not self.logger:
            return
        
        status = self.get_attack_status()
        self.logger.info(f"Attack Status: Active={status['attack_active']}, "
                        f"Scenarios={status['active_scenarios']}/{status['total_scenarios']}, "
                        f"TotalApplied={status['total_attacks_applied']}")
        
        if status['attack_active']:
            for scenario in self.current_scenarios:
                self.logger.info(f"  Active: {scenario.scenario_name} - "
                               f"Type={scenario.attack_type.value}, "
                               f"Mod={scenario.modification_type.value}, "
                               f"Fields={scenario.target_fields}")
