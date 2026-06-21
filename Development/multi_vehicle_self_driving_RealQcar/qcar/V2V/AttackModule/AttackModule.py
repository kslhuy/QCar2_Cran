"""
Attack Module for V2V Communication Fault Injection

This module provides attack simulation capabilities for vehicle-to-vehicle (V2V)
communication in fleet frameworks. It can inject faults into V2V data including:
- Local state broadcasts (position, velocity, heading)
- Fleet state estimates
- Heartbeat messages

Supports multiple attack types:
- Bogus messages (scaling, bias, linear drift, sinusoidal, faulty/noise)
- Denial of Service (DoS)
- Position/Velocity/Acceleration specific attacks
- Collusion attacks

Designed for integration with V2VManager and V2VCommunication systems.

Author: Fleet Framework Security Research
Version: 2.0.0
Updated: January 2026
"""

import numpy as np
import time
import logging
from typing import Dict, List, Optional, Tuple, Any, Union
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path


class AttackType(Enum):
    """Types of attacks that can be simulated."""
    NONE = "None"
    BOGUS = "Bogus"           # Falsified data injection
    DOS = "DoS"               # Denial of Service
    MIX_TEST = "Mix_test"     # MATLAB Mix_test scenario family
    COLLUSION = "Collusion"   # Coordinated multi-attacker
    POS = "POS"               # Position-specific attack
    VEL = "VEL"               # Velocity-specific attack
    ACC = "ACC"               # Acceleration-specific attack
    REPLAY = "Replay"         # Replay old messages
    DELAY = "Delay"           # Message delay injection


class DataType(Enum):
    """Types of V2V data that can be attacked."""
    LOCAL = "local"           # Attack local state broadcasts
    FLEET = "fleet"           # Attack fleet estimate broadcasts
    BOTH = "both"             # Attack both local and fleet
    HEARTBEAT = "heartbeat"   # Attack heartbeat messages
    NONE = "none"             # No data attacked


class ModificationType(Enum):
    """Types of modifications for fault injection."""
    SCALING = "scaling"           # Multiply by factor: value * factor
    BIAS = "bias"                 # Add constant offset: value + offset
    LINEAR = "linear"             # Linear drift over time: value + rate * t
    SINUSOIDAL = "sinusoidal"     # Oscillating pattern: value + A*sin(2πft + φ)
    FAULTY = "faulty"             # Random noise injection: value + N(0, σ)
    ZERO = "zero"                 # Set to zero (DoS variant)
    CONSTANT = "constant"         # Set to constant value
    RANDOM = "random"             # Random value in range
    STEP = "step"                 # Step change at specific time
    DROP = "drop"                 # Drop the outgoing V2V packet probabilistically


@dataclass
class AttackScenario:
    """
    Defines a single attack scenario with timing, targets, and modification parameters.
    
    Attributes:
        t_start: Attack start time (seconds from simulation start)
        t_end: Attack end time (seconds from simulation start)
        attacker_id: Vehicle ID performing the attack
        victim_ids: Target vehicle IDs (-1 means all vehicles)
        attack_type: Type of attack (Bogus, DoS, etc.)
        modification_type: How to modify data (scaling, bias, etc.)
        data_type: What V2V data to attack (local, fleet, both)
        intensity: Attack intensity (float or dict for complex patterns)
        target_fields: State fields to attack (x, y, theta, velocity, etc.)
        scenario_name: Human-readable scenario name
        description: Description of the attack
    """
    # Timing
    t_start: float                      
    t_end: float                        
    
    # Participants
    attacker_id: int                    
    victim_ids: List[int]               
    
    # Attack configuration
    attack_type: AttackType             
    modification_type: ModificationType 
    data_type: DataType                 
    
    # Modification parameters
    intensity: Any                      
    target_fields: List[str]            
    
    # Metadata
    scenario_name: str = ""             
    description: str = ""               
    
    # Internal state
    _attack_count: int = field(default=0, repr=False)
    
    def is_active(self, current_time: float) -> bool:
        """Check if attack is currently active based on time."""
        return self.t_start <= current_time <= self.t_end
    
    def is_victim(self, vehicle_id: int) -> bool:
        """Check if a vehicle is a victim of this attack."""
        if -1 in self.victim_ids:
            return True
        return vehicle_id in self.victim_ids
    
    def should_attack_local(self) -> bool:
        """Check if this scenario attacks local state broadcasts."""
        return self.data_type in [DataType.LOCAL, DataType.BOTH]
    
    def should_attack_fleet(self) -> bool:
        """Check if this scenario attacks fleet state broadcasts."""
        return self.data_type in [DataType.FLEET, DataType.BOTH]
    
    def should_attack_heartbeat(self) -> bool:
        """Check if this scenario attacks heartbeat messages."""
        return self.data_type == DataType.HEARTBEAT

    def is_drop_attack(self) -> bool:
        """Check if this scenario represents a packet drop attack."""
        return self.modification_type == ModificationType.DROP
    
    def get_attack_progress(self, current_time: float) -> float:
        """Get attack progress as fraction (0.0 to 1.0)."""
        if current_time < self.t_start:
            return 0.0
        if current_time > self.t_end:
            return 1.0
        duration = self.t_end - self.t_start
        if duration <= 0:
            return 1.0
        return (current_time - self.t_start) / duration


class AttackModule:
    """
    Core attack module for V2V communication fault injection.
    
    This module integrates with V2VManager to intercept and modify
    vehicle state data before broadcasting, simulating various attack scenarios.
    
    Usage:
        attack_module = AttackModule(vehicle_id=1, logger=my_logger)
        attack_module.add_scenario(scenario)
        
        # In V2V broadcast loop:
        attack_module.update(elapsed_time)
        modified_data = attack_module.apply_attack_to_local_state(original_data)
    """
    
    # V2V message field mapping for local state
    LOCAL_STATE_FIELDS = {
        'x': 'x',
        'X': 'x',
        'y': 'y',
        'Y': 'y',
        'theta': 'theta',
        'heading': 'theta',
        'velocity': 'velocity',
        'vel': 'velocity',
        'acceleration': 'acceleration',
        'acc': 'acceleration',
    }
    
    # V2V message field mapping for fleet state
    FLEET_STATE_FIELDS = {
        'x': 'x',
        'X': 'x',
        'y': 'y',
        'Y': 'y',
        'theta': 'theta',
        'heading': 'theta',
        'velocity': 'velocity',
        'vel': 'velocity',
        'confidence': 'confidence',
    }

    SNAPSHOT_FIELDS = ('x', 'y', 'theta', 'velocity', 'acceleration', 'confidence')
    
    def __init__(self, vehicle_id: int, logger: Optional[logging.Logger] = None,
                 dt: float = 0.01, log_interval: float = 1.0):
        """
        Initialize the Attack Module.
        
        Args:
            vehicle_id: ID of the vehicle this module is attached to
            logger: Logger instance for attack events
            dt: Simulation time step (seconds)
            log_interval: Minimum interval between detailed attack logs (seconds)
        """
        self.vehicle_id = vehicle_id
        self.logger = logger
        self.dt = dt
        self.log_interval = log_interval
        
        # Attack scenarios
        self.scenarios: List[AttackScenario] = []
        
        # Attack state tracking
        self.attack_active = False
        self.current_scenarios: List[AttackScenario] = []
        self.attack_start_time: Optional[float] = None
        self.attack_elapsed_time: float = 0.0
        self.current_time: float = 0.0
        self.last_attack_snapshot: Dict[str, Any] = {'clock_s': 0.0, 'by_vehicle': {}}
        
        # Statistics
        self.stats = {
            'total_attacks_applied': 0,
            'local_attacks': 0,
            'fleet_attacks': 0,
            'attacks_by_type': {},
            'attacks_by_scenario': {},
        }
        
        # Logging control
        self._last_log_time = 0.0
        self._log_counter = 0
        
        if self.logger:
            self.logger.info(f"AttackModule initialized for vehicle {vehicle_id}")
    
    def add_scenario(self, scenario: AttackScenario) -> None:
        """Add an attack scenario to the module."""
        self.scenarios.append(scenario)
        self.stats['attacks_by_scenario'][scenario.scenario_name] = 0
        
        if self.logger:
            self.logger.info(
                f"Added attack scenario: {scenario.scenario_name} "
                f"(Type: {scenario.attack_type.value}, "
                f"Mod: {scenario.modification_type.value}, "
                f"Time: {scenario.t_start:.1f}-{scenario.t_end:.1f}s, "
                f"Data: {scenario.data_type.value}, "
                f"Fields: {scenario.target_fields})"
            )
    
    def add_scenarios(self, scenarios: List[AttackScenario]) -> None:
        """Add multiple attack scenarios."""
        for scenario in scenarios:
            self.add_scenario(scenario)
    
    def clear_scenarios(self) -> None:
        """Clear all attack scenarios."""
        self.scenarios.clear()
        self.current_scenarios.clear()
        self.attack_active = False
        self._clear_attack_snapshot()
        if self.logger:
            self.logger.info("All attack scenarios cleared")
    
    def update(self, current_time: float) -> None:
        """
        Update attack module state based on current time.
        
        Args:
            current_time: Current simulation time (seconds)
        """
        self.current_time = current_time
        
        # Find active scenarios
        active_scenarios = [s for s in self.scenarios if s.is_active(current_time)]
        
        # Track attack state transitions
        was_active = self.attack_active
        self.attack_active = len(active_scenarios) > 0
        self.current_scenarios = active_scenarios
        
        # Handle attack start
        if self.attack_active and not was_active:
            self.attack_start_time = current_time
            if self.logger:
                scenario_names = [s.scenario_name for s in active_scenarios]
                self.logger.warning(
                    f"🔴 ATTACK STARTED at t={current_time:.3f}s - "
                    f"{len(active_scenarios)} scenario(s) active: {scenario_names}"
                )
        
        # Handle attack end
        if was_active and not self.attack_active:
            if self.logger:
                self.logger.warning(f"🟢 ATTACK ENDED at t={current_time:.3f}s")
            self.attack_elapsed_time = 0.0
            self._clear_attack_snapshot()
        elif self.attack_active:
            self.attack_elapsed_time = current_time - self.attack_start_time
            self._clear_attack_snapshot()
    
    def should_attack_local_data(self) -> bool:
        """Check if any active scenario should attack local state data."""
        if not self.attack_active:
            return False
        
        return any(
            s.attacker_id == self.vehicle_id
            and s.should_attack_local()
            and not s.is_drop_attack()
            for s in self.current_scenarios
        )
    
    def should_attack_fleet_data(self) -> bool:
        """Check if any active scenario should attack fleet state data."""
        if not self.attack_active:
            return False
        
        return any(
            s.attacker_id == self.vehicle_id
            and s.should_attack_fleet()
            and not s.is_drop_attack()
            for s in self.current_scenarios
        )
    
    def should_drop_local_message(self) -> bool:
        """Check whether the current local-state broadcast should be dropped."""
        if not self.attack_active:
            return False

        drop_scenarios = [
            s for s in self.current_scenarios
            if (
                s.attacker_id == self.vehicle_id
                and s.should_attack_local()
                and s.is_drop_attack()
            )
        ]
        return self._should_drop_message(drop_scenarios, is_fleet=False)

    def should_drop_fleet_message(self) -> bool:
        """Check whether the current fleet-state broadcast should be dropped."""
        if not self.attack_active:
            return False

        drop_scenarios = [
            s for s in self.current_scenarios
            if (
                s.attacker_id == self.vehicle_id
                and s.should_attack_fleet()
                and s.is_drop_attack()
            )
        ]
        return self._should_drop_message(drop_scenarios, is_fleet=True)

    @staticmethod
    def _drop_probability(intensity: Any) -> float:
        """Normalize a drop scenario intensity into a probability in [0, 1]."""
        if isinstance(intensity, dict):
            raw_probability = intensity.get(
                'probability',
                intensity.get('drop_probability', intensity.get('p', 1.0)),
            )
        else:
            raw_probability = intensity

        try:
            probability = float(raw_probability)
        except (TypeError, ValueError):
            probability = 1.0
        return float(np.clip(probability, 0.0, 1.0))

    def _should_drop_message(
        self, scenarios: List[AttackScenario], is_fleet: bool = False
    ) -> bool:
        """Apply probabilistic drop scenarios and record actual dropped packets."""
        for scenario in scenarios:
            if np.random.random() >= self._drop_probability(scenario.intensity):
                continue

            self._record_drop_attack(scenario, is_fleet=is_fleet)
            return True

        return False

    def _record_drop_attack(
        self, scenario: AttackScenario, is_fleet: bool = False
    ) -> None:
        """Update statistics/logs for a packet dropped by an active scenario."""
        self.stats['total_attacks_applied'] += 1
        if is_fleet:
            self.stats['fleet_attacks'] += 1
            channel = "FLEET"
        else:
            self.stats['local_attacks'] += 1
            channel = "LOCAL"

        attack_key = f"{scenario.attack_type.value}_{scenario.modification_type.value}"
        self.stats['attacks_by_type'][attack_key] = \
            self.stats['attacks_by_type'].get(attack_key, 0) + 1
        self.stats['attacks_by_scenario'][scenario.scenario_name] = \
            self.stats['attacks_by_scenario'].get(scenario.scenario_name, 0) + 1
        scenario._attack_count += 1

        self.last_attack_snapshot['clock_s'] = float(self.current_time)

        if not self.logger:
            return

        current_time = self.current_time
        if current_time - self._last_log_time < self.log_interval:
            return

        self._last_log_time = current_time
        self.logger.warning(
            f"DROP {channel} STATE ATTACK - Vehicle {self.vehicle_id} "
            f"at t={current_time:.2f}s via {scenario.scenario_name}"
        )

    def apply_attack_to_local_state(self, local_state: Dict) -> Dict:
        """
        Apply active attacks to local state before V2V broadcasting.
        
        This method intercepts local state data from VehicleObserver.get_local_state_for_broadcast()
        and applies attack modifications before it's sent to peers.
        
        Args:
            local_state: Local state dictionary from VehicleObserver
                Expected format: {
                    'vehicle_id': int,
                    'x': float, 'y': float, 'theta': float, 'velocity': float,
                    'acceleration': float (optional),
                    'source': 'local_sensors'
                }
                
        Returns:
            Modified local state dictionary (or original if no attack active)
        """
        if not self.attack_active:
            return local_state
        
        # Find scenarios where this vehicle is the attacker
        attacker_scenarios = [
            s for s in self.current_scenarios
            if s.attacker_id == self.vehicle_id and s.should_attack_local()
        ]
        
        if not attacker_scenarios:
            return local_state
        
        # Create modified copy
        modified_state = local_state.copy()
        
        # Apply each active scenario
        for scenario in attacker_scenarios:
            modified_state = self._apply_modification_to_dict(
                modified_state, scenario, self.LOCAL_STATE_FIELDS, is_fleet=False
            )

        self._record_local_attack_snapshot(local_state, modified_state, attacker_scenarios)

        # Log attack (throttled)
        self._log_local_attack(local_state, modified_state, attacker_scenarios)
        
        return modified_state
    
    def apply_attack_to_fleet_state(self, fleet_state: Dict) -> Dict:
        """
        Apply active attacks to fleet state before V2V broadcasting.
        
        This method intercepts fleet state data from VehicleObserver.get_fleet_state_for_broadcast()
        and applies attack modifications before it's sent to peers.
        
        Args:
            fleet_state: Fleet state dictionary from VehicleObserver
                Expected format: {
                    'sender_id': int,
                    'fleet_states': {
                        vehicle_id: {'x': float, 'y': float, 'theta': float, 
                                    'velocity': float, 'confidence': float},
                        ...
                    },
                    'source': 'fleet_consensus'
                }
                
        Returns:
            Modified fleet state dictionary (or original if no attack active)
        """
        if not self.attack_active:
            return fleet_state
        
        # Find scenarios where this vehicle is the attacker
        attacker_scenarios = [
            s for s in self.current_scenarios
            if s.attacker_id == self.vehicle_id and s.should_attack_fleet()
        ]
        
        if not attacker_scenarios:
            return fleet_state
        
        # Create modified copy
        modified_fleet_state = fleet_state.copy()
        
        if 'fleet_states' in modified_fleet_state:
            modified_fleet_states = {}
            
            for vehicle_id, vehicle_state in fleet_state['fleet_states'].items():
                # Parse vehicle ID
                try:
                    vid = int(str(vehicle_id).replace('V', '').replace('vehicle_', ''))
                except (ValueError, AttributeError):
                    vid = vehicle_id
                
                modified_vehicle_state = vehicle_state.copy()
                
                # Apply attacks from scenarios targeting this victim
                for scenario in attacker_scenarios:
                    if scenario.is_victim(vid):
                        modified_vehicle_state = self._apply_modification_to_dict(
                            modified_vehicle_state, scenario, 
                            self.FLEET_STATE_FIELDS, is_fleet=True
                        )

                self._record_fleet_attack_snapshot(
                    vid=vid,
                    original_state=vehicle_state,
                    modified_state=modified_vehicle_state,
                    scenarios=[
                        scenario for scenario in attacker_scenarios if scenario.is_victim(vid)
                    ],
                )
                
                modified_fleet_states[vehicle_id] = modified_vehicle_state
            
            modified_fleet_state['fleet_states'] = modified_fleet_states
        
        # Log attack (throttled)
        self._log_fleet_attack(fleet_state, modified_fleet_state, attacker_scenarios)
        
        return modified_fleet_state
    
    def _apply_modification_to_dict(self, data: Dict, scenario: AttackScenario,
                                   field_mapping: Dict, is_fleet: bool = False) -> Dict:
        """
        Apply scenario modification to a dictionary.
        
        Args:
            data: Dictionary to modify
            scenario: Attack scenario to apply
            field_mapping: Mapping of attack fields to data keys
            is_fleet: Whether this is fleet data
            
        Returns:
            Modified dictionary
        """
        modified = data.copy()
        attack_time = self.current_time - scenario.t_start
        
        for target_field in scenario.target_fields:
            # Normalize field name
            normalized_field = field_mapping.get(target_field, target_field.lower())
            
            if normalized_field not in modified:
                continue
            
            original_value = modified[normalized_field]
            
            # Skip non-numeric values
            if not isinstance(original_value, (int, float)):
                continue
            
            # Apply modification
            modified_value = self._apply_modification(
                original_value, scenario, attack_time
            )
            
            modified[normalized_field] = modified_value
        
        # Update statistics
        self.stats['total_attacks_applied'] += 1
        if is_fleet:
            self.stats['fleet_attacks'] += 1
        else:
            self.stats['local_attacks'] += 1
        
        attack_key = f"{scenario.attack_type.value}_{scenario.modification_type.value}"
        self.stats['attacks_by_type'][attack_key] = \
            self.stats['attacks_by_type'].get(attack_key, 0) + 1
        self.stats['attacks_by_scenario'][scenario.scenario_name] = \
            self.stats['attacks_by_scenario'].get(scenario.scenario_name, 0) + 1
        
        scenario._attack_count += 1
        
        return modified

    def _clear_attack_snapshot(self) -> None:
        """Clear the latest per-vehicle attack value snapshot."""
        self.last_attack_snapshot = {
            'clock_s': float(self.current_time),
            'by_vehicle': {},
        }

    @staticmethod
    def _unique_snapshot_values(scenarios: List[AttackScenario], attr: str) -> List[str]:
        values: List[str] = []
        for scenario in scenarios:
            raw_value = getattr(scenario, attr, "")
            value = getattr(raw_value, "value", raw_value)
            value = str(value or "")
            if value and value not in values:
                values.append(value)
        return values

    def _build_value_snapshot(
        self,
        original_state: Dict,
        modified_state: Dict,
        field_mapping: Dict,
        scenarios: List[AttackScenario],
    ) -> Dict[str, Dict[str, float]]:
        """Build original/modified/delta values for all targeted numeric fields."""
        value_snapshot: Dict[str, Dict[str, float]] = {}
        targeted_fields = {
            field_mapping.get(target_field, str(target_field).lower())
            for scenario in scenarios
            for target_field in scenario.target_fields
        }

        for normalized_field in targeted_fields:
            if normalized_field not in self.SNAPSHOT_FIELDS:
                continue
            original_value = original_state.get(normalized_field)
            modified_value = modified_state.get(normalized_field)
            if not isinstance(original_value, (int, float)):
                continue
            if not isinstance(modified_value, (int, float)):
                continue
            value_snapshot[normalized_field] = {
                'original': float(original_value),
                'modified': float(modified_value),
                'delta': float(modified_value - original_value),
            }
        return value_snapshot

    def _merge_attack_snapshot_entry(
        self,
        vehicle_id: int,
        scenarios: List[AttackScenario],
        value_snapshot: Dict[str, Dict[str, float]],
    ) -> None:
        """Merge the latest attack values for one attacked vehicle."""
        if not scenarios or not value_snapshot:
            return

        by_vehicle = self.last_attack_snapshot.setdefault('by_vehicle', {})
        existing = by_vehicle.get(vehicle_id, {})
        existing_values = existing.get('values', {})
        existing_values.update(value_snapshot)

        fields = sorted(set(existing.get('fields', [])) | set(value_snapshot.keys()))
        existing_types = list(existing.get('types', []))
        existing_names = list(existing.get('names', []))
        existing_modifications = list(existing.get('modifications', []))
        existing_data_types = list(existing.get('data_types', []))

        for value in self._unique_snapshot_values(scenarios, 'attack_type'):
            if value not in existing_types:
                existing_types.append(value)
        for value in self._unique_snapshot_values(scenarios, 'scenario_name'):
            if value not in existing_names:
                existing_names.append(value)
        for value in self._unique_snapshot_values(scenarios, 'modification_type'):
            if value not in existing_modifications:
                existing_modifications.append(value)
        for value in self._unique_snapshot_values(scenarios, 'data_type'):
            if value not in existing_data_types:
                existing_data_types.append(value)

        by_vehicle[vehicle_id] = {
            'active': True,
            'attacker_id': int(scenarios[0].attacker_id),
            'types': existing_types,
            'names': existing_names,
            'modifications': existing_modifications,
            'data_types': existing_data_types,
            'fields': fields,
            'values': existing_values,
        }

    def _record_local_attack_snapshot(
        self,
        original_state: Dict,
        modified_state: Dict,
        scenarios: List[AttackScenario],
    ) -> None:
        """Record the last injected local-state values for logging/plotting."""
        if not scenarios:
            return
        self.last_attack_snapshot['clock_s'] = float(self.current_time)
        value_snapshot = self._build_value_snapshot(
            original_state,
            modified_state,
            self.LOCAL_STATE_FIELDS,
            scenarios,
        )
        self._merge_attack_snapshot_entry(self.vehicle_id, scenarios, value_snapshot)

    def _record_fleet_attack_snapshot(
        self,
        vid: int,
        original_state: Dict,
        modified_state: Dict,
        scenarios: List[AttackScenario],
    ) -> None:
        """Record the last injected fleet-state values for one targeted vehicle."""
        if not scenarios:
            return
        self.last_attack_snapshot['clock_s'] = float(self.current_time)
        value_snapshot = self._build_value_snapshot(
            original_state,
            modified_state,
            self.FLEET_STATE_FIELDS,
            scenarios,
        )
        self._merge_attack_snapshot_entry(int(vid), scenarios, value_snapshot)
    
    def _apply_modification(self, original_value: float, scenario: AttackScenario,
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
        intensity = scenario.intensity
        
        if scenario.modification_type == ModificationType.SCALING:
            return original_value * float(intensity)
        
        elif scenario.modification_type == ModificationType.BIAS:
            return original_value + float(intensity)
        
        elif scenario.modification_type == ModificationType.LINEAR:
            # Linear drift: value + rate * time
            return original_value + float(intensity) * attack_time
        
        elif scenario.modification_type == ModificationType.SINUSOIDAL:
            # Sinusoidal: value + amplitude * sin(2πft + φ)
            if isinstance(intensity, dict):
                amplitude = intensity.get('amplitude', 1.0)
                frequency = intensity.get('frequency', 1.0)
                phase = intensity.get('phase', 0.0)
            else:
                amplitude = float(intensity)
                frequency = 1.0
                phase = 0.0
            
            offset = amplitude * np.sin(2 * np.pi * frequency * attack_time + phase)
            return original_value + offset
        
        elif scenario.modification_type == ModificationType.FAULTY:
            # Random noise: value + N(0, sigma), optionally intermittent.
            if isinstance(intensity, dict):
                probability = self._drop_probability(intensity)
                if np.random.random() >= probability:
                    return original_value
                sigma = intensity.get(
                    'intensity', intensity.get('sigma', intensity.get('std', 1.0))
                )
            else:
                sigma = intensity
            noise = np.random.normal(0, float(sigma))
            return original_value + noise
        
        elif scenario.modification_type == ModificationType.ZERO:
            return 0.0
        
        elif scenario.modification_type == ModificationType.CONSTANT:
            return float(intensity)
        
        elif scenario.modification_type == ModificationType.RANDOM:
            # Random value in range
            if isinstance(intensity, dict):
                min_val = intensity.get('min', 0)
                max_val = intensity.get('max', 1)
            else:
                min_val = 0
                max_val = float(intensity)
            return np.random.uniform(min_val, max_val)
        
        elif scenario.modification_type == ModificationType.STEP:
            # Step change: apply intensity after certain fraction
            if isinstance(intensity, dict):
                step_value = intensity.get('value', 0)
                step_at = intensity.get('at', 0.5)  # Fraction of attack duration
            else:
                step_value = float(intensity)
                step_at = 0.0
            
            progress = scenario.get_attack_progress(self.current_time)
            if progress >= step_at:
                return original_value + step_value
            return original_value
        
        return original_value
    
    def _log_local_attack(self, original: Dict, modified: Dict, 
                         scenarios: List[AttackScenario]) -> None:
        """Log local state attack (throttled)."""
        if not self.logger:
            return
        
        current_time = self.current_time
        if current_time - self._last_log_time < self.log_interval:
            return
        
        self._last_log_time = current_time
        self._log_counter += 1
        
        self.logger.warning("=" * 70)
        self.logger.warning(
            f"🔴 LOCAL STATE ATTACK - Vehicle {self.vehicle_id} at t={current_time:.2f}s"
        )
        self.logger.warning("=" * 70)
        
        for field in ['x', 'y', 'theta', 'velocity']:
            if field in original and field in modified:
                orig_val = original[field]
                mod_val = modified[field]
                delta = mod_val - orig_val
                
                field_label = {
                    'x': '📍 Position X',
                    'y': '📍 Position Y',
                    'theta': '🧭 Heading',
                    'velocity': '🏎️  Velocity'
                }.get(field, field)
                
                self.logger.warning(
                    f"  {field_label}: {orig_val:.4f} → {mod_val:.4f} (Δ {delta:+.4f})"
                )
        
        for scenario in scenarios:
            self.logger.warning(
                f"  Scenario: {scenario.scenario_name} [{scenario.modification_type.value}]"
            )
        
        self.logger.warning("=" * 70)
    
    def _log_fleet_attack(self, original: Dict, modified: Dict,
                         scenarios: List[AttackScenario]) -> None:
        """Log fleet state attack (throttled)."""
        if not self.logger:
            return
        
        current_time = self.current_time
        if current_time - self._last_log_time < self.log_interval:
            return
        
        self._last_log_time = current_time
        
        self.logger.warning("=" * 70)
        self.logger.warning(
            f"🔴 FLEET STATE ATTACK - Vehicle {self.vehicle_id} at t={current_time:.2f}s"
        )
        self.logger.warning("=" * 70)
        
        if 'fleet_states' in original and 'fleet_states' in modified:
            for vid in original['fleet_states']:
                orig_state = original['fleet_states'][vid]
                mod_state = modified['fleet_states'].get(vid, {})
                
                changes = []
                for field in ['x', 'y', 'theta', 'velocity']:
                    if field in orig_state and field in mod_state:
                        if abs(mod_state[field] - orig_state[field]) > 1e-6:
                            changes.append(
                                f"{field}: {orig_state[field]:.3f}→{mod_state[field]:.3f}"
                            )
                
                if changes:
                    self.logger.warning(f"  Vehicle {vid}: {', '.join(changes)}")
        
        self.logger.warning("=" * 70)
    
    def get_attack_status(self) -> Dict:
        """Get current attack status and statistics."""
        return {
            'attack_active': self.attack_active,
            'current_time': self.current_time,
            'active_scenarios': len(self.current_scenarios),
            'total_scenarios': len(self.scenarios),
            'elapsed_time': self.attack_elapsed_time,
            'current_scenario_names': [s.scenario_name for s in self.current_scenarios],
            'active_scenario_details': self.get_active_scenario_details(),
            'all_scenario_details': self.get_scenario_details(),
            'attack_value_snapshot': self.last_attack_snapshot.copy(),
            'statistics': self.stats.copy(),
        }

    def _scenario_to_detail(self, scenario: AttackScenario) -> Dict:
        """Return plot/log friendly metadata for a scenario."""
        return {
            'name': scenario.scenario_name,
            'type': scenario.attack_type.value,
            'modification': scenario.modification_type.value,
            'data_type': scenario.data_type.value,
            'target_fields': list(scenario.target_fields),
            't_start': float(scenario.t_start),
            't_end': float(scenario.t_end),
            'attacker_id': int(scenario.attacker_id),
            'victim_ids': list(scenario.victim_ids),
            'active': bool(scenario in self.current_scenarios),
            'progress': scenario.get_attack_progress(self.current_time),
            'attacks_applied': scenario._attack_count,
        }
    
    def get_active_scenario_details(self) -> List[Dict]:
        """Get details of currently active scenarios."""
        return [self._scenario_to_detail(s) for s in self.current_scenarios]

    def get_scenario_details(self) -> List[Dict]:
        """Get details for all configured scenarios."""
        return [self._scenario_to_detail(s) for s in self.scenarios]
    
    def is_attack_active(self) -> bool:
        """Check if any attack is currently active."""
        return self.attack_active
    
    def get_scenario_count(self) -> int:
        """Get total number of configured scenarios."""
        return len(self.scenarios)
    
    def log_status(self) -> None:
        """Log current attack module status."""
        if not self.logger:
            return
        
        status = self.get_attack_status()
        
        if status['attack_active']:
            self.logger.info(
                f"AttackModule Status: ACTIVE - "
                f"{status['active_scenarios']}/{status['total_scenarios']} scenarios, "
                f"Elapsed: {status['elapsed_time']:.2f}s, "
                f"Applied: {status['statistics']['total_attacks_applied']}"
            )
            for scenario in self.current_scenarios:
                self.logger.info(
                    f"  → {scenario.scenario_name}: "
                    f"{scenario.attack_type.value}/{scenario.modification_type.value} "
                    f"on {scenario.target_fields}"
                )
        else:
            self.logger.info(
                f"AttackModule Status: INACTIVE - "
                f"{status['total_scenarios']} scenario(s) configured"
            )


# Convenience function for creating attack module from config
def create_attack_module(vehicle_id: int, config_path: Optional[str] = None,
                        logger: Optional[logging.Logger] = None,
                        enabled_only: bool = True) -> Optional[AttackModule]:
    """
    Create an attack module, optionally loading scenarios from config.
    
    Args:
        vehicle_id: Vehicle ID
        config_path: Path to attack_config.yaml (optional)
        logger: Logger instance
        enabled_only: Only load enabled scenarios from config
        
    Returns:
        AttackModule instance or None if no scenarios configured
    """
    from .AttackScenarios import load_scenarios_from_config
    
    attack_module = AttackModule(vehicle_id=vehicle_id, logger=logger)
    
    if config_path:
        load_scenarios_from_config(attack_module, config_path, enabled_only)
    
    if attack_module.get_scenario_count() == 0:
        if logger:
            logger.info(f"No attack scenarios configured for vehicle {vehicle_id}")
        return None
    
    return attack_module
