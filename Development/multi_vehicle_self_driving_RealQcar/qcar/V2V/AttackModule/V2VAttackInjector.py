"""
V2V Attack Injector - Middleware for V2V Communication Attack Injection

This module provides a seamless integration layer between the AttackModule
and V2VManager, enabling transparent fault injection into V2V communications.

The V2VAttackInjector wraps the V2VManager and intercepts all outgoing
broadcasts to apply attack modifications when scenarios are active.

Usage:
    # Replace V2VManager with V2VAttackInjector in vehicle initialization
    v2v_manager = V2VManager(vehicle_id, logger, config, observer)
    attack_injector = V2VAttackInjector(v2v_manager, attack_config_path="attack_config.yaml")
    
    # Use attack_injector instead of v2v_manager
    attack_injector.update_broadcast()  # Will inject faults if attack is active

Author: Fleet Framework Security Research
Version: 2.0.0
Updated: January 2026
"""

import time
import logging
import threading
from typing import Dict, List, Optional, Callable, Any
from pathlib import Path


class V2VAttackInjector:
    """
    V2V Attack Injector - Middleware for transparent fault injection.
    
    This class wraps V2VManager and intercepts broadcast operations to inject
    faults according to configured attack scenarios. It maintains full 
    compatibility with V2VManager's interface.
    
    Features:
        - Transparent integration (drop-in replacement for V2VManager)
        - Automatic attack timing and scenario management
        - Detailed attack logging
        - Real-time attack status monitoring
        - Multiple simultaneous attack scenarios support
    """
    
    def __init__(self, v2v_manager, attack_config_path: Optional[str] = None,
                 attack_module=None, start_time: Optional[float] = None,
                 enabled: bool = True):
        """
        Initialize the V2V Attack Injector.
        
        Args:
            v2v_manager: The V2VManager instance to wrap
            attack_config_path: Path to attack_config.yaml (optional)
            attack_module: Pre-configured AttackModule instance (optional)
            start_time: Reference start time for attack timing (defaults to now)
            enabled: Whether attack injection is enabled
        """
        self.v2v_manager = v2v_manager
        self.vehicle_id = v2v_manager.vehicle_id
        self.logger = v2v_manager.logger
        self.enabled = enabled
        
        # Timing
        self.start_time = start_time or time.time()
        
        # Initialize attack module
        if attack_module is not None:
            self.attack_module = attack_module
        else:
            self._init_attack_module(attack_config_path)
        
        # State tracking
        self._lock = threading.RLock()
        self._attack_start_logged = False
        
        # Statistics
        self.stats = {
            'broadcasts_intercepted': 0,
            'broadcasts_modified': 0,
            'local_modifications': 0,
            'fleet_modifications': 0,
        }
        
        if self.logger:
            self.logger.info(
                f"V2VAttackInjector initialized for vehicle {self.vehicle_id} - "
                f"Enabled: {enabled}, "
                f"Scenarios: {self.attack_module.get_scenario_count() if self.attack_module else 0}"
            )
    
    def _init_attack_module(self, config_path: Optional[str]) -> None:
        """Initialize the attack module from config or defaults."""
        from .AttackModule import AttackModule
        from .AttackScenarios import load_scenarios_from_config
        
        self.attack_module = AttackModule(
            vehicle_id=self.vehicle_id,
            logger=self.logger
        )
        
        if config_path:
            config_file = Path(config_path)
            if config_file.exists():
                load_scenarios_from_config(
                    self.attack_module, 
                    str(config_file),
                    enabled_only=True
                )
                if self.logger:
                    self.logger.info(
                        f"Loaded {self.attack_module.get_scenario_count()} "
                        f"attack scenarios from {config_path}"
                    )
            else:
                if self.logger:
                    self.logger.warning(f"Attack config not found: {config_path}")
    
    def get_elapsed_time(self) -> float:
        """Get elapsed time since injector start."""
        return time.time() - self.start_time
    
    def reset_start_time(self, new_start_time: Optional[float] = None) -> None:
        """Reset the reference start time for attack timing."""
        self.start_time = new_start_time or time.time()
        if self.logger:
            self.logger.info(f"V2VAttackInjector: Reset start time to {self.start_time}")
    
    # =====================================================================
    # V2VManager Interface Proxy Methods
    # =====================================================================
    
    def update_broadcast(self) -> bool:
        """
        Main update method - intercepts broadcasts and applies attacks.
        This replaces the standard V2VManager.update_broadcast() method.
        
        Returns:
            True if any broadcast was sent
        """
        with self._lock:
            # Update attack module timing
            elapsed_time = self.get_elapsed_time()
            if self.attack_module and self.enabled:
                self.attack_module.update(elapsed_time)
            
            self.stats['broadcasts_intercepted'] += 1
        
        # Intercept and modify broadcast data
        broadcast_sent = self._intercepted_broadcast()
        
        return broadcast_sent
    
    def _intercepted_broadcast(self) -> bool:
        """
        Perform broadcast with attack interception.
        
        This method intercepts the normal broadcast flow and applies
        attack modifications to the data before sending.
        """
        broadcast_sent = False
        
        try:
            # Process local state broadcast with attack injection
            local_sent = self._broadcast_local_state_with_attack()
            
            # Process fleet state broadcast with attack injection  
            fleet_sent = self._broadcast_fleet_state_with_attack()

            # Trust reports are forwarded without modification (independent V2V rate)
            trust_sent = self._broadcast_trust_report()
            
            # Process heartbeat (usually not attacked but could be)
            heartbeat_sent = self._broadcast_heartbeat()
            
            # Process received messages (not attacked - incoming data)
            self._process_received_messages()
            
            broadcast_sent = local_sent or fleet_sent or trust_sent or heartbeat_sent
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"V2VAttackInjector broadcast error: {e}")
        
        return broadcast_sent
    
    def _broadcast_local_state_with_attack(self) -> bool:
        """Broadcast local state with attack injection."""
        try:
            if not self.v2v_manager.vehicle_observer:
                return False
            
            # Get original local state from observer
            local_state = self.v2v_manager.vehicle_observer.get_local_state_for_broadcast()
            
            # Apply attack modification if active
            if self.attack_module and self.enabled and self.attack_module.should_attack_local_data():
                modified_state = self.attack_module.apply_attack_to_local_state(local_state)
                
                with self._lock:
                    self.stats['broadcasts_modified'] += 1
                    self.stats['local_modifications'] += 1
            else:
                modified_state = local_state
            
            # Send modified (or original) state
            success = self.v2v_manager.v2v_communication.send_message(
                message_type="local_state",
                data=modified_state
            )
            
            if success:
                with self.v2v_manager._lock:
                    self.v2v_manager.stats['local_broadcasts'] += 1
            
            return success
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Local state broadcast with attack error: {e}")
            return False
    
    def _broadcast_fleet_state_with_attack(self) -> bool:
        """Broadcast fleet state with attack injection."""
        try:
            if not self.v2v_manager.vehicle_observer:
                return False
            
            # Get original fleet state from observer
            fleet_state = self.v2v_manager.vehicle_observer.get_fleet_state_for_broadcast()
            
            # Apply attack modification if active
            if self.attack_module and self.enabled and self.attack_module.should_attack_fleet_data():
                modified_state = self.attack_module.apply_attack_to_fleet_state(fleet_state)
                
                with self._lock:
                    self.stats['broadcasts_modified'] += 1
                    self.stats['fleet_modifications'] += 1
            else:
                modified_state = fleet_state
            
            # Send modified (or original) state
            success = self.v2v_manager.v2v_communication.send_message(
                message_type="fleet_state",
                data=modified_state
            )
            
            if success:
                with self.v2v_manager._lock:
                    self.v2v_manager.stats['fleet_broadcasts'] += 1
            
            return success
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Fleet state broadcast with attack error: {e}")
            return False
    
    def _broadcast_heartbeat(self) -> bool:
        """Broadcast heartbeat message (delegated to V2VManager)."""
        return self.v2v_manager._broadcast_heartbeat()

    def _broadcast_trust_report(self) -> bool:
        """Broadcast trust report message (delegated to V2VManager)."""
        if hasattr(self.v2v_manager, "_broadcast_trust_report"):
            return self.v2v_manager._broadcast_trust_report()
        return False
    
    def _process_received_messages(self) -> None:
        """Process received messages (delegated to V2VManager)."""
        self.v2v_manager._process_received_messages()
    
    # =====================================================================
    # Attack Module Management
    # =====================================================================
    
    def add_attack_scenario(self, scenario) -> None:
        """Add an attack scenario."""
        if self.attack_module:
            self.attack_module.add_scenario(scenario)
    
    def clear_attack_scenarios(self) -> None:
        """Clear all attack scenarios."""
        if self.attack_module:
            self.attack_module.clear_scenarios()
    
    def enable_attacks(self) -> None:
        """Enable attack injection."""
        self.enabled = True
        if self.logger:
            self.logger.info("V2VAttackInjector: Attacks ENABLED")
    
    def disable_attacks(self) -> None:
        """Disable attack injection."""
        self.enabled = False
        if self.logger:
            self.logger.info("V2VAttackInjector: Attacks DISABLED")
    
    def is_attack_active(self) -> bool:
        """Check if any attack is currently active."""
        if not self.enabled or not self.attack_module:
            return False
        return self.attack_module.is_attack_active()
    
    def get_attack_status(self) -> Dict:
        """Get comprehensive attack status."""
        if not self.attack_module:
            return {
                'enabled': self.enabled,
                'attack_module_present': False,
                'attack_active': False,
            }
        
        status = self.attack_module.get_attack_status()
        status.update({
            'enabled': self.enabled,
            'attack_module_present': True,
            'elapsed_time': self.get_elapsed_time(),
            'injector_stats': self.stats.copy(),
        })
        return status
    
    # =====================================================================
    # V2VManager Passthrough Methods
    # =====================================================================
    
    def activate(self, peer_vehicles: List[int], peer_ips: List[str]) -> bool:
        """Activate V2V communication."""
        result = self.v2v_manager.activate(peer_vehicles, peer_ips)
        if result:
            # Reset start time when V2V is activated
            self.reset_start_time()
        return result
    
    def deactivate(self) -> None:
        """Deactivate V2V communication."""
        return self.v2v_manager.deactivate()
    
    def activate_v2v(self, peer_vehicles: List[int], peer_ips: List[str]) -> bool:
        """Activate V2V (alias)."""
        result = self.v2v_manager.activate_v2v(peer_vehicles, peer_ips)
        if result:
            self.reset_start_time()
        return result
    
    def disable_v2v(self) -> bool:
        """Disable V2V communication."""
        return self.v2v_manager.disable_v2v()
    
    def is_active(self) -> bool:
        """Check if V2V is active."""
        return self.v2v_manager.is_active()
    
    def get_connection_status(self) -> Dict:
        """Get V2V connection status."""
        return self.v2v_manager.get_connection_status()
    
    def get_status_summary(self) -> str:
        """Get status summary including attack info."""
        v2v_summary = self.v2v_manager.get_status_summary()
        
        if self.is_attack_active():
            attack_info = f" | 🔴 ATTACK ACTIVE"
            active_scenarios = self.attack_module.get_active_scenario_details()
            if active_scenarios:
                scenario_names = [s['name'] for s in active_scenarios]
                attack_info += f": {', '.join(scenario_names)}"
            return v2v_summary + attack_info
        
        return v2v_summary
    
    def get_statistics(self) -> Dict:
        """Get combined statistics."""
        v2v_stats = self.v2v_manager.get_statistics()
        
        combined_stats = {
            'v2v': v2v_stats,
            'attack_injector': self.stats.copy(),
        }
        
        if self.attack_module:
            combined_stats['attack_module'] = self.attack_module.get_attack_status()['statistics']
        
        return combined_stats
    
    def get_latest_local_state_raw(self, vehicle_id: int):
        """Get latest local state (passthrough)."""
        return self.v2v_manager.get_latest_local_state_raw(vehicle_id)
    
    def get_latest_fleet_state_raw(self, vehicle_id: int):
        """Get latest fleet state (passthrough)."""
        return self.v2v_manager.get_latest_fleet_state_raw(vehicle_id)
    
    def get_direct_leader_data(self, current_vehicle_position: int):
        """Get direct leader data (passthrough)."""
        return self.v2v_manager.get_direct_leader_data(current_vehicle_position)
    
    def get_my_direct_leader_data(self):
        """Get my direct leader data (passthrough)."""
        return self.v2v_manager.get_my_direct_leader_data()
    
    def update_vehicle_observer(self, vehicle_observer) -> None:
        """Update vehicle observer reference."""
        self.v2v_manager.update_vehicle_observer(vehicle_observer)
    
    def update_vehicle_logic(self, vehicle_logic) -> None:
        """Update vehicle logic reference."""
        self.v2v_manager.update_vehicle_logic(vehicle_logic)
    
    def update_platoon_formation(self, formation_map: Dict) -> None:
        """Update platoon formation mapping."""
        self.v2v_manager.update_platoon_formation(formation_map)
    
    def send_intent(self, intention: str, parameters: Dict) -> bool:
        """Send intent message (passthrough)."""
        return self.v2v_manager.send_intent(intention, parameters)
    
    def send_warning(self, warning_type: str, urgency: str, data: Dict) -> bool:
        """Send warning message (passthrough)."""
        return self.v2v_manager.send_warning(warning_type, urgency, data)
    
    def cleanup_old_data(self) -> None:
        """Cleanup old data (passthrough)."""
        self.v2v_manager.cleanup_old_data()
    
    def log_received_data_summary(self) -> None:
        """Log received data summary (passthrough)."""
        self.v2v_manager.log_received_data_summary()
    
    # =====================================================================
    # Property Passthroughs
    # =====================================================================
    
    @property
    def v2v_communication(self):
        """Access underlying V2V communication."""
        return self.v2v_manager.v2v_communication
    
    @property
    def vehicle_observer(self):
        """Access vehicle observer."""
        return self.v2v_manager.vehicle_observer
    
    @property
    def config(self):
        """Access broadcast config."""
        return self.v2v_manager.config
    
    @property
    def received_local_states(self):
        """Access received local states."""
        return self.v2v_manager.received_local_states
    
    @property
    def received_fleet_states(self):
        """Access received fleet states."""
        return self.v2v_manager.received_fleet_states
    
    @property
    def position_to_vehicle_id_map(self):
        """Access platoon position mapping."""
        return self.v2v_manager.position_to_vehicle_id_map


def create_v2v_with_attack_injection(v2v_manager, attack_config_path: str = None,
                                    enabled: bool = True) -> V2VAttackInjector:
    """
    Factory function to create V2V communication with attack injection capability.
    
    Args:
        v2v_manager: V2VManager instance to wrap
        attack_config_path: Path to attack configuration YAML
        enabled: Whether attacks are initially enabled
        
    Returns:
        V2VAttackInjector instance that wraps the V2VManager
    """
    return V2VAttackInjector(
        v2v_manager=v2v_manager,
        attack_config_path=attack_config_path,
        enabled=enabled
    )
