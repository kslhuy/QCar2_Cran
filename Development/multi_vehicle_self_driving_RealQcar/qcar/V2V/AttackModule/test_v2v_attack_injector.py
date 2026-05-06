"""
Test Cases for V2VAttackInjector Module Integration with V2V Communication

This module tests the ACTUAL V2VAttackInjector, AttackModule, and AttackScenarios
modules to verify correct integration with V2V communication.

Usage:
    Run from the qcar directory:
        cd qcar
        python -m V2V.AttackModule.test_v2v_attack_injector

    Or from the V2V directory:
        cd V2V
        python -m pytest AttackModule/test_v2v_attack_injector.py -v

Author: Fleet Framework Security Research
Version: 1.0.0
"""

import unittest
import time
import logging
import sys
import os
from typing import Dict, List, Optional
from pathlib import Path

# =============================================================================
# Path Setup for proper imports
# =============================================================================

# Get the directory structure
_THIS_DIR = Path(__file__).parent.resolve()          # AttackModule/
_V2V_DIR = _THIS_DIR.parent.resolve()                 # V2V/
_QCAR_DIR = _V2V_DIR.parent.resolve()                 # qcar/

# Add qcar to path so we can import V2V as a package
if str(_QCAR_DIR) not in sys.path:
    sys.path.insert(0, str(_QCAR_DIR))

# Setup logging
logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)

# =============================================================================
# Import the REAL modules
# =============================================================================

from V2V.AttackModule.AttackModule import (
    AttackModule,
    AttackScenario,
    AttackType,
    DataType,
    ModificationType,
)

from V2V.AttackModule.AttackScenarios import (
    make_scenario,
    create_bogus_scenarios,
    create_dos_scenarios,
    load_scenarios_from_config,
)

from V2V.AttackModule.V2VAttackInjector import (
    V2VAttackInjector,
    create_v2v_with_attack_injection,
)

print(f"✓ Successfully imported real modules from: {_QCAR_DIR}")


# =============================================================================
# Mock Classes for V2VManager (needed since we don't want to use real sockets)
# =============================================================================

class MockVehicleObserver:
    """Mock VehicleObserver for testing V2V broadcasts."""
    
    def __init__(self, vehicle_id: int = 1):
        self.vehicle_id = vehicle_id
        self.local_state_counter = 0
        self.fleet_state_counter = 0
    
    def get_local_state_for_broadcast(self) -> Dict:
        """Return a mock local state."""
        self.local_state_counter += 1
        return {
            'vehicle_id': self.vehicle_id,
            'x': 100.0 + self.local_state_counter * 0.1,
            'y': 50.0,
            'theta': 0.5,
            'velocity': 5.0,
            'acceleration': 0.1,
            'source': 'local_sensors'
        }
    
    def get_fleet_state_for_broadcast(self) -> Dict:
        """Return a mock fleet state."""
        self.fleet_state_counter += 1
        return {
            'sender_id': self.vehicle_id,
            'fleet_states': {
                1: {'x': 100.0, 'y': 50.0, 'theta': 0.5, 'velocity': 5.0, 'confidence': 0.95},
                2: {'x': 110.0, 'y': 50.0, 'theta': 0.5, 'velocity': 5.0, 'confidence': 0.90},
                3: {'x': 120.0, 'y': 50.0, 'theta': 0.5, 'velocity': 5.0, 'confidence': 0.85},
            },
            'source': 'fleet_consensus'
        }


class MockV2VCommunication:
    """Mock V2VCommunication for testing (no real sockets)."""
    
    def __init__(self, vehicle_id: int = 1):
        self.vehicle_id = vehicle_id
        self.messages_sent = []
        self._is_active = False
    
    def send_message(self, message_type: str, data: Dict) -> bool:
        """Mock sending a message."""
        self.messages_sent.append({
            'message_type': message_type,
            'data': data.copy(),
            'timestamp': time.time()
        })
        return True
    
    def activate(self, peer_vehicles: List[int], peer_ips: List[str]) -> bool:
        self._is_active = True
        return True
    
    def deactivate(self):
        self._is_active = False
    
    @property
    def is_active(self):
        return self._is_active


class MockLogger:
    """Mock Logger for testing."""
    
    def __init__(self):
        self.logs = []
        self.logger = self
    
    def info(self, msg):
        self.logs.append(('INFO', msg))
    
    def warning(self, msg):
        self.logs.append(('WARNING', msg))
    
    def error(self, msg):
        self.logs.append(('ERROR', msg))
    
    def debug(self, msg):
        self.logs.append(('DEBUG', msg))


class MockV2VManager:
    """Mock V2VManager for testing V2VAttackInjector."""
    
    def __init__(self, vehicle_id: int = 1):
        self.vehicle_id = vehicle_id
        self.logger = MockLogger()
        self.vehicle_observer = MockVehicleObserver(vehicle_id)
        self.v2v_communication = MockV2VCommunication(vehicle_id)
        
        import threading
        self._lock = threading.RLock()
        
        self.stats = {
            'local_broadcasts': 0,
            'fleet_broadcasts': 0,
            'messages_received': 0,
            'messages_processed': 0
        }
    
    def _broadcast_heartbeat(self) -> bool:
        return True
    
    def _process_received_messages(self) -> None:
        pass
    
    def activate(self, peer_vehicles: List[int], peer_ips: List[str]) -> bool:
        return self.v2v_communication.activate(peer_vehicles, peer_ips)
    
    def deactivate(self) -> None:
        self.v2v_communication.deactivate()
    
    def activate_v2v(self, peer_vehicles: List[int], peer_ips: List[str]) -> bool:
        return self.activate(peer_vehicles, peer_ips)
    
    def disable_v2v(self) -> bool:
        self.deactivate()
        return True
    
    def is_active(self) -> bool:
        return self.v2v_communication.is_active
    
    def get_connection_status(self) -> Dict:
        return {'active': self.is_active(), 'peers': []}
    
    def get_status_summary(self) -> str:
        return f"V2V Status: Vehicle {self.vehicle_id}"
    
    def get_statistics(self) -> Dict:
        return self.stats.copy()
    
    def get_latest_local_state_raw(self, vehicle_id: int):
        return None
    
    def get_latest_fleet_state_raw(self, vehicle_id: int):
        return None
    
    def get_direct_leader_data(self, current_vehicle_position: int):
        return None
    
    def get_my_direct_leader_data(self):
        return None
    
    def update_vehicle_observer(self, vehicle_observer) -> None:
        self.vehicle_observer = vehicle_observer
    
    def update_vehicle_logic(self, vehicle_logic) -> None:
        pass
    
    def update_platoon_formation(self, formation_map: Dict) -> None:
        pass
    
    def send_intent(self, intention: str, parameters: Dict) -> bool:
        return True
    
    def send_warning(self, warning_type: str, urgency: str, data: Dict) -> bool:
        return True
    
    def cleanup_old_data(self) -> None:
        pass
    
    def log_received_data_summary(self) -> None:
        pass


# =============================================================================
# Test Class 1: AttackModule Tests
# =============================================================================

class TestAttackModule(unittest.TestCase):
    """Test cases for the real AttackModule."""
    
    def setUp(self):
        self.logger = MockLogger()
        self.attack_module = AttackModule(vehicle_id=1, logger=self.logger)
    
    def test_attack_module_initialization(self):
        """Test AttackModule initializes correctly."""
        self.assertEqual(self.attack_module.vehicle_id, 1)
        self.assertEqual(self.attack_module.get_scenario_count(), 0)
        self.assertFalse(self.attack_module.is_attack_active())
        print("  ✓ AttackModule initialization test passed")
    
    def test_add_scenario(self):
        """Test adding a scenario to AttackModule."""
        scenario = make_scenario(
            attacker_id=1, victim_ids=-1,
            t_start=5.0, t_end=10.0,
            modification_type='scaling', intensity=0.5,
            data_type='local', target_fields=['velocity'],
            scenario_name="TestScenario"
        )
        
        self.attack_module.add_scenario(scenario)
        self.assertEqual(self.attack_module.get_scenario_count(), 1)
        print("  ✓ Add scenario test passed")
    
    def test_attack_timing(self):
        """Test attack activation based on timing."""
        scenario = make_scenario(
            attacker_id=1, victim_ids=-1,
            t_start=0.0, t_end=10.0,
            modification_type='scaling', intensity=0.5,
            data_type='local', target_fields=['velocity']
        )
        self.attack_module.add_scenario(scenario)
        
        # Update with time in attack window
        self.attack_module.update(5.0)
        self.assertTrue(self.attack_module.is_attack_active())
        
        # Update with time after attack window
        self.attack_module.update(15.0)
        self.assertFalse(self.attack_module.is_attack_active())
        print("  ✓ Attack timing test passed")
    
    def test_apply_scaling_attack_to_local_state(self):
        """Test applying scaling attack to local state."""
        scenario = make_scenario(
            attacker_id=1, victim_ids=-1,
            t_start=0.0, t_end=100.0,
            modification_type='scaling', intensity=0.5,
            data_type='local', target_fields=['velocity']
        )
        self.attack_module.add_scenario(scenario)
        self.attack_module.update(5.0)
        
        local_state = {
            'vehicle_id': 1, 'x': 100.0, 'y': 50.0, 'theta': 0.5,
            'velocity': 10.0, 'source': 'local_sensors'
        }
        
        modified = self.attack_module.apply_attack_to_local_state(local_state)
        
        # Velocity should be scaled by 0.5: 10.0 * 0.5 = 5.0
        self.assertAlmostEqual(modified['velocity'], 5.0, places=5)
        print(f"  ✓ Scaling attack test passed (10.0 * 0.5 = {modified['velocity']})")
    
    def test_apply_bias_attack_to_local_state(self):
        """Test applying bias attack to local state."""
        scenario = make_scenario(
            attacker_id=1, victim_ids=-1,
            t_start=0.0, t_end=100.0,
            modification_type='bias', intensity=15.0,
            data_type='local', target_fields=['x']
        )
        self.attack_module.add_scenario(scenario)
        self.attack_module.update(5.0)
        
        local_state = {
            'vehicle_id': 1, 'x': 100.0, 'y': 50.0, 'theta': 0.5,
            'velocity': 10.0, 'source': 'local_sensors'
        }
        
        modified = self.attack_module.apply_attack_to_local_state(local_state)
        
        # X should have +15.0 bias: 100.0 + 15.0 = 115.0
        self.assertAlmostEqual(modified['x'], 115.0, places=5)
        print(f"  ✓ Bias attack test passed (100.0 + 15.0 = {modified['x']})")

    def test_attack_status_includes_local_value_snapshot(self):
        """Test attack status exposes original/modified local values."""
        scenario = make_scenario(
            attacker_id=1,
            victim_ids=-1,
            t_start=0.0,
            t_end=100.0,
            modification_type='scaling',
            intensity=0.5,
            data_type='local',
            target_fields=['velocity'],
        )
        self.attack_module.add_scenario(scenario)
        self.attack_module.update(5.0)

        local_state = {
            'vehicle_id': 1,
            'x': 100.0,
            'y': 50.0,
            'theta': 0.5,
            'velocity': 10.0,
            'acceleration': 0.1,
            'source': 'local_sensors',
        }
        self.attack_module.apply_attack_to_local_state(local_state)

        status = self.attack_module.get_attack_status()
        snapshot = status.get('attack_value_snapshot', {}).get('by_vehicle', {}).get(1, {})
        velocity_snapshot = snapshot.get('values', {}).get('velocity', {})

        self.assertAlmostEqual(velocity_snapshot.get('original'), 10.0, places=5)
        self.assertAlmostEqual(velocity_snapshot.get('modified'), 5.0, places=5)
        self.assertAlmostEqual(velocity_snapshot.get('delta'), -5.0, places=5)
        print("  ✓ Local attack value snapshot test passed")

    def test_attack_status_includes_fleet_value_snapshot(self):
        """Test attack status exposes original/modified fleet values per victim."""
        scenario = make_scenario(
            attacker_id=1,
            victim_ids=[2],
            t_start=0.0,
            t_end=100.0,
            modification_type='zero',
            intensity=0.0,
            data_type='fleet',
            target_fields=['velocity'],
            attack_type='DoS',
        )
        self.attack_module.add_scenario(scenario)
        self.attack_module.update(5.0)

        fleet_state = {
            'sender_id': 1,
            'fleet_states': {
                1: {'x': 100.0, 'y': 50.0, 'theta': 0.5, 'velocity': 5.0, 'confidence': 0.95},
                2: {'x': 110.0, 'y': 50.0, 'theta': 0.5, 'velocity': 5.0, 'confidence': 0.90},
            },
            'source': 'fleet_consensus',
        }
        self.attack_module.apply_attack_to_fleet_state(fleet_state)

        status = self.attack_module.get_attack_status()
        snapshot = status.get('attack_value_snapshot', {}).get('by_vehicle', {}).get(2, {})
        velocity_snapshot = snapshot.get('values', {}).get('velocity', {})

        self.assertAlmostEqual(velocity_snapshot.get('original'), 5.0, places=5)
        self.assertAlmostEqual(velocity_snapshot.get('modified'), 0.0, places=5)
        self.assertAlmostEqual(velocity_snapshot.get('delta'), -5.0, places=5)
        print("  ✓ Fleet attack value snapshot test passed")
    
    def test_clear_scenarios(self):
        """Test clearing all scenarios."""
        scenario = make_scenario(
            attacker_id=1, victim_ids=-1,
            t_start=0.0, t_end=10.0,
            modification_type='scaling', intensity=0.5,
            data_type='local', target_fields=['velocity']
        )
        self.attack_module.add_scenario(scenario)
        self.assertEqual(self.attack_module.get_scenario_count(), 1)
        
        self.attack_module.clear_scenarios()
        self.assertEqual(self.attack_module.get_scenario_count(), 0)
        print("  ✓ Clear scenarios test passed")


# =============================================================================
# Test Class 2: AttackScenarios Factory Tests
# =============================================================================

class TestAttackScenarios(unittest.TestCase):
    """Test cases for AttackScenarios factory functions."""
    
    def test_make_scenario(self):
        """Test make_scenario factory function."""
        scenario = make_scenario(
            attacker_id=1, victim_ids=-1,
            t_start=10.0, t_end=20.0,
            modification_type='scaling', intensity=0.5,
            data_type='local', target_fields=['velocity'],
            scenario_name="TestScaling"
        )
        
        self.assertIsInstance(scenario, AttackScenario)
        self.assertEqual(scenario.attacker_id, 1)
        self.assertEqual(scenario.t_start, 10.0)
        self.assertEqual(scenario.t_end, 20.0)
        self.assertEqual(scenario.modification_type, ModificationType.SCALING)
        self.assertEqual(scenario.data_type, DataType.LOCAL)
        print("  ✓ make_scenario test passed")
    
    def test_create_bogus_scenarios(self):
        """Test predefined bogus scenarios creation."""
        attack_module = AttackModule(vehicle_id=1, logger=MockLogger())
        
        # Create case 1: Velocity scaling negative
        create_bogus_scenarios(attack_module, case_number=1, t_start=5.0, t_end=15.0)
        
        self.assertEqual(attack_module.get_scenario_count(), 1)
        print("  ✓ create_bogus_scenarios test passed")
    
    def test_scenario_victim_check(self):
        """Test scenario victim checking."""
        # All victims
        scenario_all = make_scenario(
            attacker_id=1, victim_ids=-1,
            t_start=0.0, t_end=10.0,
            modification_type='scaling', intensity=0.5,
            data_type='local', target_fields=['velocity']
        )
        self.assertTrue(scenario_all.is_victim(1))
        self.assertTrue(scenario_all.is_victim(2))
        self.assertTrue(scenario_all.is_victim(99))
        
        # Specific victims
        scenario_specific = make_scenario(
            attacker_id=1, victim_ids=[2, 3],
            t_start=0.0, t_end=10.0,
            modification_type='scaling', intensity=0.5,
            data_type='local', target_fields=['velocity']
        )
        self.assertFalse(scenario_specific.is_victim(1))
        self.assertTrue(scenario_specific.is_victim(2))
        self.assertTrue(scenario_specific.is_victim(3))
        self.assertFalse(scenario_specific.is_victim(4))
        print("  ✓ Victim check test passed")


# =============================================================================
# Test Class 3: V2VAttackInjector Integration Tests
# =============================================================================

class TestV2VAttackInjectorIntegration(unittest.TestCase):
    """Test cases for V2VAttackInjector with real AttackModule."""
    
    def setUp(self):
        self.mock_v2v_manager = MockV2VManager(vehicle_id=1)
    
    def test_injector_initialization(self):
        """Test V2VAttackInjector initializes correctly."""
        injector = V2VAttackInjector(
            v2v_manager=self.mock_v2v_manager,
            enabled=True
        )
        
        self.assertIsNotNone(injector)
        self.assertEqual(injector.vehicle_id, 1)
        self.assertTrue(injector.enabled)
        self.assertIsNotNone(injector.attack_module)
        self.assertIsInstance(injector.attack_module, AttackModule)
        print("  ✓ V2VAttackInjector initialization test passed")
    
    def test_injector_with_preconfigured_attack_module(self):
        """Test V2VAttackInjector with pre-configured AttackModule."""
        # Create and configure attack module first
        attack_module = AttackModule(vehicle_id=1, logger=MockLogger())
        scenario = make_scenario(
            attacker_id=1, victim_ids=-1,
            t_start=0.0, t_end=100.0,
            modification_type='scaling', intensity=0.5,
            data_type='local', target_fields=['velocity']
        )
        attack_module.add_scenario(scenario)
        
        # Create injector with pre-configured module
        injector = V2VAttackInjector(
            v2v_manager=self.mock_v2v_manager,
            attack_module=attack_module,
            enabled=True
        )
        
        self.assertEqual(injector.attack_module, attack_module)
        self.assertEqual(injector.attack_module.get_scenario_count(), 1)
        print("  ✓ Pre-configured AttackModule test passed")
    
    def test_injector_add_scenario(self):
        """Test adding scenarios through V2VAttackInjector."""
        injector = V2VAttackInjector(
            v2v_manager=self.mock_v2v_manager,
            enabled=True
        )
        
        scenario = make_scenario(
            attacker_id=1, victim_ids=-1,
            t_start=0.0, t_end=100.0,
            modification_type='bias', intensity=10.0,
            data_type='local', target_fields=['x']
        )
        
        injector.add_attack_scenario(scenario)
        self.assertEqual(injector.attack_module.get_scenario_count(), 1)
        print("  ✓ Add scenario through injector test passed")
    
    def test_injector_enable_disable(self):
        """Test enabling/disabling attacks through injector."""
        injector = V2VAttackInjector(
            v2v_manager=self.mock_v2v_manager,
            enabled=False
        )
        
        self.assertFalse(injector.enabled)
        
        injector.enable_attacks()
        self.assertTrue(injector.enabled)
        
        injector.disable_attacks()
        self.assertFalse(injector.enabled)
        print("  ✓ Enable/disable attacks test passed")
    
    def test_injector_attack_status(self):
        """Test getting attack status from injector."""
        injector = V2VAttackInjector(
            v2v_manager=self.mock_v2v_manager,
            enabled=True
        )
        
        status = injector.get_attack_status()
        
        self.assertIn('enabled', status)
        self.assertIn('attack_module_present', status)
        self.assertTrue(status['enabled'])
        self.assertTrue(status['attack_module_present'])
        print("  ✓ Get attack status test passed")


# =============================================================================
# Test Class 4: Attack Injection Flow Tests
# =============================================================================

class TestAttackInjectionFlow(unittest.TestCase):
    """Test the complete attack injection flow."""
    
    def setUp(self):
        self.mock_v2v_manager = MockV2VManager(vehicle_id=1)
    
    def test_velocity_scaling_injection(self):
        """Test velocity scaling attack is injected into broadcasts."""
        # Create injector with attack module
        attack_module = AttackModule(vehicle_id=1, logger=MockLogger())
        scenario = make_scenario(
            attacker_id=1, victim_ids=-1,
            t_start=0.0, t_end=100.0,
            modification_type='scaling', intensity=0.5,
            data_type='local', target_fields=['velocity']
        )
        attack_module.add_scenario(scenario)
        
        injector = V2VAttackInjector(
            v2v_manager=self.mock_v2v_manager,
            attack_module=attack_module,
            enabled=True,
            start_time=time.time() - 5.0  # Started 5 seconds ago
        )
        
        # Trigger broadcast
        injector.update_broadcast()
        
        # Check the message was sent with modified velocity
        messages = self.mock_v2v_manager.v2v_communication.messages_sent
        self.assertGreater(len(messages), 0)
        
        # Find the local_state message (we're attacking local data type)
        local_state_msg = None
        for msg in messages:
            if msg['message_type'] == 'local_state':
                local_state_msg = msg
                break
        
        self.assertIsNotNone(local_state_msg, "local_state message not found")
        # Original velocity is 5.0, scaled by 0.5 = 2.5
        self.assertAlmostEqual(local_state_msg['data']['velocity'], 2.5, places=5)
        print(f"  ✓ Velocity scaling injection test passed (5.0 * 0.5 = {local_state_msg['data']['velocity']})")
    
    def test_position_bias_injection(self):
        """Test position bias attack is injected into broadcasts."""
        attack_module = AttackModule(vehicle_id=1, logger=MockLogger())
        scenario = make_scenario(
            attacker_id=1, victim_ids=-1,
            t_start=0.0, t_end=100.0,
            modification_type='bias', intensity=50.0,
            data_type='local', target_fields=['x']
        )
        attack_module.add_scenario(scenario)
        
        injector = V2VAttackInjector(
            v2v_manager=self.mock_v2v_manager,
            attack_module=attack_module,
            enabled=True,
            start_time=time.time() - 5.0
        )
        
        # Trigger broadcast
        injector.update_broadcast()
        
        messages = self.mock_v2v_manager.v2v_communication.messages_sent
        
        # Find the local_state message (we're attacking local data type)
        local_state_msg = None
        for msg in messages:
            if msg['message_type'] == 'local_state':
                local_state_msg = msg
                break
        
        self.assertIsNotNone(local_state_msg, "local_state message not found")
        # X should be ~100.1 + 50.0 = ~150.1
        self.assertGreater(local_state_msg['data']['x'], 140.0)
        self.assertLess(local_state_msg['data']['x'], 160.0)
        print(f"  ✓ Position bias injection test passed (x = {local_state_msg['data']['x']})")
    
    def test_no_injection_when_disabled(self):
        """Test no attack injection when disabled."""
        attack_module = AttackModule(vehicle_id=1, logger=MockLogger())
        scenario = make_scenario(
            attacker_id=1, victim_ids=-1,
            t_start=0.0, t_end=100.0,
            modification_type='scaling', intensity=0.0,  # Would zero out
            data_type='local', target_fields=['velocity']
        )
        attack_module.add_scenario(scenario)
        
        injector = V2VAttackInjector(
            v2v_manager=self.mock_v2v_manager,
            attack_module=attack_module,
            enabled=False,  # DISABLED
            start_time=time.time() - 5.0
        )
        
        injector.update_broadcast()
        
        messages = self.mock_v2v_manager.v2v_communication.messages_sent
        
        # Find the local_state message
        local_state_msg = None
        for msg in messages:
            if msg['message_type'] == 'local_state':
                local_state_msg = msg
                break
        
        self.assertIsNotNone(local_state_msg, "local_state message not found")
        # Velocity should NOT be zeroed since attacks are disabled
        self.assertEqual(local_state_msg['data']['velocity'], 5.0)
        print("  ✓ No injection when disabled test passed")
    
    def test_no_injection_when_outside_time_window(self):
        """Test no attack injection outside time window."""
        attack_module = AttackModule(vehicle_id=1, logger=MockLogger())
        scenario = make_scenario(
            attacker_id=1, victim_ids=-1,
            t_start=100.0, t_end=200.0,  # Future window
            modification_type='scaling', intensity=0.0,
            data_type='local', target_fields=['velocity']
        )
        attack_module.add_scenario(scenario)
        
        injector = V2VAttackInjector(
            v2v_manager=self.mock_v2v_manager,
            attack_module=attack_module,
            enabled=True,
            start_time=time.time()  # Just started, so elapsed ~0s
        )
        
        injector.update_broadcast()
        
        # Attack should not be active (we're at t=0, attack is at t=100-200)
        self.assertFalse(injector.is_attack_active())
        
        messages = self.mock_v2v_manager.v2v_communication.messages_sent
        
        # Find the local_state message
        local_state_msg = None
        for msg in messages:
            if msg['message_type'] == 'local_state':
                local_state_msg = msg
                break
        
        self.assertIsNotNone(local_state_msg, "local_state message not found")
        self.assertEqual(local_state_msg['data']['velocity'], 5.0)
        print("  ✓ No injection outside time window test passed")


# =============================================================================
# Test Class 5: Passthrough Methods Tests
# =============================================================================

class TestPassthroughMethods(unittest.TestCase):
    """Test V2VManager passthrough methods on injector."""
    
    def setUp(self):
        self.mock_v2v_manager = MockV2VManager(vehicle_id=1)
        self.injector = V2VAttackInjector(
            v2v_manager=self.mock_v2v_manager,
            enabled=True
        )
    
    def test_activate_passthrough(self):
        """Test activate method passes through."""
        result = self.injector.activate([2, 3], ['192.168.1.2', '192.168.1.3'])
        self.assertTrue(result)
        self.assertTrue(self.mock_v2v_manager.v2v_communication.is_active)
        print("  ✓ Activate passthrough test passed")
    
    def test_deactivate_passthrough(self):
        """Test deactivate method passes through."""
        self.injector.activate([2], ['192.168.1.2'])
        self.injector.deactivate()
        self.assertFalse(self.mock_v2v_manager.v2v_communication.is_active)
        print("  ✓ Deactivate passthrough test passed")
    
    def test_property_passthroughs(self):
        """Test property passthroughs work correctly."""
        self.assertEqual(self.injector.v2v_communication, self.mock_v2v_manager.v2v_communication)
        self.assertEqual(self.injector.vehicle_observer, self.mock_v2v_manager.vehicle_observer)
        print("  ✓ Property passthroughs test passed")


# =============================================================================
# Test Class 6: Factory Function Tests
# =============================================================================

class TestFactoryFunction(unittest.TestCase):
    """Test the factory function."""
    
    def test_create_v2v_with_attack_injection(self):
        """Test factory function creates proper injector."""
        mock_v2v_manager = MockV2VManager(vehicle_id=1)
        
        injector = create_v2v_with_attack_injection(
            v2v_manager=mock_v2v_manager,
            enabled=True
        )
        
        self.assertIsInstance(injector, V2VAttackInjector)
        self.assertTrue(injector.enabled)
        print("  ✓ Factory function test passed")


# =============================================================================
# Test Class 7: Config Loading Tests
# =============================================================================

class TestConfigLoading(unittest.TestCase):
    """Test loading attack scenarios from config file."""
    
    def test_load_scenarios_from_config(self):
        """Test loading scenarios from attack_config.yaml."""
        attack_module = AttackModule(vehicle_id=1, logger=MockLogger())
        
        # Get the config file path
        config_path = _THIS_DIR / "attack_config.yaml"
        
        if config_path.exists():
            load_scenarios_from_config(attack_module, str(config_path), enabled_only=True)
            # At least one scenario should be loaded (the enabled one in config)
            self.assertGreater(attack_module.get_scenario_count(), 0)
            print(f"  ✓ Config loading test passed ({attack_module.get_scenario_count()} scenarios loaded)")
        else:
            self.skipTest(f"Config file not found: {config_path}")


# =============================================================================
# Main Test Runner
# =============================================================================

def run_tests():
    """Run all test cases."""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    test_classes = [
        TestAttackModule,
        TestAttackScenarios,
        TestV2VAttackInjectorIntegration,
        TestAttackInjectionFlow,
        TestPassthroughMethods,
        TestFactoryFunction,
        TestConfigLoading,
    ]
    
    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)
    
    print("\n" + "="*70)
    print("V2VAttackInjector Integration Tests (Testing REAL Modules)")
    print("="*70 + "\n")
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    print("\n" + "="*70)
    print("Test Summary")
    print("="*70)
    print(f"Tests Run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success: {result.wasSuccessful()}")
    print("="*70 + "\n")
    
    return result


if __name__ == '__main__':
    run_tests()
