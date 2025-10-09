"""
Unit tests for refactored vehicle control components

Run with: python -m pytest test_components.py -v
"""

import pytest
import numpy as np
import time

from config import VehicleControlConfig, SpeedControlConfig, SteeringControlConfig
from state_machine import VehicleState, VehicleStateMachine
from safety import ControlValidator, SensorHealthMonitor, CollisionAvoidance, WatchdogTimer
from controllers import SpeedController, SteeringController


class TestConfiguration:
    """Test configuration management"""
    
    def test_default_config(self):
        """Test default configuration creation"""
        config = VehicleControlConfig()
        assert config.speed.v_ref == 0.75
        assert config.timing.controller_update_rate == 200
        assert config.network.car_id == 0
    
    def test_config_to_dict(self):
        """Test configuration serialization"""
        config = VehicleControlConfig()
        config_dict = config.to_dict()
        assert 'speed' in config_dict
        assert 'steering' in config_dict
        assert config_dict['speed']['v_ref'] == 0.75
    
    def test_config_from_dict(self):
        """Test configuration deserialization"""
        config_dict = {
            'speed': {'v_ref': 1.0, 'K_p': 0.2},
            'network': {'car_id': 5}
        }
        config = VehicleControlConfig.from_dict(config_dict)
        assert config.speed.v_ref == 1.0
        assert config.speed.K_p == 0.2
        assert config.network.car_id == 5


class TestStateMachine:
    """Test state machine functionality"""
    
    def test_initial_state(self):
        """Test initial state"""
        sm = VehicleStateMachine()
        assert sm.state == VehicleState.INITIALIZING
    
    def test_valid_transition(self):
        """Test valid state transition"""
        sm = VehicleStateMachine()
        success = sm.transition_to(VehicleState.WAITING_FOR_START)
        assert success
        assert sm.state == VehicleState.WAITING_FOR_START
        assert sm.previous_state == VehicleState.INITIALIZING
    
    def test_invalid_transition(self):
        """Test invalid state transition"""
        sm = VehicleStateMachine()
        success = sm.transition_to(VehicleState.FOLLOWING_PATH)
        assert not success
        assert sm.state == VehicleState.INITIALIZING
    
    def test_forced_transition(self):
        """Test forced state transition"""
        sm = VehicleStateMachine()
        success = sm.transition_to(VehicleState.FOLLOWING_PATH, force=True)
        assert success
        assert sm.state == VehicleState.FOLLOWING_PATH
    
    def test_state_callbacks(self):
        """Test state enter/exit callbacks"""
        sm = VehicleStateMachine()
        
        callback_called = {'enter': False, 'exit': False}
        
        def on_enter():
            callback_called['enter'] = True
        
        def on_exit():
            callback_called['exit'] = True
        
        sm.register_on_enter(VehicleState.WAITING_FOR_START, on_enter)
        sm.register_on_exit(VehicleState.INITIALIZING, on_exit)
        
        sm.transition_to(VehicleState.WAITING_FOR_START)
        
        assert callback_called['enter']
        assert callback_called['exit']
    
    def test_time_in_state(self):
        """Test time in state tracking"""
        sm = VehicleStateMachine()
        time.sleep(0.1)
        elapsed = sm.get_time_in_state()
        assert elapsed >= 0.1
    
    def test_state_queries(self):
        """Test state query methods"""
        sm = VehicleStateMachine()
        
        assert not sm.is_operational()
        assert not sm.is_stopped()
        
        sm.transition_to(VehicleState.WAITING_FOR_START)
        sm.transition_to(VehicleState.FOLLOWING_PATH)
        
        assert sm.is_operational()
        assert not sm.is_stopped()
        assert sm.should_control()


class TestSafety:
    """Test safety systems"""
    
    def test_throttle_validation(self):
        """Test throttle command validation"""
        validator = ControlValidator()
        
        # Valid throttle
        valid, clamped = validator.validate_throttle(0.2)
        assert valid
        assert clamped == 0.2
        
        # Out of bounds throttle
        valid, clamped = validator.validate_throttle(0.5)
        assert not valid
        assert clamped == 0.3  # max_throttle
        
        # Negative throttle
        valid, clamped = validator.validate_throttle(-0.2)
        assert valid
        assert clamped == -0.2
    
    def test_steering_validation(self):
        """Test steering command validation"""
        validator = ControlValidator()
        
        # Valid steering
        valid, clamped = validator.validate_steering(0.3)
        assert valid
        assert clamped == 0.3
        
        # Out of bounds steering
        valid, clamped = validator.validate_steering(1.0)
        assert not valid
        assert abs(clamped - np.pi/6) < 1e-6
    
    def test_state_validation(self):
        """Test state value validation"""
        validator = ControlValidator()
        
        # Valid state
        assert validator.validate_state(1.0, 2.0, 0.5)
        
        # Invalid state (NaN)
        assert not validator.validate_state(np.nan, 2.0, 0.5)
        
        # Invalid state (Inf)
        assert not validator.validate_state(1.0, np.inf, 0.5)
        
        # Invalid theta (out of range)
        assert not validator.validate_state(1.0, 2.0, 2*np.pi)
    
    def test_gps_health_monitoring(self):
        """Test GPS health monitoring"""
        monitor = SensorHealthMonitor()
        
        # Healthy GPS
        assert monitor.check_gps_health(True)
        assert monitor.gps_healthy
        
        # GPS timeout simulation
        for _ in range(150):  # Exceed timeout
            monitor.check_gps_health(False)
        
        assert not monitor.gps_healthy
        assert monitor.gps_failures == 1
        
        # GPS recovery
        monitor.check_gps_health(True)
        assert monitor.gps_healthy
    
    def test_collision_avoidance(self):
        """Test collision avoidance system"""
        ca = CollisionAvoidance()
        
        # Safe distances
        emergency, reason = ca.check_collision_risk(1.0, 1.0, 0.5)
        assert not emergency
        
        # Car too close
        emergency, reason = ca.check_collision_risk(0.1, 1.0, 0.5)
        assert emergency
        assert "car" in reason
        
        # Person too close
        emergency, reason = ca.check_collision_risk(1.0, 0.1, 0.5)
        assert emergency
        assert "person" in reason
    
    def test_watchdog_timer(self):
        """Test watchdog timer"""
        watchdog = WatchdogTimer(timeout=0.1)
        
        # No timeout initially
        assert not watchdog.check()
        
        # Wait for timeout
        time.sleep(0.15)
        assert watchdog.check()
        assert watchdog.timeout_count == 1
        
        # Reset and verify
        watchdog.reset()
        assert not watchdog.check()


class TestControllers:
    """Test controller implementations"""
    
    def test_speed_controller_proportional(self):
        """Test speed controller proportional term"""
        config = VehicleControlConfig()
        config.speed.K_p = 1.0
        config.speed.K_i = 0.0
        
        controller = SpeedController(config=config)
        
        # Error = 1.0 (v_ref=1.0, v=0.0)
        u = controller.update(v=0.0, v_ref=1.0, dt=0.01)
        assert u == 1.0 * config.speed.K_p  # Should be 1.0
    
    def test_speed_controller_integral(self):
        """Test speed controller integral term"""
        config = VehicleControlConfig()
        config.speed.K_p = 0.0
        config.speed.K_i = 1.0
        
        controller = SpeedController(config=config)
        
        # Apply constant error
        for _ in range(10):
            u = controller.update(v=0.0, v_ref=1.0, dt=0.1)
        
        # Integral should accumulate (1.0 error * 0.1 dt * 10 iterations = 1.0)
        assert abs(u - 1.0) < 0.1
    
    def test_speed_controller_saturation(self):
        """Test speed controller output saturation"""
        config = VehicleControlConfig()
        config.speed.K_p = 10.0  # Large gain
        
        controller = SpeedController(config=config)
        
        # Large error should saturate
        u = controller.update(v=0.0, v_ref=10.0, dt=0.01)
        assert u == config.speed.max_throttle
    
    def test_steering_controller_waypoint_tracking(self):
        """Test steering controller waypoint advancement"""
        # Create simple path
        waypoints = np.array([
            [0.0, 1.0, 2.0],
            [0.0, 0.0, 0.0]
        ])
        
        controller = SteeringController(waypoints=waypoints, cyclic=False)
        
        # Initially at waypoint 0
        assert controller.get_waypoint_index() == 0
        
        # Move past first waypoint
        delta = controller.update(p=np.array([1.5, 0.0]), th=0.0, speed=0.5)
        
        # Should advance to waypoint 1
        assert controller.get_waypoint_index() == 1
    
    def test_steering_controller_cross_track_error(self):
        """Test steering controller cross-track error calculation"""
        # Straight line path
        waypoints = np.array([
            [0.0, 10.0],
            [0.0, 0.0]
        ])
        
        controller = SteeringController(waypoints=waypoints)
        
        # Position offset from path
        delta = controller.update(p=np.array([5.0, 1.0]), th=0.0, speed=0.5)
        
        # Check that cross-track error is calculated
        cte, _ = controller.get_errors()
        assert abs(cte - 1.0) < 0.1  # Should be approximately 1.0


class TestIntegration:
    """Integration tests for component interaction"""
    
    def test_config_controller_integration(self):
        """Test configuration loading into controllers"""
        config = VehicleControlConfig()
        config.speed.v_ref = 1.5
        config.speed.K_p = 0.5
        
        controller = SpeedController(config=config)
        assert controller.kp == 0.5
        assert controller.max_throttle == config.speed.max_throttle
    
    def test_validator_controller_integration(self):
        """Test validator with controller output"""
        config = VehicleControlConfig()
        controller = SpeedController(config=config)
        validator = ControlValidator(config=config)
        
        # Generate control command
        u = controller.update(v=0.0, v_ref=10.0, dt=0.01)  # Large error
        
        # Validate and clamp
        valid, u_clamped = validator.validate_throttle(u)
        
        # Should be clamped to max_throttle
        assert u_clamped == config.speed.max_throttle


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
