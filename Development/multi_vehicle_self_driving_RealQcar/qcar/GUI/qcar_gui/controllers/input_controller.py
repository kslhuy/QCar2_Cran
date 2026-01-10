"""
Input Controller module for QCar Fleet Controller.

This module handles keyboard and steering wheel input for manual vehicle control.
"""

import threading
from typing import Optional, Set, Callable
from dataclasses import dataclass

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

from ..config import ManualControlConfig


@dataclass
class ManualControlState:
    """Current state of manual control inputs."""
    throttle: float = 0.0
    steering: float = 0.0
    emergency_stop: bool = False


class KeyboardController:
    """Handles keyboard input for manual vehicle control."""
    
    def __init__(self, config: ManualControlConfig = None):
        self.config = config or ManualControlConfig()
        self._keys_pressed: Set[str] = set()
        self._current_steering: float = 0.0
        self._bound = False
    
    def bind_to_widget(self, widget) -> None:
        """Bind keyboard events to a tkinter widget."""
        if self._bound:
            return
        
        widget.bind('<KeyPress>', self._on_key_press)
        widget.bind('<KeyRelease>', self._on_key_release)
        self._bound = True
    
    def unbind_from_widget(self, widget) -> None:
        """Unbind keyboard events from a tkinter widget."""
        if not self._bound:
            return
        
        widget.unbind('<KeyPress>')
        widget.unbind('<KeyRelease>')
        self._bound = False
    
    def _on_key_press(self, event) -> None:
        """Handle key press event."""
        key = event.keysym.lower()
        self._keys_pressed.add(key)
    
    def _on_key_release(self, event) -> None:
        """Handle key release event."""
        key = event.keysym.lower()
        self._keys_pressed.discard(key)
    
    def get_control_state(self) -> ManualControlState:
        """Get current control state based on pressed keys."""
        cfg = self.config
        state = ManualControlState()
        
        # Throttle control
        if cfg.forward_key in self._keys_pressed:
            state.throttle = 0.15
        elif cfg.backward_key in self._keys_pressed:
            state.throttle = -0.15
        
        # Gradual steering
        if cfg.left_key in self._keys_pressed:
            self._current_steering = min(
                self._current_steering + cfg.steering_scale * 0.2,
                cfg.steering_scale
            )
        elif cfg.right_key in self._keys_pressed:
            self._current_steering = max(
                self._current_steering - cfg.steering_scale * 0.2,
                -cfg.steering_scale
            )
        else:
            # Decay steering when no input
            self._current_steering *= 0.7
            if abs(self._current_steering) < 0.05:
                self._current_steering = 0.0
        
        state.steering = self._current_steering
        
        # Emergency stop
        if cfg.stop_key in self._keys_pressed:
            state.throttle = 0.0
            state.steering = 0.0
            state.emergency_stop = True
            self._current_steering = 0.0
        
        return state
    
    def reset(self) -> None:
        """Reset controller state."""
        self._keys_pressed.clear()
        self._current_steering = 0.0
    
    @staticmethod
    def get_help_text() -> str:
        """Get help text for keyboard controls."""
        return (
            "Keyboard Controls:\n"
            "  W = Forward\n"
            "  S = Backward\n"
            "  A = Steer Left (hold to increase)\n"
            "  D = Steer Right (hold to increase)\n"
            "  Space = Emergency Stop"
        )


class SteeringWheelController:
    """Handles steering wheel/gamepad input for manual vehicle control."""
    
    def __init__(self, config: ManualControlConfig = None):
        self.config = config or ManualControlConfig()
        self._wheel: Optional['pygame.joystick.Joystick'] = None
        self._initialized = False
        self._error_message: Optional[str] = None
    
    @property
    def is_available(self) -> bool:
        """Check if pygame is available."""
        return PYGAME_AVAILABLE
    
    def initialize(self) -> bool:
        """Initialize steering wheel."""
        if not PYGAME_AVAILABLE:
            self._error_message = "pygame not available"
            return False
        
        try:
            pygame.init()
            pygame.joystick.init()
            
            if pygame.joystick.get_count() == 0:
                self._error_message = "No steering wheel or gamepad detected"
                return False
            
            self._wheel = pygame.joystick.Joystick(0)
            self._wheel.init()
            self._initialized = True
            
            return True
            
        except Exception as e:
            self._error_message = str(e)
            return False
    
    def get_device_info(self) -> dict:
        """Get information about the connected device."""
        if not self._initialized or not self._wheel:
            return {'connected': False, 'error': self._error_message}
        
        return {
            'connected': True,
            'name': self._wheel.get_name(),
            'axes': self._wheel.get_numaxes(),
            'buttons': self._wheel.get_numbuttons()
        }
    
    def get_control_state(self) -> ManualControlState:
        """Get current control state from steering wheel."""
        state = ManualControlState()
        
        if not self._initialized or not self._wheel:
            return state
        
        try:
            # Process pygame events
            pygame.event.pump()
            
            cfg = self.config
            
            # Steering input
            if self._wheel.get_numaxes() > cfg.steering_axis:
                steering_input = self._wheel.get_axis(cfg.steering_axis)
                if abs(steering_input) < cfg.deadzone:
                    steering_input = 0
                state.steering = steering_input * cfg.steering_scale
            
            # Accelerator pedal
            accelerator = 0.0
            if self._wheel.get_numaxes() > cfg.accelerator_axis:
                acc_input = self._wheel.get_axis(cfg.accelerator_axis)
                accelerator = (acc_input + 1) / 2  # Normalize to 0-1
                if accelerator < cfg.deadzone:
                    accelerator = 0
            
            # Brake pedal
            brake = 0.0
            if self._wheel.get_numaxes() > cfg.brake_axis:
                brake_input = self._wheel.get_axis(cfg.brake_axis)
                brake = (brake_input + 1) / 2  # Normalize to 0-1
                if brake < cfg.deadzone:
                    brake = 0
            
            # Combined throttle
            state.throttle = (accelerator - brake) * cfg.throttle_scale
            
            # Emergency stop button
            if self._wheel.get_numbuttons() > 0 and self._wheel.get_button(0):
                state.throttle = 0.0
                state.steering = 0.0
                state.emergency_stop = True
            
        except Exception:
            pass
        
        return state
    
    def cleanup(self) -> None:
        """Cleanup pygame resources."""
        if self._initialized:
            try:
                pygame.quit()
            except Exception:
                pass
            self._initialized = False
            self._wheel = None
    
    @staticmethod
    def get_help_text() -> str:
        """Get help text for wheel controls."""
        return (
            "Steering Wheel Controls:\n"
            "  Wheel = Steering\n"
            "  Accelerator Pedal = Forward\n"
            "  Brake Pedal = Backward\n"
            "  Button 0 = Emergency Stop"
        )


class ManualInputController:
    """
    Unified controller for manual input handling.
    
    Manages both keyboard and steering wheel input sources.
    """
    
    def __init__(self, config: ManualControlConfig = None):
        self.config = config or ManualControlConfig()
        self._keyboard = KeyboardController(config)
        self._wheel = SteeringWheelController(config)
        self._active_controller: str = 'keyboard'
        self._running = False
        self._loop_thread: Optional[threading.Thread] = None
        self._callback: Optional[Callable[[int, float, float], None]] = None
        self._active_car_id: Optional[int] = None
    
    def set_control_type(self, control_type: str) -> bool:
        """
        Set the active control type.
        
        Args:
            control_type: 'keyboard' or 'wheel'
            
        Returns:
            True if control type was set successfully
        """
        if control_type == 'wheel':
            if not self._wheel.is_available:
                return False
            if not self._wheel.initialize():
                return False
        
        self._active_controller = control_type
        return True
    
    def bind_keyboard(self, widget) -> None:
        """Bind keyboard to a widget."""
        self._keyboard.bind_to_widget(widget)
    
    def start(self, car_id: int, callback: Callable[[int, float, float], None],
              interval_ms: int = 20) -> None:
        """
        Start the manual control loop.
        
        Args:
            car_id: ID of the car to control
            callback: Function to call with (car_id, throttle, steering)
            interval_ms: Update interval in milliseconds
        """
        self._active_car_id = car_id
        self._callback = callback
        self._running = True
        
        def control_loop():
            import time
            interval = interval_ms / 1000.0
            
            while self._running and self._active_car_id is not None:
                state = self.get_control_state()
                
                if self._callback and self._active_car_id is not None:
                    self._callback(
                        self._active_car_id,
                        state.throttle,
                        state.steering
                    )
                
                time.sleep(interval)
        
        self._loop_thread = threading.Thread(target=control_loop, daemon=True)
        self._loop_thread.start()
    
    def stop(self) -> None:
        """Stop the manual control loop."""
        self._running = False
        self._active_car_id = None
        if self._loop_thread:
            self._loop_thread.join(timeout=0.5)
            self._loop_thread = None
    
    def get_control_state(self) -> ManualControlState:
        """Get current control state from the active controller."""
        if self._active_controller == 'wheel':
            return self._wheel.get_control_state()
        return self._keyboard.get_control_state()
    
    def reset(self) -> None:
        """Reset all controllers."""
        self._keyboard.reset()
    
    def cleanup(self) -> None:
        """Cleanup all resources."""
        self.stop()
        self._wheel.cleanup()
    
    @property
    def active_controller(self) -> str:
        """Get the active controller type."""
        return self._active_controller
    
    def get_help_text(self) -> str:
        """Get help text for the active controller."""
        if self._active_controller == 'wheel':
            return self._wheel.get_help_text()
        return self._keyboard.get_help_text()
