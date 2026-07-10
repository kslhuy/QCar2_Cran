"""
Input Controller module for QCar Fleet Controller.

This module handles keyboard and steering wheel input for manual vehicle control.
"""

import threading
from typing import Dict, Optional, Set, Callable
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


@dataclass
class KeyboardControlProfile:
    """Local keyboard tuning profile for a specific vehicle."""

    vehicle_type: str = "Qcar"
    starting_forward_throttle: float = 0.15
    max_forward_throttle: float = 0.30
    max_reverse_throttle: float = 0.18
    throttle_step: float = 0.02
    steering_limit: float = 0.50
    steering_step: float = 0.05

    def to_dict(self) -> Dict[str, float]:
        """Return a serializable profile dictionary for the GUI."""
        return {
            "vehicle_type": self.vehicle_type,
            "starting_forward_throttle": self.starting_forward_throttle,
            "max_forward_throttle": self.max_forward_throttle,
            "max_reverse_throttle": self.max_reverse_throttle,
            "throttle_step": self.throttle_step,
            "steering_limit": self.steering_limit,
            "steering_step": self.steering_step,
        }


class KeyboardController:
    """Handles keyboard input for manual vehicle control."""

    PROFILE_LIMITS = {
        "starting_forward_throttle": (0.05, 0.30),
        "max_forward_throttle": (0.05, 0.60),
        "max_reverse_throttle": (0.05, 0.40),
        "throttle_step": (0.005, 0.15),
        "steering_limit": (0.10, 1.00),
        "steering_step": (0.01, 0.25),
    }

    HOTKEY_ADJUSTMENTS = {
        "u": ("max_forward_throttle", -0.01, "Forward max"),
        "i": ("max_forward_throttle", 0.01, "Forward max"),
        "o": ("max_reverse_throttle", -0.01, "Reverse max"),
        "p": ("max_reverse_throttle", 0.01, "Reverse max"),
        "j": ("steering_limit", -0.02, "Steering limit"),
        "k": ("steering_limit", 0.02, "Steering limit"),
        "l": ("steering_step", -0.01, "Steering step"),
        "m": ("steering_step", 0.01, "Steering step"),
    }

    def __init__(self, config: ManualControlConfig = None):
        self.config = config or ManualControlConfig()
        self._keys_pressed: Set[str] = set()
        self._current_throttle: float = 0.0
        self._current_steering: float = 0.0
        self._active_car_id: Optional[int] = None
        self._profiles: Dict[int, KeyboardControlProfile] = {}
        self._profile_callback: Optional[
            Callable[[int, Dict[str, float], str], None]
        ] = None
        self._bound = False
        self._lock = threading.Lock()  # Protect key state from race conditions

    def bind_to_widget(self, widget) -> None:
        """Bind keyboard events to a tkinter widget."""
        if self._bound:
            return

        widget.bind("<KeyPress>", self._on_key_press)
        widget.bind("<KeyRelease>", self._on_key_release)
        self._bound = True

    def unbind_from_widget(self, widget) -> None:
        """Unbind keyboard events from a tkinter widget."""
        if not self._bound:
            return

        widget.unbind("<KeyPress>")
        widget.unbind("<KeyRelease>")
        self._bound = False

    def _on_key_press(self, event) -> None:
        """Handle key press event."""
        key = event.keysym.lower()
        with self._lock:
            is_new_press = key not in self._keys_pressed
            self._keys_pressed.add(key)

        if not is_new_press or self._active_car_id is None:
            return

        if key in self.HOTKEY_ADJUSTMENTS:
            field, delta, label = self.HOTKEY_ADJUSTMENTS[key]
            self.adjust_profile_value(
                self._active_car_id,
                field,
                delta,
                reason=f"{label} {'+' if delta > 0 else '-'}",
            )

    def _on_key_release(self, event) -> None:
        """Handle key release event."""
        key = event.keysym.lower()
        with self._lock:
            self._keys_pressed.discard(key)

    @staticmethod
    def _normalize_vehicle_type(vehicle_type: Optional[str]) -> str:
        """Normalize vehicle type names used by the GUI."""
        if str(vehicle_type or "").strip().lower() == "limo":
            return "Limo"
        return "Qcar"

    @classmethod
    def _clamp_profile_value(cls, field: str, value: float) -> float:
        """Clamp a profile value to its valid range."""
        lower, upper = cls.PROFILE_LIMITS[field]
        clamped = max(lower, min(upper, value))
        if field.startswith("steering") or field.endswith("step"):
            return round(clamped, 3)
        return round(clamped, 3)

    def _build_vehicle_profile(self, vehicle_type: Optional[str]) -> KeyboardControlProfile:
        """Create a balanced default profile for a vehicle type."""
        normalized_type = self._normalize_vehicle_type(vehicle_type)
        if normalized_type == "Limo":
            return KeyboardControlProfile(
                vehicle_type="Limo",
                starting_forward_throttle=0.15,
                max_forward_throttle=0.30,
                max_reverse_throttle=0.18,
                throttle_step=0.02,
                steering_limit=0.40,
                steering_step=0.04,
            )

        return KeyboardControlProfile(
            vehicle_type="Qcar",
            starting_forward_throttle=0.15,
            max_forward_throttle=max(0.20, self.config.throttle_scale),
            max_reverse_throttle=0.18,
            throttle_step=0.02,
            steering_limit=max(0.40, self.config.steering_scale),
            steering_step=0.05,
        )

    def set_profile_callback(
        self, callback: Optional[Callable[[int, Dict[str, float], str], None]]
    ) -> None:
        """Register a callback for live keyboard profile updates."""
        self._profile_callback = callback

    def _notify_profile_change(
        self, car_id: int, profile: KeyboardControlProfile, reason: str
    ) -> None:
        """Notify the GUI that a profile changed."""
        if self._profile_callback:
            self._profile_callback(car_id, profile.to_dict(), reason)

    def ensure_profile(
        self, car_id: int, vehicle_type: Optional[str] = None
    ) -> KeyboardControlProfile:
        """Ensure a car has a keyboard profile."""
        with self._lock:
            profile = self._profiles.get(car_id)
            normalized_type = self._normalize_vehicle_type(vehicle_type)

            if profile is None:
                profile = self._build_vehicle_profile(normalized_type)
                self._profiles[car_id] = profile
            elif vehicle_type and profile.vehicle_type != normalized_type:
                profile.vehicle_type = normalized_type

            return KeyboardControlProfile(**profile.to_dict())

    def get_profile(
        self, car_id: int, vehicle_type: Optional[str] = None
    ) -> Dict[str, float]:
        """Get the current keyboard profile for a car."""
        return self.ensure_profile(car_id, vehicle_type).to_dict()

    def set_active_car(self, car_id: Optional[int], vehicle_type: Optional[str] = None) -> None:
        """Set the vehicle currently controlled by the keyboard."""
        if car_id != self._active_car_id:
            self._current_throttle = 0.0
            self._current_steering = 0.0
        self._active_car_id = car_id
        if car_id is None:
            return
        self.ensure_profile(car_id, vehicle_type)

    def update_profile(
        self,
        car_id: int,
        updates: Dict[str, float],
        vehicle_type: Optional[str] = None,
        reason: str = "Keyboard profile",
        notify: bool = True,
    ) -> Dict[str, float]:
        """Update one or more profile values for a specific car."""
        with self._lock:
            profile = self._profiles.get(car_id)
            normalized_type = self._normalize_vehicle_type(vehicle_type)
            if profile is None:
                profile = self._build_vehicle_profile(normalized_type)
                self._profiles[car_id] = profile
            elif vehicle_type:
                profile.vehicle_type = normalized_type

            for field, value in updates.items():
                if field in self.PROFILE_LIMITS:
                    setattr(profile, field, self._clamp_profile_value(field, value))

            result = KeyboardControlProfile(**profile.to_dict())

        if notify:
            self._notify_profile_change(car_id, result, reason)
        return result.to_dict()

    def adjust_profile_value(
        self,
        car_id: int,
        field: str,
        delta: float,
        vehicle_type: Optional[str] = None,
        reason: str = "Keyboard profile",
        notify: bool = True,
    ) -> Dict[str, float]:
        """Increment or decrement a single profile value."""
        current = self.ensure_profile(car_id, vehicle_type).to_dict()
        value = current.get(field)
        if value is None:
            return current
        return self.update_profile(
            car_id,
            {field: value + delta},
            vehicle_type=vehicle_type,
            reason=reason,
            notify=notify,
        )

    def get_control_state(self) -> ManualControlState:
        """Get current control state based on pressed keys."""
        cfg = self.config
        state = ManualControlState()

        with self._lock:
            keys = self._keys_pressed.copy()

        profile = self.ensure_profile(self._active_car_id or 0)
        starting_forward_throttle = min(
            profile.starting_forward_throttle, profile.max_forward_throttle
        )
        throttle_step = profile.throttle_step
        steering_step = profile.steering_step

        # Throttle control
        if cfg.forward_key in keys:
            if self._current_throttle <= 0.0:
                self._current_throttle = starting_forward_throttle
            else:
                self._current_throttle = min(
                    self._current_throttle + throttle_step,
                    profile.max_forward_throttle,
                )
        elif cfg.backward_key in keys:
            self._current_throttle = max(
                self._current_throttle - throttle_step,
                -profile.max_reverse_throttle,
            )
        elif self._current_throttle > 0.0:
            self._current_throttle = max(0.0, self._current_throttle - throttle_step)
        elif self._current_throttle < 0.0:
            self._current_throttle = min(0.0, self._current_throttle + throttle_step)

        # Steering control with configurable limit and return rate
        if cfg.left_key in keys:
            self._current_steering = min(
                self._current_steering + steering_step,
                profile.steering_limit,
            )
        elif cfg.right_key in keys:
            self._current_steering = max(
                self._current_steering - steering_step,
                -profile.steering_limit,
            )
        elif self._current_steering > 0.0:
            self._current_steering = max(0.0, self._current_steering - steering_step)
        elif self._current_steering < 0.0:
            self._current_steering = min(0.0, self._current_steering + steering_step)
        else:
            self._current_steering = 0.0

        state.throttle = self._current_throttle
        state.steering = self._current_steering

        # Emergency stop
        if cfg.stop_key in keys:
            state.throttle = 0.0
            state.steering = 0.0
            state.emergency_stop = True
            self._current_throttle = 0.0
            self._current_steering = 0.0

        return state

    def reset(self) -> None:
        """Reset controller state."""
        with self._lock:
            self._keys_pressed.clear()
        self._current_throttle = 0.0
        self._current_steering = 0.0

    @staticmethod
    def get_help_text() -> str:
        """Get help text for keyboard controls."""
        return (
            "Keyboard Controls:\n"
            "  Z/S = Throttle ramp up/down\n"
            "  Q/D = Steering ramp left/right\n"
            "  Space = Emergency stop\n"
            "  U/I = Forward max -/+ | O/P = Reverse max -/+\n"
            "  J/K = Steering limit -/+ | L/M = Steering step -/+\n"
            "  Note: Forward/Reverse are limits, not the live throttle value"
        )


class SteeringWheelController:
    """Handles steering wheel/gamepad input for manual vehicle control."""

    def __init__(self, config: ManualControlConfig = None):
        self.config = config or ManualControlConfig()
        self._wheel: Optional["pygame.joystick.Joystick"] = None
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
            return {"connected": False, "error": self._error_message}

        return {
            "connected": True,
            "name": self._wheel.get_name(),
            "axes": self._wheel.get_numaxes(),
            "buttons": self._wheel.get_numbuttons(),
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
        self._active_controller: str = "keyboard"
        self._running = False
        self._loop_thread: Optional[threading.Thread] = None
        self._callback: Optional[Callable[[int, float, float], None]] = None
        self._active_car_id: Optional[int] = None

    def set_keyboard_profile_callback(
        self, callback: Optional[Callable[[int, Dict[str, float], str], None]]
    ) -> None:
        """Register a callback for live keyboard profile changes."""
        self._keyboard.set_profile_callback(callback)

    def get_keyboard_profile(
        self, car_id: int, vehicle_type: Optional[str] = None
    ) -> Dict[str, float]:
        """Get the current keyboard profile for a specific car."""
        return self._keyboard.get_profile(car_id, vehicle_type)

    def update_keyboard_profile(
        self,
        car_id: int,
        updates: Dict[str, float],
        vehicle_type: Optional[str] = None,
        reason: str = "Keyboard profile",
        notify: bool = True,
    ) -> Dict[str, float]:
        """Update the keyboard profile for a specific car."""
        return self._keyboard.update_profile(
            car_id,
            updates,
            vehicle_type=vehicle_type,
            reason=reason,
            notify=notify,
        )

    def adjust_keyboard_profile(
        self,
        car_id: int,
        field: str,
        delta: float,
        vehicle_type: Optional[str] = None,
        reason: str = "Keyboard profile",
        notify: bool = True,
    ) -> Dict[str, float]:
        """Increment or decrement a keyboard profile value."""
        return self._keyboard.adjust_profile_value(
            car_id,
            field,
            delta,
            vehicle_type=vehicle_type,
            reason=reason,
            notify=notify,
        )

    def set_control_type(self, control_type: str) -> bool:
        """
        Set the active control type.

        Args:
            control_type: 'keyboard' or 'wheel'

        Returns:
            True if control type was set successfully
        """
        if control_type == "wheel":
            if not self._wheel.is_available:
                return False
            if not self._wheel.initialize():
                return False

        self._active_controller = control_type
        return True

    def bind_keyboard(self, widget) -> None:
        """Bind keyboard to a widget."""
        self._keyboard.bind_to_widget(widget)

    def start(
        self,
        car_id: int,
        callback: Callable[[int, float, float], None],
        vehicle_type: Optional[str] = None,
        interval_ms: Optional[int] = None,
    ) -> None:
        """
        Start the manual control loop.

        Args:
            car_id: ID of the car to control
            callback: Function to call with (car_id, throttle, steering)
            vehicle_type: Vehicle type used to seed keyboard defaults
            interval_ms: Update interval in milliseconds. Defaults to the
                configured manual-control update interval.
        """
        if self._running:
            self.stop()

        self._active_car_id = car_id
        self._callback = callback
        self._running = True
        self._keyboard.set_active_car(car_id, vehicle_type)

        def control_loop():
            import time

            configured_interval_ms = (
                interval_ms
                if interval_ms is not None
                else max(10, int(self.config.update_interval_ms))
            )
            interval = configured_interval_ms / 1000.0

            while self._running and self._active_car_id is not None:
                state = self.get_control_state()

                if self._callback and self._active_car_id is not None:
                    self._callback(self._active_car_id, state.throttle, state.steering)

                time.sleep(interval)

        self._loop_thread = threading.Thread(target=control_loop, daemon=True)
        self._loop_thread.start()

    def stop(self) -> None:
        """Stop the manual control loop."""
        self._running = False
        self._active_car_id = None
        self._keyboard.set_active_car(None)
        self._keyboard.reset()
        if self._loop_thread:
            self._loop_thread.join(timeout=0.5)
            self._loop_thread = None

    def get_control_state(self) -> ManualControlState:
        """Get current control state from the active controller."""
        if self._active_controller == "wheel":
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
        if self._active_controller == "wheel":
            return self._wheel.get_help_text()
        return self._keyboard.get_help_text()
