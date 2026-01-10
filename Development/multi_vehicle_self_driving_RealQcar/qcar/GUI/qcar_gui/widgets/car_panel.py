"""
Car Panel Widget for QCar Fleet Controller.

This module contains the CarPanelWidget class that displays
individual vehicle controls, telemetry, and status information.
"""

import tkinter as tk
from tkinter import ttk
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass, field

from ..theme import Theme, DEFAULT_THEME
from ..config import TELEMETRY_FIELDS, DEFAULT_PATHS, VehicleConfig
from .base import (
    BaseWidget, ThemedButton, ThemedEntry, ThemedLabel,
    ThemedLabelFrame, ExpandablePanel, StatusIndicator
)


@dataclass
class CarState:
    """Data class holding current state of a car."""
    car_id: int
    connected: bool = False
    state: str = 'Unknown'
    position: tuple = (0.0, 0.0)
    velocity: float = 0.0
    heading: float = 0.0
    throttle: float = 0.0
    steering: float = 0.0
    v2v_active: bool = False
    v2v_peers: int = 0
    platoon_enabled: bool = False
    platoon_is_leader: bool = False
    platoon_position: Optional[int] = None
    manual_mode: bool = False


@dataclass
class CarPanelCallbacks:
    """Callbacks for car panel actions."""
    on_start: Callable[[int], None] = None
    on_stop: Callable[[int], None] = None
    on_calibrate: Callable[[int], None] = None
    on_set_velocity: Callable[[int, float], None] = None
    on_set_path: Callable[[int, list], None] = None
    on_set_initial_position: Callable[[int, float, float, float, bool], None] = None
    on_toggle_manual: Callable[[int], None] = None
    on_update_control_type: Callable[[int, str], None] = None
    on_platoon_position_change: Callable[[int, int], None] = None


class TelemetryDisplay(BaseWidget):
    """Widget for displaying vehicle telemetry data."""
    
    def __init__(self, parent: tk.Widget, theme: Theme = None):
        self._labels: Dict[str, tk.Label] = {}
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the telemetry display."""
        c = self.theme.colors
        
        self.frame = ThemedLabelFrame(
            self.parent,
            text="📊 Telemetry",
            theme=self.theme
        )
        
        grid = tk.Frame(self.frame, bg=c.bg_medium)
        grid.pack(fill='x', padx=6, pady=4)
        
        for i, (key, label_text, default_value, width) in enumerate(TELEMETRY_FIELDS):
            row, col = i // 2, i % 2
            
            label_frame = tk.Frame(grid, bg=c.bg_medium)
            label_frame.grid(row=row, column=col, padx=4, pady=2, sticky='w')
            
            ThemedLabel(
                label_frame,
                text=label_text,
                style='muted',
                theme=self.theme
            ).pack(side='left')
            
            value_label = tk.Label(
                label_frame,
                text=default_value,
                bg=c.bg_medium,
                fg=c.fg_primary,
                font=self.theme.fonts.small_bold(),
                width=width,
                anchor='w'
            )
            value_label.pack(side='left', padx=(5, 0))
            
            self._labels[key] = value_label
    
    def update(self, state: CarState) -> None:
        """Update telemetry display with current state."""
        if 'position' in self._labels:
            x, y = state.position
            self._labels['position'].config(text=f"({x:.2f}, {y:.2f})")
        
        if 'velocity' in self._labels:
            self._labels['velocity'].config(text=f"{state.velocity:.2f}")
        
        if 'heading' in self._labels:
            self._labels['heading'].config(text=f"{state.heading:.2f}")
        
        if 'throttle' in self._labels:
            self._labels['throttle'].config(text=f"{state.throttle:.2f}")
        
        if 'steering' in self._labels:
            self._labels['steering'].config(text=f"{state.steering:.2f}")
        
        if 'state' in self._labels:
            self._labels['state'].config(text=state.state)


class ControlButtons(BaseWidget):
    """Widget for vehicle control buttons (Start, Stop, Calibrate)."""
    
    def __init__(self, parent: tk.Widget, car_id: int,
                 callbacks: CarPanelCallbacks,
                 theme: Theme = None):
        self.car_id = car_id
        self.callbacks = callbacks
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the control buttons."""
        c = self.theme.colors
        
        self.frame = tk.Frame(self.parent, bg=c.bg_medium)
        
        # Start button
        ThemedButton(
            self.frame,
            text="▶ START",
            button_type='start',
            command=lambda: self._safe_callback(self.callbacks.on_start),
            padx=12,
            pady=4
        ).pack(side='left', expand=True, fill='x', padx=(0, 3))
        
        # Calibrate button
        ThemedButton(
            self.frame,
            text="🔧 Calibrate",
            button_type='warning',
            command=lambda: self._safe_callback(self.callbacks.on_calibrate),
            padx=10,
            pady=4
        ).pack(side='left', expand=True, fill='x', padx=(3, 3))
        
        # Stop button
        ThemedButton(
            self.frame,
            text="⬛ STOP",
            button_type='stop',
            command=lambda: self._safe_callback(self.callbacks.on_stop),
            padx=12,
            pady=4
        ).pack(side='left', expand=True, fill='x', padx=(3, 0))
    
    def _safe_callback(self, callback: Callable) -> None:
        """Safely call a callback if it exists."""
        if callback:
            callback(self.car_id)


class ManualControlPanel(BaseWidget):
    """Widget for manual control mode settings."""
    
    def __init__(self, parent: tk.Widget, car_id: int,
                 callbacks: CarPanelCallbacks,
                 theme: Theme = None):
        self.car_id = car_id
        self.callbacks = callbacks
        self._control_type_var: Optional[tk.StringVar] = None
        self._manual_btn: Optional[tk.Button] = None
        self._manual_active = False
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the manual control panel."""
        c = self.theme.colors
        
        self.frame = ThemedLabelFrame(
            self.parent,
            text="🎮 Manual Control",
            theme=self.theme
        )
        
        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill='x', padx=6, pady=4)
        
        # Control type selection
        control_type_frame = tk.Frame(content, bg=c.bg_medium)
        control_type_frame.pack(fill='x', pady=(0, 3))
        
        ThemedLabel(
            control_type_frame,
            text="Control:",
            style='muted',
            theme=self.theme
        ).pack(side='left', padx=(0, 8))
        
        self._control_type_var = tk.StringVar(value='keyboard')
        
        # Radio buttons
        for text, value in [("⌨️ Keyboard", 'keyboard'), ("🎡 Wheel", 'wheel')]:
            tk.Radiobutton(
                control_type_frame,
                text=text,
                variable=self._control_type_var,
                value=value,
                bg=c.bg_medium,
                fg=c.fg_primary,
                selectcolor=c.bg_light,
                font=self.theme.fonts.tiny(),
                command=lambda v=value: self._on_control_type_change(v)
            ).pack(side='left', padx=(0, 10))
        
        # Manual mode toggle button
        self._manual_btn = ThemedButton(
            content,
            text="🎮 Manual Mode",
            button_type='platoon',
            command=self._toggle_manual,
            padx=10,
            pady=3
        )
        self._manual_btn.pack(fill='x', pady=(3, 0))
    
    def _on_control_type_change(self, control_type: str) -> None:
        """Handle control type change."""
        if self.callbacks.on_update_control_type:
            self.callbacks.on_update_control_type(self.car_id, control_type)
    
    def _toggle_manual(self) -> None:
        """Toggle manual mode."""
        if self.callbacks.on_toggle_manual:
            self.callbacks.on_toggle_manual(self.car_id)
    
    def set_manual_active(self, active: bool) -> None:
        """Update manual mode button state."""
        self._manual_active = active
        if self._manual_btn:
            if active:
                self._manual_btn.config(
                    text="🎮 Manual: ON",
                    bg=self.theme.colors.accent_green
                )
            else:
                self._manual_btn.config(
                    text="🎮 Manual Mode",
                    bg=self.theme.colors.accent_purple
                )
    
    @property
    def control_type(self) -> str:
        """Get the selected control type."""
        return self._control_type_var.get() if self._control_type_var else 'keyboard'


class VelocityControl(BaseWidget):
    """Widget for velocity control."""
    
    def __init__(self, parent: tk.Widget, car_id: int,
                 callbacks: CarPanelCallbacks,
                 config: VehicleConfig = None,
                 theme: Theme = None):
        self.car_id = car_id
        self.callbacks = callbacks
        self.config = config or VehicleConfig()
        self._entry: Optional[ThemedEntry] = None
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the velocity control."""
        c = self.theme.colors
        
        self.frame = ThemedLabelFrame(
            self.parent,
            text="🎯 Velocity",
            theme=self.theme
        )
        
        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill='x', padx=6, pady=4)
        
        ThemedLabel(
            content,
            text="Target:",
            style='muted',
            theme=self.theme
        ).pack(side='left', padx=(0, 5))
        
        self._entry = ThemedEntry(content, width=8, theme=self.theme)
        self._entry.insert(0, "1.0")
        self._entry.pack(side='left', padx=(0, 5))
        
        ThemedButton(
            content,
            text="Set",
            button_type='command',
            command=self._set_velocity,
            padx=8,
            pady=3
        ).pack(side='left')
    
    def _set_velocity(self) -> None:
        """Handle set velocity button click."""
        if self.callbacks.on_set_velocity and self._entry:
            velocity = self._entry.get_float()
            if self.config.min_velocity <= velocity <= self.config.max_velocity:
                self.callbacks.on_set_velocity(self.car_id, velocity)


class PathControl(BaseWidget):
    """Widget for path and initial position control."""
    
    def __init__(self, parent: tk.Widget, car_id: int,
                 callbacks: CarPanelCallbacks,
                 theme: Theme = None):
        self.car_id = car_id
        self.callbacks = callbacks
        self._path_entry: Optional[ThemedEntry] = None
        self._x_entry: Optional[ThemedEntry] = None
        self._y_entry: Optional[ThemedEntry] = None
        self._theta_entry: Optional[ThemedEntry] = None
        self._calibrate_var: Optional[tk.BooleanVar] = None
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the path control."""
        c = self.theme.colors
        
        self.frame = ThemedLabelFrame(
            self.parent,
            text="🛤️ Path & Position",
            theme=self.theme
        )
        
        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill='x', padx=8, pady=6)
        
        # Initial position row
        self._build_position_row(content)
        
        # Path row
        self._build_path_row(content)
    
    def _build_position_row(self, parent: tk.Frame) -> None:
        """Build the initial position row."""
        c = self.theme.colors
        
        row = tk.Frame(parent, bg=c.bg_medium)
        row.pack(fill='x', pady=(0, 3))
        
        ThemedLabel(row, text="Init Pos:", style='muted', theme=self.theme).pack(side='left', padx=(0, 4))
        
        # X, Y, Theta entries
        for label, attr_name in [("X:", '_x_entry'), ("Y:", '_y_entry'), ("θ:", '_theta_entry')]:
            ThemedLabel(row, text=label, style='muted', theme=self.theme).pack(side='left', padx=(0, 2))
            entry = ThemedEntry(row, width=5, theme=self.theme)
            entry.insert(0, "0.0")
            entry.pack(side='left', padx=(0, 3))
            setattr(self, attr_name, entry)
        
        # Calibrate checkbox
        self._calibrate_var = tk.BooleanVar(value=False)
        tk.Checkbutton(
            row,
            text="Calibrate GPS",
            variable=self._calibrate_var,
            bg=c.bg_medium,
            fg=c.accent_green,
            selectcolor=c.bg_dark,
            activebackground=c.bg_medium,
            activeforeground=c.accent_green,
            font=self.theme.fonts.tiny()
        ).pack(side='left', padx=(0, 3))
        
        # Set button
        ThemedButton(
            row,
            text="Set",
            button_type='command',
            command=self._set_initial_position,
            padx=10,
            pady=3
        ).pack(side='left')
    
    def _build_path_row(self, parent: tk.Frame) -> None:
        """Build the path nodes row."""
        c = self.theme.colors
        
        row = tk.Frame(parent, bg=c.bg_medium)
        row.pack(fill='x')
        
        ThemedLabel(row, text="Nodes:", style='muted', theme=self.theme).pack(side='left', padx=(0, 5))
        
        self._path_entry = ThemedEntry(row, width=15, theme=self.theme)
        default_path = DEFAULT_PATHS.get(self.car_id, DEFAULT_PATHS['default'])
        self._path_entry.insert(0, default_path)
        self._path_entry.pack(side='left', padx=(0, 5))
        
        ThemedButton(
            row,
            text="Set",
            button_type='command',
            command=self._set_path,
            padx=8,
            pady=3
        ).pack(side='left')
    
    def _set_initial_position(self) -> None:
        """Handle set initial position."""
        if self.callbacks.on_set_initial_position:
            x = self._x_entry.get_float()
            y = self._y_entry.get_float()
            theta = self._theta_entry.get_float()
            calibrate = self._calibrate_var.get()
            self.callbacks.on_set_initial_position(self.car_id, x, y, theta, calibrate)
    
    def _set_path(self) -> None:
        """Handle set path."""
        if self.callbacks.on_set_path and self._path_entry:
            nodes = self._path_entry.get_list(separator=',', item_type=int)
            if len(nodes) >= 2:
                self.callbacks.on_set_path(self.car_id, nodes)


class PlatoonControl(BaseWidget):
    """Widget for platoon configuration."""
    
    def __init__(self, parent: tk.Widget, car_id: int,
                 callbacks: CarPanelCallbacks,
                 theme: Theme = None):
        self.car_id = car_id
        self.callbacks = callbacks
        self._position_var: Optional[tk.StringVar] = None
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the platoon control."""
        c = self.theme.colors
        
        self.frame = ThemedLabelFrame(
            self.parent,
            text="🚗 Platoon",
            theme=self.theme
        )
        
        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill='x', padx=6, pady=4)
        
        # Position selection
        position_frame = tk.Frame(content, bg=c.bg_medium)
        position_frame.pack(fill='x', pady=(0, 3))
        
        ThemedLabel(
            position_frame,
            text="Position:",
            style='muted',
            theme=self.theme
        ).pack(side='left', padx=(0, 5))
        
        self._position_var = tk.StringVar(value=str(self.car_id + 1))
        position_entry = ThemedEntry(
            position_frame,
            width=3,
            theme=self.theme
        )
        position_entry.insert(0, str(self.car_id + 1))
        position_entry.pack(side='left', padx=(0, 5))
        position_entry.bind('<KeyRelease>', self._on_position_change)
        self._position_entry = position_entry
        
        # Role indicator
        ThemedLabel(
            position_frame,
            text="(1=Leader, 2,3...=Followers)",
            style='muted',
            theme=self.theme
        ).pack(side='left', padx=(5, 0))
    
    def _on_position_change(self, event=None) -> None:
        """Handle position change."""
        if self.callbacks.on_platoon_position_change:
            try:
                position = int(self._position_entry.get())
                self.callbacks.on_platoon_position_change(self.car_id, position)
            except ValueError:
                pass
    
    @property
    def position(self) -> int:
        """Get the current platoon position."""
        try:
            return int(self._position_entry.get())
        except (ValueError, AttributeError):
            return self.car_id + 1


class CarPanelWidget(BaseWidget):
    """Complete car control panel widget."""
    
    def __init__(self, parent: tk.Widget, car_id: int,
                 callbacks: CarPanelCallbacks = None,
                 expanded: bool = True,
                 theme: Theme = None):
        self.car_id = car_id
        self.callbacks = callbacks or CarPanelCallbacks()
        self._expanded = expanded
        
        # Child widgets
        self._expandable: Optional[ExpandablePanel] = None
        self._telemetry: Optional[TelemetryDisplay] = None
        self._control_buttons: Optional[ControlButtons] = None
        self._manual_control: Optional[ManualControlPanel] = None
        self._velocity_control: Optional[VelocityControl] = None
        self._path_control: Optional[PathControl] = None
        self._platoon_control: Optional[PlatoonControl] = None
        
        # Status indicators
        self._conn_indicator: Optional[StatusIndicator] = None
        self._v2v_indicator: Optional[tk.Label] = None
        self._platoon_indicator: Optional[tk.Label] = None
        self._state_label: Optional[tk.Label] = None
        
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the car panel widget."""
        c = self.theme.colors
        
        # Create expandable panel
        self._expandable = ExpandablePanel(
            self.parent,
            title=f"🚗 Car {self.car_id}",
            expanded=self._expanded,
            theme=self.theme
        )
        self.frame = self._expandable.frame
        
        # Add state label to header
        self._build_header_indicators()
        
        # Build content sections
        self._build_content()
    
    def _build_header_indicators(self) -> None:
        """Build header status indicators."""
        c = self.theme.colors
        header = self._expandable.header
        
        # Connection indicator
        self._conn_indicator = StatusIndicator(
            header,
            status='connected',
            theme=self.theme,
            font=self.theme.fonts.heading(),
            padx=12
        )
        self._conn_indicator.pack(side='right', padx=12, pady=12)
        
        # V2V indicator
        self._v2v_indicator = tk.Label(
            header,
            text="📡 V2V: OFF",
            bg=c.bg_header,
            fg=c.fg_muted,
            font=self.theme.fonts.small(),
            padx=8
        )
        self._v2v_indicator.pack(side='right', padx=(0, 6), pady=12)
        
        # Platoon indicator
        self._platoon_indicator = tk.Label(
            header,
            text="🚗 Solo",
            bg=c.bg_header,
            fg=c.fg_muted,
            font=self.theme.fonts.small(),
            padx=8
        )
        self._platoon_indicator.pack(side='right', padx=(0, 6), pady=12)
        
        # Add state label under title in header
        title_frame = tk.Frame(self._expandable.header, bg=c.bg_header)
        title_frame.pack(side='left', fill='y', expand=True, padx=10)
        
        self._state_label = tk.Label(
            title_frame,
            text="State: Unknown",
            bg=c.bg_header,
            fg=c.fg_muted,
            font=self.theme.fonts.small(),
            cursor='hand2'
        )
        self._state_label.pack(anchor='w')
    
    def _build_content(self) -> None:
        """Build the content area."""
        c = self.theme.colors
        content = self._expandable.content
        
        # Main layout - left and right sections
        main_layout = tk.Frame(content, bg=c.bg_medium)
        main_layout.pack(fill='both', expand=True, pady=(0, 6))
        
        # Left section: Telemetry + Control buttons + Manual control
        left_section = tk.Frame(main_layout, bg=c.bg_medium)
        left_section.pack(side='left', fill='both', expand=True, padx=(0, 5))
        
        self._telemetry = TelemetryDisplay(left_section, theme=self.theme)
        self._telemetry.pack(fill='x', pady=(0, 5))
        
        self._control_buttons = ControlButtons(
            left_section,
            self.car_id,
            self.callbacks,
            theme=self.theme
        )
        self._control_buttons.pack(fill='x', pady=(3, 0))
        
        self._manual_control = ManualControlPanel(
            left_section,
            self.car_id,
            self.callbacks,
            theme=self.theme
        )
        self._manual_control.pack(fill='x', pady=(5, 0))
        
        # Right section: Velocity + Path + Platoon controls
        right_section = tk.Frame(main_layout, bg=c.bg_medium)
        right_section.pack(side='right', fill='both', expand=True, padx=(5, 0))
        
        self._velocity_control = VelocityControl(
            right_section,
            self.car_id,
            self.callbacks,
            theme=self.theme
        )
        self._velocity_control.pack(fill='x', pady=(0, 4))
        
        self._path_control = PathControl(
            right_section,
            self.car_id,
            self.callbacks,
            theme=self.theme
        )
        self._path_control.pack(fill='x', pady=(0, 4))
        
        self._platoon_control = PlatoonControl(
            right_section,
            self.car_id,
            self.callbacks,
            theme=self.theme
        )
        self._platoon_control.pack(fill='x')
    
    def update_state(self, state: CarState) -> None:
        """Update the panel with current car state."""
        # Update telemetry
        if self._telemetry:
            self._telemetry.update(state)
        
        # Update state label
        if self._state_label:
            self._state_label.config(text=f"State: {state.state}")
        
        # Update V2V indicator
        if self._v2v_indicator:
            if state.v2v_active and state.v2v_peers > 0:
                self._v2v_indicator.config(
                    text=f"📡 V2V: ON ({state.v2v_peers})",
                    fg=self.theme.colors.accent_green
                )
            else:
                self._v2v_indicator.config(
                    text="📡 V2V: OFF",
                    fg=self.theme.colors.fg_muted
                )
        
        # Update platoon indicator
        if self._platoon_indicator:
            if state.platoon_enabled and state.platoon_position is not None:
                if state.platoon_is_leader:
                    self._platoon_indicator.config(
                        text="🚗 LEADER",
                        fg=self.theme.colors.platoon_leader
                    )
                else:
                    self._platoon_indicator.config(
                        text=f"🚗 FOLLOWER-{state.platoon_position}",
                        fg=self.theme.colors.platoon_follower
                    )
            else:
                self._platoon_indicator.config(
                    text="🚗 Solo",
                    fg=self.theme.colors.fg_muted
                )
        
        # Update manual mode
        if self._manual_control:
            self._manual_control.set_manual_active(state.manual_mode)
    
    def set_connected(self, connected: bool) -> None:
        """Update connection status."""
        if self._conn_indicator:
            self._conn_indicator.set_status(
                'connected' if connected else 'disconnected'
            )
    
    @property
    def platoon_position(self) -> int:
        """Get the configured platoon position."""
        return self._platoon_control.position if self._platoon_control else self.car_id + 1
    
    @property
    def control_type(self) -> str:
        """Get the selected manual control type."""
        return self._manual_control.control_type if self._manual_control else 'keyboard'
