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
    perception_active: bool = False
    scopes_active: bool = False
    # Observer and Controller types
    local_observer_type: str = 'unknown'
    fleet_observer_type: str = 'unknown'
    longitudinal_ctrl_type: str = 'unknown'
    lateral_ctrl_type: str = 'unknown'


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
    on_toggle_perception: Callable[[int], None] = None
    on_toggle_scopes: Callable[[int], None] = None
    on_toggle_remote_plot_local: Callable[[int], None] = None
    on_toggle_remote_plot_fleet: Callable[[int], None] = None
    # Runtime switching callbacks
    on_set_local_observer: Callable[[int, str], None] = None
    on_set_fleet_observer: Callable[[int, str], None] = None
    on_set_controller: Callable[[int, str, str], None] = None  # car_id, category, type


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
            
        if 'longitudinal_ctrl_type' in self._labels:
            self._labels['longitudinal_ctrl_type'].config(text=state.longitudinal_ctrl_type)
            
        if 'lateral_ctrl_type' in self._labels:
            self._labels['lateral_ctrl_type'].config(text=state.lateral_ctrl_type)


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


class PerceptionControl(BaseWidget):
    """Widget for perception system (YOLO) control."""
    
    def __init__(self, parent: tk.Widget, car_id: int,
                 callbacks: CarPanelCallbacks,
                 theme: Theme = None):
        self.car_id = car_id
        self.callbacks = callbacks
        self._perception_btn: Optional[tk.Button] = None
        self._perception_active = False
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the perception control panel."""
        c = self.theme.colors
        
        self.frame = ThemedLabelFrame(
            self.parent,
            text="👁️ Perception",
            theme=self.theme
        )
        
        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill='x', padx=6, pady=4)
        
        # Perception toggle button
        self._perception_btn = ThemedButton(
            content,
            text="👁️ Activate YOLO",
            button_type='command',
            command=self._toggle_perception,
            padx=10,
            pady=3
        )
        self._perception_btn.pack(fill='x')
    
    def _toggle_perception(self) -> None:
        """Toggle perception system."""
        if self.callbacks.on_toggle_perception:
            self.callbacks.on_toggle_perception(self.car_id)
    
    def set_perception_active(self, active: bool) -> None:
        """Update perception button state."""
        self._perception_active = active
        if self._perception_btn:
            if active:
                self._perception_btn.config(
                    text="👁️ YOLO: ON",
                    bg=self.theme.colors.accent_green
                )
            else:
                self._perception_btn.config(
                    text="👁️ Activate YOLO",
                    bg=self.theme.colors.accent_blue
                )
    
    @property
    def is_active(self) -> bool:
        """Get whether perception is active."""
        return self._perception_active


class ScopesControl(BaseWidget):
    """Widget for estimation scopes and remote plotting control."""
    
    def __init__(self, parent: tk.Widget, car_id: int,
                 callbacks: CarPanelCallbacks,
                 theme: Theme = None):
        self.car_id = car_id
        self.callbacks = callbacks
        self._scopes_btn: Optional[tk.Button] = None
        self._remote_plot_btn: Optional[tk.Button] = None
        self._scopes_active = False
        self._remote_plot_active = False
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the scopes control panel."""
        c = self.theme.colors
        
        self.frame = ThemedLabelFrame(
            self.parent,
            text="📊 Scopes",
            theme=self.theme
        )
        
        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill='x', padx=6, pady=4)
        
        # Remote plot Local button (streaming local data to Ground Station)
        self._remote_local_btn = ThemedButton(
            content,
            text="📡 Local Plot",
            button_type='command',
            command=self._toggle_remote_local,
            padx=10,
            pady=3
        )
        self._remote_local_btn.pack(fill='x')
        
        # Remote plot Fleet button (streaming fleet data to Ground Station)
        self._remote_fleet_btn = ThemedButton(
            content,
            text="📡 Fleet Plot",
            button_type='command',
            command=self._toggle_remote_fleet,
            padx=10,
            pady=3
        )
        self._remote_fleet_btn.pack(fill='x', pady=(3, 0))
        
        # Fleet button starts disabled (needs V2V active)
        self._remote_fleet_btn.config(state='disabled')
        self._fleet_enabled = False
    
    def _toggle_scopes(self) -> None:
        """Toggle estimation scopes on vehicle."""
        if self.callbacks.on_toggle_scopes:
            self.callbacks.on_toggle_scopes(self.car_id)
    
    def _toggle_remote_local(self) -> None:
        """Toggle remote local scope streaming to Ground Station."""
        if self.callbacks.on_toggle_remote_plot_local:
            self.callbacks.on_toggle_remote_plot_local(self.car_id)
    
    def _toggle_remote_fleet(self) -> None:
        """Toggle remote fleet scope streaming to Ground Station."""
        if self.callbacks.on_toggle_remote_plot_fleet:
            self.callbacks.on_toggle_remote_plot_fleet(self.car_id)
    
    def set_scopes_active(self, active: bool) -> None:
        """Update scopes button state."""
        self._scopes_active = active
        if hasattr(self, '_scopes_btn') and self._scopes_btn:
            if active:
                self._scopes_btn.config(
                    text="📊 Local: ON",
                    bg=self.theme.colors.accent_green
                )
            else:
                self._scopes_btn.config(
                    text="📊 Local Plots",
                    bg=self.theme.colors.accent_blue
                )
    
    def set_remote_local_active(self, active: bool) -> None:
        """Update remote local plot button state."""
        self._remote_local_active = active
        if self._remote_local_btn:
            if active:
                self._remote_local_btn.config(
                    text="📡 Local: ON",
                    bg=self.theme.colors.accent_green
                )
            else:
                self._remote_local_btn.config(
                    text="📡 Local Plot",
                    bg=self.theme.colors.accent_blue
                )
    
    def set_remote_fleet_active(self, active: bool) -> None:
        """Update remote fleet plot button state."""
        self._remote_fleet_active = active
        if self._remote_fleet_btn:
            if active:
                self._remote_fleet_btn.config(
                    text="📡 Fleet: ON",
                    bg=self.theme.colors.accent_green
                )
            else:
                self._remote_fleet_btn.config(
                    text="📡 Fleet Plot",
                    bg=self.theme.colors.accent_blue
                )
    
    def set_fleet_button_enabled(self, enabled: bool) -> None:
        """Enable/disable Fleet plot button based on V2V status."""
        self._fleet_enabled = enabled
        if self._remote_fleet_btn:
            self._remote_fleet_btn.config(
                state='normal' if enabled else 'disabled'
            )
    
    @property
    def is_active(self) -> bool:
        """Get whether scopes are active."""
        return self._scopes_active
    
    @property
    def is_remote_local_active(self) -> bool:
        """Get whether remote local plot is active."""
        return getattr(self, '_remote_local_active', False)
    
    @property
    def is_remote_fleet_active(self) -> bool:
        """Get whether remote fleet plot is active."""
        return getattr(self, '_remote_fleet_active', False)


class RuntimeSwitchingControl(BaseWidget):
    """Widget for runtime observer and controller switching."""
    
    # Available options (matching config files)
    LOCAL_OBSERVERS = ['ekf', 'luenberger', 'neural_luenberger']
    FLEET_OBSERVERS = ['consensus', 'distributed_luenberger', 
                       'trust_consensus', 'trust_kalman']
    LONGITUDINAL_CONTROLLERS = ['cacc', 'pid', 'hybrid']
    LATERAL_CONTROLLERS = ['pure_pursuit', 'stanley', 'lookahead', 'hybrid', 'fusion', 'path']
    
    def __init__(self, parent: tk.Widget, car_id: int,
                 callbacks: CarPanelCallbacks,
                 theme: Theme = None):
        self.car_id = car_id
        self.callbacks = callbacks
        self._local_obs_var: Optional[tk.StringVar] = None
        self._fleet_obs_var: Optional[tk.StringVar] = None
        self._long_ctrl_var: Optional[tk.StringVar] = None
        self._lat_ctrl_var: Optional[tk.StringVar] = None
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the runtime switching control panel."""
        c = self.theme.colors
        
        self.frame = ThemedLabelFrame(
            self.parent,
            text="⚙️ Runtime Config",
            theme=self.theme
        )
        
        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill='x', padx=6, pady=4)
        
        # Local Observer row
        self._build_dropdown_row(
            content, "Local Obs:", self.LOCAL_OBSERVERS,
            '_local_obs_var', self._apply_local_observer
        )
        
        # Fleet Observer row
        self._build_dropdown_row(
            content, "Fleet Obs:", self.FLEET_OBSERVERS,
            '_fleet_obs_var', self._apply_fleet_observer
        )
        
        # Longitudinal Controller row
        self._build_dropdown_row(
            content, "Long Ctrl:", self.LONGITUDINAL_CONTROLLERS,
            '_long_ctrl_var', self._apply_longitudinal_controller
        )
        
        # Lateral Controller row
        self._build_dropdown_row(
            content, "Lat Ctrl:", self.LATERAL_CONTROLLERS,
            '_lat_ctrl_var', self._apply_lateral_controller
        )
    
    def _build_dropdown_row(self, parent: tk.Frame, label_text: str, 
                            options: list, var_attr: str, 
                            apply_callback: Callable) -> None:
        """Build a dropdown row with label and apply button."""
        c = self.theme.colors
        
        row = tk.Frame(parent, bg=c.bg_medium)
        row.pack(fill='x', pady=2)
        
        # Label
        ThemedLabel(
            row, text=label_text, style='muted', theme=self.theme
        ).pack(side='left', padx=(0, 5))
        
        # Dropdown variable
        var = tk.StringVar(value=options[0] if options else '')
        setattr(self, var_attr, var)
        
        # Dropdown (OptionMenu)
        dropdown = tk.OptionMenu(row, var, *options)
        dropdown.config(
            bg=c.bg_light,
            fg=c.fg_primary,
            activebackground=c.bg_medium,
            activeforeground=c.fg_primary,
            highlightthickness=0,
            font=self.theme.fonts.tiny(),
            width=12
        )
        dropdown["menu"].config(
            bg=c.bg_light,
            fg=c.fg_primary,
            activebackground=c.accent_blue,
            activeforeground=c.fg_primary
        )
        dropdown.pack(side='left', padx=(0, 5))
        
        # Apply button
        ThemedButton(
            row,
            text="Apply",
            button_type='command',
            command=apply_callback,
            padx=6,
            pady=1
        ).pack(side='left')
    
    def _apply_local_observer(self) -> None:
        """Apply local observer change."""
        if self.callbacks.on_set_local_observer and self._local_obs_var:
            observer_type = self._local_obs_var.get()
            self.callbacks.on_set_local_observer(self.car_id, observer_type)
    
    def _apply_fleet_observer(self) -> None:
        """Apply fleet observer change."""
        if self.callbacks.on_set_fleet_observer and self._fleet_obs_var:
            observer_type = self._fleet_obs_var.get()
            self.callbacks.on_set_fleet_observer(self.car_id, observer_type)
    
    def _apply_longitudinal_controller(self) -> None:
        """Apply longitudinal controller change."""
        if self.callbacks.on_set_controller and self._long_ctrl_var:
            controller_type = self._long_ctrl_var.get()
            self.callbacks.on_set_controller(self.car_id, 'longitudinal', controller_type)
    
    def _apply_lateral_controller(self) -> None:
        """Apply lateral controller change."""
        if self.callbacks.on_set_controller and self._lat_ctrl_var:
            controller_type = self._lat_ctrl_var.get()
            self.callbacks.on_set_controller(self.car_id, 'lateral', controller_type)
    
    def set_current_values(self, local_obs: str = None, fleet_obs: str = None,
                           long_ctrl: str = None, lat_ctrl: str = None) -> None:
        """Set the current values in the dropdowns (useful for syncing with telemetry)."""
        if local_obs and self._local_obs_var and local_obs in self.LOCAL_OBSERVERS:
            self._local_obs_var.set(local_obs)
        if fleet_obs and self._fleet_obs_var and fleet_obs in self.FLEET_OBSERVERS:
            self._fleet_obs_var.set(fleet_obs)
        if long_ctrl and self._long_ctrl_var and long_ctrl in self.LONGITUDINAL_CONTROLLERS:
            self._long_ctrl_var.set(long_ctrl)
        if lat_ctrl and self._lat_ctrl_var and lat_ctrl in self.LATERAL_CONTROLLERS:
            self._lat_ctrl_var.set(lat_ctrl)


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
        self._perception_control: Optional[PerceptionControl] = None
        self._scopes_control: Optional[ScopesControl] = None
        self._runtime_switching: Optional[RuntimeSwitchingControl] = None
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
        
        self._perception_control = PerceptionControl(
            left_section,
            self.car_id,
            self.callbacks,
            theme=self.theme
        )
        self._perception_control.pack(fill='x', pady=(5, 0))
        
        # Scopes control (estimation visualization)
        self._scopes_control = ScopesControl(
            left_section,
            self.car_id,
            self.callbacks,
            theme=self.theme
        )
        self._scopes_control.pack(fill='x', pady=(5, 0))
        
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
        self._platoon_control.pack(fill='x', pady=(0, 4))
        
        # Runtime switching control (observer/controller selection)
        self._runtime_switching = RuntimeSwitchingControl(
            right_section,
            self.car_id,
            self.callbacks,
            theme=self.theme
        )
        self._runtime_switching.pack(fill='x')
    
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
        
        # Update perception status
        if self._perception_control:
            self._perception_control.set_perception_active(state.perception_active)
        
        # Update scopes status
        if self._scopes_control:
            self._scopes_control.set_scopes_active(state.scopes_active)
        
        # # Sync runtime switching dropdowns with current values from vehicle
        # if self._runtime_switching:
        #     self._runtime_switching.set_current_values(
        #         local_obs=state.local_observer_type,
        #         fleet_obs=state.fleet_observer_type,
        #         long_ctrl=state.longitudinal_ctrl_type,
        #         lat_ctrl=state.lateral_ctrl_type
        #     )
    
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
