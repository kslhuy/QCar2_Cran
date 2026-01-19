"""
Fleet Controls Widget for QCar Fleet Controller.

This module contains the FleetControlsWidget class that provides
fleet-wide control operations like start all, stop all, platoon setup, etc.
"""

import tkinter as tk
from typing import Optional, Callable
from dataclasses import dataclass

from ..theme import Theme, DEFAULT_THEME
from .base import BaseWidget, ThemedButton, ThemedLabelFrame


@dataclass
class FleetControlCallbacks:
    """Callbacks for fleet control actions."""
    on_start_all: Callable[[], None] = None
    on_stop_all: Callable[[], None] = None
    on_setup_platoon: Callable[[], None] = None
    on_trigger_platoon: Callable[[], None] = None
    on_disable_all_platoons: Callable[[], None] = None
    on_activate_v2v: Callable[[], None] = None
    on_disable_v2v: Callable[[], None] = None
    on_activate_perception: Callable[[], None] = None
    on_disable_perception: Callable[[], None] = None


class FleetControlsWidget(BaseWidget):
    """Widget for fleet-wide control operations."""
    
    def __init__(self, parent: tk.Widget,
                 callbacks: FleetControlCallbacks = None,
                 theme: Theme = None):
        self.callbacks = callbacks or FleetControlCallbacks()
        self._v2v_btn: Optional[tk.Button] = None
        self._disable_v2v_btn: Optional[tk.Button] = None
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the fleet controls widget."""
        c = self.theme.colors
        
        self.frame = ThemedLabelFrame(
            self.parent,
            text="🚁 Fleet Operations",
            theme=self.theme
        )
        
        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill='x', padx=15, pady=10)
        
        # Row 1: Basic fleet controls
        self._build_basic_controls(content)
        
        # Row 2: Platoon and V2V controls
        self._build_platoon_controls(content)
        
        # Row 3: Perception controls
        self._build_perception_controls(content)
    
    def _build_basic_controls(self, parent: tk.Frame) -> None:
        """Build basic fleet control buttons."""
        c = self.theme.colors
        
        row = tk.Frame(parent, bg=c.bg_medium)
        row.pack(fill='x', pady=(0, 8))
        
        ThemedButton(
            row,
            text="▶ Start All",
            button_type='start',
            command=self._safe_callback(self.callbacks.on_start_all),
            padx=20,
            pady=8
        ).pack(side='left', expand=True, fill='x', padx=(0, 5))
        
        ThemedButton(
            row,
            text="⬛ Stop All",
            button_type='stop',
            command=self._safe_callback(self.callbacks.on_stop_all),
            padx=20,
            pady=8
        ).pack(side='left', expand=True, fill='x', padx=(5, 0))
    
    def _build_platoon_controls(self, parent: tk.Frame) -> None:
        """Build platoon and V2V control buttons."""
        c = self.theme.colors
        
        # First row of platoon controls
        row1 = tk.Frame(parent, bg=c.bg_medium)
        row1.pack(fill='x', pady=(8, 5))
        
        ThemedButton(
            row1,
            text="⚙️ Setup Platoon",
            button_type='command',
            command=self._safe_callback(self.callbacks.on_setup_platoon),
            padx=15,
            pady=6
        ).pack(side='left', expand=True, fill='x', padx=(0, 2))
        
        ThemedButton(
            row1,
            text="🚗🚗 Trigger Platoon",
            button_type='platoon',
            command=self._safe_callback(self.callbacks.on_trigger_platoon),
            padx=15,
            pady=6
        ).pack(side='left', expand=True, fill='x', padx=(2, 2))
        
        self._v2v_btn = ThemedButton(
            row1,
            text="📡 V2V Active",
            button_type='warning',
            command=self._safe_callback(self.callbacks.on_activate_v2v),
            padx=15,
            pady=6
        )
        self._v2v_btn.pack(side='left', expand=True, fill='x', padx=(2, 0))
        
        # Second row of platoon controls
        row2 = tk.Frame(parent, bg=c.bg_medium)
        row2.pack(fill='x')
        
        ThemedButton(
            row2,
            text="Disable All Platoons",
            button_type='secondary',
            command=self._safe_callback(self.callbacks.on_disable_all_platoons),
            padx=15,
            pady=6
        ).pack(side='left', expand=True, fill='x', padx=(0, 3))
        
        self._disable_v2v_btn = tk.Button(
            row2,
            text="📡 Disable V2V",
            bg='#4d4d4d',
            fg='white',
            font=self.theme.fonts.small_bold(),
            command=self._safe_callback(self.callbacks.on_disable_v2v),
            relief='flat',
            state='disabled',
            padx=15,
            pady=6
        )
        self._disable_v2v_btn.pack(side='left', expand=True, fill='x', padx=(3, 0))
    
    def _build_perception_controls(self, parent: tk.Frame) -> None:
        """Build perception (YOLO) control buttons."""
        c = self.theme.colors
        
        row = tk.Frame(parent, bg=c.bg_medium)
        row.pack(fill='x', pady=(8, 0))
        
        ThemedButton(
            row,
            text="👁️ Activate Perception",
            button_type='command',
            command=self._safe_callback(self.callbacks.on_activate_perception),
            padx=15,
            pady=6
        ).pack(side='left', expand=True, fill='x', padx=(0, 3))
        
        ThemedButton(
            row,
            text="👁️ Disable Perception",
            button_type='secondary',
            command=self._safe_callback(self.callbacks.on_disable_perception),
            padx=15,
            pady=6
        ).pack(side='left', expand=True, fill='x', padx=(3, 0))
    
    def _safe_callback(self, callback: Callable) -> Callable:
        """Create a safe callback wrapper."""
        def wrapper():
            if callback:
                callback()
        return wrapper
    
    def set_v2v_activating(self, activating: bool) -> None:
        """Set V2V button to activating state."""
        if self._v2v_btn:
            if activating:
                self._v2v_btn.config(
                    state='disabled',
                    bg='#4d4d4d',
                    text="📡 Activating..."
                )
            else:
                self._v2v_btn.config(
                    state='normal',
                    bg=self.theme.colors.accent_orange,
                    text="📡 V2V Active"
                )
    
    def set_v2v_connected(self, connected: bool) -> None:
        """Set V2V button state based on connection status."""
        if connected:
            if self._v2v_btn:
                self._v2v_btn.config(
                    state='disabled',
                    bg='#4d4d4d',
                    text="📡 V2V Connected"
                )
            if self._disable_v2v_btn:
                self._disable_v2v_btn.config(
                    state='normal',
                    bg=self.theme.colors.accent_brown
                )
        else:
            if self._v2v_btn:
                self._v2v_btn.config(
                    state='normal',
                    bg=self.theme.colors.accent_orange,
                    text="📡 V2V Active"
                )
            if self._disable_v2v_btn:
                self._disable_v2v_btn.config(
                    state='disabled',
                    bg='#4d4d4d'
                )
    
    def reset_v2v_buttons(self) -> None:
        """Reset V2V buttons to default state."""
        self.set_v2v_connected(False)
