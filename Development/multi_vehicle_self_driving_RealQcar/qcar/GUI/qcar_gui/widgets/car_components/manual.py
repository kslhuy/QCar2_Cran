import tkinter as tk
from tkinter import ttk
from typing import Optional

from ...theme import Theme
from ...config import VehicleConfig
from ..base import BaseWidget, ThemedLabelFrame, ThemedLabel, ThemedButton, ThemedEntry
from .types import CarPanelCallbacks

class ManualAndVelocityControl(BaseWidget):
    """Combined widget for manual control, velocity, and gear to save space."""

    def __init__(
        self,
        parent: tk.Widget,
        car_id: int,
        callbacks: CarPanelCallbacks,
        config: VehicleConfig = None,
        theme: Theme = None,
    ):
        self.car_id = car_id
        self.callbacks = callbacks
        self.config = config or VehicleConfig()
        
        self._control_type_var: Optional[tk.StringVar] = None
        self._manual_btn: Optional[tk.Button] = None
        self._manual_active = False
        
        self._entry: Optional[ThemedEntry] = None
        self._gear_label: Optional[tk.Label] = None
        self._current_gear = "DRIVE_1"
        
        super().__init__(parent, theme)

    def _build(self) -> None:
        c = self.theme.colors

        self.frame = ThemedLabelFrame(
            self.parent, text="🕹️ Manual & Velocity", theme=self.theme
        )

        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill="both", expand=True, padx=2, pady=2)
        
        # Split into top (Manual) and bottom (Velocity/Gear)
        top_row = tk.Frame(content, bg=c.bg_medium)
        top_row.pack(side="top", fill="x", expand=True, pady=(0, 2))
        
        ttk.Separator(content, orient="horizontal").pack(side="top", fill="x", pady=2)
        
        bottom_row = tk.Frame(content, bg=c.bg_medium)
        bottom_row.pack(side="top", fill="x", expand=True, pady=(2, 0))

        # --- Top Row: Manual Control ---
        control_type_frame = tk.Frame(top_row, bg=c.bg_medium)
        control_type_frame.pack(fill="x", pady=(0, 3))

        ThemedLabel(
            control_type_frame, text="Control:", style="muted", theme=self.theme
        ).pack(side="left", padx=(0, 4))

        self._control_type_var = tk.StringVar(value="keyboard")

        for text, value in [("⌨️ Kybd", "keyboard"), ("🎡 Whl", "wheel")]:
            tk.Radiobutton(
                control_type_frame,
                text=text,
                variable=self._control_type_var,
                value=value,
                bg=c.bg_medium,
                fg=c.fg_primary,
                selectcolor=c.bg_light,
                font=self.theme.fonts.tiny(),
                command=lambda v=value: self._on_control_type_change(v),
            ).pack(side="left", padx=(0, 2))

        self._manual_btn = ThemedButton(
            top_row,
            text="🎮 Manual Mode",
            button_type="platoon",
            command=self._toggle_manual,
            padx=4,
            pady=1,
        )
        self._manual_btn.pack(fill="x", pady=(3, 0))

        # --- Bottom Row: Velocity & Gear ---
        vel_row = tk.Frame(bottom_row, bg=c.bg_medium)
        vel_row.pack(fill="x", pady=(0, 3))

        ThemedLabel(vel_row, text="Target:", style="muted", theme=self.theme).pack(
            side="left", padx=(0, 2)
        )

        self._entry = ThemedEntry(vel_row, width=5, theme=self.theme)
        self._entry.insert(0, "1.0")
        self._entry.pack(side="left", padx=(0, 2))

        ThemedButton(
            vel_row,
            text="Set",
            button_type="command",
            command=self._set_velocity,
            padx=6,
            pady=1,
        ).pack(side="left")

        gear_row = tk.Frame(bottom_row, bg=c.bg_medium)
        gear_row.pack(fill="x", pady=(3, 0))

        ThemedLabel(gear_row, text="Gear:", style="muted", theme=self.theme).pack(
            side="left", padx=(0, 2)
        )

        self._gear_label = ThemedLabel(
            gear_row,
            text=self._format_gear(self._current_gear),
            theme=self.theme,
            font=self.theme.fonts.small_bold(),
        )
        self._gear_label.pack(side="left", padx=(0, 4))

        ThemedButton(
            gear_row,
            text="▲",
            button_type="command",
            command=self._on_gear_up,
            padx=4,
            width=2,
        ).pack(side="left", padx=(0, 2))

        ThemedButton(
            gear_row,
            text="▼",
            button_type="command",
            command=self._on_gear_down,
            padx=4,
            width=2,
        ).pack(side="left")

    # --- Manual Methods ---
    def _on_control_type_change(self, control_type: str) -> None:
        if self.callbacks.on_update_control_type:
            self.callbacks.on_update_control_type(self.car_id, control_type)

    def _toggle_manual(self) -> None:
        if self.callbacks.on_toggle_manual:
            self.callbacks.on_toggle_manual(self.car_id)

    def set_manual_active(self, active: bool) -> None:
        self._manual_active = active
        if self._manual_btn:
            if active:
                self._manual_btn.config(
                    text="🎮 Manual: ON", bg=self.theme.colors.accent_green
                )
            else:
                self._manual_btn.config(
                    text="🎮 Manual Mode", bg=self.theme.colors.accent_purple
                )

    @property
    def control_type(self) -> str:
        return self._control_type_var.get() if self._control_type_var else "keyboard"

    # --- Velocity & Gear Methods ---
    def _set_velocity(self) -> None:
        if self.callbacks.on_set_velocity and self._entry:
            velocity = self._entry.get_float()
            if self.config.min_velocity <= velocity <= self.config.max_velocity:
                self.callbacks.on_set_velocity(self.car_id, velocity)

    def _format_gear(self, gear_str: str) -> str:
        gear_map = {"DRIVE_1": "D1", "DRIVE_2": "D2", "DRIVE_3": "D3", "DRIVE_4": "D4", "DRIVE_5": "D5"}
        return gear_map.get(gear_str, gear_str)

    def _on_gear_up(self) -> None:
        gear_progression = ["DRIVE_1", "DRIVE_2", "DRIVE_3", "DRIVE_4", "DRIVE_5"]
        if self._current_gear in gear_progression:
            idx = gear_progression.index(self._current_gear)
            if idx + 1 < len(gear_progression):
                next_gear = gear_progression[idx + 1]
                if self.callbacks.on_set_gear:
                    self.callbacks.on_set_gear(self.car_id, next_gear)

    def _on_gear_down(self) -> None:
        gear_progression = ["DRIVE_1", "DRIVE_2", "DRIVE_3", "DRIVE_4", "DRIVE_5"]
        if self._current_gear in gear_progression:
            idx = gear_progression.index(self._current_gear)
            if idx - 1 >= 0:
                next_gear = gear_progression[idx - 1]
                if self.callbacks.on_set_gear:
                    self.callbacks.on_set_gear(self.car_id, next_gear)

    def set_gear(self, gear: str) -> None:
        self._current_gear = gear
        if self._gear_label:
            self._gear_label.config(text=self._format_gear(gear))
