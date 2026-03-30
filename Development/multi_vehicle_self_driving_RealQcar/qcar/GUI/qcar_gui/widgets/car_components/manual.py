import tkinter as tk
from tkinter import ttk
from typing import Dict, Optional

from ...theme import Theme
from ...config import VehicleConfig
from ..base import BaseWidget, ThemedLabelFrame, ThemedLabel, ThemedButton, ThemedEntry
from .types import CarPanelCallbacks

class ManualAndVelocityControl(BaseWidget):
    """Combined widget for manual control, velocity, and gear to save space."""

    PROFILE_STEPS = {
        "max_forward_throttle": 0.01,
        "max_reverse_throttle": 0.01,
        "throttle_step": 0.005,
        "steering_limit": 0.02,
        "steering_step": 0.01,
    }

    PROFILE_FIELDS = (
        ("FMax", "max_forward_throttle"),
        ("RMax", "max_reverse_throttle"),
        ("dThr", "throttle_step"),
        ("SLim", "steering_limit"),
        ("dStr", "steering_step"),
    )

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
        self._manual_profile: Dict[str, float] = {
            "vehicle_type": "Qcar",
            "max_forward_throttle": 0.30,
            "max_reverse_throttle": 0.18,
            "throttle_step": 0.02,
            "steering_limit": 0.50,
            "steering_step": 0.05,
        }
        self._profile_value_labels: Dict[str, tk.Label] = {}

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

        # --- Top Row: Manual Control (Single Line) ---
        control_type_frame = tk.Frame(top_row, bg=c.bg_medium)
        control_type_frame.pack(fill="x", pady=(0, 2))

        ThemedLabel(
            control_type_frame, text="Control:", style="muted", theme=self.theme
        ).pack(side="left", padx=(0, 4))

        self._control_type_var = tk.StringVar(value="keyboard")

        for text, value in [("Keyboard", "keyboard"), ("Volant", "wheel")]:
            tk.Radiobutton(
                control_type_frame,
                text=text,
                variable=self._control_type_var,
                value=value,
                bg=c.bg_medium,
                fg=c.fg_primary,
                selectcolor=c.bg_light,
                activebackground=c.bg_light,
                activeforeground=c.accent_blue,
                font=self.theme.fonts.tiny(),
                command=lambda v=value: self._on_control_type_change(v),
            ).pack(side="left", padx=(0, 2))

        self._manual_btn = ThemedButton(
            control_type_frame,
            text="🎮 Manual Mode",
            button_type="platoon",
            command=self._toggle_manual,
            padx=8,
            pady=1,
        )
        self._manual_btn.pack(side="left", padx=(4, 0))

        keyboard_frame = ThemedLabelFrame(
            top_row, text="Keyboard Test Bench", theme=self.theme
        )
        keyboard_frame.pack(fill="x", pady=(5, 0))

        keyboard_content = tk.Frame(keyboard_frame, bg=c.bg_medium)
        keyboard_content.pack(fill="x", padx=4, pady=4)

        grid_frame = tk.Frame(keyboard_content, bg=c.bg_medium)
        grid_frame.pack(fill="x")

        for row_index, (label, field) in enumerate(self.PROFILE_FIELDS):
            self._build_profile_row(grid_frame, row_index, label, field)

        ThemedLabel(
            keyboard_content,
            text="Profile only: FMax/RMax are limits, dThr/dStr are ramp steps",
            style="muted",
            theme=self.theme,
            font=self.theme.fonts.tiny(),
            anchor="w",
        ).pack(fill="x", pady=(4, 0))

        ThemedLabel(
            keyboard_content,
            text="Hotkeys: U/I, O/P, J/K, L/M | live output is in Telemetry",
            style="muted",
            theme=self.theme,
            font=self.theme.fonts.tiny(),
            anchor="w",
        ).pack(fill="x")

        self._refresh_manual_profile_labels()

        # --- Bottom Row: Velocity & Gear (Single Line) ---
        ThemedLabel(bottom_row, text="Target:", style="muted", theme=self.theme).pack(
            side="left", padx=(0, 2)
        )

        self._entry = ThemedEntry(bottom_row, width=4, theme=self.theme)
        self._entry.insert(0, "1.0")
        self._entry.pack(side="left", padx=(0, 2))

        ThemedButton(
            bottom_row,
            text="Set",
            button_type="command",
            command=self._set_velocity,
            padx=4,
            pady=1,
        ).pack(side="left", padx=(0, 8))

        # Vertical separator (optional, but looks better)
        ttk.Separator(bottom_row, orient="vertical").pack(side="left", fill="y", padx=4, pady=2)

        ThemedLabel(bottom_row, text="Gear:", style="muted", theme=self.theme).pack(
            side="left", padx=(4, 2)
        )

        self._gear_label = ThemedLabel(
            bottom_row,
            text=self._format_gear(self._current_gear),
            theme=self.theme,
            font=self.theme.fonts.small_bold(),
        )
        self._gear_label.pack(side="left", padx=(0, 4))

        ThemedButton(
            bottom_row,
            text="▲",
            button_type="command",
            command=self._on_gear_up,
            padx=2,
            width=2,
        ).pack(side="left", padx=(0, 2))

        ThemedButton(
            bottom_row,
            text="▼",
            button_type="command",
            command=self._on_gear_down,
            padx=2,
            width=2,
        ).pack(side="left")

    def _build_profile_row(
        self, parent: tk.Widget, row_index: int, label: str, field: str
    ) -> None:
        """Build a compact keyboard profile adjustment column."""
        row = tk.Frame(parent, bg=self.theme.colors.bg_medium)
        columns_per_row = 3
        grid_row = row_index // columns_per_row
        grid_col = row_index % columns_per_row

        row.grid(row=grid_row, column=grid_col, sticky="ew", padx=1, pady=1)
        parent.grid_columnconfigure(grid_col, weight=1)

        ThemedLabel(
            row,
            text=f"{label}:",
            style="muted",
            theme=self.theme,
            font=self.theme.fonts.tiny(),
            width=6,
            anchor="center",
        ).pack(side="top")

        controls = tk.Frame(row, bg=self.theme.colors.bg_medium)
        controls.pack(side="top", pady=(1, 0))

        ThemedButton(
            controls,
            text="-",
            button_type="warning",
            command=lambda f=field: self._bump_manual_profile(
                f, -self.PROFILE_STEPS[f]
            ),
            padx=4,
            pady=0,
            width=2,
            font=self.theme.fonts.tiny(),
        ).pack(side="left", padx=(0, 1))

        value_label = ThemedLabel(
            controls,
            text="0.00",
            theme=self.theme,
            font=self.theme.fonts.tiny(),
            width=6,
            anchor="center",
        )
        value_label.pack(side="left", padx=(0, 1))
        self._profile_value_labels[field] = value_label

        ThemedButton(
            controls,
            text="+",
            button_type="command",
            command=lambda f=field: self._bump_manual_profile(
                f, self.PROFILE_STEPS[f]
            ),
            padx=4,
            pady=0,
            width=2,
            font=self.theme.fonts.tiny(),
        ).pack(side="left")

    # --- Manual Methods ---
    def _on_control_type_change(self, control_type: str) -> None:
        if self.callbacks.on_update_control_type:
            self.callbacks.on_update_control_type(self.car_id, control_type)

    def _toggle_manual(self) -> None:
        if self.callbacks.on_toggle_manual:
            self.callbacks.on_toggle_manual(self.car_id)

    def _bump_manual_profile(self, field: str, delta: float) -> None:
        current_value = float(self._manual_profile.get(field, 0.0))
        new_value = current_value + delta
        self._manual_profile[field] = round(new_value, 3)
        self._refresh_manual_profile_labels()

        if self.callbacks.on_update_manual_profile:
            self.callbacks.on_update_manual_profile(self.car_id, {field: new_value})

    def _refresh_manual_profile_labels(self) -> None:
        for field, label in self._profile_value_labels.items():
            value = self._manual_profile.get(field, 0.0)
            label.config(text=f"{value:.3f}" if field.endswith("step") else f"{value:.2f}")

    def set_manual_profile(self, profile: Dict[str, float]) -> None:
        """Update the displayed keyboard profile values."""
        if not profile:
            return
        self._manual_profile.update(profile)
        self._refresh_manual_profile_labels()

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
