import tkinter as tk
from tkinter import ttk
from typing import Optional

from ...theme import Theme
from ...config import DEFAULT_PATHS
from ..base import BaseWidget, ThemedLabelFrame, ThemedLabel, ThemedButton, ThemedEntry
from .types import CarPanelCallbacks

class NavigationControl(BaseWidget):
    """Combined widget for Path/Position and Taxi Mode control."""

    def __init__(
        self,
        parent: tk.Widget,
        car_id: int,
        callbacks: CarPanelCallbacks,
        theme: Theme = None,
    ):
        self.car_id = car_id
        self.callbacks = callbacks
        
        # Path entries
        self._path_entry: Optional[ThemedEntry] = None
        self._x_entry: Optional[ThemedEntry] = None
        self._y_entry: Optional[ThemedEntry] = None
        self._theta_entry: Optional[ThemedEntry] = None
        self._calibrate_var: Optional[tk.BooleanVar] = None
        
        # Taxi entries
        self._nodes_entry: Optional[ThemedEntry] = None
        self._taxi_mode_active = False
        self._toggle_btn: Optional[tk.Button] = None
        
        super().__init__(parent, theme)

    def _build(self) -> None:
        c = self.theme.colors

        self.frame = ThemedLabelFrame(
            self.parent, text="🧭 Navigation & Taxi", theme=self.theme
        )

        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill="both", expand=True, padx=2, pady=2)

        # Split into left (Path) and right (Taxi)
        left_col = tk.Frame(content, bg=c.bg_medium)
        left_col.pack(side="left", fill="both", expand=True, padx=(0, 2))
        
        ttk.Separator(content, orient="vertical").pack(side="left", fill="y", padx=2)
        
        right_col = tk.Frame(content, bg=c.bg_medium)
        right_col.pack(side="left", fill="both", expand=True, padx=(2, 0))

        # --- Left Col: Path & Position ---
        pos_row = tk.Frame(left_col, bg=c.bg_medium)
        pos_row.pack(fill="x", pady=(0, 3))

        ThemedLabel(pos_row, text="Init Pos:", style="muted", theme=self.theme).pack(
            side="left", padx=(0, 2)
        )

        for label, attr_name in [("X:", "_x_entry"), ("Y:", "_y_entry"), ("θ:", "_theta_entry")]:
            ThemedLabel(pos_row, text=label, style="muted", theme=self.theme).pack(
                side="left", padx=(0, 1)
            )
            entry = ThemedEntry(pos_row, width=4, theme=self.theme)
            entry.insert(0, "0.0")
            entry.pack(side="left", padx=(0, 2))
            setattr(self, attr_name, entry)

        self._calibrate_var = tk.BooleanVar(value=False)
        tk.Checkbutton(
            pos_row,
            text="Cal GPS",
            variable=self._calibrate_var,
            bg=c.bg_medium,
            fg=c.accent_green,
            selectcolor=c.bg_dark,
            activebackground=c.bg_medium,
            activeforeground=c.accent_green,
            font=self.theme.fonts.tiny(),
        ).pack(side="left", padx=(0, 2))

        ThemedButton(
            pos_row, text="Set", button_type="command", command=self._set_initial_position, padx=4, pady=1
        ).pack(side="left")

        path_row = tk.Frame(left_col, bg=c.bg_medium)
        path_row.pack(fill="x", pady=(3, 0))

        ThemedLabel(path_row, text="Nodes:", style="muted", theme=self.theme).pack(
            side="left", padx=(0, 5)
        )

        self._path_entry = ThemedEntry(path_row, width=12, theme=self.theme)
        default_path = DEFAULT_PATHS.get(self.car_id, DEFAULT_PATHS["default"])
        self._path_entry.insert(0, default_path)
        self._path_entry.pack(side="left", fill="x", expand=True, padx=(0, 5))

        ThemedButton(
            path_row, text="Set", button_type="command", command=self._set_path, padx=6, pady=1
        ).pack(side="left")

        # --- Right Col: Taxi Mode ---
        mode_row = tk.Frame(right_col, bg=c.bg_medium)
        mode_row.pack(fill="x", pady=(0, 3))

        self._toggle_btn = ThemedButton(
            mode_row,
            text="Enable Taxi Mode",
            button_type="start",
            command=self._toggle_taxi_mode,
            padx=4,
            pady=1,
        )
        self._toggle_btn.pack(side="left", fill="x", expand=True)

        trip_row = tk.Frame(right_col, bg=c.bg_medium)
        trip_row.pack(fill="x", pady=(3, 0))

        ThemedLabel(
            trip_row, text="Stops:", style="muted", theme=self.theme
        ).pack(side="left", padx=(0, 5))

        self._nodes_entry = ThemedEntry(trip_row, width=10, theme=self.theme)
        self._nodes_entry.insert(0, "20,9")
        self._nodes_entry.pack(side="left", fill="x", expand=True, padx=(0, 5))

        ThemedButton(
            trip_row,
            text="Set Trip",
            button_type="command",
            command=self._set_trip,
            padx=6,
            pady=1,
        ).pack(side="left")

    # --- Path Methods ---
    def _set_initial_position(self) -> None:
        if self.callbacks.on_set_initial_position:
            x = self._x_entry.get_float()
            y = self._y_entry.get_float()
            theta = self._theta_entry.get_float()
            calibrate = self._calibrate_var.get()
            self.callbacks.on_set_initial_position(self.car_id, x, y, theta, calibrate)

    def _set_path(self) -> None:
        if self.callbacks.on_set_path and self._path_entry:
            nodes = self._path_entry.get_list(separator=",", item_type=int)
            if len(nodes) >= 2:
                self.callbacks.on_set_path(self.car_id, nodes)

    # --- Taxi Methods ---
    def _toggle_taxi_mode(self) -> None:
        if not self._taxi_mode_active and self.callbacks.on_enable_taxi_mode:
            self.callbacks.on_enable_taxi_mode(self.car_id)
        elif self._taxi_mode_active and self.callbacks.on_disable_taxi_mode:
            self.callbacks.on_disable_taxi_mode(self.car_id)

    def _set_trip(self) -> None:
        if self.callbacks.on_set_taxi_trip and self._nodes_entry:
            nodes = self._nodes_entry.get_list(separator=",", item_type=int)
            if len(nodes) > 0:
                self.callbacks.on_set_taxi_trip(self.car_id, nodes)

    def set_taxi_mode_active(self, active: bool) -> None:
        self._taxi_mode_active = active
        if active:
            self._toggle_btn.config(
                text="Disable Taxi Mode", bg=self.theme.colors.accent_red
            )
        else:
            self._toggle_btn.config(
                text="Enable Taxi Mode", bg=self.theme.colors.accent_green
            )
