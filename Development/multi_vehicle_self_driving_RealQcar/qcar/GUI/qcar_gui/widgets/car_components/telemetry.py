import tkinter as tk
from typing import Dict

from ...theme import Theme
from ...config import TELEMETRY_FIELDS
from ..base import BaseWidget, ThemedLabelFrame, ThemedLabel
from .types import CarState

class TelemetryDisplay(BaseWidget):
    """Widget for displaying vehicle telemetry data."""

    def __init__(self, parent: tk.Widget, theme: Theme = None):
        self._labels: Dict[str, tk.Label] = {}
        super().__init__(parent, theme)

    def _build(self) -> None:
        """Build the telemetry display."""
        c = self.theme.colors

        self.frame = ThemedLabelFrame(
            self.parent, text="📊 Telemetry", theme=self.theme
        )

        grid = tk.Frame(self.frame, bg=c.bg_medium)
        grid.pack(fill="x", padx=2, pady=1)

        for i, (key, label_text, default_value, width) in enumerate(TELEMETRY_FIELDS):
            row, col = i // 2, i % 2

            label_frame = tk.Frame(grid, bg=c.bg_medium)
            label_frame.grid(row=row, column=col, padx=2, pady=0, sticky="w")

            ThemedLabel(
                label_frame, text=label_text, style="muted", theme=self.theme
            ).pack(side="left")

            value_label = tk.Label(
                label_frame,
                text=default_value,
                bg=c.bg_medium,
                fg=c.fg_primary,
                font=self.theme.fonts.small_bold(),
                width=width,
                anchor="w",
            )
            value_label.pack(side="left", padx=(5, 0))

            self._labels[key] = value_label

    def update(self, state: CarState) -> None:
        """Update telemetry display with current state."""
        if "position" in self._labels:
            x, y = state.position
            self._labels["position"].config(text=f"({x:.2f}, {y:.2f})")

        if "velocity" in self._labels:
            self._labels["velocity"].config(text=f"{state.velocity:.2f}")

        if "heading" in self._labels:
            self._labels["heading"].config(text=f"{state.heading:.2f}")

        if "throttle" in self._labels:
            self._labels["throttle"].config(text=f"{state.throttle:.2f}")

        if "steering" in self._labels:
            self._labels["steering"].config(text=f"{state.steering:.2f}")

        if "state" in self._labels:
            self._labels["state"].config(text=state.state)

        for key in [
            "path_long_ctrl",
            "path_lat_ctrl",
            "leader_long_ctrl",
            "leader_lat_ctrl",
        ]:
            if key in self._labels:
                self._labels[key].config(text=getattr(state, key, "unknown"))

        if "perception" in self._labels:
            self._labels["perception"].config(
                text="ON" if state.perception_active else "OFF"
            )
