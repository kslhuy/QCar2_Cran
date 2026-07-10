import tkinter as tk
from typing import Callable

from ...theme import Theme
from ..base import BaseWidget, ThemedButton
from .types import CarPanelCallbacks

class ControlButtons(BaseWidget):
    """Widget for vehicle control buttons (Start, Stop, Calibrate)."""

    def __init__(
        self,
        parent: tk.Widget,
        car_id: int,
        callbacks: CarPanelCallbacks,
        theme: Theme = None,
    ):
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
            button_type="start",
            command=lambda: self._safe_callback(self.callbacks.on_start),
            padx=4,
            pady=2,
        ).pack(side="left", expand=True, fill="x", padx=(0, 1))

        # Calibrate button
        ThemedButton(
            self.frame,
            text="🔧 Calibrate",
            button_type="warning",
            command=lambda: self._safe_callback(self.callbacks.on_calibrate),
            padx=4,
            pady=2,
        ).pack(side="left", expand=True, fill="x", padx=(1, 1))

        # Stop button
        ThemedButton(
            self.frame,
            text="⬛ STOP",
            button_type="stop",
            command=lambda: self._safe_callback(self.callbacks.on_stop),
            padx=4,
            pady=2,
        ).pack(side="left", expand=True, fill="x", padx=(1, 0))

    def _safe_callback(self, callback: Callable) -> None:
        """Safely call a callback if it exists."""
        if callback:
            callback(self.car_id)
