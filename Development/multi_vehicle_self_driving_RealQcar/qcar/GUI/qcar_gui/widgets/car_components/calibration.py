import tkinter as tk
from typing import Optional

from ...theme import Theme
from ..base import BaseWidget, ThemedLabelFrame, ThemedLabel, ThemedButton
from .types import CarPanelCallbacks

class OnlineSysidControl(BaseWidget):
    """Widget for Online SysID control."""

    def __init__(
        self,
        parent: tk.Widget,
        car_id: int,
        callbacks: CarPanelCallbacks,
        theme: Theme = None,
    ):
        self.car_id = car_id
        self.callbacks = callbacks
        self._status_label: Optional[tk.Label] = None
        super().__init__(parent, theme)

    def _build(self) -> None:
        c = self.theme.colors

        self.frame = ThemedLabelFrame(
            self.parent, text="🧠 Online SysID", theme=self.theme
        )

        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill="x", padx=6, pady=4)

        # Row 1: Status
        row1 = tk.Frame(content, bg=c.bg_medium)
        row1.pack(fill="x", pady=(0, 3))

        self._status_label = ThemedLabel(
            row1, text="Status: Offline", style="muted", theme=self.theme
        )
        self._status_label.pack(side="left")

        # Row 2: Controls
        row2 = tk.Frame(content, bg=c.bg_medium)
        row2.pack(fill="x")

        # Start collection
        ThemedButton(
            row2,
            text="Collect",
            button_type="start",
            command=lambda: self._send_action("start"),
            padx=8,
            pady=3,
        ).pack(side="left", expand=True, fill="x", padx=(0, 2))

        # Stop collection
        ThemedButton(
            row2,
            text="Pause",
            button_type="warning",
            command=lambda: self._send_action("stop"),
            padx=8,
            pady=3,
        ).pack(side="left", expand=True, fill="x", padx=2)

        # Train model
        ThemedButton(
            row2,
            text="Train",
            button_type="command",
            command=lambda: self._send_action("train"),
            padx=8,
            pady=3,
        ).pack(side="left", expand=True, fill="x", padx=2)

        # Clear buffer
        ThemedButton(
            row2,
            text="Clear",
            button_type="stop",
            command=lambda: self._send_action("clear"),
            padx=8,
            pady=3,
        ).pack(side="left", expand=True, fill="x", padx=(2, 0))

    def _send_action(self, action: str):
        if self.callbacks.on_set_online_sysid:
            self.callbacks.on_set_online_sysid(self.car_id, action, {})

    def update_status(self, sysid_status: dict) -> None:
        """Update status label based on telemetry."""
        if not self._status_label:
            return

        if not sysid_status or not hasattr(sysid_status, "get"):
            self._status_label.config(
                text="Status: Offline", fg=self.theme.colors.fg_muted
            )
            return

        zmq_status = sysid_status.get("zmq", {})
        if not zmq_status:
            self._status_label.config(
                text="Status: Offline", fg=self.theme.colors.fg_muted
            )
            return

        is_running = zmq_status.get("running", False)
        is_collecting = zmq_status.get("collecting", False)

        remote_status = zmq_status.get("last_remote_status", {})
        if isinstance(remote_status, dict) and "status" in remote_status:
            worker_info = remote_status["status"]
        else:
            worker_info = {}

        samples = worker_info.get("buffered_samples", zmq_status.get("samples_sent", 0))

        if not is_running:
            self._status_label.config(
                text="Status: Process Offline", fg=self.theme.colors.accent_red
            )
        elif is_collecting:
            self._status_label.config(
                text=f"Status: Collecting ({samples} samples)",
                fg=self.theme.colors.accent_green,
            )
        else:
            self._status_label.config(
                text=f"Status: Idle ({samples} samples)",
                fg=self.theme.colors.accent_blue,
            )


class CalibrationControl(BaseWidget):
    """Widget for active and passive Calibration control."""

    def __init__(
        self,
        parent: tk.Widget,
        car_id: int,
        callbacks: CarPanelCallbacks,
        theme: Theme = None,
    ):
        self.car_id = car_id
        self.callbacks = callbacks
        self._status_label: Optional[tk.Label] = None
        self._cal_type_var: Optional[tk.StringVar] = None
        super().__init__(parent, theme)

    def _build(self) -> None:
        c = self.theme.colors

        self.frame = ThemedLabelFrame(
            self.parent, text="📐 Calibration", theme=self.theme
        )

        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill="x", padx=6, pady=4)

        # Row 1: Status
        row1 = tk.Frame(content, bg=c.bg_medium)
        row1.pack(fill="x", pady=(0, 3))

        self._status_label = ThemedLabel(
            row1, text="Passive Status: Offline", style="muted", theme=self.theme
        )
        self._status_label.pack(side="left")

        # Row 2: Passive Controls
        row2 = tk.Frame(content, bg=c.bg_medium)
        row2.pack(fill="x", pady=(0, 5))

        ThemedLabel(row2, text="Passive:", style="muted", theme=self.theme).pack(side="left", padx=(0, 3))

        ThemedButton(
            row2,
            text="Collect",
            button_type="start",
            command=self._start_passive,
            padx=5,
            pady=2,
        ).pack(side="left", expand=True, fill="x", padx=1)

        ThemedButton(
            row2,
            text="Pause",
            button_type="warning",
            command=self._stop_passive,
            padx=5,
            pady=2,
        ).pack(side="left", expand=True, fill="x", padx=1)

        ThemedButton(
            row2,
            text="Analyse",
            button_type="command",
            command=self._analyse_passive,
            padx=5,
            pady=2,
        ).pack(side="left", expand=True, fill="x", padx=1)

        ThemedButton(
            row2,
            text="Clear",
            button_type="stop",
            command=self._clear_passive,
            padx=5,
            pady=2,
        ).pack(side="left", expand=True, fill="x", padx=1)

        # Row 3: Active Controls
        row3 = tk.Frame(content, bg=c.bg_medium)
        row3.pack(fill="x")

        ThemedLabel(row3, text="Active:", style="muted", theme=self.theme).pack(side="left", padx=(0, 3))

        self._cal_type_var = tk.StringVar(value="throttle_velocity")
        cal_menu = tk.OptionMenu(
            row3,
            self._cal_type_var,
            "throttle_velocity",
            "steering_curvature",
            "throttle_acceleration",
        )
        cal_menu.config(
            bg=c.bg_light,
            fg=c.fg_primary,
            highlightthickness=0,
            font=self.theme.fonts.tiny(),
            width=15,
        )
        cal_menu["menu"].config(bg=c.bg_light, fg=c.fg_primary)
        cal_menu.pack(side="left", padx=(0, 3), fill="x", expand=True)

        ThemedButton(
            row3,
            text="Trigger",
            button_type="command",
            command=self._trigger_active,
            padx=5,
            pady=2,
        ).pack(side="left")

    def _start_passive(self):
        if self.callbacks.on_start_online_calibration:
            self.callbacks.on_start_online_calibration(self.car_id)

    def _stop_passive(self):
        if self.callbacks.on_stop_online_calibration:
            self.callbacks.on_stop_online_calibration(self.car_id)

    def _analyse_passive(self):
        if self.callbacks.on_trigger_online_analysis:
            cal_type = self._cal_type_var.get()
            self.callbacks.on_trigger_online_analysis(self.car_id, cal_type, {})

    def _clear_passive(self):
        if self.callbacks.on_clear_online_calibration:
            self.callbacks.on_clear_online_calibration(self.car_id)

    def _trigger_active(self):
        if self.callbacks.on_trigger_active_calibration:
            cal_type = self._cal_type_var.get()
            self.callbacks.on_trigger_active_calibration(self.car_id, cal_type, {})

    def update_status(self, calib_status: dict) -> None:
        if not self._status_label or not calib_status:
            return

        zmq_status = calib_status.get("zmq", {})
        if not zmq_status:
            self._status_label.config(
                text="Passive Status: Offline", fg=self.theme.colors.fg_muted
            )
            return

        is_running = zmq_status.get("running", False)
        is_collecting = zmq_status.get("collecting", False)
        
        remote_status = zmq_status.get("last_remote_status", {})
        if isinstance(remote_status, dict) and "status" in remote_status:
            worker_info = remote_status["status"]
        else:
            worker_info = {}
            
        samples = worker_info.get("buffered_samples", zmq_status.get("samples_sent", 0))

        if not is_running:
            self._status_label.config(
                text="Passive Status: Offline", fg=self.theme.colors.accent_red
            )
        elif is_collecting:
            self._status_label.config(
                text=f"Passive: Collecting ({samples})",
                fg=self.theme.colors.accent_green,
            )
        else:
            self._status_label.config(
                text=f"Passive: Idle ({samples})",
                fg=self.theme.colors.accent_blue,
            )


class RobustDatasetControl(BaseWidget):
    """Widget for Robust KalmanNet offline dataset collection."""

    def __init__(
        self,
        parent: tk.Widget,
        car_id: int,
        callbacks: CarPanelCallbacks,
        theme: Theme = None,
    ):
        self.car_id = car_id
        self.callbacks = callbacks
        self._status_label: Optional[tk.Label] = None
        super().__init__(parent, theme)

    def _build(self) -> None:
        c = self.theme.colors

        self.frame = ThemedLabelFrame(
            self.parent, text="Offline RKNet Data", theme=self.theme
        )

        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill="x", padx=6, pady=4)

        row1 = tk.Frame(content, bg=c.bg_medium)
        row1.pack(fill="x", pady=(0, 3))

        self._status_label = ThemedLabel(
            row1, text="Status: Idle", style="muted", theme=self.theme
        )
        self._status_label.pack(side="left")

        row2 = tk.Frame(content, bg=c.bg_medium)
        row2.pack(fill="x")

        self._buttons = {}
        
        # Start Button
        self._buttons["start"] = ThemedButton(
            row2,
            text="Start",
            button_type="start",
            command=lambda: self._send_action("start"),
            padx=4,
            pady=3,
        )
        self._buttons["start"].pack(side="left", expand=True, fill="x", padx=(0, 1))

        # Stop Button
        self._buttons["stop"] = ThemedButton(
            row2,
            text="Stop",
            button_type="warning",
            command=lambda: self._send_action("stop"),
            padx=4,
            pady=3,
        )
        self._buttons["stop"].pack(side="left", expand=True, fill="x", padx=1)

        # Save Button
        self._buttons["save"] = ThemedButton(
            row2,
            text="Save",
            button_type="command",
            command=lambda: self._send_action("save"),
            padx=4,
            pady=3,
        )
        self._buttons["save"].pack(side="left", expand=True, fill="x", padx=1)

        # Discard Button
        self._buttons["discard"] = ThemedButton(
            row2,
            text="Discard",
            button_type="danger",
            command=lambda: self._send_action("discard"),
            padx=4,
            pady=3,
        )
        self._buttons["discard"].pack(side="left", expand=True, fill="x", padx=(1, 0))

    def _send_action(self, action: str) -> None:
        if self.callbacks.on_set_robust_kalmannet_dataset:
            self.callbacks.on_set_robust_kalmannet_dataset(self.car_id, action, {})


    def update_status(self, status: dict) -> None:
        if not self._status_label or not self._buttons:
            return

        if not status or not hasattr(status, "get"):
            self._status_label.config(
                text="Status: Idle", fg=self.theme.colors.fg_muted
            )
            self._update_button_states(False, False, 0)
            return

        is_enabled = bool(status.get("enabled", False))
        is_recording = bool(status.get("recording", False))
        samples = int(status.get("buffered_samples", 0))

        if not is_enabled:
            self._status_label.config(
                text="Status: Idle", fg=self.theme.colors.fg_muted
            )
        elif is_recording:
            self._status_label.config(
                text=f"Status: Recording ({samples})",
                fg=self.theme.colors.accent_green,
            )
        elif samples > 0:
            self._status_label.config(
                text=f"Status: Stopped ({samples} buffered)",
                fg=self.theme.colors.accent_blue,
            )
        else:
            self._status_label.config(
                text="Status: Ready", fg=self.theme.colors.fg_muted
            )

        self._update_button_states(is_enabled, is_recording, samples)

    def _update_button_states(
        self, is_enabled: bool, is_recording: bool, samples: int
    ) -> None:
        """Helper to set button states based on recording status."""
        if not self._buttons:
            return

        if is_recording:
            self._buttons["start"].set_enabled(False)
            self._buttons["stop"].set_enabled(True)
            self._buttons["save"].set_enabled(False)
            self._buttons["discard"].set_enabled(False)
        elif samples > 0:
            self._buttons["start"].set_enabled(True)
            self._buttons["stop"].set_enabled(False)
            self._buttons["save"].set_enabled(True)
            self._buttons["discard"].set_enabled(True)
        else:
            self._buttons["start"].set_enabled(True)
            self._buttons["stop"].set_enabled(False)
            self._buttons["save"].set_enabled(False)
            self._buttons["discard"].set_enabled(False)


