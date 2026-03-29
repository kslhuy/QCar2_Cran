#!/usr/bin/env python3
"""
Live Plot-All viewer for distributed observer data.

This script no longer reads CSV files. It subscribes to Ground Station
WebSocket telemetry and plots in real time while vehicles are running.

Requirements:
- Ground Station WebSocket server is running (default ws://127.0.0.1:8080)
- Vehicle telemetry includes distributed-observer debug fields

Usage:
    python plot_all_observer_viewer.py
    python plot_all_observer_viewer.py --ws-url ws://127.0.0.1:8080
    python plot_all_observer_viewer.py --refresh-ms 300 --time-window 20
"""

from __future__ import annotations

import argparse
import asyncio
import json
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import tkinter as tk


STATE_NAMES = ("position", "velocity", "acceleration")
NUMERIC_FIELD_PREFIXES = (
    "x_vec_after_",
    "x_vec_before_",
    "dynamics_",
    "measurement_",
    "consensus_",
    "true_position_",
    "true_velocity_",
    "true_acceleration_",
)
NUMERIC_EXACT_KEYS = (
    "x",
    "y",
    "th",
    "v",
    "u",
    "delta",
    "acceleration",
    "control_input",
)

TRUE_ROW_VEHICLES = (0, 1, 2, 3)
OBSERVER_ROWS = (1, 2, 3)
COMPONENT_COLORS = {
    1: "b",
    2: "r",
    3: "g",
}
VEHICLE_STYLE = {
    0: ("k", "--"),
    1: ("b", "-"),
    2: ("r", "-"),
    3: ("g", "-"),
}


@dataclass
class ViewerConfig:
    ws_url: str = "ws://127.0.0.1:8080"
    refresh_ms: int = 300
    time_window: Optional[float] = 20.0
    max_points: int = 6000


class LiveObserverBuffer:
    """Thread-safe in-memory buffer keyed by observer vehicle id."""

    def __init__(self, max_points: int = 6000):
        self.max_points = max_points
        self._lock = threading.Lock()
        self._times: Dict[int, Deque[float]] = {}
        self._fields: Dict[int, Dict[str, Deque[float]]] = {}

    def append(self, observer_id: int, t: float, values: Dict[str, float]) -> None:
        with self._lock:
            if observer_id not in self._times:
                self._times[observer_id] = deque(maxlen=self.max_points)
                self._fields[observer_id] = {}

            self._times[observer_id].append(float(t))
            curr_len = len(self._times[observer_id])

            field_map = self._fields[observer_id]
            for key, value in values.items():
                if key not in field_map:
                    # Backfill historical samples so new fields align with existing time axis.
                    field_map[key] = deque([np.nan] * (curr_len - 1), maxlen=self.max_points)
                field_map[key].append(float(value))

            # Keep lengths aligned by appending nan to fields not present in this message.
            for key, series in field_map.items():
                if len(series) < curr_len:
                    series.append(np.nan)

    def snapshot(self) -> Dict[int, Dict[str, np.ndarray]]:
        with self._lock:
            snap: Dict[int, Dict[str, np.ndarray]] = {}
            for observer_id, t_deque in self._times.items():
                snap[observer_id] = {
                    "time": np.asarray(t_deque, dtype=float),
                    "fields": {
                        key: np.asarray(series, dtype=float)
                        for key, series in self._fields[observer_id].items()
                    },
                }
            return snap


class WsSubscriber:
    """Background WebSocket subscriber that feeds LiveObserverBuffer."""

    def __init__(self, ws_url: str, buffer: LiveObserverBuffer):
        self.ws_url = ws_url
        self.buffer = buffer
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def _run_forever(self) -> None:
        asyncio.run(self._main_loop())

    async def _main_loop(self) -> None:
        try:
            import websockets
        except ImportError:
            return

        while not self._stop_event.is_set():
            try:
                async with websockets.connect(self.ws_url, ping_interval=20, ping_timeout=20) as ws:
                    while not self._stop_event.is_set():
                        raw = await ws.recv()
                        if not isinstance(raw, str):
                            continue
                        self._handle_json_message(raw)
            except Exception:
                await asyncio.sleep(1.0)

    def _handle_json_message(self, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except Exception:
            return

        observer_id = self._extract_observer_id(msg)
        if observer_id is None:
            return

        t = self._extract_time(msg)
        values: Dict[str, float] = {}

        for key, value in msg.items():
            if key == "time":
                continue
            if not key.startswith(NUMERIC_FIELD_PREFIXES) and key not in NUMERIC_EXACT_KEYS:
                continue
            try:
                values[key] = float(value)
            except Exception:
                continue

        # Nothing relevant in this packet.
        if not values:
            return

        self.buffer.append(observer_id, t, values)

    @staticmethod
    def _extract_observer_id(msg: Dict) -> Optional[int]:
        vehicle_label = msg.get("vehicle_id")
        if isinstance(vehicle_label, str):
            match = re.search(r"(\d+)$", vehicle_label)
            if match:
                return int(match.group(1))

        car_id = msg.get("car_id")
        if isinstance(car_id, int):
            return car_id

        return None

    @staticmethod
    def _extract_time(msg: Dict) -> float:
        # Prefer vehicle elapsed time for stable axis.
        try:
            if "time" in msg:
                return float(msg["time"])
        except Exception:
            pass
        return time.time()


class PlotAllLiveApp:
    """Tk launcher + matplotlib live viewer."""

    def __init__(self, cfg: ViewerConfig):
        self.cfg = cfg
        self.buffer = LiveObserverBuffer(max_points=cfg.max_points)
        self.subscriber = WsSubscriber(cfg.ws_url, self.buffer)

        self.root = tk.Tk()
        self.root.title("Live Distributed Observer Viewer")
        self.root.geometry("420x160")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.status_var = tk.StringVar(value=f"WS: {cfg.ws_url}")
        self.figure = None
        self.anim = None

        self._build_ui()
        self.subscriber.start()

    def _build_ui(self) -> None:
        frame = tk.Frame(self.root, padx=14, pady=14)
        frame.pack(fill="both", expand=True)

        btn = tk.Button(frame, text="Plot All", width=26, height=2, command=self._open_viewer)
        btn.pack(pady=(0, 10))

        label = tk.Label(frame, textvariable=self.status_var, anchor="w", justify="left")
        label.pack(fill="x")

        hint = tk.Label(
            frame,
            text=(
                "Need fields like x_vec_before_p* / true_position_* in telemetry.\n"
                "If blank, verify Ground Station and vehicle streaming pipeline."
            ),
            anchor="w",
            justify="left",
            fg="#555555",
        )
        hint.pack(fill="x", pady=(6, 0))

    def _open_viewer(self) -> None:
        if self.figure is not None and plt.fignum_exists(self.figure.number):
            try:
                self.figure.canvas.manager.window.lift()
            except Exception:
                pass
            self.status_var.set("Viewer already open")
            return

        self.figure = plt.figure(figsize=(14, 8))
        self.figure.suptitle("Live: True State vs Distributed Observer Estimates", fontsize=12)
        self.anim = FuncAnimation(
            self.figure,
            self._update_plot,
            interval=self.cfg.refresh_ms,
            blit=False,
            cache_frame_data=False,
        )
        plt.show(block=False)
        self.status_var.set("Viewer running")

    def _update_plot(self, _frame_idx: int):
        snap = self.buffer.snapshot()
        if not snap:
            self._draw_message("Waiting for websocket data...")
            return

        # Fixed 4x3 layout as requested.
        n_rows = 4
        n_cols = 3
        self.figure.clf()
        axes = self.figure.subplots(n_rows, n_cols, squeeze=False)

        self._plot_true_state_row(axes[0], snap)
        self._plot_observer_estimation_row(axes[1], snap, observer_id=1)
        self._plot_observer_estimation_row(axes[2], snap, observer_id=2)
        self._plot_observer_estimation_row(axes[3], snap, observer_id=3)

        for r in range(n_rows):
            for c in range(n_cols):
                axes[r][c].set_xlabel("Time [s]")
                axes[r][c].set_xlim(left=0.0)

        self.figure.tight_layout(rect=(0.02, 0.02, 0.98, 0.95))
        self.figure.canvas.draw_idle()

        active_observers = sorted(k for k in snap.keys() if k in OBSERVER_ROWS)
        self.status_var.set(f"Observers active: {active_observers} | Refresh: {self.cfg.refresh_ms} ms")

    def _plot_true_state_row(self, row_axes, snap: Dict[int, Dict[str, np.ndarray]]) -> None:
        ax_pos, ax_vel, ax_u = row_axes

        ax_pos.set_title("True Position")
        ax_pos.set_ylabel("Position")
        ax_pos.grid(True, alpha=0.3)

        ax_vel.set_title("True Velocity")
        ax_vel.set_ylabel("Velocity")
        ax_vel.set_ylim(0.0, 1.0)
        ax_vel.grid(True, alpha=0.3)

        ax_u.set_title("Control Input")
        ax_u.set_ylabel("Control Input")
        ax_u.set_ylim(0.0, 0.5)
        ax_u.grid(True, alpha=0.3)

        for veh_id in TRUE_ROW_VEHICLES:
            color, style = VEHICLE_STYLE.get(veh_id, ("k", "-"))

            t_pos, y_pos = self._get_vehicle_series(
                snap,
                veh_id,
                candidates=(f"true_position_{veh_id}", "x"),
            )
            if t_pos.size > 0:
                ax_pos.plot(t_pos, y_pos, color=color, linestyle=style, linewidth=1.8, label=f"V{veh_id}")

            t_vel, y_vel = self._get_vehicle_series(
                snap,
                veh_id,
                candidates=(f"true_velocity_{veh_id}", "v"),
            )
            if t_vel.size > 0:
                ax_vel.plot(t_vel, y_vel, color=color, linestyle=style, linewidth=1.8, label=f"V{veh_id}")

            t_u, y_u = self._get_vehicle_series(
                snap,
                veh_id,
                candidates=("u", "control_input", f"collective_control_{veh_id + 1}"),
            )
            if t_u.size > 0:
                ax_u.plot(t_u, y_u, color=color, linestyle=style, linewidth=1.8, label=f"V{veh_id}")

        for ax in (ax_pos, ax_vel, ax_u):
            handles, labels = ax.get_legend_handles_labels()
            if handles:
                ax.legend(fontsize=8, loc="upper right")

    def _plot_observer_estimation_row(self, row_axes, snap: Dict[int, Dict[str, np.ndarray]], observer_id: int) -> None:
        ax_p, ax_v, ax_a = row_axes

        ax_p.set_title(f"Observer {observer_id} - x_vec_before")
        ax_p.set_ylabel("Estimation of pi - p0 +di0")
        ax_p.grid(True, alpha=0.3)

        ax_v.set_title(f"Observer {observer_id} - x_vec_before")
        ax_v.set_ylabel("Estimation of vi - v0")
        ax_v.grid(True, alpha=0.3)

        ax_a.set_title(f"Observer {observer_id} - x_vec_before")
        ax_a.set_ylabel("Estimation of ai - a0")
        ax_a.grid(True, alpha=0.3)

        if observer_id not in snap:
            for ax in (ax_p, ax_v, ax_a):
                ax.text(0.5, 0.5, f"Observer {observer_id} no data", transform=ax.transAxes, ha="center", va="center", fontsize=9)
            return

        fields = snap[observer_id]["fields"]
        t_ref = snap[observer_id]["time"]

        for comp_idx in (1, 2, 3):
            color = COMPONENT_COLORS[comp_idx]

            key_p = f"x_vec_before_p{comp_idx}"
            key_v = f"x_vec_before_v{comp_idx}"
            key_a = f"x_vec_before_a{comp_idx}"

            if key_p in fields:
                t, y = self._window_series(t_ref, fields[key_p])
                if t.size > 0:
                    ax_p.plot(t, y, color=color, linestyle="-", linewidth=1.6, label=key_p)

            if key_v in fields:
                t, y = self._window_series(t_ref, fields[key_v])
                if t.size > 0:
                    ax_v.plot(t, y, color=color, linestyle="-", linewidth=1.6, label=key_v)

            if key_a in fields:
                t, y = self._window_series(t_ref, fields[key_a])
                if t.size > 0:
                    ax_a.plot(t, y, color=color, linestyle="-", linewidth=1.6, label=key_a)

        for ax in (ax_p, ax_v, ax_a):
            handles, labels = ax.get_legend_handles_labels()
            if handles:
                ax.legend(fontsize=7, loc="upper right")

    def _get_vehicle_series(
        self,
        snap: Dict[int, Dict[str, np.ndarray]],
        vehicle_id: int,
        candidates: Tuple[str, ...],
    ) -> Tuple[np.ndarray, np.ndarray]:
        # Prefer same vehicle stream first.
        source_ids = [vehicle_id] + [sid for sid in sorted(snap.keys()) if sid != vehicle_id]

        for sid in source_ids:
            if sid not in snap:
                continue
            t = snap[sid]["time"]
            fields = snap[sid]["fields"]

            for key in candidates:
                if key in fields:
                    return self._window_series(t, fields[key])

        return np.array([]), np.array([])

    def _plot_one_axis(
        self,
        ax,
        snap: Dict[int, Dict[str, np.ndarray]],
        observer_ids: List[int],
        target_id: int,
        state_name: str,
    ) -> None:
        ax.grid(True, alpha=0.3)

        true_candidates = self._true_field_candidates(target_id, state_name)
        est_field = self._estimate_field(target_id, state_name)

        # Plot one true line from first observer that has it.
        true_plotted = False
        for obs_id in observer_ids:
            fields = snap[obs_id]["fields"]
            t = snap[obs_id]["time"]

            true_key = next((k for k in true_candidates if k in fields), None)
            if true_key is None:
                continue

            y = fields[true_key]
            t2, y2 = self._window_series(t, y)
            if t2.size > 0:
                ax.plot(t2, y2, "k-", linewidth=2.0, label="true")
                true_plotted = True
            break

        # Plot estimate from each observer.
        for obs_id in observer_ids:
            fields = snap[obs_id]["fields"]
            t = snap[obs_id]["time"]
            if est_field not in fields:
                continue

            y = fields[est_field]
            t2, y2 = self._window_series(t, y)
            if t2.size > 0:
                ax.plot(t2, y2, "--", linewidth=1.2, label=f"obs_{obs_id}")

        if not true_plotted:
            ax.text(0.5, 0.5, "true unavailable", transform=ax.transAxes, ha="center", va="center", fontsize=8)

    def _draw_message(self, msg: str) -> None:
        if self.figure is None:
            return
        self.figure.clf()
        ax = self.figure.add_subplot(111)
        ax.text(0.5, 0.5, msg, ha="center", va="center", fontsize=12)
        ax.set_axis_off()
        self.figure.canvas.draw_idle()

    def _window_series(self, t: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if t.size == 0 or y.size == 0:
            return np.array([]), np.array([])

        # Be defensive against occasional length mismatch from asynchronous updates.
        if t.size != y.size:
            n = min(t.size, y.size)
            t = t[:n]
            y = y[:n]

        mask = ~(np.isnan(t) | np.isnan(y))
        t = t[mask]
        y = y[mask]
        if t.size == 0:
            return t, y

        # Force time axis to start from zero for every plotted series.
        t = t - t[0]

        if self.cfg.time_window is not None:
            t_end = t[-1]
            keep = t >= (t_end - self.cfg.time_window)
            t = t[keep]
            y = y[keep]

        return t, y

    def _on_close(self) -> None:
        self.subscriber.stop()
        try:
            plt.close("all")
        except Exception:
            pass
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> ViewerConfig:
    parser = argparse.ArgumentParser(description="Real-time Plot-All viewer via WebSocket")
    parser.add_argument("--ws-url", type=str, default="ws://127.0.0.1:8080", help="Ground Station websocket URL")
    parser.add_argument("--refresh-ms", type=int, default=300, help="Plot refresh interval in ms")
    parser.add_argument("--time-window", type=float, default=20.0, help="Trailing time window in seconds (<=0 for full)")
    parser.add_argument("--max-points", type=int, default=6000, help="Max samples buffered per observer")
    args = parser.parse_args()

    tw = None if args.time_window <= 0 else float(args.time_window)
    return ViewerConfig(
        ws_url=args.ws_url,
        refresh_ms=max(100, int(args.refresh_ms)),
        time_window=tw,
        max_points=max(1000, int(args.max_points)),
    )


def main() -> None:
    cfg = parse_args()
    app = PlotAllLiveApp(cfg)
    app.run()


if __name__ == "__main__":
    main()
