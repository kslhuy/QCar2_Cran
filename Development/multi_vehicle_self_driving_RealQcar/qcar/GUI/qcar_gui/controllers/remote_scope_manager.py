"""
Remote Scope Manager - Manages real-time scope data reception from vehicles

This module handles receiving, buffering, and visualizing scope data
streamed from vehicles to the Ground Station.

Components:
- ScopeDataBuffer: Ring buffer for storing incoming scope samples
- RemoteScopeManager: Manages multiple vehicle scope streams
- RemoteScopeViewer: Matplotlib-based real-time visualization
"""

import threading
import multiprocessing
import time
import struct
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass, field
from collections import deque
import numpy as np

# Import unpacking utilities from vehicle-side module
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", ".."))
from scope_data_streamer import (
    unpack_scope_data,
    PRESET_FIELDS,
    DEFAULT_FIELDS,
    FLEET_FIELDS,
    build_fleet_fields,
)


# ==============================================================================
# Scope Data Buffer
# ==============================================================================


class ScopeDataBuffer:
    """
    Thread-safe ring buffer for storing incoming scope data samples.

    Maintains separate time and value arrays for efficient plotting.
    """

    def __init__(self, max_samples: int = 6000, field_names: List[str] = None):
        """
        Initialize the buffer.

        Args:
            max_samples: Maximum samples to store (default 2 min at 50Hz)
            field_names: List of field names to store
        """
        self.max_samples = max_samples
        self.field_names = field_names if field_names else DEFAULT_FIELDS.copy()
        self.num_fields = len(self.field_names)

        # Pre-allocate numpy arrays for efficiency
        self.timestamps = np.zeros(max_samples)
        self.data = np.zeros((max_samples, self.num_fields))

        # Current write position and sample count
        self.write_pos = 0
        self.sample_count = 0

        # Thread safety
        self._lock = threading.Lock()

        # Statistics
        self.samples_received = 0
        self.start_time = time.time()

    def add_sample(self, timestamp: float, values: List[float]):
        """
        Add a sample to the buffer.

        Args:
            timestamp: Sample timestamp
            values: List of field values
        """
        with self._lock:
            # Store timestamp
            self.timestamps[self.write_pos] = timestamp

            # Store values (pad with zeros if needed)
            for i, v in enumerate(values[: self.num_fields]):
                self.data[self.write_pos, i] = v

            # Advance write position
            self.write_pos = (self.write_pos + 1) % self.max_samples
            self.sample_count = min(self.sample_count + 1, self.max_samples)
            self.samples_received += 1

    def get_data(self, field_name: str, last_n: Optional[int] = None) -> tuple:
        """
        Get time and value arrays for a field.

        Args:
            field_name: Name of field to get
            last_n: Only return last N samples (None = all)

        Returns:
            Tuple of (timestamps, values) numpy arrays
        """
        if field_name not in self.field_names:
            return np.array([]), np.array([])

        field_idx = self.field_names.index(field_name)

        with self._lock:
            if self.sample_count == 0:
                return np.array([]), np.array([])

            # Get valid range
            n = self.sample_count if last_n is None else min(last_n, self.sample_count)

            if self.sample_count >= self.max_samples:
                # Buffer is full, wrap-around
                start = (self.write_pos - n) % self.max_samples
                if start < self.write_pos:
                    t = self.timestamps[start : self.write_pos].copy()
                    v = self.data[start : self.write_pos, field_idx].copy()
                else:
                    t = np.concatenate(
                        [self.timestamps[start:], self.timestamps[: self.write_pos]]
                    )
                    v = np.concatenate(
                        [
                            self.data[start:, field_idx],
                            self.data[: self.write_pos, field_idx],
                        ]
                    )
            else:
                # Buffer not full yet
                end = min(self.sample_count, self.write_pos)
                start = max(0, end - n)
                t = self.timestamps[start:end].copy()
                v = self.data[start:end, field_idx].copy()

            return t, v

    def get_latest_values(self) -> Dict[str, float]:
        """Get the most recent values for all fields."""
        with self._lock:
            if self.sample_count == 0:
                return {}

            latest_idx = (self.write_pos - 1) % self.max_samples
            return {
                name: float(self.data[latest_idx, i])
                for i, name in enumerate(self.field_names)
            }

    def get_statistics(self) -> Dict[str, Any]:
        """Get buffer statistics."""
        duration = time.time() - self.start_time
        return {
            "samples_received": self.samples_received,
            "buffer_used": self.sample_count,
            "buffer_max": self.max_samples,
            "fill_percent": (self.sample_count / self.max_samples) * 100,
            "receive_rate_hz": self.samples_received / duration if duration > 0 else 0,
            "duration_s": duration,
        }

    def clear(self):
        """Clear the buffer."""
        with self._lock:
            self.timestamps.fill(0)
            self.data.fill(0)
            self.write_pos = 0
            self.sample_count = 0


# ==============================================================================
# Remote Scope Manager
# ==============================================================================


class RemoteScopeManager:
    """
    Manages real-time scope data reception from multiple vehicles.

    Provides buffering, statistics, and viewer management.
    """

    def __init__(self):
        """Initialize the scope manager."""
        self.vehicle_buffers: Dict[int, ScopeDataBuffer] = {}
        self.vehicle_field_names: Dict[int, List[str]] = {}
        self.viewers: Dict[int, "RemoteScopeViewer"] = {}

        # Streaming state
        self._streaming_cars: set = set()

        # Statistics
        self.total_packets = 0
        self.parse_errors = 0

        # Lock for thread safety
        self._lock = threading.Lock()

    def is_streaming(self, car_id: int) -> bool:
        """Check if a car is currently streaming scope data."""
        return car_id in self._streaming_cars

    def start_stream(self, car_id: int, preset_names: List[str] = None, fleet_size: int = None):
        """
        Start receiving scope data from a vehicle.

        Args:
            car_id: Vehicle ID
            preset_names: List of presets being streamed
            fleet_size: Actual fleet size for dynamic fleet field generation
        """
        with self._lock:
            # Build field list from presets
            field_names = []
            if preset_names:
                for preset in preset_names:
                    if preset == 'fleet_state':
                        # Use dynamic fleet fields based on actual fleet size
                        n = fleet_size if fleet_size and fleet_size > 0 else 2
                        for field in build_fleet_fields(n):
                            if field not in field_names:
                                field_names.append(field)
                    elif preset in PRESET_FIELDS:
                        for field in PRESET_FIELDS[preset]:
                            if field not in field_names:
                                field_names.append(field)

            if not field_names:
                field_names = DEFAULT_FIELDS.copy()

            # Create buffer if needed
            if car_id not in self.vehicle_buffers:
                self.vehicle_buffers[car_id] = ScopeDataBuffer(
                    max_samples=6000,  # 2 min at 50Hz
                    field_names=field_names,
                )
            else:
                self.vehicle_buffers[car_id].clear()

            self.vehicle_field_names[car_id] = field_names
            self._streaming_cars.add(car_id)

            print(
                f"[RemoteScopeManager] Started stream for Car {car_id}: {field_names}"
            )

    def stop_stream(self, car_id: int):
        """Stop receiving scope data from a vehicle."""
        with self._lock:
            self._streaming_cars.discard(car_id)

            # Close viewer if open
            if car_id in self.viewers:
                self.viewers[car_id].stop()
                del self.viewers[car_id]

            print(f"[RemoteScopeManager] Stopped stream for Car {car_id}")

    def receive_scope_data(self, car_id: int, payload: str):
        """
        Process incoming scope data packet.

        Args:
            car_id: Vehicle ID
            payload: Hex-encoded scope data
        """
        try:
            if car_id not in self._streaming_cars:
                return

            # Decode hex payload
            packet = bytes.fromhex(payload)

            # Unpack data
            result = unpack_scope_data(packet)
            if result is None:
                self.parse_errors += 1
                return

            # Add to buffer
            with self._lock:
                if car_id in self.vehicle_buffers:
                    self.vehicle_buffers[car_id].add_sample(
                        result["timestamp"], result["values"]
                    )

            self.total_packets += 1

        except Exception as e:
            self.parse_errors += 1
            print(f"[RemoteScopeManager] Error processing scope data: {e}")

    def update_vehicle_info(self, car_id: int, info: Dict[str, Any]):
        """
        Update vehicle information (e.g. node sequence) from telemetry.

        Args:
            car_id: Vehicle ID
            info: Dictionary containing vehicle info
        """
        if car_id in self.viewers:
            self.viewers[car_id].update_info(info)

    def get_buffer(self, car_id: int) -> Optional[ScopeDataBuffer]:
        """Get the buffer for a vehicle."""
        return self.vehicle_buffers.get(car_id)

    def open_viewer(self, car_id: int, preset_names: List[str] = None):
        """
        Open a scope viewer window for a vehicle.

        Args:
            car_id: Vehicle ID
            preset_names: Presets to visualize
        """
        if car_id not in self.vehicle_buffers:
            print(f"[RemoteScopeManager] No buffer for Car {car_id}")
            return

        with self._lock:
            # Close existing viewer if any
            if car_id in self.viewers:
                self.viewers[car_id].stop()

            # Create new viewer
            field_names = self.vehicle_field_names.get(car_id, DEFAULT_FIELDS)
            self.viewers[car_id] = RemoteScopeViewer(
                car_id=car_id,
                buffer=self.vehicle_buffers[car_id],
                field_names=field_names,
                fps=20,
            )
            self.viewers[car_id].start()

    def close_viewer(self, car_id: int):
        """Close the viewer for a vehicle."""
        with self._lock:
            if car_id in self.viewers:
                self.viewers[car_id].stop()
                del self.viewers[car_id]

    def get_statistics(self) -> Dict[str, Any]:
        """Get overall statistics."""
        stats = {
            "total_packets": self.total_packets,
            "parse_errors": self.parse_errors,
            "streaming_cars": list(self._streaming_cars),
            "active_viewers": list(self.viewers.keys()),
        }

        # Add per-vehicle stats
        for car_id, buffer in self.vehicle_buffers.items():
            stats[f"car_{car_id}"] = buffer.get_statistics()

        return stats

    def shutdown(self):
        """
        Shutdown all viewers and cleanup resources.

        Call this when the application is closing to ensure all
        multiprocessing resources are properly terminated.
        """
        print("[RemoteScopeManager] Shutting down all viewers...")

        with self._lock:
            # Stop all viewers
            for car_id in list(self.viewers.keys()):
                try:
                    self.viewers[car_id].stop()
                except Exception as e:
                    print(f"[RemoteScopeManager] Error stopping viewer {car_id}: {e}")

            self.viewers.clear()
            self._streaming_cars.clear()

        print("[RemoteScopeManager] Shutdown complete")


# ==============================================================================
# Remote Scope Viewer
# ==============================================================================


class RemoteScopeViewer:
    """
    Real-time scope visualization window for Ground Station.

    Uses multiprocessing to run matplotlib in a separate process,
    avoiding conflicts with the main tkinter GUI.
    """

    def __init__(
        self,
        car_id: int,
        buffer: ScopeDataBuffer,
        field_names: List[str],
        fps: int = 20,
    ):
        """
        Initialize the viewer.

        Args:
            car_id: Vehicle ID
            buffer: Data buffer to read from
            field_names: Fields to plot
            fps: Update rate
        """
        self.car_id = car_id
        self.buffer = buffer
        self.field_names = field_names
        self.fps = fps
        self.time_window = 15.0  # Show last 20 seconds

        # Process control
        self.running = False
        self._process = None
        self._data_queue = None
        self._stop_event = None

    def start(self):
        """Start the viewer in a background process."""
        if self.running:
            return

        import multiprocessing as mp

        self.running = True
        self._stop_event = mp.Event()
        self._data_queue = mp.Queue(maxsize=200)

        # Start plot process
        self._process = mp.Process(
            target=_run_scope_plot_process,
            args=(
                self.car_id,
                self.field_names,
                self._data_queue,
                self._stop_event,
                self.fps,
                self.time_window,
            ),
            daemon=True,
        )
        self._process.start()

        # Start data feeder thread
        self._feeder_thread = threading.Thread(
            target=self._feed_data_to_process, daemon=True
        )
        self._feeder_thread.start()

        print(f"[ScopeViewer] Started viewer for Car {self.car_id}")

    def _feed_data_to_process(self):
        """Feed data from buffer to the plot process."""
        import time

        last_feed_time = 0
        feed_interval = 1.0 / self.fps
        last_sent_count = 0

        while self.running and not self._stop_event.is_set():
            try:
                current_time = time.time()
                if current_time - last_feed_time >= feed_interval:
                    current_count = self.buffer.sample_count
                    if current_count > last_sent_count:
                        new_samples = min(current_count - last_sent_count, self.buffer.max_samples)
                        
                        data_packet = {"type": "incremental", "timestamp": current_time, "fields": {}}

                        for field in self.field_names:
                            t, v = self.buffer.get_data(
                                field, last_n=new_samples
                            )
                            if len(t) > 0:
                                data_packet["fields"][field] = {"t": t.tolist(), "v": v.tolist()}

                        # Send to process (non-blocking)
                        try:
                            self._data_queue.put_nowait(data_packet)
                            last_sent_count = current_count
                        except:
                            pass  # Queue full, skip this update

                    last_feed_time = current_time

                time.sleep(0.005)  # Small sleep to prevent busy-waiting

            except Exception as e:
                print(f"[ScopeViewer] Data feed error: {e}")
                break

    def stop(self):
        """Stop the viewer."""
        self.running = False

        if self._stop_event:
            self._stop_event.set()

        if self._process and self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)

        print(f"[ScopeViewer] Stopped viewer for Car {self.car_id}")

    def update_info(self, info: Dict[str, Any]):
        """Update viewer info (thread-safe)."""
        if self.running and self._data_queue:
            try:
                # Send info packet with special 'info' key
                self._data_queue.put_nowait({"info": info})
            except:
                pass


def _run_scope_plot_process(
    car_id, field_names, data_queue, stop_event, fps, time_window
):
    """
    Run the matplotlib plot in a separate process.

    Layouts match plot_scope_data.py (Interactive Mode):
    - Uses GridSpec for complex layouts (Trajectory X-Y, etc.)
    """
    import matplotlib

    # Try Qt5Agg first (more reliable on Ubuntu), fallback to TkAgg, then Agg
    backends = ["TkAgg", "Qt5Agg", "GTK3Agg", "Agg"]
    for backend in backends:
        try:
            matplotlib.use(backend)
            # Test if it actually works by importing pyplot
            import matplotlib.pyplot as plt
            break
        except Exception:
            continue
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    import numpy as np

    # Detect if this is Fleet or Local data based on field names
    is_fleet = any(f.startswith("fleet_") for f in field_names)

    if is_fleet:
        fig, lines, axes = _create_fleet_layout(plt, car_id, field_names)
    else:
        fig, lines, axes = _create_local_layout(plt, car_id, field_names)

    # Path visualization data from vehicle
    path_viz_data = {}

    # Latest data storage
    latest_data = {}

    # Path generation
    roadmap = None
    path_generated = False
    last_node_sequence = None
    
    # Store accumulated data for incremental updates
    from collections import deque
    # Assuming max 50Hz for 15s = 750 points. 1000 is safe.
    max_pts = 1000
    accumulated_data = {
        f: {"t": deque(maxlen=max_pts), "v": deque(maxlen=max_pts)}
        for f in field_names
    }
    # Add accumulation fields for X, Y, and GPS components
    for extra_field in ["x", "y", "x_gps", "y_gps", "theta"]:
        if extra_field not in accumulated_data:
            accumulated_data[extra_field] = {"t": deque(maxlen=max_pts), "v": deque(maxlen=max_pts)}

    def update_path(node_sequence):
        nonlocal roadmap, path_generated, last_node_sequence

        # Avoid re-generating same path
        if last_node_sequence == node_sequence:
            return

        last_node_sequence = node_sequence

        try:
            # Import SDCSRoadMap here to avoid dependency issues in main process if not needed
            sys.path.append(
                os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
            )
            from hal.products.mats import SDCSRoadMap

            if roadmap is None:
                roadmap = SDCSRoadMap(leftHandTraffic=False, useSmallMap=False)

            # Generate path
            # TODO : Need to more dynamic to know virtual or real path
            scale = 0.975
            waypoints = roadmap.generate_path(node_sequence)
            waypoints *= scale

            # Plot path
            if "trajectory_ref" in lines:
                lines["trajectory_ref"].set_data(waypoints[0, :], waypoints[1, :])
                path_generated = True
                print(f"[ScopePlot] Generated path with {waypoints.shape[1]} points")

        except Exception as e:
            print(f"[ScopePlot] Error generating path: {e}")

    def update(frame):
        nonlocal latest_data, path_viz_data

        # Check if we should stop
        if stop_event.is_set():
            plt.close(fig)
            return list(lines.values())

        # Get latest data from queue
        has_new_data = False
        try:
            while not data_queue.empty():
                packet = data_queue.get_nowait()

                # Check for info packet
                if "info" in packet:
                    info = packet["info"]
                    if "node_sequence" in info and info["node_sequence"]:
                        update_path(info["node_sequence"])
                    if "path_viz" in info:
                        path_viz_data.update(info["path_viz"])
                elif packet.get("type") == "incremental":
                    fields = packet.get("fields", {})
                    for field, data in fields.items():
                        if field in accumulated_data:
                            accumulated_data[field]["t"].extend(data["t"])
                            accumulated_data[field]["v"].extend(data["v"])
                    has_new_data = True
        except:
            pass

        if not has_new_data and not path_generated:
            return list(lines.values())

        # Update standard time-series lines
        for field, line in lines.items():
            # Skip special trajectory lines which are handled below
            if (
                field.startswith("traj_")
                or field == "trajectory"
                or field == "trajectory_gps"
                or field == "trajectory_ref"
            ):
                continue

            if field in accumulated_data and accumulated_data[field]["t"]:
                line.set_data(list(accumulated_data[field]["t"]), list(accumulated_data[field]["v"]))

        # Update Local Trajectory (X vs Y)
        if not is_fleet:
            if "trajectory" in lines and accumulated_data["x"]["v"] and accumulated_data["y"]["v"]:
                x = list(accumulated_data["x"]["v"])
                y = list(accumulated_data["y"]["v"])
                min_len = min(len(x), len(y))
                if min_len > 0:
                    lines["trajectory"].set_data(x[:min_len], y[:min_len])

                    # Update heading arrow using latest theta
                    if accumulated_data.get("theta") and accumulated_data["theta"]["v"]:
                        last_theta = accumulated_data["theta"]["v"][-1]
                        last_x = x[min_len - 1]
                        last_y = y[min_len - 1]
                        arrow_len = 0.15
                        if "heading_arrow" in lines:
                            q = lines["heading_arrow"]
                            q.set_offsets(np.array([[last_x, last_y]]))
                            import math
                            q.set_UVC(
                                [arrow_len * math.cos(last_theta)],
                                [arrow_len * math.sin(last_theta)],
                            )

            if (
                "trajectory_gps" in lines
                and accumulated_data.get("x_gps") and accumulated_data["x_gps"]["v"]
                and accumulated_data.get("y_gps") and accumulated_data["y_gps"]["v"]
            ):
                x = list(accumulated_data["x_gps"]["v"])
                y = list(accumulated_data["y_gps"]["v"])
                min_len = min(len(x), len(y))
                if min_len > 0:
                    lines["trajectory_gps"].set_data(x[:min_len], y[:min_len])

        # Update Fleet Trajectories (V0, V1, V2...)
        else:
            # We assume fleet_x_0, fleet_y_0, etc.
            # Find all vehicle indices from field names
            fleet_indices = set()
            for f in field_names:
                parts = f.split("_")
                if len(parts) == 3 and parts[0] == "fleet" and parts[1] == "x":
                    try:
                        fleet_indices.add(int(parts[2]))
                    except:
                        pass

            for i in fleet_indices:
                line_key = f"traj_{i}"
                fx = f"fleet_x_{i}"
                fy = f"fleet_y_{i}"

                if line_key in lines and fx in accumulated_data and accumulated_data[fx]["v"] and fy in accumulated_data and accumulated_data[fy]["v"]:
                    x = list(accumulated_data[fx]["v"])
                    y = list(accumulated_data[fy]["v"])
                    min_len = min(len(x), len(y))
                    if min_len > 0:
                        lines[line_key].set_data(x[:min_len], y[:min_len])
                        # Update head
                        head_key = f"traj_head_{i}"
                        if head_key in lines:
                            lines[head_key].set_data([x[min_len - 1]], [y[min_len - 1]])

        # ---- Update path visualization (global / local / obstacles) ----
        if path_viz_data:
            # if "global_path_x" in path_viz_data and "global_path_y" in path_viz_data:
            #     if "global_path" in lines:
            #         lines["global_path"].set_data(
            #             path_viz_data["global_path_x"],
            #             path_viz_data["global_path_y"],
            #         )
            if "local_path_x" in path_viz_data and "local_path_y" in path_viz_data:
                if "local_path" in lines:
                    lines["local_path"].set_data(
                        path_viz_data["local_path_x"],
                        path_viz_data["local_path_y"],
                    )
            obstacles = path_viz_data.get("obstacles", [])
            if "obstacles" in lines:
                if obstacles:
                    ox = [o[0] for o in obstacles]
                    oy = [o[1] for o in obstacles]
                    lines["obstacles"].set_data(ox, oy)
                else:
                    lines["obstacles"].set_data([], [])

        # Update axis limits
        for ax in axes.values():
            ax.relim()
            ax.autoscale_view()

        return list(lines.values())

    # Create animation
    ani = animation.FuncAnimation(
        fig, update, interval=1000 // fps, blit=False, cache_frame_data=False
    )

    # Show (blocks until window is closed)
    try:
        plt.show()
    except:
        pass


def _create_local_layout(plt, car_id, field_names):
    """
    Create Local with GridSpec (matching plot_scope_data.py).
    """
    fig = plt.figure(figsize=(12, 8))
    fig.suptitle(f"Local State Estimation - Car {car_id}", fontsize=12)

    # 3 rows, 3 cols
    gs = fig.add_gridspec(3, 3, height_ratios=[2, 1, 1], hspace=0.35, wspace=0.3)

    axes = {}
    lines = {}

    # 1. Trajectory (Left 2x2) - X vs Y
    ax_traj = fig.add_subplot(gs[0:2, 0:2])
    ax_traj.set_xlabel("X [m]")
    ax_traj.set_ylabel("Y [m]")
    ax_traj.set_title("Trajectory")
    ax_traj.grid(True, alpha=0.3)
    ax_traj.set_xlim(-5, 5)
    ax_traj.set_ylim(-5, 5)
    axes["trajectory"] = ax_traj

    # Reference Path (Planned) - now also labeled as "Global"  
    (line_ref,) = ax_traj.plot([], [], "k--", lw=1.5, alpha=0.6, label="Planned")
    lines["trajectory_ref"] = line_ref

    # # Global path from path_viz (green dashed)
    # (line_global,) = ax_traj.plot([], [], color="#2ca02c", ls="--", lw=1.8, alpha=0.7, label="Global Path")
    # lines["global_path"] = line_global

    # Local path from path_viz (orange solid, shows obstacle avoidance)
    (line_local,) = ax_traj.plot([], [], color="#ff7f0e", ls="-", lw=2.5, alpha=0.85, label="Local Path")
    lines["local_path"] = line_local

    # Obstacle markers (red circles)
    (obstacle_scatter,) = ax_traj.plot(
        [], [], "ro", ms=10, alpha=0.8, markerfacecolor="none",
        markeredgewidth=2.5, label="Obstacles", zorder=8,
    )
    lines["obstacles"] = obstacle_scatter

    # Est path
    (line,) = ax_traj.plot([], [], "b-", lw=2, label="Path Est")
    lines["trajectory"] = line
    # Head marker removed — heading arrow already indicates car position

    # Heading arrow (quiver) — initially empty
    heading_quiver = ax_traj.quiver(
        [], [], [], [], color="blue", scale=5, width=0.012,
        headwidth=4, headlength=5, zorder=11,
    )
    lines["heading_arrow"] = heading_quiver

    # GPS path
    if "x_gps" in field_names:
        (line_gps,) = ax_traj.plot([], [], "r.", markersize=3, label="GPS")
        lines["trajectory_gps"] = line_gps
    ax_traj.legend(loc="upper right", fontsize=7, ncol=2)

    # 2. Velocity (Top Right)
    ax_vel = fig.add_subplot(gs[0, 2])
    ax_vel.set_ylabel("Velocity [m/s]")
    ax_vel.grid(True, alpha=0.3)
    axes["velocity"] = ax_vel

    for f, c, s in [("velocity", "b", "-"), ("v_ref", "k", "--"), ("v_ref_actual", "r", "--")]:
        if f in field_names:
            (l,) = ax_vel.plot(
                [], [], color=c, linestyle=s, marker="." if s == "None" else "None", markersize=2, label=f
            )
            lines[f] = l
    ax_vel.legend(fontsize=8)

    # 3. Heading (Mid Right)
    ax_th = fig.add_subplot(gs[1, 2])
    ax_th.set_ylabel("Heading [rad]")
    ax_th.grid(True, alpha=0.3)
    axes["heading"] = ax_th

    # Estimated Heading
    if "theta" in field_names:
        (l,) = ax_th.plot(
            [], [], color="g", linestyle="None", marker=".", markersize=2, label="theta"
        )
        lines["theta"] = l

    # GPS Heading (Scatter)
    if "theta_gps" in field_names:
        (l,) = ax_th.plot([], [], "r.", markersize=2, label="theta_gps")
        lines["theta_gps"] = l
    ax_th.legend(fontsize=8)

    # 4. Controls (Bottom Left 2 cols)
    ax_ctrl = fig.add_subplot(gs[2, 0:2])
    ax_ctrl.set_ylabel("Control")
    ax_ctrl.set_xlabel("Time [s]")
    ax_ctrl.grid(True, alpha=0.3)
    axes["control"] = ax_ctrl

    for f, c in [("throttle", "g"), ("steering", "r")]:
        if f in field_names:
            (l,) = ax_ctrl.plot(
                [], [], color=c, linestyle="None", marker=".", markersize=2, label=f
            )
            lines[f] = l
    ax_ctrl.legend(fontsize=8)

    # 5. Acceleration Magnitude (Bottom Right)
    ax_acc = fig.add_subplot(gs[2, 2])
    ax_acc.set_ylabel("|a| [m/s^2]")
    ax_acc.set_xlabel("Time [s]")
    ax_acc.grid(True, alpha=0.3)
    axes["acceleration"] = ax_acc

    if "accel_magnitude" in field_names:
        (l,) = ax_acc.plot(
            [], [], color="#9467bd", linestyle="-", linewidth=1.5, label="|a|"
        )
        lines["accel_magnitude"] = l
        ax_acc.legend(fontsize=8)

    return fig, lines, axes


def _create_fleet_layout(plt, car_id, field_names):
    """
    Create Fleet layout with GridSpec.
    Dynamically creates subplots only for vehicles present in field_names.
    No trust subplot — only fleet estimation state (x, y, theta, v).
    """
    fig = plt.figure(figsize=(12, 10))
    fig.suptitle(f"Fleet State Estimation - Car {car_id}", fontsize=12)

    # 3 rows, 2 cols
    gs = fig.add_gridspec(3, 2, height_ratios=[2, 1, 1], hspace=0.35, wspace=0.3)

    axes = {}
    lines = {}

    # Identify vehicles dynamically from field names
    fleet_indices = set()
    for f in field_names:
        parts = f.split("_")
        if len(parts) == 3 and parts[0] == "fleet" and parts[1] == "x":
            try:
                fleet_indices.add(int(parts[2]))
            except:
                pass

    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
              "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"]

    # 1. Fleet Trajectories (Left col, top 2 rows)
    ax_traj = fig.add_subplot(gs[0:2, 0])
    ax_traj.set_xlabel("X [m]")
    ax_traj.set_ylabel("Y [m]")
    ax_traj.set_title(f"Fleet Trajectories ({len(fleet_indices)} vehicles)")
    ax_traj.grid(True, alpha=0.3)
    ax_traj.set_xlim(-5, 5)
    ax_traj.set_ylim(-5, 5)
    axes["trajectory"] = ax_traj

    for i in sorted(fleet_indices):
        c = colors[i % len(colors)]
        (l,) = ax_traj.plot([], [], color=c, lw=2, label=f"V{i}")
        lines[f"traj_{i}"] = l

        # Head marker
        (l_head,) = ax_traj.plot(
            [], [], marker="o", color=c, ms=8, linestyle="None", zorder=10
        )
        lines[f"traj_head_{i}"] = l_head

    ax_traj.legend(fontsize=8)

    # 2. Fleet Velocities (Top Right)
    ax_vel = fig.add_subplot(gs[0, 1])
    ax_vel.set_ylabel("Velocity [m/s]")
    ax_vel.set_title("Fleet Velocities")
    ax_vel.grid(True, alpha=0.3)
    axes["velocity"] = ax_vel

    for i in sorted(fleet_indices):
        c = colors[i % len(colors)]
        f = f"fleet_v_{i}"
        if f in field_names:
            (l,) = ax_vel.plot(
                [], [], color=c, linestyle="None", marker=".",
                markersize=2, label=f"V{i}",
            )
            lines[f] = l
    ax_vel.legend(fontsize=8)

    # 3. Fleet Headings (Mid Right)
    ax_theta = fig.add_subplot(gs[1, 1])
    ax_theta.set_ylabel("Heading [rad]")
    ax_theta.set_title("Fleet Headings")
    ax_theta.grid(True, alpha=0.3)
    axes["heading"] = ax_theta

    for i in sorted(fleet_indices):
        c = colors[i % len(colors)]
        f = f"fleet_theta_{i}"
        if f in field_names:
            (l,) = ax_theta.plot(
                [], [], color=c, linestyle="None", marker=".",
                markersize=2, label=f"V{i}",
            )
            lines[f] = l
    ax_theta.legend(fontsize=8)

    # 4. Fleet Size Info (Bottom Left)
    ax_size = fig.add_subplot(gs[2, 0])
    ax_size.set_ylabel("Fleet Size")
    ax_size.set_xlabel("Time [s]")
    ax_size.grid(True, alpha=0.3)
    axes["fleet_size"] = ax_size

    if "fleet_size" in field_names:
        (l,) = ax_size.plot([], [], "k.", markersize=3, label="Fleet Size")
        lines["fleet_size"] = l
    ax_size.legend(fontsize=8)

    # 5. Info (Bottom Right)
    ax_info = fig.add_subplot(gs[2, 1])
    ax_info.axis("off")
    axes["info"] = ax_info

    return fig, lines, axes


# ==============================================================================
# Test
# ==============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("Remote Scope Manager Test")
    print("=" * 60)

    # Create manager
    manager = RemoteScopeManager()

    # Start stream for car 0
    manager.start_stream(0, ["local_state", "local_control"])

    # Simulate receiving some data
    import struct

    def create_test_packet(vehicle_id, timestamp, values):
        """Create a test scope packet."""
        header = struct.pack("!BB", 0xAB, vehicle_id)
        ts = struct.pack("!d", timestamp)
        num_fields = struct.pack("!B", len(values))
        data = struct.pack(f"!{len(values)}f", *values)
        return header + ts + num_fields + data

    print("\nSimulating scope data reception...")
    for i in range(100):
        t = i * 0.02  # 50Hz
        values = [
            1.0 + t * 0.1,  # x
            2.0 + t * 0.05,  # y
            0.5 + t * 0.01,  # theta
            0.8,  # velocity
            1.0 + t * 0.1,  # x_gps
            2.0 + t * 0.05,  # y_gps
            0.5 + t * 0.01,  # theta_gps
            1.0,  # v_ref
        ]

        packet = create_test_packet(0, t, values)
        manager.receive_scope_data(0, packet.hex())

    # Print statistics
    stats = manager.get_statistics()
    print(f"\nStatistics:")
    print(f"  Total packets: {stats['total_packets']}")
    print(f"  Parse errors: {stats['parse_errors']}")
    print(f"  Streaming cars: {stats['streaming_cars']}")

    if "car_0" in stats:
        car_stats = stats["car_0"]
        print(f"  Car 0 samples: {car_stats['samples_received']}")
        print(f"  Car 0 buffer fill: {car_stats['fill_percent']:.1f}%")

    # Test buffer data retrieval
    buffer = manager.get_buffer(0)
    if buffer:
        t, x = buffer.get_data("x")
        print(f"\n  Buffer 'x' has {len(t)} samples")
        if len(t) > 0:
            print(f"  X range: {x.min():.2f} to {x.max():.2f}")

    print("\n✓ All tests passed!")
    print("\nTo test the viewer, run with --viewer flag")

    # Optionally show viewer
    import sys

    if "--viewer" in sys.argv:
        print("\nOpening viewer for Car 0...")
        manager.open_viewer(0, ["local_state"])

        # Keep adding data
        import time

        start = time.time()
        while time.time() - start < 30:
            t = time.time() - start
            values = [
                np.sin(t) * 2,  # x
                np.cos(t) * 2,  # y
                t * 0.1,  # theta
                0.8 + 0.2 * np.sin(t * 2),  # velocity
                np.sin(t) * 2,  # x_gps
                np.cos(t) * 2,  # y_gps
                t * 0.1,  # theta_gps
                1.0,  # v_ref
            ]

            packet = create_test_packet(0, t, values)
            manager.receive_scope_data(0, packet.hex())
            time.sleep(0.02)
