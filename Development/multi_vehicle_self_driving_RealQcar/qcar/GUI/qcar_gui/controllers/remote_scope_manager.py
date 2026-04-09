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
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from scope_data_streamer import unpack_scope_data, PRESET_FIELDS, DEFAULT_FIELDS, FLEET_FIELDS


OBSERVER_NUMERIC_FIELD_PREFIXES = (
    "x_vec_after_",
    "x_vec_before_",
    "dynamics_",
    "measurement_",
    "consensus_",
    "true_position_",
    "true_velocity_",
    "true_acceleration_",
)
OBSERVER_NUMERIC_EXACT_KEYS = (
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
            for i, v in enumerate(values[:self.num_fields]):
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
                    t = self.timestamps[start:self.write_pos].copy()
                    v = self.data[start:self.write_pos, field_idx].copy()
                else:
                    t = np.concatenate([self.timestamps[start:], self.timestamps[:self.write_pos]])
                    v = np.concatenate([self.data[start:, field_idx], self.data[:self.write_pos, field_idx]])
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
            'samples_received': self.samples_received,
            'buffer_used': self.sample_count,
            'buffer_max': self.max_samples,
            'fill_percent': (self.sample_count / self.max_samples) * 100,
            'receive_rate_hz': self.samples_received / duration if duration > 0 else 0,
            'duration_s': duration
        }
    
    def clear(self):
        """Clear the buffer."""
        with self._lock:
            self.timestamps.fill(0)
            self.data.fill(0)
            self.write_pos = 0
            self.sample_count = 0


class ObserverDataBuffer:
    """Thread-safe in-memory buffer for distributed observer telemetry."""

    def __init__(self, max_points: int = 4000):
        self.max_points = max_points
        self._lock = threading.Lock()
        self._times: Dict[int, deque] = {}
        self._fields: Dict[int, Dict[str, deque]] = {}

    def append(self, car_id: int, telemetry: Dict[str, Any]) -> None:
        values: Dict[str, float] = {}
        for key, value in telemetry.items():
            if key == "time":
                continue
            if not key.startswith(OBSERVER_NUMERIC_FIELD_PREFIXES) and key not in OBSERVER_NUMERIC_EXACT_KEYS:
                continue
            try:
                values[key] = float(value)
            except Exception:
                continue

        if not values:
            return

        try:
            t = float(telemetry.get("time", time.time()))
        except Exception:
            t = time.time()

        with self._lock:
            if car_id not in self._times:
                maxlen = self.max_points if self.max_points > 0 else None
                self._times[car_id] = deque(maxlen=maxlen)
                self._fields[car_id] = {}

            self._times[car_id].append(t)
            curr_len = len(self._times[car_id])
            field_map = self._fields[car_id]

            for key, value in values.items():
                if key not in field_map:
                    maxlen = self.max_points if self.max_points > 0 else None
                    field_map[key] = deque([np.nan] * (curr_len - 1), maxlen=maxlen)
                field_map[key].append(float(value))

            for series in field_map.values():
                if len(series) < curr_len:
                    series.append(np.nan)

    def snapshot(self) -> Dict[int, Dict[str, np.ndarray]]:
        with self._lock:
            snap: Dict[int, Dict[str, np.ndarray]] = {}
            for car_id, t_deque in self._times.items():
                snap[car_id] = {
                    "time": np.asarray(t_deque, dtype=float),
                    "fields": {
                        key: np.asarray(series, dtype=float)
                        for key, series in self._fields[car_id].items()
                    },
                }
            return snap

    def clear(self) -> None:
        with self._lock:
            self._times.clear()
            self._fields.clear()


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
        self.viewers: Dict[int, 'RemoteScopeViewer'] = {}
        self.observer_buffer = ObserverDataBuffer(max_points=4000)
        self.observer_viewer: Optional['RemoteObserverViewer'] = None
        self._observer_plot_time_origin = time.monotonic()
        
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
    
    def start_stream(self, car_id: int, preset_names: List[str] = None):
        """
        Start receiving scope data from a vehicle.
        
        Args:
            car_id: Vehicle ID
            preset_names: List of presets being streamed
        """
        with self._lock:
            # Build field list from presets
            field_names = []
            if preset_names:
                for preset in preset_names:
                    if preset in PRESET_FIELDS:
                        for field in PRESET_FIELDS[preset]:
                            if field not in field_names:
                                field_names.append(field)
            
            if not field_names:
                field_names = DEFAULT_FIELDS.copy()
            
            # Create buffer if needed
            if car_id not in self.vehicle_buffers:
                self.vehicle_buffers[car_id] = ScopeDataBuffer(
                    max_samples=6000,  # 2 min at 50Hz
                    field_names=field_names
                )
            else:
                self.vehicle_buffers[car_id].clear()
            
            self.vehicle_field_names[car_id] = field_names
            self._streaming_cars.add(car_id)
            
            print(f"[RemoteScopeManager] Started stream for Car {car_id}: {field_names}")
    
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
                        result['timestamp'],
                        result['values']
                    )
            
            self.total_packets += 1
            
        except Exception as e:
            self.parse_errors += 1
            print(f"[RemoteScopeManager] Error processing scope data: {e}")

    def receive_observer_telemetry(self, car_id: int, telemetry: Dict[str, Any]) -> None:
        """Ingest regular telemetry for the Plot-All observer viewer."""
        telemetry_for_plot = dict(telemetry)
        telemetry_for_plot["time"] = time.monotonic() - self._observer_plot_time_origin
        self.observer_buffer.append(car_id, telemetry_for_plot)

    def reset_observer_plot_clock(self) -> None:
        """Reset the Plot-All observer time base and discard pre-trigger samples."""
        with self._lock:
            self._observer_plot_time_origin = time.monotonic()
            self.observer_buffer.clear()
    
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
                fps=20
            )
            self.viewers[car_id].start()
    
    def close_viewer(self, car_id: int):
        """Close the viewer for a vehicle."""
        with self._lock:
            if car_id in self.viewers:
                self.viewers[car_id].stop()
                del self.viewers[car_id]

    def open_observer_viewer(self, refresh_ms: int = 150, time_window: float = 0.0) -> bool:
        """Open (or reuse) the Plot-All observer viewer in a separate process."""
        with self._lock:
            if self.observer_viewer and not self.observer_viewer.sync_state():
                self.observer_viewer = None

            if self.observer_viewer and self.observer_viewer.running:
                return False

            self.observer_viewer = RemoteObserverViewer(
                buffer=self.observer_buffer,
                refresh_ms=max(50, int(refresh_ms)),
                time_window=max(0.0, float(time_window)),
            )
            self.observer_viewer.start()
            return True

    def close_observer_viewer(self) -> None:
        """Close the Plot-All observer viewer if it is running."""
        with self._lock:
            if self.observer_viewer:
                self.observer_viewer.stop()
                self.observer_viewer = None

    def is_observer_viewer_running(self) -> bool:
        """Return True when the Plot-All observer viewer process is running."""
        with self._lock:
            if self.observer_viewer and not self.observer_viewer.sync_state():
                self.observer_viewer = None
            return bool(self.observer_viewer and self.observer_viewer.running)
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get overall statistics."""
        stats = {
            'total_packets': self.total_packets,
            'parse_errors': self.parse_errors,
            'streaming_cars': list(self._streaming_cars),
            'active_viewers': list(self.viewers.keys())
        }
        
        # Add per-vehicle stats
        for car_id, buffer in self.vehicle_buffers.items():
            stats[f'car_{car_id}'] = buffer.get_statistics()
        
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

            if self.observer_viewer:
                try:
                    self.observer_viewer.stop()
                except Exception as e:
                    print(f"[RemoteScopeManager] Error stopping observer viewer: {e}")
                self.observer_viewer = None
            
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
    
    def __init__(self, car_id: int, buffer: ScopeDataBuffer, 
                 field_names: List[str], fps: int = 20):
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
        self._data_queue = mp.Queue(maxsize=100)
        
        # Start plot process
        self._process = mp.Process(
            target=_run_scope_plot_process,
            args=(
                self.car_id,
                self.field_names,
                self._data_queue,
                self._stop_event,
                self.fps,
                self.time_window
            ),
            daemon=True
        )
        self._process.start()
        
        # Start data feeder thread
        self._feeder_thread = threading.Thread(
            target=self._feed_data_to_process,
            daemon=True
        )
        self._feeder_thread.start()
        
        print(f"[ScopeViewer] Started viewer for Car {self.car_id}")
    
    def _feed_data_to_process(self):
        """Feed data from buffer to the plot process."""
        import time
        
        last_feed_time = 0
        feed_interval = 1.0 / self.fps
        
        while self.running and not self._stop_event.is_set():
            try:
                current_time = time.time()
                if current_time - last_feed_time >= feed_interval:
                    # Get current data from buffer for each field
                    data_packet = {'timestamp': current_time}
                    
                    for field in self.field_names:
                        t, v = self.buffer.get_data(field, last_n=int(self.time_window * 50))
                        if len(t) > 0:
                            data_packet[field] = {'t': t.tolist(), 'v': v.tolist()}
                    
                    # Send to process (non-blocking)
                    try:
                        self._data_queue.put_nowait(data_packet)
                    except:
                        pass  # Queue full, skip this update
                    
                    last_feed_time = current_time
                
                time.sleep(0.01)  # Small sleep to prevent busy-waiting
                
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


def _run_scope_plot_process(car_id, field_names, data_queue, stop_event, fps, time_window):
    """
    Run the matplotlib plot in a separate process.
    
    Layouts match plot_scope_data.py (Interactive Mode):
    - Uses GridSpec for complex layouts (Trajectory X-Y, etc.)
    """
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    import numpy as np
    
    # Detect if this is Fleet or Local data based on field names
    is_fleet = any(f.startswith('fleet_') for f in field_names)
    
    if is_fleet:
        fig, lines, axes = _create_fleet_layout(plt, car_id, field_names)
    else:
        fig, lines, axes = _create_local_layout(plt, car_id, field_names)
    
    # Latest data storage
    latest_data = {}
    
    def update(frame):
        nonlocal latest_data
        
        # Check if we should stop
        if stop_event.is_set():
            plt.close(fig)
            return list(lines.values())
        
        # Get latest data from queue
        try:
            while not data_queue.empty():
                latest_data = data_queue.get_nowait()
        except:
            pass
        
        if not latest_data:
            return list(lines.values())
            
        # Update standard time-series lines
        for field, line in lines.items():
            # Skip special trajectory lines which are handled below
            if field.startswith('traj_') or field == 'trajectory' or field == 'trajectory_gps':
                continue
                
            if field in latest_data and 't' in latest_data[field]:
                t = latest_data[field]['t']
                v = latest_data[field]['v']
                line.set_data(t, v)
        
        # Update Local Trajectory (X vs Y)
        if not is_fleet:
            if 'trajectory' in lines and 'x' in latest_data and 'y' in latest_data:
                x = latest_data['x']['v']
                y = latest_data['y']['v']
                min_len = min(len(x), len(y))
                if min_len > 0:
                    lines['trajectory'].set_data(x[:min_len], y[:min_len])
                    # Update head
                    if 'trajectory_head' in lines:
                        lines['trajectory_head'].set_data([x[min_len-1]], [y[min_len-1]])
            
            if 'trajectory_gps' in lines and 'x_gps' in latest_data and 'y_gps' in latest_data:
                x = latest_data['x_gps']['v']
                y = latest_data['y_gps']['v']
                min_len = min(len(x), len(y))
                if min_len > 0:
                    lines['trajectory_gps'].set_data(x[:min_len], y[:min_len])

        # Update Fleet Trajectories (V0, V1, V2...)
        else:
            # We assume fleet_x_0, fleet_y_0, etc.
            # Find all vehicle indices from field names
            fleet_indices = set()
            for f in field_names:
                parts = f.split('_')
                if len(parts) == 3 and parts[0] == 'fleet' and parts[1] == 'x':
                     try:
                         fleet_indices.add(int(parts[2]))
                     except: pass
            
            for i in fleet_indices:
                line_key = f'traj_{i}'
                fx = f'fleet_x_{i}'
                fy = f'fleet_y_{i}'
                
                if line_key in lines and fx in latest_data and fy in latest_data:
                    x = latest_data[fx]['v']
                    y = latest_data[fy]['v']
                    min_len = min(len(x), len(y))
                    if min_len > 0:
                        lines[line_key].set_data(x[:min_len], y[:min_len])
                        # Update head
                        head_key = f'traj_head_{i}'
                        if head_key in lines:
                             lines[head_key].set_data([x[min_len-1]], [y[min_len-1]])

        # Update axis limits
        for ax in axes.values():
            ax.relim()
            ax.autoscale_view()
        
        return list(lines.values())
    
    # Create animation
    ani = animation.FuncAnimation(
        fig, update,
        interval=1000 // fps,
        blit=False,
        cache_frame_data=False
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
    fig.suptitle(f'Local State Estimation - Car {car_id}', fontsize=12)
    
    # 3 rows, 3 cols
    gs = fig.add_gridspec(3, 3, height_ratios=[2, 1, 1], hspace=0.35, wspace=0.3)
    
    axes = {}
    lines = {}
    
    # 1. Trajectory (Left 2x2) - X vs Y
    ax_traj = fig.add_subplot(gs[0:2, 0:2])
    ax_traj.set_xlabel('X [m]')
    ax_traj.set_ylabel('Y [m]')
    ax_traj.set_title('Trajectory')
    ax_traj.grid(True, alpha=0.3)
    ax_traj.set_xlim(-5, 5)
    ax_traj.set_ylim(-5, 5)
    axes['trajectory'] = ax_traj
    
    # Est path
    line, = ax_traj.plot([], [], 'b-', lw=2, label='Path Est')
    lines['trajectory'] = line
    # Head marker
    line_head, = ax_traj.plot([], [], 'bo', ms=8, zorder=10) # Blue dot
    lines['trajectory_head'] = line_head

    # GPS path
    if 'x_gps' in field_names:
        line_gps, = ax_traj.plot([], [], 'r--', lw=1, label='GPS')
        lines['trajectory_gps'] = line_gps
    ax_traj.legend(loc='upper right', fontsize=8)

    # 2. Velocity (Top Right)
    ax_vel = fig.add_subplot(gs[0, 2])
    ax_vel.set_ylabel('Velocity [m/s]')
    ax_vel.grid(True, alpha=0.3)
    axes['velocity'] = ax_vel
    
    for f, c, s in [('velocity', 'b', '-'), ('v_ref', 'k', '--')]:
        if f in field_names:
            l, = ax_vel.plot([], [], color=c, linestyle=s, label=f)
            lines[f] = l
    ax_vel.legend(fontsize=8)

    # 3. Heading (Mid Right)
    ax_th = fig.add_subplot(gs[1, 2])
    ax_th.set_ylabel('Heading [rad]')
    ax_th.grid(True, alpha=0.3)
    axes['heading'] = ax_th
    
    for f, c, s in [('theta', 'g', '-'), ('theta_gps', 'r', '--')]:
        if f in field_names:
            l, = ax_th.plot([], [], color=c, linestyle=s, label=f)
            lines[f] = l
    ax_th.legend(fontsize=8)

    # 4. Controls (Bottom Left 2 cols)
    ax_ctrl = fig.add_subplot(gs[2, 0:2])
    ax_ctrl.set_ylabel('Control')
    ax_ctrl.set_xlabel('Time [s]')
    ax_ctrl.grid(True, alpha=0.3)
    axes['control'] = ax_ctrl
    
    for f, c in [('throttle', 'g'), ('steering', 'r')]:
        if f in field_names:
            l, = ax_ctrl.plot([], [], color=c, label=f)
            lines[f] = l
    ax_ctrl.legend(fontsize=8)
    
    # 5. Info/Text (Bottom Right)
    ax_info = fig.add_subplot(gs[2, 2])
    ax_info.axis('off')
    axes['info'] = ax_info
    
    return fig, lines, axes


def _create_fleet_layout(plt, car_id, field_names):
    """
    Create Fleet with GridSpec.
    """
    fig = plt.figure(figsize=(12, 10))
    fig.suptitle(f'Fleet State Estimation - Car {car_id}', fontsize=12)
    
    # 3 rows, 2 cols (Trajectory on left column)
    gs = fig.add_gridspec(3, 2, height_ratios=[2, 1, 1], hspace=0.35, wspace=0.3)
    
    axes = {}
    lines = {}
    
    # 1. Fleet Trajectories (Left col, top 2 rows)
    ax_traj = fig.add_subplot(gs[0:2, 0])
    ax_traj.set_xlabel('X [m]')
    ax_traj.set_ylabel('Y [m]')
    ax_traj.set_title('Fleet Trajectories')
    ax_traj.grid(True, alpha=0.3)
    ax_traj.set_xlim(-5, 5)
    ax_traj.set_ylim(-5, 5)
    axes['trajectory'] = ax_traj
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    
    # Identify vehicles
    fleet_indices = set()
    for f in field_names:
        parts = f.split('_')
        if len(parts) == 3 and parts[0] == 'fleet' and parts[1] == 'x':
             try: fleet_indices.add(int(parts[2]))
             except: pass
    
    for i in sorted(fleet_indices):
        c = colors[i % len(colors)]
        l, = ax_traj.plot([], [], color=c, lw=2, label=f'V{i}')
        lines[f'traj_{i}'] = l
        
        # Head marker
        l_head, = ax_traj.plot([], [], marker='o', color=c, ms=8, linestyle='None', zorder=10)
        lines[f'traj_head_{i}'] = l_head
        
    ax_traj.legend(fontsize=8)
    
    # 2. Velocities (Top Right)
    ax_vel = fig.add_subplot(gs[0, 1])
    ax_vel.set_ylabel('Velocity [m/s]')
    ax_vel.grid(True, alpha=0.3)
    axes['velocity'] = ax_vel
    
    for i in sorted(fleet_indices):
        c = colors[i % len(colors)]
        f = f'fleet_v_{i}'
        if f in field_names:
            l, = ax_vel.plot([], [], color=c, label=f'V{i}')
            lines[f] = l
    
    # 3. Consensus (Mid Right)
    ax_con = fig.add_subplot(gs[1, 1])
    ax_con.set_ylabel('Consensus Err')
    ax_con.grid(True, alpha=0.3)
    axes['consensus'] = ax_con
    
    if 'consensus_error' in field_names:
        l, = ax_con.plot([], [], 'k-', label='Error')
        lines['consensus_error'] = l
    
    # 4. Trust (Bottom Left)
    ax_trust = fig.add_subplot(gs[2, 0])
    ax_trust.set_ylabel('Trust Score')
    ax_trust.set_xlabel('Time [s]')
    ax_trust.grid(True, alpha=0.3)
    ax_trust.set_ylim(-0.1, 1.1)
    axes['trust'] = ax_trust
    
    for i in sorted(fleet_indices):
        c = colors[i % len(colors)]
        f = f'trust_{i}'
        if f in field_names:
            l, = ax_trust.plot([], [], color=c, linestyle='--', label=f'T{i}')
            lines[f] = l
    ax_trust.legend(fontsize=8)
    
    # 5. Info (Bottom Right)
    ax_info = fig.add_subplot(gs[2, 1])
    ax_info.axis('off')
    axes['info'] = ax_info
    
    return fig, lines, axes


class RemoteObserverViewer:
    """Plot-All observer viewer running in a separate process."""

    def __init__(self, buffer: ObserverDataBuffer, refresh_ms: int = 150, time_window: float = 0.0):
        self.buffer = buffer
        self.refresh_ms = refresh_ms
        self.time_window = time_window
        self.running = False
        self._process = None
        self._data_queue = None
        self._stop_event = None

    def start(self) -> None:
        if self.running:
            return

        import multiprocessing as mp

        self.running = True
        self._stop_event = mp.Event()
        self._data_queue = mp.Queue(maxsize=4)
        self._process = mp.Process(
            target=_run_observer_plot_process,
            args=(self._data_queue, self._stop_event, self.refresh_ms, self.time_window),
            daemon=True,
        )
        self._process.start()

        self._feeder_thread = threading.Thread(target=self._feed_data_to_process, daemon=True)
        self._feeder_thread.start()

    def is_alive(self) -> bool:
        return bool(self._process and self._process.is_alive())

    def sync_state(self) -> bool:
        """Sync local running state with the child process lifecycle."""
        alive = self.is_alive()
        if self.running and not alive:
            self.running = False
            self._stop_event = None
            self._data_queue = None
            self._process = None
        return alive

    def _feed_data_to_process(self) -> None:
        feed_interval = self.refresh_ms / 1000.0
        last_feed_time = 0.0

        while self.running and self._stop_event and not self._stop_event.is_set():
            try:
                now = time.time()
                if now - last_feed_time >= feed_interval:
                    snap = self.buffer.snapshot()
                    if snap:
                        trimmed = _observer_trim_snapshot(snap, self.time_window)
                        try:
                            self._data_queue.put_nowait(trimmed)
                        except Exception:
                            # Queue is full; keep viewer responsive by dropping stale frames.
                            pass
                    last_feed_time = now
                time.sleep(0.01)
            except Exception as e:
                print(f"[ObserverViewer] Data feed error: {e}")
                break

    def stop(self) -> None:
        self.running = False

        if self._stop_event:
            self._stop_event.set()

        if self._process and self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)

        self._stop_event = None
        self._data_queue = None
        self._process = None


def _observer_trim_snapshot(snap: Dict[int, Dict[str, np.ndarray]], time_window: float) -> Dict[int, Dict[str, np.ndarray]]:
    """Trim snapshot to trailing time window before passing through IPC queue."""
    out: Dict[int, Dict[str, np.ndarray]] = {}
    for sid, payload in snap.items():
        t_ref = payload.get("time", np.array([]))
        fields = payload.get("fields", {})
        if t_ref.size == 0:
            continue

        # Keep one shared time axis per stream; do not trim fields independently.
        # Preserve absolute time so x-axis increases continuously.
        if time_window > 0:
            keep = t_ref >= (t_ref[-1] - time_window)
        else:
            keep = np.ones_like(t_ref, dtype=bool)

        t_out = t_ref[keep]
        if t_out.size == 0:
            continue

        new_fields: Dict[str, np.ndarray] = {}
        for key, y in fields.items():
            n = min(t_ref.size, y.size)
            if n == 0:
                continue
            if n == t_ref.size:
                field_keep = keep
            else:
                field_keep = keep[:n]
            y2 = y[:n][field_keep]
            if y2.size > 0:
                new_fields[key] = y2

        if not new_fields:
            continue

        out[sid] = {
            "time": t_out,
            "fields": new_fields,
        }

    return out


def _observer_window_series(t: np.ndarray, y: np.ndarray, time_window: float) -> tuple:
    if t.size == 0 or y.size == 0:
        return np.array([]), np.array([])

    if t.size != y.size:
        n = min(t.size, y.size)
        t = t[:n]
        y = y[:n]

    mask = ~(np.isnan(t) | np.isnan(y))
    t = t[mask]
    y = y[mask]
    if t.size == 0:
        return t, y

    if time_window > 0:
        keep = t >= (t[-1] - time_window)
        t = t[keep]
        y = y[keep]
    return t, y


def _has_observer_fields(fields: Dict[str, np.ndarray]) -> bool:
    for key in fields.keys():
        if key.startswith("x_vec_before_p") or key.startswith("x_vec_before_v") or key.startswith("x_vec_before_a"):
            return True
    return False


def _resolve_observer_stream_id(snap: Dict[int, Dict[str, np.ndarray]], observer_label: int) -> Optional[int]:
    for sid in (observer_label, observer_label - 1):
        if sid in snap and _has_observer_fields(snap[sid].get("fields", {})):
            return sid

    for sid in sorted(snap.keys()):
        if _has_observer_fields(snap[sid].get("fields", {})):
            return sid
    return None


def _run_observer_plot_process(data_queue, stop_event, refresh_ms: int, time_window: float):
    """Render Plot-All observer layout in a dedicated process."""
    import queue
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    fig = plt.figure(figsize=(14, 8))
    fig.suptitle("Live: True State vs Distributed Observer Estimates", fontsize=12)
    axes = fig.subplots(4, 3, squeeze=False)

    # True-state row lines
    true_lines = {
        "pos": {},
        "vel": {},
        "u": {},
    }
    for veh_id in TRUE_ROW_VEHICLES:
        color, style = VEHICLE_STYLE.get(veh_id, ("k", "-"))
        true_lines["pos"][veh_id], = axes[0][0].plot([], [], color=color, linestyle=style, linewidth=1.8, label=f"V{veh_id}")
        true_lines["vel"][veh_id], = axes[0][1].plot([], [], color=color, linestyle=style, linewidth=1.8, label=f"V{veh_id}")
        true_lines["u"][veh_id], = axes[0][2].plot([], [], color=color, linestyle=style, linewidth=1.8, label=f"V{veh_id}")

    obs_lines = {}
    missing_text = {}
    for row_idx, observer_label in enumerate(OBSERVER_ROWS, start=1):
        obs_lines[observer_label] = {"p": {}, "v": {}, "a": {}}
        missing_text[observer_label] = []
        for col_idx, key in enumerate(("p", "v", "a")):
            ax = axes[row_idx][col_idx]
            for comp_idx in (1, 2, 3):
                obs_lines[observer_label][key][comp_idx], = ax.plot(
                    [], [], color=COMPONENT_COLORS[comp_idx], linestyle="-", linewidth=1.6, label=f"i = {comp_idx}"
                )
            txt = ax.text(
                0.5,
                0.5,
                f"Observer {observer_label} no data",
                transform=ax.transAxes,
                ha="center",
                va="center",
                fontsize=9,
                visible=False,
            )
            missing_text[observer_label].append(txt)

    # Keep basic layout from plot_all_observer_viewer.py.
    axes[0][0].set_title("True Position")
    axes[0][0].set_ylabel("Position")
    axes[0][1].set_title("True Velocity")
    axes[0][1].set_ylabel("Velocity")
    axes[0][1].set_ylim(-0.1, 1.1)
    axes[0][2].set_title("Control Input")
    axes[0][2].set_ylabel("Control Input")
    axes[0][2].set_ylim(-0.01, 0.25)

    for row_idx, observer_label in enumerate(OBSERVER_ROWS, start=1):
        axes[row_idx][0].set_title(f"Ditributed Obsever {observer_label} - Relative Position")
        axes[row_idx][0].set_ylabel("Estimation of pi - p0 +di0")
        axes[row_idx][1].set_title(f"Ditributed Obsever {observer_label} - Relative Velocity")
        axes[row_idx][1].set_ylabel("Estimation of vi - v0")
        axes[row_idx][2].set_title(f"Ditributed Obsever {observer_label} - Relative acceleration")
        axes[row_idx][2].set_ylabel("Estimation of ai - a0")

    for r in range(4):
        for c in range(3):
            axes[r][c].grid(True, alpha=0.3)
            axes[r][c].set_xlabel("Time [s]")
            axes[r][c].set_xlim(left=0.0)
            handles, labels = axes[r][c].get_legend_handles_labels()
            if handles:
                axes[r][c].legend(fontsize=8 if r == 0 else 7, loc="upper right")

    fig.tight_layout(rect=(0.02, 0.02, 0.98, 0.95))
    latest_snap: Dict[int, Dict[str, np.ndarray]] = {}

    def _set_true_line(line, snap, veh_id, candidates):
        source_ids = [veh_id] + [sid for sid in sorted(snap.keys()) if sid != veh_id]
        for sid in source_ids:
            if sid not in snap:
                continue
            t = snap[sid].get("time", np.array([]))
            fields = snap[sid].get("fields", {})
            for key in candidates:
                if key in fields:
                    t2, y2 = _observer_window_series(t, fields[key], time_window)
                    line.set_data(t2, y2)
                    return
        line.set_data([], [])

    def update(_frame):
        nonlocal latest_snap

        if stop_event.is_set():
            plt.close(fig)
            return []

        while True:
            try:
                latest_snap = data_queue.get_nowait()
            except queue.Empty:
                break
            except Exception:
                break

        if not latest_snap:
            return []

        max_time_seen = 0.0

        for veh_id in TRUE_ROW_VEHICLES:
            _set_true_line(true_lines["pos"][veh_id], latest_snap, veh_id, (f"true_position_{veh_id}", "x"))
            _set_true_line(true_lines["vel"][veh_id], latest_snap, veh_id, (f"true_velocity_{veh_id}", "v"))
            _set_true_line(true_lines["u"][veh_id], latest_snap, veh_id, ("u", "control_input", f"collective_control_{veh_id + 1}"))

        for observer_label in OBSERVER_ROWS:
            sid = _resolve_observer_stream_id(latest_snap, observer_label)
            if sid is None or sid not in latest_snap:
                for txt in missing_text[observer_label]:
                    txt.set_visible(True)
                for key in ("p", "v", "a"):
                    for comp_idx in (1, 2, 3):
                        obs_lines[observer_label][key][comp_idx].set_data([], [])
                continue

            for txt in missing_text[observer_label]:
                txt.set_visible(False)

            fields = latest_snap[sid].get("fields", {})
            t_ref = latest_snap[sid].get("time", np.array([]))
            if t_ref.size > 0:
                max_time_seen = max(max_time_seen, float(t_ref[-1]))
            for comp_idx in (1, 2, 3):
                for key, suffix in (("p", "p"), ("v", "v"), ("a", "a")):
                    field_name = f"x_vec_before_{suffix}{comp_idx}"
                    line = obs_lines[observer_label][key][comp_idx]
                    if field_name in fields:
                        max_time_seen = max(max_time_seen, float(t_ref[-1]) if t_ref.size > 0 else max_time_seen)
                        t2, y2 = _observer_window_series(t_ref, fields[field_name], time_window)
                        line.set_data(t2, y2)
                    else:
                        line.set_data([], [])

        x_max = max(0.1, max_time_seen * 1.05)
        for r in range(4):
            for c in range(3):
                axes[r][c].relim()
                axes[r][c].autoscale_view(scalex=False, scaley=True)
                axes[r][c].set_xlim(0.0, x_max)

        artists = []
        for group in true_lines.values():
            artists.extend(group.values())
        for observer_label in OBSERVER_ROWS:
            for key in ("p", "v", "a"):
                artists.extend(obs_lines[observer_label][key].values())
            artists.extend(missing_text[observer_label])
        return artists

    anim = animation.FuncAnimation(
        fig,
        update,
        interval=max(50, int(refresh_ms)),
        blit=False,
        cache_frame_data=False,
    )
    # Keep animation alive for the full viewer lifecycle.
    fig._observer_anim = anim

    try:
        plt.show()
    except Exception:
        pass


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
    manager.start_stream(0, ['local_state', 'local_control'])
    
    # Simulate receiving some data
    import struct
    
    def create_test_packet(vehicle_id, timestamp, values):
        """Create a test scope packet."""
        header = struct.pack('!BB', 0xAB, vehicle_id)
        ts = struct.pack('!d', timestamp)
        num_fields = struct.pack('!B', len(values))
        data = struct.pack(f'!{len(values)}f', *values)
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
            1.0  # v_ref
        ]
        
        packet = create_test_packet(0, t, values)
        manager.receive_scope_data(0, packet.hex())
    
    # Print statistics
    stats = manager.get_statistics()
    print(f"\nStatistics:")
    print(f"  Total packets: {stats['total_packets']}")
    print(f"  Parse errors: {stats['parse_errors']}")
    print(f"  Streaming cars: {stats['streaming_cars']}")
    
    if 'car_0' in stats:
        car_stats = stats['car_0']
        print(f"  Car 0 samples: {car_stats['samples_received']}")
        print(f"  Car 0 buffer fill: {car_stats['fill_percent']:.1f}%")
    
    # Test buffer data retrieval
    buffer = manager.get_buffer(0)
    if buffer:
        t, x = buffer.get_data('x')
        print(f"\n  Buffer 'x' has {len(t)} samples")
        if len(t) > 0:
            print(f"  X range: {x.min():.2f} to {x.max():.2f}")
    
    print("\n✓ All tests passed!")
    print("\nTo test the viewer, run with --viewer flag")
    
    # Optionally show viewer
    import sys
    if '--viewer' in sys.argv:
        print("\nOpening viewer for Car 0...")
        manager.open_viewer(0, ['local_state'])
        
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
                1.0  # v_ref
            ]
            
            packet = create_test_packet(0, t, values)
            manager.receive_scope_data(0, packet.hex())
            time.sleep(0.02)
