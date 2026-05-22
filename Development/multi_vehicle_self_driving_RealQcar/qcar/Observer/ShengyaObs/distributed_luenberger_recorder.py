"""
Distributed Luenberger Estimator Data Recorder

Records internal state and debug data from DistributedLuenbergerEstimator
for offline analysis and debugging.

Features:
- Non-blocking recording using background thread
- Queue-based write buffering to avoid blocking main update loop
- Automatic flush on stop or timeout
"""
import os
import csv
import time
import threading
import queue
from datetime import datetime
from typing import Dict, List, Optional
import numpy as np


class DistributedLuenbergerRecorder:
    """
    Non-blocking recorder for DistributedLuenbergerEstimator debug data.
    
    Uses a background thread with a queue to write data without blocking
    the main estimator update loop.
    
    Performance features:
    - Queue-based buffering (main thread just puts data in queue)
    - Background thread handles all file I/O
    - Configurable queue size and flush interval
    - Graceful shutdown with data flush
    
    Usage:
        recorder = DistributedLuenbergerRecorder(
            output_dir="observer_recordings",
            vehicle_id=1,
            observer_size=3
        )
        recorder.start()
        
        # In update loop (non-blocking):
        recorder.record(t, estimator.get_debug_data())
        
        # When done:
        recorder.stop()
    """
    
    # Queue sentinel to signal shutdown
    _STOP_SENTINEL = object()
    
    def __init__(self, output_dir: str = "observer_recordings", 
                 vehicle_id: int = 0, 
                 observer_size: int = 3,
                 fleet_size: int = 4,
                 queue_size: int = 1000,
                 flush_interval: float = 5.0):
        """
        Initialize the recorder.
        
        Args:
            output_dir: Directory to save CSV files
            vehicle_id: ID of the vehicle running this estimator
            observer_size: Number of follower vehicles (fleet_size - 1)
            fleet_size: Total fleet size including leader
            queue_size: Max items in write queue before blocking (default 1000)
            flush_interval: Seconds between automatic file flushes (default 5.0)
        """
        self.output_dir = output_dir
        self.vehicle_id = vehicle_id
        self.observer_size = observer_size
        self.fleet_size = fleet_size
        self.queue_size = queue_size
        self.flush_interval = flush_interval
        
        self.file = None
        self.writer = None
        self.recording = False
        self.filepath = None
        self.columns = []
        
        # Threading components
        self._write_queue: queue.Queue = None
        self._writer_thread: threading.Thread = None
        self._last_flush_time: float = 0.0
        self._record_count = 0
        self._dropped_count = 0
        
    def start(self) -> str:
        """
        Start recording. Creates output directory, CSV file, and background thread.
        
        Returns:
            filepath: Path to the created CSV file
        """
        os.makedirs(self.output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"dist_luenberger_v{self.vehicle_id}_{timestamp}.csv"
        self.filepath = os.path.join(self.output_dir, filename)
        
        # Build column list
        self.columns = self._build_columns()
        
        # Open file with buffering
        self.file = open(self.filepath, 'w', newline='', buffering=8192)
        self.writer = csv.DictWriter(self.file, fieldnames=self.columns)
        self.writer.writeheader()
        self.file.flush()
        
        # Initialize queue and start background thread
        self._write_queue = queue.Queue(maxsize=self.queue_size)
        self._last_flush_time = time.time()
        self._record_count = 0
        self._dropped_count = 0
        
        self.recording = True
        
        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            name=f"LuenbergerRecorder_V{self.vehicle_id}",
            daemon=True
        )
        self._writer_thread.start()
        
        return self.filepath
    
    def _build_columns(self) -> List[str]:
        """Build column names based on observer_size and fleet_size."""
        columns = ['time']
        
        # Observer state vector (3 per vehicle: position, velocity, acceleration)
        for prefix in ['x_vec']:
            for i in range(self.observer_size):
                vid = i + 1
                columns.extend([
                    f'{prefix}_p{vid}',
                    f'{prefix}_v{vid}',
                    f'{prefix}_a{vid}'
                ])
        
        # Observer terms
        for term in ['dynamics', 'measurement', 'consensus']:
            for i in range(self.observer_size):
                vid = i + 1
                columns.extend([
                    f'{term}_p{vid}',
                    f'{term}_v{vid}',
                    f'{term}_a{vid}'
                ])
        
        # Measurement data
        columns.extend([
            'local_meas_rel_pos', 'local_meas_vel',
            'est_meas_rel_pos', 'est_meas_vel',
            'meas_err_rel_pos', 'meas_err_vel'
        ])
        
        # Consensus info
        columns.extend(['neighbor_count', 'consensus_norm'])
        
        # Collective control input for each follower vehicle
        for i in range(self.observer_size):
            vid = i + 1
            columns.append(f'collective_control_{vid}')
        
        # Fleet states
        for vid in range(self.fleet_size):
            columns.extend([f'fleet_x_{vid}', f'fleet_v_{vid}'])

        # Time step
        columns.append('dt')

        # Local vehicle states and local measurements (single vehicle running this estimator)
        columns.extend([
            'position',
            'velocity',
            'acceleration',
            'control_input',
            'local_measurement_p',
            'local_measurement_v',
        ])

        # True states of all vehicles (via V2V)
        # Synchronized to current vehicle's timestamp
        # Includes leader (vehicle 0) and all followers (1 to fleet_size-1)
        for vid in range(0, self.fleet_size):
            columns.extend([
                f'true_position_{vid}',
                f'true_velocity_{vid}',
                f'true_acceleration_{vid}',
                f'true_throttle_{vid}',
            ])

        return columns
    
    def record(self, t: float, debug_data: Dict) -> bool:
        """
        Queue a data sample for recording (non-blocking).
        
        Args:
            t: Current time (seconds)
            debug_data: Dictionary from estimator.get_debug_data()
            
        Returns:
            True if queued successfully, False if queue full (data dropped)
        """
        if not self.recording or self._write_queue is None or not debug_data:
            return False
        
        try:
            # Build row dict (fast, just dictionary operations)
            row = self._build_row(t, debug_data)
            
            # Non-blocking put - if queue full, drop the sample
            try:
                self._write_queue.put_nowait(row)
                return True
            except queue.Full:
                self._dropped_count += 1
                return False
                
        except Exception:
            return False
    
    def _build_row(self, t: float, debug_data: Dict) -> Dict:
        """Build CSV row dict from debug data (fast path)."""
        row = {'time': t}
        
        # x_vec states
        for prefix in ['x_vec']:
            data = debug_data.get(prefix, None)
            if data is not None:
                for i in range(self.observer_size):
                    vid = i + 1
                    base = i * 3
                    row[f'{prefix}_p{vid}'] = data[base] if base < len(data) else 0.0
                    row[f'{prefix}_v{vid}'] = data[base+1] if base+1 < len(data) else 0.0
                    row[f'{prefix}_a{vid}'] = data[base+2] if base+2 < len(data) else 0.0
        
        # Observer terms
        for term_key, term_name in [('dynamics_term', 'dynamics'), 
                                     ('measurement_term', 'measurement'),
                                     ('consensus_term', 'consensus')]:
            data = debug_data.get(term_key, None)
            if data is not None:
                for i in range(self.observer_size):
                    vid = i + 1
                    base = i * 3
                    row[f'{term_name}_p{vid}'] = data[base] if base < len(data) else 0.0
                    row[f'{term_name}_v{vid}'] = data[base+1] if base+1 < len(data) else 0.0
                    row[f'{term_name}_a{vid}'] = data[base+2] if base+2 < len(data) else 0.0
        
        # Measurement data
        for key, cols in [('local_measurement', ('local_meas_rel_pos', 'local_meas_vel')),
                          ('estimated_measurement', ('est_meas_rel_pos', 'est_meas_vel')),
                          ('measurement_error', ('meas_err_rel_pos', 'meas_err_vel'))]:
            data = debug_data.get(key, None)
            if data is not None:
                row[cols[0]] = data[0] if len(data) > 0 else 0.0
                row[cols[1]] = data[1] if len(data) > 1 else 0.0
        
        # Consensus info
        row['neighbor_count'] = debug_data.get('neighbor_count', 0)
        row['consensus_norm'] = debug_data.get('consensus_norm', 0.0)
        
        # Collective control inputs
        cc_data = debug_data.get('collective_control', None)
        if cc_data is not None:
            for i in range(self.observer_size):
                vid = i + 1
                row[f'collective_control_{vid}'] = cc_data[i] if i < len(cc_data) else 0.0
        
        # Fleet states
        fleet = debug_data.get('fleet_states', None)
        if fleet is not None:
            for vid in range(self.fleet_size):
                if vid < fleet.shape[1]:
                    row[f'fleet_x_{vid}'] = fleet[0, vid]
                    row[f'fleet_v_{vid}'] = fleet[3, vid]

        # Time step
        row['dt'] = debug_data.get('dt', 0.0)

        # Local vehicle states and local measurements
        row['position'] = debug_data.get('position', 0.0)
        row['velocity'] = debug_data.get('velocity', 0.0)
        row['acceleration'] = debug_data.get('acceleration', 0.0)
        row['control_input'] = debug_data.get('control_input', 0.0)
        row['local_measurement_p'] = debug_data.get('local_measurement_p', 0.0)
        row['local_measurement_v'] = debug_data.get('local_measurement_v', 0.0)

        # True states of all vehicles (via V2V)
        # Includes leader (vehicle 0) and all followers (1 to fleet_size-1)
        for vid in range(0, self.fleet_size):
            row[f'true_position_{vid}'] = debug_data.get(f'true_position_{vid}', 0.0)
            row[f'true_velocity_{vid}'] = debug_data.get(f'true_velocity_{vid}', 0.0)
            row[f'true_acceleration_{vid}'] = debug_data.get(f'true_acceleration_{vid}', 0.0)
            row[f'true_throttle_{vid}'] = debug_data.get(f'true_throttle_{vid}', 0.0)

        return row
    
    def _writer_loop(self):
        """Background thread loop that writes queued data to file."""
        while True:
            try:
                # Wait for data with timeout for periodic flush
                try:
                    item = self._write_queue.get(timeout=0.5)
                except queue.Empty:
                    # Check for periodic flush
                    if time.time() - self._last_flush_time > self.flush_interval:
                        self._flush()
                    continue
                
                # Check for stop signal
                if item is self._STOP_SENTINEL:
                    break
                
                # Write the row
                if self.writer is not None:
                    self.writer.writerow(item)
                    self._record_count += 1
                
                # Periodic flush
                if time.time() - self._last_flush_time > self.flush_interval:
                    self._flush()
                    
            except Exception:
                pass  # Don't crash the thread
        
        # Final flush before exit
        self._flush()
    
    def _flush(self):
        """Flush file buffer to disk."""
        if self.file is not None:
            try:
                self.file.flush()
                self._last_flush_time = time.time()
            except Exception:
                pass
    
    def stop(self) -> Dict:
        """
        Stop recording and close the file.
        
        Returns:
            Stats dict with record_count, dropped_count
        """
        self.recording = False
        
        stats = {
            'record_count': self._record_count,
            'dropped_count': self._dropped_count,
            'filepath': self.filepath
        }
        
        # Signal writer thread to stop
        if self._write_queue is not None:
            try:
                self._write_queue.put(self._STOP_SENTINEL, timeout=1.0)
            except Exception:
                pass
        
        # Wait for writer thread to finish
        if self._writer_thread is not None and self._writer_thread.is_alive():
            self._writer_thread.join(timeout=2.0)
        
        # Close file
        if self.file is not None:
            try:
                self.file.flush()
                self.file.close()
            except Exception:
                pass
            self.file = None
            self.writer = None
        
        self._write_queue = None
        self._writer_thread = None
        
        return stats
    
    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self.recording
    
    def get_filepath(self) -> Optional[str]:
        """Get the path to the current recording file."""
        return self.filepath
    
    def get_stats(self) -> Dict:
        """Get current recording statistics."""
        return {
            'record_count': self._record_count,
            'dropped_count': self._dropped_count,
            'queue_size': self._write_queue.qsize() if self._write_queue else 0,
            'filepath': self.filepath
        }
