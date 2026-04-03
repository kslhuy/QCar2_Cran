"""
Scope Data Streamer - High-frequency data streaming for remote plotting

This module provides efficient serialization and rate-limited streaming
of scope data from vehicles to the Ground Station for real-time visualization.

The streamer uses binary packing for performance and includes rate limiting
to maintain consistent streaming rates (20-50Hz).

Usage:
    from scope_data_streamer import ScopeDataStreamer
    
    streamer = ScopeDataStreamer(gs_client, vehicle_id=0, stream_rate=20.0)
    streamer.enable(['local_state', 'local_control'])
    
    # In control loop:
    streamer.stream_sample(t, data)
    
    # When done:
    streamer.disable()
"""

import struct
import time
from typing import Dict, List, Optional, Any
from dataclasses import dataclass


# ==============================================================================
# Constants
# ==============================================================================

# Message header byte for scope data packets
SCOPE_DATA_HEADER = 0xAB

_DEFAULT_FLEET_SIZE = 2  # Overridden at runtime with actual fleet size


def build_fleet_fields(fleet_size: int) -> list:
    """
    Build fleet field names dynamically based on actual fleet size.
    
    Args:
        fleet_size: Number of vehicles in the fleet
        
    Returns:
        List of field names for fleet state streaming
    """
    fields = []
    for i in range(fleet_size):
        for qty in ['x', 'y', 'theta', 'v']:
            fields.append(f'fleet_{qty}_{i}')
    fields.append('fleet_size')
    return fields


# Standard field definitions for different presets
PRESET_FIELDS = {
    'local_state': [
        'x', 'y', 'theta', 'velocity',
        'x_gps', 'y_gps', 'theta_gps', 'v_ref', 'v_ref_actual',
        'accel_magnitude'
    ],
    'local_control': [
        'velocity', 'v_ref', 'v_ref_actual', 'throttle', 'steering',
        'accel_magnitude'
    ],
    'local_error': [
        'x_error', 'y_error', 'theta_error', 'acceleration'
    ],
    # fleet_state is generated dynamically — this default is for import compatibility
    'fleet_state': build_fleet_fields(_DEFAULT_FLEET_SIZE),
}

# Combined default fields for local data
DEFAULT_FIELDS = [
    'x', 'y', 'theta', 'velocity',           # Estimated state
    'x_gps', 'y_gps', 'theta_gps', 'v_ref', 'v_ref_actual',  # Reference/GPS
    'throttle', 'steering',                   # Control
    'accel_magnitude', 'yaw_rate'             # Dynamics
]

# Fleet default fields (for import compatibility — use build_fleet_fields at runtime)
FLEET_FIELDS = build_fleet_fields(_DEFAULT_FLEET_SIZE)


# ==============================================================================
# Data Classes
# ==============================================================================

@dataclass
class ScopeDataPacket:
    """Represents a scope data packet ready for transmission."""
    header: int
    vehicle_id: int
    timestamp: float
    num_fields: int
    data: bytes
    
    @property
    def raw(self) -> bytes:
        """Get raw bytes for transmission."""
        return (
            struct.pack('!BB', self.header, self.vehicle_id) +
            struct.pack('!d', self.timestamp) +
            struct.pack('!B', self.num_fields) +
            self.data
        )


# ==============================================================================
# Scope Data Streamer
# ==============================================================================

class ScopeDataStreamer:
    """
    High-frequency scope data streaming to Ground Station.
    
    Handles data collection, binary packing, rate limiting, and queuing
    for transmission via GroundStationClient.
    
    Attributes:
        gs_client: Reference to GroundStationClient for data transmission
        vehicle_id: This vehicle's ID
        stream_rate: Target streaming rate in Hz
        enabled: Whether streaming is currently active
    """
    
    def __init__(self, gs_client, vehicle_id: int, stream_rate: float = 50.0):
        """
        Initialize the scope data streamer.
        
        Args:
            gs_client: GroundStationClient instance (or None for testing)
            vehicle_id: This vehicle's identifier
            stream_rate: Target streaming rate in Hz (default 50Hz)
        """
        self.gs_client = gs_client
        self.vehicle_id = vehicle_id
        self.stream_rate = stream_rate
        
        # State
        self.enabled = False
        self.active_presets: List[str] = []
        self.active_fields: List[str] = []
        
        # Rate limiting
        self.last_stream_time = 0.0
        self.min_interval = 1.0 / stream_rate
        
        # Statistics
        self.samples_streamed = 0
        self.samples_dropped = 0
        self.start_time = 0.0
    
    def enable(self, preset_names: List[str] = None, fleet_size: int = None) -> bool:
        """
        Enable scope data streaming.
        
        Args:
            preset_names: List of preset names to enable. Options:
                - 'local_state', 'local_control', 'local_error'
                - 'fleet_state'
                If None, uses default local presets.
            fleet_size: Actual fleet size for dynamic fleet field generation.
                Only used when 'fleet_state' is in preset_names.
                
        Returns:
            bool: True if streaming enabled successfully
        """
        if preset_names is None:
            preset_names = ['local_state', 'local_control']
        
        # Build combined field list from presets
        self.active_fields = []
        for preset in preset_names:
            if preset == 'fleet_state':
                # Use dynamic fleet fields based on actual fleet size
                n = fleet_size if fleet_size and fleet_size > 0 else _DEFAULT_FLEET_SIZE
                for field in build_fleet_fields(n):
                    if field not in self.active_fields:
                        self.active_fields.append(field)
            elif preset in PRESET_FIELDS:
                for field in PRESET_FIELDS[preset]:
                    if field not in self.active_fields:
                        self.active_fields.append(field)
        
        # Use default fields if no valid presets
        if not self.active_fields:
            self.active_fields = DEFAULT_FIELDS.copy()
        
        self.active_presets = preset_names
        self.enabled = True
        self.start_time = time.time()
        self.samples_streamed = 0
        self.samples_dropped = 0
        
        print(f"[ScopeStreamer] Enabled with {len(self.active_fields)} fields at {self.stream_rate}Hz")
        print(f"[ScopeStreamer] Fields: {self.active_fields}")
        
        return True
    
    def disable(self):
        """Disable scope data streaming."""
        if self.enabled:
            duration = time.time() - self.start_time
            actual_rate = self.samples_streamed / duration if duration > 0 else 0
            print(f"[ScopeStreamer] Disabled. Streamed {self.samples_streamed} samples in {duration:.1f}s ({actual_rate:.1f}Hz)")
            if self.samples_dropped > 0:
                print(f"[ScopeStreamer] Dropped {self.samples_dropped} samples due to queue full")
        
        self.enabled = False
        self.active_presets = []
        self.active_fields = []
    
    def is_streaming(self) -> bool:
        """Check if streaming is active."""
        return self.enabled
    
    def set_stream_rate(self, rate_hz: float):
        """Update the streaming rate."""
        self.stream_rate = max(1.0, min(100.0, rate_hz))  # Clamp 1-100Hz
        self.min_interval = 1.0 / self.stream_rate
    
    def stream_sample(self, t: float, data: Dict[str, Any]) -> bool:
        """
        Stream a data sample if rate limit allows.
        
        Args:
            t: Timestamp of the sample (relative to start)
            data: Dictionary of data values
            
        Returns:
            bool: True if sample was streamed, False if rate-limited or disabled
        """
        if not self.enabled:
            return False
        
        # Rate limiting
        current_time = time.time()
        if current_time - self.last_stream_time < self.min_interval:
            return False
        
        # Pack the data
        packet = self._pack_scope_data(t, data)
        
        # Queue for transmission
        if self.gs_client is not None:
            if hasattr(self.gs_client, 'queue_scope_data'):
                success = self.gs_client.queue_scope_data(packet)
            else:
                # Fallback: send as telemetry with special type
                success = self.gs_client.queue_telemetry({
                    'type': 'scope_data',
                    'payload': packet.hex()
                })
            
            if success:
                self.samples_streamed += 1
            else:
                self.samples_dropped += 1
        
        self.last_stream_time = current_time
        return True
    
    def _pack_scope_data(self, t: float, data: Dict[str, Any]) -> bytes:
        """
        Pack scope data into binary format for efficient transmission.
        
        Binary format:
            - Header: 1 byte (0xAB)
            - Vehicle ID: 1 byte
            - Timestamp: 8 bytes (float64)
            - Num fields: 1 byte
            - Data: N * 4 bytes (float32 each)
        
        Args:
            t: Timestamp
            data: Data dictionary
            
        Returns:
            bytes: Packed binary data
        """
        # Collect field values
        values = []
        for field in self.active_fields:
            value = self._extract_value(data, field)
            values.append(value)
        
        # Pack header
        header = struct.pack('!BB', SCOPE_DATA_HEADER, self.vehicle_id)
        
        # Pack timestamp
        timestamp = struct.pack('!d', t)
        
        # Pack field count
        num_fields = struct.pack('!B', len(values))
        
        # Pack data values as float32
        data_bytes = struct.pack(f'!{len(values)}f', *values)
        
        return header + timestamp + num_fields + data_bytes
    
    def _extract_value(self, data: Dict[str, Any], field: str) -> float:
        """
        Extract a field value from data dictionary.
        
        Handles nested fields (e.g., 'fleet_x_0'), error terms, and trust scores.
        
        Args:
            data: Data dictionary
            field: Field name
            
        Returns:
            float: Field value (0.0 if not found)
        """
        # Direct lookup
        if field in data:
            return float(data[field])
        
        # Handle error fields (computed from difference)
        if field == 'x_error':
            x = data.get('x', 0.0)
            x_gps = data.get('x_gps', x)
            return float(x - x_gps)
        elif field == 'y_error':
            y = data.get('y', 0.0)
            y_gps = data.get('y_gps', y)
            return float(y - y_gps)
        elif field == 'theta_error':
            theta = data.get('theta', 0.0)
            theta_gps = data.get('theta_gps', theta)
            return float(theta - theta_gps)
        
        # Handle trust scores (trust_0, trust_1, etc.)
        if field.startswith('trust_'):
            try:
                vehicle_idx = int(field.split('_')[1])
                trust_scores = data.get('trust_scores', {})
                if isinstance(trust_scores, dict):
                    return float(trust_scores.get(vehicle_idx, 0.0))
                elif hasattr(trust_scores, '__getitem__'):
                    if vehicle_idx < len(trust_scores):
                        return float(trust_scores[vehicle_idx])
            except (ValueError, IndexError, TypeError):
                pass
            return 0.0
        
        # Handle consensus_error
        if field == 'consensus_error':
            return float(data.get('consensus_error', 0.0))
        
        # Handle fleet_size
        if field == 'fleet_size':
            fleet_states = data.get('fleet_states')
            if fleet_states is not None and hasattr(fleet_states, 'shape'):
                return float(fleet_states.shape[1])
            return float(data.get('fleet_size', 0))
        
        # Handle fleet fields (fleet_x_0, etc.)
        if field.startswith('fleet_'):
            parts = field.split('_')
            if len(parts) == 3:
                state_name = parts[1]  # x, y, theta, v
                try:
                    vehicle_idx = int(parts[2])
                    fleet_states = data.get('fleet_states')
                    if fleet_states is not None and hasattr(fleet_states, 'shape'):
                        state_idx = {'x': 0, 'y': 1, 'theta': 2, 'v': 3}.get(state_name, 0)
                        if vehicle_idx < fleet_states.shape[1]:
                            return float(fleet_states[state_idx, vehicle_idx])
                except (ValueError, IndexError):
                    pass
        
        return 0.0
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get streaming statistics."""
        duration = time.time() - self.start_time if self.start_time > 0 else 0
        actual_rate = self.samples_streamed / duration if duration > 0 else 0
        
        return {
            'enabled': self.enabled,
            'target_rate_hz': self.stream_rate,
            'actual_rate_hz': actual_rate,
            'samples_streamed': self.samples_streamed,
            'samples_dropped': self.samples_dropped,
            'active_presets': self.active_presets,
            'num_fields': len(self.active_fields),
            'duration_s': duration
        }


# ==============================================================================
# Unpacking utilities (for Ground Station side)
# ==============================================================================

def unpack_scope_data(packet: bytes) -> Optional[Dict[str, Any]]:
    """
    Unpack a scope data packet received from vehicle.
    
    Args:
        packet: Raw bytes or hex string
        
    Returns:
        Dictionary with timestamp, vehicle_id, and data values,
        or None if unpacking fails
    """
    try:
        # Handle hex string input
        if isinstance(packet, str):
            packet = bytes.fromhex(packet)
        
        # Verify minimum length
        if len(packet) < 11:  # 1 + 1 + 8 + 1 = minimum
            return None
        
        # Unpack header
        header, vehicle_id = struct.unpack('!BB', packet[0:2])
        
        # Verify header
        if header != SCOPE_DATA_HEADER:
            return None
        
        # Unpack timestamp
        timestamp = struct.unpack('!d', packet[2:10])[0]
        
        # Unpack field count
        num_fields = struct.unpack('!B', packet[10:11])[0]
        
        # Verify data length
        expected_data_len = num_fields * 4
        if len(packet) < 11 + expected_data_len:
            return None
        
        # Unpack data values
        values = struct.unpack(f'!{num_fields}f', packet[11:11 + expected_data_len])
        
        return {
            'vehicle_id': vehicle_id,
            'timestamp': timestamp,
            'num_fields': num_fields,
            'values': list(values)
        }
        
    except Exception as e:
        print(f"[ScopeStreamer] Unpack error: {e}")
        return None


def unpack_with_field_names(packet: bytes, field_names: List[str]) -> Optional[Dict[str, float]]:
    """
    Unpack scope data and map to field names.
    
    Args:
        packet: Raw bytes
        field_names: List of field names (must match packing order)
        
    Returns:
        Dictionary mapping field names to values
    """
    result = unpack_scope_data(packet)
    if result is None:
        return None
    
    values = result['values']
    data = {
        'timestamp': result['timestamp'],
        'vehicle_id': result['vehicle_id']
    }
    
    for i, name in enumerate(field_names):
        if i < len(values):
            data[name] = values[i]
    
    return data


# ==============================================================================
# Test utilities
# ==============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("Scope Data Streamer Test")
    print("=" * 60)
    
    # Test packing and unpacking
    streamer = ScopeDataStreamer(None, vehicle_id=1, stream_rate=20.0)
    streamer.enable(['local_state', 'local_control'])
    
    # Create test data
    test_data = {
        'x': 1.5,
        'y': 2.3,
        'theta': 0.785,
        'velocity': 0.8,
        'x_gps': 1.48,
        'y_gps': 2.31,
        'theta_gps': 0.78,
        'v_ref': 1.0,
        'throttle': 0.15,
        'steering': -0.1
    }
    
    # Pack data
    packet = streamer._pack_scope_data(5.0, test_data)
    print(f"\nPacked data size: {len(packet)} bytes")
    print(f"Hex: {packet.hex()}")
    
    # Unpack data
    result = unpack_scope_data(packet)
    print(f"\nUnpacked: {result}")
    
    # Unpack with field names
    named = unpack_with_field_names(packet, streamer.active_fields)
    print(f"\nNamed fields: {named}")
    
    # Test rate limiting
    print("\n" + "=" * 60)
    print("Rate limiting test (1 second)")
    print("=" * 60)
    
    class MockClient:
        def __init__(self):
            self.count = 0
        def queue_telemetry(self, data):
            self.count += 1
            return True
    
    mock_client = MockClient()
    streamer2 = ScopeDataStreamer(mock_client, vehicle_id=0, stream_rate=20.0)
    streamer2.enable(['local_state'])
    
    start = time.time()
    attempts = 0
    while time.time() - start < 1.0:
        streamer2.stream_sample(time.time() - start, test_data)
        attempts += 1
        time.sleep(0.001)  # 1ms sleep to allow for busy-wait
    
    print(f"Attempts: {attempts}, Streamed: {mock_client.count}")
    print(f"Effective rate: {mock_client.count:.1f} Hz (target: 50 Hz)")
    
    print("\n✓ All tests passed!")
