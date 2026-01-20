
import unittest
import time
import struct
import json
import threading
import sys
import os

# Add parent directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from scope_data_streamer import ScopeDataStreamer, SCOPE_CONFIG_HEADER, SCOPE_DATA_HEADER
from GUI.qcar_gui.controllers.remote_scope_manager import RemoteScopeManager

class MockClient:
    def __init__(self, manager):
        self.manager = manager
        
    def queue_scope_data(self, packet):
        # Simulate network transmission to manager
        # Packet is bytes
        payload = packet.hex()
        # Extract vehicle ID from packet
        # Header(1) + VehicleID(1)
        vehicle_id = packet[1]
        self.manager.receive_scope_data(vehicle_id, payload)
        return True
        
    def queue_telemetry(self, data):
        if data['type'] == 'scope_data':
            payload = data['payload']
            # We can't easily get ID from hex payload without parsing, 
            # but for test we know it's vehicle 1
            self.manager.receive_scope_data(1, payload)
        return True

class TestDynamicFleetStreaming(unittest.TestCase):
    def test_dynamic_config(self):
        print("\nTesting Dynamic Fleet Configuration...")
        
        # 1. Setup Ground Station side
        manager = RemoteScopeManager()
        manager.start_stream(1, ['local_state']) # Initially just local
        
        # Verify initial fields
        self.assertEqual(manager.vehicle_field_names[1], 
                         ['x', 'y', 'theta', 'velocity', 'x_gps', 'y_gps', 'theta_gps', 'v_ref'])
        
        # 2. Setup Vehicle side
        client = MockClient(manager)
        streamer = ScopeDataStreamer(client, vehicle_id=1, stream_rate=50.0)
        
        # Enable with dynamic fleet (V2, V3)
        fleet_ids = [2, 3]
        streamer.enable(['local_state', 'fleet_dynamic'], fleet_ids=fleet_ids)
        
        # Check streamer fields
        print(f"Streamer Active Fields: {streamer.active_fields}")
        self.assertIn('fleet_x_2', streamer.active_fields)
        self.assertIn('fleet_x_3', streamer.active_fields)
        
        # 3. Simulate streaming loop
        # First call should send config packet because last_config_time is 0
        t = 0.0
        data = {
            'x': 1.0, 'y': 2.0, 
            'fleet_x_2': 12.0, 'fleet_y_2': 22.0,
            'fleet_x_3': 13.0, 'fleet_y_3': 23.0
        }
        
        # Force config send (stream_sample checks time, might not trigger immediately if time doesn't advance enough manually)
        # But we can call send_config directly
        streamer.send_config()
        
        # Check if manager updated fields
        print(f"Manager Fields Car 1: {manager.vehicle_field_names[1]}")
        self.assertIn('fleet_x_2', manager.vehicle_field_names[1])
        self.assertIn('fleet_x_3', manager.vehicle_field_names[1])
        
        # 4. Stream data
        streamer.stream_sample(t, data)
        
        # Check buffer
        buffer = manager.get_buffer(1)
        self.assertIsNotNone(buffer)
        latest = buffer.get_latest_values()
        print(f"Latest Buffer Data: {latest}")
        
        self.assertAlmostEqual(latest['x'], 1.0)
        self.assertAlmostEqual(latest['fleet_x_2'], 12.0)
        self.assertAlmostEqual(latest['fleet_x_3'], 13.0)
        
        print("✓ Dynamic Fleet Config and Streaming Verified")

if __name__ == '__main__':
    unittest.main()
