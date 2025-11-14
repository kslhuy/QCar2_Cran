"""
V2V Communication Test Script
Tests the V2V functionality and Ground Station reporting
"""
import time
import json
from v2v_communication import V2VCommunication
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("V2V_Test")

def test_v2v_communication():
    """Test V2V communication between simulated vehicles"""
    print("🚗 Starting V2V Communication Test")
    print("=" * 50)
    
    # Create two V2V instances (simulating two vehicles)
    v2v_car0 = V2VCommunication(vehicle_id=0, logger=logger, base_port=6000)
    v2v_car1 = V2VCommunication(vehicle_id=1, logger=logger, base_port=6000)
    
    # Test 1: Activate V2V communication
    print("\n📡 Test 1: Activating V2V communication")
    
    # Car 0 connects to Car 1
    success0 = v2v_car0.activate([0, 1], ["127.0.0.1"])
    print(f"Car 0 activation: {'✅ Success' if success0 else '❌ Failed'}")
    
    # Car 1 connects to Car 0  
    success1 = v2v_car1.activate([0, 1], ["127.0.0.1"])
    print(f"Car 1 activation: {'✅ Success' if success1 else '❌ Failed'}")
    
    # Wait for connections to establish
    time.sleep(3)
    
    # Test 2: Check connection status
    print("\n🔗 Test 2: Checking connection status")
    
    stats0 = v2v_car0.get_statistics()
    stats1 = v2v_car1.get_statistics()
    
    print(f"Car 0 connected peers: {stats0['connected_peers']}")
    print(f"Car 1 connected peers: {stats1['connected_peers']}")
    
    # Test 3: Send messages
    print("\n💬 Test 3: Sending V2V messages")
    
    # Car 0 sends telemetry
    v2v_car0.send_telemetry({
        'x': 10.0, 'y': 5.0, 'theta': 0.5, 'velocity': 2.0
    })
    print("Car 0 sent telemetry")
    
    # Car 1 sends intent
    v2v_car1.send_intent('lane_change', {'direction': 'left', 'urgency': 'medium'})
    print("Car 1 sent intent")
    
    # Car 0 sends warning
    v2v_car0.send_warning('obstacle', 'high', {'distance': 15.0, 'type': 'pedestrian'})
    print("Car 0 sent warning")
    
    # Wait for message processing
    time.sleep(2)
    
    # Test 4: Check message statistics
    print("\n📊 Test 4: Message statistics")
    
    stats0_final = v2v_car0.get_statistics()
    stats1_final = v2v_car1.get_statistics()
    
    print(f"Car 0 - Sent: {stats0_final['messages_sent']}, Received: {stats0_final['messages_received']}")
    print(f"Car 1 - Sent: {stats1_final['messages_sent']}, Received: {stats1_final['messages_received']}")
    
    # Test 5: Verify Ground Station reporting format
    print("\n📡 Test 5: Ground Station report format")
    
    # Simulate the reporting format that would be sent to Ground Station
    gs_report_car0 = {
        'type': 'v2v_status',
        'car_id': 0,
        'data': {
            'status': 'connected',
            'expected_peers': 1,
            'connected_peers': len(v2v_car0.get_connected_peers()),
            'peer_list': v2v_car0.get_connected_peers(),
            'vehicle_id': 0,
            'timestamp': time.time()
        }
    }
    
    gs_report_car1 = {
        'type': 'v2v_status', 
        'car_id': 1,
        'data': {
            'status': 'connected',
            'expected_peers': 1,
            'connected_peers': len(v2v_car1.get_connected_peers()),
            'peer_list': v2v_car1.get_connected_peers(),
            'vehicle_id': 1,
            'timestamp': time.time()
        }
    }
    
    print("Sample Ground Station reports:")
    print(f"Car 0: {json.dumps(gs_report_car0, indent=2)}")
    print(f"Car 1: {json.dumps(gs_report_car1, indent=2)}")
    
    # Verify network consistency
    car0_peers = len(v2v_car0.get_connected_peers())
    car1_peers = len(v2v_car1.get_connected_peers())
    
    if car0_peers == car1_peers == 1:
        print("\n🎉 V2V NETWORK SUCCESS!")
        print("✅ Both vehicles report 1 connected peer")
        print("✅ Network topology is consistent")
    else:
        print(f"\n⚠️ V2V network issue: Car0={car0_peers} peers, Car1={car1_peers} peers")
    
    # Cleanup
    print("\n🧹 Cleaning up...")
    v2v_car0.deactivate()
    v2v_car1.deactivate()
    
    print("✅ Test completed!")

if __name__ == "__main__":
    test_v2v_communication()