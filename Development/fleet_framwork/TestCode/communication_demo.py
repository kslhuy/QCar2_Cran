"""
Example script demonstrating Vehicle-to-Vehicle Communication
This shows how vehicles can communicate their state and use that data for control.
"""

import time
import logging
from Vehicle import Vehicle
from FleetConfig import FleetConfig, ConfigPresets

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

def demo_communication():
    """Demonstrate vehicle communication without QLabs setup."""
    print("=== Vehicle Communication Demo ===")
    
    # Create a simple config
    config = ConfigPresets.get_studio_cacc()
    
    # Create mock vehicles (without actual QLabs connection)
    # In real usage, you would pass actual QLabsQCar2 instances
    
    # For demo purposes, we'll simulate without QLabs
    leader_vehicle = Vehicle(
        vehicle_id=0,
        qcar=None,  # In real scenario, pass QLabsQCar2 instance
        controller_type="CACC",
        is_leader=True,
        config=config,
        target_ip="127.0.0.1",
        base_send_port=8000,
        base_recv_port=9000,
        base_ack_port=10000
    )
    
    follower_vehicle = Vehicle(
        vehicle_id=1,
        qcar=None,  # In real scenario, pass QLabsQCar2 instance
        controller_type="CACC",
        is_leader=False,
        config=config,
        target_ip="127.0.0.1",
        base_send_port=8000,
        base_recv_port=9000,
        base_ack_port=10000
    )
    
    # Set up follower to follow leader
    follower_vehicle.set_leader(leader_vehicle)
    
    print(f"Leader Vehicle {leader_vehicle.vehicle_id} ports:")
    print(f"  Send: {leader_vehicle.send_port}")
    print(f"  Receive: {leader_vehicle.recv_port}")
    print(f"  ACK: {leader_vehicle.ack_port}")
    
    print(f"Follower Vehicle {follower_vehicle.vehicle_id} ports:")
    print(f"  Send: {follower_vehicle.send_port}")
    print(f"  Receive: {follower_vehicle.recv_port}")
    print(f"  ACK: {follower_vehicle.ack_port}")
    
    print(f"Follower sends to Leader's receive port: {follower_vehicle.comm.send_port}")
    
    # Simulate some vehicle state for demonstration
    leader_vehicle.current_pos = [10.0, 5.0, 0.0]
    leader_vehicle.current_rot = [0.0, 0.0, 0.1]
    leader_vehicle.velocity = 2.5
    
    follower_vehicle.current_pos = [8.0, 5.0, 0.0]
    follower_vehicle.current_rot = [0.0, 0.0, 0.0]
    follower_vehicle.velocity = 2.0
    
    try:
        # Start communication threads (without control threads for this demo)
        print("\\nStarting communication...")
        
        # Start the vehicles - this will start communication threads
        # Note: This will fail without actual QLabs connection, but shows the concept
        # leader_vehicle.start()
        # follower_vehicle.start()
        
        print("Communication system initialized!")
        print("\\nIn a real scenario:")
        print("1. Leader vehicle sends state (position, rotation, velocity) via UDP")
        print("2. Follower vehicle receives leader state")
        print("3. Follower uses leader state in its controller for formation control")
        print("4. ACK mechanism ensures reliable communication")
        
        # Show vehicle states
        print("\\n=== Vehicle States ===")
        leader_state = leader_vehicle.get_state()
        follower_state = follower_vehicle.get_state()
        
        print(f"Leader State: {leader_state}")
        print(f"Follower State: {follower_state}")
        
        print("\\n=== Communication Flow ===")
        print("Leader -> Follower:")
        print(f"  Data: position={leader_vehicle.current_pos}, velocity={leader_vehicle.velocity}")
        print(f"  Network: {leader_vehicle.target_ip}:{follower_vehicle.recv_port}")
        print(f"  ACK: {follower_vehicle.target_ip}:{leader_vehicle.ack_port}")
        
    except Exception as e:
        print(f"Demo error (expected without QLabs): {e}")
    
    finally:
        print("\\nDemo completed!")
        print("\\nTo use in real scenario:")
        print("1. Replace 'qcar=None' with actual QLabsQCar2 instances")
        print("2. Ensure QLabs simulation is running")
        print("3. Configure proper IP addresses if using multiple machines")
        print("4. Start vehicles with vehicle.start()")
        print("5. Monitor communication via logging")

if __name__ == "__main__":
    demo_communication()
