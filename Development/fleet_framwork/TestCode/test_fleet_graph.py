#!/usr/bin/env python3
"""
Test script for QcarFleet graph functionality

This script demonstrates how to use the adjacency matrix graph representation
for fleet vehicle communication topology.
"""

import numpy as np
import sys
import os

# Add the current directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Mock configuration class for testing
class MockConfig:
    def __init__(self):
        self.max_steering = 0.6
        self.lookahead_distance = 7.0
        self.simulation_time = 60
        self.enable_steering_control = True
        self.node_sequence = [0, 1, 2]
        self.dummy_controller_params = {}
        self.use_physical_qcar = False
        self.use_control_observer_mode = True
    
    def get_road_type_name(self):
        return "OpenRoad"
    
    def get_controller_type_name(self):
        return "CACC"

def test_fleet_graphs():
    """
    Test different graph topologies for the fleet
    """
    print("=" * 60)
    print("QCAR FLEET GRAPH TOPOLOGY TESTING")
    print("=" * 60)
    
    # Mock the QcarFleet class for testing (without actual QLabs connection)
    try:
        from QcarFleet import QcarFleet
    except ImportError:
        print("Error: Could not import QcarFleet. Make sure the file exists.")
        return
    
    config = MockConfig()
    
    # Test 1: 4 vehicles, fully connected (like your MATLAB example)
    print("\n1. FULLY CONNECTED TOPOLOGY (4 vehicles)")
    print("-" * 40)
    
    try:
        # Create a mock fleet (this will fail at QLabs connection, but we can still test graph creation)
        fleet = QcarFleet.__new__(QcarFleet)  # Create instance without calling __init__
        fleet.NumQcar = 4
        fleet.LeaderIndex = 0
        fleet.QcarIndexList = range(0, 4)
        fleet.Distance = 5.0
        fleet.Controller = "CACC"
        fleet.Observer = "EKF"
        fleet.config = config
        
        # Test graph creation
        fully_connected_graph = fleet.create_fleet_graph("fully_connected")
        print("Fully connected graph (4x4):")
        fleet.print_fleet_graph(fully_connected_graph)
        
        # Verify it matches your MATLAB example
        expected_matlab = np.array([
            [0, 1, 1, 1],
            [1, 0, 1, 1],
            [1, 1, 0, 1],
            [1, 1, 1, 0]
        ])
        
        if np.array_equal(fully_connected_graph, expected_matlab):
            print("✓ Graph matches MATLAB example!")
        else:
            print("✗ Graph doesn't match MATLAB example")
            
    except Exception as e:
        print(f"Error creating fully connected graph: {e}")
    
    # Test 2: Chain topology
    print("\n2. CHAIN TOPOLOGY (4 vehicles)")
    print("-" * 40)
    
    try:
        chain_graph = fleet.create_fleet_graph("chain")
        fleet.print_fleet_graph(chain_graph)
        
        # Test vehicle connections
        for i in range(4):
            connections = [j for j in range(4) if chain_graph[i][j] == 1]
            print(f"Vehicle {i} connected to: {connections}")
            
    except Exception as e:
        print(f"Error creating chain graph: {e}")
    
    # Test 3: Star topology
    print("\n3. STAR TOPOLOGY (4 vehicles, leader=0)")
    print("-" * 40)
    
    try:
        star_graph = fleet.create_fleet_graph("star")
        fleet.print_fleet_graph(star_graph)
        
        # Test vehicle connections
        for i in range(4):
            connections = [j for j in range(4) if star_graph[i][j] == 1]
            print(f"Vehicle {i} connected to: {connections}")
            
    except Exception as e:
        print(f"Error creating star graph: {e}")
    
    # Test 4: Custom topology (3 vehicles)
    print("\n4. CUSTOM TOPOLOGY (3 vehicles)")
    print("-" * 40)
    
    try:
        fleet.NumQcar = 3  # Change to 3 vehicles
        custom_graph = fleet.create_fleet_graph("custom")
        fleet.print_fleet_graph(custom_graph)
        
    except Exception as e:
        print(f"Error creating custom graph: {e}")
    
    # Test 5: Graph utility functions
    print("\n5. GRAPH UTILITY FUNCTIONS")
    print("-" * 40)
    
    try:
        fleet.NumQcar = 4  # Back to 4 vehicles
        fleet.fleet_graph = fully_connected_graph
        
        # Test connection queries
        print(f"Vehicle 0 connected to vehicle 1: {fleet.is_connected(0, 1)}")
        print(f"Vehicle 0 connections: {fleet.get_vehicle_connections(0)}")
        
        # Test graph update
        print("\nUpdating to chain topology:")
        new_chain = fleet.create_fleet_graph("chain")
        fleet.update_fleet_graph(new_chain)
        
    except Exception as e:
        print(f"Error testing utility functions: {e}")
    
    print("\n" + "=" * 60)
    print("GRAPH TESTING COMPLETE")
    print("=" * 60)

def demonstrate_graph_usage():
    """
    Demonstrate how to integrate graph with actual fleet usage
    """
    print("\n" + "=" * 60)
    print("GRAPH USAGE DEMONSTRATION")
    print("=" * 60)
    
    print("""
How to use the graph functionality in your QcarFleet:

1. CREATE FLEET WITH SPECIFIC TOPOLOGY:
   fleet = QcarFleet(NumQcar=4, LeaderIndex=0, Distance=5.0, 
                     Controller="CACC", Observer="EKF")
   
   The graph is automatically created as "fully_connected" by default.

2. CHANGE TOPOLOGY:
   # Change to chain topology
   chain_graph = fleet.create_fleet_graph("chain")
   fleet.update_fleet_graph(chain_graph)
   
   # Change to star topology  
   star_graph = fleet.create_fleet_graph("star")
   fleet.update_fleet_graph(star_graph)

3. CREATE CUSTOM TOPOLOGY:
   custom_graph = np.array([
       [0, 1, 0, 1],  # Vehicle 0 -> 1, 3
       [1, 0, 1, 0],  # Vehicle 1 -> 0, 2  
       [0, 1, 0, 1],  # Vehicle 2 -> 1, 3
       [1, 0, 1, 0]   # Vehicle 3 -> 0, 2
   ])
   fleet.update_fleet_graph(custom_graph)

4. QUERY CONNECTIONS:
   connections = fleet.get_vehicle_connections(0)  # Get vehicle 0's connections
   is_connected = fleet.is_connected(0, 1)         # Check if 0 and 1 are connected
   graph = fleet.get_fleet_graph()                 # Get current graph

5. EACH VEHICLE PROCESS RECEIVES:
   - 'fleet_graph': Complete adjacency matrix as list
   - 'connected_vehicles': List of vehicle IDs this vehicle can talk to  
   - 'num_connections': Number of connections for this vehicle
   - 'peer_ports': Communication ports (filtered by graph connections)
    """)

if __name__ == "__main__":
    test_fleet_graphs()
    demonstrate_graph_usage()
