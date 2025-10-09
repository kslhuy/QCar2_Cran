#!/usr/bin/env python3
"""
Example script demonstrating the new multi-file logging system for fleet vehicles.

This script shows how to use different loggers for different purposes:
- Individual vehicle operations
- Communication data (send/receive)
- GPS synchronization events
- Control operations

Run this script to see how logs are separated into different files.
"""

import time
from md_logging_config import (
    get_individual_vehicle_logger,
    get_communication_logger, 
    get_gps_logger,
    get_control_logger,
    enable_console_logging
)

def demonstrate_logging():
    """Demonstrate the different logging capabilities."""
    
    # Enable console output to see logs in terminal (optional)
    enable_console_logging(True)
    
    print("=== Fleet Logging System Demonstration ===")
    print("This will create separate log files in the 'logs' directory:")
    print("- logs/vehicle_1.log (individual vehicle general operations)")
    print("- logs/vehicle_2.log (individual vehicle general operations)")
    print("- logs/communication_vehicle_1.log (Vehicle 1 communication)")
    print("- logs/communication_vehicle_2.log (Vehicle 2 communication)")
    print("- logs/gps_vehicle_1.log (Vehicle 1 GPS events)")
    print("- logs/gps_vehicle_2.log (Vehicle 2 GPS events)")
    print("- logs/control_vehicle_1.log (Vehicle 1 control operations)")
    print("- logs/control_vehicle_2.log (Vehicle 2 control operations)")
    print("- logs/fleet_operations.log (general fleet coordination)")
    print()
    
    # Simulate Vehicle 1 operations
    print("Simulating Vehicle 1 operations...")
    vehicle1_logger = get_individual_vehicle_logger(1)
    comm1_logger = get_communication_logger(1)
    gps1_logger = get_gps_logger(1)
    control1_logger = get_control_logger(1)
    
    vehicle1_logger.info("Vehicle 1 initialized and starting operations")
    gps1_logger.info("Initial GPS synchronization completed, offset: +0.023 sec")
    control1_logger.info("Leader controller activated, starting autonomous driving")
    comm1_logger.info("SENT: Seq: 1, Pos: [10.5, 20.3, 0.0], V: 15.2")
    comm1_logger.info("ACK received for seq: 1 from Vehicle 2")
    
    time.sleep(1)
    
    # Simulate Vehicle 2 operations
    print("Simulating Vehicle 2 operations...")
    vehicle2_logger = get_individual_vehicle_logger(2)
    comm2_logger = get_communication_logger(2)
    gps2_logger = get_gps_logger(2)
    control2_logger = get_control_logger(2)
    
    vehicle2_logger.info("Vehicle 2 initialized as follower")
    gps2_logger.info("Initial GPS synchronization completed, offset: +0.019 sec")
    control2_logger.info("Follower controller (CACC) activated")
    comm2_logger.info("RECEIVED STATE: Seq: 1, Sender ID: 1, Pos: [10.5, 20.3, 0.0]")
    comm2_logger.info("SENT: ACK for seq: 1 to Vehicle 1")
    control2_logger.info("Following leader at distance: 25.3m, speed: 14.8")
    
    time.sleep(1)
    
    # Simulate ongoing operations
    print("Simulating ongoing fleet operations...")
    for i in range(3):
        # Vehicle 1 (Leader) operations
        gps1_logger.debug(f"GPS sync check #{i+1} completed")
        comm1_logger.info(f"SENT: Seq: {i+2}, Pos: [{10.5+i}, {20.3+i*0.5}, 0.0], V: {15.2-i*0.1}")
        control1_logger.debug(f"Leader waypoint #{i+1} reached, continuing to next")
        
        # Vehicle 2 (Follower) operations  
        comm2_logger.info(f"RECEIVED STATE: Seq: {i+2}, Sender ID: 1, Pos: [{10.5+i}, {20.3+i*0.5}, 0.0]")
        control2_logger.info(f"Adjusting speed to maintain distance: {25.3-i*0.5}m")
        gps2_logger.debug(f"GPS time validation passed for received state #{i+2}")
        
        time.sleep(0.5)
    
    # Simulate error conditions
    print("Simulating error conditions...")
    comm1_logger.warning("ACK timeout for seq: 5 after 3 retries")
    gps2_logger.error("GPS server unreachable, using fallback offset: +0.019 sec")
    control2_logger.warning("Leader state too old (1.2s), stopping vehicle")
    vehicle2_logger.error("Communication lost with leader vehicle")
    
    print("\nLogging demonstration completed!")
    print("Check the 'logs' directory for the generated log files.")

def demonstrate_individual_vehicle_logs():
    """Show how each vehicle gets its own dedicated log file."""
    
    print("\n=== Individual Vehicle Logs Demonstration ===")
    
    # Create loggers for multiple vehicles
    vehicles = []
    for vehicle_id in range(1, 4):  # Vehicles 1, 2, 3
        logger = get_individual_vehicle_logger(vehicle_id)
        vehicles.append((vehicle_id, logger))
        
        logger.info(f"Vehicle {vehicle_id} starting up")
        logger.info(f"Vehicle {vehicle_id} configuration loaded")
        logger.info(f"Vehicle {vehicle_id} ready for fleet operations")
    
    print(f"Created individual log files for {len(vehicles)} vehicles")
    print("Each vehicle has its own dedicated log file:")
    for vehicle_id, _ in vehicles:
        print(f"  - logs/vehicle_{vehicle_id}.log")

if __name__ == "__main__":
    try:
        demonstrate_logging()
        demonstrate_individual_vehicle_logs()
        
        print("\n=== Summary ===")
        print("The logging system creates the following files:")
        print("1. logs/vehicle_N.log - Individual vehicle general operations")
        print("2. logs/communication_vehicle_N.log - Each vehicle's communication events")
        print("3. logs/gps_vehicle_N.log - Each vehicle's GPS synchronization events")
        print("4. logs/control_vehicle_N.log - Each vehicle's control operations")
        print("5. logs/fleet_operations.log - General fleet operations")
        print("\nEach log file is automatically rotated when it reaches 5MB")
        print("and up to 3 backup files are kept for each log type.")
        print("\nNow each vehicle has completely separate log files for easier debugging!")
        
    except Exception as e:
        print(f"Error during logging demonstration: {e}")
        import traceback
        traceback.print_exc()
