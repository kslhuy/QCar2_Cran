#!/usr/bin/env python3
"""
Example: How to profile your QCar fleet simulation.
This script demonstrates various profiling techniques.
"""

import time
import sys
import os

# Add the current directory to Python path for imports
sys.path.append(os.path.dirname(__file__))

from performance_profiler import profiler, profile_simulation, log_performance_stats, print_system_status

def example_basic_profiling():
    """Example of basic function profiling."""
    print("=== Basic Profiling Example ===")

    @profiler.profile_function
    def simulate_workload():
        """Simulate some computational work."""
        total = 0
        for i in range(100000):
            total += i ** 2
        return total

    result = simulate_workload()
    print(f"Computation result: {result}")

def example_context_profiling():
    """Example of context manager profiling."""
    print("\n=== Context Manager Profiling Example ===")

    def heavy_computation():
        """Simulate heavy computation."""
        matrix = [[i*j for j in range(100)] for i in range(100)]
        return sum(sum(row) for row in matrix)

    with profiler.profile_context("heavy_computation"):
        result = heavy_computation()
        print(f"Matrix computation result: {result}")

def example_memory_profiling():
    """Example of memory profiling."""
    print("\n=== Memory Profiling Example ===")

    profiler.start_memory_tracking()
    profiler.snapshot_memory("before_allocation")

    # Allocate some memory
    big_list = [i for i in range(100000)]
    big_dict = {f"key_{i}": f"value_{i}" for i in range(10000)}

    profiler.snapshot_memory("after_allocation")

    # Clean up
    del big_list
    del big_dict

    profiler.snapshot_memory("after_cleanup")
    profiler.report_memory_usage()
    profiler.stop_memory_tracking()

def example_qcar_simulation_profiling():
    """Example of profiling a QCar simulation-like workload."""
    print("\n=== QCar Simulation Profiling Example ===")

    @profile_simulation
    def simulate_vehicle_fleet():
        """Simulate a simplified vehicle fleet."""
        vehicles = []

        # Simulate vehicle processes
        for vehicle_id in range(3):
            vehicle = {
                'id': vehicle_id,
                'position': [0.0, 0.0, 0.0],
                'velocity': 0.0,
                'state_history': []
            }
            vehicles.append(vehicle)

        # Simulate communication and control loops
        for _ in range(100):  # 100 simulation steps
            for vehicle in vehicles:
                # Simulate GPS update
                vehicle['position'][0] += vehicle['velocity'] * 0.05
                vehicle['velocity'] += 0.1  # Accelerate

                # Simulate state broadcasting (communication)
                state_msg = {
                    'id': vehicle['id'],
                    'pos': vehicle['position'].copy(),
                    'vel': vehicle['velocity'],
                    'timestamp': time.time()
                }
                vehicle['state_history'].append(state_msg)

                # Simulate control computation
                if vehicle['id'] > 0:  # Followers
                    # Simple following logic
                    leader_pos = vehicles[0]['position'][0]
                    follower_pos = vehicle['position'][0]
                    error = leader_pos - follower_pos - 5.0  # 5m spacing
                    vehicle['velocity'] += error * 0.1  # Simple P controller

        return vehicles

    vehicles = simulate_vehicle_fleet()

    # Print final state
    for vehicle in vehicles:
        final_pos = vehicle['position'][0]
        final_vel = vehicle['velocity']
        msg_count = len(vehicle['state_history'])
        print(f"Vehicle {vehicle['id']}: pos={final_pos:.2f}, vel={final_vel:.2f}, messages={msg_count}")

def monitor_qcar_system():
    """Monitor the actual QCar system."""
    print("\n=== QCar System Monitoring ===")

    print("Initial system status:")
    print_system_status()

    print("\nMonitoring for 5 seconds...")
    measurements = profiler.monitor_system_resources(duration=5, interval=1)

    print("\nResource usage over time:")
    for i, measurement in enumerate(measurements):
        print(f"  {i+1}s: CPU={measurement['cpu_percent']:.1f}%, "
              f"Mem={measurement['memory_rss']:.1f}MB, "
              f"Threads={measurement['num_threads']}")

    print("\nFinal system status:")
    print_system_status()

if __name__ == "__main__":
    print("QCar Fleet Simulation Profiling Examples")
    print("=" * 50)

    # Run examples
    example_basic_profiling()
    example_context_profiling()
    example_memory_profiling()
    example_qcar_simulation_profiling()
    monitor_qcar_system()

    print("\n" + "=" * 50)
    print("Profiling complete! Check the 'profiling_results' directory for detailed reports.")
    print("\nTo visualize profiles, install snakeviz:")
    print("  pip install snakeviz")
    print("  snakeviz profiling_results/profile.prof")