#!/usr/bin/env python3
"""
Script to run QCar simulation with profiling enabled.
This integrates the performance profiler with the main simulation.
"""

import sys
import os
import time
import cProfile
import pstats
import io
from performance_profiler import PerformanceProfiler

# Add the current directory to Python path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def profile_simulation():
    """Run the QCar simulation with profiling enabled."""
    print("Starting QCar simulation with profiling...")

    # Import after path setup
    from main import main as simulation_main

    # Create profiler
    profiler = PerformanceProfiler()

    # Start profiling
    profiler.start_profiling()

    try:
        # Run the simulation
        print("Running simulation...")
        simulation_main()

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")

    except Exception as e:
        print(f"Simulation error: {e}")
        raise

    finally:
        # Stop profiling and generate report
        profiler.stop_profiling()
        profiler.generate_report()

        print("\nProfiling complete. Check the reports directory for detailed analysis.")

def run_with_cprofile():
    """Run simulation with cProfile for detailed function-level profiling."""
    print("Starting QCar simulation with cProfile...")

    # Import after path setup
    from main import main as simulation_main

    # Create profiler
    pr = cProfile.Profile()
    pr.enable()

    try:
        # Run the simulation
        print("Running simulation...")
        simulation_main()

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")

    except Exception as e:
        print(f"Simulation error: {e}")
        raise

    finally:
        # Stop profiling and generate stats
        pr.disable()

        # Create stats and sort by cumulative time
        s = io.StringIO()
        ps = pstats.Stats(pr, stream=s).sort_stats('cumulative')
        ps.print_stats()

        # Save to file
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        profile_file = f"../logs/cprofile_simulation_{timestamp}.txt"

        with open(profile_file, 'w') as f:
            f.write(s.getvalue())

        print(f"\ncProfile stats saved to: {profile_file}")
        print("\nTop 20 functions by cumulative time:")
        ps.print_stats(20)

def memory_profile_simulation():
    """Run simulation with memory profiling."""
    print("Starting QCar simulation with memory profiling...")

    # Import after path setup
    from main import main as simulation_main

    # Create profiler
    profiler = PerformanceProfiler()

    # Start memory tracking
    profiler.start_memory_tracking()

    try:
        # Run the simulation
        print("Running simulation...")
        simulation_main()

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")

    except Exception as e:
        print(f"Simulation error: {e}")
        raise

    finally:
        # Generate memory report
        profiler.generate_memory_report()

        print("\nMemory profiling complete. Check the reports directory.")

def main():
    """Main function to choose profiling mode."""
    if len(sys.argv) < 2:
        print("Usage: python run_with_profiling.py <mode>")
        print("Modes:")
        print("  profile    - Run with comprehensive profiling")
        print("  cprofile   - Run with cProfile for function-level analysis")
        print("  memory     - Run with memory profiling")
        return

    mode = sys.argv[1].lower()

    if mode == "profile":
        profile_simulation()
    elif mode == "cprofile":
        run_with_cprofile()
    elif mode == "memory":
        memory_profile_simulation()
    else:
        print(f"Unknown mode: {mode}")
        print("Available modes: profile, cprofile, memory")

if __name__ == "__main__":
    main()