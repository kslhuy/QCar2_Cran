# QCar Fleet Simulation Profiling Tools

This directory contains comprehensive profiling tools to help identify performance bottlenecks in the QCar fleet simulation.

## Files Overview

### Core Profiling Tools
- `performance_profiler.py` - Main profiling class with CPU, memory, and system monitoring
- `profiling_example.py` - Examples of how to use profiling decorators and context managers
- `profile_qcar_simulation.py` - Integration script for adding profiling to existing QCar code

### Monitoring and Running
- `monitor_system.py` - Real-time system monitoring script
- `run_with_profiling.py` - Script to run the simulation with different profiling modes

## Quick Start

### 1. Monitor Current System State
```bash
python monitor_system.py
```

### 2. Run Simulation with Profiling
```bash
# Comprehensive profiling (CPU, memory, system)
python run_with_profiling.py profile

# Detailed function-level profiling
python run_with_profiling.py cprofile

# Memory usage profiling
python run_with_profiling.py memory
```

### 3. Real-time Monitoring
```bash
# Monitor for 30 seconds
python monitor_system.py realtime 30

# Find QCar processes
python monitor_system.py qcar
```

## Profiling Reports

Reports are automatically generated in the `../logs/` directory:
- `performance_report_YYYYMMDD_HHMMSS.txt` - Comprehensive performance analysis
- `memory_report_YYYYMMDD_HHMMSS.txt` - Memory usage snapshots
- `cprofile_simulation_YYYYMMDD_HHMMSS.txt` - Detailed function profiling

## Integration Examples

### Add Profiling to Existing Code
```python
from performance_profiler import PerformanceProfiler

profiler = PerformanceProfiler()

# Profile a function
@profiler.profile_function
def my_expensive_function():
    # Your code here
    pass

# Profile a code block
with profiler.profile_context("my_operation"):
    # Your code here
    pass
```

### Monitor System Resources
```python
from performance_profiler import PerformanceProfiler

profiler = PerformanceProfiler()
profiler.start_system_monitoring(interval=1.0)  # Monitor every second

# Your simulation code here

profiler.stop_system_monitoring()
```

## Key Metrics to Monitor

1. **CPU Usage**: Look for functions consuming >10% of total CPU time
2. **Memory Growth**: Check for memory leaks in long-running simulations
3. **Thread/Process Count**: Ensure proper cleanup of background threads
4. **Communication Latency**: Monitor message send/receive times
5. **Queue Sizes**: Watch for growing queues indicating processing bottlenecks

## Optimization Targets

Based on profiling results, focus on:
- High CPU usage functions (optimize algorithms)
- Memory-intensive operations (use numpy arrays, avoid copies)
- Blocking operations (make async)
- Frequent allocations (reuse objects)
- Inefficient data structures (use appropriate types)

## Troubleshooting

- If profiling slows down the simulation too much, reduce monitoring intervals
- For memory profiling, ensure sufficient RAM (profiling uses additional memory)
- Check log files for any profiling-related errors
- Use `monitor_system.py` to verify system resources before/after runs

## Performance Improvements Already Implemented

The following optimizations have been applied to improve performance:

1. **Communication**: Reduced frequency to 20Hz, added batching, topology-aware routing
2. **Data Handling**: Numpy arrays for interpolation, GPS time caching, reduced copying
3. **Logging**: Async queue-based logging with background processing
4. **Timing**: Monotonic clocks for reliable timing measurements
5. **Topology**: Graph-based communication optimization

Use these profiling tools to identify any remaining bottlenecks and guide further optimizations.