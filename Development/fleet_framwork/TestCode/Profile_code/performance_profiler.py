#!/usr/bin/env python3
"""
Performance profiling utilities for QCar fleet simulation.
Provides comprehensive profiling tools to identify bottlenecks.
"""

import cProfile
import pstats
import io
import time
import threading
import multiprocessing
import psutil
import os
import gc
from functools import wraps
from contextlib import contextmanager
import tracemalloc
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class PerformanceProfiler:
    """Comprehensive performance profiling for QCar simulation."""

    def __init__(self, output_dir="profiling_results"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.snapshots = []

    @contextmanager
    def profile_context(self, name="profile"):
        """Context manager for profiling code blocks."""
        profiler = cProfile.Profile()
        profiler.enable()
        try:
            yield profiler
        finally:
            profiler.disable()
            self._save_profile(profiler, f"{name}_{int(time.time())}")

    def profile_function(self, func):
        """Decorator to profile a function."""
        @wraps(func)
        def wrapper(*args, **kwargs):
            profiler = cProfile.Profile()
            profiler.enable()
            try:
                result = func(*args, **kwargs)
                return result
            finally:
                profiler.disable()
                self._save_profile(profiler, f"{func.__name__}_{int(time.time())}")
        return wrapper

    def _save_profile(self, profiler, name):
        """Save profiling results to file."""
        stats = pstats.Stats(profiler)
        stats.sort_stats('cumulative')

        # Save text report
        text_file = os.path.join(self.output_dir, f"{name}.txt")
        with open(text_file, 'w') as f:
            stats.stream = f
            stats.print_stats()

        # Save stats file for visualization
        stats_file = os.path.join(self.output_dir, f"{name}.prof")
        stats.dump_stats(stats_file)

        logger.info(f"Profile saved: {text_file}")

    def start_memory_tracking(self):
        """Start memory tracking."""
        tracemalloc.start()
        self.snapshots = []

    def snapshot_memory(self, label="snapshot"):
        """Take a memory snapshot."""
        if tracemalloc.is_tracing():
            snapshot = tracemalloc.take_snapshot()
            self.snapshots.append((label, snapshot))
            logger.info(f"Memory snapshot taken: {label}")

    def report_memory_usage(self):
        """Report memory usage statistics."""
        if not tracemalloc.is_tracing():
            logger.warning("Memory tracking not started")
            return

        if len(self.snapshots) >= 2:
            # Compare first and last snapshots
            first_label, first_snapshot = self.snapshots[0]
            last_label, last_snapshot = self.snapshots[-1]

            stats = last_snapshot.compare_to(first_snapshot, 'lineno')

            report_file = os.path.join(self.output_dir, f"memory_report_{int(time.time())}.txt")
            with open(report_file, 'w') as f:
                f.write(f"Memory usage report: {first_label} -> {last_label}\n")
                f.write("=" * 60 + "\n")

                for stat in stats[:20]:  # Top 20 memory consumers
                    f.write(f"{stat.traceback.format()[0]}: {stat.size_diff / 1024:.1f} KiB\n")

            logger.info(f"Memory report saved: {report_file}")

    def stop_memory_tracking(self):
        """Stop memory tracking."""
        if tracemalloc.is_tracing():
            tracemalloc.stop()

    @staticmethod
    def get_system_info():
        """Get system resource information."""
        process = psutil.Process()
        memory_info = process.memory_info()
        cpu_percent = process.cpu_percent()

        return {
            'cpu_percent': cpu_percent,
            'memory_rss': memory_info.rss / 1024 / 1024,  # MB
            'memory_vms': memory_info.vms / 1024 / 1024,  # MB
            'num_threads': process.num_threads(),
            'num_fds': getattr(process, 'num_fds', lambda: 'N/A')(),
        }

    @staticmethod
    def get_active_threads():
        """Get information about active threads."""
        threads = []
        for thread in threading.enumerate():
            threads.append({
                'name': thread.name,
                'ident': thread.ident,
                'is_alive': thread.is_alive(),
                'is_daemon': thread.daemon,
            })
        return threads

    @staticmethod
    def get_active_processes():
        """Get information about active child processes."""
        processes = []
        for proc in multiprocessing.active_children():
            try:
                processes.append({
                    'pid': proc.pid,
                    'name': proc.name,
                    'is_alive': proc.is_alive(),
                    'exitcode': proc.exitcode,
                })
            except Exception as e:
                processes.append({
                    'pid': proc.pid,
                    'name': proc.name,
                    'error': str(e),
                })
        return processes

    @staticmethod
    def monitor_system_resources(duration=10, interval=1):
        """Monitor system resources over time."""
        logger.info(f"Monitoring system resources for {duration}s...")

        measurements = []
        start_time = time.time()

        while time.time() - start_time < duration:
            info = PerformanceProfiler.get_system_info()
            info['timestamp'] = time.time()
            measurements.append(info)
            time.sleep(interval)

        return measurements

# Global profiler instance
profiler = PerformanceProfiler()

def profile_simulation(func):
    """Decorator specifically for profiling simulation functions."""
    @wraps(func)
    def wrapper(*args, **kwargs):
        logger.info(f"Starting profiling for {func.__name__}")

        # Start memory tracking
        profiler.start_memory_tracking()
        profiler.snapshot_memory("start")

        # Profile the function
        with profiler.profile_context(func.__name__):
            result = func(*args, **kwargs)

        # Memory analysis
        profiler.snapshot_memory("end")
        profiler.report_memory_usage()
        profiler.stop_memory_tracking()

        return result
    return wrapper

def log_performance_stats(func):
    """Decorator to log performance statistics."""
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        start_memory = psutil.Process().memory_info().rss

        try:
            result = func(*args, **kwargs)
            return result
        finally:
            end_time = time.time()
            end_memory = psutil.Process().memory_info().rss

            duration = end_time - start_time
            memory_delta = (end_memory - start_memory) / 1024 / 1024  # MB

            logger.info(f"{func.__name__} performance: "
                       f"duration={duration:.3f}s, "
                       f"memory_delta={memory_delta:.1f}MB")
    return wrapper

# Convenience functions for quick profiling
def start_profiling():
    """Start comprehensive profiling."""
    profiler.start_memory_tracking()
    profiler.snapshot_memory("start")
    logger.info("Profiling started")

def stop_profiling():
    """Stop profiling and generate reports."""
    profiler.snapshot_memory("end")
    profiler.report_memory_usage()
    profiler.stop_memory_tracking()
    logger.info("Profiling stopped")

def print_system_status():
    """Print current system status."""
    print("\n=== System Status ===")

    # System info
    sys_info = profiler.get_system_info()
    print(f"CPU Usage: {sys_info['cpu_percent']:.1f}%")
    print(f"Memory RSS: {sys_info['memory_rss']:.1f} MB")
    print(f"Memory VMS: {sys_info['memory_vms']:.1f} MB")
    print(f"Threads: {sys_info['num_threads']}")

    # Active threads
    threads = profiler.get_active_threads()
    print(f"\nActive Threads ({len(threads)}):")
    for thread in threads:
        status = "alive" if thread['is_alive'] else "dead"
        daemon = "daemon" if thread['is_daemon'] else "main"
        print(f"  - {thread['name']} (ID: {thread['ident']}, {status}, {daemon})")

    # Active processes
    processes = profiler.get_active_processes()
    print(f"\nActive Child Processes ({len(processes)}):")
    for proc in processes:
        if 'error' in proc:
            print(f"  - {proc['name']} (PID: {proc['pid']}, Error: {proc['error']})")
        else:
            status = "alive" if proc['is_alive'] else f"exit code {proc['exitcode']}"
            print(f"  - {proc['name']} (PID: {proc['pid']}, {status})")

    print("=" * 20)

if __name__ == "__main__":
    # Example usage
    print("Performance Profiling Utilities")
    print_system_status()