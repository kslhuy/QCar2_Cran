#!/usr/bin/env python3
"""
Quick monitoring script for QCar simulation.
Run this to see active threads, processes, and system resources.
"""

import time
import psutil
import threading
import multiprocessing
import os
import sys

def get_process_info():
    """Get detailed information about the current process and children."""
    current_process = psutil.Process()
    children = current_process.children(recursive=True)

    print(f"\n=== Process Information ===")
    print(f"Main Process: {current_process.name()} (PID: {current_process.pid})")
    print(f"  CPU: {current_process.cpu_percent():.1f}%")
    print(f"  Memory: {current_process.memory_info().rss / 1024 / 1024:.1f} MB")
    print(f"  Threads: {current_process.num_threads()}")
    print(f"  Status: {current_process.status()}")

    if children:
        print(f"\nChild Processes ({len(children)}):")
        for child in children:
            try:
                print(f"  - {child.name()} (PID: {child.pid})")
                print(f"    CPU: {child.cpu_percent():.1f}%, Memory: {child.memory_info().rss / 1024 / 1024:.1f} MB")
                print(f"    Threads: {child.num_threads()}, Status: {child.status()}")
            except psutil.NoSuchProcess:
                print(f"  - Process {child.pid} no longer exists")
    else:
        print("No child processes found")

def get_thread_info():
    """Get information about active threads."""
    threads = threading.enumerate()

    print(f"\n=== Thread Information ===")
    print(f"Active Threads: {len(threads)}")

    for thread in threads:
        daemon_status = "daemon" if thread.daemon else "main"
        alive_status = "alive" if thread.is_alive() else "dead"
        print(f"  - {thread.name} (ID: {thread.ident}, {alive_status}, {daemon_status})")

def get_multiprocessing_info():
    """Get information about multiprocessing processes."""
    active_processes = multiprocessing.active_children()

    print(f"\n=== Multiprocessing Information ===")
    print(f"Active Child Processes: {len(active_processes)}")

    for proc in active_processes:
        alive_status = "alive" if proc.is_alive() else f"exit code {proc.exitcode}"
        print(f"  - {proc.name} (PID: {proc.pid}, {alive_status})")

def get_system_resources():
    """Get system-wide resource usage."""
    print(f"\n=== System Resources ===")

    # CPU
    cpu_percent = psutil.cpu_percent(interval=0.1)
    cpu_count = psutil.cpu_count()
    cpu_logical = psutil.cpu_count(logical=True)
    print(f"CPU: {cpu_percent:.1f}% ({cpu_count} physical, {cpu_logical} logical cores)")

    # Memory
    memory = psutil.virtual_memory()
    print(f"Memory: {memory.percent:.1f}% used ({memory.used / 1024 / 1024 / 1024:.1f}GB / {memory.total / 1024 / 1024 / 1024:.1f}GB)")

    # Disk
    disk = psutil.disk_usage('/')
    print(f"Disk: {disk.percent:.1f}% used ({disk.used / 1024 / 1024 / 1024:.1f}GB / {disk.total / 1024 / 1024 / 1024:.1f}GB)")

def monitor_realtime(duration=30, interval=2):
    """Monitor system in real-time for a specified duration."""
    print(f"\n=== Real-time Monitoring ({duration}s) ===")
    print("Time  | CPU% | Mem% | Threads | Processes")
    print("-" * 45)

    start_time = time.time()
    while time.time() - start_time < duration:
        current_process = psutil.Process()
        cpu = psutil.cpu_percent(interval=0.1)
        mem = psutil.virtual_memory().percent
        threads = current_process.num_threads()
        processes = len(multiprocessing.active_children())

        elapsed = int(time.time() - start_time)
        print("4d")

        time.sleep(interval)

    print("-" * 45)

def find_qcar_processes():
    """Find any running QCar-related processes."""
    print(f"\n=== QCar-related Processes ===")

    qcar_processes = []
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if any('qcar' in str(attr).lower() for attr in [proc.info['name'], proc.info['cmdline']]):
                qcar_processes.append(proc.info)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    if qcar_processes:
        for proc in qcar_processes:
            print(f"  - {proc['name']} (PID: {proc['pid']})")
            if proc['cmdline']:
                print(f"    Command: {' '.join(proc['cmdline'])}")
    else:
        print("No QCar-related processes found")

def main():
    """Main monitoring function."""
    print("QCar Simulation System Monitor")
    print("=" * 40)

    # Get command line arguments
    if len(sys.argv) > 1:
        if sys.argv[1] == "realtime":
            duration = int(sys.argv[2]) if len(sys.argv) > 2 else 30
            monitor_realtime(duration)
        elif sys.argv[1] == "qcar":
            find_qcar_processes()
        else:
            print("Usage: python monitor_system.py [realtime [duration] | qcar]")
            return
    else:
        # Default: show all information once
        get_process_info()
        get_thread_info()
        get_multiprocessing_info()
        get_system_resources()
        find_qcar_processes()

        print(f"\n=== Quick Commands ===")
        print("python monitor_system.py realtime 30    # Monitor for 30 seconds")
        print("python monitor_system.py qcar          # Find QCar processes")

if __name__ == "__main__":
    main()