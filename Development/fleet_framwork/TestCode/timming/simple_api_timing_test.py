#!/usr/bin/env python3
"""
Simple QLabs API Timing Test

Quick test to measure get_world_transform() call times to identify
if this is the source of the 8ms bottleneck in the observer loop.
"""

import time
import sys
import os
import threading
import queue
from concurrent.futures import ThreadPoolExecutor
import statistics

from multiprocessing import Process, Queue
import psutil

# For thread priority control
if os.name == 'nt':  # Windows
    import ctypes
    from ctypes import wintypes

from qvl.qcar2 import QLabsQCar2
from qvl.qlabs import QuanserInteractiveLabs


# Add the project path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# =============================================================================
# CONFIGURATION - Change these values to test different API call rates
# =============================================================================
DEFAULT_TARGET_HZ = 20  # Default rate in Hz (calls per second)
TEST_RATES = [DEFAULT_TARGET_HZ]  # Different rates to test
TEST_DURATION = 30  # Duration for each test in seconds
# =============================================================================

def set_process_priority(priority_level="normal"):
    try:
        p = psutil.Process(os.getpid())
        if os.name == 'nt':
            if priority_level == "high":
                p.nice(psutil.HIGH_PRIORITY_CLASS)
            elif priority_level == "highest":
                p.nice(psutil.REALTIME_PRIORITY_CLASS)
            else:
                p.nice(psutil.NORMAL_PRIORITY_CLASS)
        else:  # Unix/Linux
            if priority_level == "high":
                os.nice(-5)
            elif priority_level == "highest":
                os.nice(-10)
        return True
    except Exception as e:
        print(f"Could not set process priority: {e}")
        return False

def vehicle_observer_process(vehicle_id, spawn_location, spawn_rotation, scale, vehicle_name, results_queue, test_duration=TEST_DURATION, target_hz=DEFAULT_TARGET_HZ):
    """Simulate a vehicle observer process like in the fleet framework."""
    
    # Create QLabs connection inside this process
    try:
        qlabs = QuanserInteractiveLabs()
        qlabs.open("localhost")
        
        # Create vehicle object and connect to the spawned vehicle by ID
        vehicle = QLabsQCar2(qlabs)
        vehicle.spawn_id(actorNumber=vehicle_id, location=spawn_location, rotation=spawn_rotation, scale=scale)

        
    except Exception as e:
        print(f"Failed to connect to QLabs in {vehicle_name} process: {e}")
        results_queue.put({'error': f"Connection failed: {e}"})
        return

    # Set process priority
    if "Leader" in vehicle_name:
        success = set_process_priority("high")
        priority_msg = "HIGH" if success else "NORMAL (failed to set high)"
    else:
        success = set_process_priority("normal")
        priority_msg = "NORMAL"
    
    print(f"Starting {vehicle_name} observer thread at {target_hz}Hz (Priority: {priority_msg})")
    
    thread_times = []
    successful_calls = 0
    failed_calls = 0
    
    start_time = time.time()
    call_count = 0
    
    while time.time() - start_time < test_duration:
        call_start = time.perf_counter()
        
        try:
            # Get world transform (like GPS data collection)
            _, pos, rot, _ = vehicle.get_world_transform()
            
            # Simulate additional observer processing
            time.sleep(0.001)  # 1ms of processing time
            
            call_end = time.perf_counter()
            call_time_ms = (call_end - call_start) * 1000
            thread_times.append(call_time_ms)
            successful_calls += 1
            
            # Print status every 50 calls with detailed info
            if call_count % 5 == 0:
                print(f"{vehicle_name}: Call {call_count}, Time: {call_time_ms:.3f}ms")
                print(f" ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}) | ({rot[0]:6.3f}, {rot[1]:6.3f}, {rot[2]:6.3f}) ")

                
        except Exception as e:
            failed_calls += 1
            print(f"{vehicle_name}: Call {call_count} failed: {e}")
        
        call_count += 1
        
        # Target frequency (configurable Hz)
        target_interval = 1.0 / target_hz  # Convert Hz to interval in seconds
        elapsed = time.time() - (start_time + call_count * target_interval)
        if elapsed < target_interval:
            time.sleep(target_interval - elapsed)
    
    # Store results
    results = {
        'vehicle_name': vehicle_name,
        'times': thread_times,
        'successful_calls': successful_calls,
        'failed_calls': failed_calls,
        'total_calls': call_count,
        'test_duration': time.time() - start_time
    }
    
    results_queue.put(results)
    print(f"{vehicle_name} observer process completed")
    
    # Log GPS performance data for comparison
    log_gps_performance(
        method_name='multiprocessing',
        vehicle_name=vehicle_name,
        gps_times=thread_times,
        successful_calls=successful_calls,
        failed_calls=failed_calls,
        total_calls=call_count,
        test_duration=time.time() - start_time,
        target_hz=target_hz
    )
    
    # Cleanup QLabs connection
    try:
        qlabs.close()
    except:
        pass

def test_multi_vehicle_processing(leader_id, follower_id, target_hz=DEFAULT_TARGET_HZ):
    print(f"Testing 2 vehicles in parallel processes...")
    results_queue = Queue()
    # Spawn vehicles with detailed verification
    leader_spawn_location = [-1.205, -0.83, 0.000]
    leader_spawn_rotation = [0, 0, -44.7]
    follower_spawn_location = [-1.8, -0.83, 0.000]
    follower_spawn_rotation = [0, 0, -44.7]
    leader_scale = [0.1, 0.1, 0.1]
    follower_scale = [0.1, 0.1, 0.1]

    
    leader_proc = Process(target=vehicle_observer_process, args=(leader_id, leader_spawn_location, leader_spawn_rotation, leader_scale, "QCar-Leader", results_queue, TEST_DURATION, target_hz))
    follower_proc = Process(target=vehicle_observer_process, args=(follower_id, follower_spawn_location, follower_spawn_rotation, follower_scale, "QCar-Follower", results_queue, TEST_DURATION, target_hz))

    leader_proc.start()
    follower_proc.start()

    leader_proc.join()
    follower_proc.join()

    results = []
    while not results_queue.empty():
        result = results_queue.get()
        if 'error' not in result:  # Only add successful results
            results.append(result)
        else:
            print(f"Process error: {result['error']}")

    if results:
        analyze_multi_vehicle_results(results)
        
        # # After multiprocessing test, run comparison if we have threading data too
        # print(f"\n{'='*60}")
        # print("🔍 RUNNING AUTOMATED COMPARISON FROM LOGGED DATA")
        # print(f"{'='*60}")
        # compare_threading_vs_multiprocessing_from_logs()
        
    else:
        print("No successful results from multi-vehicle test")
    

def analyze_multi_vehicle_results(results):
    """Analyze the multi-vehicle test results."""
    
    print(f"\n=== Multi-Vehicle Performance Analysis ===")
    
    total_api_calls = 0
    all_times = []
    
    for result in results:
        vehicle_name = result['vehicle_name']
        times = result['times']
        successful = result['successful_calls']
        failed = result['failed_calls']
        total = result['total_calls']
        duration = result['test_duration']
        
        if times:
            avg_time = statistics.mean(times)
            min_time = min(times)
            max_time = max(times)
            std_time = statistics.stdev(times) if len(times) > 1 else 0
            
            # Performance categorization
            fast_calls = sum(1 for t in times if t < 5.0)
            moderate_calls = sum(1 for t in times if 5.0 <= t < 8.0)
            slow_calls = sum(1 for t in times if t >= 8.0)
            
            actual_rate = successful / duration
            
            print(f"\n{vehicle_name} Results:")
            print(f"  Duration: {duration:.1f}s")
            print(f"  Total calls: {total}")
            print(f"  Successful: {successful}")
            print(f"  Failed: {failed}")
            print(f"  Success rate: {successful/total*100:.1f}%")
            print(f"  Actual rate: {actual_rate:.1f}Hz")
            
            print(f"  API Call Times:")
            print(f"    Average: {avg_time:.3f}ms")
            print(f"    Min: {min_time:.3f}ms")
            print(f"    Max: {max_time:.3f}ms")
            print(f"    Std Dev: {std_time:.3f}ms")
            
            print(f"  Performance Distribution:")
            print(f"    Fast (<5ms): {fast_calls} ({fast_calls/len(times)*100:.1f}%)")
            print(f"    Moderate (5-8ms): {moderate_calls} ({moderate_calls/len(times)*100:.1f}%)")
            print(f"    Slow (≥8ms): {slow_calls} ({slow_calls/len(times)*100:.1f}%)")
            
            # Performance assessment
            if avg_time > 8.0:
                print(f"    🔴 BOTTLENECK: {vehicle_name} API calls are very slow")
            elif avg_time > 5.0:
                print(f"    🟡 CONCERN: {vehicle_name} API calls are moderately slow")
            else:
                print(f"    ✅ GOOD: {vehicle_name} API performance acceptable")
            
            total_api_calls += successful
            all_times.extend(times)
    
    # Overall analysis
    if all_times:
        overall_avg = statistics.mean(all_times)
        overall_min = min(all_times)
        overall_max = max(all_times)
        overall_std = statistics.stdev(all_times) if len(all_times) > 1 else 0
        
        print(f"\n=== Overall Fleet Performance ===")
        print(f"Total API calls across all vehicles: {total_api_calls}")
        print(f"Combined average API time: {overall_avg:.3f}ms")
        print(f"Combined min API time: {overall_min:.3f}ms")
        print(f"Combined max API time: {overall_max:.3f}ms")
        print(f"Combined std deviation: {overall_std:.3f}ms")
        
        # Fleet performance assessment
        slow_calls_total = sum(1 for t in all_times if t >= 8.0)
        slow_percentage = slow_calls_total / len(all_times) * 100
        
        print(f"\nFleet Assessment:")
        print(f"Slow calls (≥8ms): {slow_calls_total}/{len(all_times)} ({slow_percentage:.1f}%)")
        
        if overall_avg > 8.0:
            print("🔴 CRITICAL: Fleet GPS bottleneck confirmed")
            print("   Recommendation: Reduce GPS rate to 25-50Hz")
        elif overall_avg > 5.0:
            print("🟡 WARNING: Fleet GPS performance concerning")
            print("   Recommendation: Reduce GPS rate to 50-75Hz")
        elif slow_percentage > 20:
            print("🟡 WARNING: Too many slow GPS calls")
            print("   Recommendation: Monitor GPS performance and consider rate reduction")
        else:
            print("✅ ACCEPTABLE: Fleet GPS performance within limits")
        
        # Threading impact analysis
        print(f"\nThreading Impact:")
        target_loop_time_ms = 1000 / DEFAULT_TARGET_HZ
        print(f"Target loop time ({DEFAULT_TARGET_HZ}Hz): {target_loop_time_ms:.1f}ms")
        print(f"Average GPS time: {overall_avg:.3f}ms ({overall_avg/target_loop_time_ms*100:.1f}% of loop)")
        print(f"Time remaining for other operations: {target_loop_time_ms-overall_avg:.3f}ms")
        
        if overall_avg > 8.0:
            print("❌ CRITICAL: GPS leaves insufficient time for other operations")
        elif overall_avg > 6.0:
            print("⚠️  WARNING: GPS uses significant portion of loop time")
        else:
            print("✅ GOOD: GPS leaves sufficient time for other operations")

def log_gps_performance(method_name, vehicle_name, gps_times, successful_calls, failed_calls, total_calls, test_duration, target_hz):
    """Log GPS performance data to a single file for later comparison."""
    import json
    import time
    from datetime import datetime
    
    # Use a single log file instead of creating new ones each time
    log_filename = "gps_performance_log.json"
    lock_filename = "gps_performance_log.lock"
    
    # Calculate statistics
    avg_time = statistics.mean(gps_times) if gps_times else 0
    min_time = min(gps_times) if gps_times else 0
    max_time = max(gps_times) if gps_times else 0
    std_time = statistics.stdev(gps_times) if len(gps_times) > 1 else 0
    
    performance_data = {
        'timestamp': datetime.now().strftime("%Y%m%d_%H%M%S"),
        'method': method_name,  # 'threading' or 'multiprocessing'
        'vehicle_name': vehicle_name,
        'target_hz': target_hz,
        'test_duration': test_duration,
        'total_calls': total_calls,
        'successful_calls': successful_calls,
        'failed_calls': failed_calls,
        'success_rate': (successful_calls / total_calls * 100) if total_calls > 0 else 0,
        'gps_statistics': {
            'average_ms': avg_time,
            'min_ms': min_time,
            'max_ms': max_time,
            'std_dev_ms': std_time,
            'fast_calls': sum(1 for t in gps_times if t < 5.0),
            'moderate_calls': sum(1 for t in gps_times if 5.0 <= t < 8.0),
            'slow_calls': sum(1 for t in gps_times if t >= 8.0)
        },
        'raw_times': gps_times  # Keep raw data for detailed analysis
    }
    
    # Simple file locking for cross-platform compatibility
    max_retries = 10
    retry_delay = 0.1
    
    for attempt in range(max_retries):
        try:
            # Check if lock file exists (simple lock mechanism)
            if os.path.exists(lock_filename):
                time.sleep(retry_delay)
                continue
            
            # Create lock file
            with open(lock_filename, 'w') as lock_f:
                lock_f.write(f"{os.getpid()}")
            
            # Read existing data or create new structure
            try:
                with open(log_filename, 'r') as f:
                    log_data = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                log_data = {'test_runs': []}
            
            # Add new data
            log_data['test_runs'].append(performance_data)
            
            # Write back to file (overwrite)
            with open(log_filename, 'w') as f:
                json.dump(log_data, f, indent=2)
            
            # Remove lock file
            try:
                os.remove(lock_filename)
            except:
                pass
            
            print(f"✅ Performance data logged to: {log_filename}")
            return log_filename
            
        except Exception as e:
            # Remove lock file in case of error
            try:
                os.remove(lock_filename)
            except:
                pass
            
            if attempt < max_retries - 1:
                print(f"⚠️ Logging attempt {attempt + 1} failed, retrying...")
                time.sleep(retry_delay)
                retry_delay *= 1.5  # Exponential backoff
            else:
                print(f"❌ Failed to log performance data after {max_retries} attempts: {e}")
                return None
    
    return None

def compare_threading_vs_multiprocessing_from_logs(log_file=None):
    """Compare threading vs multiprocessing performance from the single log file."""
    import json
    from datetime import datetime
    
    print(f"\n{'='*60}")
    print(f"GPS PERFORMANCE COMPARISON FROM LOG FILE")
    print(f"{'='*60}")
    
    # Use single log file
    if not log_file:
        log_file = "gps_performance_log.json"
    
    # Load log data
    try:
        with open(log_file, 'r') as f:
            log_data = json.load(f)
        
        print(f"📁 Analyzing: {log_file}")
        
    except FileNotFoundError:
        print("❌ No log file found. Run threading and multiprocessing tests first.")
        return None
    except Exception as e:
        print(f"❌ Error reading log file: {e}")
        return None
    
    # Organize data by method
    threading_data = []
    multiprocessing_data = []
    
    for run in log_data.get('test_runs', []):
        if run['method'] == 'threading':
            threading_data.append(run)
        elif run['method'] == 'multiprocessing':
            multiprocessing_data.append(run)
    
    if not threading_data:
        print("❌ No threading data found in log file")
        return None
    if not multiprocessing_data:
        print("❌ No multiprocessing data found in log file")
        return None
    
    # Analyze and compare
    def analyze_method_data(method_data, method_name):
        total_calls = sum(run['successful_calls'] for run in method_data)
        all_times = []
        for run in method_data:
            all_times.extend(run['raw_times'])
        
        if not all_times:
            return None
            
        stats = {
            'method': method_name,
            'total_runs': len(method_data),
            'total_calls': total_calls,
            'avg_time': statistics.mean(all_times),
            'min_time': min(all_times),
            'max_time': max(all_times),
            'std_time': statistics.stdev(all_times) if len(all_times) > 1 else 0,
            'fast_calls': sum(1 for t in all_times if t < 5.0),
            'moderate_calls': sum(1 for t in all_times if 5.0 <= t < 8.0),
            'slow_calls': sum(1 for t in all_times if t >= 8.0),
            'success_rate': sum(run['success_rate'] for run in method_data) / len(method_data)
        }
        return stats
    
    threading_stats = analyze_method_data(threading_data, 'Threading')
    multiprocessing_stats = analyze_method_data(multiprocessing_data, 'Multiprocessing')
    
    # Print comparison
    print(f"\n🧵 THREADING PERFORMANCE:")
    if threading_stats:
        print(f"   Test Runs: {threading_stats['total_runs']}")
        print(f"   Total GPS Calls: {threading_stats['total_calls']}")
        print(f"   Average Time: {threading_stats['avg_time']:.3f}ms")
        print(f"   Min/Max: {threading_stats['min_time']:.3f}ms / {threading_stats['max_time']:.3f}ms")
        print(f"   Std Deviation: {threading_stats['std_time']:.3f}ms")
        print(f"   Success Rate: {threading_stats['success_rate']:.1f}%")
        print(f"   Call Distribution: Fast={threading_stats['fast_calls']}, Moderate={threading_stats['moderate_calls']}, Slow={threading_stats['slow_calls']}")
    
    print(f"\n🔄 MULTIPROCESSING PERFORMANCE:")
    if multiprocessing_stats:
        print(f"   Test Runs: {multiprocessing_stats['total_runs']}")
        print(f"   Total GPS Calls: {multiprocessing_stats['total_calls']}")
        print(f"   Average Time: {multiprocessing_stats['avg_time']:.3f}ms")
        print(f"   Min/Max: {multiprocessing_stats['min_time']:.3f}ms / {multiprocessing_stats['max_time']:.3f}ms")
        print(f"   Std Deviation: {multiprocessing_stats['std_time']:.3f}ms")
        print(f"   Success Rate: {multiprocessing_stats['success_rate']:.1f}%")
        print(f"   Call Distribution: Fast={multiprocessing_stats['fast_calls']}, Moderate={multiprocessing_stats['moderate_calls']}, Slow={multiprocessing_stats['slow_calls']}")
    
    # Direct comparison
    if threading_stats and multiprocessing_stats:
        print(f"\n📊 DIRECT COMPARISON:")
        thread_avg = threading_stats['avg_time']
        process_avg = multiprocessing_stats['avg_time']
        
        diff = abs(thread_avg - process_avg)
        better_method = "Threading" if thread_avg < process_avg else "Multiprocessing"
        improvement_pct = (diff / max(thread_avg, process_avg)) * 100
        
        print(f"   Threading Average: {thread_avg:.3f}ms")
        print(f"   Multiprocessing Average: {process_avg:.3f}ms")
        print(f"   Difference: {diff:.3f}ms")
        print(f"   Better Method: {better_method}")
        print(f"   Performance Improvement: {improvement_pct:.1f}%")
        
        # Stability comparison
        thread_stability = threading_stats['std_time']
        process_stability = multiprocessing_stats['std_time']
        more_stable = "Threading" if thread_stability < process_stability else "Multiprocessing"
        
        print(f"\n📈 STABILITY ANALYSIS:")
        print(f"   Threading Std Dev: {thread_stability:.3f}ms")
        print(f"   Multiprocessing Std Dev: {process_stability:.3f}ms")
        print(f"   More Stable: {more_stable}")
        
        # Recommendations
        print(f"\n💡 RECOMMENDATIONS:")
        if improvement_pct > 15:
            print(f"   🎯 SIGNIFICANT: Use {better_method} - {improvement_pct:.1f}% faster")
        elif improvement_pct > 5:
            print(f"   📊 MODERATE: Prefer {better_method} - {improvement_pct:.1f}% faster")
        else:
            print(f"   ⚖️ SIMILAR: Both methods perform comparably")
        
        if max(thread_avg, process_avg) > 8.0:
            print(f"   ⚠️ CRITICAL: Both methods exceed 8ms - reduce GPS rate")
        elif max(thread_avg, process_avg) > 5.0:
            print(f"   🟡 WARNING: GPS calls approaching bottleneck threshold")
        else:
            print(f"   ✅ GOOD: GPS performance within acceptable limits")
    
    print(f"\n{'='*60}")
    
    # Save comparison report to single file (overwrite)
    report_file = "gps_comparison_report.json"
    comparison_report = {
        'timestamp': datetime.now().strftime("%Y%m%d_%H%M%S"),
        'log_file_analyzed': log_file,
        'threading_stats': threading_stats,
        'multiprocessing_stats': multiprocessing_stats,
        'comparison_summary': {
            'better_method': better_method if threading_stats and multiprocessing_stats else None,
            'performance_improvement_pct': improvement_pct if threading_stats and multiprocessing_stats else None,
            'more_stable_method': more_stable if threading_stats and multiprocessing_stats else None
        }
    }
    
    try:
        with open(report_file, 'w') as f:
            json.dump(comparison_report, f, indent=2)
        print(f"📋 Comparison report saved to: {report_file}")
    except Exception as e:
        print(f"❌ Failed to save comparison report: {e}")
    
    return comparison_report

def simple_timing_test():
    """Simple timing test for get_world_transform()."""
    

    
    print("=== Simple QLabs API Timing Test ===")
    print("Testing get_world_transform() call times...\n")
    
    # Initialize QLabs
    try:
        qlabs = QuanserInteractiveLabs()
        print("Connecting to QLabs...")
        try:
            qlabs.open("localhost")
            print("Connected to QLabs")
        except:
            print("Unable to connect to QLabs")
            quit()

        qlabs.destroy_all_spawned_actors()


        
        # Wait a moment for spawn to complete
        time.sleep(0.5)
        print("\nVerifying spawn positions:")

        
        # Wait a bit more to ensure vehicles are fully loaded
        print("\nWaiting for vehicles to fully initialize...")
        time.sleep(1.0)
        
    except Exception as e:
        print(f"Failed to connect to QLabs: {e}")
        print("Make sure QLabs is running")
        return
    
    # Cleanup
    try:
        qlabs.close()
    except:
        pass
    
    # # Test the API call timing
    # print("\nMeasuring get_world_transform() times:")
    # print("Call#  | Time (ms) | Position (x, y, z)      | Rotation (x, y, z)      | Status")
    # print("-------|-----------|-------------------------|-------------------------|-------")
    
    # # Single vehicle test first
    # print("=== Single Vehicle Test ===")
    # times = []
    # for i in range(15):
    #     start_time = time.perf_counter()
        
    #     try:
    #         _, pos, rot, _ = follower.get_world_transform()
    #         end_time = time.perf_counter()
            
    #         call_time_ms = (end_time - start_time) * 1000
    #         times.append(call_time_ms)
            
    #         status = "SLOW" if call_time_ms > 8.0 else "OK" if call_time_ms > 5.0 else "FAST"
    #         print(f"{i+1:5d}  | {call_time_ms:7.3f}   | ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}) | ({rot[0]:6.3f}, {rot[1]:6.3f}, {rot[2]:6.3f}) | {status}")
            
    #     except Exception as e:
    #         print(f"{i+1:5d}  | ERROR     | N/A                     | N/A                     | FAILED: {e}")
        
    #     # Small delay to avoid overwhelming the API
    #     time.sleep(1/DEFAULT_TARGET_HZ)
    #  # Calculate statistics
    # if times:
    #     avg_time = sum(times) / len(times)
    #     min_time = min(times)
    #     max_time = max(times)
    #     slow_calls = sum(1 for t in times if t > 8.0)
        
    #     print(f"\n=== Results ===")
    #     print(f"Total calls: {len(times)}")
    #     print(f"Average time: {avg_time:.3f}ms")
    #     print(f"Min time: {min_time:.3f}ms")
    #     print(f"Max time: {max_time:.3f}ms")
    #     print(f"Calls > 8ms: {slow_calls}/{len(times)} ({slow_calls/len(times)*100:.1f}%)")
        
    #     print(f"\n=== Analysis ===")
    #     if avg_time > 8.0:
    #         print("🔴 PROBLEM CONFIRMED: API calls are consistently slow (>8ms)")
    #         print("   This explains the observer loop performance issues")
    #         print("   Recommendation: Reduce GPS update rate significantly")
    #     elif avg_time > 5.0:
    #         print("🟡 MODERATE ISSUE: API calls are somewhat slow (>5ms)")
    #         print("   This contributes to observer loop slowdown")
    #         print("   Recommendation: Reduce GPS update rate moderately")
    #     else:
    #         print("🟢 API PERFORMANCE OK: Bottleneck is elsewhere")
    #         print("   Look for other causes in the observer loop")
        
    #     print(f"\n=== Observer Loop Impact ===")
    #     target_loop_time = 1000 / DEFAULT_TARGET_HZ
    #     print(f"Target observer loop time: {target_loop_time:.1f}ms ({DEFAULT_TARGET_HZ}Hz)")
    #     print(f"GPS API time: {avg_time:.3f}ms")
    #     print(f"Time remaining for other operations: {target_loop_time - avg_time:.3f}ms")
        
    #     if avg_time > 8.0:
    #         print("❌ CRITICAL: Almost no time left for other observer operations")
    #     elif avg_time > 5.0:
    #         print("⚠️  WARNING: Limited time for other observer operations")
    #     else:
    #         print("✅ GOOD: Sufficient time for other observer operations")
    
    # # Cleanup
    # Test multi threading performance
    print("\n=== Multi-Vehicle Threading Test ===")
    # Store vehicle IDs for process communication
    leader_id = 6
    follower_id = 7
    for rate in TEST_RATES:
        print(f"\n--- Testing at {rate}Hz ---")
        test_multi_vehicle_threading(leader_id, follower_id, target_hz=rate)
    

    
    # Test multi-process performance
    # Store vehicle IDs for process communication
    leader_id = 3
    follower_id = 4
    print("\n=== Multi-Vehicle Performance Test ===")
    for rate in TEST_RATES:
        print(f"\n--- Testing at {rate}Hz ---")
        test_multi_vehicle_processing(leader_id, follower_id, target_hz=rate)
    



def test_multi_vehicle_threading(leader_id, follower_id, target_hz=DEFAULT_TARGET_HZ):
    """Test multi-vehicle performance using threading (for comparison with multiprocessing)."""
    print(f"Testing 2 vehicles in parallel threads (for comparison)...")
    
    results_queue = queue.Queue()

    def thread_vehicle_observer(id, spawn_location, spawn_rotation, scale, vehicle_name):
        qlabs = QuanserInteractiveLabs()
        qlabs.open("localhost")


        vehicle = QLabsQCar2(qlabs)
        qcar_spawn_success = vehicle.spawn_id(actorNumber=id, location=spawn_location, rotation=spawn_rotation, scale=scale)

        """Thread-based vehicle observer for comparison."""
        thread_times = []
        successful_calls = 0
        failed_calls = 0
        
        start_time = time.time()
        call_count = 0
        
        while time.time() - start_time < TEST_DURATION:  # Shorter test for threads
            call_start = time.perf_counter()
            
            try:
                _, pos, rot, _ = vehicle.get_world_transform()
                time.sleep(0.001)  # 1ms of processing time
                
                call_end = time.perf_counter()
                call_time_ms = (call_end - call_start) * 1000
                thread_times.append(call_time_ms)
                successful_calls += 1
                
                if call_count % 10 == 0:
                    print(f"{vehicle_name} (Thread): Call {call_count}, Time: {call_time_ms:.3f}ms")
                    print(f" ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}) | ({rot[0]:6.3f}, {rot[1]:6.3f}, {rot[2]:6.3f}) ")

                    # # print(f"{vehicle_name}: Call {call_count}, Time: {call_time_ms:.3f}ms")
                    # print(f"  Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
                    # print(f"  Rotation: ({rot[0]:.3f}, {rot[1]:.3f}, {rot[2]:.3f})")
                    
            except Exception as e:
                failed_calls += 1
                print(f"{vehicle_name} (Thread): Call {call_count} failed: {e}")
            
            call_count += 1
            
            # Target frequency
            target_interval = 1.0 / target_hz
            elapsed = time.time() - (start_time + call_count * target_interval)
            if elapsed < target_interval:
                time.sleep(target_interval - elapsed)
        
        results = {
            'vehicle_name': vehicle_name + " (Thread)",
            'times': thread_times,
            'successful_calls': successful_calls,
            'failed_calls': failed_calls,
            'total_calls': call_count,
            'test_duration': time.time() - start_time
        }
        results_queue.put(results)
        
        # Log GPS performance data for comparison
        log_gps_performance(
            method_name='threading',
            vehicle_name=vehicle_name,
            gps_times=thread_times,
            successful_calls=successful_calls,
            failed_calls=failed_calls,
            total_calls=call_count,
            test_duration=time.time() - start_time,
            target_hz=target_hz
        )
    

    leader_spawn_location = [-3, -0.83, 0.000]
    leader_spawn_rotation = [0, 0, -44.7]
    follower_spawn_location = [-5, -0.83, 0.000]
    follower_spawn_rotation = [0, 0, -44.7]
    leader_scale = [0.1, 0.1, 0.1]
    follower_scale = [0.1, 0.1, 0.1]


    # Start threads
    leader_thread = threading.Thread(target=thread_vehicle_observer, args=(leader_id, leader_spawn_location, leader_spawn_rotation, leader_scale, "QCar-Leader"))
    time.sleep(0.5)  # Ensure leader spawns first
    follower_thread = threading.Thread(target=thread_vehicle_observer, args=(follower_id, follower_spawn_location, follower_spawn_rotation, follower_scale, "QCar-Follower"))

    leader_thread.start()
    follower_thread.start()
    
    leader_thread.join()
    follower_thread.join()


    # qcar.destroy()
    # follower.destroy()
    
    # Collect results
    results = []
    while not results_queue.empty():
        results.append(results_queue.get())
    
    if results:
        analyze_multi_vehicle_results(results)
        
        # After both threading and multiprocessing tests, run comparison
        print(f"\n{'='*60}")
        print("🔍 RUNNING AUTOMATED COMPARISON FROM LOGGED DATA")
        print(f"{'='*60}")
        compare_threading_vs_multiprocessing_from_logs()
        
    else:
        print("No results from threading test")


def run_gps_comparison_analysis():
    """Standalone function to analyze the single log file and compare performance."""
    print("🔍 GPS Performance Comparison Tool")
    print("=" * 50)
    
    log_file = "gps_performance_log.json"
    
    try:
        import json
        with open(log_file, 'r') as f:
            log_data = json.load(f)
        
        test_runs = log_data.get('test_runs', [])
        threading_runs = [run for run in test_runs if run['method'] == 'threading']
        multiprocessing_runs = [run for run in test_runs if run['method'] == 'multiprocessing']
        
        print(f"📁 Found log file: {log_file}")
        print(f"   Threading runs: {len(threading_runs)}")
        print(f"   Multiprocessing runs: {len(multiprocessing_runs)}")
        
        if len(threading_runs) == 0:
            print("❌ No threading data found. Run threading test first.")
            return None
        elif len(multiprocessing_runs) == 0:
            print("❌ No multiprocessing data found. Run multiprocessing test first.")
            return None
        
    except FileNotFoundError:
        print(f"❌ Log file not found: {log_file}")
        print("   Run threading and multiprocessing tests first.")
        return None
    except Exception as e:
        print(f"❌ Error reading log file: {e}")
        return None
    
    # Run automatic comparison
    comparison_result = compare_threading_vs_multiprocessing_from_logs()
    
    if comparison_result:
        print("\n✅ Comparison completed successfully!")
        print("📋 Check gps_comparison_report.json for detailed results.")
    else:
        print("\n❌ Comparison failed. Check log file contents.")
    
    return comparison_result

if __name__ == "__main__":
    """
    QUICK GUIDE: How to use the GPS comparison system
    
    1. Configuration (at the top of this file):
       - DEFAULT_TARGET_HZ = 50    # Changes default rate to 50Hz
       - TEST_RATES = [25, 50, 75, 100]  # Test multiple rates
       - TEST_DURATION = 10  # Change test duration
    
    2. Single file logging system:
       - All data logs to: gps_performance_log.json (overwrites previous data)
       - Threading and multiprocessing data stored in same file
       - Comparison report saved to: gps_comparison_report.json (overwrites)
    
    3. Automatic comparison runs after tests complete, or run manually:
       - Auto: Runs after threading/multiprocessing tests
       - Manual: Call run_gps_comparison_analysis()
       - Command line: python script.py --compare
    
    4. File structure:
       - gps_performance_log.json: Raw performance data (single file)
       - gps_comparison_report.json: Analysis and recommendations (single file)
    
    Usage options:
    - Option 1: Run full test suite (includes auto-comparison)
    - Option 2: Run comparison analysis only
    
    Common rates:
    - 25Hz = 40ms intervals (conservative, low CPU)
    - 50Hz = 20ms intervals (recommended for most applications)
    - 75Hz = 13.3ms intervals (higher performance)
    - 100Hz = 10ms intervals (maximum performance)
    """
    try:
        import sys
        
        # Check if user wants to run comparison only
        if len(sys.argv) > 1 and sys.argv[1] == '--compare':
            print("🔍 Running GPS comparison analysis only...")
            run_gps_comparison_analysis()
        else:
            # Option 1: Run the full test suite (includes automatic comparison)
            print("🚀 Running full GPS timing test suite...")
            simple_timing_test()
        
        # Option 2: Manual comparison (uncomment to use)
        # print("\n🔍 Running manual GPS comparison analysis...")
        # run_gps_comparison_analysis()
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed: {e}")
        import traceback
        traceback.print_exc()
