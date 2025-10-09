#!/usr/bin/env python3
"""
Performance monitoring tool for the fleet communication system.
This tool measures and reports on the processing chain performance.
"""

import time
import logging
import threading
from typing import Dict, List
import statistics

class CommunicationPerformanceMonitor:
    """
    Monitor and analyze communication processing performance.
    """
    
    def __init__(self):
        self.processing_times = []
        self.validation_times = []
        self.json_decode_times = []
        self.lock = threading.Lock()
        
        # Performance targets
        self.target_processing_time = 0.05  # 50ms target
        self.warning_threshold = 0.1       # 100ms warning
        self.critical_threshold = 0.5      # 500ms critical
        
        self.logger = logging.getLogger("PerfMonitor")
    
    def start_timing(self) -> float:
        """Start timing a processing operation."""
        return time.perf_counter()
    
    def end_timing(self, start_time: float, operation_type: str = "processing") -> float:
        """
        End timing and record the measurement.
        
        Args:
            start_time: Start time from start_timing()
            operation_type: Type of operation being measured
            
        Returns:
            Duration in seconds
        """
        duration = time.perf_counter() - start_time
        
        with self.lock:
            if operation_type == "processing":
                self.processing_times.append(duration)
            elif operation_type == "validation":
                self.validation_times.append(duration)
            elif operation_type == "json_decode":
                self.json_decode_times.append(duration)
        
        # Log performance issues
        if duration > self.critical_threshold:
            self.logger.error(f"CRITICAL: {operation_type} took {duration*1000:.1f}ms (threshold: {self.critical_threshold*1000:.1f}ms)")
        elif duration > self.warning_threshold:
            self.logger.warning(f"SLOW: {operation_type} took {duration*1000:.1f}ms (threshold: {self.warning_threshold*1000:.1f}ms)")
        
        return duration
    
    def get_performance_stats(self) -> Dict:
        """Get comprehensive performance statistics."""
        with self.lock:
            def calc_stats(times: List[float]) -> Dict:
                if not times:
                    return {'count': 0, 'avg': 0, 'min': 0, 'max': 0, 'p95': 0, 'p99': 0}
                
                return {
                    'count': len(times),
                    'avg': statistics.mean(times) * 1000,  # Convert to ms
                    'min': min(times) * 1000,
                    'max': max(times) * 1000,
                    'p95': statistics.quantiles(times, n=20)[18] * 1000 if len(times) >= 20 else max(times) * 1000,
                    'p99': statistics.quantiles(times, n=100)[98] * 1000 if len(times) >= 100 else max(times) * 1000
                }
            
            stats = {
                'processing': calc_stats(self.processing_times),
                'validation': calc_stats(self.validation_times),
                'json_decode': calc_stats(self.json_decode_times),
                'targets': {
                    'target_ms': self.target_processing_time * 1000,
                    'warning_ms': self.warning_threshold * 1000,
                    'critical_ms': self.critical_threshold * 1000
                }
            }
            
            # Calculate performance health
            recent_processing = self.processing_times[-100:] if len(self.processing_times) >= 100 else self.processing_times
            if recent_processing:
                avg_recent = statistics.mean(recent_processing)
                if avg_recent <= self.target_processing_time:
                    health = "EXCELLENT"
                elif avg_recent <= self.warning_threshold:
                    health = "GOOD"
                elif avg_recent <= self.critical_threshold:
                    health = "WARNING"
                else:
                    health = "CRITICAL"
                    
                stats['health'] = {
                    'status': health,
                    'recent_avg_ms': avg_recent * 1000,
                    'target_achievement': (self.target_processing_time / avg_recent) * 100 if avg_recent > 0 else 100
                }
            else:
                # No data collected yet
                stats['health'] = {
                    'status': 'WAITING_FOR_DATA',
                    'recent_avg_ms': 0,
                    'target_achievement': 0,
                    'message': 'No performance data collected yet. Ensure the system is processing messages.'
                }
            
            return stats
    
    def print_performance_report(self):
        """Print a detailed performance report."""
        stats = self.get_performance_stats()
        
        print("\n" + "="*60)
        print("COMMUNICATION PERFORMANCE REPORT")
        print("="*60)
        
        health_status = stats.get('health', {}).get('status', 'UNKNOWN')
        print(f"\nOVERALL HEALTH: {health_status}")
        
        if 'health' in stats:
            if health_status == 'WAITING_FOR_DATA':
                print(f"Message: {stats['health'].get('message', 'No data available')}")
                print("Troubleshooting:")
                print("  1. Ensure your fleet communication system is running")
                print("  2. Check that PERFORMANCE_MONITORING = True in CommHandler.py")
                print("  3. Verify that start_timing() and end_timing() are being called")
                print("  4. Check for any import errors in the logs")
            else:
                print(f"Recent Average: {stats['health']['recent_avg_ms']:.1f}ms")
                print(f"Target Achievement: {stats['health']['target_achievement']:.1f}%")
        
        print(f"\nPERFORMANCE TARGETS:")
        print(f"  Target: ≤{stats['targets']['target_ms']:.1f}ms")
        print(f"  Warning: ≤{stats['targets']['warning_ms']:.1f}ms")
        print(f"  Critical: ≤{stats['targets']['critical_ms']:.1f}ms")
        
        # Show data collection status
        total_measurements = (stats['processing']['count'] + 
                            stats['validation']['count'] + 
                            stats['json_decode']['count'])
        
        if total_measurements == 0:
            print(f"\nDATA COLLECTION STATUS:")
            print(f"  No measurements recorded yet")
            print(f"  Processing operations: {stats['processing']['count']}")
            print(f"  Validation operations: {stats['validation']['count']}")
            print(f"  JSON decode operations: {stats['json_decode']['count']}")
        else:
            for operation, data in stats.items():
                if operation in ['processing', 'validation', 'json_decode'] and data['count'] > 0:
                    print(f"\n{operation.upper()} PERFORMANCE:")
                    print(f"  Count: {data['count']:,} operations")
                    print(f"  Average: {data['avg']:.1f}ms")
                    print(f"  Min/Max: {data['min']:.1f}ms / {data['max']:.1f}ms")
                    print(f"  95th percentile: {data['p95']:.1f}ms")
                    print(f"  99th percentile: {data['p99']:.1f}ms")
        
        print("="*60)
    
    def reset_measurements(self):
        """Reset all measurements for a fresh start."""
        with self.lock:
            self.processing_times.clear()
            self.validation_times.clear()
            self.json_decode_times.clear()
        
        self.logger.info("Performance measurements reset")
    
    def add_test_data(self, count: int = 10):
        """Add some test data for testing the performance monitor."""
        import random
        
        with self.lock:
            # Add some realistic test data
            for _ in range(count):
                # Simulate processing times (20-80ms)
                test_time = random.uniform(0.02, 0.08)
                self.processing_times.append(test_time)
                
                # Simulate validation times (1-5ms)
                test_time = random.uniform(0.001, 0.005)
                self.validation_times.append(test_time)
                
                # Simulate JSON decode times (0.5-2ms)
                test_time = random.uniform(0.0005, 0.002)
                self.json_decode_times.append(test_time)
        
        self.logger.info(f"Added {count} test measurements for testing")
    
    def get_integration_status(self) -> Dict:
        """Check if performance monitoring is properly integrated."""
        status = {
            'monitor_available': True,
            'measurements_recorded': len(self.processing_times) > 0,
            'total_measurements': len(self.processing_times) + len(self.validation_times) + len(self.json_decode_times),
            'processing_count': len(self.processing_times),
            'validation_count': len(self.validation_times),
            'json_decode_count': len(self.json_decode_times)
        }
        return status

# Global performance monitor instance
perf_monitor = CommunicationPerformanceMonitor()
