import time
import threading
from collections import deque
from typing import Dict, Any, Optional, List
import logging
import numpy as np

class StateQueue:
    """
    Queue system for managing received vehicle states with validity checking.
    
    Features:
    - Time-based validation using GPS-synchronized timestamps
    - Automatic cleanup of old/invalid states
    - Thread-safe operations
    - Configurable thresholds for state acceptance
    """
    
    def __init__(self, max_queue_size: int = 32, max_age_seconds: float = 0.2, 
                 max_delay_threshold: float = 0.5, logger: logging.Logger = None):
        """
        Initialize the state queue.
        
        Args:
            max_queue_size: Maximum number of states to keep in queue
            max_age_seconds: Maximum age of states to consider valid (seconds)
            max_delay_threshold: Maximum acceptable delay between GPS time and state timestamp
            logger: Logger instance for debugging
        """
        self.max_queue_size = max_queue_size
        self.max_age_seconds = max_age_seconds
        self.max_delay_threshold = max_delay_threshold
        self.logger = logger or logging.getLogger(__name__)
        
        # Thread-safe queue for received states
        self.state_queue = deque(maxlen=max_queue_size)
        self.lock = threading.Lock()
        
        # GPS time caching for performance optimization
        self._cached_gps_time = 0.0
        self._cache_update_time = 0.0
        self._cache_validity_period = 0.1  # Cache GPS time for 100ms
        
        # Statistics for monitoring
        self.stats = {
            'total_received': 0,
            'valid_states': 0,
            'expired_states': 0,
            'delayed_states': 0,
            'duplicate_states': 0,
            'queue_size': 0,
            'last_cleanup': time.time(),
            'gps_cache_hits': 0,
            'gps_cache_misses': 0
        }
        
        self.logger.info(f"StateQueue initialized - max_size: {max_queue_size}, "
                        f"max_age: {max_age_seconds}s, max_delay: {max_delay_threshold}s")
    
    def _get_cached_gps_time(self, gps_sync) -> float:
        """
        Get GPS time with caching for performance optimization.
        
        Args:
            gps_sync: GPS synchronization object
            
        Returns:
            Current GPS time (cached or fresh)
        """
        current_time = time.time()
        
        # Check if cache is still valid
        if (current_time - self._cache_update_time) < self._cache_validity_period:
            self.stats['gps_cache_hits'] += 1
            return self._cached_gps_time
        
        # Cache miss - get fresh GPS time
        self.stats['gps_cache_misses'] += 1
        self._cached_gps_time = gps_sync.get_synced_time()
        self._cache_update_time = current_time
        
        return self._cached_gps_time
    
    def add_state(self, state_data: Dict[str, Any], gps_sync) -> bool:
        """
        Add a new state to the queue with validation.
        
        Args:
            state_data: State data dictionary containing timestamp, pos, rot, velocity, etc.
            gps_sync: GPS synchronization object for time validation
            
        Returns:
            True if state was added successfully, False if rejected
        """
        with self.lock:
            self.stats['total_received'] += 1
            
            # Extract sender_id early for error reporting
            sender_id = state_data.get('id', state_data.get('vehicle_id' , state_data.get('sender_id', 'unknown')))
            
            # Extract and validate timestamp
            state_timestamp = state_data.get('timestamp')
            # Use cached GPS time for better performance
            current_gps_time = self._get_cached_gps_time(gps_sync)
            
            if state_timestamp is None:
                self.logger.warning(f"State rejected: Missing timestamp from vehicle {sender_id}")
                return False
            
            # Calculate delay between state timestamp and current GPS time
            time_delay = current_gps_time - state_timestamp
            
            # Check if state is too old or from the future (with small tolerance)
            if time_delay > self.max_age_seconds:
                self.stats['expired_states'] += 1
                self.logger.warning(f"State rejected: Too old (delay: {time_delay:.3f}s, max_age: {self.max_age_seconds}s, current_gps: {current_gps_time:.3f}, state_ts: {state_timestamp:.3f}) from vehicle {sender_id}")
                return False
            
            if time_delay < -0.1:  # Allow small future tolerance for network jitter
                self.stats['delayed_states'] += 1
                self.logger.warning(f"State rejected: From future (delay: {time_delay:.3f}s, current_gps: {current_gps_time:.3f}, state_ts: {state_timestamp:.3f}) from vehicle {sender_id}")
                return False
            
            # Check for excessive delay that indicates network problems
            if abs(time_delay) > self.max_delay_threshold:
                self.stats['delayed_states'] += 1
                self.logger.warning(f"State rejected: Excessive delay ({time_delay:.3f}s) from vehicle {sender_id}, "
                                  f"threshold: {self.max_delay_threshold}s, current_gps: {current_gps_time:.3f}, "
                                  f"state_timestamp: {state_timestamp:.3f}")
                return False
            
            # Check for duplicate sequence numbers (if available)
            seq_num = state_data.get('seq')
            
            if seq_num is not None and sender_id is not None:
                # More intelligent duplicate detection:
                # Only reject if we have the EXACT same sequence number from the same sender
                # AND it was received recently (within last 2 seconds)
                duplicate_found = False
                current_time = current_gps_time
                
                for existing_state in self.state_queue:
                    if (existing_state.get('seq') == seq_num and 
                        existing_state.get('id') == sender_id):
                        # Check if the existing state is recent (within 2 seconds)
                        existing_received_time = existing_state.get('received_time', 0)
                        time_since_existing = current_time - existing_received_time
                        
                        if time_since_existing <= 2.0:  # Only consider recent duplicates
                            duplicate_found = True
                            self.logger.debug(f"State rejected: Recent duplicate seq {seq_num} from vehicle {sender_id} (within {time_since_existing:.2f}s)")
                            break
                
                if duplicate_found:
                    self.stats['duplicate_states'] += 1
                    self.logger.warning(f"State rejected: Recent duplicate seq {seq_num} from vehicle {sender_id} "
                                      f"(within {time_since_existing:.2f}s)")
                    return False
            
            # Add metadata for tracking
            enhanced_state = state_data.copy()
            enhanced_state.update({
                'received_time': current_gps_time,
                'processing_delay': time_delay,
                'is_valid': True
            })
            
            # Add to queue (automatically removes oldest if at max capacity)
            self.state_queue.append(enhanced_state)
            self.stats['valid_states'] += 1
            self.stats['queue_size'] = len(self.state_queue)
            
            self.logger.debug(f"State added: Seq {seq_num}, Vehicle {sender_id}, "
                            f"Delay: {time_delay:.3f}s, Queue size: {len(self.state_queue)}")
            
            return True
    
    def get_latest_valid_state(self, sender_id: Optional[int] = None, current_gps_time: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """
        Get the most recent valid state, optionally filtered by sender.
        
        Args:
            sender_id: Optional vehicle ID to filter by
            current_gps_time: Current GPS time for validation
            
        Returns:
            Most recent valid state dictionary or None if no valid states
        """
        with self.lock:
            if not self.state_queue:
                return None
            
            # Search from newest to oldest
            for state in reversed(self.state_queue):
                if sender_id is None or state.get('id') == sender_id:
                    # Double-check state is still valid (not too old)
                    if self._is_state_currently_valid(state, current_gps_time):
                        return state  # Return reference for performance (caller should not modify)
                    
            return None
    
    def get_interpolated_state(self, target_time: float, sender_id: Optional[int] = None, current_gps_time: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """
        Get interpolated state for a specific time using nearby states.
        
        Args:
            target_time: GPS-synchronized time to interpolate for
            sender_id: Optional vehicle ID to filter by
            current_gps_time: Current GPS time for validation
            
        Returns:
            Interpolated state dictionary or None if interpolation not possible
        """
        with self.lock:
            # Find states from the same sender around the target time
            relevant_states = []
            for state in self.state_queue:
                if sender_id is None or state.get('id') == sender_id:
                    if self._is_state_currently_valid(state, current_gps_time):
                        relevant_states.append(state)
            
            if len(relevant_states) < 2:
                # Not enough states for interpolation, return latest if available
                return relevant_states[-1].copy() if relevant_states else None
            
            # Sort by timestamp
            relevant_states.sort(key=lambda x: x.get('timestamp', 0))
            
            # Find bounding states
            before_state = None
            after_state = None
            
            for i, state in enumerate(relevant_states):
                state_time = state.get('timestamp', 0)
                if state_time <= target_time:
                    before_state = state
                else:
                    after_state = state
                    break
            
            # Perform linear interpolation if we have bounding states
            if before_state and after_state:
                return self._interpolate_states(before_state, after_state, target_time)
            elif before_state:
                # Use latest available state
                return before_state.copy()
            elif after_state:
                # Use earliest available state
                return after_state.copy()
            
            return None
    
    def _interpolate_states(self, state1: Dict[str, Any], state2: Dict[str, Any], 
                          target_time: float) -> Dict[str, Any]:
        """
        Linearly interpolate between two states.
        
        Args:
            state1: Earlier state
            state2: Later state
            target_time: Target time for interpolation
            
        Returns:
            Interpolated state dictionary
        """
        t1 = state1.get('timestamp', 0)
        t2 = state2.get('timestamp', 0)
        
        if t2 - t1 <= 1e-6:  # Avoid division by zero
            return state1.copy()
        
        # Interpolation factor (0 = state1, 1 = state2)
        alpha = (target_time - t1) / (t2 - t1)
        alpha = max(0, min(1, alpha))  # Clamp to [0, 1]
        
        # Interpolate position
        pos1 = np.array(state1.get('pos', [0, 0, 0]))
        pos2 = np.array(state2.get('pos', [0, 0, 0]))
        interp_pos = pos1 + alpha * (pos2 - pos1)
        
        # Interpolate rotation (simple linear, could use slerp for better results)
        rot1 = np.array(state1.get('rot', [0, 0, 0]))
        rot2 = np.array(state2.get('rot', [0, 0, 0]))
        interp_rot = rot1 + alpha * (rot2 - rot1)
        
        # Interpolate velocity
        v1 = state1.get('v', 0)
        v2 = state2.get('v', 0)
        interp_v = v1 + alpha * (v2 - v1)
        
        # Create interpolated state
        interpolated_state = {
            'id': state1.get('id'),
            'pos': interp_pos,
            'rot': interp_rot,
            'v': interp_v,
            'timestamp': target_time,
            'interpolated': True,
            'alpha': alpha,
            'source_states': [state1.get('seq'), state2.get('seq')]
        }
        
        return interpolated_state
    
    def _is_state_currently_valid(self, state: Dict[str, Any], current_gps_time: Optional[float] = None) -> bool:
        """
        Check if a state is currently valid based on its age.
        
        Args:
            state: State dictionary to validate
            current_gps_time: Current GPS time for validation (if None, uses wall time)
            
        Returns:
            True if state is still valid, False otherwise
        """
        # Use GPS timestamp for age validation if available, otherwise fall back to received_time
        state_timestamp = state.get('timestamp', state.get('received_time', 0))
        
        if current_gps_time is not None:
            # Use GPS time for validation (preferred for synchronized systems)
            age = current_gps_time - state_timestamp
        else:
            # Fallback to wall time if GPS time not available
            current_time = time.time()
            received_time = state.get('received_time', 0)
            age = current_time - received_time
        
        return age <= self.max_age_seconds
    
    def cleanup_old_states(self, gps_sync) -> int:
        """
        Remove old/invalid states from the queue.
        
        Args:
            gps_sync: GPS synchronization object for current time
            
        Returns:
            Number of states removed
        """
        with self.lock:
            initial_size = len(self.state_queue)
            current_gps_time = gps_sync.get_synced_time()
            
            # Remove states that are too old
            valid_states = deque()
            for state in self.state_queue:
                state_age = current_gps_time - state.get('timestamp', 0)
                if state_age <= self.max_age_seconds:
                    valid_states.append(state)
            
            self.state_queue = valid_states
            removed_count = initial_size - len(self.state_queue)
            
            self.stats['queue_size'] = len(self.state_queue)
            self.stats['last_cleanup'] = time.time()
            
            if removed_count > 0:
                self.logger.debug(f"Cleaned up {removed_count} old states, "
                                f"queue size now: {len(self.state_queue)}")
            
            return removed_count
    
    def get_queue_stats(self) -> Dict[str, Any]:
        """
        Get current queue statistics for monitoring.
        
        Returns:
            Dictionary with queue statistics
        """
        with self.lock:
            stats = self.stats.copy()
            stats.update({
                'current_queue_size': len(self.state_queue),
                'queue_utilization': len(self.state_queue) / self.max_queue_size * 100,
                'acceptance_rate': (stats['valid_states'] / max(1, stats['total_received'])) * 100,
                'rejection_breakdown': {
                    'expired': stats['expired_states'],
                    'delayed': stats['delayed_states'],
                    'duplicates': stats['duplicate_states']
                },
                'performance_metrics': {
                    'gps_cache_hits': stats.get('gps_cache_hits', 0),
                    'gps_cache_misses': stats.get('gps_cache_misses', 0),
                    'cache_hit_rate': (stats.get('gps_cache_hits', 0) / 
                                     max(1, stats.get('gps_cache_hits', 0) + stats.get('gps_cache_misses', 0))) * 100
                }
            })
            return stats
    
    def get_all_states(self, sender_id: Optional[int] = None, current_gps_time: Optional[float] = None) -> List[Dict[str, Any]]:
        """
        Get all valid states in the queue, optionally filtered by sender.
        
        Args:
            sender_id: Optional vehicle ID to filter by
            current_gps_time: Current GPS time for validation
            
        Returns:
            List of state dictionaries
        """
        with self.lock:
            states = []
            for state in self.state_queue:
                if sender_id is None or state.get('id') == sender_id:
                    if self._is_state_currently_valid(state, current_gps_time):
                        states.append(state.copy())
            return states
    
    def clear(self):
        """Clear all states from the queue."""
        with self.lock:
            self.state_queue.clear()
            self.stats['queue_size'] = 0
            self.logger.info("State queue cleared")
