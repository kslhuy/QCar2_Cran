"""
Fleet State Estimators for Distributed Observation

Provides different distributed state estimation strategies with a common interface.
Easy to switch between different algorithms (Consensus, Kalman Consensus, etc.).
"""
import numpy as np
import time
import os
import yaml
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple
from collections import defaultdict

# Canonical ordering for state fields used in V2V/local state messages.
STATE_FIELDS = ("x", "y", "theta", "velocity", "acceleration")


def _normalize_state_array(state_array, expected_dim: int, logger=None) -> Optional[np.ndarray]:
    """
    Convert any array-like to a 1D numpy array with the expected dimension.
    Pads with zeros or truncates if needed to avoid shape errors in downstream algorithms.
    """
    try:
        arr = np.asarray(state_array, dtype=float).flatten()
        if arr.shape[0] == expected_dim:
            return arr
        if arr.shape[0] > expected_dim:
            if logger:
                logger.logger.debug(
                    f"State dim {arr.shape[0]} larger than expected {expected_dim}, truncating"
                )
            return arr[:expected_dim]
        # Pad with zeros when shorter than expected
        if logger:
            logger.logger.debug(
                f"State dim {arr.shape[0]} smaller than expected {expected_dim}, padding with zeros"
            )
        return np.pad(arr, (0, expected_dim - arr.shape[0]), mode="constant")
    except Exception as exc:
        if logger:
            logger.log_error("Failed to normalize state array", exc)
        return None


def _state_dict_to_array(state_dict: Dict, expected_dim: int, logger=None) -> Optional[np.ndarray]:
    """
    Convert a state dict (as used in communication/log layers) to ndarray in canonical order.
    """
    try:
        base_arr = np.array([state_dict.get(k, 0.0) for k in STATE_FIELDS], dtype=float)
        return _normalize_state_array(base_arr, expected_dim, logger=logger)
    except Exception as exc:
        if logger:
            logger.log_error("Failed to convert state dict to array", exc)
        return None


class FleetStateEstimatorBase(ABC):
    """Base class for all fleet state estimators"""
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5, 
                 config: Dict = None, logger=None):
        """
        Initialize base fleet estimator
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension (default 5: x, y, theta, v, a) based on vehicle model
            config: Configuration dict
            logger: Logger instance
        """
        self.vehicle_id = vehicle_id
        self.fleet_size = fleet_size
        self.state_dim = state_dim
        self.config = config or {}
        self.logger = logger
        
        # Fleet state estimates [state_dim x fleet_size]
        self.fleet_states = np.zeros((self.state_dim, fleet_size))
        print(f"fleet_states initialized with shape: {self.fleet_states.shape}")
        
        # Communication data storage
        self.received_local_states = defaultdict(list)  # vehicle_id -> [(timestamp_ns, state)]

        self.received_fleet_states = defaultdict(list) # vehicle_sender_id -> [(timestamp_ns, fleet_state)]
        
        # Observer state storage for direct consensus (avoids di0 conversion errors)
        # sender_id -> [(timestamp_ns, observer_state_vector)]
        self.received_observer_states = defaultdict(list)

        max_state_age_ns_cfg = self.config.get("max_state_age_ns")
        if max_state_age_ns_cfg is None:
            max_state_age_s = float(self.config.get("max_state_age_s", 1.0))
            self.max_state_age_ns = int(max_state_age_s * 1e9)
        else:
            self.max_state_age_ns = int(max_state_age_ns_cfg)
        self.received_local_states_limit = int(self.config.get("received_local_states_limit", 10))
    
    @abstractmethod
    def update(self, local_state: np.ndarray, dt: float, 
               current_time_ns: int, control: np.ndarray) -> np.ndarray:
        """
        Update fleet state estimates
        
        Args:
            local_state: Host vehicle's local state estimate [state_dim]
            dt: Time step
            current_time: Current timestamp in nanoseconds
            control: Current control input [steering, throttle]
            
        Returns:
            Updated fleet states [state_dim x fleet_size]
        """
        pass
    
    def add_received_local_state(self, sender_id: int, state: Dict, timestamp_ns: int) -> bool:
        """Add a received LOCAL state (dict or ndarray) and store BOTH dict and ndarray.

        CRITICAL: We now store BOTH the original dict (with control_input) and the state array.
        This allows _get_latest_received_control() to access control signals while
        _get_latest_received_state() gets the state vector.
        
        Storage format: (timestamp_ns, state_data)
        - If input is dict: state_data = original dict (preserves control_input)
        - If input is array: state_data = normalized array
        """
        # # print(f"Adding received local state from vehicle_id {sender_id}")
        try:
            if sender_id == self.vehicle_id:
                return False  # Don't store own state

            # IMPORTANT: Store the original data type to preserve control_input
            if isinstance(state, dict):
                # Store the complete dict (includes control_input if present)
                state_data = state.copy()
            else:
                # Normalize array
                state_vec = _normalize_state_array(state, self.state_dim, logger=self.logger)
                if state_vec is None:
                    return False
                state_data = state_vec.copy()

            # Store timestamp in nanoseconds with original data
            self.received_local_states[sender_id].append((timestamp_ns, state_data))

            if len(self.received_local_states[sender_id]) > self.received_local_states_limit:
                self.received_local_states[sender_id] = (
                    self.received_local_states[sender_id][-self.received_local_states_limit:]
                )

            return True

        except Exception as e:
            if self.logger:
                self.logger.log_error("Add received local state error", e)
            return False

    def add_received_fleet_state(self, sender_id: int, fleet_estimates: Dict, timestamp_ns: int) -> bool:
        """Add a received FLEET state broadcast from another vehicle.

        Default implementation stores the raw fleet dictionary in
        `received_fleet_states` and also unpacks per-vehicle states into
        `received_states` for easy access by other algorithms.
        """
        try:
            if sender_id == self.vehicle_id:
                return False

            # Check for new vehicle IDs and expand capacity if required
            try:
                max_id_in_msg = max((int(vid) for vid in fleet_estimates.keys()), default=0)
            except Exception:
                max_id_in_msg = 0

            if max_id_in_msg >= self.fleet_size:
                self._ensure_fleet_capacity(max_id_in_msg)

            # Store the raw fleet dictionary with timestamp
            self.received_fleet_states[sender_id].append((timestamp_ns, fleet_estimates))


            # Limit history for fleet snapshots per neighbor (default 5)
            if len(self.received_fleet_states[sender_id]) > 5:
                self.received_fleet_states[sender_id] = self.received_fleet_states[sender_id][-5:]

            return True

        except Exception as e:
            if self.logger:
                self.logger.log_error("Add received fleet state error", e)
            return False

    def add_received_observer_state(self, sender_id: int, observer_state: np.ndarray, timestamp_ns: int) -> bool:
        """Add a received OBSERVER state (x_vec) from another vehicle for direct consensus.
        
        This allows direct consensus calculation without converting fleet_states,
        which avoids errors introduced by different di0 calculations across vehicles.
        
        Args:
            sender_id: Vehicle ID of the sender
            observer_state: Observer state vector [3*observer_size] containing:
                           [p1-p0+d10, v1-v0, a1-a0, p2-p0+d20, v2-v0, a2-a0, ...]
            timestamp_ns: Timestamp in nanoseconds
        
        Returns:
            True if successfully stored, False otherwise
        """
        try:
            if sender_id == self.vehicle_id:
                return False
            
            # Ensure it's a numpy array
            if not isinstance(observer_state, np.ndarray):
                observer_state = np.array(observer_state)
            
            # Store with timestamp
            self.received_observer_states[sender_id].append((timestamp_ns, observer_state.copy()))
            
            # Keep only recent history (default 5 entries)
            if len(self.received_observer_states[sender_id]) > 5:
                self.received_observer_states[sender_id] = self.received_observer_states[sender_id][-5:]
            
            return True
        
        except Exception as e:
            if self.logger:
                self.logger.log_error("Add received observer state error", e)
            return False

    def _get_latest_observer_state(self, neighbor_id: int, current_time_ns: int) -> Optional[np.ndarray]:
        """Return the newest observer state vector from a neighbor that is still valid.
        
        Args:
            neighbor_id: Vehicle ID of the neighbor
            current_time_ns: Current timestamp in nanoseconds
        
        Returns:
            Observer state vector [3*observer_size] or None if no valid data
        """
        if neighbor_id not in self.received_observer_states:
            return None
        
        history = self.received_observer_states[neighbor_id]
        if not history:
            return None
        
        # Iterate backwards to find newest valid data
        for ts_ns, observer_state in reversed(history):
            if (current_time_ns - ts_ns) <= self.max_state_age_ns:
                return observer_state.copy()
        return None

    def _get_latest_fleet_data(self, neighbor_id: int, current_time_ns: int) -> Optional[Dict]:
        """Return the newest fleet dictionary from a neighbor that is still valid.

        Returns None if there is no recent valid snapshot.
        """
        if neighbor_id not in self.received_fleet_states:
            return None

        history = self.received_fleet_states[neighbor_id]
        if not history:
            return None

        # Iterate backwards to find newest valid data
        for ts_ns, fleet_data in reversed(history):
            if (current_time_ns - ts_ns) <= self.max_state_age_ns:
                return fleet_data
        return None

    def _get_latest_received_state(self, vehicle_id: int, current_time_ns: int) -> Optional[np.ndarray]:
        """Return the most recent received state (as ndarray) within the age limit.

        Returns None if no recent state is available or conversion fails.
        """
        if vehicle_id not in self.received_local_states:
            print(f"vehicle_id {vehicle_id} not in self.received_local_states")
            return None

        states_list = self.received_local_states[vehicle_id]
        if not states_list:
            print(f"states_list for vehicle_id {vehicle_id} is empty")
            return None

        # Iterate backwards (newest first) and return first valid entry
        for timestamp_ns, state in reversed(states_list):
            age_ns = current_time_ns - timestamp_ns
            if age_ns > self.max_state_age_ns:
                continue

            # History stores ndarray; older entries might still be dict, so normalize defensively
            if isinstance(state, dict):
                state_vec = _state_dict_to_array(state, self.state_dim, logger=self.logger)
            else:
                state_vec = _normalize_state_array(state, self.state_dim, logger=self.logger)

            if state_vec is not None:
                return state_vec
            
        print(f"No valid recent state for vehicle_id {vehicle_id}")
        return None

    def _get_latest_received_control(self, vehicle_id: int, current_time_ns: int) -> Optional[np.ndarray]:
        """
        Get the most recent control input from another vehicle via V2V communication.
        
        Control signals are transmitted in local_state broadcasts with the structure:
        {
            'vehicle_id': int,
            'x': float, 'y': float, 'theta': float,
            'velocity': float, 'acceleration': float,
            'control_input': {
                'steering': float,
                'throttle': float
            },
            ...
        }
        
        Args:
            vehicle_id: ID of the vehicle to get control from
            current_time_ns: Current timestamp in nanoseconds
        
        Returns:
            np.array([steering, throttle]) or None if no recent data available
        """
        if vehicle_id not in self.received_local_states:
            return None
        
        states_list = self.received_local_states[vehicle_id]
        if not states_list:
            return None
        
        # Iterate backwards (newest first) to find valid control data
        for timestamp_ns, state in reversed(states_list):
            age_ns = current_time_ns - timestamp_ns
            if age_ns > self.max_state_age_ns:
                continue
            
            # Extract control input if available
            # Note: state can be dict (from V2V) or ndarray (normalized)
            # We need the original dict to get control_input
            if isinstance(state, dict) and 'control_input' in state:
                control_dict = state['control_input']
                try:
                    return np.array([
                        float(control_dict.get('steering', 0.0)),
                        float(control_dict.get('throttle', 0.0))
                    ])
                except (ValueError, TypeError) as e:
                    if self.logger:
                        self.logger.logger.warning(
                            f"Failed to extract control from vehicle {vehicle_id}: {e}"
                        )
                    continue
        
        return None
    
    

    
    def get_fleet_states(self) -> np.ndarray:
        """Get current fleet state estimates"""
        return self.fleet_states.copy()
    
    def get_vehicle_state(self, vehicle_id: int) -> Optional[np.ndarray]:
        """Get state estimate for specific vehicle"""
        if 0 <= vehicle_id < self.fleet_size:
            return self.fleet_states[:, vehicle_id].copy()
        return None
    
    # Problem: Car_2 was trying to access index 2 in a fleet_states array that only had size 2 (indices 0-1), 
    # causing IndexError: index 2 is out of bounds for axis 1 with size 2.

    # Root Cause: When V2V activated with 2 cars (Car_0 and Car_1), the fleet was initialized with size 2. 
    # When Car_2 later joined, it tried to write its state to index 2, which didn't exist.

    # Solution: Added auto-expansion logic to fleet state estimators:

        # 1  Added _ensure_fleet_capacity() helper method to base class
        # 2  Modified ConsensusFleetEstimator.update() to auto-expand before writing
        # 3  Modified DistributedKalmanEstimator.update() to auto-expand and also update weights array
        # 4  The fleet_states array now dynamically grows to accommodate new vehicles with higher IDs
    
    def _ensure_fleet_capacity(self, min_vehicle_id: int):
        """
        Ensure fleet_states array can accommodate the given vehicle_id.
        Auto-expands the array if needed.
        
        Args:
            min_vehicle_id: Minimum vehicle ID that must be accommodated
        """
        if min_vehicle_id >= self.fleet_states.shape[1]:
            old_fleet_size = self.fleet_states.shape[1]
            new_fleet_size = min_vehicle_id + 1
            
            # Expand fleet_states array
            new_fleet_states = np.zeros((self.state_dim, new_fleet_size))
            new_fleet_states[:, :old_fleet_size] = self.fleet_states
            self.fleet_states = new_fleet_states
            self.fleet_size = new_fleet_size
            
            if self.logger:
                self.logger.logger.info(
                    f"{self.__class__.__name__}: Auto-expanded fleet size {old_fleet_size} -> {new_fleet_size} "
                    f"to accommodate vehicle_{min_vehicle_id}"
                )
    
    def reset(self):
        """Reset fleet estimator"""
        self.fleet_states = np.zeros((self.state_dim, self.fleet_size))
        self.received_local_states.clear()
        self.received_fleet_states.clear()

    def _cleanup_old_data(self, current_time_ns: int):
        """Clean up old entries from both received_states and received_fleet_states."""
        try:
            for vehicle_id in list(self.received_local_states.keys()):
                states_list = self.received_local_states[vehicle_id]

                # Remove old states (all in nanoseconds)
                valid_states = [(ts_ns, state) for ts_ns, state in states_list
                               if current_time_ns - ts_ns <= self.max_state_age_ns]

                if valid_states:
                    self.received_local_states[vehicle_id] = valid_states
                else:
                    del self.received_local_states[vehicle_id]


            # Clean fleet snapshots
            for sender_id in list(self.received_fleet_states.keys()):
                history = self.received_fleet_states[sender_id]
                valid_history = [
                    (ts, data) for ts, data in history
                    if (current_time_ns - ts) <= self.max_state_age_ns
                ]

                if valid_history:
                    self.received_fleet_states[sender_id] = valid_history
                else:
                    del self.received_fleet_states[sender_id]

        except Exception as e:
            if self.logger:
                self.logger.log_error("Data cleanup error", e)
class ConsensusFleetEstimator(FleetStateEstimatorBase):
    """
    Consensus-based distributed fleet estimator
    Simple and robust consensus algorithm
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5,
                 config: Dict = None, logger=None):
        """
        Initialize consensus fleet estimator
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension (default 5: x, y, theta, v, a)
            config: Configuration dict with 'consensus_gain'
            logger: Logger instance
        """
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)
        
        # Consensus gain (0-1, higher = faster consensus but less stable)
        self.consensus_gain = self.config.get('consensus_gain', 0.3)
        self.direct_gain = self.config.get('direct_gain', 0.5)
    
    def update(self, local_state: np.ndarray, dt: float, 
               current_time_ns: int, control: np.ndarray) -> np.ndarray:
        """
        Update fleet estimates using Consensus + Direct measurements
        
        Algorithm for Target T (estimated by Host H):
        New_Est(T) = Old_Est(T) 
                   + k_consensus * Sum(Neighbor_N's Est(T) - Host_H's Est(T))
                   + k_direct    * (Target_T's Self_Report - Host_H's Est(T))
        """
        try:
            # 1. Ensure capacity and set own state (Ground Truth for self)
            self._ensure_fleet_capacity(self.vehicle_id)
            self.fleet_states[:, self.vehicle_id] = local_state.copy()
            
            # 2. Update estimates for every other vehicle in the fleet
            for target_id in range(self.fleet_size):
                if target_id == self.vehicle_id:
                    continue  # Skip self
                
                # --- Step A: Get Current Estimate ---
                current_est = self.fleet_states[:, target_id].copy()
                total_correction = np.zeros_like(current_est)
                
                # --- Step B: Calculate Consensus Term (What neighbors think of Target) ---
                # "I trust my neighbors N1, N2... to tell me where Target T is"
                neighbor_count = 0
                consensus_accum = np.zeros_like(current_est)
                
                for neighbor_id, history in self.received_fleet_states.items():
                    # Get neighbor's latest fleet view
                    neighbor_fleet_dict = self._get_latest_fleet_data(neighbor_id, current_time_ns)
                    
                    if neighbor_fleet_dict and target_id in neighbor_fleet_dict:
                        # Extract what Neighbor thinks of Target
                        neigh_est_dict = neighbor_fleet_dict[target_id]
                        neigh_est_vec = np.array([
                            neigh_est_dict['x'], neigh_est_dict['y'], 
                            neigh_est_dict['theta'], neigh_est_dict['velocity'],
                            neigh_est_dict.get('acceleration', 0.0)
                        ])
                        
                        # Add difference (Neighbor - Self)
                        consensus_accum += (neigh_est_vec - current_est)
                        neighbor_count += 1
                
                if neighbor_count > 0:
                    # Average the consensus difference and apply gain
                    # Using average prevents gain from exploding with many neighbors
                    total_correction += self.consensus_gain * (consensus_accum / neighbor_count)

                # --- Step C: Calculate Direct Term (What Target says about itself) ---
                # "I trust Target T to tell me where Target T is" (Highest Confidence)
                direct_state = self._get_latest_received_state(target_id, current_time_ns)
                
                if direct_state is not None:
                    # Innovation: Direct_Broadcast - Current_Estimate
                    total_correction += self.direct_gain * (direct_state - current_est)
                
                # --- Step D: Apply Update ---
                self.fleet_states[:, target_id] = current_est + total_correction

            # 3. Cleanup old data from both storages
            self._cleanup_old_data(current_time_ns)
            
            return self.fleet_states.copy()
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Consensus fleet update error", e)
            return self.fleet_states.copy()
    

class FleetEstimatorFactory:
    """Factory to create fleet state estimators by name"""
    
    # Base estimator types (always available)
    ESTIMATOR_TYPES = {
        'consensus': ConsensusFleetEstimator,
        # 'distributed_luenberger' is loaded lazily to avoid circular imports
    }
    
    # Lazy loading flags
    _distributed_luenberger_loaded = False
    _leadering_observer_loaded = False
    _classical_luenberger_observer_loaded = False
    _high_gain_luenberger_observer_loaded = False
    _parallel_observers_loaded = False
    _trust_estimators_loaded = False
    
    @classmethod
    def _load_distributed_luenberger(cls):
        """Lazily load DistributedLuenbergerEstimator to avoid circular imports"""
        if cls._distributed_luenberger_loaded:
            return
        
        try:
            from .ShengyaObs.distributed_luenberger_estimator import DistributedLuenbergerEstimator
            cls.ESTIMATOR_TYPES['distributed_luenberger'] = DistributedLuenbergerEstimator
            cls._distributed_luenberger_loaded = True
        except ImportError as e:
            print(f"Warning: Could not load DistributedLuenbergerEstimator: {e}")

    @classmethod
    def _load_leadering_observer(cls):
        """Lazily load LeaderingObserverEstimator to avoid circular imports"""
        if cls._leadering_observer_loaded:
            return

        try:
            from .ShengyaObs.leadering_observer import LeaderingObserverEstimator
            cls.ESTIMATOR_TYPES['leadering_observer'] = LeaderingObserverEstimator
            cls._leadering_observer_loaded = True
        except ImportError as e:
            print(f"Warning: Could not load LeaderingObserverEstimator: {e}")

    @classmethod
    def _load_classical_luenberger_observer(cls):
        """Lazily load ClassicalLuenbergerObserverEstimator to avoid circular imports"""
        if cls._classical_luenberger_observer_loaded:
            return

        try:
            from .ShengyaObs.classical_luenberger_observer import ClassicalLuenbergerObserverEstimator
            cls.ESTIMATOR_TYPES['classical_luenberger_observer'] = ClassicalLuenbergerObserverEstimator
            cls.ESTIMATOR_TYPES['classical_luenberge_observer'] = ClassicalLuenbergerObserverEstimator
            cls._classical_luenberger_observer_loaded = True
        except ImportError as e:
            print(f"Warning: Could not load ClassicalLuenbergerObserverEstimator: {e}")

    @classmethod
    def _load_high_gain_luenberger_observer(cls):
        """Lazily load HighGainLuenbergerObserverEstimator to avoid circular imports"""
        if cls._high_gain_luenberger_observer_loaded:
            return

        try:
            from .ShengyaObs.classical_luenberger_observer import HighGainLuenbergerObserverEstimator
            cls.ESTIMATOR_TYPES['high_gain_luenberger_observer'] = HighGainLuenbergerObserverEstimator
            cls._high_gain_luenberger_observer_loaded = True
        except ImportError as e:
            print(f"Warning: Could not load HighGainLuenbergerObserverEstimator: {e}")

    @classmethod
    def _load_parallel_observers(cls):
        """Lazily load ParallelObserverEstimator to avoid circular imports"""
        if cls._parallel_observers_loaded:
            return

        try:
            from .ShengyaObs.parallel_observer_estimator import ParallelObserverEstimator
            cls.ESTIMATOR_TYPES['parallel_observers'] = ParallelObserverEstimator
            cls._parallel_observers_loaded = True
        except ImportError as e:
            print(f"Warning: Could not load ParallelObserverEstimator: {e}")
    
    @classmethod
    def _load_trust_estimators(cls):
        """Lazily load trust-based estimators to avoid circular imports"""
        if cls._trust_estimators_loaded:
            return
        
        try:
            from Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator import (
                TrustBasedFleetEstimator,
                TrustBasedKalmanEstimator
            )
            cls.ESTIMATOR_TYPES['trust_consensus'] = TrustBasedFleetEstimator
            cls.ESTIMATOR_TYPES['trust_kalman'] = TrustBasedKalmanEstimator
            cls._trust_estimators_loaded = True
        except ImportError as e:
            # Trust-based estimators not available
            pass
    
    @staticmethod
    def create(estimator_type: str, vehicle_id: int, fleet_size: int,
               state_dim: int = 5, config: Dict = None, logger=None):
        """
        Create a fleet state estimator
        
        Args:
            estimator_type: One of 'consensus', 'distributed_kalman', 
                           'distributed_luenberger', 'trust_consensus', 'trust_kalman'
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension (default 5)
            config: Configuration dict
            logger: Logger instance
            
        Returns:
            Fleet state estimator instance
        """
        # Load estimator lazily when requested
        if estimator_type == 'distributed_luenberger':
            FleetEstimatorFactory._load_distributed_luenberger()
        if estimator_type == 'leadering_observer':
            FleetEstimatorFactory._load_leadering_observer()
        if estimator_type in ('classical_luenberger_observer', 'classical_luenberge_observer'):
            FleetEstimatorFactory._load_classical_luenberger_observer()
        if estimator_type == 'high_gain_luenberger_observer':
            FleetEstimatorFactory._load_high_gain_luenberger_observer()
        if estimator_type == 'parallel_observers':
            FleetEstimatorFactory._load_parallel_observers()
        
        # Try to load trust-based estimators if requesting one
        if estimator_type.startswith('trust_'):
            FleetEstimatorFactory._load_trust_estimators()
        
        if estimator_type not in FleetEstimatorFactory.ESTIMATOR_TYPES:
            raise ValueError(
                f"Unknown fleet estimator type: {estimator_type}. "
                f"Available: {list(FleetEstimatorFactory.ESTIMATOR_TYPES.keys())}"
            )
        
        estimator_class = FleetEstimatorFactory.ESTIMATOR_TYPES[estimator_type]
        return estimator_class(
            vehicle_id=vehicle_id,
            fleet_size=fleet_size,
            state_dim=state_dim,
            config=config,
            logger=logger
        )
    
    @classmethod
    def get_available_types(cls) -> List[str]:
        """Get list of available estimator types"""
        cls._load_distributed_luenberger()
        cls._load_leadering_observer()
        cls._load_classical_luenberger_observer()
        cls._load_high_gain_luenberger_observer()
        cls._load_parallel_observers()
        cls._load_trust_estimators()
        return list(cls.ESTIMATOR_TYPES.keys())
