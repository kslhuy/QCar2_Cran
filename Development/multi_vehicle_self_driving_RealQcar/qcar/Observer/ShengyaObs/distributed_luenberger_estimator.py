"""
Distributed Luenberger Observer for Fleet State Estimation

This module provides a distributed Luenberger observer implementation
for fleet longitudinal state estimation with consensus-based communication.

Moved from fleet_state_estimators.py for better code organization.
"""
import numpy as np
import os
import yaml
from typing import Dict, List, Optional

from ..fleet_state_estimators import FleetStateEstimatorBase, ConsensusFleetEstimator


class DistributedLuenbergerEstimator(FleetStateEstimatorBase):
    """
    Distributed Luenberger Observer for fleet longitudinal estimation
    More sophisticated, uses dynamics model and consensus
    Inspired by VehicleObserver.py _distributed_observer_each
    
    Communication Topology:
    - Uses adjacency matrix to define which FOLLOWER vehicles can communicate
    - Adjacency matrix is [observer_size x observer_size], NOT including leader (vehicle 0)
    - Matrix indices 0..observer_size-1 correspond to follower vehicles 1..observer_size
    - Default: Chain topology (follower i talks to follower i-1 and i+1)
    - Can override with config['adjacency_matrix']
    - Consensus term only considers neighbors defined in adjacency matrix
    
    Example config for 4-vehicle fleet (1 leader + 3 followers):
        config = {
            'observer_gain': 0.1,
            'consensus_gain': 0.2,
            'adjacency_matrix': [  # Optional: custom topology [3x3] for 3 followers
                [0, 1, 0],  # Follower 1 (vehicle 1) connected to follower 2 (vehicle 2)
                [1, 0, 1],  # Follower 2 (vehicle 2) connected to followers 1 and 3
                [0, 1, 0]   # Follower 3 (vehicle 3) connected to follower 2 (vehicle 2)
            ]
        }
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5,
                 config: Dict = None, logger=None):
        """
        Initialize distributed Luenberger Observer
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet, including the leader labeled 0
            state_dim: State dimension (default 5: x, y, theta, v, a) based on vehicle model
            config: Configuration dict
            logger: Logger instance
        """
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)
        
        # Flag for delegating to consensus estimator if specified in config
        self.consensus_estimator = None
        
        # Initialize vehicle-specific parameters
        self.m_i = self.config.get('m_i', 0.5)
        self.tau_i = self.config.get('tau_i', 0.16) # 对估计结果的影响不大。

        self.rho_i = self.config.get('rho_i', 0.12)
        self.Cd_i = self.config.get('Cd_i', 0.035)
        self.AF_i = self.config.get('AF_i', 0.22)
        self.mu_i = self.config.get('mu_i', 0.01)

        # Load Gains (using extra configs if available)
        self._load_extra_config()

        if self.consensus_estimator:
            return

        # System matrices of the longitudinal model
        tau = self.tau_i  # Time constant
        self.h = 1     # time headway
        self.d = 0.8    # Desired distance
        self.observer_size = self.fleet_size - 1  # Exclude leader from observer size

        A_tau = np.array([
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0/tau],
        ])

        B_tau = np.array([
            [0.0],
            [0.0],
            [1.0/tau],
        ])

        A_h = np.array([
            [0.0, 0.0, self.h],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ])

        # B_delta = blkdiag(B_tau, B_tau, ..., B_tau)
        B_blocks = []
        for i in range(self.observer_size):
            row_blocks = []
            for j in range(self.observer_size):
                if i == j:
                    row_blocks.append(B_tau)
                else:
                    # B_tau is (3, 1), so zeros should be (3, 1)
                    row_blocks.append(np.zeros_like(B_tau))
            B_blocks.append(row_blocks)
        self.B_delta = np.block(B_blocks)

        A_h_tau = A_h + A_tau

        # A_delta = Lower triangular block matrix
        A_blocks = []
        Z_block = np.zeros_like(A_h_tau) # 3x3 zeros
        
        for i in range(self.observer_size):
            row_blocks = []
            for j in range(self.observer_size):
                if i == j:
                    row_blocks.append(A_h_tau)
                elif i > j:
                    row_blocks.append(A_h)
                else:
                    row_blocks.append(Z_block)
            A_blocks.append(row_blocks)
        self.A_delta = np.block(A_blocks)

        # -- Measurement matrix --
        self.Cv = np.array([-self.h, 1])
        self.Cd = np.array([-1, 0])
        self.Cf = np.array([[1, -self.h, 0],
                            [0, 1, 0]])
        self.Cp = np.array([[-1, 0, 0],
                            [0, 0, 0]])
        self.local_measurement_dim = 2 # measuring relative position and velocity pi - pi-1 vi

        # CRITICAL: Validate and correct gain matrix dimensions
        dim_distributed_observer = 3 * self.observer_size
        
        # Validate observer_gain: should be [dim_distributed_observer x local_measurement_dim]
        expected_observer_gain_shape = (dim_distributed_observer, self.local_measurement_dim)
        if hasattr(self.observer_gain, 'shape'):
            if self.observer_gain.shape != expected_observer_gain_shape:
                if self.logger:
                    self.logger.logger.error(
                        f"Vehicle {self.vehicle_id}: observer_gain shape mismatch! "
                        f"Expected {expected_observer_gain_shape}, got {self.observer_gain.shape}. "
                        f"Using default identity-based gain."
                    )
                # Use a simple default gain matrix
                self.observer_gain = np.eye(dim_distributed_observer, self.local_measurement_dim) * 0.1
        
        # Validate consensus_gain: should be [dim_distributed_observer x dim_distributed_observer]
        expected_consensus_gain_shape = (dim_distributed_observer, dim_distributed_observer)
        if hasattr(self.consensus_gain, 'shape'):
            if self.consensus_gain.shape != expected_consensus_gain_shape:
                if self.logger:
                    self.logger.logger.error(
                        f"Vehicle {self.vehicle_id}: consensus_gain shape mismatch! "
                        f"Expected {expected_consensus_gain_shape}, got {self.consensus_gain.shape}. "
                        f"Using default identity-based gain."
                    )
                # Use a simple default gain matrix
                self.consensus_gain = np.eye(dim_distributed_observer) * 0.2

        # Communication weights (uniform for now)
        self.weights = np.ones(self.observer_size) / self.observer_size
        
        # Communication adjacency matrix
        if 'adjacency_matrix' in self.config:
            self.adjacency_matrix = np.array(self.config['adjacency_matrix'])
        else:
            # Default chain topology
            self.adjacency_matrix = self._create_chain_topology()

        # Validate adjacency matrix dimensions
        if self.adjacency_matrix.shape != (self.observer_size, self.observer_size):
            self.adjacency_matrix = self._create_chain_topology()
        
        # Cache neighbor list at initialization
        self.my_neighbors = self.get_neighbors(self.vehicle_id)
        
        # Cache output matrix Ci at initialization
        self.Ci = self.compute_output_matrix_Ci(self.vehicle_id)
        
        # === Debug Data Storage and Recording ===
        # Dictionary to store internal data for each update cycle for debugging
        self.debug_data = {}
        self.debug_recording_enabled = self.config.get('debug_recording', True)
        self.debug_output_dir = self.config.get('debug_output_dir', 'observer_recordings')
        self.recorder = None
        self._update_count = 0
        self._recording_start_time = None
        
        # Auto-start recorder if debug_recording is enabled in config
        if self.debug_recording_enabled:
            self._init_recorder()
        
        # Initialize estimated_state (will be updated in the first update() call)
        # Distributed observer state dimension: 3 * observer_size
        self.estimated_state = np.zeros(3 * self.observer_size)
        
        # CRITICAL: Force fleet_states to correct dimensions
        # The parent class initializes it, but we must ensure it's correct
        # fleet_size includes leader (vehicle 0) + all followers
        if self.fleet_states.shape != (self.state_dim, self.fleet_size):
            self.fleet_states = np.zeros((self.state_dim, self.fleet_size))
    
        # Controller input
        # Store K matrices directly for each vehicle
        # K_matrices[vehicle_id][j] = K_{vehicle_id,j}
        # Load K matrices from yaml files for all vehicles
        self.K_all_vehicles = self._load_K_matrices_from_yaml()


        # Extract K matrices for current vehicle
        self.K_matrices = []
        if self.vehicle_id in self.K_all_vehicles:
            # Get K_i0, K_i1, ..., K_i(i-1) for vehicle i
            for j in range(self.vehicle_id):
                if j in self.K_all_vehicles[self.vehicle_id]:
                    self.K_matrices.append(self.K_all_vehicles[self.vehicle_id][j])
                else:
                    self.K_matrices.append(None)
        
        # Initialize filter variables for sensor measurements
        self.prev_distance_measurement = 0.0
        self.prev_velocity_measurement = 0.0
        self.prev_v0 = 0.0  # Previous filtered leader velocity
        self.prev_p0 = 0.0  # Previous filtered leader position
        # Default filter coefficients for different sensor measurements
        # Set to 1.0 to disable filtering (alpha=1.0 means: output = current measurement, no filtering)
        self.distance_filter_alpha = self.config.get('distance_filter_alpha', 0.6)  # Low-pass filter for distance (0-1), 1.0 = disabled
        self.velocity_filter_alpha = self.config.get('velocity_filter_alpha', 0.7)  # Low-pass filter for velocity (0-1), 1.0 = disabled
        self.leader_velocity_filter_alpha = self.config.get('leader_velocity_filter_alpha', 1.0)  # Low-pass filter for leader velocity (0-1), 1.0 = disabled
        self.leader_position_filter_alpha = self.config.get('leader_position_filter_alpha', 1.0)  # Low-pass filter for leader position (0-1), 1.0 = disabled
    def _init_recorder(self):
        """Initialize and start the debug data recorder."""
        try:
            from .distributed_luenberger_recorder import DistributedLuenbergerRecorder
            
            self.recorder = DistributedLuenbergerRecorder(
                output_dir=self.debug_output_dir,
                vehicle_id=self.vehicle_id,
                observer_size=self.observer_size,
                fleet_size=self.fleet_size
            )
            filepath = self.recorder.start()
            self._recording_start_time = 0.0
        except Exception as e:
            if self.logger:
                self.logger.log_error(f"Failed to initialize debug recorder", e)
            self.recorder = None
    
    def stop_recording(self):
        """Stop the debug data recorder and close the file."""
        if self.recorder is not None:
            self.recorder.stop()
            self.recorder = None
    
    def __del__(self):
        """Cleanup: stop recording when estimator is destroyed."""
        try:
            self.stop_recording()
        except Exception:
            pass  # Ignore errors during cleanup
    
    def _create_chain_topology(self) -> np.ndarray:
        """
        Create chain topology adjacency matrix (followers only, no leader)
        
        Matrix dimension: [observer_size x observer_size]
        Matrix indices: 0..observer_size-1 correspond to vehicle IDs 1..observer_size
        
        Connection rules:
        - Vehicle 1: connected to vehicle 2 (if exists)
        - Vehicle i (2 <= i <= observer_size-1): connected to vehicles i-1 and i+1
        - Vehicle observer_size: connected to vehicle observer_size-1
        
        Returns:
            adjacency_matrix: [observer_size x observer_size] adjacency matrix
        """
        adj = np.zeros((self.observer_size, self.observer_size))
        
        for i in range(self.observer_size):
            # Connect to previous follower (if not the first follower)
            if i > 0:
                adj[i, i-1] = 1
            # Connect to next follower (if not the last follower)
            if i < self.observer_size - 1:
                adj[i, i+1] = 1
        
        return adj
    
    def _create_fully_connected_topology(self) -> np.ndarray:
        """
        Create fully connected topology adjacency matrix (all followers communicate)
        
        Returns:
            adjacency_matrix: [observer_size x observer_size] adjacency matrix
        """
        adj = np.ones((self.observer_size, self.observer_size))
        # No self-loops
        np.fill_diagonal(adj, 0)
        return adj
    
    def get_neighbors(self, vehicle_id: int) -> List[int]:
        """
        Get neighbor vehicle IDs for a given follower vehicle
        
        Note: Adjacency matrix only includes followers (vehicles 1 to observer_size)
        Matrix index mapping: matrix index i = vehicle_id - 1
        
        Args:
            vehicle_id: Vehicle ID (1 to observer_size, i.e., followers)
        
        Returns:
            neighbors: List of neighbor vehicle IDs (also followers)
        """
        # Leader (vehicle 0) has no neighbors, or ID out of range
        if vehicle_id < 1 or vehicle_id > self.observer_size:
            return []
        
        # Convert vehicle ID to matrix index
        matrix_idx = vehicle_id - 1
        
        # Find all neighbors where adjacency_matrix[matrix_idx, neighbor_matrix_idx] > 0
        neighbors = []
        for j in range(self.observer_size):
            if self.adjacency_matrix[matrix_idx, j] > 0:
                # Convert matrix index back to vehicle ID
                neighbor_vehicle_id = j + 1
                neighbors.append(neighbor_vehicle_id)
        
        return neighbors
    
    def _sensor_filter(self, current_measurement: float, prev_measurement: float, 
                      filter_alpha: float, dt: float = None) -> float:
        """
        Generic low-pass filter for sensor measurements to suppress noise.
        Uses exponential moving average (EMA) for smoothing.
        
        Args:
            current_measurement: Current measurement from sensor
            prev_measurement: Previous filtered measurement
            filter_alpha: Filter coefficient (0-1)
                         - closer to 1 = more responsive (less filtering)
                         - closer to 0 = more smoothing (more filtering)
            dt: Time step (optional, not used in this simple implementation, 
                but can be used for adaptive filtering in future)
        
        Returns:
            filtered_measurement: Filtered measurement value
        """
        # Simple exponential moving average (low-pass filter)
        # filtered = alpha * current + (1 - alpha) * previous
        filtered_measurement = filter_alpha * current_measurement + (1 - filter_alpha) * prev_measurement
        
        return filtered_measurement
    
    def _get_leader_state_with_fallback(self, current_time_ns: int) -> tuple:
        """
        Retrieve leader state with fallback logic and apply sensor filtering.
        
        Args:
            current_time_ns: Current time in nanoseconds
        
        Returns:
            tuple: (v0, p0, v0_raw, state_leader)
                - v0: Filtered leader velocity
                - p0: Filtered leader position
                - v0_raw: Unfiltered leader velocity (for measurement equations)
                - state_leader: Full state array from V2V or None
        """
        state_leader = self._get_latest_received_state(0, current_time_ns)
        
        if state_leader is not None:
            v0_raw = state_leader[3]
            p0_raw = state_leader[0]
        else:
            if self.logger:
                self.logger.logger.warning(
                    f"Vehicle {self.vehicle_id}: No V2V state data from the leader, using fleet_states"
                )
            v0_raw = self.fleet_states[3, 0]
            p0_raw = self.fleet_states[0, 0]
        
        # Apply sensor filtering
        v0 = self._sensor_filter(v0_raw, self.prev_v0, self.leader_velocity_filter_alpha)
        self.prev_v0 = v0
        
        p0 = self._sensor_filter(p0_raw, self.prev_p0, self.leader_position_filter_alpha)
        self.prev_p0 = p0
        
        return v0, p0, v0_raw, state_leader
    
    def _compute_dynamics_term(self, x_vec: np.ndarray, collective_control: np.ndarray) -> np.ndarray:
        """
        Compute dynamics prediction term for observer update.
        
        Args:
            x_vec: Current observer state vector [3*observer_size]
            collective_control: Control input vector for all followers [observer_size]
        
        Returns:
            dynamics_term: Dynamics prediction [3*observer_size]
        """
        return self.A_delta @ x_vec + self.B_delta @ collective_control
    
    def _compute_measurement_term(self, x_vec: np.ndarray, local_state: np.ndarray, 
                                 current_time_ns: int, v0: float) -> tuple:
        """
        Compute measurement correction term for observer update.
        
        Args:
            x_vec: Current observer state vector [3*observer_size]
            local_state: Own vehicle state [x, y, theta, v, a]
            current_time_ns: Current time in nanoseconds
            v0: Filtered leader velocity
        
        Returns:
            tuple: (measurement_term, local_measurement, estimated_measurement, measurement_error)
        """
        measurement_term = np.zeros(3 * self.observer_size)
        local_measurement = np.zeros(self.local_measurement_dim)
        
        # Read self state from local_state input
        p_i = local_state[0]
        v_i = local_state[3]
        
        # Get preceding vehicle state from communication
        state_prev = self._get_latest_received_state(self.vehicle_id - 1, current_time_ns)
        if state_prev is not None:
            p_prev = state_prev[0]
        else:
            if self.logger:
                self.logger.logger.warning(
                    f"Vehicle {self.vehicle_id}: No V2V state data from vehicle {self.vehicle_id - 1}, using fleet_states"
                )
            p_prev = self.fleet_states[0, self.vehicle_id - 1]
        
        # CTH measurement: y1 = p_i - p_{i-1} - h*v_i (Constant Time Headway spacing error)
        # This matches the observer design in the paper (IFAC 2026)
        local_measurement[0] = p_i - p_prev 
        local_measurement[1] = v_i  # velocity (raw)
        
        # Apply low-pass filter to local measurements to suppress noise
        local_measurement[0] = self._sensor_filter(
            local_measurement[0], self.prev_distance_measurement, self.distance_filter_alpha
        )
        local_measurement[1] = self._sensor_filter(
            local_measurement[1], self.prev_velocity_measurement, self.velocity_filter_alpha
        )
        
        # Store filtered values for next iteration
        self.prev_distance_measurement = local_measurement[0]
        self.prev_velocity_measurement = local_measurement[1]
        
        # Compute estimated measurement
        estimated_measurement = self.Ci @ x_vec + self.Cv * v0 + self.Cd * self.d
        
        # Compute measurement error
        measurement_error = local_measurement - estimated_measurement
        
        # Apply observer gain
        measurement_term = self.observer_gain @ measurement_error  # Add a small bias to prevent stagnation
        
        return measurement_term, local_measurement, estimated_measurement, measurement_error
    
    def _calculate_estimated_collective_control(self, x_vec: np.ndarray) -> np.ndarray:
        """
        Calculate collective control input for all follower vehicles using K-matrix based feedback.
        
        Control law:
        - Vehicle 1: u1 = K10 @ F1 @ x_vec
        - Vehicle 2: u2 = K20 @ F2 @ x_vec + K21 @ (F2-F1) @ x_vec
        - Vehicle 3: u3 = K30 @ F3 @ x_vec + K31 @ (F3-F1) @ x_vec + K32 @ (F3-F2) @ x_vec
        
        Args:
            x_vec: Observer state vector [3*observer_size]
        
        Returns:
            collective_control: Control input for each follower vehicle [observer_size]
        """
        collective_control = np.zeros(self.observer_size)
        
        for vehicle_id in range(1, self.observer_size + 1):
            # Calculate Fi for current vehicle
            Fi = self.calculate_Fi(num_vehicles=self.observer_size, vehicle_index=vehicle_id)
            
            # Check if K matrices exist for this vehicle
            if vehicle_id in self.K_all_vehicles:
                # First term: Ki0 @ Fi @ x_vec
                if 0 in self.K_all_vehicles[vehicle_id]:
                    Ki0 = self.K_all_vehicles[vehicle_id][0]
                    collective_control[vehicle_id - 1] = (Ki0 @ (Fi @ x_vec))[0]
                
                # Sum over preceding vehicles j=1 to i-1
                # Add terms: Kij @ (Fi - Fj) @ x_vec
                for j in range(1, vehicle_id):
                    if j in self.K_all_vehicles[vehicle_id]:
                        Kij = self.K_all_vehicles[vehicle_id][j]
                        Fj = self.calculate_Fi(num_vehicles=self.observer_size, vehicle_index=j)
                        collective_control[vehicle_id - 1] += (Kij @ ((Fi - Fj) @ x_vec))[0]
        
        return collective_control
    
    def feedforward_throttle(self, vehicle_id: int, current_time_ns: int, local_state: np.ndarray = None) -> float:
        """
        Calculate feedforward throttle component based on vehicle parameters and current velocity.
        
        Args:
            vehicle_id: ID of the vehicle to get feedforward throttle for
            current_time_ns: Current time in nanoseconds
            local_state: Optional local state array [x, y, theta, v, a] for own vehicle
        
        Returns:
            feedforward: Feedforward throttle value to compensate for drag and rolling resistance
        """
        # For own vehicle, use local_state if provided, fallback to fleet_states
        if vehicle_id == self.vehicle_id:
            if local_state is not None:
                v_i = local_state[3]  # velocity is at index 3: [x, y, theta, v, a]
            else:
                v_i = self.fleet_states[3, vehicle_id]
                if self.logger:
                    self.logger.logger.warning(
                        f"Vehicle {self.vehicle_id}: No local state provided for feedforward throttle, using fleet_states"
                    )
        else:
            # For other vehicles, try V2V first, fallback to fleet_states
            vehicle_state = self._get_latest_received_state(vehicle_id, current_time_ns)
            if vehicle_state is not None:
                v_i = vehicle_state[3]  # velocity is at index 3: [x, y, theta, v, a]
            else:
                v_i = self.fleet_states[3, vehicle_id]  # fallback to fleet_states
                if self.logger:
                    self.logger.logger.warning(
                        f"Vehicle {self.vehicle_id}: No V2V state for vehicle {vehicle_id} to calculate feedforward throttle, using fleet_states"
                    )
        
        throttle_ff = 0.329609 * v_i**2 - 0.000272 * v_i + 0.038744
        
        return throttle_ff

    def _calculate_collective_control_v2v(self, control: np.ndarray, current_time_ns: int, local_state: np.ndarray = None) -> np.ndarray:
        """
        Calculate collective control input for all follower vehicles using V2V communication.
        
        Args:
            control: Own control signal [steering, throttle]
            current_time_ns: Current time in nanoseconds
            local_state: Optional local state array [x, y, theta, v, a] for own vehicle
        
        Returns:
            collective_control: Control input for each follower vehicle [observer_size]
        """
        collective_control = np.zeros(self.observer_size)
        
        for vehicle_id in range(1, self.fleet_size):
            follower_idx = vehicle_id - 1
            
            if follower_idx >= self.observer_size:
                break  # Safety check
            
            if vehicle_id == self.vehicle_id:
                # Use own control signal (full throttle including feedforward)
                collective_control[follower_idx] = control[1] - self.feedforward_throttle(vehicle_id, current_time_ns, local_state)  # Complete throttle signal
            else:
                # Get neighbor's control signal via V2V
                neighbor_control = self._get_latest_received_control(vehicle_id, current_time_ns)
                
                if neighbor_control is not None:
                    # Use complete control signal (feedforward + feedback)
                    collective_control[follower_idx] = neighbor_control[1] - self.feedforward_throttle(vehicle_id, current_time_ns)  # Complete throttle signal
                    
                    if self.logger and self.debug_recording_enabled:
                        self.logger.logger.debug(
                            f"Vehicle {self.vehicle_id}: Using V2V control from vehicle {vehicle_id}: "
                            f"throttle={neighbor_control[1]:.3f}, steering={neighbor_control[0]:.3f}"
                        )
                else:
                    # No V2V data: use default value (zero throttle)
                    collective_control[follower_idx] = 0.0
                    
                    if self.logger:
                        self.logger.logger.warning(
                            f"Vehicle {self.vehicle_id}: No V2V control data from vehicle {vehicle_id}, using default"
                        )
        
        return collective_control
    
    
    def _compute_consensus_term(self, x_vec: np.ndarray, current_time_ns: int) -> np.ndarray:
        """
        Compute consensus correction term based on neighbor states.
        
        IMPROVED: Prioritizes directly received observer states (x_vec) from neighbors
        to avoid di0 conversion errors. Falls back to fleet_states conversion if
        direct observer states are not available.
        
        Args:
            x_vec: Current observer state vector [3*observer_size]
            current_time_ns: Current time in nanoseconds
        
        Returns:
            consensus_term: Consensus correction [3*observer_size]
        """
        dim_distributed_observer = 3 * self.observer_size
        consensus_term = np.zeros(dim_distributed_observer)
        consensus_accum = np.zeros(dim_distributed_observer)
        neighbor_count = 0
        
        if self.logger and self.debug_recording_enabled:
            self.logger.logger.debug(
                f"Vehicle {self.vehicle_id}: Starting consensus calculation. "
                f"My neighbors: {self.my_neighbors}, x_vec norm: {np.linalg.norm(x_vec):.6f}"
            )
        
        # Loop through each neighbor defined by adjacency matrix
        for neighbor_id in self.my_neighbors:
            neighbor_x_vec = None
            source = "none"
            
            # --- Step 1: Try to get DIRECT observer state (Primary Source - No conversion needed) ---
            neighbor_x_vec = self._get_latest_observer_state(neighbor_id, current_time_ns)
            
            if neighbor_x_vec is not None:
                # Validate dimension
                if len(neighbor_x_vec) == dim_distributed_observer:
                    source = "direct_observer"
                    if self.logger and self.debug_recording_enabled:
                        self.logger.logger.debug(
                            f"Vehicle {self.vehicle_id}: Using DIRECT observer state from neighbor {neighbor_id}"
                        )
                else:
                    # Dimension mismatch, discard and try fallback
                    neighbor_x_vec = None
                    if self.logger:
                        self.logger.logger.warning(
                            f"Vehicle {self.vehicle_id}: Observer state dimension mismatch from neighbor {neighbor_id}. "
                            f"Expected {dim_distributed_observer}, got {len(neighbor_x_vec)}. Falling back."
                        )
            
            # --- Step 2: Fallback to FLEET states conversion if direct observer state not available ---
            if neighbor_x_vec is None:
                neighbor_fleet_dict = self._get_latest_fleet_data(neighbor_id, current_time_ns)
                
                # Validate fleet data completeness
                is_complete_fleet = False
                missing_vehicles = []
                
                if neighbor_fleet_dict is not None:
                    expected_vehicles = set(range(self.fleet_size))
                    received_vehicles = set(int(vid) for vid in neighbor_fleet_dict.keys())
                    missing_vehicles = list(expected_vehicles - received_vehicles)
                    
                    if not missing_vehicles:
                        is_complete_fleet = True
                
                # Fill missing vehicles with local state broadcasts or current estimates
                if not is_complete_fleet:
                    if neighbor_fleet_dict is None:
                        neighbor_fleet_dict = {}
                    
                    for vid in missing_vehicles:
                        vehicle_local_state = self._get_latest_received_state(vid, current_time_ns)
                        
                        if vehicle_local_state is not None:
                            neighbor_fleet_dict[vid] = {
                                'x': float(vehicle_local_state[0]),
                                'y': float(vehicle_local_state[1]),
                                'theta': float(vehicle_local_state[2]),
                                'velocity': float(vehicle_local_state[3]),
                                'acceleration': float(vehicle_local_state[4] if len(vehicle_local_state) > 4 else 0.0)
                            }
                        else:
                            neighbor_fleet_dict[vid] = {
                                'x': float(self.fleet_states[0, vid]),
                                'y': float(self.fleet_states[1, vid]),
                                'theta': float(self.fleet_states[2, vid]),
                                'velocity': float(self.fleet_states[3, vid]),
                                'acceleration': float(self.fleet_states[4, vid])
                            }
                        if self.logger:
                            self.logger.logger.warning(
                                f"Vehicle {self.vehicle_id}: Missing state for vehicle {vid} from neighbor {neighbor_id}. "
                                f"Falling back to local state or estimate. Missing vehicles: {missing_vehicles}"
                            )
                
                # Build neighbor's complete fleet_states matrix
                if neighbor_fleet_dict:
                    neighbor_fleet_states = np.zeros((self.state_dim, self.fleet_size))
                    
                    for vid, vehicle_state in neighbor_fleet_dict.items():
                        try:
                            vid_int = int(vid)
                            if 0 <= vid_int < self.fleet_size:
                                neighbor_fleet_states[0, vid_int] = vehicle_state.get('x', 0.0)
                                neighbor_fleet_states[1, vid_int] = vehicle_state.get('y', 0.0)
                                neighbor_fleet_states[2, vid_int] = vehicle_state.get('theta', 0.0)
                                neighbor_fleet_states[3, vid_int] = vehicle_state.get('velocity', vehicle_state.get('v', 0.0))
                                neighbor_fleet_states[4, vid_int] = vehicle_state.get('acceleration', 0.0)
                            
                        except (ValueError, TypeError) as e:
                            if self.logger:
                                self.logger.logger.error(
                                    f"Vehicle {self.vehicle_id}: Invalid vehicle_id {vid} from neighbor {neighbor_id}: {e}"
                                )
                            continue
                    
                    # Convert to distributed observer state format (introduces di0 error!)
                    neighbor_x_vec, neighbor_di0_values = self._transfer_fleet_states_to_estimated_states(
                        neighbor_fleet_states, current_time_ns
                    )
                    source = "fleet_conversion"
            
            # --- Step 3: Calculate consensus difference with adjacency weight ---
            if neighbor_x_vec is not None:
                my_matrix_idx = self.vehicle_id - 1
                neighbor_matrix_idx = neighbor_id - 1
                weight = self.adjacency_matrix[my_matrix_idx, neighbor_matrix_idx]
                
                # Accumulate weighted difference: (own_estimate - neighbor_estimate)
                consensus_diff = x_vec - neighbor_x_vec
                consensus_accum += weight * consensus_diff
                neighbor_count += 1
                
                if self.logger and self.debug_recording_enabled:
                    self.logger.logger.debug(
                        f"Vehicle {self.vehicle_id}: Neighbor {neighbor_id} (source={source}) - "
                        f"weight={weight:.6f}, diff_norm={np.linalg.norm(consensus_diff):.6f}, "
                        f"neighbor_x_norm={np.linalg.norm(neighbor_x_vec):.6f}, "
                        f"accum_norm={np.linalg.norm(consensus_accum):.6f}"
                    )
        
        # --- Step 4: Apply consensus gain ---
        if neighbor_count > 0:
            consensus_term = self.consensus_gain @ consensus_accum 
            # Numerical protection: prevent consensus term explosion
            consensus_norm = np.linalg.norm(consensus_term)
            max_consensus_threshold = 50.0
            if consensus_norm > max_consensus_threshold:
                consensus_term = consensus_term / consensus_norm * max_consensus_threshold
            
            if self.logger and self.debug_recording_enabled:
                self.logger.logger.debug(
                    f"Vehicle {self.vehicle_id}: Consensus applied. "
                    f"accum_norm={np.linalg.norm(consensus_accum):.6f}, "
                    f"gain shape={self.consensus_gain.shape}, "
                    f"term_norm={consensus_norm:.6f}"
                )
        else:
            consensus_term = np.zeros(dim_distributed_observer)
            if self.logger and self.debug_recording_enabled:
                self.logger.logger.debug(
                    f"Vehicle {self.vehicle_id}: No neighbors found for consensus"
                )
        
        return consensus_term

    def compute_output_matrix_Ci(self, i: int) -> np.ndarray:
        """
        Compute output matrix Ci, where i starts from 1
        
        For vehicle i in the fleet, the output matrix structure is:
        - C1 = [Cf, 0, 0]     
        - C2 = [Cp, Cf, 0]     
        - C3 = [0, Cp, Cf]  
        
        Pattern: Ci has Cf at position i, Cp at position i-1 (if i>1), zeros elsewhere
        
        Args:
            i: Vehicle index, starting from 1 (1 <= i <= fleet_size)
        
        Returns:
            Ci: Output matrix, shape [2, 3*observer_size]
        """
        if i < 1 or i > self.observer_size:
            raise ValueError(f"Vehicle index i={i} out of range [1, {self.observer_size}]")
        
        measurement_dim = self.Cf.shape[0]  # 2
        state_block_dim = self.Cf.shape[1]  # 3
        
        # Create zero matrix block
        zero_block = np.zeros((measurement_dim, state_block_dim))
        
        # Build Ci matrix
        blocks = []
        for j in range(1, self.observer_size + 1):
            if j == i - 1 and i > 1:
                # Position i-1: observe relationship with preceding vehicle
                blocks.append(self.Cp)
            elif j == i:
                # Position i: observe self
                blocks.append(self.Cf)
            else:
                # Other positions: zeros
                blocks.append(zero_block)
        
        # Horizontal concatenation of all blocks
        Ci = np.hstack(blocks)
        
        return Ci
    
    def _load_extra_config(self):
        """
        Load observer and consensus gains from extra YAML config files based on vehicle ID.
        """
        # Default initialization from main config or defaults
        self.observer_gain = self.config.get('observer_gain', 0.1)
        self.consensus_gain = self.config.get('consensus_gain', 0.2)
        
        # Store config directory for later use by K matrix loading
        self._extra_config_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'extra_configs')
        
        # Try to load specific parameters from extra_configs file
        try:
            # extra_configs is in the parent directory (Observer/)
            config_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'extra_configs')
            config_file = os.path.join(config_dir, f'car{self.vehicle_id}.yaml')
            
            if os.path.exists(config_file):
                with open(config_file, 'r') as f:
                    extra_conf = yaml.safe_load(f)
                    
                if extra_conf and 'observer' in extra_conf:
                    obs_conf = extra_conf['observer']
                    
                    # Check if we should use ConsensusFleetEstimator logic instead
                    if obs_conf.get('fleet_estimator_type') == 'consensus':
                        self.consensus_estimator = ConsensusFleetEstimator(
                            self.vehicle_id, self.fleet_size, self.state_dim, 
                            config=obs_conf, logger=self.logger
                        )
                        # Sync state dictionaries to ensure they use the same data
                        self.consensus_estimator.received_local_states = self.received_local_states
                        self.consensus_estimator.received_fleet_states = self.received_fleet_states
                    
                    if 'observer_gain' in obs_conf:
                        self.observer_gain = np.array(obs_conf['observer_gain'])
                            
                    if 'consensus_gain' in obs_conf:
                        self.consensus_gain = np.array(obs_conf['consensus_gain'])

                    # Load vehicle-specific parameters from extra config if available
                    for param in ['m_i', 'tau_i', 'rho_i', 'Cd_i', 'AF_i', 'mu_i']:
                        if param in obs_conf:
                            setattr(self, param, obs_conf[param])
            else:
                pass
        except Exception as e:
            if self.logger:
                self.logger.log_error(f"Error loading extra config for vehicle {self.vehicle_id}", e)
    
    def _load_K_matrices_from_yaml(self) -> Dict:
        """
        Load K matrices for all follower vehicles from their respective yaml config files.
        
        Each carX.yaml should contain a 'K_matrices' section under 'observer':
            observer:
                K_matrices:
                    0: [[-0.3105, -0.5413, 0.0062]]   # K_i0
                    1: [[-0.0481, -0.0799, 0.0417]]   # K_i1 (for vehicle >= 2)
        
        Returns:
            K_all_vehicles: Dict[int, Dict[int, np.ndarray]] mapping vehicle_id -> {j: K_ij}
        """
        K_all_vehicles = {}
        
        # Default fallback K matrices (used if yaml loading fails)
        default_K_matrices = {
            1: {
                0: np.array([[-0.3105, -0.5413, 0.0062]])
            },
            2: {
                0: np.array([[-0.3203, -0.4258, -0.0517]]),
                1: np.array([[-0.0481, -0.0799, 0.0417]])
            },
            3: {
                0: np.array([[-0.3555, -0.4238, -0.0850]]),
                1: np.array([[-0.0351, -0.0553, 0.0272]]),
                2: np.array([[-0.0336, -0.0528, 0.0339]])
            }
        }
        
        # Get config directory (set in _load_extra_config or default)
        config_dir = getattr(self, '_extra_config_dir', 
                            os.path.join(os.path.dirname(os.path.dirname(__file__)), 'extra_configs'))
        
        # Load K matrices for each follower vehicle (1 to observer_size)
        for vehicle_id in range(1, self.observer_size + 1):
            config_file = os.path.join(config_dir, f'car{vehicle_id}.yaml')
            
            try:
                if os.path.exists(config_file):
                    with open(config_file, 'r') as f:
                        config = yaml.safe_load(f)
                    
                    if config and 'observer' in config and 'K_matrices' in config['observer']:
                        K_matrices_raw = config['observer']['K_matrices']
                        K_all_vehicles[vehicle_id] = {}
                        
                        for j_str, K_values in K_matrices_raw.items():
                            j = int(j_str)  # Convert string key to int
                            K_all_vehicles[vehicle_id][j] = np.array(K_values)
                        
                        if self.logger:
                            self.logger.logger.info(
                                f"Loaded K matrices for vehicle {vehicle_id} from {config_file}: "
                                f"keys={list(K_all_vehicles[vehicle_id].keys())}"
                            )
                    else:
                        # No K_matrices in config, use default
                        if vehicle_id in default_K_matrices:
                            K_all_vehicles[vehicle_id] = default_K_matrices[vehicle_id]
                            if self.logger:
                                self.logger.logger.warning(
                                    f"No K_matrices found in {config_file}, using default for vehicle {vehicle_id}"
                                )
                else:
                    # Config file not found, use default
                    if vehicle_id in default_K_matrices:
                        K_all_vehicles[vehicle_id] = default_K_matrices[vehicle_id]
                        if self.logger:
                            self.logger.logger.warning(
                                f"Config file {config_file} not found, using default K matrices for vehicle {vehicle_id}"
                            )
            except Exception as e:
                # Error loading, use default
                if vehicle_id in default_K_matrices:
                    K_all_vehicles[vehicle_id] = default_K_matrices[vehicle_id]
                if self.logger:
                    self.logger.log_error(
                        f"Error loading K matrices for vehicle {vehicle_id} from {config_file}", e
                    )
        
        return K_all_vehicles
    
    def update(self, local_state: np.ndarray, dt: float, 
               current_time_ns: int, control: np.ndarray) -> np.ndarray:
        """Update using distributed observer with dynamics
        
        Args:
            local_state: Own vehicle state [x, y, theta, v, a] - used ONLY for local measurement calculation
            dt: Time step in seconds
            current_time_ns: Current time in nanoseconds
            control: Control input [steering, throttle]
        
        Note:
            local_state is NOT stored in fleet_states. It is used ONLY for measurement calculation.
            fleet_states contains ONLY distributed observer estimates for all vehicles.
        """
        # If we are in consensus mode, delegate to ConsensusFleetEstimator
        if self.consensus_estimator:
            self.fleet_states = self.consensus_estimator.update(local_state, dt, current_time_ns, control)
            return self.fleet_states

        try:
            # Ensure we have capacity
            self._ensure_fleet_capacity(self.vehicle_id)
            
            # Distributed observer for this vehicle
            self.estimated_state = self._distributed_luenberger_observer_update(
                local_state, current_time_ns, control, dt
            )
            

            # self.estimated_state = self._fake_estimated_state_for_debugging(
            #     self.estimated_state, local_state, current_time_ns
            # )
            
            # Transfer the estimated states back to fleet_states. 
            self.fleet_states, _ = self._transfer_estimated_states_to_fleet_states(self.estimated_state, local_state, current_time_ns)
            # Cleanup old data
            self._cleanup_old_data(current_time_ns)
            
            # === Record debug data if enabled ===
            if self.recorder is not None:
                # Calculate time in seconds from start
                if self._recording_start_time is None:
                    self._recording_start_time = current_time_ns / 1e9
                t = current_time_ns / 1e9 - self._recording_start_time
                
                self.recorder.record(t, self.debug_data)
                self._update_count += 1
            
            return self.fleet_states
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Distributed Luenberger update error", e)
            return self.fleet_states
    
    def _fake_estimated_state_for_debugging(self, estimated_state: np.ndarray, local_state: np.ndarray, 
                                           current_time_ns: int) -> np.ndarray:
        """
        Calculate fake estimated state using true vehicle states from V2V communication for debugging.
        This is used to compare observer estimates with ground truth.
        
        Format (column-major):
        - estimated_state[0] = p1 - p0 + d10
        - estimated_state[1] = v1 - v0
        - estimated_state[2] = a1 - a0
        - estimated_state[3] = p2 - p0 + d20
        - estimated_state[4] = v2 - v0
        - estimated_state[5] = a2 - a0
        - ... and so on for all follower vehicles
        
        Args:
            estimated_state: Current observer estimated state (not used, just for signature consistency)
            local_state: Own vehicle state [x, y, theta, v, a]
            current_time_ns: Current timestamp in nanoseconds
        
        Returns:
            fake_estimated_state: "Ground truth" estimated state vector [3*observer_size]
        """
        # Get leader (vehicle 0) state from V2V communication
        state_leader = self._get_latest_received_state(0, current_time_ns)
        if state_leader is not None:
            p0 = state_leader[0]
            v0 = state_leader[3]
            a0 = state_leader[4] 
        else:
            # Fallback to fleet_states if no V2V data available
            p0 = self.fleet_states[0, 0]
            v0 = self.fleet_states[3, 0]
            a0 = self.fleet_states[4, 0]
        
        # Initialize fake estimated state matrix [3 x observer_size]
        fake_estimated_state_mat = np.zeros((3, self.observer_size))
        
        # For each follower vehicle (vehicle_id >= 1), compute relative state from true states
        for vehicle_id in range(1, self.observer_size + 1):
            col_idx = vehicle_id - 1
            
            # Get true state from V2V communication or local state
            if vehicle_id == self.vehicle_id:
                # Use own local state
                pi = local_state[0]
                vi = local_state[3]
                ai = local_state[4] 
            else:
                # Get state from V2V communication
                state_i = self._get_latest_received_state(vehicle_id, current_time_ns)
                if state_i is not None:
                    pi = state_i[0]
                    vi = state_i[3]
                    ai = state_i[4] if len(state_i) > 4 else 0.0
                else:
                    # No V2V data available, use fleet_states estimate
                    pi = self.fleet_states[0, vehicle_id]
                    vi = self.fleet_states[3, vehicle_id]
                    ai = self.fleet_states[4, vehicle_id]
            
            # Calculate di0 = vehicle_id * d + h * sum(v_k for k=1 to vehicle_id)
            di0 = vehicle_id * self.d
            
            # Sum true absolute velocities from vehicle 1 to vehicle_id
            velocity_sum = 0.0
            for k in range(1, vehicle_id + 1):
                if k == self.vehicle_id:
                    # Use own velocity
                    vk = local_state[3]
                else:
                    # Get from V2V communication
                    state_k = self._get_latest_received_state(k, current_time_ns)
                    if state_k is not None:
                        vk = state_k[3]
                    else:
                        # Fallback to fleet_states
                        vk = self.fleet_states[3, k]
                
                velocity_sum += vk
            
            di0 += self.h * velocity_sum
            
            # Calculate relative state (ground truth)
            relative_position = pi - p0 + di0
            relative_velocity = vi - v0
            relative_accel = ai - a0
            
            # Store in fake estimated matrix
            fake_estimated_state_mat[0, col_idx] = relative_position
            fake_estimated_state_mat[1, col_idx] = relative_velocity
            fake_estimated_state_mat[2, col_idx] = relative_accel
        
        # Flatten matrix to vector (column-major order)
        fake_estimated_state = fake_estimated_state_mat.flatten(order="F")
        
        return fake_estimated_state
    
    def _transfer_fleet_states_to_estimated_states(self, fleet_states: np.ndarray, current_time_ns: int) -> np.ndarray:
        """
        Convert complete fleet state matrix to distributed observer estimated state format
        
        Converts absolute states to relative states:
        - Position estimate: pi - p0 + di0
        - Velocity estimate: vi - v0
        - Acceleration estimate: ai - a0
        
        Where di0 = vehicle_id * d + h * sum(v_k for k=1 to vehicle_id)
        
        Args:
            fleet_states: Complete fleet state matrix [state_dim x fleet_size]
            current_time_ns: Current timestamp in nanoseconds
            local_state: Optional local state [x, y, theta, v, a] for this vehicle. 
                        If provided, will be used when k == self.vehicle_id instead of fleet_states

        Returns:
            estimated_state: Distributed observer state vector [3*observer_size]
            di0_values: Array of di0 values for each follower vehicle [observer_size]
        """

        # Use the passed-in fleet_states (not always self.fleet_states) to support neighbor state conversion
        # fleet_states = self.fleet_states  # ❌ This was overriding the parameter!
        
        # Get leader (vehicle 0) absolute state
        state_leader = self._get_latest_received_state(0, current_time_ns)
        if state_leader is not None:
            p0 = state_leader[0]
            v0 = state_leader[3]
            a0 = state_leader[4]
        else:
            p0 = fleet_states[0, 0]
            v0 = fleet_states[3, 0]
            a0 = fleet_states[4, 0]
        
        # Initialize estimated state matrix [3 x observer_size]
        estimated_state_mat = np.zeros((3, self.observer_size))
        di0_values = np.zeros(self.observer_size)  # Store di0 for each follower
        
        # For each follower vehicle (vehicle_id >= 1) compute relative state
        for vehicle_id in range(1, min(self.fleet_size, self.observer_size + 1)):
            col_idx = vehicle_id - 1
            
            # Get absolute state from fleet states
            pi = fleet_states[0, vehicle_id]
            vi = fleet_states[3, vehicle_id]
            ai = fleet_states[4, vehicle_id]
            
            # Calculate di0
            di0 = vehicle_id * self.d
            
            # Sum absolute velocities from vehicle 1 to vehicle_id (use fleet_states)
            velocity_sum = 0.0
            for k in range(1, vehicle_id + 1):
                vk = fleet_states[3, k]
                velocity_sum += vk
            
            di0 += self.h * velocity_sum
            di0_values[col_idx] = di0  # Store di0 value
            
            # Calculate relative state
            relative_position = pi - p0 + di0
            relative_velocity = vi - v0
            relative_accel = ai - a0
            
            # Store in estimated matrix
            estimated_state_mat[0, col_idx] = relative_position
            estimated_state_mat[1, col_idx] = relative_velocity
            estimated_state_mat[2, col_idx] = relative_accel
        
        # Flatten matrix to vector (column-major order)
        estimated_state = estimated_state_mat.flatten(order="F")
        
        return estimated_state, di0_values
    
    def _transfer_estimated_states_to_fleet_states(self, estimated_state: np.ndarray, local_state: np.ndarray, current_time_ns: int) -> np.ndarray:
        """
        Convert distributed observer estimated state to complete fleet state matrix
        
        Distributed observer estimates relative states:
        - Position estimate: pi - p0 + di0
        - Velocity estimate: vi - v0
        - Acceleration estimate: ai - a0
        
        Need to compute absolute states:
        - vi = estimate_i_v + v0
        - ai = estimate_i_a + a0
        - pi = estimate_i_p + p0 - di0
        - Where di0 = vehicle_id * d + h * sum(hat_v_k for k=1 to vehicle_id)
        -hat_v_k = sum (estimate_k_v + v0) for k=1 to vehicle_id
        
        Args:
            estimated_state: Distributed observer state vector [3*observer_size]
            local_state: Current vehicle's local state [x, y, theta, v, a]
            current_time_ns: Current timestamp (nanoseconds)

        Returns:
            fleet_states: Complete fleet state matrix [state_dim x fleet_size]
        """
        
        # Reshape estimated state to [3 x observer_size] matrix (column-major)
        estimated_state_mat = estimated_state.reshape((3, self.observer_size), order="F")
        
        # Initialize output matrix, preserving y and theta information
        fleet_states_new = self.fleet_states.copy()
        di0_values = np.zeros(self.observer_size)  # Store di0 for each follower
        
        # Get leader (vehicle 0) absolute state
        state_leader = self._get_latest_received_state(0, current_time_ns)
        if state_leader is not None:
            p0 = state_leader[0]
            v0 = state_leader[3]
            a0 = state_leader[4]
        else:
            p0 = self.fleet_states[0, 0]
            v0 = self.fleet_states[3, 0]
            a0 = self.fleet_states[4, 0]

        # Leader state unchanged
        fleet_states_new[:, 0] = self.fleet_states[:, 0]
        
        # For each follower vehicle compute absolute state
        for vehicle_id in range(1, self.fleet_size):
            col_idx = vehicle_id - 1
            if col_idx >= self.observer_size:
                break

            # Extract relative state from estimated matrix
            relative_position = estimated_state_mat[0, col_idx]
            relative_velocity = estimated_state_mat[1, col_idx]
            relative_accel = estimated_state_mat[2, col_idx]
            
            # Calculate di0
            # di0 = vehicle_id * d + h * sum(hat_v_k for k=1 to vehicle_id)
            # where hat_v_k = estimate_k_v + v0
            di0 = vehicle_id * self.d
            
            # Sum estimated absolute velocities: hat_v_k = estimate_k_v + v0
            velocity_sum = 0.0
            for k in range(1, vehicle_id + 1):
                k_idx = k - 1  # Convert vehicle_id to matrix index
                if k_idx < self.observer_size:
                    # Get estimated relative velocity from the estimated state matrix
                    estimate_k_v = estimated_state_mat[1, k_idx]  # Relative velocity: vk - v0
                    hat_v_k = estimate_k_v + v0  # Convert to absolute velocity
                    velocity_sum += hat_v_k
            
            di0 += self.h * velocity_sum
            di0_values[col_idx] = di0  # Store di0 value
            
            # Calculate absolute state
            pi = relative_position + p0 - di0
            vi = relative_velocity + v0
            ai = relative_accel + a0
            
            # Update fleet state matrix
            fleet_states_new[0, vehicle_id] = pi
            fleet_states_new[3, vehicle_id] = vi
            fleet_states_new[4, vehicle_id] = ai
            
            # y and theta preserved
            fleet_states_new[1, vehicle_id] = self.fleet_states[1, vehicle_id]
            fleet_states_new[2, vehicle_id] = self.fleet_states[2, vehicle_id]
        
        return fleet_states_new, di0_values

    def _distributed_luenberger_observer_update(self, local_state: np.ndarray, current_time_ns: int,
                                     control: np.ndarray, dt: float) -> np.ndarray:
        """
        Distributed observer update for one target vehicle
        Combines dynamics prediction, measurement correction, and consensus
        Args:
            local_state: Own vehicle state [x, y, theta, v, a] - used ONLY for local measurement calculation
            current_time_ns: Current time in nanoseconds
            control: Control input [steering, throttle]
            dt: Time step in seconds
        """
        # Distributed observer dimension (3: pi-p0+di0, vi-v0, ai-a0)
        longitudinal_state_dim = 3
        dim_distributed_observer = longitudinal_state_dim * self.observer_size

        # Read current observer state directly without conversion
        x_vec = self.estimated_state.copy()

        
        # Get leader state with fallback and filtering
        v0, p0, v0_raw, state_leader = self._get_leader_state_with_fallback(current_time_ns)

        # Calculate collective control input using K-matrix feedback
        # Uncomment to use K-matrix based control, otherwise use V2V communication
        collective_control = self._calculate_estimated_collective_control(x_vec)
        
        # Get actual control signals from V2V communication for all follower vehicles
        # collective_control = self._calculate_collective_control_v2v(control, current_time_ns, local_state)

        # 1. Compute dynamics prediction term
        dynamics_term = self._compute_dynamics_term(x_vec, collective_control)
        
        # 2. Compute measurement correction term
        measurement_term, local_measurement, estimated_measurement, measurement_error = \
            self._compute_measurement_term(x_vec, local_state, current_time_ns, v0)
        
        # 3. Compute consensus correction term
        consensus_term = self._compute_consensus_term(x_vec, current_time_ns)
        neighbor_count = len(self.my_neighbors)  # For debug data recording
              
        x_i_new = x_vec + dt * (dynamics_term + measurement_term - consensus_term)
        
        # State constraint: prevent numerical overflow
        x_i_new = np.clip(x_i_new, -1e4, 1e4)

        # x_i_new = self._fake_estimated_state_for_debugging(
        #         self.estimated_state, local_state, current_time_ns
        #     )

        
        # === Store debug data for recording (ALWAYS, not just when debug_recording_enabled) ===
        # Get leader (vehicle 0) state for recording
        leader_x = p0  # Already retrieved earlier in the function
        leader_v = v0  # Already retrieved earlier in the function
        leader_a = state_leader[4] if state_leader is not None and len(state_leader) > 4 else self.fleet_states[4, 0]
        
        self.debug_data = {
            'x_vec': x_vec.copy(),  # Observer state (before update)
            'dynamics_term': dynamics_term.copy(),  # Dynamics prediction term
            'measurement_term': measurement_term.copy(),  # Measurement correction term
            'consensus_term': consensus_term.copy(),  # Consensus correction term
            'local_measurement': local_measurement.copy(),  # Local measurement vector
            'estimated_measurement': estimated_measurement.copy(),  # Estimated measurement vector
            'measurement_error': measurement_error.copy(),  # Measurement error vector
            'neighbor_count': neighbor_count,  # Number of neighbors used
            'consensus_norm': np.linalg.norm(consensus_term) if neighbor_count > 0 else 0.0,  # Norm of consensus term
            'fleet_states': self.fleet_states.copy(),  # Current fleet states
            'collective_control': collective_control.copy(),  # Control input for each follower
            # True throttle input for this vehicle
            'control_input': control[1],  # Throttle input
            'local_measurement_p': local_measurement[0],  # Local relative position measurement
            'local_measurement_v': local_measurement[1],  # Local velocity measurement
            'dt': dt,  # Time step
            # Position, velocity, acceleration for local vehicle
            'position': local_state[0],
            'velocity': local_state[3],
            'acceleration': local_state[4] if len(local_state) > 4 else 0.0,
        }
        
        # Record true states of all vehicles via V2V communication
        # This ensures all data is synchronized to current vehicle's timestamp
        
        # Record leader (vehicle 0) state
        self.debug_data['true_position_0'] = leader_x  # Leader position (x)
        self.debug_data['true_velocity_0'] = leader_v  # Leader velocity
        self.debug_data['true_acceleration_0'] = leader_a  # Leader acceleration
        
        # Record follower vehicles' true states
        for vehicle_id in range(1, self.fleet_size):
            if vehicle_id == self.vehicle_id:
                # Own state already recorded above as position, velocity, acceleration
                self.debug_data[f'true_position_{vehicle_id}'] = local_state[0]
                self.debug_data[f'true_velocity_{vehicle_id}'] = local_state[3]
                self.debug_data[f'true_acceleration_{vehicle_id}'] = local_state[4] if len(local_state) > 4 else 0.0
                continue
            
            # Get latest received state from V2V communication
            other_state = self._get_latest_received_state(vehicle_id, current_time_ns)
            
            if other_state is not None:
                self.debug_data[f'true_position_{vehicle_id}'] = other_state[0]
                self.debug_data[f'true_velocity_{vehicle_id}'] = other_state[3]
                self.debug_data[f'true_acceleration_{vehicle_id}'] = other_state[4] if len(other_state) > 4 else 0.0
            else:
                # No V2V data available, use NaN to indicate missing data
                self.debug_data[f'true_position_{vehicle_id}'] = np.nan
                self.debug_data[f'true_velocity_{vehicle_id}'] = np.nan
                self.debug_data[f'true_acceleration_{vehicle_id}'] = np.nan
        
        return x_i_new

    def _get_nonlinear_term_phi_i(self, v_i, a_i, g_s: float = 9.81):
        """
        Compute the nonlinear term phi_i(v_i, a_i) using vehicle parameters on this estimator.

        Args:
            v_i: Velocity (scalar or numpy array)
            a_i: Acceleration (scalar or numpy array)
            g_s: Gravitational acceleration constant
        """
        v_i = np.asarray(v_i)
        a_i = np.asarray(a_i)

        try:
            m_i = self.m_i # mass of vehicle i
            tau_i = self.tau_i # engine time constant
            rho_i = self.rho_i # air density
            Cd_i = self.Cd_i # drag coefficient
            AF_i = self.AF_i # frontal area
            mu_i = self.mu_i   # rolling resistance coefficient
        except AttributeError as exc:
            raise AttributeError(
                "Vehicle parameters (m_i, tau_i, rho_i, Cd_i, AF_i, mu_i) must be set on the estimator before calling nonlear_term_phi_i"
            ) from exc

        # Aerodynamic drag term
        drag_term = -(rho_i * Cd_i * AF_i / (2.0 * m_i * tau_i)) * (v_i**2 + 2.0 * tau_i * v_i * a_i)

        # Acceleration damping term
        accel_term = -(1.0 / tau_i) * a_i

        # Grade/rolling resistance term
        grade_term = (1.0 / tau_i) * mu_i * g_s

        return drag_term + accel_term + grade_term
    
    def get_debug_data(self) -> Dict:
        """Return the latest debug data dictionary for recording."""
        return self.debug_data
    
    def enable_debug_recording(self, enable: bool = True):
        """Enable or disable debug data recording."""
        self.debug_recording_enabled = enable

    def calculate_Fi(self, num_vehicles: int, vehicle_index: int) -> np.ndarray:
        """
        Calculate the index matrix Fi for vehicle i in a platoon of num_vehicles
        
        Args:
            num_vehicles: Total number of vehicles in the platoon, not including the leader
            vehicle_index: Index of the current vehicle (1-based)
        
        Returns:
            Fi: Index matrix as a numpy array of shape (n0, n) where n0=3, n=3*num_vehicles
            The matrix places a 3x3 identity matrix at the i-th block position, zeros elsewhere
        """
        n0 = 3
        n = n0 * num_vehicles
        Fi = np.zeros((n0, n))
        
        # Place n0 x n0 identity matrix at the i-th block position
        block_start = n0 * (vehicle_index - 1)
        Fi[:, block_start:block_start + n0] = np.eye(n0)
        
        return Fi
    
    def get_observer_state(self) -> np.ndarray:
        """
        Get the current observer state vector for V2V broadcasting.
        
        This method returns the raw observer state (x_vec) which can be
        directly shared with neighbors for consensus calculation, avoiding
        di0 conversion errors.
        
        Returns:
            observer_state: Observer state vector [3*observer_size] containing:
                           [p1-p0+d10, v1-v0, a1-a0, p2-p0+d20, v2-v0, a2-a0, ...]
                           Returns zeros if observer state is not initialized (e.g., leader vehicle)
        """
        if not hasattr(self, 'estimated_state') or self.estimated_state is None:
            # For leader vehicle or when consensus_estimator is used, return zeros
            observer_size = getattr(self, 'observer_size', self.fleet_size - 1)
            return np.zeros(3 * observer_size)
        return self.estimated_state.copy()
    
    def get_observer_state_for_broadcast(self) -> Dict:
        """
        Get observer state formatted for V2V broadcast message.
        
        Returns:
            dict: Message-ready dictionary with observer state data
        """
        observer_size = getattr(self, 'observer_size', self.fleet_size - 1)
        
        if not hasattr(self, 'estimated_state') or self.estimated_state is None:
            # For leader vehicle or when consensus_estimator is used, return zeros
            observer_state_list = [0.0] * (3 * observer_size)
        else:
            observer_state_list = self.estimated_state.tolist()
        
        return {
            'vehicle_id': self.vehicle_id,
            'observer_state': observer_state_list,
            'observer_size': observer_size,
            'timestamp': None  # Will be filled by caller
        }