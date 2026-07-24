"""
Distributed HG Observer for Fleet State Estimation

This module provides a distributed HG observer implementation
for fleet longitudinal state estimation with consensus-based communication.

Moved from fleet_state_estimators.py for better code organization.
"""
import numpy as np
import os
import yaml
from typing import Dict, List, Optional

from ..fleet_state_estimators import FleetStateEstimatorBase, ConsensusFleetEstimator


class DistributedHGEstimator(FleetStateEstimatorBase):
    """
    Distributed HG Observer for fleet longitudinal estimation
    More sophisticated, uses dynamics model and consensus
    Inspired by VehicleObserver.py _distributed_observer_each
    
    Communication Topology:
    - Uses adjacency matrix to define which FOLLOWER vehicles can communicate
    - Adjacency matrix is [observer_size x observer_size], including every vehicle
    - Matrix indices 0..observer_size-1 correspond to vehicle IDs 0..observer_size-1
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
        self.tau_i = self.config.get('tau_i', 0.16) # 对估计结果的影响不大。

        self.c0 = self.config.get('c0', 0.007023)
        self.c1 = self.config.get('c1', 0.14878)
        # The affine rolling-resistance model is only valid while the vehicle is
        # moving.  At u=0 it otherwise has the non-physical equilibrium
        # v=-c0/c1 and makes unmeasured vehicle blocks drift backwards.
        self.enable_stationary_deadzone = self.config.get('enable_stationary_deadzone', True)
        self.zero_speed_deadzone = max(
            0.0, float(self.config.get('zero_speed_deadzone', 0.05))
        )
        self.throttle_deadzone = max(
            0.0, float(self.config.get('throttle_deadzone', self.c0))
        )
        self.gamma = self.config.get('gamma', 1.0)
        self.r_i = self.config.get('r_i', 1.0)
        self.high_gain_theta = self.config.get('high_gain_theta', self.config.get('theta', 1.0))

        # Load Gains (using extra configs if available)
        self._load_extra_config()

        if self.consensus_estimator:
            return

        # System matrices of the longitudinal model in prime form:
        # A = diag(A_ni), B = diag(B_ni), B_tau = diag(B_tau_i)
        tau = self.tau_i  # Time constant
        self.throttle_gain = self.config.get('throttle_gain', self.config.get('Kth', 8.6993))
        self.Kth = self.throttle_gain
        self.observer_size = self.fleet_size

        self.block_state_dim = 3 # Each vehicle's observer state dimension (position, velocity, acceleration)
        A_ni = np.array([
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
        ])

        B_ni = np.array([
            [0.0],
            [0.0],
            [1.0],
        ])

        B_tau_i = self.throttle_gain * B_ni / tau

        A_blocks = []
        B_blocks = []
        B_tau_blocks = []
        for i in range(self.observer_size):
            a_row_blocks = []
            b_row_blocks = []
            b_tau_row_blocks = []
            for j in range(self.observer_size):
                if i == j:
                    a_row_blocks.append(A_ni)
                    b_row_blocks.append(B_ni)
                    b_tau_row_blocks.append(B_tau_i)
                else:
                    a_row_blocks.append(np.zeros_like(A_ni))
                    b_row_blocks.append(np.zeros_like(B_ni))
                    b_tau_row_blocks.append(np.zeros_like(B_tau_i))
            A_blocks.append(a_row_blocks)
            B_blocks.append(b_row_blocks)
            B_tau_blocks.append(b_tau_row_blocks)
        self.A_delta = np.block(A_blocks)
        self.B_nonlinear = np.block(B_blocks)
        self.B_tau = np.block(B_tau_blocks)
        # Keep the old attribute name for code paths that expect B_delta.
        self.B_delta = self.B_tau

        # -- Measurement matrix --
   
        self.local_measurement_dim = 1 # measuring position

        # CRITICAL: Validate and correct gain matrix dimensions
        dim_distributed_observer = self.block_state_dim * self.observer_size
        
        self.observer_gain = self._build_observer_gain_from_k_hg()
        
        # Validate P_i: should be [dim_distributed_observer x dim_distributed_observer]
        expected_consensus_gain_shape = (dim_distributed_observer, dim_distributed_observer)
        if hasattr(self.P_i, 'shape'):
            if self.P_i.shape != expected_consensus_gain_shape:
                if self.logger:
                    self.logger.logger.error(
                        f"Vehicle {self.vehicle_id}: P_i shape mismatch! "
                        f"Expected {expected_consensus_gain_shape}, got {self.P_i.shape}. "
                        f"Using default identity-based gain."
                    )
                self.P_i = np.eye(dim_distributed_observer) * 0.2
        else:
            self.P_i = np.eye(dim_distributed_observer) * float(self.P_i)
        # Backward-compatible alias for diagnostics that still print consensus_gain.
        self.consensus_gain = self.P_i

        self.consensus_coupling_gain = self._build_consensus_coupling_gain()

        # Communication weights (uniform for now)
        self.weights = np.ones(self.observer_size) / self.observer_size
        
        # Communication adjacency matrix
        if 'adjacency_matrix' in self.config:
            self.adjacency_matrix = np.array(self.config['adjacency_matrix'])
        else:
            # Default chain topology
            self.adjacency_matrix = self._create_fully_connected_topology()

        # Validate adjacency matrix dimensions
        if self.adjacency_matrix.shape != (self.observer_size, self.observer_size):
            self.adjacency_matrix = self._create_fully_connected_topology()
        
        # Cache neighbor list at initialization
        self.my_neighbors = self.get_neighbors(self.vehicle_id)
        
        # Cache output matrix Ci at initialization
        self.Ci = self.compute_output_matrix_Ci(self.vehicle_id)
        
        # === Debug Data Storage and Recording ===
        # Dictionary to store internal data for each update cycle for debugging
        self.debug_data = {}
        self.debug_recording_enabled = self.config.get('debug_recording', True)
        self.debug_output_dir = self.config.get('debug_output_dir', 'observer_recordings')
        self.debug_run_id = self.config.get('debug_run_id')
        self.recorder = None
        self._update_count = 0
        self._recording_start_time = None
        
        # Auto-start recorder if debug_recording is enabled in config
        if self.debug_recording_enabled:
            self._init_recorder()
        
        # Initialize estimated_state (will be updated in the first update() call)
        # Distributed observer state dimension: 3 * observer_size
        self.estimated_state = np.zeros(3 * self.observer_size)
        self.initialize_from_true_states = self.config.get('initialize_from_true_states', True)
        self._true_state_initialization_complete = False
        # Keep consensus disabled throughout true-state initialization.  If all
        # states become available in an update, consensus is enabled only after
        # that update so the initialization frame cannot receive a large kick.
        self._consensus_ready = not self.initialize_from_true_states
        
        # CRITICAL: Force fleet_states to correct dimensions
        # The parent class initializes it, but we must ensure it's correct
        # fleet_size includes leader (vehicle 0) + all followers
        if self.fleet_states.shape != (self.state_dim, self.fleet_size):
            self.fleet_states = np.zeros((self.state_dim, self.fleet_size))


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
            from .distributed_hg_recorder import DistributedHGRecorder
            
            self.recorder = DistributedHGRecorder(
                output_dir=self.debug_output_dir,
                vehicle_id=self.vehicle_id,
                observer_size=self.observer_size,
                fleet_size=self.fleet_size,
                run_id=self.debug_run_id,
            )
            filepath = self.recorder.start()
            self._recording_start_time = 0.0
            if self.logger:
                self.logger.logger.info(
                    f"Vehicle {self.vehicle_id}: HG debug recording started: {filepath}"
                )
        except Exception as e:
            if self.logger:
                self.logger.log_error(f"Failed to initialize HG debug recorder", e)
            self.recorder = None
    
    def stop_recording(self):
        """Stop the debug data recorder and close the file."""
        if self.recorder is not None:
            self.recorder.stop()
            self.recorder = None

    def reset(self):
        """Reset HG state and require initialization before consensus again."""
        super().reset()
        self.received_observer_states.clear()
        self.estimated_state = np.zeros(self.block_state_dim * self.observer_size)
        self._true_state_initialized_mask = np.zeros(self.observer_size, dtype=bool)
        self._true_state_initialization_complete = False
        self._consensus_ready = not self.initialize_from_true_states
        self.debug_data = {}
    
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
        Get neighbor vehicle IDs for a given vehicle
        
        Note: Adjacency matrix includes all vehicles.
        Matrix index mapping: matrix index i = vehicle_id
        
        Args:
            vehicle_id: Vehicle ID (0 to observer_size-1)
        
        Returns:
            neighbors: List of neighbor vehicle IDs (also followers)
        """
        if vehicle_id < 0 or vehicle_id >= self.observer_size:
            return []
        
        # Convert vehicle ID to matrix index
        matrix_idx = vehicle_id
        
        # Find all neighbors where adjacency_matrix[matrix_idx, neighbor_matrix_idx] > 0
        neighbors = []
        for j in range(self.observer_size):
            if self.adjacency_matrix[matrix_idx, j] > 0:
                neighbors.append(j)
        
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

    def _build_high_gain_scaling_matrix(self) -> np.ndarray:
        """
        Build block-diagonal T(theta) for the prime-form observer.

        For each third-order prime block, T_i(theta)=diag(theta^2, theta, 1).
        """
        theta = float(self.high_gain_theta)
        if abs(theta) < 1e-9:
            theta = 1e-9

        powers = np.arange(self.block_state_dim - 1, -1, -1, dtype=float)
        T_i = np.diag(theta ** powers)
        blocks = []
        zero_block = np.zeros_like(T_i)

        for i in range(self.observer_size):
            row_blocks = []
            for j in range(self.observer_size):
                row_blocks.append(T_i if i == j else zero_block.copy())
            blocks.append(row_blocks)

        return np.block(blocks)

    def _build_consensus_coupling_gain(self) -> np.ndarray:
        """
        Build gamma*r_i*theta*T(theta)*P_i^{-1}*T^{-1}(theta).
        """
        dim_distributed_observer = self.block_state_dim * self.observer_size
        theta = float(self.high_gain_theta)
        T_theta = self._build_high_gain_scaling_matrix()
        T_theta_inv = np.linalg.pinv(T_theta)

        if hasattr(self.P_i, 'shape'):
            P_i_inv = np.linalg.pinv(self.P_i)
        else:
            scalar_gain = float(self.P_i)
            if abs(scalar_gain) < 1e-9:
                scalar_gain = 1e-9
            P_i_inv = np.eye(dim_distributed_observer) / scalar_gain

        return self.gamma * self.r_i * theta * T_theta @ P_i_inv @ T_theta_inv

    def _build_observer_gain_from_k_hg(self) -> np.ndarray:
        """
        Build L_i = [0 ... theta*K_i_HG^T ... 0]^T.

        K_i_HG is configured as a 3-vector for the local prime-form block.
        The returned gain is a full (3*fleet_size)x1 vector with only the
        vehicle_id-th block populated.
        """
        k_i_hg = np.asarray(self.K_i_HG, dtype=float).reshape(self.block_state_dim, 1)
        theta = float(self.high_gain_theta)
        if abs(theta) < 1e-9:
            theta = 1e-9

        local_gain = theta * k_i_hg

        L_i = np.zeros((self.block_state_dim * self.observer_size, self.local_measurement_dim))
        block_start = self.block_state_dim * self.vehicle_id
        L_i[block_start:block_start + self.block_state_dim, :] = local_gain
        return L_i

    def _compute_nonlinear_f_term(self, x_vec: np.ndarray) -> np.ndarray:
        """
        Compute f_i(hat{x}_i) for each vehicle block:
            f_i = -(hat{a}_i + Kth*(c0 + c1*hat{v}_i))/tau

        Together with B_tau*u, this gives:
            tau*dot{a}_i + a_i = Kth*(u_th - c0 - c1*v_i)
        """
        x_mat = x_vec.reshape((self.block_state_dim, self.observer_size), order="F")
        v_hat = x_mat[1, :]
        x3_hat = x_mat[2, :]
        return -(x3_hat + self.throttle_gain * (self.c0 + self.c1 * v_hat)) / self.tau_i
    
    def _compute_dynamics_term(self, x_vec: np.ndarray, collective_control: np.ndarray) -> np.ndarray:
        """
        Compute dynamics prediction term for observer update.
        
        Args:
            x_vec: Current observer state vector [3*observer_size]
            collective_control: Control input vector for all followers [observer_size]
        
        Returns:
            dynamics_term: Dynamics prediction [3*observer_size]
        """
        nonlinear_f = self._compute_nonlinear_f_term(x_vec)
        dynamics_term = (
            self.A_delta @ x_vec
            + self.B_tau @ collective_control
            + self.B_nonlinear @ nonlinear_f
        )

        if not self.enable_stationary_deadzone:
            return dynamics_term

        # Static-friction/zero-speed mode.  The normal affine model contains the
        # constant term -Kth*c0/tau, so zero throttle at zero speed predicts a
        # backwards acceleration.  Freeze position prediction in this small
        # region and dissipate residual v/a estimates toward zero.  Consensus
        # and measurement corrections are still applied by the caller.
        x_mat = np.asarray(x_vec, dtype=float).reshape(
            (self.block_state_dim, self.observer_size), order="F"
        )
        control_vec = np.asarray(collective_control, dtype=float).reshape(-1)
        stationary_mask = (
            (np.abs(x_mat[1, :]) <= self.zero_speed_deadzone)
            & (np.abs(control_vec) <= self.throttle_deadzone)
        )
        decay_tau = max(float(self.tau_i), 1e-6)

        for vehicle_idx in np.flatnonzero(stationary_mask):
            block_start = self.block_state_dim * vehicle_idx
            dynamics_term[block_start] = 0.0
            dynamics_term[block_start + 1] = -x_mat[1, vehicle_idx] / decay_tau
            dynamics_term[block_start + 2] = -x_mat[2, vehicle_idx] / decay_tau

        return dynamics_term
    
    def _compute_measurement_term(self, x_vec: np.ndarray, local_state: np.ndarray, 
                                 current_time_ns: int) -> tuple:
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
        
        local_measurement[0] = p_i  
        
        
        # Compute estimated measurement
        estimated_measurement = self.Ci @ x_vec
        
        # Compute measurement error
        measurement_error = local_measurement - estimated_measurement
        
        # Apply observer gain
        measurement_term = self.observer_gain @ measurement_error  # Add a small bias to prevent stagnation
        
        return measurement_term, local_measurement, estimated_measurement, measurement_error
       

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

        for vehicle_id in range(self.fleet_size):
            vehicle_idx = vehicle_id
            if vehicle_idx >= self.observer_size:
                break

            if vehicle_id == self.vehicle_id and control is not None and len(control) > 1:
                collective_control[vehicle_idx] = control[1]
                continue

            received_control = self._get_latest_received_control(vehicle_id, current_time_ns)
            if received_control is not None and len(received_control) > 1:
                collective_control[vehicle_idx] = received_control[1]

                if self.logger and self.debug_recording_enabled:
                    self.logger.logger.debug(
                        f"Vehicle {self.vehicle_id}: Using V2V throttle from vehicle {vehicle_id}: "
                        f"throttle={received_control[1]:.3f}"
                    )
            elif self.logger:
                self.logger.logger.warning(
                    f"Vehicle {self.vehicle_id}: No V2V throttle data from vehicle {vehicle_id}, using 0.0"
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
                
            
            # --- Step 3: Calculate consensus difference with adjacency weight ---
            if neighbor_x_vec is not None:
                my_matrix_idx = self.vehicle_id
                neighbor_matrix_idx = neighbor_id
                weight = self.adjacency_matrix[my_matrix_idx, neighbor_matrix_idx]
                
                # Accumulate weighted difference: (neighbor_estimate - own_estimate)
                consensus_diff = neighbor_x_vec - x_vec
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
            consensus_term = self.consensus_coupling_gain @ consensus_accum
            # Numerical protection: prevent consensus term explosion
            consensus_norm = np.linalg.norm(consensus_term)
            max_consensus_threshold = 2.0e3
            if consensus_norm > max_consensus_threshold:
                consensus_term = consensus_term / consensus_norm * max_consensus_threshold
            
            if self.logger and self.debug_recording_enabled:
                self.logger.logger.debug(
                    f"Vehicle {self.vehicle_id}: Consensus applied. "
                    f"accum_norm={np.linalg.norm(consensus_accum):.6f}, "
                    f"gain shape={self.consensus_coupling_gain.shape}, "
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
        Compute C_i = [0 ... C_ni ... 0], where C_ni = [1, 0, ..., 0].
        
        Args:
            i: Vehicle ID, starting from 0.
        
        Returns:
            Ci: Output matrix, shape [1, block_state_dim*observer_size]
        """
        if i < 0 or i >= self.observer_size:
            raise ValueError(f"Vehicle index i={i} out of range [0, {self.observer_size - 1}]")
        
        Ci = np.zeros((1, self.block_state_dim * self.observer_size))
        block_start = self.block_state_dim * i
        Ci[0, block_start] = 1.0
        return Ci

    def _load_extra_config(self):
        """
        Load HG observer parameters from the config passed by VehicleObserverSimple.

        Per-vehicle YAML overrides are merged before estimator construction, so this
        method intentionally does not read Observer/extra_configs/carX.yaml.
        """
        self.K_i_HG = self.config.get('K_i_HG', self.config.get('observer_gain', [0.1, 0.1, 0.1]))
        self.P_i = self.config.get('P_i', self.config.get('consensus_gain', 0.2))

        for matrix_param in ['K_i_HG', 'observer_gain', 'P_i', 'consensus_gain', 'adjacency_matrix']:
            if matrix_param in self.config and isinstance(self.config[matrix_param], list):
                setattr(self, matrix_param, np.array(self.config[matrix_param], dtype=float))

        if 'observer_gain' in self.config and 'K_i_HG' not in self.config:
            legacy_gain = np.asarray(self.config['observer_gain'], dtype=float)
            if legacy_gain.size == 3:
                self.K_i_HG = legacy_gain.reshape(3)

        if 'P_i' in self.config:
            self.P_i = np.array(self.config['P_i'], dtype=float)
        elif 'consensus_gain' in self.config:
            self.P_i = np.array(self.config['consensus_gain'], dtype=float)

        for param in ['tau_i', 'c0', 'c1', 'throttle_gain', 'Kth', 'gamma', 'r_i', 'high_gain_theta', 'theta']:
            if param in self.config:
                if param == 'theta':
                    self.high_gain_theta = self.config[param]
                elif param == 'Kth':
                    self.throttle_gain = self.config[param]
                    self.Kth = self.throttle_gain
                else:
                    setattr(self, param, self.config[param])

    def _transfer_estimated_state_to_fleet_states(
        self, estimated_state: np.ndarray, local_state: np.ndarray = None
    ) -> np.ndarray:
        """
        Map HG observer state [x0,v0,a0,x1,v1,a1,...] into fleet_states.

        fleet_states keeps its original meaning and shape:
            row 0: x
            row 1: y
            row 2: theta
            row 3: velocity
            row 4: acceleration
        The HG observer does not estimate y/theta, so those rows are preserved.
        """
        estimated_state = np.asarray(estimated_state, dtype=float).reshape(-1)
        expected_dim = self.block_state_dim * self.fleet_size
        if estimated_state.size != expected_dim:
            raise ValueError(
                f"HG estimated_state dimension mismatch: expected {expected_dim}, "
                f"got {estimated_state.size}"
            )

        estimated_mat = estimated_state.reshape(
            (self.block_state_dim, self.fleet_size), order="F"
        )
        fleet_states_new = self.fleet_states.copy()

        fleet_states_new[0, :] = estimated_mat[0, :]
        fleet_states_new[3, :] = estimated_mat[1, :]
        fleet_states_new[4, :] = estimated_mat[2, :]

        if local_state is not None and len(local_state) > 2 and 0 <= self.vehicle_id < self.fleet_size:
            fleet_states_new[1, self.vehicle_id] = local_state[1]
            fleet_states_new[2, self.vehicle_id] = local_state[2]

        return fleet_states_new

    def _state_to_prime_block(self, state: np.ndarray) -> np.ndarray:
        """Convert [x, y, theta, v, a] to the HG block [x, v, a]."""
        state = np.asarray(state, dtype=float).reshape(-1)
        return np.array([
            state[0] if len(state) > 0 else 0.0,
            state[3] if len(state) > 3 else 0.0,
            state[4] if len(state) > 4 else 0.0,
        ], dtype=float)

    def _initialize_estimated_state_from_true_states(
        self, local_state: np.ndarray, current_time_ns: int
    ) -> None:
        """
        Initialize HG estimated_state from the true/local V2V states.

        The estimator state layout is [x0, v0, a0, x1, v1, a1, ...]. We fill
        every block whose real state is currently available. The flag is marked
        complete only after all vehicles have been initialized from true states.
        """
        if not self.initialize_from_true_states or self._true_state_initialization_complete:
            return

        initialized_mask = getattr(
            self,
            "_true_state_initialized_mask",
            np.zeros(self.observer_size, dtype=bool),
        )
        updated_state = self.estimated_state.copy()

        for vid in range(self.observer_size):
            state = None
            if vid == self.vehicle_id:
                state = local_state
            else:
                state = self._get_latest_received_state(vid, current_time_ns)

            if state is None:
                continue

            block_start = self.block_state_dim * vid
            updated_state[block_start:block_start + self.block_state_dim] = (
                self._state_to_prime_block(state)
            )
            initialized_mask[vid] = True

        self.estimated_state = updated_state
        self._true_state_initialized_mask = initialized_mask

        if np.all(initialized_mask[:self.observer_size]):
            self._true_state_initialization_complete = True
            self.fleet_states = self._transfer_estimated_state_to_fleet_states(
                self.estimated_state, local_state
            )
            if self.logger:
                self.logger.logger.info(
                    f"Vehicle {self.vehicle_id}: HG observer initialized from true "
                    f"states for {self.observer_size} vehicles."
                )
    
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
            self._initialize_estimated_state_from_true_states(local_state, current_time_ns)
            
            # Distributed observer for this vehicle
            self.estimated_state = self._distributed_hg_observer_update(
                local_state, current_time_ns, control, dt
            )

            if self._true_state_initialization_complete:
                self._consensus_ready = True
            
            # Add the estimated x/v/a states back to fleet_states.
            self.fleet_states = self._transfer_estimated_state_to_fleet_states(
                self.estimated_state, local_state
            )
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
    
  
    def _distributed_hg_observer_update(self, local_state: np.ndarray, current_time_ns: int,
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
        # Distributed observer dimension: [x0,v0,a0,x1,v1,a1,...]
        longitudinal_state_dim = 3
        dim_distributed_observer = longitudinal_state_dim * self.observer_size

        # Read current observer state directly without conversion
        x_vec = self.estimated_state.copy()

    

        # Calculate collective control input from directly communicated throttle values.
        collective_control = self._calculate_collective_control_v2v(
            control, current_time_ns, local_state
        )

        # 1. Compute dynamics prediction term
        dynamics_term = self._compute_dynamics_term(x_vec, collective_control)
        
        # 2. Compute measurement correction term
        measurement_term, local_measurement, estimated_measurement, measurement_error = \
            self._compute_measurement_term(x_vec, local_state, current_time_ns)
        
        # 3. Compute consensus correction only after initialization has already
        # completed in a previous update.  This prevents partially initialized
        # zero blocks from entering the distributed feedback loop.
        if self._consensus_ready:
            consensus_term = self._compute_consensus_term(x_vec, current_time_ns)
            neighbor_count = len(self.my_neighbors)
        else:
            consensus_term = np.zeros(dim_distributed_observer)
            neighbor_count = 0
              
        x_i_new = x_vec + dt * (dynamics_term + measurement_term + consensus_term)
        
        # State constraint: prevent numerical overflow
        # x_i_new = np.clip(x_i_new, -1e4, 1e4)

        
        # === Store debug data for recording (ALWAYS, not just when debug_recording_enabled) ===
        
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
            'local_measurement_p': local_measurement[0],  # Local position measurement
            'local_measurement_v': np.nan,  # Velocity is not part of C_i measurement
            'dt': dt,  # Time step
            # Position, velocity, acceleration for local vehicle
            'position': local_state[0],
            'velocity': local_state[3],
            'acceleration': local_state[4] if len(local_state) > 4 else 0.0,
        }
        
        # Record true states of all vehicles via V2V communication
        # This ensures all data is synchronized to current vehicle's timestamp
        
        
        # Record every vehicle's true state, including vehicle 0.
        for vehicle_id in range(self.fleet_size):
            if vehicle_id == self.vehicle_id:
                self.debug_data[f'true_position_{vehicle_id}'] = local_state[0]
                self.debug_data[f'true_velocity_{vehicle_id}'] = local_state[3]
                self.debug_data[f'true_acceleration_{vehicle_id}'] = local_state[4] if len(local_state) > 4 else 0.0
                self.debug_data[f'true_throttle_{vehicle_id}'] = control[1]
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

            other_control = self._get_latest_received_control(vehicle_id, current_time_ns)
            self.debug_data[f'true_throttle_{vehicle_id}'] = (
                other_control[1]
                if other_control is not None and len(other_control) > 1
                else np.nan
            )
        
        return x_i_new

    def _get_nonlinear_term_phi_i(self, v_i, x3_i, g_s: float = 9.81):
        """
        Compute the requested nonlinear term f_i(hat{x}_i).

        Args:
            v_i: Velocity (scalar or numpy array)
            x3_i: Third prime-form state component (scalar or numpy array)
            g_s: Gravitational acceleration constant
        """
        v_i = np.asarray(v_i)
        x3_i = np.asarray(x3_i)

        return -(x3_i + self.c0 * self.Kth + self.c1 * v_i * self.Kth) / self.tau_i
    
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
                           [x0, v0, a0, x1, v1, a1, ...]
                           Returns zeros if observer state is not initialized (e.g., leader vehicle)
        """
        if not hasattr(self, 'estimated_state') or self.estimated_state is None:
            # For leader vehicle or when consensus_estimator is used, return zeros
            observer_size = getattr(self, 'observer_size', self.fleet_size)
            return np.zeros(3 * observer_size)
        return self.estimated_state.copy()
    
    def get_observer_state_for_broadcast(self) -> Dict:
        """
        Get observer state formatted for V2V broadcast message.
        
        Returns:
            dict: Message-ready dictionary with observer state data
        """
        observer_size = getattr(self, 'observer_size', self.fleet_size)
        
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
