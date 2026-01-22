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
        self.tau_i = self.config.get('tau_i', 0.16)
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
        self.h = 1.0     # time headway
        self.d = 0.4    # Desired distance
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

        if self.logger:
            self.logger.logger.info(
                f"Vehicle {self.vehicle_id}: Observer gain: {self.observer_gain}, Consensus gain: {self.consensus_gain}"
            )

        # Communication weights (uniform for now)
        self.weights = np.ones(self.observer_size) / self.observer_size
        
        # Communication adjacency matrix
        if 'adjacency_matrix' in self.config:
            self.adjacency_matrix = np.array(self.config['adjacency_matrix'])
        else:
            # Default chain topology
            self.adjacency_matrix = self._create_chain_topology()

        if self.logger:
            self.logger.logger.info(
                f"Vehicle {self.vehicle_id}: Adjacency matrix initialized "
                f"[{self.observer_size}x{self.observer_size}]:\n{self.adjacency_matrix}"
            )

        # Validate adjacency matrix dimensions
        if self.adjacency_matrix.shape != (self.observer_size, self.observer_size):
            if self.logger:
                self.logger.logger.warning(
                    f"Adjacency matrix shape {self.adjacency_matrix.shape} doesn't match observer_size {self.observer_size}. "
                    f"Using default chain topology."
                )
            self.adjacency_matrix = self._create_chain_topology()
        
        # Cache neighbor list at initialization
        self.my_neighbors = self.get_neighbors(self.vehicle_id)
        
        # Cache output matrix Ci at initialization
        self.Ci = self.compute_output_matrix_Ci(self.vehicle_id)
        
        if self.logger:
            self.logger.logger.info(
                f"Vehicle {self.vehicle_id}: Neighbors list initialized: {self.my_neighbors}"
            )
            self.logger.logger.info(
                f"Vehicle {self.vehicle_id}: Output matrix Ci computed: {self.Ci}. System matrices A_delta: {self.A_delta}, Input matrix B_delta: {self.B_delta}."
            )
        
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
            
            if self.logger:
                self.logger.logger.info(
                    f"Vehicle {self.vehicle_id}: Debug recording started, saving to {filepath}"
                )
        except Exception as e:
            if self.logger:
                self.logger.log_error(f"Failed to initialize debug recorder", e)
            self.recorder = None
    
    def stop_recording(self):
        """Stop the debug data recorder and close the file."""
        if self.recorder is not None:
            stats = self.recorder.stop()
            if self.logger:
                self.logger.logger.info(
                    f"Vehicle {self.vehicle_id}: Debug recording stopped - "
                    f"recorded={stats.get('record_count', 0)}, "
                    f"dropped={stats.get('dropped_count', 0)}, "
                    f"file={stats.get('filepath', 'unknown')}"
                )
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
        
        # Try to load specific parameters from extra_configs file
        try:
            # extra_configs is in the parent directory (Observer/)
            config_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'extra_configs')
            config_file = os.path.join(config_dir, f'car{self.vehicle_id}.yaml')
            
            if os.path.exists(config_file):
                if self.logger:
                    self.logger.logger.info(f"Vehicle {self.vehicle_id}: Loading extra config from {config_file}")
                
                with open(config_file, 'r') as f:
                    extra_conf = yaml.safe_load(f)
                    
                if extra_conf and 'observer' in extra_conf:
                    obs_conf = extra_conf['observer']
                    
                    # Check if we should use ConsensusFleetEstimator logic instead
                    if obs_conf.get('fleet_estimator_type') == 'consensus':
                        if self.logger:
                            self.logger.logger.info(f"Vehicle {self.vehicle_id}: Switching to ConsensusFleetEstimator logic.")
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
                if self.logger:
                    self.logger.logger.warning(
                        f"Vehicle {self.vehicle_id}: Extra config file {config_file} not found. Using defaults."
                    )
        except Exception as e:
            if self.logger:
                self.logger.log_error(f"Error loading extra config for vehicle {self.vehicle_id}", e)
    
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
            # Transfer the estimated states back to fleet_states. 
            self.fleet_states, _ = self._transfer_estimated_states_to_fleet_states(self.estimated_state, local_state, current_time_ns)
            # Cleanup old data
            self._cleanup_old_data(current_time_ns)
            
            # === Record debug data if enabled ===
            if self.recorder is not None and self.debug_data:
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
    
    def _transfer_fleet_states_to_estimated_states(self, fleet_states: np.ndarray, current_time_ns: int, local_state: np.ndarray = None) -> np.ndarray:
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
            if self.logger:
                self.logger.logger.warning(
                    f"Vehicle {self.vehicle_id}: No recent leader state, using current estimate"
                )
        
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
                                     collective_control: np.ndarray, dt: float) -> np.ndarray:
        """
        Distributed observer update for one target vehicle
        Combines dynamics prediction, measurement correction, and consensus
        """
        # Distributed observer dimension (3: pi-p0+di0, vi-v0, ai-a0)
        longitudinal_state_dim = 3
        dim_distributed_observer = longitudinal_state_dim * self.observer_size

        # Distributed observer state (x_vec: pi-p0+di0, vi-v0, ai-a0)
        x_vec, di0_values_before = self._transfer_fleet_states_to_estimated_states(self.fleet_states, current_time_ns, local_state)

        # Get leader state with fallback to current estimate
        state_leader = self._get_latest_received_state(0, current_time_ns)
        if state_leader is not None:
            v0 = state_leader[3]
            p0 = state_leader[0]
        else:
            v0 = self.fleet_states[3, 0]
            p0 = self.fleet_states[0, 0]
            if self.logger:
                self.logger.logger.warning(
                    f"Vehicle {self.vehicle_id}: No recent leader state of the leader 0, using current estimate"
                )

        # Get the control input of all follower vehicles
        # TODO: get the latest control input from V2V messages
        test_throttle_value = 0.075
        collective_control = np.zeros(self.observer_size)
        for i in range(self.observer_size):
            collective_control[i] = test_throttle_value
           
        # 1. Dynamics prediction (collective longitudinal model)
        dynamics_term = self.A_delta @ x_vec + self.B_delta @ collective_control
        
        # 2. Measurement correction (if we have data from target)
        measurement_term = np.zeros(dim_distributed_observer)  
        local_measurement = np.zeros(self.local_measurement_dim)
        
        # Read self state from local_state input
        p_i = local_state[0]
        v_i = local_state[3]
        
        # Get preceding vehicle state from communication
        state_prev = self._get_latest_received_state(self.vehicle_id - 1, current_time_ns)
        if state_prev is not None:
            p_prev = state_prev[0]
        else:
            p_prev = self.fleet_states[0, self.vehicle_id - 1]
            if self.logger:
                self.logger.logger.warning(
                    f"Vehicle {self.vehicle_id}: No recent local state of previous vehicle {self.vehicle_id - 1}, using current estimate"
                )
        
        local_measurement[0] = p_i - p_prev  # relative position
        local_measurement[1] = v_i  # velocity 
        
        estimated_measurement = self.Ci @ x_vec + self.Cv * v0 + self.Cd * self.d

        measurement_error = local_measurement - estimated_measurement
        measurement_term = self.observer_gain @ measurement_error
        
        # 3. Consensus term - based on adjacency matrix
        consensus_term = np.zeros(dim_distributed_observer)
        neighbor_count = 0
        consensus_accum = np.zeros(dim_distributed_observer)
        
        # Loop through each neighbor defined by adjacency matrix
        for neighbor_id in self.my_neighbors:
            # --- Step 1: Try to get FLEET state (Primary Source) ---
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
                    # if self.logger:
                    #     self.logger.logger.info(
                    #         f"Vehicle {self.vehicle_id}: Complete fleet_state from neighbor {neighbor_id}"
                    #     )
                # else:
                #     if self.logger:
                #         self.logger.logger.warning(
                #             f"Vehicle {self.vehicle_id}: Incomplete fleet_state from neighbor {neighbor_id}, "
                #             f"missing vehicles: {missing_vehicles}"
                #         )
            
            # --- Step 2: Fallback to LOCAL states if needed ---
            # Only need it when stop the cars
            if not is_complete_fleet:
                if self.logger:
                    self.logger.logger.warning(
                        f"Vehicle {self.vehicle_id}: Using fallback strategy for neighbor {neighbor_id}"
                    )
                # Only need it when stop the cars
                if neighbor_fleet_dict is None:
                    neighbor_fleet_dict = {}
                    if self.logger:
                        self.logger.logger.warning(
                            f"Vehicle {self.vehicle_id}: No fleet_state from neighbor {neighbor_id}, "
                            f"building from scratch"
                        )
                
                # Fill missing vehicles with local state broadcasts or current estimates
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
                        if self.logger:
                            self.logger.logger.info(
                                f"Vehicle {self.vehicle_id}: Filled vehicle_{vid} using its local_state"
                            )
                    else:
                        neighbor_fleet_dict[vid] = {
                            'x': float(self.fleet_states[0, vid]),
                            'y': float(self.fleet_states[1, vid]),
                            'theta': float(self.fleet_states[2, vid]),
                            'velocity': float(self.fleet_states[3, vid]),
                            'acceleration': float(self.fleet_states[4, vid])
                        }
                        # Never happens
                        if self.logger:
                            self.logger.logger.info(
                                f"Vehicle {self.vehicle_id}: Filled vehicle_{vid} using current estimate "
                                f"(no local_state available)"
                            )
            
            # --- Step 3: Build neighbor's complete fleet_states matrix ---
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
            
            # --- Step 4: Convert to distributed observer state format ---
            neighbor_x_vec, neighbor_di0_values = self._transfer_fleet_states_to_estimated_states(
                neighbor_fleet_states, current_time_ns
            )
            
            # --- Step 5: Calculate consensus difference with adjacency weight ---
            my_matrix_idx = self.vehicle_id - 1
            neighbor_matrix_idx = neighbor_id - 1
            weight = self.adjacency_matrix[my_matrix_idx, neighbor_matrix_idx]
            
            # Accumulate weighted difference: (own_estimate - neighbor_estimate)
            consensus_diff = x_vec - neighbor_x_vec
            consensus_accum += consensus_diff
            neighbor_count += 1
        
        # --- Step 6: Apply consensus gain (normalize by neighbor count) ---
        if neighbor_count > 0:
            consensus_term = self.consensus_gain  @ consensus_accum
            
            # Numerical protection: prevent consensus term explosion
            consensus_norm = np.linalg.norm(consensus_term)
            max_consensus_threshold = 50.0
            if consensus_norm > max_consensus_threshold:
                # if self.logger:
                #     self.logger.logger.warning(
                #         f"Vehicle {self.vehicle_id}: Consensus term too large ({consensus_norm:.2f}), "
                #         f"clamping to {max_consensus_threshold}"
                #     )
                consensus_term = consensus_term / consensus_norm * max_consensus_threshold
        
            # if self.logger:
            #     self.logger.logger.info(
            #         f"Vehicle {self.vehicle_id}: Final consensus term applied, "
            #         f"neighbor_count={neighbor_count}, consensus_norm={np.linalg.norm(consensus_term):.4f}"
            #     )
        else:
            consensus_term = np.zeros(dim_distributed_observer)
            if self.logger:
                self.logger.logger.warning(
                    f"Vehicle {self.vehicle_id}: No valid neighbors for consensus"
                )
        
        # Combine all terms (note: consensus term is subtracted, per theory)
        # consensus_term = np.zeros(dim_distributed_observer) 
        x_i_new = x_vec + dt * (dynamics_term + measurement_term - consensus_term)
        
        # State constraint: prevent numerical overflow
        x_i_new = np.clip(x_i_new, -1e4, 1e4)
        
        # === Store debug data for recording ===
        if self.debug_recording_enabled:
            self.debug_data = {
                'x_vec_before': x_vec.copy(), # Observer state before update
                'x_vec_after': x_i_new.copy(),# Observer state after update
                'dynamics_term': dynamics_term.copy(), # Dynamics prediction term
                'measurement_term': measurement_term.copy(), # Measurement correction term
                'consensus_term': consensus_term.copy(), # Consensus correction term
                'local_measurement': local_measurement.copy(), # Local measurement vector
                'estimated_measurement': estimated_measurement.copy(), # Estimated measurement vector
                'measurement_error': measurement_error.copy(), # Measurement error vector
                'neighbor_count': neighbor_count, # Number of neighbors used
                'consensus_norm': np.linalg.norm(consensus_term) if neighbor_count > 0 else 0.0, # Norm of consensus term
                'fleet_states': self.fleet_states.copy(), # Current fleet states
                'di0_values': di0_values_before.copy(), # di0 values for each follower
                'dt': dt, # Time step
            }
        
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
            m_i = self.m_i
            tau_i = self.tau_i
            rho_i = self.rho_i
            Cd_i = self.Cd_i
            AF_i = self.AF_i
            mu_i = self.mu_i
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
