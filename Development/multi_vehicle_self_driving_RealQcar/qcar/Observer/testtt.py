
class ConsensusFleetEstimator(FleetStateEstimatorBase):
    """
    Consensus-based distributed fleet estimator
    Separates direct observations (local states) from neighbor opinions (fleet states).
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5,
                 config: Dict = None, logger=None):
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)
        
        # Consensus parameters
        self.consensus_gain = self.config.get('consensus_gain', 0.1)  # Weight for neighbor opinions
        self.direct_gain = self.config.get('direct_gain', 0.6)        # Weight for direct broadcasts (higher trust)
        
        # NEW: Store fleet views received from neighbors
        # Format: sender_id -> [(timestamp_ns, fleet_estimates_dict)]
        self.received_fleet_states = defaultdict(list)
    
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

    def add_received_fleet_state(self, sender_id: int, fleet_estimates: Dict, timestamp_ns: int) -> bool:
        """
        Store received FLEET state in separate storage for consensus
        """
        try:
            if sender_id == self.vehicle_id:
                return False
            
            # Check if fleet estimate contains new vehicles we don't know about
            max_id_in_msg = max((int(vid) for vid in fleet_estimates.keys()), default=0)
            if max_id_in_msg >= self.fleet_size:
                self._ensure_fleet_capacity(max_id_in_msg)

            # Store the raw fleet dictionary with timestamp
            self.received_fleet_states[sender_id].append((timestamp_ns, fleet_estimates))
            
            # Keep history limited (e.g., last 5 fleet snapshots per neighbor)
            if len(self.received_fleet_states[sender_id]) > 5:
                self.received_fleet_states[sender_id] = self.received_fleet_states[sender_id][-5:]
            
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Add received fleet state error", e)
            return False

    def _get_latest_fleet_data(self, neighbor_id: int, current_time_ns: int) -> Optional[Dict]:
        """Helper to get latest valid fleet dict from a neighbor"""
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

    def _cleanup_old_data(self, current_time_ns: int):
        """Override to clean up both local and fleet storages"""
        # 1. Clean local states (parent logic)
        super()._cleanup_old_data(current_time_ns)
        
        # 2. Clean fleet states
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