import numpy as np
from typing import Dict, Optional, Any
from md_logging_config import get_weight_logger


# ═══════════════════════════════════════════════════════════════════════
# WEEK 2: SIMPLIFIED TRUST-BASED WEIGHT CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════
DEFAULT_WEIGHT_CONFIG = {
    # Fixed weights (no self-covariance adaptation)
    'w0_fixed': 0.3,          # Virtual node weight (fixed)
    'w_self_base': 0.2,       # Self weight (fixed)
    
    # Safety limits
    'w_cap': 0.4,             # Maximum weight per neighbor (40%)
    'kappa': 5,               # Maximum number of neighbors to consider
    
    # Temporal smoothing
    'eta': 0.15,              # EMA smoothing factor (0=no smoothing, 1=no memory)
    'enable_smoothing': True  # Enable temporal smoothing
}


class WeightTrustModule:
    """
    TrustManager - A class to manage trust scores, trusted neighbors, and weights
    for a platoon of vehicles under potential attacks.
    """
    
    def __init__(self, graph, trust_threshold, kappa, vehicle_id: Optional[int] = None):
        """
        Initialize the TrustManager with graph and parameters
        
        Parameters:
            graph              - Adjacency matrix (n x n) as numpy array
            trust_threshold    - Trust score threshold (e.g., 0.5)
            kappa              - Influence limit parameter
            vehicle_id         - ID of the host vehicle (for logging)
        """
        self.graph = np.array(graph)
        self.trust_threshold = trust_threshold
        self.kappa = kappa
        self.num_vehicles = self.graph.shape[0]
        self.vehicle_id = vehicle_id
        
        # Week 2: Previous weights for EMA smoothing
        self.prev_weights: Dict[int, np.ndarray] = {}  # vehicle_index -> previous weight array
        
        # Initialize weight logger if vehicle_id is provided
        self.weight_logger = get_weight_logger(vehicle_id) if vehicle_id is not None else None
    
    def get_trusted_neighbors(self, car_idx, trust_scores):
        """
        Compute trusted neighbors for each vehicle based on current trust scores
        
        Parameters:
            car_idx      - Index of the vehicle (0-based)
            trust_scores - Array or list of trust scores for all vehicles
            
        Returns:
            neighbors    - Array of indices of trusted neighbors of the vehicle
        """
        # Convert trust_scores to numpy array if it's not already
        trust_scores_array = np.array(trust_scores)
        
        # Find direct neighbors (non-zero entries in adjacency matrix)
        neighbors = np.where(self.graph[car_idx, :])[0]
        
        # Remove neighbors whose trust score is below the threshold
        trusted_mask = trust_scores_array[neighbors] > self.trust_threshold
        trusted_neighbors = neighbors[trusted_mask]
        
        return trusted_neighbors
    
    def calculate_weights_trust(self, vehicle_index, trust_scores, weight_type="local"):
        """
        Compute weight matrix based on trusted neighbors
        
        Parameters:
            vehicle_index - Index of the vehicle (0-based)
            trust_scores  - Array of trust scores for all vehicles
            weight_type   - Type of weighting ("local" or "distributed")
            
        Returns:
            weights_dis   - Weight array (1 x num_vehicles + 1)
        """
        virtual_graph = self.generate_virtual_graph(self.graph, vehicle_index)
        trusted_neighbors_set = self.get_trusted_neighbors(vehicle_index, trust_scores)
        num_nodes = virtual_graph.shape[0]
        weights_dis = np.zeros((1, num_nodes))
        
        N_i_t = trusted_neighbors_set  # Trusted neighbors (0-based indices)
        n_w_i = max(self.kappa, len(N_i_t) + 1 + 1)  # +1 for self, +1 for virtual node
        weight = 1.0 / n_w_i
        
        # Set self-weight: weights_dis[0, vehicle_index+1]
        weights_dis[0, vehicle_index + 1] = weight
        
        # Set weights for trusted neighbors: weights_dis[0, l+1]
        for l in N_i_t:
            weights_dis[0, l + 1] = weight
        
        # Set weight to virtual node: weights_dis[0, 0]
        if weight_type == "local":
            # Using weight matrix prioritizing local estimation
            weights_dis[0, 0] = 1 - (len(N_i_t) + 1) * weight
        else:
            # Using weight matrix distributed equally
            weights_dis[0, 0] = weight
            
        # NOTE: Either using local or distributed weight matrix
        # One problem is the weight matrix weights_dis[0, 0] still needs to be adjusted based on the trust
        # See more details in Observer class
        
        return weights_dis
    
    def calculate_weights_default(self, vehicle_index):
        """
        Function to calculate consensus weights for a given virtual graph
        
        Parameters:
            vehicle_index - Index of the vehicle (0-based)
            
        Returns:
            weights_dis   - Weight array (1 x num_vehicles + 1)
        """
        Vj = self.generate_virtual_graph(self.graph, vehicle_index)
        num_nodes = Vj.shape[0]
        W = np.zeros((num_nodes, num_nodes))  # Initialize weights matrix
        
        # Start from index 1 (second node) since index 0 is the virtual node
        for i in range(1, num_nodes):
            d_i = np.sum(Vj[i, :])  # Degree of node i
            for l in range(num_nodes):  # Include all nodes in the virtual graph
                if Vj[i, l] == 1 or i == l:  # Neighbor or self
                    W[i, l] = 1.0 / (d_i + 1)
        
        # Return the row corresponding to the vehicle (convert to 0-based indexing)
        weights_dis = W[vehicle_index + 1, :].reshape(1, -1)  # Shape: (1, num_vehicles + 1)
        
        return weights_dis
    
    def calculate_weights_trust_v2(
        self,
        vehicle_index: int,
        trust_scores: np.ndarray,
        config: Optional[Dict] = None
    ) -> Dict[str, Any]:
        """
        WEEK 2: Simplified trust-based adaptive weight calculation.
        
        Trust evaluation (Week 1) already handles quality gating and scoring.
        This method adds weight distribution based on trust scores and topology.
        
        Algorithm:
        1. Trust scores come directly from GraphBasedTrustEvaluator (0-1 range)
        2. Use fleet topology (graph adjacency) - vehicle_id encodes position
        3. Normalize with fixed virtual and self weights
        4. Apply influence capping (max 40% per neighbor)
        5. Apply EMA smoothing for temporal stability
        
        Parameters:
            vehicle_index  - Index of the vehicle (0-based)
            trust_scores   - Trust scores for all vehicles (0-1 range from Week 1)
            config         - Optional configuration (uses DEFAULT_WEIGHT_CONFIG if None)
            
        Returns:
            Dictionary containing:
                'weights': Weight array (1 x num_vehicles + 1)
                'trusted_neighbors': List of trusted neighbor indices
                'debug_info': Dictionary with intermediate values
        """
        # Use default config if not provided
        if config is None:
            config = DEFAULT_WEIGHT_CONFIG
        
        # Extract configuration
        w0_fixed = config.get('w0_fixed', 0.3)
        w_self_base = config.get('w_self_base', 0.2)
        w_cap = config.get('w_cap', 0.4)
        kappa = config.get('kappa', self.kappa)
        eta = config.get('eta', 0.15)
        enable_smoothing = config.get('enable_smoothing', True)
        
        # Initialize weights
        num_nodes = self.num_vehicles + 1  # +1 for virtual node
        weights = np.zeros(num_nodes)
        
        # Get trusted neighbors (trust > threshold)
        trust_scores_array = np.array(trust_scores)
        neighbors = np.where(self.graph[vehicle_index, :])[0]
        trusted_mask = trust_scores_array[neighbors] > self.trust_threshold
        trusted_neighbors = neighbors[trusted_mask]
        
        # Debug info
        debug_info = {
            'all_neighbors': neighbors.tolist(),
            'trusted_neighbors': trusted_neighbors.tolist(),
            'trust_values': {},
            'raw_weights': {},
            'final_weights': {}
        }
        
        # ═══════════════════════════════════════════════════════════
        # STEP 1: Calculate base weights from trust scores
        # ═══════════════════════════════════════════════════════════
        # Trust scores already incorporate quality metrics from Week 1
        # Fleet topology (vehicle_id) already encodes spatial relationships
        neighbor_weights = {}
        total_trust = 0.0
        
        for neighbor_idx in trusted_neighbors:
            trust = trust_scores_array[neighbor_idx]
            neighbor_weights[neighbor_idx] = trust
            total_trust += trust
            
            # Store debug info
            debug_info['trust_values'][int(neighbor_idx)] = float(trust)
            debug_info['raw_weights'][int(neighbor_idx)] = float(trust)
        
        # ═══════════════════════════════════════════════════════════
        # STEP 2: Normalize weights with fixed virtual and self weights
        # ═══════════════════════════════════════════════════════════
        # Fixed weights for virtual node and self
        weights[0] = w0_fixed
        weights[vehicle_index + 1] = w_self_base
        
        # Remaining weight budget for neighbors
        neighbor_budget = 1.0 - w0_fixed - w_self_base
        
        if total_trust > 0 and len(trusted_neighbors) > 0:
            # Distribute neighbor_budget proportionally to trust scores
            for neighbor_idx, trust in neighbor_weights.items():
                raw_weight = neighbor_budget * (trust / total_trust)
                
                # Apply influence capping
                final_weight = min(raw_weight, w_cap)
                weights[neighbor_idx + 1] = final_weight
                
                debug_info['final_weights'][int(neighbor_idx)] = float(final_weight)
        
        # ═══════════════════════════════════════════════════════════
        # STEP 3: Ensure row-stochastic (weights sum to 1)
        # ═══════════════════════════════════════════════════════════
        weight_sum = np.sum(weights)
        if weight_sum > 0:
            weights = weights / weight_sum
        
        # ═══════════════════════════════════════════════════════════
        # STEP 4: Apply EMA smoothing (temporal stability)
        # ═══════════════════════════════════════════════════════════
        if enable_smoothing and vehicle_index in self.prev_weights:
            prev_weights = self.prev_weights[vehicle_index]
            if prev_weights.shape == weights.shape:
                # EMA: w_new = η * w_current + (1-η) * w_prev
                weights = eta * weights + (1 - eta) * prev_weights
                
                # Re-normalize after smoothing
                weight_sum = np.sum(weights)
                if weight_sum > 0:
                    weights = weights / weight_sum
        
        # Store current weights for next iteration
        self.prev_weights[vehicle_index] = weights.copy()
        
        # # ═══════════════════════════════════════════════════════════
        # # STEP 5: Log weight calculation details
        # # ═══════════════════════════════════════════════════════════
        # if self.weight_logger:
        #     # Log configuration
        #     self.weight_logger.info(f"════════ Weight Calculation V2 ════════")
        #     self.weight_logger.info(f"Config: w0={w0_fixed:.3f}, w_self={w_self_base:.3f}, "
        #                            f"w_cap={w_cap:.3f}, eta={eta:.3f}")
            
        #     # Log trusted neighbors and their trust scores
        #     self.weight_logger.info(f"All neighbors: {neighbors.tolist()}")
        #     self.weight_logger.info(f"Trusted neighbors ({len(trusted_neighbors)}): {trusted_neighbors.tolist()}")
            
        #     for neighbor_idx in trusted_neighbors:
        #         trust = trust_scores_array[neighbor_idx]
        #         self.weight_logger.info(f"  Neighbor V{neighbor_idx}: trust={trust:.4f}")
            
        #     # Log weight distribution
        #     self.weight_logger.info(f"Virtual node (w0): {weights[0]:.4f}")
        #     self.weight_logger.info(f"Self weight: {weights[vehicle_index + 1]:.4f}")
            
        #     for neighbor_idx in trusted_neighbors:
        #         nbr_weight = weights[neighbor_idx + 1]
        #         self.weight_logger.info(f"  Neighbor V{neighbor_idx}: {nbr_weight:.4f}")
            
        #     # Log weight sum and smoothing
        #     weight_sum_before = np.sum(weights)
        #     self.weight_logger.info(f"Weight sum (before normalization): {weight_sum_before:.6f}")
            
        #     if enable_smoothing and vehicle_index in self.prev_weights:
        #         self.weight_logger.info(f"EMA smoothing applied (eta={eta:.3f})")
        
        # ═══════════════════════════════════════════════════════════
        # STEP 6: Return results
        # ═══════════════════════════════════════════════════════════
        weights_reshaped = weights.reshape(1, -1)
        
        debug_info['weight_sum'] = float(np.sum(weights))
        debug_info['config'] = config
        
        return {
            'weights': weights_reshaped,
            'trusted_neighbors': trusted_neighbors.tolist(),
            'debug_info': debug_info
        }

    def generate_virtual_graph(self, graph, vehicle_index):
        """
        Generate a virtual graph by adding an extra node (node 0) and connecting it
        to the specified vehicle and its neighbors.
        
        Parameters:
            graph         - Adjacency matrix of the original graph
            vehicle_index - Index of the vehicle to which node 0 will be connected (0-based)
            
        Returns:
            Vj           - Virtual graph adjacency matrix with an extra node
        """
        # Initialize virtual graph with an extra node
        Vj = np.zeros((self.num_vehicles + 1, self.num_vehicles + 1))
        
        # Copy the adjacency matrix to the virtual graph (shift by 1 to account for virtual node)
        Vj[1:, 1:] = graph
        
        # Set the bidirectional edge between node 0 and the specified vehicle
        Vj[0, vehicle_index + 1] = 1
        Vj[vehicle_index + 1, 0] = 1
        
        # Set the edges between node 0 and vehicle's neighbors
        neighbors = np.where(graph[vehicle_index, :])[0]
        for neighbor in neighbors:
            Vj[0, neighbor + 1] = graph[vehicle_index, neighbor]
            Vj[neighbor + 1, 0] = graph[neighbor, vehicle_index]
        
        return Vj
