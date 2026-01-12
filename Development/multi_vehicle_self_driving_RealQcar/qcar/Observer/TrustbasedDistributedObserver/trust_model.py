"""
TriP Trust Model - Trust-based Intelligent Platoon

Implements the TriP (Trust-based Intelligent Platoon) model for evaluating 
the trustworthiness of neighboring vehicles based on multiple independent metrics.

This module provides:
1. Multi-component trust evaluation (velocity, distance, acceleration, heading)
2. Communication quality assessment
3. Dirichlet-based trust level updates
4. Attack detection flags

Reference: Trust-Based Distributed Observer Framework
"""
import numpy as np
from typing import Dict, List, Optional, Tuple
from collections import defaultdict
from dataclasses import dataclass, field
import time


@dataclass
class TrustConfig:
    """Configuration for trust model parameters"""
    # Trust component weights
    weight_velocity: float = 12.0
    weight_distance: float = 2.0
    weight_acceleration: float = 0.3
    weight_heading: float = 1.0
    
    # Trust thresholds
    trust_threshold: float = 0.5
    
    # Dirichlet parameters
    num_trust_levels: int = 5
    dirichlet_update_rate: float = 0.1
    dirichlet_type: str = "Dual"  # "Single" or "Dual"
    
    # Attack detection
    monitor_sudden_change: bool = True
    sudden_change_threshold: float = 0.5
    attack_detection_window: int = 10
    
    # Communication quality
    max_message_age_s: float = 1.0
    expected_beacon_interval_s: float = 0.1
    
    # EMA smoothing
    ema_alpha: float = 0.3


@dataclass  
class VehicleData:
    """Data structure for received vehicle information"""
    vehicle_id: int
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    velocity: float = 0.0
    acceleration: float = 0.0
    timestamp_ns: int = 0
    
    # Derived metrics
    distance_from_host: float = 0.0
    relative_heading: float = 0.0


@dataclass
class TrustScore:
    """Trust score output for a vehicle"""
    vehicle_id: int
    final_score: float = 1.0
    
    # Component scores
    velocity_score: float = 1.0
    distance_score: float = 1.0
    acceleration_score: float = 1.0
    heading_score: float = 1.0
    beacon_score: float = 1.0
    quality_factor: float = 1.0
    
    # Local and global trust samples
    local_trust_sample: float = 1.0
    global_trust_sample: float = 1.0
    
    # Trust rating vector (5 levels)
    trust_levels: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.2, 0.4, 0.4]))
    
    # Attack flags
    flag_target_attack: bool = False
    flag_global_est_check: bool = False
    flag_local_est_check: bool = False
    
    timestamp: float = 0.0


class TriPTrustModel:
    """
    TriP (Trust-based Intelligent Platoon) Trust Model
    
    Evaluates the trustworthiness of neighboring vehicles using multiple
    independent metrics and Dirichlet-based trust level updates.
    """
    
    def __init__(self, vehicle_id: int, config: TrustConfig = None, logger=None):
        """
        Initialize TriP Trust Model
        
        Args:
            vehicle_id: ID of the host vehicle
            config: Trust configuration parameters
            logger: Logger instance
        """
        self.vehicle_id = vehicle_id
        self.config = config or TrustConfig()
        self.logger = logger
        
        # Trust scores storage: vehicle_id -> TrustScore
        self.trust_scores: Dict[int, TrustScore] = {}
        
        # Historical data for trend analysis
        self.trust_history: Dict[int, List[float]] = defaultdict(list)
        self.max_history_size = 50
        
        # Last received data timestamps for beacon quality
        self.last_beacon_times: Dict[int, float] = {}
        self.beacon_drop_counts: Dict[int, int] = defaultdict(int)
        self.beacon_receive_counts: Dict[int, int] = defaultdict(int)
        
        # Cross-check data from neighbors (for gamma_cross)
        self.neighbor_trust_reports: Dict[int, Dict[int, float]] = defaultdict(dict)
        # neighbor_trust_reports[reporter_id][target_id] = trust_score
        
    def calculate_trust(self, 
                        host_state: Dict,
                        target_data: VehicleData,
                        leader_data: Optional[VehicleData] = None,
                        neighbor_estimates: Optional[Dict[int, VehicleData]] = None,
                        current_time_ns: int = None) -> TrustScore:
        """
        Calculate comprehensive trust score for a target vehicle
        
        Args:
            host_state: Host vehicle state dict {x, y, theta, velocity, acceleration}
            target_data: Data received from target vehicle
            leader_data: Optional leader vehicle data (for reference)
            neighbor_estimates: Optional estimates from neighbors about target
            current_time_ns: Current time in nanoseconds
            
        Returns:
            TrustScore with all components and final score
        """
        if current_time_ns is None:
            current_time_ns = int(time.time() * 1e9)
        
        target_id = target_data.vehicle_id
        
        # Initialize or get existing trust score
        if target_id not in self.trust_scores:
            self.trust_scores[target_id] = TrustScore(vehicle_id=target_id)
        
        trust = self.trust_scores[target_id]
        trust.timestamp = current_time_ns / 1e9
        
        # === Calculate Component Scores ===
        
        # 1. Velocity Score
        trust.velocity_score = self._calculate_velocity_score(
            host_state, target_data, leader_data
        )
        
        # 2. Distance Score
        trust.distance_score = self._calculate_distance_score(
            host_state, target_data
        )
        
        # 3. Acceleration Score
        trust.acceleration_score = self._calculate_acceleration_score(
            host_state, target_data
        )
        
        # 4. Heading Score
        trust.heading_score = self._calculate_heading_score(
            host_state, target_data
        )
        
        # 5. Beacon Score (binary 0/1)
        trust.beacon_score = self._calculate_beacon_score(
            target_id, current_time_ns
        )
        
        # 6. Communication Quality Factor
        trust.quality_factor = self._calculate_quality_factor(
            target_data, current_time_ns
        )
        
        # === Calculate Local Trust Sample (gamma_local) ===
        # Weighted combination of component scores
        trust.local_trust_sample = self._compute_local_trust_sample(trust)
        
        # === Calculate Global Trust Sample (gamma_cross) ===
        # Cross-check with neighbor reports
        trust.global_trust_sample = self._compute_global_trust_sample(
            target_id, neighbor_estimates
        )
        
        # === Update Dirichlet Trust Levels ===
        trust.trust_levels = self._update_trust_levels(
            target_id, trust.local_trust_sample, trust.global_trust_sample
        )
        
        # === Compute Final Score from Trust Levels ===
        trust.final_score = self._compute_final_score(trust.trust_levels)
        
        # === Set Attack Detection Flags ===
        self._set_attack_flags(trust)
        
        # Store in history
        self._update_history(target_id, trust.final_score)
        
        return trust
    
    def _calculate_velocity_score(self, host_state: Dict, 
                                   target_data: VehicleData,
                                   leader_data: Optional[VehicleData]) -> float:
        """
        Calculate velocity consistency score
        
        v_score = max(1 - |v_target - v_ref| / v_ref, 0)^w_v
        """
        v_target = target_data.velocity
        
        # Use leader velocity as reference, or host velocity
        if leader_data is not None:
            v_ref = max(leader_data.velocity, 0.1)  # Avoid division by zero
        else:
            v_ref = max(host_state.get('velocity', 0.5), 0.1)
        
        # Compute normalized error
        v_error = abs(v_target - v_ref)
        normalized_error = v_error / v_ref
        
        # Apply weight exponent
        score = max(1.0 - normalized_error, 0.0) ** self.config.weight_velocity
        
        return float(np.clip(score, 0.0, 1.0))
    
    def _calculate_distance_score(self, host_state: Dict, 
                                   target_data: VehicleData) -> float:
        """
        Calculate distance consistency score
        
        Validates that reported position matches observed distance.
        d_score = max(1 - |d_reported - d_measured| / d_measured, 0)^w_d
        """
        # Calculate distance from host to target based on reported positions
        dx = target_data.x - host_state.get('x', 0.0)
        dy = target_data.y - host_state.get('y', 0.0)
        d_reported = np.sqrt(dx**2 + dy**2)
        
        # For now, use reported as "measured" (in real system, use sensor measurement)
        # This can be enhanced with actual sensor distance measurement
        d_measured = max(d_reported, 0.1)  # Placeholder
        
        # Calculate error (this becomes meaningful when d_measured comes from sensors)
        d_error = abs(d_reported - d_measured)
        normalized_error = d_error / d_measured
        
        score = max(1.0 - normalized_error, 0.0) ** self.config.weight_distance
        
        return float(np.clip(score, 0.0, 1.0))
    
    def _calculate_acceleration_score(self, host_state: Dict,
                                        target_data: VehicleData) -> float:
        """
        Calculate acceleration consistency score
        
        Checks if acceleration is physically reasonable.
        a_score = max(1 - |d_add * delta_a|, 0)^w_a
        """
        a_target = target_data.acceleration
        
        # Maximum reasonable acceleration for QCar (m/s^2)
        a_max = 3.0
        
        # Normalize acceleration
        normalized_a = abs(a_target) / a_max
        
        score = max(1.0 - normalized_a, 0.0) ** self.config.weight_acceleration
        
        return float(np.clip(score, 0.0, 1.0))
    
    def _calculate_heading_score(self, host_state: Dict,
                                  target_data: VehicleData) -> float:
        """
        Calculate heading consistency score
        
        Validates heading from trajectory.
        h_score = max(1 - theta_diff / theta_max, 0)
        """
        theta_target = target_data.theta
        theta_host = host_state.get('theta', 0.0)
        
        # Calculate heading difference (normalized to [-pi, pi])
        theta_diff = np.arctan2(
            np.sin(theta_target - theta_host),
            np.cos(theta_target - theta_host)
        )
        
        # Maximum expected heading difference
        theta_max = np.pi / 2  # 90 degrees
        
        normalized_diff = abs(theta_diff) / theta_max
        score = max(1.0 - normalized_diff, 0.0)
        
        return float(np.clip(score, 0.0, 1.0))
    
    def _calculate_beacon_score(self, target_id: int, 
                                 current_time_ns: int) -> float:
        """
        Calculate beacon reception score (binary)
        
        Returns 1.0 if beacon received within expected interval, 0.0 otherwise.
        """
        current_time_s = current_time_ns / 1e9
        
        if target_id not in self.last_beacon_times:
            return 0.0
        
        last_time = self.last_beacon_times[target_id]
        age = current_time_s - last_time
        
        # Check if within expected interval (with some tolerance)
        if age <= self.config.expected_beacon_interval_s * 3:
            return 1.0
        return 0.0
    
    def _calculate_quality_factor(self, target_data: VehicleData,
                                   current_time_ns: int) -> float:
        """
        Calculate communication quality factor
        
        q = exp(-2 * age) * (1 - drop_rate) * 1/(1 + 0.2 * covariance)
        """
        current_time_s = current_time_ns / 1e9
        target_id = target_data.vehicle_id
        
        # Age factor
        message_age_s = (current_time_ns - target_data.timestamp_ns) / 1e9
        age_factor = np.exp(-2.0 * message_age_s)
        
        # Drop rate factor
        total = self.beacon_receive_counts[target_id] + self.beacon_drop_counts[target_id]
        if total > 0:
            drop_rate = self.beacon_drop_counts[target_id] / total
        else:
            drop_rate = 0.0
        drop_factor = 1.0 - drop_rate
        
        # Covariance factor (placeholder - use 0 for now)
        covariance = 0.0
        cov_factor = 1.0 / (1.0 + 0.2 * covariance)
        
        quality = age_factor * drop_factor * cov_factor
        
        return float(np.clip(quality, 0.0, 1.0))
    
    def _compute_local_trust_sample(self, trust: TrustScore) -> float:
        """
        Compute local trust sample (gamma_local) from component scores
        
        Weighted geometric mean of component scores
        """
        # Weights for each component (normalized)
        weights = {
            'velocity': 0.3,
            'distance': 0.2,
            'acceleration': 0.15,
            'heading': 0.15,
            'beacon': 0.1,
            'quality': 0.1
        }
        
        # Compute weighted product
        scores = [
            (trust.velocity_score, weights['velocity']),
            (trust.distance_score, weights['distance']),
            (trust.acceleration_score, weights['acceleration']),
            (trust.heading_score, weights['heading']),
            (trust.beacon_score, weights['beacon']),
            (trust.quality_factor, weights['quality'])
        ]
        
        weighted_product = 1.0
        total_weight = 0.0
        
        for score, weight in scores:
            if score > 0:
                weighted_product *= score ** weight
                total_weight += weight
        
        if total_weight > 0:
            local_sample = weighted_product ** (1.0 / total_weight)
        else:
            local_sample = 0.5  # Neutral if no valid scores
        
        return float(np.clip(local_sample, 0.0, 1.0))
    
    def _compute_global_trust_sample(self, target_id: int,
                                      neighbor_estimates: Optional[Dict[int, VehicleData]]) -> float:
        """
        Compute global trust sample (gamma_cross) from neighbor reports
        
        Cross-validation with what neighbors report about target
        """
        if not self.neighbor_trust_reports or target_id not in self.trust_scores:
            return 0.5  # Neutral if no cross-check data
        
        # Collect trust scores from neighbors about target
        neighbor_scores = []
        for reporter_id, reports in self.neighbor_trust_reports.items():
            if target_id in reports:
                neighbor_scores.append(reports[target_id])
        
        if not neighbor_scores:
            return 0.5
        
        # Average neighbor trust scores
        global_sample = np.mean(neighbor_scores)
        
        return float(np.clip(global_sample, 0.0, 1.0))
    
    def _update_trust_levels(self, target_id: int, 
                              gamma_local: float, 
                              gamma_cross: float) -> np.ndarray:
        """
        Update Dirichlet-based trust levels
        
        5 trust levels: [Untrusted, Suspicious, Neutral, Trusted, Highly Trusted]
        """
        # Get current trust levels
        if target_id in self.trust_scores:
            current_levels = self.trust_scores[target_id].trust_levels.copy()
        else:
            # Initialize with neutral distribution
            current_levels = np.array([0.0, 0.0, 0.2, 0.4, 0.4])
        
        # Combine local and global samples
        if self.config.dirichlet_type == "Dual":
            combined_sample = 0.6 * gamma_local + 0.4 * gamma_cross
        else:
            combined_sample = gamma_local
        
        # Map sample to level index (0-4)
        level_idx = int(combined_sample * (self.config.num_trust_levels - 1))
        level_idx = np.clip(level_idx, 0, self.config.num_trust_levels - 1)
        
        # Update levels using EMA-like update
        update_rate = self.config.dirichlet_update_rate
        update_vector = np.zeros(self.config.num_trust_levels)
        update_vector[level_idx] = 1.0
        
        new_levels = (1 - update_rate) * current_levels + update_rate * update_vector
        
        # Normalize to sum to 1
        new_levels = new_levels / np.sum(new_levels)
        
        return new_levels
    
    def _compute_final_score(self, trust_levels: np.ndarray) -> float:
        """
        Compute final trust score from trust level distribution
        
        Weighted sum: level_value * level_probability
        """
        # Level values: [0.1, 0.3, 0.5, 0.7, 0.9]
        level_values = np.array([0.1, 0.3, 0.5, 0.7, 0.9])
        
        final_score = np.dot(trust_levels, level_values)
        
        return float(np.clip(final_score, 0.0, 1.0))
    
    def _set_attack_flags(self, trust: TrustScore):
        """
        Set attack detection flags based on trust samples
        """
        # Flag: Target may be under attack
        # High local trust but low cross-validation
        trust.flag_target_attack = (
            trust.local_trust_sample > 0.5 and 
            trust.global_trust_sample < 0.5
        )
        
        # Flag: Global estimate needs verification
        # Low local trust but high cross-validation
        trust.flag_global_est_check = (
            trust.local_trust_sample < 0.5 and 
            trust.global_trust_sample > 0.5
        )
        
        # Flag: Local measurements unreliable
        trust.flag_local_est_check = trust.local_trust_sample < 0.5
    
    def _update_history(self, target_id: int, score: float):
        """Update trust score history for trend analysis"""
        self.trust_history[target_id].append(score)
        
        # Limit history size
        if len(self.trust_history[target_id]) > self.max_history_size:
            self.trust_history[target_id] = self.trust_history[target_id][-self.max_history_size:]
    
    def update_beacon_reception(self, target_id: int, received: bool,
                                 timestamp_s: float):
        """
        Update beacon reception tracking for quality calculation
        
        Args:
            target_id: Vehicle ID
            received: Whether beacon was received
            timestamp_s: Reception timestamp in seconds
        """
        if received:
            self.last_beacon_times[target_id] = timestamp_s
            self.beacon_receive_counts[target_id] += 1
        else:
            self.beacon_drop_counts[target_id] += 1
    
    def add_neighbor_trust_report(self, reporter_id: int, target_id: int, 
                                   trust_score: float):
        """
        Add trust report from a neighbor about a target
        
        Used for cross-validation (gamma_cross)
        """
        self.neighbor_trust_reports[reporter_id][target_id] = trust_score
    
    def get_trust_score(self, target_id: int) -> Optional[TrustScore]:
        """Get current trust score for a vehicle"""
        return self.trust_scores.get(target_id)
    
    def get_all_trust_scores(self) -> Dict[int, float]:
        """Get all trust scores as dict {vehicle_id: final_score}"""
        return {vid: ts.final_score for vid, ts in self.trust_scores.items()}
    
    def get_trusted_vehicles(self, threshold: float = None) -> List[int]:
        """Get list of vehicle IDs with trust above threshold"""
        if threshold is None:
            threshold = self.config.trust_threshold
        
        return [vid for vid, ts in self.trust_scores.items() 
                if ts.final_score >= threshold]
    
    def is_vehicle_trusted(self, target_id: int, threshold: float = None) -> bool:
        """Check if a specific vehicle is trusted"""
        if threshold is None:
            threshold = self.config.trust_threshold
        
        if target_id in self.trust_scores:
            return self.trust_scores[target_id].final_score >= threshold
        return False
    
    def get_attack_flags(self) -> Dict[int, Dict[str, bool]]:
        """Get all attack detection flags"""
        flags = {}
        for vid, ts in self.trust_scores.items():
            flags[vid] = {
                'target_attack': ts.flag_target_attack,
                'global_est_check': ts.flag_global_est_check,
                'local_est_check': ts.flag_local_est_check
            }
        return flags
    
    def reset(self):
        """Reset all trust scores and history"""
        self.trust_scores.clear()
        self.trust_history.clear()
        self.last_beacon_times.clear()
        self.beacon_drop_counts.clear()
        self.beacon_receive_counts.clear()
        self.neighbor_trust_reports.clear()
