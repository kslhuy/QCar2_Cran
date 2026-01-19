import numpy as np
import math
from typing import Optional, Dict, List, Any, Tuple
import logging


class TriPTrustModel:
    """
    Python implementation of the TriPTrustModel for trust evaluation in vehicle platoons.
    
    This class evaluates trust between vehicles based on velocity, distance, acceleration, 
    heading, and beacon reception metrics. It supports both local and global trust evaluation
    with anomaly detection and sudden change monitoring.
    
    Overview: https://discord.com/channels/1123389035713400902/1326223031667789885/1327306982218141727
    """
    
    def __init__(self):
        """Initialize the TriPTrustModel with default parameters."""
        
        # Weights for different trust components
        self.wv = 12.0     # Weight for velocity
        self.wd = 2.0      # Weight for distance
        self.wa = 0.3      # Weight for acceleration
        self.wj = 1.0      # Weight for jerkiness
        self.wh = 1.0      # Weight for heading
        
        # Weights for nearby vehicles (stricter evaluation)
        self.wv_nearby = 12.0
        self.wd_nearby = 8.0
        self.wa_nearby = 0.3
        self.wj_nearby = 1.0
        self.wh_nearby = 1.0
        
        # Trust decay weights
        self.wt = 0.1         # Trust decay weight for local
        self.wt_global = 0.5  # Trust decay weight for global
        
        # Model parameters
        self.C = 0.2          # Regularization constant
        self.tacc = 1.2       # Trust-based acceleration scaling factor
        self.k = 5            # Number of trust levels
        
        # Trust rating vectors
        self.rating_vector = np.zeros(self.k)
        self.rating_vector[4] = 1  # Initialize to highest trust level
        
        self.rating_vector_global = np.zeros(self.k)
        self.rating_vector_global[4] = 1
        
        # Score components
        self.alpha_v_score = 0.7  # Weight for velocity score
        
        # Global estimate check parameters
        self.sigma2 = 1.0     # Sensitivity parameter for cross-validation trust factor
        self.tau2 = 0.5       # Sensitivity parameter for local consistency trust factor
        
        # Distance and time tracking
        self.last_d = 20.0
        self.last_time_d = 0
        self.Period_a_score_distane = 10
        self.buffer_size = 5
        self.distance_buffer = np.zeros(self.buffer_size)
        
        # Logging arrays for analysis
        self.trust_sample_log = []
        self.gamma_cross_log = []
        self.gamma_local_log = []
        self.gamma_expected_log = []
        self.v_score_log = []
        self.d_score_log = []
        self.a_score_log = []
        self.h_score_log = []
        self.beacon_score_log = []
        self.final_score_log = []
        
        # Flag logs
        self.flag_glob_est_check_log = []
        self.flag_taget_attk_log = []
        self.flag_local_est_check_log = []
        
        # Flags
        self.flag_taget_attk = False
        self.flag_glob_est_check = False
        self.flag_local_est_check = False
        
        # Leader state tracking
        self.lead_state_lastest = np.zeros(4)
        self.lead_state_lastest_timestamp = 0
        
        # Anomaly detection logs
        self.D_pos_log = []
        self.D_vel_log = []
        self.anomaly_pos_log = []
        self.anomaly_vel_log = []
        self.anomaly_gamma_log = []
        self.anomaly_acc_log = []
        
        # Anomaly detection parameters
        self.w = 10               # Sliding window size
        self.Threshold_anomalie = 3   # Threshold for cumulative anomalies
        self.reduce_factor = 0.5  # Trust reduction factor
        
        # Previous state for heading evaluation
        self.previous_state = np.zeros(4)
        
        # Covariance matrices
        tau2_diag_element = [1.5, 0.5]
        self.tau2_matrix_gamma_local = np.diag(tau2_diag_element)
        
        sigma2_diag_element = [1.5, 1.0, 0.01, 0.5, 0.1]
        self.sigma2_matrix_gamma_cross = np.diag(sigma2_diag_element)
        
        # Additional logging arrays
        self.v_rel_log = []
        self.d_add_log = []
        self.acc_rel_log = []
        self.delta_acc_expected_log = []
        self.distance_log = []
        self.scale_d_expected_log = []
        self.delta_d_log = []
        
        # Acceleration score component logs
        self.a_score_expected_diff_acc_log = []
        self.a_score_diff_acc_log = []
        self.a_score_vrel_dis_log = []
        self.a_score_defaut_log = []
        
        # Quality factor logs (NEW for communication quality)
        self.age_factor_log = []
        self.reliability_factor_log = []
        self.uncertainty_factor_log = []
        self.consistency_factor_log = []
        self.quality_factor_log = []
    
    def evaluate_velocity(self, host_id: int, target_id: int, v_y: float, v_host: float,
                         v_leader: float, a_leader: float, b_leader: float, 
                         is_nearby: bool, tolerance: float = 0.1) -> float:
        """
        Evaluate velocity trust score with tolerance range for minor deviations.
        
        Args:
            host_id: Host vehicle ID
            target_id: Target vehicle ID
            v_y: Reported velocity of the target vehicle
            v_host: Host vehicle velocity
            v_leader: Leader vehicle velocity
            a_leader: Leader vehicle acceleration
            b_leader: Beacon interval (time since last beacon)
            is_nearby: Whether target is a nearby vehicle
            tolerance: Acceptable fraction of v_ref deviation without penalty
            
        Returns:
            Velocity trust score (0 to 1)
        """
        # Get current leader velocity with predicted motion
        # v_ref = v_leader + b_leader * a_leader
        v_ref = v_leader 

        
        # Determine alpha based on relative position
        if (host_id - target_id) > 0:  # Target is ahead of host
            alpha = self.alpha_v_score
        else:
            alpha = 1 - self.alpha_v_score  # Target is following host
        
        # Handle edge cases for stopped or braking scenarios
        if (v_ref == 0 or v_leader == 0 or 
            np.sign(v_ref * v_leader) < 0 or np.sign(v_ref * v_host) < 0):
            v_score_final = max(1 - abs(v_y), 0)
            if is_nearby:
                return v_score_final ** self.wv_nearby
            else:
                return v_score_final ** self.wv
        
        # Calculate deviations
        deviation_ref = (abs(v_y - v_ref) + 1) / v_ref
        deviation_host = abs(v_y - v_host) / v_host if v_host != 0 else 0
        
        # Calculate reference score
        if deviation_ref <= tolerance:
            v_score_ref = 1.0
        else:
            scaled_penalty = (deviation_ref - tolerance) / (1 - tolerance)
            v_score_ref = max(1 - scaled_penalty, 0)
        
        # Calculate host score
        v_score_host = max(1 - deviation_host, 0)
        
        # Combine scores
        v_score_final = (1 - alpha) * v_score_ref + alpha * v_score_host
        
        # Apply weighting based on proximity
        if is_nearby:
            return v_score_final ** self.wv_nearby
        else:
            return v_score_final ** self.wv
        

    def evaluate_velocity_no_Accel(self, host_id: int, target_id: int, v_y: float, v_host: float,
                         v_leader: float, a_leader: float, b_leader: float, 
                         is_nearby: bool, tolerance: float = 0.1) -> float:
        """
        Evaluate velocity trust score with tolerance range for minor deviations.

        Args:
            host_id: Host vehicle ID
            target_id: Target vehicle ID
            v_y: Reported velocity of the target vehicle
            v_host: Host vehicle velocity
            v_leader: Leader vehicle velocity
            a_leader: Leader vehicle acceleration
            b_leader: Beacon interval (time since last beacon)
            is_nearby: Whether target is a nearby vehicle
            tolerance: Acceptable fraction of v_ref deviation without penalty
            
        Returns:
            Velocity trust score (0 to 1)
        """
        # Get current leader velocity with predicted motion
        v_ref = v_leader 
        
        # Determine alpha based on relative position
        if (host_id - target_id) > 0:  # Target is ahead of host
            alpha = self.alpha_v_score
        else:
            alpha = 1 - self.alpha_v_score  # Target is following host
        
        # Handle edge cases for stopped or braking scenarios
        if (v_ref == 0 or v_leader == 0 or 
            np.sign(v_ref * v_leader) < 0 or np.sign(v_ref * v_host) < 0):
            v_score_final = max(1 - abs(v_y), 0)
            if is_nearby:
                return v_score_final ** self.wv_nearby
            else:
                return v_score_final ** self.wv
        
        # Calculate deviations
        deviation_ref = (abs(v_y - v_ref) + 1) / v_ref
        deviation_host = abs(v_y - v_host) / v_host if v_host != 0 else 0
        
        # Calculate reference score
        if deviation_ref <= tolerance:
            v_score_ref = 1.0
        else:
            scaled_penalty = (deviation_ref - tolerance) / (1 - tolerance)
            v_score_ref = max(1 - scaled_penalty, 0)
        
        # Calculate host score
        v_score_host = max(1 - deviation_host, 0)
        
        # Combine scores
        v_score_final = (1 - alpha) * v_score_ref + alpha * v_score_host
        
        # Apply weighting based on proximity
        if is_nearby:
            return v_score_final ** self.wv_nearby
        else:
            return v_score_final ** self.wv
    
    def evaluate_distance(self, d_y: float, d_measured: float, is_nearby: bool) -> float:
        """
        Evaluate distance trust score.
        
        Args:
            d_y: Reported distance
            d_measured: Measured distance
            is_nearby: Whether target is a nearby vehicle
            
        Returns:
            Distance trust score (0 to 1)
        """
        if d_measured == 0:
            return 0.0
            
        relative_error = abs((d_y - d_measured) / d_measured)
        base_score = max(1 - relative_error, 0)
        
        if is_nearby:
            return base_score ** self.wd_nearby
        else:
            return base_score ** self.wd
    
    def evaluate_acceleration(self, host_vehicle: Any, host_id: int, target_id: int,
                            a_y: float, a_host: float, d: List[float], 
                            ts: float, is_nearby: bool) -> float:
        """
        Evaluate acceleration trust score based on relative acceleration and expected dynamics.
        
        Args:
            host_vehicle: Host vehicle object containing parameters
            host_id: Host vehicle ID
            target_id: Target vehicle ID
            a_y: Reported acceleration of target vehicle
            a_host: Host vehicle acceleration
            d: Distance measurements [current, previous]
            ts: Time step
            is_nearby: Whether target is a nearby vehicle
            
        Returns:
            Acceleration trust score (0 to 1)
        """
        # Update distance buffer (shift left and add new distance)
        self.distance_buffer = np.roll(self.distance_buffer, 1)
        self.distance_buffer[0] = d[0]
        
        # Calculate relative velocity and acceleration
        if np.all(self.distance_buffer != 0):  # Buffer is filled
            v_rel = (self.distance_buffer[-2] - self.distance_buffer[0]) / ((self.buffer_size - 1) * ts)
            v_rel_m1 = (self.distance_buffer[-1] - self.distance_buffer[1]) / ((self.buffer_size - 1) * ts)
            expected_a_relative_diff = (v_rel - v_rel_m1) / ts
        else:
            v_rel = (d[1] - d[0]) / ts if len(d) > 1 else 0
            expected_a_relative_diff = 0
        
        # Normalization factor
        d_norm = 19.0
        
        # Calculate relative acceleration difference
        a_relative_diff = a_y - a_host
        
        # Expected acceleration calculations
        a_y_expected = a_host + expected_a_relative_diff
        delta_acc = a_y_expected - a_y
        
        # Calculate d_add based on proximity and vehicle dynamics
        constance_regulation = 1.0
        
        if not is_nearby:
            d_add = d_norm * abs(target_id - host_id) / d[0]
        else:
            if target_id > 1:
                # Calculate sign mismatch for nearby vehicles
                sign_mismatch_relative = (np.sign(a_host) != np.sign(a_relative_diff))
                
                if sign_mismatch_relative == 0:
                    constance_regulation = 0.7  # Sign consistent
                else:
                    constance_regulation = 0.8  # Sign inconsistent
                
                d_add = d_norm * constance_regulation / d[0]
            else:
                d_add = d_norm * constance_regulation / d[0]
        
        # Calculate expected distance based on controller type
        if hasattr(host_vehicle, 'Param_opt') and hasattr(host_vehicle, 'observer'):
            hi = host_vehicle.Param_opt.hi  # Time gap
            ri = host_vehicle.Param_opt.ri  # Minimum gap distance
            v_local_host = host_vehicle.observer.est_local_state_current[3]  # velocity index 3 in Python (0-based)
            v_0 = host_vehicle.Param_opt.v0  # Desired velocity in free flow
            delta = host_vehicle.Param_opt.delta
            
            if hasattr(host_vehicle, 'scenarios_config'):
                controller_type = host_vehicle.scenarios_config.controller_type
                
                if controller_type == "mix":
                    s_CACC_expected = ri + hi * v_local_host / (host_id - 1)
                    s_IDM_expected = ri + hi * v_local_host / math.sqrt(1 - (v_local_host / v_0) ** delta)
                    gamma_control = getattr(host_vehicle, 'gamma', 0.5)
                    d_expected = (1 - gamma_control) * s_CACC_expected + gamma_control * s_IDM_expected
                elif controller_type == "local":
                    d_expected = (ri + hi * v_local_host) / math.sqrt(1 - (v_local_host / v_0) ** delta)
                else:  # "coop"
                    d_expected = ri + hi * v_local_host / (host_id - 1)
            else:
                d_expected = ri + hi * v_local_host / (host_id - 1)
        else:
            d_expected = 20.0  # Default expected distance
        
        # Calculate scenario-based adjustments
        scale_d_expected = d[0] / d_expected if d_expected != 0 else 1.0
        delta_d = d[0] - d_expected
        
        # Define scenario flags
        target_decelerating = a_y < 0
        host_decelerating = a_host < 0
        both_accelerating = a_y > 0 and a_host > 0
        both_decelerating = a_y < 0 and a_host < 0
        target_acc_host_dec = a_y > 0 and a_host < 0
        
        # Apply scenario-based adjustments
        d_add_scale = 1.0
        
        if target_decelerating:
            d_add_scale *= 0.8
        if both_decelerating and delta_d < 0:
            d_add_scale *= 0.7
        if target_acc_host_dec and delta_d < 0:
            d_add_scale *= 0.8
        if (both_accelerating or both_decelerating) and abs(delta_d) > 0.2 * d_expected:
            d_add_scale *= 0.9
        
        d_add_adjusted = d_add * d_add_scale
        
        # Calculate acceleration scores using different methods
        a_score_expected_diff_acc = max(1 - abs(d_add_adjusted * delta_acc), 0)
        a_score_diff_acc = max(1 - abs(v_rel / d_add_adjusted * a_relative_diff), 0) if d_add_adjusted != 0 else 0
        a_score_vrel_dis = max(1 - abs(v_rel / d[0] * a_relative_diff), 0) if d[0] != 0 else 0
        a_score_defaut = max(1 - abs(v_rel / ((self.buffer_size - 1) * ts) * a_relative_diff), 0)
        
        # Use expected acceleration difference score as primary metric
        a_score = a_score_expected_diff_acc
        
        # Apply weighting based on proximity
        if is_nearby:
            a_score = a_score ** self.wa_nearby
        else:
            a_score = a_score ** self.wa
        
        # Log data for analysis
        self.v_rel_log.append(v_rel)
        self.distance_log.append(d[0])
        self.d_add_log.append(d_add_adjusted)
        self.acc_rel_log.append(a_relative_diff)
        self.delta_acc_expected_log.append(delta_acc)
        self.scale_d_expected_log.append(scale_d_expected)
        self.delta_d_log.append(delta_d)
        
        self.a_score_expected_diff_acc_log.append(a_score_expected_diff_acc)
        self.a_score_diff_acc_log.append(a_score_diff_acc)
        self.a_score_vrel_dis_log.append(a_score_vrel_dis)
        self.a_score_defaut_log.append(a_score_defaut)
        
        return a_score
    
    def evaluate_jerkiness(self, j_y: float, j_thresh: float = 2.0) -> float:
        """
        Evaluate jerkiness trust score.
        
        Args:
            j_y: Reported jerk value
            j_thresh: Jerk threshold
            
        Returns:
            Jerkiness trust score (0 to 1)
        """
        if abs(j_y) > j_thresh:
            return min(j_thresh / abs(j_y), 1)
        else:
            return 1.0
    
    def evaluate_beacon_timeout(self, beacon_received: bool) -> float:
        """
        Evaluate beacon reception trust score.
        
        Args:
            beacon_received: Whether beacon was received
            
        Returns:
            Beacon trust score (0 or 1)
        """
        return 1.0 if beacon_received else 0.0
    

    def evaluate_communication_quality(self, message_age: float, drop_rate: float, 
                                       covariance: float, innovation: float) -> float:
        """
        Evaluate communication quality factor for trust calculation.
        
        This factor reduces trust when:
        - Messages are stale (high age)
        - Packet loss is high (high drop_rate)
        - Uncertainty is high (high covariance)
        - Prediction error is high (high innovation)
        
        Args:
            message_age: Time since last message (seconds)
            drop_rate: Packet loss rate [0, 1]
            covariance: Covariance trace (uncertainty)
            innovation: Prediction error (meters)
            
        Returns:
            Quality factor [0, 1] where 1 = perfect quality
        """
        # Age penalty (exponential decay - message freshness)
        age_factor = np.exp(-2.0 * message_age)  # 2.0 = decay rate (half-life ~0.35s)
        
        # Drop rate penalty (linear - reliability)
        reliability_factor = 1.0 - drop_rate
        
        # Covariance penalty (inverse relationship - uncertainty)
        uncertainty_factor = 1.0 / (1.0 + 0.2 * covariance)  # 0.2 = penalty weight
        
        # Innovation penalty (prediction accuracy - consistency)
        consistency_factor = np.exp(-0.5 * innovation)  # 0.5 = sensitivity (half-life ~1.4m)
        
        # Combine all factors (multiplicative - any bad factor reduces trust)
        quality_factor = age_factor * reliability_factor * uncertainty_factor * consistency_factor
        
        # Log components for debugging
        self.age_factor_log.append(age_factor)
        self.reliability_factor_log.append(reliability_factor)
        self.uncertainty_factor_log.append(uncertainty_factor)
        self.consistency_factor_log.append(consistency_factor)
        self.quality_factor_log.append(quality_factor)
        
        return quality_factor
    
    def evaluate_heading(self, target_pos_X: float, target_pos_Y: float,
                        reported_heading: float, instant_idx: int) -> float:
        """
        Evaluate heading trust score based on estimated vs reported heading.
        
        Args:
            target_pos_X: Target's current X position
            target_pos_Y: Target's current Y position  
            reported_heading: Target's reported heading (radians)
            instant_idx: Current simulation time step
            
        Returns:
            Heading trust score (0 to 1)
        """
        # Check if we have previous position data
        if np.array_equal(self.previous_state, np.zeros(4)):
            h_score = 1.0  # Cannot compute heading without previous position
            self.previous_state = np.array([target_pos_X, target_pos_Y, 0, 0])
            return h_score
        
        # Extract previous position
        prev_target_pos_X = self.previous_state[0]
        prev_target_pos_Y = self.previous_state[1]
        
        # Calculate estimated heading from position change
        delta_Y = target_pos_Y - prev_target_pos_Y
        delta_X = target_pos_X - prev_target_pos_X
        theta_est = math.atan2(delta_Y, delta_X)
        
        # Calculate heading difference (handle circular nature of angles)
        theta_diff = abs(reported_heading - theta_est)
        theta_diff = min(theta_diff, 2 * math.pi - theta_diff)
        
        # Compute trust score (10 degrees threshold)
        theta_max = math.pi / 18  # 10 degrees
        h_score = max(1 - theta_diff / theta_max, 0)
        
        # Update previous state
        self.previous_state = np.array([target_pos_X, target_pos_Y, 0, 0])
        
        return h_score
    
    def calculate_trust_sample_wo_acc(self, v_score: float, d_score: float, 
                                     a_score: float, beacon_score: float,
                                     h_score: float, is_nearby: bool) -> float:
        """Calculate trust sample without acceleration component."""
        return beacon_score * v_score * d_score
    
    def calculate_trust_sample_w_jerk(self, v_score: float, d_score: float,
                                     a_score: float, j_score: float, 
                                     beacon_score: float) -> float:
        """Calculate trust sample with jerkiness component."""
        return (beacon_score * (v_score ** self.wv) * (d_score ** self.wd) * 
                (a_score ** self.wa) * (j_score ** self.wj))
    
    def calculate_trust_sample_normal(self, v_score: float, d_score: float,
                                     a_score: float, beacon_score: float,
                                     h_score: float, is_nearby: bool) -> float:
        """Calculate normal trust sample with heading component."""
        if is_nearby:
            return (beacon_score * (v_score ** self.wv_nearby) * 
                   (d_score ** self.wd_nearby) * (h_score ** self.wh_nearby))
        else:
            return beacon_score * (v_score ** self.wv) * (d_score ** self.wd)
    
    def calculate_trust_sample(self, v_score: float, d_score: float, 
                              a_score: float, beacon_score: float,
                              h_score: float, is_nearby: bool) -> float:
        """Calculate trust sample based on all components."""
        return beacon_score * v_score * d_score 
    
    def update_rating_vector(self, trust_sample: float, vector_type: str = "local"):
        """
        Update the trust rating vector based on trust sample.
        
        Args:
            trust_sample: Current trust sample value
            vector_type: "local" or "global" rating vector to update
        """
        # Map trust sample to trust level
        trust_level = min(round(trust_sample * (self.k - 1)) + 1, self.k)
        trust_level = max(1, trust_level)  # Ensure at least level 1
        
        # Create trust vector (one-hot encoding)
        trust_vector = np.zeros(self.k)
        trust_vector[trust_level - 1] = 1  # Convert to 0-based indexing
        
        # Calculate current trust score for lambda calculation
        if vector_type == "local":
            current_trust_score = self.calculate_trust_score(self.rating_vector)
            lambda_y = current_trust_score * self.wt
            # Update rating vector
            self.rating_vector = (1 - lambda_y) * self.rating_vector + trust_vector
        else:  # "global"
            current_trust_score = self.calculate_trust_score(self.rating_vector_global)
            lambda_y = current_trust_score * self.wt_global
            # Update rating vector
            self.rating_vector_global = (1 - lambda_y) * self.rating_vector_global + trust_vector
    
    def calculate_trust_score(self, rating_vector: np.ndarray) -> float:
        """
        Calculate trust score from rating vector.
        
        Args:
            rating_vector: Trust rating vector
            
        Returns:
            Trust score (0 to 1)
        """
        # Normalize rating vector to probabilities
        a = self.C / self.k
        S_y = (rating_vector + a) / (self.C + np.sum(rating_vector))
        
        # Define weights with epsilon to avoid zero weight for lowest level
        epsilon = 0.01
        weights = (np.arange(self.k) + epsilon) / (self.k - 1 + epsilon)
        
        # Calculate weighted trust score
        trust_score = np.sum(weights * S_y)
        return trust_score
    
    def compute_cross_host_target_factor(self, host_id: int, host_vehicle: Any,
                                        target_id: int, target_vehicle: Any) -> Tuple[float, float, float, float]:
        """
        Compute cross-host target trust factor based on global estimate comparison.
        
        Args:
            host_id: Host vehicle ID
            host_vehicle: Host vehicle object
            target_id: Target vehicle ID  
            target_vehicle: Target vehicle object
            
        Returns:
            Tuple of (gamma_cross, D_pos, D_vel, D_acc)
        """
        # Get global estimates
        if hasattr(host_vehicle, 'observer') and hasattr(host_vehicle, 'center_communication'):
            host_global_estimate = host_vehicle.observer.est_global_state_current
            target_global_estimate = host_vehicle.center_communication.get_global_state(target_id, host_id)
        else:
            # Fallback values
            return 1.0, 0.0, 0.0, 0.0
        
        if target_global_estimate is None or np.any(np.isnan(target_global_estimate)):
            return 0.0, float('inf'), float('inf'), float('inf')
        
        # Initialize discrepancies
        D = 0.0
        D_pos = 0.0
        D_vel = 0.0
        D_acc = 0.0
        
        num_vehicles = target_global_estimate.shape[1]
        
        for j in range(num_vehicles):
            # Position difference (assuming first 2 dimensions are x, y)
            pos_diff = target_global_estimate[0:2, j] - host_global_estimate[0:2, j]
            pos_covar_inv = np.linalg.inv(self.sigma2_matrix_gamma_cross[0:2, 0:2])
            D_pos += pos_diff.T @ pos_covar_inv @ pos_diff
            
            # Velocity difference (assuming 4th dimension is velocity)
            vel_diff = target_global_estimate[3, j] - host_global_estimate[3, j]
            vel_var_inv = 1.0 / self.sigma2_matrix_gamma_cross[3, 3]
            D_vel += vel_diff * vel_var_inv * vel_diff
            
            # Acceleration difference (assuming 5th dimension is acceleration)
            if target_global_estimate.shape[0] > 4:
                acc_diff = target_global_estimate[4, j] - host_global_estimate[4, j]
                acc_var_inv = 1.0 / self.sigma2_matrix_gamma_cross[4, 4]
                D_acc += acc_diff * acc_var_inv * acc_diff
            
            # Total discrepancy
            x_diff = target_global_estimate[:, j] - host_global_estimate[:, j]
            try:
                covar_inv = np.linalg.inv(self.sigma2_matrix_gamma_cross)
                D += x_diff.T @ covar_inv @ x_diff
            except np.linalg.LinAlgError:
                # Fallback if matrix is singular
                D += np.sum(x_diff ** 2)
        
        # Compute trust factor
        gamma_cross = math.exp(-D)
        
        return gamma_cross, D_pos, D_vel, D_acc
    
    def compute_local_consistency_factor(self, host_vehicle: Any, target_vehicle: Any,
                                       neighbors: List[Any]) -> float:
        """
        Compute local consistency trust factor.
        
        Args:
            host_vehicle: Host vehicle object
            target_vehicle: Target vehicle object
            neighbors: List of neighbor vehicle objects
            
        Returns:
            Local consistency trust factor (0 to 1)
        """
        if not hasattr(target_vehicle, 'param') or not hasattr(host_vehicle, 'center_communication'):
            return 1.0
        
        # Vehicle parameters
        half_length_vehicle = target_vehicle.param.l_r
        host_id = host_vehicle.vehicle_number
        
        # Get target's global estimate
        target_global_estimate = host_vehicle.center_communication.get_global_state(
            target_vehicle.vehicle_number, host_id)
        
        if target_global_estimate is None or np.any(np.isnan(target_global_estimate)):
            return 0.0
        
        # Get host vehicle state from target's estimate
        x_l_i = target_global_estimate[[0, 3], host_id - 1]  # Position and velocity of host
        
        # Find predecessor and successor
        M_i = 0
        predecessor = None
        successor = None
        
        for neighbor in neighbors:
            car_idx = neighbor.vehicle_number
            if abs(car_idx - host_id) == 1:
                if car_idx > host_id:
                    M_i += 1
                    successor = neighbor
                elif car_idx < host_id:
                    M_i += 1
                    predecessor = neighbor
        
        # If no neighbors found, return maximum trust
        if M_i == 0:
            return 1.0
        
        E = 0.0  # Total error
        
        # Check predecessor consistency
        if predecessor is not None:
            # Local measurements
            host_distance_measurement = (predecessor.state[0] - host_vehicle.state[0] - 
                                       half_length_vehicle)
            velocity_diff_measurement = abs(predecessor.state[3] - host_vehicle.state[3])
            y_i_predecessor = np.array([host_distance_measurement, velocity_diff_measurement])
            
            # Get predecessor state from target's global estimate
            pred_id = predecessor.vehicle_number
            x_l_pred = target_global_estimate[[0, 3], pred_id - 1]
            
            # Compute expected relative state
            rel_state_est = np.abs(x_l_pred - x_l_i - np.array([half_length_vehicle, 0]))
            
            # Consistency error
            e = rel_state_est - y_i_predecessor
            try:
                covar_inv = np.linalg.inv(self.tau2_matrix_gamma_local)
                E += e.T @ covar_inv @ e
            except np.linalg.LinAlgError:
                E += np.sum(e ** 2)
        
        # Check successor consistency
        if successor is not None:
            # Local measurements
            host_distance_measurement = (host_vehicle.state[0] - successor.state[0] - 
                                       half_length_vehicle)
            velocity_diff_measurement = abs(successor.state[3] - host_vehicle.state[3])
            y_i_successor = np.array([host_distance_measurement, velocity_diff_measurement])
            
            # Get successor state from target's global estimate
            successor_id = successor.vehicle_number
            x_l_successor = target_global_estimate[[0, 3], successor_id - 1]
            
            # Compute expected relative state
            rel_state_est = np.abs(x_l_i - x_l_successor - np.array([half_length_vehicle, 0]))
            
            # Consistency error
            e = rel_state_est - y_i_successor
            try:
                covar_inv = np.linalg.inv(self.tau2_matrix_gamma_local)
                E += e.T @ covar_inv @ e
            except np.linalg.LinAlgError:
                E += np.sum(e ** 2)
        
        # Compute trust factor
        gamma_local = math.exp(-E)
        return gamma_local
    
    def monitor_sudden(self, gamma_cross: float, D_pos: float, 
                      D_vel: float, D_acc: float) -> float:
        """
        Monitor for sudden changes and anomalies in trust factors.
        
        Args:
            gamma_cross: Current cross-validation trust factor
            D_pos: Position discrepancy
            D_vel: Velocity discrepancy  
            D_acc: Acceleration discrepancy
            
        Returns:
            Beta factor for trust adjustment (0 to 1)
        """
        # Anomaly detection for gamma_cross
        anomaly_gamma = 0
        if len(self.gamma_cross_log) >= self.w:
            window = self.gamma_cross_log[-self.w:]
            mu_gamma = np.mean(window)
            sigma_gamma = np.std(window)
            if sigma_gamma > 0 and abs(gamma_cross - mu_gamma) > 2 * sigma_gamma:
                anomaly_gamma = 1
        
        self.anomaly_gamma_log.append(anomaly_gamma)
        
        # Log discrepancies
        self.D_pos_log.append(D_pos)
        self.D_vel_log.append(D_vel)
        
        # Anomaly detection for position
        anomaly_pos = 0
        if len(self.D_pos_log) >= self.w:
            window = self.D_pos_log[-self.w:]
            mu_pos = np.mean(window)
            sigma_pos = np.std(window)
            if sigma_pos > 0 and abs(D_pos - mu_pos) > 2 * sigma_pos:
                anomaly_pos = 1
        
        self.anomaly_pos_log.append(anomaly_pos)
        
        # Anomaly detection for velocity
        anomaly_vel = 0
        if len(self.D_vel_log) >= self.w:
            window = self.D_vel_log[-self.w:]
            mu_vel = np.mean(window)
            sigma_vel = np.std(window)
            if sigma_vel > 0 and abs(D_vel - mu_vel) > 2 * sigma_vel:
                anomaly_vel = 1
        
        self.anomaly_vel_log.append(anomaly_vel)
        
        # Anomaly detection for acceleration
        if not hasattr(self, 'D_acc_log'):
            self.D_acc_log = []
        
        self.D_acc_log.append(D_acc)
        anomaly_acc = 0
        if len(self.D_acc_log) >= self.w:
            window = self.D_acc_log[-self.w:]
            mu_acc = np.mean(window)
            sigma_acc = np.std(window)
            if sigma_acc > 0 and abs(D_acc - mu_acc) > 2 * sigma_acc:
                anomaly_acc = 1
        
        if not hasattr(self, 'anomaly_acc_log'):
            self.anomaly_acc_log = []
        self.anomaly_acc_log.append(anomaly_acc)
        
        beta = 1.0  # Default value
        
        # Cumulative check for trust adjustment
        if len(self.anomaly_gamma_log) >= self.w:
            # Count dropped packets
            anomaly_drop_packet = (self.w - 1) - sum(self.beacon_score_log[-(self.w-1):])
            
            # Count anomalies in sliding window
            anomaly_count_gamma = sum(self.anomaly_gamma_log[-self.w:])
            anomaly_count_pos = sum(self.anomaly_pos_log[-self.w:])
            anomaly_count_vel = sum(self.anomaly_vel_log[-self.w:])
            anomaly_count_acc = sum(self.anomaly_acc_log[-self.w:])
            
            min_anomali = min([anomaly_count_gamma, anomaly_count_pos, 
                              anomaly_count_vel, anomaly_count_acc, anomaly_drop_packet])
            
            if min_anomali > self.Threshold_anomalie:
                beta = 1 - self.reduce_factor * min_anomali / self.w
                beta = max(beta, 0)  # Ensure non-negative
        
        return beta
    
    def calculate_trust(self, host_vehicle: Any, target_vehicle: Any, 
                       leader_vehicle: Any, neighbors: List[Any], 
                       is_nearby: bool, instant_idx: int) -> Tuple[float, float, float, float, float, float, float]:
        """
        Main function to calculate trust scores for a vehicle.
        
        Args:
            host_vehicle: Host vehicle object
            target_vehicle: Target vehicle object
            leader_vehicle: Leader vehicle object
            neighbors: List of neighbor vehicles
            is_nearby: Whether target is nearby
            instant_idx: Current time step
            
        Returns:
            Tuple of (final_score, local_trust_sample, gamma_cross, v_score, d_score, a_score, beacon_score)
        """
        host_id = host_vehicle.vehicle_number
        target_id = target_vehicle.vehicle_number
        
        # Get vehicle parameters
        if hasattr(target_vehicle, 'param'):
            vehicle_length = target_vehicle.param.l_r + target_vehicle.param.l_f
        else:
            vehicle_length = 4.0  # Default vehicle length
        
        # Get leader state
        if hasattr(host_vehicle, 'other_vehicles') and len(host_vehicle.other_vehicles) > 0:
            leader_state = host_vehicle.other_vehicles[0].state
        else:
            leader_state = np.full(5, np.nan)
        
        # Handle leader beacon interval
        if np.any(np.isnan(leader_state)):
            leader_state = self.lead_state_lastest.copy()
            leader_beacon_interval = (instant_idx - self.lead_state_lastest_timestamp) * host_vehicle.dt
        else:
            self.lead_state_lastest = leader_state.copy()
            self.lead_state_lastest_timestamp = instant_idx
            leader_beacon_interval = 0
        
        # Extract leader data
        leader_velocity = leader_state[3] if len(leader_state) > 3 else 0
        leader_acceleration = leader_state[4] if len(leader_state) > 4 else 0
        
        # Get host data
        if hasattr(host_vehicle, 'observer'):
            host_pos_X = host_vehicle.observer.est_local_state_current[0]
            host_velocity = host_vehicle.observer.est_local_state_current[3]
            host_acceleration = host_vehicle.observer.est_local_state_current[4]
        else:
            host_pos_X = host_vehicle.state[0] if hasattr(host_vehicle, 'state') else 0
            host_velocity = host_vehicle.state[3] if hasattr(host_vehicle, 'state') and len(host_vehicle.state) > 3 else 0
            host_acceleration = host_vehicle.state[4] if hasattr(host_vehicle, 'state') and len(host_vehicle.state) > 4 else 0
        
        # Calculate host distance measurement
        if is_nearby:
            delta_X = target_vehicle.state[0] - host_pos_X
            if delta_X >= 0:
                host_distance_measurement = delta_X + vehicle_length
            else:
                host_distance_measurement = abs(delta_X) - vehicle_length
        else:
            # Handle non-nearby vehicles
            if (hasattr(host_vehicle, 'scenarios_config') and 
                hasattr(host_vehicle.scenarios_config, 'is_know_data_not_nearby') and
                host_vehicle.scenarios_config.is_know_data_not_nearby):
                delta_X = target_vehicle.state[0] - host_pos_X
                if delta_X >= 0:
                    host_distance_measurement = delta_X + vehicle_length
                else:
                    host_distance_measurement = abs(delta_X) - vehicle_length
            else:
                # Estimate distance based on expected spacing
                nb_space = abs(host_id - target_id)
                if hasattr(host_vehicle, 'Param_opt'):
                    T = host_vehicle.Param_opt.hi
                    s0 = host_vehicle.Param_opt.ri
                else:
                    T = 0.5  # Default time gap
                    s0 = 8.0  # Default minimum distance
                
                s_acc_expected = nb_space * (s0 + T * host_velocity)
                host_distance_measurement = s_acc_expected - vehicle_length
        
        # Get target state from communication
        if hasattr(host_vehicle, 'center_communication'):
            target_state = host_vehicle.center_communication.get_local_state(target_id, host_id)
        else:
            target_state = None
        
        # Evaluate trust components
        if target_state is None or np.any(np.isnan(target_state)):
            beacon_score = 0.0
            v_score = 0.0
            d_score = 0.0
            a_score = 0.0
            h_score = 0.0
            local_trust_sample = 0.0
        else:
            beacon_score = 1.0
            
            # Extract target data
            target_pos_X = target_state[0]
            target_pos_Y = target_state[1]
            target_reported_velocity = target_state[3]
            target_reported_acceleration = target_state[4] if len(target_state) > 4 else 0
            target_reported_heading = target_state[2]
            
            # Calculate target reported distance
            delta_X = target_pos_X - host_pos_X
            if delta_X >= 0:
                target_reported_distance = delta_X + vehicle_length
            else:
                target_reported_distance = abs(delta_X) - vehicle_length
            
            # Evaluate trust scores
            v_score = self.evaluate_velocity(host_id, target_id, target_reported_velocity,
                                           host_velocity, leader_velocity, leader_acceleration,
                                           leader_beacon_interval, is_nearby, 0.1)
            
            d_score = self.evaluate_distance(target_reported_distance, 
                                           host_distance_measurement, is_nearby)
            
            a_score = self.evaluate_acceleration(host_vehicle, host_id, target_id,
                                               target_reported_acceleration, host_acceleration,
                                               [host_distance_measurement, self.last_d],
                                               host_vehicle.dt, is_nearby)
            
            h_score = self.evaluate_heading(target_pos_X, target_pos_Y,
                                          target_reported_heading, instant_idx)
            
            # Calculate local trust sample
            local_trust_sample = self.calculate_trust_sample(v_score, d_score, a_score,
                                                           beacon_score, h_score, is_nearby)
        
        # Update last distance
        self.last_d = host_distance_measurement
        
        # Compute global estimate trust factors
        if (hasattr(target_vehicle, 'center_communication') and 
            target_vehicle.center_communication.get_global_state is not None):
            try:
                gamma_cross, D_pos, D_vel, D_acc = self.compute_cross_host_target_factor(
                    host_id, host_vehicle, target_id, target_vehicle)
                gamma_local = self.compute_local_consistency_factor(
                    host_vehicle, target_vehicle, neighbors)
                global_trust_sample = gamma_cross * gamma_local
            except:
                gamma_cross = 0.0
                gamma_local = 0.0
                global_trust_sample = 0.0
                D_pos = D_vel = D_acc = float('inf')
        else:
            gamma_cross = 0.0
            gamma_local = 0.0
            global_trust_sample = 0.0
            D_pos = D_vel = D_acc = 0.0
        
        # Monitor sudden changes
        if (hasattr(host_vehicle, 'scenarios_config') and 
            hasattr(host_vehicle.scenarios_config, 'Monitor_sudden_change') and
            host_vehicle.scenarios_config.Monitor_sudden_change):
            beta = self.monitor_sudden(gamma_cross, D_pos, D_vel, D_acc)
        else:
            beta = 1.0
        
        # Calculate final trust score based on configuration
        if (hasattr(host_vehicle, 'scenarios_config') and 
            hasattr(host_vehicle.scenarios_config, 'Dichiret_type')):
            if host_vehicle.scenarios_config.Dichiret_type == "Single":
                trust_sample_ext = local_trust_sample * global_trust_sample
                self.update_rating_vector(trust_sample_ext, "local")
                final_score = self.calculate_trust_score(self.rating_vector)
            else:  # "Dual"
                self.update_rating_vector(local_trust_sample, "local")
                local_trust_sample = self.calculate_trust_score(self.rating_vector)
                
                self.update_rating_vector(global_trust_sample, "global")
                global_trust_sample = self.calculate_trust_score(self.rating_vector_global)
                
                final_score = local_trust_sample * global_trust_sample
        else:
            # Default behavior
            trust_sample_ext = local_trust_sample * global_trust_sample
            self.update_rating_vector(trust_sample_ext, "local")
            final_score = self.calculate_trust_score(self.rating_vector)
        
        final_score *= beta
        
        # Update flags
        self.flag_taget_attk = False
        self.flag_glob_est_check = False
        self.flag_local_est_check = False
        
        if gamma_local > 0.5 and gamma_cross < 0.5:
            self.flag_taget_attk = True
        if gamma_local < 0.5 and gamma_cross > 0.5:
            self.flag_glob_est_check = True
        if local_trust_sample < 0.5:
            self.flag_local_est_check = True
        
        # Log data for analysis
        self.trust_sample_log.append(local_trust_sample)
        self.gamma_cross_log.append(gamma_cross)
        self.gamma_local_log.append(gamma_local)
        self.v_score_log.append(v_score)
        self.d_score_log.append(d_score)
        self.a_score_log.append(a_score)
        self.h_score_log.append(h_score)
        self.beacon_score_log.append(beacon_score)
        self.final_score_log.append(final_score)
        
        self.flag_taget_attk_log.append(self.flag_taget_attk)
        self.flag_glob_est_check_log.append(self.flag_glob_est_check)
        self.flag_local_est_check_log.append(self.flag_local_est_check)
        
        return final_score, local_trust_sample, gamma_cross, v_score, d_score, a_score, beacon_score


# Example usage and integration with VehicleProcess
if __name__ == "__main__":
    # Create trust model instance
    trust_model = TriPTrustModel()
    
    # Example usage (you would integrate this into your VehicleProcess class)
    print("TriPTrustModel Python implementation created successfully!")
    print(f"Initial trust score: {trust_model.calculate_trust_score(trust_model.rating_vector)}")
