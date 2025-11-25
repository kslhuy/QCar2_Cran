#!/usr/bin/env python3
"""
Hybrid Graph-Based Trust Evaluation with Buffering

This file contains the buffered trust evaluation methods for fleet graph-based
trust evaluation. It implements a hybrid approach:
1. Buffer received states (local and fleet) for efficient batch processing
2. Calculate trust at fixed intervals using buffered data
3. Support immediate evaluation for critical situations
"""

from typing import Optional, Dict, List, Any, Deque
from dataclasses import dataclass
from collections import deque
import time
import math
import numpy as np


@dataclass
class FleetStateSnapshot:
    """
    Snapshot of received state for trust evaluation.
    Stores both fleet estimates and local states in the same structure.
    """
    sender_id: int                    # Vehicle that sent this state
    timestamp: float                  # When the state was received
    state_data: dict                  # The actual state data
    relative_distance: Optional[float] = None  # Measured distance from sensors (if available)
    state_type: str = "local"        # "local" or "fleet"


class GraphBasedTrustEvaluator:
    """
    Hybrid trust evaluator for graph-connected vehicles with buffering.
    
    Features:
    - Buffers received states (local and fleet) for batch processing
    - Calculates trust at fixed intervals using TriPTrustModel.calculate_trust()
    - Supports immediate evaluation for critical situations
    - Uses deque for efficient buffering with automatic size limits
    """
    
    def __init__(self, vehicle_id: int, connected_vehicles: List[int], trust_models: Dict,
                 trust_update_interval: float = 0.5, max_buffer_size: int = 20):
        """
        Initialize the graph-based trust evaluator with buffering.
        
        Args:
            vehicle_id: This vehicle's ID
            connected_vehicles: List of connected vehicle IDs (from fleet graph)
            trust_models: Dictionary mapping vehicle_id -> TriPTrustModel instance
            trust_update_interval: Time interval (seconds) between trust calculations
            max_buffer_size: Maximum number of samples to buffer per vehicle
        """
        self.vehicle_id = vehicle_id
        self.connected_vehicles = connected_vehicles
        self.trust_models = trust_models
        self.trust_scores = {}
        
        # Trust update timing configuration
        self.trust_update_interval = trust_update_interval
        self.last_trust_update_time = 0.0
        self.max_buffer_size = max_buffer_size
        
        # Buffering for fleet states (one buffer per connected vehicle)
        self.fleet_state_buffer: Dict[int, Deque[FleetStateSnapshot]] = {}
        
        # Buffering for local states (one buffer per connected vehicle)
        self.local_state_buffer: Dict[int, Deque[FleetStateSnapshot]] = {}
        
        # Initialize buffers and trust scores for connected vehicles
        for vid in connected_vehicles:
            if vid != self.vehicle_id:
                self.trust_scores[vid] = 1.0  # Start with maximum trust
                self.fleet_state_buffer[vid] = deque(maxlen=max_buffer_size)
                self.local_state_buffer[vid] = deque(maxlen=max_buffer_size)
        
        # Reference to VehicleProcess for quality metrics (set after initialization)
        self.vehicle_process = None
    
    def buffer_fleet_state(self, sender_id: int, fleet_state: dict, 
                          timestamp: Optional[float] = None , relative_distance: Optional[float] = None):
        """
        Buffer received fleet state for later trust evaluation.
        
        Args:
            sender_id: ID of vehicle that sent the fleet state
            fleet_state: Fleet state data received
            timestamp: When state was received (None = use current time)
        """
        if sender_id not in self.connected_vehicles or sender_id == self.vehicle_id:
            return
        
        if timestamp is None:
            timestamp = time.time()
        
        snapshot = FleetStateSnapshot(
            sender_id=sender_id,
            timestamp=timestamp,
            state_data=fleet_state.copy(),
            relative_distance=relative_distance,
            state_type="fleet"
        )
        
        self.fleet_state_buffer[sender_id].append(snapshot)
    
    def buffer_local_state(self, sender_id: int, local_state: dict, 
                          relative_distance: Optional[float] = None,
                          timestamp: Optional[float] = None):
        """
        Buffer received local state for later trust evaluation.
        
        Args:
            sender_id: ID of vehicle that sent the state
            local_state: Local state data received
            relative_distance: Measured distance from sensors
            timestamp: When state was received (None = use current time)
        """
        if sender_id not in self.connected_vehicles or sender_id == self.vehicle_id:
            return
        
        if timestamp is None:
            timestamp = time.time()
        
        snapshot = FleetStateSnapshot(
            sender_id=sender_id,
            timestamp=timestamp,
            state_data=local_state.copy(),
            relative_distance=relative_distance,
            state_type="local"
        )
        
        self.local_state_buffer[sender_id].append(snapshot)
    
    def set_vehicle_process_reference(self, vehicle_process):
        """
        Set reference to VehicleProcess for accessing quality metrics.
        Call this after creating GraphBasedTrustEvaluator.
        
        Args:
            vehicle_process: VehicleProcess instance
        """
        self.vehicle_process = vehicle_process
    
    def _get_quality_metrics_for_vehicle(self, target_id: int) -> Optional[Dict]:
        """
        Get quality metrics from VehicleProcess for a specific vehicle.
        
        Args:
            target_id: Target vehicle ID
            
        Returns:
            Dictionary with quality metrics or None if not available
        """
        if self.vehicle_process is None:
            return None
        
        # Use VehicleProcess method to get quality metrics
        return self.vehicle_process.get_quality_metrics_for_vehicle(target_id)
    
    def should_update_trust(self, current_time: Optional[float] = None) -> bool:
        """
        Check if it's time to update trust scores based on the interval.
        
        Args:
            current_time: Current time (None = use time.time())
            
        Returns:
            True if trust should be updated, False otherwise
        """
        if current_time is None:
            current_time = time.time()
        
        return (current_time - self.last_trust_update_time) >= self.trust_update_interval
    
    def update_all_trust_scores(self, our_local_state: dict, our_fleet_estimates: dict, 
                               logger=None):
        """
        Update trust scores for all connected vehicles using buffered data.
        This method is called at fixed intervals (trust_update_interval).
        
        Evaluates both local trust (from direct messages) and global trust (from fleet estimates).
        
        Args:
            our_local_state: OUR local state from our observer (for local trust comparison)
            our_fleet_estimates: OUR distributed observer's fleet estimates (for global trust comparison)
            logger: Logger for output
        """
        current_time = time.time()
        
        if not self.should_update_trust(current_time):
            return
        
        self.last_trust_update_time = current_time
        
        # Update trust for each connected vehicle
        for vehicle_id in self.connected_vehicles:
            if vehicle_id == self.vehicle_id:
                continue
            
            # Evaluate trust using buffered states (both local and global)
            trust_score = self._evaluate_trust_from_buffers(
                vehicle_id, our_local_state, our_fleet_estimates, logger
            )
            
            if trust_score is not None:
                self.trust_scores[vehicle_id] = trust_score
        
        # Log summary after update
        if logger:
            self.log_trust_summary(logger)
    
    def _evaluate_trust_from_buffers(self, target_id: int, our_local_state: dict,
                                    our_fleet_estimates: dict, logger) -> Optional[float]:
        """
        Evaluate trust using buffered state data combining local and global trust.
        
        This method evaluates:
        1. Local trust: Compare received direct messages with our_local_state
        2. Global trust: Compare received fleet estimates with our_fleet_estimates
        
        Args:
            target_id: Vehicle ID to evaluate trust for
            our_local_state: OUR local state from our observer
            our_fleet_estimates: OUR distributed observer's fleet estimates
            logger: Logger
            neighbors_states: List of neighbor vehicle states for consistency checks
            
        Returns:
            Combined trust score or None if evaluation failed
        """
        if target_id not in self.trust_models:
            return None
        
        trust_model = self.trust_models[target_id]
        
        # Get most recent states from buffers
        local_buffer = self.local_state_buffer.get(target_id, deque())
        fleet_buffer = self.fleet_state_buffer.get(target_id, deque())
        
        if len(local_buffer) == 0 and len(fleet_buffer) == 0:
            if logger:
                logger.debug(f"TRUST: No buffered data for vehicle {target_id}")
            return None
        
        # ═══════════════════════════════════════════════════════════
        # PART 1: LOCAL TRUST EVALUATION (from direct state messages)
        # ═══════════════════════════════════════════════════════════
        # Compare received direct messages with OUR local state
        local_trust_score = None
        if len(local_buffer) > 0:
            latest_local = local_buffer[-1]
            local_trust_score = self._evaluate_local_trust(
                target_id, latest_local, our_local_state, trust_model, logger
            )
        
        # ═══════════════════════════════════════════════════════════
        # PART 2: GLOBAL TRUST EVALUATION (from fleet estimates)
        # ═══════════════════════════════════════════════════════════
        # Compare received fleet estimates with OUR distributed observer's fleet estimates
        global_trust_score = None
        if len(fleet_buffer) > 0:
            latest_fleet = fleet_buffer[-1]
            global_trust_score = self._evaluate_global_trust(
                target_id, latest_fleet, our_local_state, our_fleet_estimates, 
                trust_model, logger
            )
        
        # ═══════════════════════════════════════════════════════════
        # PART 3: COMBINE LOCAL AND GLOBAL TRUST
        # ═══════════════════════════════════════════════════════════
        if local_trust_score is not None and global_trust_score is not None:
            # Both available: multiply for combined trust
            final_trust_score = local_trust_score * global_trust_score
            data_source = "local+global"
            buffer_info = f"local={len(local_buffer)},fleet={len(fleet_buffer)}"
        elif local_trust_score is not None:
            # Only local trust available
            final_trust_score = local_trust_score
            data_source = "local_only"
            buffer_info = f"local={len(local_buffer)},fleet=0"
        elif global_trust_score is not None:
            # Only global trust available
            final_trust_score = global_trust_score
            data_source = "global_only"
            buffer_info = f"local=0,fleet={len(fleet_buffer)}"
        else:
            return None
        
        if logger:
            local_str = f"{local_trust_score:.3f}" if local_trust_score is not None else "N/A"
            global_str = f"{global_trust_score:.3f}" if global_trust_score is not None else "N/A"
            logger.info(f"TRUST_UPDATE: Vehicle {target_id} -> {final_trust_score:.3f} "
                      f"(local={local_str}, global={global_str}, "
                      f"source={data_source}, buffers=[{buffer_info}])")
        
        return final_trust_score
    
    def _evaluate_local_trust(self, target_id: int, snapshot: FleetStateSnapshot,
                             our_state: dict, trust_model, logger) -> Optional[float]:
        """
        Evaluate local trust from direct state messages.
        Uses velocity, distance, and acceleration scores.
        
        Args:
            target_id: Target vehicle ID
            snapshot: State snapshot from buffer
            our_state: Our current state
            trust_model: TriPTrustModel instance
            logger: Logger
            
        Returns:
            Local trust score or None
        """
        try:
            target_state = snapshot.state_data
            relative_distance = snapshot.relative_distance
            
            # Extract state information
            reported_velocity = target_state.get('vel', target_state.get('velocity', 0.0))
            reported_position = target_state.get('pos', target_state.get('position', [0, 0, 0]))
            reported_acceleration = target_state.get('acceleration', 0.0)
            
            our_velocity = our_state.get('vel', our_state.get('velocity', 0.0))
            our_position = our_state.get('pos', our_state.get('position', [0, 0, 0]))
            our_acceleration = our_state.get('acceleration', 0.0)

            # Calculate distance received
            if len(reported_position) >= 2 and len(our_position) >= 2:
                distance_received = math.sqrt((reported_position[0] - our_position[0])**2 + 
                                    (reported_position[1] - our_position[1])**2) 
            else:
                distance_received = 99.0
                
            # Use relative distance from sensors if available
            if relative_distance is not None and relative_distance > 0:
                distance_measured = relative_distance
                distance_measured_source = "sensor"
            else:
                distance_measured = distance_received
                distance_measured_source = "position"
            
            is_nearby = abs(target_id - self.vehicle_id) == 1 

            # Evaluate trust components using TriPTrustModel
            v_score = trust_model.evaluate_velocity(
                host_id=self.vehicle_id,
                target_id=target_id,
                v_y=reported_velocity,
                v_host=our_velocity,
                v_leader=reported_velocity,  # Simplified
                a_leader=reported_acceleration,
                b_leader=0.1,
                is_nearby=is_nearby
            )
            
            d_score = trust_model.evaluate_distance(
                d_y=distance_received,
                d_measured=distance_measured,
                is_nearby=is_nearby
            )
            
            # Simplified acceleration and other scores
            a_score = 1.0 if abs(reported_acceleration) < 5.0 else 0.5
            beacon_score = 1.0  # We received the message
            h_score = 1.0  # Simplified heading
            
            # Calculate base trust sample (from velocity, distance, etc.)
            trust_sample = trust_model.calculate_trust_sample(
                v_score=v_score,
                d_score=d_score,
                a_score=a_score,
                beacon_score=beacon_score,
                h_score=h_score,
                is_nearby=is_nearby
            )
            
            # NEW: Get quality metrics and apply quality factor
            quality_factor = 1.0  # Default if no quality data (initialize first!)
            quality_metrics = self._get_quality_metrics_for_vehicle(target_id)
            
            if quality_metrics is not None:
                # Extract and validate quality metrics
                msg_age = quality_metrics.get('age', 0.0)
                drop_rate = quality_metrics.get('drop_rate', 0.0)
                covariance = quality_metrics.get('covariance', 1.0)
                innovation = quality_metrics.get('innovation', 0.0)
                
                # Ensure all inputs are valid numbers (not None or NaN)
                if msg_age is None or not isinstance(msg_age, (int, float)) or math.isnan(msg_age):
                    msg_age = 0.0
                if drop_rate is None or not isinstance(drop_rate, (int, float)) or math.isnan(drop_rate):
                    drop_rate = 0.0
                if covariance is None or not isinstance(covariance, (int, float)) or math.isnan(covariance):
                    covariance = 1.0
                if innovation is None or not isinstance(innovation, (int, float)) or math.isnan(innovation):
                    innovation = 0.0
                
                # Evaluate communication quality factor
                quality_factor = trust_model.evaluate_communication_quality(
                    message_age=msg_age,
                    drop_rate=drop_rate,
                    covariance=covariance,
                    innovation=innovation
                )
                
                # Safety check: ensure quality_factor is not None or NaN
                if quality_factor is None or not isinstance(quality_factor, (int, float)) or math.isnan(quality_factor):
                    quality_factor = 1.0
                    if logger:
                        logger.warning(f"QUALITY_FACTOR: V{target_id} returned invalid value, using default 1.0")
                
                # Apply quality factor to trust sample
                trust_sample = trust_sample * quality_factor
                
                if logger:
                    logger.debug(f"QUALITY_FACTOR: V{target_id} = {quality_factor:.3f} "
                               f"(age={quality_metrics['age']:.2f}s, drop={quality_metrics['drop_rate']:.2f}, "
                               f"cov={quality_metrics['covariance']:.3f}, innov={quality_metrics['innovation']:.3f}m)")
            else:
                if logger:
                    logger.debug(f"QUALITY_FACTOR: V{target_id} = {quality_factor:.3f} (no quality data available)")
            
            # Update rating vector for local trust
            trust_model.update_rating_vector(trust_sample, vector_type="local")
            
            # Calculate local trust score
            local_trust_score = trust_model.calculate_trust_score(trust_model.rating_vector)
            
            if logger:
                logger.debug(f"LOCAL_TRUST: V{target_id} -> {local_trust_score:.3f} "
                           f"(v={v_score:.3f}, d={d_score:.3f}, q={quality_factor:.3f}, "
                           f"dist={distance_received:.1f}m [{distance_measured_source}])")
           
            return local_trust_score
            
        except Exception as e:
            if logger:
                logger.error(f"LOCAL_TRUST_EVAL: Error for vehicle {target_id}: {e}")
            return None
    
    def _evaluate_global_trust(self, target_id: int, fleet_snapshot: FleetStateSnapshot,
                              our_local_state: dict, our_fleet_estimates: dict,
                              trust_model, logger) -> Optional[float]:
        """
        Evaluate global trust from fleet estimates using cross-validation and local consistency.
        
        This implements the global trust evaluation from TriPTrustModel.calculate_trust():
        - gamma_cross: Cross-validation between OUR fleet estimate and target's fleet estimate
        - gamma_local: Local consistency check comparing target's estimates with our estimates
        - global_trust_sample = gamma_cross * gamma_local
        
        Args:
            target_id: Target vehicle ID
            fleet_snapshot: Fleet state snapshot from buffer (received from target)
            our_local_state: OUR local state from our observer
            our_fleet_estimates: OUR distributed observer's fleet estimates
            trust_model: TriPTrustModel instance
            logger: Logger
            
        Returns:
            Global trust score or None
        """
        try:
            fleet_data = fleet_snapshot.state_data
            relative_distance = fleet_snapshot.relative_distance

            # Extract fleet estimates from the target vehicle (what they broadcast)
            target_fleet_estimates = fleet_data.get('estimates', {})
            
            if not target_fleet_estimates:
                if logger:
                    logger.debug(f"GLOBAL_TRUST: No estimates in fleet data from V{target_id}")
                return None
            
            # ═══════════════════════════════════════════════════════════
            # GAMMA_CROSS: Cross-validation trust factor
            # ═══════════════════════════════════════════════════════════
            # Compare target's fleet estimates with OUR fleet estimates
            gamma_cross = self._compute_gamma_cross(
                target_id, target_fleet_estimates, our_local_state, 
                our_fleet_estimates, logger
            )
            
            # ═══════════════════════════════════════════════════════════
            # GAMMA_LOCAL: Local consistency trust factor  
            # ═══════════════════════════════════════════════════════════
            # Check if target's fleet estimates are consistent with our fleet estimates
            gamma_local = self._compute_gamma_local(
                target_id, target_fleet_estimates, our_local_state, 
                our_fleet_estimates,  logger , relative_distance
            )
            
            # ═══════════════════════════════════════════════════════════
            # COMBINE: Global trust sample
            # ═══════════════════════════════════════════════════════════
            global_trust_sample = gamma_cross * gamma_local
            
            # Update rating vector for global trust
            trust_model.update_rating_vector(global_trust_sample, vector_type="global")
            
            # Calculate global trust score
            global_trust_score = trust_model.calculate_trust_score(trust_model.rating_vector_global)
            
            if logger:
                logger.debug(f"GLOBAL_TRUST: V{target_id} -> {global_trust_score:.3f} "
                           f"(gamma_cross={gamma_cross:.3f}, gamma_local={gamma_local:.3f}, "
                           f"sample={global_trust_sample:.3f})")
            
            return global_trust_score
            
        except Exception as e:
            if logger:
                logger.error(f"GLOBAL_TRUST_EVAL: Error for vehicle {target_id}: {e}")
            return None
    
    def _compute_gamma_cross(self, target_id: int, target_fleet_estimates: dict,
                            our_local_state: dict, our_fleet_estimates: dict, 
                            logger) -> float:
        """
        Compute cross-validation trust factor (gamma_cross).
        Compares target's fleet estimates with OUR distributed observer's fleet estimates.
        
        Formula: gamma_cross = exp(-Σ D) where D is discrepancy
        
        Args:
            target_id: Target vehicle ID
            target_fleet_estimates: Fleet estimates received from target vehicle
            our_local_state: OUR local state
            our_fleet_estimates: OUR distributed observer's fleet estimates
            logger: Logger
            
        Returns:
            gamma_cross factor (0 to 1)
        """
        try:
            # For each vehicle in fleet, compare target's estimate with OUR estimate
            total_discrepancy = 0.0
            num_vehicles = 0
            
            for veh_id_str, target_estimate in target_fleet_estimates.items():
                veh_id = int(veh_id_str) if isinstance(veh_id_str, str) else veh_id_str
                
                # Get target's estimate for this vehicle
                target_estimate_pos = target_estimate.get('pos', [0, 0])
                target_estimate_vel = target_estimate.get('vel', 0.0)
                
                # Get OUR estimate for this vehicle
                our_estimate = None
                if veh_id == self.vehicle_id:
                    # Compare with our actual local state
                    our_estimate = our_local_state
                elif veh_id in our_fleet_estimates:
                    # Compare with our distributed observer's estimate
                    our_estimate = our_fleet_estimates[veh_id]
                
                if our_estimate is None:
                    continue  # Skip if we don't have estimate for this vehicle
                
                our_pos = our_estimate.get('pos', [0, 0, 0])
                our_vel = our_estimate.get('vel', 0.0)
                
                # Position discrepancy
                if len(target_estimate_pos) >= 2 and len(our_pos) >= 2:
                    pos_error = math.sqrt((target_estimate_pos[0] - our_pos[0])**2 + 
                                        (target_estimate_pos[1] - our_pos[1])**2)
                    # Normalized by sensitivity (sigma2 = 2.0 for position)
                    D_pos = (pos_error / 2.0) ** 2
                else:
                    D_pos = 0.0
                
                # Velocity discrepancy
                vel_error = abs(target_estimate_vel - our_vel)
                D_vel = (vel_error / 1.0) ** 2  # sigma2 = 1.0 for velocity
                
                total_discrepancy += D_pos + D_vel
                num_vehicles += 1
            
            if num_vehicles == 0:
                return 1.0  # No data to compare, assume neutral trust
            
            # Calculate gamma_cross using exponential decay
            avg_discrepancy = total_discrepancy / num_vehicles
            gamma_cross = math.exp(-avg_discrepancy)
            
            if logger:
                logger.debug(f"GAMMA_CROSS: V{target_id} -> {gamma_cross:.3f} "
                           f"(vehicles_compared={num_vehicles}, avg_D={avg_discrepancy:.3f})")
            
            return max(0.0, min(1.0, gamma_cross))  # Clamp to [0, 1]
            
        except Exception as e:
            if logger:
                logger.warning(f"GAMMA_CROSS: Error computing for V{target_id}: {e}")
            return 1.0  # Default to neutral trust
    
    def _compute_gamma_local(self, target_id: int, target_fleet_estimates: dict,
                            our_local_state: dict, our_fleet_estimates: dict,
                            logger ,relative_distance: float = None) -> float:
        """
        Compute local consistency trust factor (gamma_local).
        
        Checks if target's internal fleet estimate consistency matches our actual measurements.
        For direct neighbors (predecessor/successor), we compare:
        - Target's internal position differences (from their fleet estimates)
        - Our actual measured relative distances
        
        This follows the logic from TriPTrustModel.compute_local_consistency_factor().
        
        Formula: gamma_local = exp(-E) where E is consistency error
        
        Args:
            target_id: Target vehicle ID
            target_fleet_estimates: Fleet estimates received from target vehicle
            our_local_state: OUR local state
            our_fleet_estimates: OUR distributed observer's fleet estimates
            logger: Logger
            
        Returns:
            gamma_local factor (0 to 1)
        """
        try:
            # Vehicle parameters (simplified - would come from config)
            # half_length_vehicle = 2.0  # Approximate half vehicle length in meters
            
            # Get our vehicle's state from target's fleet estimate
            our_id_str = str(self.vehicle_id)
            if our_id_str not in target_fleet_estimates:
                if logger:
                    logger.debug(f"GAMMA_LOCAL: Our vehicle {self.vehicle_id} not in target's estimates")
                return 1.0  # Can't check consistency
            
            target_estimate_of_us = target_fleet_estimates[our_id_str]
            x_l_i_pos = target_estimate_of_us.get('pos', [0, 0])  # Target's estimate of our position
            x_l_i_vel = target_estimate_of_us.get('vel', 0.0)      # Target's estimate of our velocity
            
            # Find direct neighbors (predecessor = vehicle_id - 1, successor = vehicle_id + 1)
            E = 0.0  # Total consistency error
            M_i = 0  # Number of neighbors checked
            
            # Check predecessor (vehicle ahead of us)
            predecessor_id = self.vehicle_id - 1
            if predecessor_id >= 0 and predecessor_id in self.connected_vehicles:
                pred_id_str = str(predecessor_id)
                
                # Check if we have actual measurements for predecessor
                if pred_id_str in target_fleet_estimates and predecessor_id in our_fleet_estimates:
                    # Get target's estimate of predecessor
                    x_l_pred_pos = target_fleet_estimates[pred_id_str].get('pos', [0, 0])
                    x_l_pred_vel = target_fleet_estimates[pred_id_str].get('vel', 0.0)
                    
                    # Our actual measured relative state with predecessor
                    our_pred_estimate = our_fleet_estimates[predecessor_id]
                    our_pred_pos = our_pred_estimate.get('pos', [0, 0, 0])
                    our_pred_vel = our_pred_estimate.get('vel', 0.0)
                    our_pos = our_local_state.get('pos', [0, 0, 0])
                    our_vel = our_local_state.get('vel', 0.0)
                    
                    # Calculate our measured relative distance (position difference)
                    host_distance_measurement = abs(our_pred_pos[0] - our_pos[0]) 
                    velocity_diff_measurement = abs(our_pred_vel - our_vel)
                    y_i_predecessor = np.array([host_distance_measurement, velocity_diff_measurement])
                    
                    # Calculate expected relative state from target's estimates
                    # (position difference between predecessor and us in target's estimate)
                    rel_pos_est = abs(x_l_pred_pos[0] - x_l_i_pos[0]) 
                    rel_vel_est = abs(x_l_pred_vel - x_l_i_vel)
                    rel_state_est = np.array([rel_pos_est, rel_vel_est])
                    
                    # Consistency error: difference between target's internal estimate and our measurement
                    e = rel_state_est - y_i_predecessor
                    
                    # Weighted error using tau2 covariance matrix
                    # tau2_matrix_gamma_local = diag([1.5, 0.5])  (position, velocity)
                    E_pos = (e[0] / 1.5) ** 2
                    E_vel = (e[1] / 0.5) ** 2
                    E += E_pos + E_vel
                    M_i += 1
                    
                    if logger:
                        logger.debug(f"GAMMA_LOCAL: Predecessor check V{predecessor_id} - "
                                   f"measured_dist={host_distance_measurement:.3f}, "
                                   f"target_est_dist={rel_pos_est:.3f}, error={e[0]:.3f}")
            
            # Check successor (vehicle behind us)
            successor_id = self.vehicle_id + 1
            if successor_id in self.connected_vehicles:
                succ_id_str = str(successor_id)
                
                # Check if we have actual measurements for successor
                if succ_id_str in target_fleet_estimates and successor_id in our_fleet_estimates:
                    # Get target's estimate of successor
                    x_l_succ_pos = target_fleet_estimates[succ_id_str].get('pos', [0, 0])
                    x_l_succ_vel = target_fleet_estimates[succ_id_str].get('vel', 0.0)
                    
                    # Our actual measured relative state with successor
                    our_succ_estimate = our_fleet_estimates[successor_id]
                    our_succ_pos = our_succ_estimate.get('pos', [0, 0, 0])
                    our_succ_vel = our_succ_estimate.get('vel', 0.0)
                    our_pos = our_local_state.get('pos', [0, 0, 0])
                    our_vel = our_local_state.get('vel', 0.0)
                    
                    # Calculate our measured relative distance (position difference)
                    host_distance_measurement = abs(our_pos[0] - our_succ_pos[0]) 
                    velocity_diff_measurement = abs(our_vel - our_succ_vel)
                    y_i_successor = np.array([host_distance_measurement, velocity_diff_measurement])
                    
                    # Calculate expected relative state from target's estimates
                    # (position difference between us and successor in target's estimate)
                    rel_pos_est = abs(x_l_i_pos[0] - x_l_succ_pos[0]) 
                    rel_vel_est = abs(x_l_i_vel - x_l_succ_vel)
                    rel_state_est = np.array([rel_pos_est, rel_vel_est])
                    
                    # Consistency error
                    e = rel_state_est - y_i_successor
                    
                    # Weighted error
                    E_pos = (e[0] / 1.5) ** 2
                    E_vel = (e[1] / 0.5) ** 2
                    E += E_pos + E_vel
                    M_i += 1
                    
                    if logger:
                        logger.debug(f"GAMMA_LOCAL: Successor check V{successor_id} - "
                                   f"measured_dist={host_distance_measurement:.3f}, "
                                   f"target_est_dist={rel_pos_est:.3f}, error={e[0]:.3f}")
            
            # If no neighbors found, return neutral trust
            if M_i == 0:
                if logger:
                    logger.debug(f"GAMMA_LOCAL: V{target_id} -> No direct neighbors to check")
                return 1.0
            
            # Calculate gamma_local using exponential decay
            gamma_local = math.exp(-E)
            
            if logger:
                logger.debug(f"GAMMA_LOCAL: V{target_id} -> {gamma_local:.3f} "
                           f"(neighbors_checked={M_i}, total_E={E:.3f})")
            
            return max(0.0, min(1.0, gamma_local))  # Clamp to [0, 1]
            
        except Exception as e:
            if logger:
                logger.warning(f"GAMMA_LOCAL: Error computing for V{target_id}: {e}")
            return 1.0  # Default to neutral trust
    
    def evaluate_trust_for_local_state(self, sender_id: int, received_state: dict, 
                                        our_state: dict, logger=None, relative_distance: float = None) -> Optional[float]:
        """
        BACKWARD COMPATIBLE: Buffer received local state for trust evaluation.
        This method maintains compatibility with existing code by buffering the state
        and returning the current trust score (not immediately evaluating).
        
        Args:
            sender_id: ID of sender vehicle
            received_state: Received state data
            our_state: Our own vehicle state
            logger: Logger for output
            relative_distance: Measured relative distance from CamLidarFusion (optional)
            
        Returns:
            Current trust score (0.0 to 1.0) or None if not applicable
        """
        # Only buffer for connected vehicles
        if sender_id not in self.connected_vehicles or sender_id not in self.trust_models:
            return None
        
        # Buffer the received state
        self.buffer_local_state(
            sender_id=sender_id,
            local_state=received_state,
            relative_distance=relative_distance
        )
        
        # Return current trust score (will be updated at next interval)
        return self.trust_scores.get(sender_id, 1.0)
    
    def evaluate_trust_immediate(self, sender_id: int, received_state: dict, 
                                 our_state: dict, logger=None, relative_distance: float = None) -> Optional[float]:
        """
        IMMEDIATE EVALUATION: Evaluate trust immediately for critical situations.
        Use this for time-sensitive trust evaluation (e.g., emergency detection).
        
        Args:
            sender_id: ID of sender vehicle
            received_state: Received state data
            our_state: Our own vehicle state
            logger: Logger for output
            relative_distance: Measured relative distance from CamLidarFusion (optional)
            
        Returns:
            Trust score (0.0 to 1.0) or None if not applicable
        """
        # Only evaluate for connected vehicles
        if sender_id not in self.connected_vehicles or sender_id not in self.trust_models:
            return None
        
        # Buffer the state first
        self.buffer_local_state(sender_id, received_state, relative_distance)
        
        # Immediately evaluate trust
        trust_score = self._evaluate_trust_from_buffers(sender_id, our_state, logger)
        
        if trust_score is not None:
            self.trust_scores[sender_id] = trust_score
            if logger:
                logger.info(f"TRUST_IMMEDIATE: Vehicle {sender_id} evaluated immediately -> {trust_score:.3f}")
        
        return trust_score
    
    def get_trust_score(self, vehicle_id: int) -> float:
        """Get current trust score for a vehicle."""
        return self.trust_scores.get(vehicle_id, 1.0)
    
    def get_all_trust_scores(self) -> Dict[int, float]:
        """Get all current trust scores."""
        return self.trust_scores.copy()
    
    def is_vehicle_trusted(self, vehicle_id: int, threshold: float = 0.5) -> bool:
        """Check if a vehicle is trusted above threshold."""
        return self.get_trust_score(vehicle_id) >= threshold
    
    def apply_trust_based_control(self, following_target: int, base_distance: float, logger=None) -> float:
        """
        Apply trust-based control adjustments.
        
        Args:
            following_target: ID of vehicle we're following
            base_distance: Base following distance
            logger: Logger for output
            
        Returns:
            Adjusted following distance
        """
        if following_target not in self.connected_vehicles:
            return base_distance
            
        trust_score = self.get_trust_score(following_target)
        
        # Adjust distance based on trust (lower trust = larger distance)
        trust_factor = max(0.5, trust_score)  # Minimum 50% trust
        adjusted_distance = base_distance / trust_factor
        
        # Log significant adjustments
        if abs(adjusted_distance - base_distance) > 1.0 and logger:
            logger.info(f"TRUST_CONTROL: Distance adjustment for vehicle {following_target} - "
                       f"base={base_distance:.1f}m -> adjusted={adjusted_distance:.1f}m "
                       f"(trust={trust_score:.3f})")
        
        return adjusted_distance

    def log_trust_summary(self, logger=None):
        """
        Log a summary of all current trust scores for monitoring.
        
        Args:
            logger: Logger for output
        """
        if logger and len(self.trust_scores) > 0:
            trust_summary = []
            for vehicle_id, score in self.trust_scores.items():
                trust_summary.append(f"V{vehicle_id}={score:.3f}")
            
            logger.info(f"TRUST_SUMMARY: Connected vehicles trust scores: {', '.join(trust_summary)}")
    
    def get_trust_statistics(self) -> Dict[str, float]:
        """
        Get statistical information about current trust scores.
        
        Returns:
            Dictionary with trust statistics
        """
        if not self.trust_scores:
            return {'count': 0, 'mean': 0.0, 'min': 0.0, 'max': 0.0}
        
        scores = list(self.trust_scores.values())
        return {
            'count': len(scores),
            'mean': sum(scores) / len(scores),
            'min': min(scores),
            'max': max(scores)
        }


# Example integration helper methods for VehicleProcess
def integrate_graph_based_trust_into_vehicle_process():
    """
    Instructions for integrating buffered trust system into VehicleProcess.py
    
    ═══════════════════════════════════════════════════════════════════════
    HYBRID BUFFERED TRUST EVALUATION INTEGRATION
    ═══════════════════════════════════════════════════════════════════════
    
    1. INITIALIZATION (in __init__ method):
    ────────────────────────────────────────
        if self.trust_enabled and len(self.connected_vehicles) > 0:
            self.trust_evaluator = GraphBasedTrustEvaluator(
                vehicle_id=self.vehicle_id,
                connected_vehicles=self.connected_vehicles,
                trust_models={vid: TriPTrustModel() for vid in self.connected_vehicles if vid != self.vehicle_id},
                trust_update_interval=0.5,  # Calculate trust every 0.5s
                max_buffer_size=20          # Keep last 20 samples
            )
        else:
            self.trust_evaluator = None
    
    2. BUFFER LOCAL STATES (in _handle_vehicle_state_direct method):
    ─────────────────────────────────────────────────────────────────
        # After successfully adding state to queue, buffer for trust evaluation
        if self.trust_enabled and sender_id != self.vehicle_id:
            if hasattr(self, 'trust_evaluator') and self.trust_evaluator is not None:
                # Get relative distance from sensor fusion
                relative_distance = None
                if hasattr(self, 'cam_lidar_fusion') and self.cam_lidar_fusion:
                    relative_distance = self.cam_lidar_fusion.get_closest_distance()
                
                # Buffer the state (no immediate calculation)
                self.trust_evaluator.buffer_local_state(
                    sender_id=sender_id,
                    local_state=received_state,
                    relative_distance=relative_distance
                )
    
    3. BUFFER FLEET STATES (in _handle_fleet_estimates_direct method):
    ───────────────────────────────────────────────────────────────────
        # After processing fleet estimates, buffer for trust evaluation
        if self.trust_enabled and sender_id != self.vehicle_id:
            if hasattr(self, 'trust_evaluator') and self.trust_evaluator is not None:
                self.trust_evaluator.buffer_fleet_state(
                    sender_id=sender_id,
                    fleet_state=fleet_message,
                    timestamp=message_timestamp
                )
    
    4. PERIODIC TRUST UPDATE (in run method - main control loop):
    ──────────────────────────────────────────────────────────────
        # Add this in the main control loop (after control updates)
        if self.trust_enabled and hasattr(self, 'trust_evaluator') and self.trust_evaluator:
            # Get our current state
            our_state = self.get_best_available_state()
            
            # Update all trust scores if interval has passed
            self.trust_evaluator.update_all_trust_scores(
                our_state=our_state,
                logger=self.trust_logger
            )
    
    5. OPTIONAL: IMMEDIATE TRUST EVALUATION (for critical situations):
    ───────────────────────────────────────────────────────────────────
        # Use this only when you need immediate trust feedback
        if critical_situation_detected:
            trust_score = self.trust_evaluator.evaluate_trust_immediate(
                sender_id=sender_id,
                received_state=received_state,
                our_state=our_state,
                logger=self.trust_logger,
                relative_distance=relative_distance
            )
            
            if trust_score < 0.3:  # Low trust threshold
                trigger_emergency_response()
    
    6. TRUST-BASED CONTROL (optional - in controller):
    ───────────────────────────────────────────────────
        # Adjust following distance based on trust
        if self.following_target is not None:
            trust_score = self.trust_evaluator.get_trust_score(self.following_target)
            base_distance = 8.0
            adjusted_distance = self.trust_evaluator.apply_trust_based_control(
                self.following_target, base_distance, self.trust_logger
            )
            # Use adjusted_distance in controller
    
    ═══════════════════════════════════════════════════════════════════════
    KEY ADVANTAGES:
    ═══════════════════════════════════════════════════════════════════════
    ✓ Efficient: Buffering is O(1), trust calculation only every 0.5s
    ✓ Robust: Aggregates multiple samples for better accuracy
    ✓ Flexible: Supports both buffered and immediate evaluation
    ✓ Complete: Uses full TriPTrustModel.calculate_trust() function
    ✓ Compatible: Backward compatible with existing code
    """
    pass

if __name__ == "__main__":
    print("Hybrid Graph-Based Trust Evaluator with Buffering")
    print("See integration instructions in integrate_graph_based_trust_into_vehicle_process()")
