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
    weight_velocity: float = 3.0  # Reduced from 12.0 to be less sensitive
    weight_distance: float = 2.0
    weight_acceleration: float = 0.3
    weight_heading: float = 1.0

    # Trust thresholds
    trust_threshold: float = 0.5
    stationary_velocity_threshold: float = 0.2  # m/s - consider stationary below this
    velocity_tolerance: float = 0.3  # m/s - absolute tolerance before penalizing velocity differences
    min_velocity_tolerance: float = 0.05  # m/s - lower bound for adaptive tolerance
    turn_velocity_tolerance_gain: float = 0.35  # [m/s]/[rad/s] gain for turn-aware tolerance
    accel_velocity_tolerance_gain: float = 0.15  # [m/s]/[m/s^2] gain for acceleration-aware tolerance

    # Dirichlet parameters
    num_trust_levels: int = 5
    dirichlet_update_rate: float = 0.1  # Used for "ema" method
    dirichlet_type: str = "Dual"  # "Single" or "Dual"
    # Dirichlet method: "ema" (simple EMA) or "matlab" (aging-factor + regularized score)
    dirichlet_method: str = "ema"
    # MATLAB Dirichlet parameters (only used when dirichlet_method == "matlab")
    dirichlet_C: float = 0.2       # Regularization constant C
    dirichlet_wt_local: float = 0.4   # Aging weight wt for local trust
    dirichlet_wt_global: float = 0.5  # Aging weight wt for global trust

    # Attack detection
    monitor_sudden_change: bool = True
    sudden_change_threshold: float = 0.5
    attack_detection_window: int = 10

    # Communication quality
    max_message_age_s: float = 1.0
    expected_beacon_interval_s: float = 0.1
    use_message_age_quality: bool = True

    distributed_trust_fallback: float = 0.5
    # Sigma^2 covariance for gamma_host (cross-validation) — full 5-state
    distributed_trust_covariance_diag: Tuple[float, ...] = (
        1.5,
        1.0,
        0.5,
        0.5,
        0.25,
    )
    # Tau^2 covariance for gamma_local_peer / gamma_self  (relative measurement)
    # [relative_distance, relative_velocity]  — matches MATLAB tau2_diag_element
    distributed_local_tau2_diag: Tuple[float, ...] = (1.5, 0.5)

    # Generalized trust vector O_i
    use_generalized_trust_vector: bool = False
    trust_vector_theta_min: float = 0.5

    # EMA smoothing
    ema_alpha: float = 0.3

    # Noise tolerance for stationary vehicles
    stationary_noise_tolerance: float = (
        0.15  # Tolerance for measurement noise when stationary
    )

    # Validation gates (ported from MATLAB logic)
    use_physical_constraints_check: bool = True
    use_temporal_consistency_check: bool = True
    max_velocity: float = 6.0
    max_acceleration: float = 3.5
    max_deceleration: float = -6.0
    max_jerk: float = 6.0
    temporal_pos_tolerance_m: float = 2.0
    temporal_vel_tolerance: float = 1.0

    # Trust decay for missing beacons
    trust_decay_lambda: float = 0.2


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
    beacon_score_local: float = 1.0   # 1 if target's local state broadcast received
    beacon_score_global: float = 1.0  # 1 if target's fleet estimation broadcast received
    quality_factor: float = 1.0

    # Local and global trust samples
    local_trust_sample: float = 1.0
    global_trust_sample: float = 1.0
    gamma_host: float = 1.0
    gamma_local_peer: float = 1.0
    gamma_self: float = 1.0
    d_host_mean: float = float("nan")
    d_local_mean: float = float("nan")
    d_self: float = float("nan")

    # Trust rating vector (5 levels)
    trust_levels: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.2, 0.4, 0.4])
    )

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

        # Per-vehicle history for dynamic scoring
        self.previous_states: Dict[int, VehicleData] = {}
        self.distance_buffers: Dict[int, np.ndarray] = defaultdict(lambda: np.zeros(5))
        self.buffer_size = 5

        # Generalized trust vector O_i(j)
        self.opinion_vector: Dict[int, float] = {self.vehicle_id: 1.0}

        # Separate local/global trust memories for packet-loss decay
        self.previous_trust_local: Dict[int, float] = defaultdict(lambda: 1.0)
        self.previous_trust_global: Dict[int, float] = defaultdict(lambda: 1.0)

        # MATLAB Dirichlet rating vectors (per-target, per-type)
        self.rating_vectors_local: Dict[int, np.ndarray] = {}
        self.rating_vectors_global: Dict[int, np.ndarray] = {}

    def calculate_trust(
        self,
        host_state: Dict,
        target_data: VehicleData,
        leader_data: Optional[VehicleData] = None,
        neighbor_estimates: Optional[Dict[int, VehicleData]] = None,
        neighbor_host_estimates: Optional[Dict[int, VehicleData]] = None,
        host_target_estimate: Optional[np.ndarray] = None,
        host_fleet_estimates: Optional[np.ndarray] = None,
        target_fleet_estimates: Optional[Dict[int, Dict]] = None,
        current_time_ns: int = None,
        has_fleet_data: bool = False,
    ) -> TrustScore:
        """
        Calculate comprehensive trust score for a target vehicle

        Args:
            host_state: Host vehicle's own local state {x, y, theta, velocity, acceleration}
            target_data: V2V received local state from the target vehicle
            leader_data: Optional leader vehicle data (for reference)
            neighbor_estimates: Neighbors' global estimates of the TARGET (Dict[neighbor_id -> VehicleData])
            neighbor_host_estimates: Neighbors' global estimates of the HOST (Dict[neighbor_id -> VehicleData])
            host_target_estimate: Host's own global/distributed estimate of the target [state_dim]
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
        trust.distance_score = self._calculate_distance_score(host_state, target_data)

        # 3. Acceleration Score
        trust.acceleration_score = self._calculate_acceleration_score(
            host_state, target_data
        )

        # 4. Heading Score
        trust.heading_score = self._calculate_heading_score(host_state, target_data)

        # Optional physical constraints gate
        if self.config.use_physical_constraints_check:
            prev_state = self.previous_states.get(target_id)
            dt_phys = 0.1
            if prev_state is not None and prev_state.timestamp_ns > 0:
                dt_phys = max(
                    (target_data.timestamp_ns - prev_state.timestamp_ns) / 1e9,
                    0.01,
                )
            if not self._check_physical_constraints(target_data, prev_state, dt_phys):
                trust.velocity_score *= 0.1
                trust.distance_score *= 0.1
                trust.acceleration_score *= 0.1

        # Optional temporal consistency gate
        if self.config.use_temporal_consistency_check:
            temporal_score = self._evaluate_temporal_consistency(target_id, target_data)
            trust.velocity_score *= temporal_score
            trust.distance_score *= temporal_score
            trust.acceleration_score *= temporal_score

        # 5. Beacon Scores (binary 0/1) — local and global channels
        trust.beacon_score = self._calculate_beacon_score(target_id, current_time_ns)
        trust.beacon_score_local = 1.0   # Always 1 when calculate_trust is called (we received local data)
        trust.beacon_score_global = 1.0 if has_fleet_data else 0.0  # 1 if target's fleet estimates available

        # 6. Communication Quality Factor
        trust.quality_factor = self._calculate_quality_factor(
            target_data, current_time_ns
        )

        # === Calculate Local Trust Sample (gamma_local) ===
        # Weighted combination of component scores
        trust.local_trust_sample = self._compute_local_trust_sample(trust)

        # === Calculate Global Trust Sample (gamma_cross) ===
        # Paper-style: gamma_host * gamma_local_peer (with gamma_self modulator)
        trust.global_trust_sample, global_components = self._compute_global_trust_sample(
            target_id=target_id,
            host_state=host_state,
            target_data=target_data,
            neighbor_estimates=neighbor_estimates,
            neighbor_host_estimates=neighbor_host_estimates,
            host_target_estimate=host_target_estimate,
            host_fleet_estimates=host_fleet_estimates,
            target_fleet_estimates=target_fleet_estimates,
        )
        trust.gamma_host = float(global_components.get("gamma_host", 1.0))
        trust.gamma_local_peer = float(global_components.get("gamma_local_peer", 1.0))
        trust.gamma_self = float(global_components.get("gamma_self", 1.0))
        trust.d_host_mean = float(global_components.get("d_host_mean", float("nan")))
        trust.d_local_mean = float(global_components.get("d_local_mean", float("nan")))
        trust.d_self = float(global_components.get("d_self", float("nan")))

        # Apply separate local/global decay using local/global beacon channels
        trust.local_trust_sample = self._apply_trust_decay(
            target_id=target_id,
            current_trust=trust.local_trust_sample,
            beacon_received=trust.beacon_score_local > 0.5,
            trust_type="local",
        )
        trust.global_trust_sample = self._apply_trust_decay(
            target_id=target_id,
            current_trust=trust.global_trust_sample,
            beacon_received=trust.beacon_score_global > 0.5,
            trust_type="global",
        )

        # === Update Dirichlet Trust Levels & Compute Final Score ===
        if self.config.dirichlet_method == "matlab":
            # MATLAB-style: separate rating vectors with aging factor
            trust.final_score = self._dirichlet_matlab(
                target_id, trust.local_trust_sample, trust.global_trust_sample
            )
            # Also update trust_levels for logging purposes
            trust.trust_levels = self._update_trust_levels(
                target_id, trust.local_trust_sample, trust.global_trust_sample
            )
        else:
            # Default EMA-style Dirichlet update
            trust.trust_levels = self._update_trust_levels(
                target_id, trust.local_trust_sample, trust.global_trust_sample
            )
            trust.final_score = self._compute_final_score(trust.trust_levels)

        # Optional sudden-change moderation (MATLAB-style beta factor)
        if self.config.monitor_sudden_change:
            beta = self._monitor_sudden_change(target_id, trust.global_trust_sample)
            trust.final_score = float(np.clip(trust.final_score * beta, 0.0, 1.0))

        # EMA smoothing on final score (if history exists)
        history = self.trust_history.get(target_id, [])
        if history:
            alpha = float(np.clip(self.config.ema_alpha, 0.0, 1.0))
            prev_score = history[-1]
            trust.final_score = float(
                np.clip(alpha * trust.final_score + (1.0 - alpha) * prev_score, 0.0, 1.0)
            )

        # === Set Attack Detection Flags ===
        self._set_attack_flags(trust)

        # Store in history
        self._update_history(target_id, trust.final_score)

        # Store current state for next iteration (used by heading, distance, acceleration scores)
        self.previous_states[target_id] = VehicleData(
            vehicle_id=target_id,
            x=target_data.x,
            y=target_data.y,
            theta=target_data.theta,
            velocity=target_data.velocity,
            acceleration=target_data.acceleration,
            timestamp_ns=current_time_ns,
        )

        return trust

    def _calculate_velocity_score(
        self,
        host_state: Dict,
        target_data: VehicleData,
        leader_data: Optional[VehicleData],
    ) -> float:
        """
        Calculate velocity consistency score

        v_score = max(1 - |v_target - v_ref| / v_ref, 0)^w_v

        Special handling for stationary vehicles to avoid penalizing noise.
        """
        target_id = target_data.vehicle_id
        v_target = float(target_data.velocity)

        # Build robust velocity references:
        # - host speed
        # - optional leader speed
        # - target's own previous speed (temporal consistency)
        reference_candidates = [float(host_state.get("velocity", 0.0))]
        if leader_data is not None:
            reference_candidates.append(float(leader_data.velocity))
        prev_state = self.previous_states.get(target_id)
        if prev_state is not None:
            reference_candidates.append(float(prev_state.velocity))

        finite_refs = [v for v in reference_candidates if np.isfinite(v)]
        if not finite_refs:
            return 1.0
        v_ref = float(np.median(np.asarray(finite_refs, dtype=float)))

        # Special case: both vehicles are stationary
        # Give high trust if both are below stationary threshold
        stationary_threshold = self.config.stationary_velocity_threshold
        if abs(v_target) < stationary_threshold and abs(v_ref) < stationary_threshold:
            # Both stationary - check if velocities are similar (within noise tolerance)
            v_error = abs(v_target - v_ref)
            if v_error < self.config.stationary_noise_tolerance:
                return (
                    1.0  # Perfect trust for stationary vehicles with similar readings
                )
            else:
                # Small difference in stationary readings - still give high trust
                normalized_error = v_error / stationary_threshold
                score = max(1.0 - normalized_error, 0.0) ** self.config.weight_velocity
                return float(np.clip(score, 0.0, 1.0))

        # Normal case: at least one vehicle is moving
        # Use absolute tolerance so small natural velocity differences
        # (e.g. 0.1 m/s in a 0.5 m/s platoon) don't destroy trust.
        base_tolerance = max(
            self.config.velocity_tolerance,
            self.config.min_velocity_tolerance,
            0.01,
        )

        # Turning vehicles naturally exhibit speed deviations (inside/outside arc,
        # transient steering corrections). Increase tolerance with yaw-rate.
        turn_bonus = 0.0
        if prev_state is not None and prev_state.timestamp_ns > 0:
            dt = max((target_data.timestamp_ns - prev_state.timestamp_ns) / 1e9, 0.01)
            heading_delta = np.arctan2(
                np.sin(target_data.theta - prev_state.theta),
                np.cos(target_data.theta - prev_state.theta),
            )
            yaw_rate = abs(heading_delta) / dt
            speed_scale = max(abs(v_target), abs(v_ref), 0.3)
            turn_bonus = (
                float(self.config.turn_velocity_tolerance_gain) * yaw_rate * speed_scale
            )

        accel_bonus = float(self.config.accel_velocity_tolerance_gain) * max(
            abs(float(target_data.acceleration)),
            abs(float(host_state.get("acceleration", 0.0))),
        )

        v_tolerance = max(base_tolerance + turn_bonus + accel_bonus, 0.01)
        v_error = abs(v_target - v_ref)
        normalized_error = v_error / v_tolerance

        # Smooth decay is more robust than hard clipping:
        # avoids collapsing to zero immediately for mild mismatch.
        score = np.exp(-self.config.weight_velocity * (normalized_error**2))

        return float(np.clip(score, 0.0, 1.0))

    def _calculate_distance_score(
        self, host_state: Dict, target_data: VehicleData
    ) -> float:
        """
        Calculate distance consistency score.

        Compares expected distance change (from relative velocity) with reported change.
        d_score = max(1 - |d_error| / d_measured, 0)^w_d
        """
        target_id = target_data.vehicle_id

        # Calculate current distance from host to target
        dx = target_data.x - host_state.get("x", 0.0)
        dy = target_data.y - host_state.get("y", 0.0)
        d_current = np.sqrt(dx**2 + dy**2)

        # Get previous state if available
        if target_id in self.previous_states:
            prev = self.previous_states[target_id]

            # Previous distance
            prev_host_x = host_state.get(
                "x", 0.0
            )  # Approximate, using current host position
            prev_host_y = host_state.get("y", 0.0)
            prev_dx = prev.x - prev_host_x
            prev_dy = prev.y - prev_host_y
            d_prev = np.sqrt(prev_dx**2 + prev_dy**2)

            # Expected distance change based on relative velocity
            v_target = target_data.velocity
            v_host = host_state.get("velocity", 0.0)
            v_rel = v_target - v_host

            # Approximate dt based on timestamps or default
            dt = 0.1  # Default 100ms update rate
            d_expected = d_prev + v_rel * dt

            # Calculate error between expected and actual distance
            d_measured = max(d_current, 0.1)
            d_error = abs(d_current - d_expected)
            normalized_error = d_error / d_measured

            score = max(1.0 - normalized_error, 0.0) ** self.config.weight_distance
        else:
            # First observation - no comparison possible
            score = 1.0

        return float(np.clip(score, 0.0, 1.0))

    def _calculate_acceleration_score(
        self, host_state: Dict, target_data: VehicleData
    ) -> float:
        """
        Calculate acceleration consistency score based on dynamics.

        Uses distance buffer to compute relative velocity and expected acceleration.
        a_score = max(1 - |d_add * delta_acc|, 0)^w_a

        Special handling for stationary vehicles to tolerate measurement noise.
        """
        target_id = target_data.vehicle_id
        a_target = target_data.acceleration
        a_host = host_state.get("acceleration", 0.0)
        v_target = target_data.velocity
        v_host = host_state.get("velocity", 0.0)

        # Special case: both vehicles are stationary
        stationary_threshold = self.config.stationary_velocity_threshold
        if abs(v_target) < stationary_threshold and abs(v_host) < stationary_threshold:
            # Both stationary - expect near-zero acceleration
            # Allow for measurement noise
            a_error = abs(a_target - a_host)
            noise_tolerance = (
                0.5  # m/s^2 - typical sensor noise for stationary vehicles
            )

            if a_error < noise_tolerance:
                return (
                    1.0  # High trust for small acceleration differences when stationary
                )
            else:
                # Larger acceleration difference - penalize but not too harshly
                normalized_error = min(
                    a_error / noise_tolerance, 2.0
                )  # Cap at 2x tolerance
                score = (
                    max(1.0 - normalized_error / 2.0, 0.0)
                    ** self.config.weight_acceleration
                )
                return float(np.clip(score, 0.0, 1.0))

        # Normal case: at least one vehicle is moving
        # Calculate expected acceleration based on velocity change rather than double differentiating distance
        if target_id in self.previous_states:
            prev = self.previous_states[target_id]
            dt = max(
                (target_data.timestamp_ns - prev.timestamp_ns) / 1e9, 0.01
            )  # Avoid div by zero

            # Expected acceleration of target
            expected_a_target = (v_target - prev.velocity) / dt

            # Calculate error
            a_error = abs(a_target - expected_a_target)

            # Normalize error (tolerate up to 2.0 m/s^2 difference gracefully)
            normalization_factor = 2.0
            normalized_error = min(a_error / normalization_factor, 2.0)

            score = (
                max(1.0 - normalized_error / 2.0, 0.0)
                ** self.config.weight_acceleration
            )
        else:
            score = 1.0  # First observation

        return float(np.clip(score, 0.0, 1.0))

    def _calculate_heading_score(
        self, host_state: Dict, target_data: VehicleData
    ) -> float:
        """
        Calculate heading consistency score.

        Compares trajectory-based estimated heading with reported heading.
        h_score = max(1 - theta_diff / theta_max, 0)
        """
        target_id = target_data.vehicle_id
        theta_reported = target_data.theta

        # Check if we have previous position data
        if target_id not in self.previous_states:
            return 1.0  # First observation, cannot compute heading

        prev = self.previous_states[target_id]

        # Calculate position change
        delta_x = target_data.x - prev.x
        delta_y = target_data.y - prev.y

        # Only compute if there's meaningful movement
        movement = np.sqrt(delta_x**2 + delta_y**2)
        if (
            movement < 0.5
        ):  # Less than 0.5m movement makes heading estimation too susceptible to GPS noise
            return 1.0  # Not enough movement, can't estimate heading reliably

        # Estimate heading from trajectory
        theta_est = np.arctan2(delta_y, delta_x)

        # Calculate heading difference (handle circular nature of angles)
        theta_diff = np.arctan2(
            np.sin(theta_reported - theta_est), np.cos(theta_reported - theta_est)
        )

        # Maximum expected heading difference (90 degrees)
        theta_max = np.pi / 2
        normalized_diff = abs(theta_diff) / theta_max
        score = max(1.0 - normalized_diff, 0.0)

        return float(np.clip(score, 0.0, 1.0))

    def _calculate_beacon_score(self, target_id: int, current_time_ns: int) -> float:
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

    def _calculate_quality_factor(
        self, target_data: VehicleData, current_time_ns: int
    ) -> float:
        """
        Calculate communication quality factor

        q = exp(-2 * age) * (1 - drop_rate) * 1/(1 + 0.2 * covariance)
        """
        current_time_s = current_time_ns / 1e9
        target_id = target_data.vehicle_id

        # Age factor (configurable: can be disabled)
        message_age_s = max((current_time_ns - target_data.timestamp_ns) / 1e9, 0.0)
        if self.config.use_message_age_quality:
            if message_age_s > self.config.max_message_age_s:
                age_factor = 0.0
            else:
                age_factor = np.exp(-2.0 * message_age_s)
        else:
            age_factor = 1.0

        # Drop rate factor
        total = (
            self.beacon_receive_counts[target_id] + self.beacon_drop_counts[target_id]
        )
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

    def _check_physical_constraints(
        self,
        current: VehicleData,
        previous: Optional[VehicleData],
        dt: float,
    ) -> bool:
        """Validate velocity/acceleration/jerk plausibility."""
        if abs(current.velocity) > self.config.max_velocity:
            return False
        if (
            current.acceleration > self.config.max_acceleration
            or current.acceleration < self.config.max_deceleration
        ):
            return False
        if previous is not None and dt > 1e-3:
            jerk = abs(current.acceleration - previous.acceleration) / dt
            if jerk > self.config.max_jerk:
                return False
        return True

    def _evaluate_temporal_consistency(self, target_id: int, current: VehicleData) -> float:
        """
        Compare current kinematic state against one-step motion prediction.
        Returns score in [0, 1].
        """
        prev = self.previous_states.get(target_id)
        if prev is None or prev.timestamp_ns <= 0:
            return 1.0

        dt = max((current.timestamp_ns - prev.timestamp_ns) / 1e9, 0.01)
        expected_x = prev.x + prev.velocity * np.cos(prev.theta) * dt
        expected_y = prev.y + prev.velocity * np.sin(prev.theta) * dt
        expected_v = prev.velocity + prev.acceleration * dt

        pos_err = np.sqrt((current.x - expected_x) ** 2 + (current.y - expected_y) ** 2)
        vel_err = abs(current.velocity - expected_v)

        pos_score = max(1.0 - pos_err / max(self.config.temporal_pos_tolerance_m, 1e-3), 0.0)
        vel_score = max(1.0 - vel_err / max(self.config.temporal_vel_tolerance, 1e-3), 0.0)
        return float(np.clip(0.6 * pos_score + 0.4 * vel_score, 0.0, 1.0))

    def _apply_trust_decay(
        self,
        target_id: int,
        current_trust: float,
        beacon_received: bool,
        trust_type: str,
    ) -> float:
        """
        Local/global trust decay:
        T(t) = T_current if beacon received else (1-lambda)*T_prev
        """
        decay = float(np.clip(self.config.trust_decay_lambda, 0.0, 1.0))
        if trust_type == "local":
            memory = self.previous_trust_local
        else:
            memory = self.previous_trust_global

        if beacon_received:
            updated = float(np.clip(current_trust, 0.0, 1.0))
        else:
            prev = float(np.clip(memory.get(target_id, 1.0), 0.0, 1.0))
            updated = (1.0 - decay) * prev

        # Floor: never decay below fallback to prevent values like 1e-20
        # that can never recover.
        floor = max(float(self.config.distributed_trust_fallback) * 0.1, 0.01)
        updated = max(updated, floor)

        memory[target_id] = updated
        return updated

    def _monitor_sudden_change(self, target_id: int, gamma_cross: float) -> float:
        """Return multiplicative beta in [0,1] when abrupt changes persist."""
        window = max(int(self.config.attack_detection_window), 2)
        history = self.trust_history.get(target_id, [])
        if len(history) < window:
            return 1.0

        recent = np.asarray(history[-window:], dtype=float)
        mu = float(np.mean(recent))
        sigma = float(np.std(recent))
        if sigma <= 1e-6:
            return 1.0

        z = abs(gamma_cross - mu) / sigma
        if z > max(self.config.sudden_change_threshold, 0.1):
            return float(np.clip(1.0 - 0.2 * min(z / 5.0, 1.0), 0.0, 1.0))
        return 1.0

    def _compute_local_trust_sample(self, trust: TrustScore) -> float:
        """
        Compute local trust sample (gamma_local) from component scores

        Weighted geometric mean of component scores
        """
        # Weights for each component (normalized)
        weights = {
            "velocity": 0.3,
            "distance": 0.2,
            "acceleration": 0.15,
            "heading": 0.15,
            "beacon": 0.1,
            "quality": 0.1,
        }

        # Compute weighted product
        scores = [
            (trust.velocity_score, weights["velocity"]),
            (trust.distance_score, weights["distance"]),
            (trust.acceleration_score, weights["acceleration"]),
            (trust.heading_score, weights["heading"]),
            (trust.beacon_score, weights["beacon"]),
            (trust.quality_factor, weights["quality"]),
        ]

        weighted_product = 1.0
        total_weight = 0.0

        for score, weight in scores:
            # Floor the score at 0.01 to ensure completely false data (0.0)
            # dragging down the geometric mean significantly, rather than being discarded
            safe_score = max(score, 0.01)
            weighted_product *= safe_score**weight
            total_weight += weight

        if total_weight > 0:
            local_sample = weighted_product ** (1.0 / total_weight)
        else:
            local_sample = 0.5  # Neutral if no valid scores

        return float(np.clip(local_sample, 0.0, 1.0))

    def _compute_global_trust_sample(
        self,
        target_id: int,
        host_state: Dict,
        target_data: VehicleData,
        neighbor_estimates: Optional[Dict[int, VehicleData]],
        neighbor_host_estimates: Optional[Dict[int, VehicleData]],
        host_target_estimate: Optional[np.ndarray],
        host_fleet_estimates: Optional[np.ndarray] = None,
        target_fleet_estimates: Optional[Dict[int, Dict]] = None,
    ) -> Tuple[float, Dict[str, float]]:
        """
        Compute global trust sample using paper-style gamma computation.

        gamma_host * gamma_local_peer (with gamma_self modulator)
        using relative measurements (MATLAB-unified).
        """
        return self._compute_global_trust_sample_paper(
            host_state=host_state,
            target_data=target_data,
            neighbor_estimates=neighbor_estimates,
            neighbor_host_estimates=neighbor_host_estimates,
            host_target_estimate=host_target_estimate,
            host_fleet_estimates=host_fleet_estimates,
            target_fleet_estimates=target_fleet_estimates,
        )

    def _distance_to_gamma(self, distance: float) -> float:
        """
        Convert Mahalanobis distance to trust gamma in [0,1].

        MATLAB-faithful: gamma = exp(-D)  (TriPTrustModel.m lines 1734, 1828)
        """
        d = max(float(distance), 0.0)
        gamma = np.exp(-d)
        return float(np.clip(gamma, 0.0, 1.0))

    # ------------------------------------------------------------------ #
    #  Relative-measurement helpers  (MATLAB: tau2_matrix_gamma_local)    #
    # ------------------------------------------------------------------ #

    def _compute_relative_measurement(
        self, state_a: Dict, state_b_data: VehicleData
    ) -> np.ndarray:
        """
        Compute a 'fake sensor' relative measurement between two vehicles,
        acting as an approximation of a distance/velocity sensor (LiDAR/radar).

        Returns:
            np.array([relative_distance, relative_velocity])  (2-element vector)

        This measurement is analogous to what MATLAB computes from
        ``predecessor.state(1) - host_vehicle.state(1)`` and
        ``|predecessor.state(4) - host_vehicle.state(4)|``.

        When real LiDAR/radar is available in the future, replace this
        with actual sensor readings.
        """
        dx = float(state_b_data.x) - float(state_a.get("x", state_a.get(0, 0.0)))
        dy = float(state_b_data.y) - float(state_a.get("y", state_a.get(1, 0.0)))
        rel_distance = np.sqrt(dx ** 2 + dy ** 2)

        vel_a = float(state_a.get("velocity", state_a.get(3, 0.0)))
        vel_b = float(state_b_data.velocity)
        rel_velocity = abs(vel_b - vel_a)

        return np.array([rel_distance, rel_velocity], dtype=float)

    def _compute_relative_from_estimates(
        self, est_host: VehicleData, est_target: VehicleData
    ) -> np.ndarray:
        """
        Compute the implied relative measurement between two vehicles
        using a neighbour's global estimates of both.

        Returns:
            np.array([relative_distance, relative_velocity])  (2-element vector)
        """
        dx = float(est_target.x) - float(est_host.x)
        dy = float(est_target.y) - float(est_host.y)
        rel_distance = np.sqrt(dx ** 2 + dy ** 2)
        rel_velocity = abs(float(est_target.velocity) - float(est_host.velocity))
        return np.array([rel_distance, rel_velocity], dtype=float)

    def _relative_mahalanobis(self, y_measured: np.ndarray, y_estimated: np.ndarray) -> float:
        """
        Mahalanobis distance between measured and estimated relative state,
        using the tau2 covariance diagonal (MATLAB: tau2_matrix_gamma_local).

        Returns:
            e' * inv(Tau2) * e   (scalar)
        """
        tau2_diag = np.array(self.config.distributed_local_tau2_diag, dtype=float)
        # Ensure same length as the relative vector (2 elements)
        if len(tau2_diag) < 2:
            tau2_diag = np.pad(tau2_diag, (0, 2 - len(tau2_diag)), constant_values=1.0)
        tau2_diag = tau2_diag[:2]
        tau2_inv = 1.0 / np.maximum(tau2_diag, 1e-9)

        e = y_estimated - y_measured
        return float(e @ np.diag(tau2_inv) @ e)

    # ------------------------------------------------------------------ #
    #  Paper-style global trust (MATLAB-unified)                         #
    # ------------------------------------------------------------------ #

    def _compute_global_trust_sample_paper(
        self,
        host_state: Dict,
        target_data: VehicleData,
        neighbor_estimates: Optional[Dict[int, VehicleData]],
        neighbor_host_estimates: Optional[Dict[int, VehicleData]],
        host_target_estimate: Optional[np.ndarray],
        host_fleet_estimates: Optional[np.ndarray] = None,
        target_fleet_estimates: Optional[Dict[int, Dict]] = None,
    ) -> Tuple[float, Dict[str, float]]:
        """
        Paper-style distributed trust (unified with MATLAB TriPTrustModel):

        Three gamma components:
        - gamma_host   (= MATLAB gamma_cross):
              Cross-validation between host's global estimate and neighbors'
              global estimates of the target using FULL state Mahalanobis (Sigma^2).
              gamma_host = exp( -sum_j  (x_host_est - x_neighbor_j_est)' * Sigma^-1 * ... )

        - gamma_local_peer  (= MATLAB gamma_local):
              Local consistency — compare the relative measurement
              [distance, velocity] that the host observes (from host_state
              and target_data) against the implied relative measurement
              from each neighbor's global estimates (of both host and target).
              gamma_local_peer = exp( -sum_j  e_j' * Tau^-1 * e_j )
              where e_j = neighbor_j_relative - host_local_relative

        - gamma_self  (= MATLAB gamma_local_our_self):
              Self-consistency — compare the host's local relative measurement
              against the implied relative from the host's OWN global estimate.
              gamma_self = exp( -e' * Tau^-1 * e )
              where e = host_global_relative - host_local_relative

        Global trust formula (MATLAB line 1493):
            DT = gamma_host * gamma_local_peer

        With gamma_self as a conditional modulator (MATLAB lines 1494-1499):
            if gamma_self < self_trust_threshold:
                DT *= gamma_self
        """
        fallback = float(self.config.distributed_trust_fallback)
        self_trust_threshold = float(self.config.trust_threshold)

        _nan = float("nan")
        _default = {
            "gamma_host": fallback,
            "gamma_local_peer": fallback,
            "gamma_self": fallback,
            "d_host_mean": _nan,
            "d_local_mean": _nan,
            "d_self": _nan,
        }

        if host_target_estimate is None:
            return fallback, dict(_default)

        host_vec = self._state_to_array(host_target_estimate)
        target_vec = self._state_to_array(target_data)

        if host_vec is None or target_vec is None:
            return fallback, dict(_default)

        # ===== Host's local relative measurement (fake sensor) =====
        # y_local = [distance(host, target), |v_target - v_host|]
        # In the future this can be replaced by real LiDAR/radar readings.
        y_local = self._compute_relative_measurement(host_state, target_data)

        # ===== gamma_self (MATLAB: gamma_local_our_self) =====
        # Host's OWN global estimate implied relative vs host's local relative.
        # Build VehicleData from host_target_estimate for the helper.
        host_est_as_vd = VehicleData(
            vehicle_id=-1,
            x=float(host_target_estimate[0]),
            y=float(host_target_estimate[1]),
            theta=float(host_target_estimate[2]),
            velocity=float(host_target_estimate[3]) if len(host_target_estimate) > 3 else 0.0,
            acceleration=float(host_target_estimate[4]) if len(host_target_estimate) > 4 else 0.0,
        )
        # Host's global estimate implies a relative state:
        #   y_self_est = [dist(host_local, host_global_est_of_target),
        #                 |v_target_est - v_host|]
        y_self_est = self._compute_relative_measurement(host_state, host_est_as_vd)
        d_self = self._relative_mahalanobis(y_local, y_self_est)
        gamma_self = self._distance_to_gamma(d_self)

        # ===== gamma_host (MATLAB: gamma_cross) =====
        # Full-state Mahalanobis: Host's entire fleet estimates vs Target's entire fleet estimates
        d_host_total = 0.0
        n_host_valid = 0

        if target_fleet_estimates is not None and host_fleet_estimates is not None:
            # target_fleet_estimates is Dict[int, Dict] parsing Target's broadcast
            for vid, b_est_dict in target_fleet_estimates.items():
                if vid >= host_fleet_estimates.shape[1]:
                    continue
                a_est_vec = host_fleet_estimates[:, vid]
                if a_est_vec is None or not np.any(a_est_vec):
                    continue
                    
                b_est_vec = self._state_to_array(b_est_dict)
                if b_est_vec is None:
                    continue
                    
                d_host_total += self._mahalanobis_distance(a_est_vec, b_est_vec)
                n_host_valid += 1

        if n_host_valid > 0:
            d_host_mean_val = d_host_total / n_host_valid
            gamma_host = self._distance_to_gamma(d_host_mean_val)
        else:
            gamma_host = fallback

        # ===== gamma_local_peer (MATLAB: gamma_local) =====
        # Relative-measurement consistency: Compare Host's local relative perception
        # against Target's implied relative measurement of Host and Target
        d_local_total = 0.0
        n_local_valid = 0

        if neighbor_estimates and neighbor_host_estimates:
            # We ONLY want Target's estimate of Target and Target's estimate of Host
            est_target = neighbor_estimates.get(int(target_data.vehicle_id))
            est_host = neighbor_host_estimates.get(int(target_data.vehicle_id))
            
            if est_target is not None and est_host is not None:
                # Target's implied relative: [dist(Target_est_host, Target_est_target), |v_diff|]
                y_target_rel = self._compute_relative_from_estimates(est_host, est_target)

                # Compare against our local relative measurement
                d_local_total += self._relative_mahalanobis(y_local, y_target_rel)
                n_local_valid += 1

        if n_local_valid > 0:
            d_local_mean_val = d_local_total / n_local_valid
            gamma_local_peer = self._distance_to_gamma(d_local_mean_val)
        else:
            # The target didn't have estimates of both host and itself → fallback
            gamma_local_peer = fallback

        # ===== Global trust formula (MATLAB line 1493) =====
        # DT = gamma_host * gamma_local_peer
        distributed = gamma_host * gamma_local_peer

        # ===== Self-consistency modulator (MATLAB lines 1494-1499) =====
        if gamma_self < self_trust_threshold:
            distributed *= gamma_self
            gamma_host *= gamma_self

        distributed = float(np.clip(distributed, 0.0, 1.0))

        return distributed, {
            "gamma_host": float(gamma_host),
            "gamma_local_peer": float(gamma_local_peer),
            "gamma_self": float(gamma_self),
            "d_host_mean": float(d_host_total / max(1, n_host_valid)),
            "d_local_mean": float(d_local_total / max(1, n_local_valid)),
            "d_self": float(d_self),
        }

    def _state_to_array(self, state: object) -> Optional[np.ndarray]:
        """
        Convert VehicleData/ndarray/list to a compact state vector used by
        distributed trust checks: [x, y, theta, velocity, acceleration].
        """
        if state is None:
            return None
        if isinstance(state, VehicleData):
            return np.array(
                [state.x, state.y, state.theta, state.velocity, state.acceleration],
                dtype=float,
            )   
        if isinstance(state, dict):
            try:
                state_vec = np.zeros(5)
                state_vec[0] = state.get("x", 0.0)
                state_vec[1] = state.get("y", 0.0)
                state_vec[2] = state.get("theta", 0.0)
                state_vec[3] = state.get("velocity", state.get("v", 0.0))
                state_vec[4] = state.get("acceleration", state.get("a", 0.0))
                return state_vec
            except Exception:
                pass
        try:
            arr = np.asarray(state, dtype=float).flatten()
            if arr.size == 0:
                return None
            if arr.size < 5:
                return np.pad(arr, (0, 5 - arr.size), mode="constant")
            return arr[:5]
        except Exception:
            return None

    def _mahalanobis_distance(self, x1: np.ndarray, x2: np.ndarray) -> float:
        """Compute Mahalanobis distance with configurable diagonal covariance."""
        diff = x1 - x2
        n = diff.size
        diag = np.asarray(self.config.distributed_trust_covariance_diag, dtype=float)
        if diag.size < n:
            diag = np.pad(diag, (0, n - diag.size), mode="edge")
        diag = diag[:n]
        inv_diag = 1.0 / np.maximum(diag, 1e-6)
        return float(np.dot(diff * inv_diag, diff))

    def _update_trust_levels(
        self, target_id: int, gamma_local: float, gamma_cross: float
    ) -> np.ndarray:
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
            # Paper-style coupling: T_{i,l} = LT_{i,l} * DT_{i,l}
            combined_sample = gamma_local * gamma_cross
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

    # ------------------------------------------------------------------ #
    #  MATLAB-style Dirichlet (dirichlet_method == "matlab")              #
    # ------------------------------------------------------------------ #

    def _dirichlet_matlab(
        self, target_id: int, local_trust_sample: float, global_trust_sample: float
    ) -> float:
        """
        MATLAB-faithful Dirichlet rating vector update & trust score.

        Implements the exact MATLAB TriPTrustModel formulas:
          - update_rating_vector:  R = (1 - λ) * R + r_y
          - calculate_trust_score: σ = dot(weights, (R + C/k) / (C + sum(R)))
          - λ = σ * wt  (aging factor)

        Supports Single (one combined vector) and Dual (separate local/global vectors).
        """
        k = self.config.num_trust_levels
        C = self.config.dirichlet_C
        wt_local = self.config.dirichlet_wt_local
        wt_global = self.config.dirichlet_wt_global

        if self.config.dirichlet_type == "Single":
            # Single: combine samples, use one rating vector
            combined = local_trust_sample * global_trust_sample
            rv = self._get_rating_vector(target_id, "local")
            rv = self._matlab_update_rv(rv, combined, k, C, wt_local)
            self.rating_vectors_local[target_id] = rv
            return float(np.clip(self._matlab_trust_score(rv, k, C), 0.0, 1.0))

        # Dual: separate local and global rating vectors
        rv_local = self._get_rating_vector(target_id, "local")
        rv_local = self._matlab_update_rv(rv_local, local_trust_sample, k, C, wt_local)
        self.rating_vectors_local[target_id] = rv_local
        score_local = self._matlab_trust_score(rv_local, k, C)

        rv_global = self._get_rating_vector(target_id, "global")
        rv_global = self._matlab_update_rv(rv_global, global_trust_sample, k, C, wt_global)
        self.rating_vectors_global[target_id] = rv_global
        score_global = self._matlab_trust_score(rv_global, k, C)

        return float(np.clip(score_local * score_global, 0.0, 1.0))

    def _get_rating_vector(self, target_id: int, trust_type: str) -> np.ndarray:
        """Get or initialize a rating vector for a target."""
        k = self.config.num_trust_levels
        store = self.rating_vectors_local if trust_type == "local" else self.rating_vectors_global
        if target_id not in store:
            store[target_id] = np.zeros(k)
        return store[target_id].copy()

    @staticmethod
    def _matlab_update_rv(
        rv: np.ndarray, trust_sample: float, k: int, C: float, wt: float
    ) -> np.ndarray:
        """
        MATLAB rating vector update (TriPTrustModel.m lines 1186-1214).

        1. Map sample → one-hot trust vector r_y
        2. Compute current trust score σ_y from R
        3. Aging factor λ_y = σ_y * wt
        4. R = (1 - λ_y) * R + r_y
        """
        # Map trust sample to level index (0-based, MATLAB is 1-based)
        level = int(np.clip(round(trust_sample * (k - 1)), 0, k - 1))
        r_y = np.zeros(k)
        r_y[level] = 1.0

        # Current trust score (sigma_y)
        sigma_y = TriPTrustModel._matlab_trust_score(rv, k, C)

        # Aging factor
        lambda_y = sigma_y * wt

        # Update rating vector
        rv = (1.0 - lambda_y) * rv + r_y
        return rv

    @staticmethod
    def _matlab_trust_score(rv: np.ndarray, k: int, C: float) -> float:
        """
        MATLAB trust score from rating vector (TriPTrustModel.m lines 1217-1230).

        S_y = (R + C/k) / (C + sum(R))
        weights = (arange(k) + epsilon) / (k-1 + epsilon)
        score = dot(weights, S_y)
        """
        epsilon = 0.01
        total = np.sum(rv)
        S_y = (rv + C / k) / (C + total)
        weights = (np.arange(k, dtype=float) + epsilon) / (k - 1 + epsilon)
        return float(np.dot(weights, S_y))

    def _set_attack_flags(self, trust: TrustScore):
        """
        Set attack detection flags based on trust samples
        """
        threshold = float(np.clip(self.config.trust_threshold, 0.0, 1.0))

        # Flag: Target may be under attack
        # High local trust but low cross-validation
        trust.flag_target_attack = (
            trust.local_trust_sample > threshold and trust.global_trust_sample < threshold
        )

        # Flag: Global estimate needs verification
        # Low local trust but high cross-validation
        trust.flag_global_est_check = (
            trust.local_trust_sample < threshold and trust.global_trust_sample > threshold
        )

        # Flag: Local measurements unreliable
        trust.flag_local_est_check = trust.local_trust_sample < threshold

    def _update_history(self, target_id: int, score: float):
        """Update trust score history for trend analysis"""
        self.trust_history[target_id].append(score)

        # Limit history size
        if len(self.trust_history[target_id]) > self.max_history_size:
            self.trust_history[target_id] = self.trust_history[target_id][
                -self.max_history_size :
            ]

    def update_beacon_reception(
        self, target_id: int, received: bool, timestamp_s: float
    ):
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

    def update_missing_observation(self, target_id: int, current_time_ns: int) -> TrustScore:
        """
        Update trust when no valid packet was received for target_id in this cycle.
        """
        if target_id not in self.trust_scores:
            # Initialize unknown targets at fallback trust, not full trust.
            initial = float(np.clip(self.config.distributed_trust_fallback, 0.0, 1.0))
            trust = TrustScore(vehicle_id=target_id, final_score=initial)
            self.trust_scores[target_id] = trust
            self.previous_trust_local[target_id] = initial
            self.previous_trust_global[target_id] = initial
        else:
            trust = self.trust_scores[target_id]

        current_time_s = current_time_ns / 1e9
        self.update_beacon_reception(target_id, False, current_time_s)

        never_seen = (
            target_id not in self.last_beacon_times
            and self.beacon_receive_counts[target_id] == 0
        )

        # Missing packet drives beacon score down and decays local/global trust
        trust.timestamp = current_time_s
        trust.beacon_score = 0.0
        trust.quality_factor = 0.0
        if never_seen:
            # Keep unknown vehicles near neutral until at least one packet is observed.
            neutral = float(np.clip(self.config.distributed_trust_fallback, 0.0, 1.0))
            trust.local_trust_sample = neutral
            trust.global_trust_sample = neutral
            self.previous_trust_local[target_id] = neutral
            self.previous_trust_global[target_id] = neutral
        else:
            trust.local_trust_sample = self._apply_trust_decay(
                target_id, trust.local_trust_sample, False, "local"
            )
            trust.global_trust_sample = self._apply_trust_decay(
                target_id, trust.global_trust_sample, False, "global"
            )
        trust.trust_levels = self._update_trust_levels(
            target_id, trust.local_trust_sample, trust.global_trust_sample
        )
        trust.final_score = self._compute_final_score(trust.trust_levels)

        # Smooth to avoid oscillation spikes due to packet jitter
        history = self.trust_history.get(target_id, [])
        if history:
            alpha = float(np.clip(self.config.ema_alpha, 0.0, 1.0))
            trust.final_score = float(
                np.clip(alpha * trust.final_score + (1.0 - alpha) * history[-1], 0.0, 1.0)
            )

        self._set_attack_flags(trust)
        self._update_history(target_id, trust.final_score)
        return trust

    def add_neighbor_trust_report(
        self, reporter_id: int, target_id: int, trust_score: float
    ):
        """
        Add trust report from a neighbor about a target

        Used for cross-validation (gamma_cross)
        """
        self.neighbor_trust_reports[reporter_id][target_id] = trust_score

    def  compute_generalized_trust_vector(
        self,
        all_vehicle_ids: List[int],
        direct_neighbor_trust: Dict[int, float],
        neighbor_opinions: Optional[Dict[int, Dict[int, float]]] = None,
        theta_min: Optional[float] = None,
    ) -> Dict[int, float]:
        """
        Build generalized trust vector O_i(j) following the paper:
        1) O_i(i)=1
        2) O_i(j)=VT_i,j for direct neighbors
        3) Weighted median from credible neighbors when available
        4) Fallback index-distance weighted average
        """
        if theta_min is None:
            theta_min = self.config.trust_vector_theta_min

        neighbor_opinions = neighbor_opinions or {}
        direct_neighbors = [vid for vid in direct_neighbor_trust.keys() if vid != self.vehicle_id]

        O_i: Dict[int, float] = {}
        for j in all_vehicle_ids:
            if j == self.vehicle_id:
                O_i[j] = 1.0
                continue

            if j in direct_neighbor_trust:
                O_i[j] = float(np.clip(direct_neighbor_trust[j], 0.0, 1.0))
                continue

            # Credible one-hop set: trusted direct neighbors with opinion on j
            credible = []
            for k in direct_neighbors:
                vt_i_k = direct_neighbor_trust.get(k, 0.0)
                if vt_i_k <= theta_min:
                    continue
                if k in neighbor_opinions and j in neighbor_opinions[k]:
                    credible.append((k, float(neighbor_opinions[k][j]), vt_i_k))

            if credible:
                values = [v for _, v, _ in credible]
                raw_weights = [w for _, _, w in credible]
                weight_sum = sum(raw_weights)
                if weight_sum > 0:
                    weights = [w / weight_sum for w in raw_weights]
                else:
                    weights = [1.0 / len(raw_weights)] * len(raw_weights)
                O_i[j] = self._weighted_median(values, weights)
                continue

            # Index-distance fallback over direct neighbors
            if direct_neighbors:
                weighted_sum = 0.0
                g_sum = 0.0
                for k in direct_neighbors:
                    # Prefer propagated opinion when available, fallback to direct trust
                    opinion_k_j = neighbor_opinions.get(k, {}).get(
                        j, direct_neighbor_trust.get(k, 0.0)
                    )
                    g_i_k = 1.0 / (1.0 + abs(self.vehicle_id - k))
                    weighted_sum += g_i_k * float(opinion_k_j)
                    g_sum += g_i_k
                O_i[j] = float(np.clip(weighted_sum / g_sum, 0.0, 1.0)) if g_sum > 0 else 0.0
            else:
                O_i[j] = float(self.config.distributed_trust_fallback)

        self.opinion_vector = O_i
        return O_i

    def _weighted_median(self, values: List[float], weights: List[float]) -> float:
        """Weighted median robust aggregation."""
        if not values:
            return 0.0
        pairs = sorted(zip(values, weights), key=lambda x: x[0])
        cumulative = 0.0
        for value, weight in pairs:
            cumulative += max(weight, 0.0)
            if cumulative >= 0.5:
                return float(np.clip(value, 0.0, 1.0))
        return float(np.clip(pairs[-1][0], 0.0, 1.0))

    def get_trust_score(self, target_id: int) -> Optional[TrustScore]:
        """Get current trust score for a vehicle"""
        return self.trust_scores.get(target_id)

    def get_all_trust_scores(self) -> Dict[int, float]:
        """Get all trust scores as dict {vehicle_id: final_score}"""
        return {vid: ts.final_score for vid, ts in self.trust_scores.items()}

    def get_generalized_trust_vector(self) -> Dict[int, float]:
        """Get latest generalized trust vector O_i(j)."""
        return self.opinion_vector.copy()

    def get_trusted_vehicles(self, threshold: float = None) -> List[int]:
        """Get list of vehicle IDs with trust above threshold"""
        if threshold is None:
            threshold = self.config.trust_threshold

        return [
            vid for vid, ts in self.trust_scores.items() if ts.final_score >= threshold
        ]

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
                "target_attack": ts.flag_target_attack,
                "global_est_check": ts.flag_global_est_check,
                "local_est_check": ts.flag_local_est_check,
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
        self.previous_states.clear()
        self.distance_buffers.clear()
        self.previous_trust_local.clear()
        self.previous_trust_global.clear()
        self.opinion_vector = {self.vehicle_id: 1.0}
        self.rating_vectors_local.clear()
        self.rating_vectors_global.clear()

