"""
Trust-Based Weight Module for Distributed Observer

Calculates adaptive consensus weights based on trust scores.
Implements trust-aware weight allocation with influence capping and temporal smoothing.

Key features:
1. Trust-aware per-target weights for direct, self, and neighbor channels
2. Trust-proportional distribution among trusted neighbors
3. Influence capping to prevent single neighbor dominance
4. EMA smoothing for temporal stability
5. Virtual graph construction for enhanced connectivity
"""
import numpy as np
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field, fields as dataclass_fields
from collections import defaultdict


@dataclass
class WeightConfig:
    """Configuration for weight calculation"""
    # Weight calculation type
    weight_type: str = "trust_based"  # 'equal', 'trust_based', 'graph_based', 'paper'
    
    # Base gains used by per-target raw-score normalization
    w0_fixed: float = 0.3          # Base gain / nominal maximum for direct measurement
    w_self_base: float = 0.2       # Base self/prediction gain
    
    # Neighbor weight constraints
    w_cap: float = 0.4             # Maximum weight per neighbor (influence cap)
    kappa: int = 5                 # Maximum neighbors to consider
    
    # Smoothing
    eta: float = 0.15              # EMA smoothing factor
    enable_smoothing: bool = True  # Enable temporal smoothing
    
    # Trust integration
    trust_threshold: float = 0.5   # Minimum trust to be considered neighbor
    use_distance_weighting: bool = False  # Weight by distance (reserved)
    startup_fixed_duration_s: float = 5.0  # Use fixed non-trust weights during startup
    use_gamma_self_weight_adaptation: bool = True
    gamma_self_weight_floor: float = 0.25

    # Flag-driven w₀ adaptation factors
    flag_w0_target_attack_factor: float = 1.0    # Keep w₀ full when target under attack (local is reliable)
    flag_w0_global_est_check_factor: float = 0.3 # Reduce w₀ when local sensors are faulty
    flag_w0_local_est_check_factor: float = 0.5  # Reduce w₀ when local measurements unreliable

    def __post_init__(self):
        """
        Align legacy defaults with the current flag semantics.

        Explicit config values loaded from YAML still override these fields.
        """
        if self.flag_w0_target_attack_factor == 1.0:
            self.flag_w0_target_attack_factor = 0.25
        if self.flag_w0_global_est_check_factor == 0.3:
            self.flag_w0_global_est_check_factor = 1.25

    @classmethod
    def from_dict(cls, d: dict) -> "WeightConfig":
        """Create config from a dictionary, using dataclass defaults for missing keys."""
        if not d:
            return cls()
        known = {f.name for f in dataclass_fields(cls)}
        kwargs = {k: v for k, v in d.items() if k in known}
        return cls(**kwargs)


@dataclass
class WeightResult:
    """Result of weight calculation"""
    weights: np.ndarray            # Weight array [fleet_size + 1] (includes virtual node)
    trusted_neighbors: List[int]   # List of trusted neighbor IDs
    neighbor_weights: Dict[int, float]  # Individual neighbor weights
    w0: float                      # Virtual node weight
    w_self: float                  # Self weight
    total_neighbor_weight: float   # Total weight allocated to neighbors
    trust_source_scores: Dict[int, float] = field(default_factory=dict)
    mean_source_trust: float = 0.0
    mean_trusted_trust: float = 0.0
    weighted_neighbor_trust: float = 0.0


class WeightTrustModule:
    """
    Trust-Based Weight Calculator for Distributed Observer
    
    Calculates adaptive weights for consensus-based state estimation
    using trust scores from the TriP Trust Model.
    
    Supports two modes:
    1. Simple mode: Uses trust_scores dict directly (no graph)
    2. Graph mode: Uses adjacency matrix for neighbor discovery (like old implementation)
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int,
                 config: WeightConfig = None, logger=None,
                 graph: np.ndarray = None, trust_threshold: float = None):
        """
        Initialize Weight Trust Module
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            config: Weight configuration parameters
            logger: Logger instance
            graph: Optional adjacency matrix (n x n) for neighbor discovery
            trust_threshold: Optional trust threshold (overrides config)
        """
        self.vehicle_id = vehicle_id
        self.fleet_size = fleet_size
        self.config = config or WeightConfig()
        self.logger = logger
        
        # Override trust threshold if provided
        if trust_threshold is not None:
            self.config.trust_threshold = trust_threshold
        
        # Adjacency matrix for graph-based neighbor discovery
        # If not provided, assume fully connected graph
        if graph is not None:
            self.graph = np.array(graph)
        else:
            # Default: fully connected (all vehicles communicate)
            self.graph = self.generate_fully_connected_graph(fleet_size)
        
        # Weight storage: includes virtual node at index 0
        # [w0, w1, w2, ..., wN] where w0 is virtual node
        self.weights = np.zeros(fleet_size + 1)
        self.weights[0] = self.config.w0_fixed
        self.weights[self.vehicle_id + 1] = self.config.w_self_base
        
        # Previous weights for smoothing
        self.prev_weights = self.weights.copy()
        
        # Virtual graph (adjacency matrix with extra node for local measurement)
        self.virtual_graph: Optional[np.ndarray] = None
    
    def calculate_weights(self, trust_scores: Dict[int, float],
                          neighbor_ids: List[int] = None) -> WeightResult:
        """
        Calculate weights for distributed observer based on configured weight_type
        
        Dispatches to appropriate weight calculation method:
        - 'equal': Equal weights for all neighbors
        - 'trust_based': Trust-proportional weights with fixed w0/w_self
        - 'graph_based': Graph topology-based weights
        - 'paper': Paper-style bounded equal weights over legitimate neighbors
        
        Args:
            trust_scores: Dict of {vehicle_id: trust_score}
            neighbor_ids: Optional list of neighbor IDs to consider
            
        Returns:
            WeightResult with weight array and metadata
        """
        if self.config.weight_type == "equal":
            return self._calculate_equal_weights(trust_scores, neighbor_ids)
        elif self.config.weight_type == "graph_based":
            return self.calculate_weights_with_graph(trust_scores)
        elif self.config.weight_type == "paper":
            return self._calculate_paper_style_weights(trust_scores, neighbor_ids)
        else:  # trust_based (default)
            return self._calculate_trust_based_weights(trust_scores, neighbor_ids)

    def _build_fixed_channel_weights(
        self, available_neighbors: List[int], include_anchor: bool
    ) -> Dict[str, object]:
        """
        Build deterministic non-trust weights for startup warmup.

        The base anchor/self gains stay fixed and any remaining budget is
        distributed equally across currently available neighbors, with capped
        overflow returned to self.
        """
        unique_neighbors = []
        for neighbor_id in available_neighbors:
            neighbor_id = int(neighbor_id)
            if neighbor_id == self.vehicle_id or neighbor_id in unique_neighbors:
                continue
            unique_neighbors.append(neighbor_id)
        unique_neighbors = unique_neighbors[: self.config.kappa]

        anchor_weight = max(0.0, float(self.config.w0_fixed)) if include_anchor else 0.0
        self_weight = max(0.0, float(self.config.w_self_base))
        neighbor_budget = max(0.0, 1.0 - anchor_weight - self_weight)

        neighbor_weights: Dict[int, float] = {}
        if unique_neighbors and neighbor_budget > 0.0:
            equal_basis = {
                neighbor_id: 1.0 / float(len(unique_neighbors))
                for neighbor_id in unique_neighbors
            }
            neighbor_weights = {
                neighbor_id: neighbor_budget * equal_basis[neighbor_id]
                for neighbor_id in unique_neighbors
            }
            neighbor_weights, overflow_to_self = self._apply_neighbor_cap_for_target(
                neighbor_weights, equal_basis
            )
            self_weight += overflow_to_self
        else:
            self_weight += neighbor_budget

        used = anchor_weight + self_weight + sum(neighbor_weights.values())
        residual = 1.0 - used
        if abs(residual) > 1e-12:
            self_weight = max(0.0, self_weight + residual)

        return {
            "w0": float(anchor_weight),
            "neighbors": {
                int(neighbor_id): float(weight)
                for neighbor_id, weight in neighbor_weights.items()
            },
            "w_self": float(self_weight),
        }

    def calculate_startup_weights(
        self, neighbor_ids: List[int] = None
    ) -> WeightResult:
        """Return stable non-trust summary weights for the startup warmup."""
        available_neighbors = []
        if neighbor_ids:
            available_neighbors = [
                int(neighbor_id)
                for neighbor_id in neighbor_ids
                if int(neighbor_id) != self.vehicle_id
            ]
        weights = self._build_fixed_channel_weights(
            available_neighbors=available_neighbors,
            include_anchor=True,
        )

        new_weights = np.zeros(self.fleet_size + 1)
        new_weights[0] = float(weights["w0"])
        new_weights[self.vehicle_id + 1] = float(weights["w_self"])

        neighbor_weights = {
            int(neighbor_id): float(weight)
            for neighbor_id, weight in weights["neighbors"].items()
        }
        for neighbor_id, weight in neighbor_weights.items():
            if 0 <= neighbor_id < self.fleet_size:
                new_weights[neighbor_id + 1] = weight

        startup_trust = {int(neighbor_id): 1.0 for neighbor_id in neighbor_weights.keys()}
        trust_summary = self._summarize_trust_context(
            trust_scores=startup_trust,
            trusted_neighbors=list(neighbor_weights.keys()),
            neighbor_weights=neighbor_weights,
        )

        return WeightResult(
            weights=new_weights,
            trusted_neighbors=list(neighbor_weights.keys()),
            neighbor_weights=neighbor_weights,
            w0=float(weights["w0"]),
            w_self=float(weights["w_self"]),
            total_neighbor_weight=float(sum(neighbor_weights.values())),
            trust_source_scores=trust_summary["source_scores"],
            mean_source_trust=trust_summary["mean_source_trust"],
            mean_trusted_trust=trust_summary["mean_trusted_trust"],
            weighted_neighbor_trust=trust_summary["weighted_neighbor_trust"],
        )

    def _summarize_trust_context(
        self,
        trust_scores: Dict[int, float],
        trusted_neighbors: List[int],
        neighbor_weights: Dict[int, float],
    ) -> Dict[str, float]:
        """
        Build compact trust/weight summary metrics for logging and diagnostics.
        """
        source_scores = {
            int(vid): float(score)
            for vid, score in trust_scores.items()
            if int(vid) != self.vehicle_id
        }

        source_vals = list(source_scores.values())
        trusted_vals = [
            float(source_scores.get(int(vid), trust_scores.get(int(vid), 0.0)))
            for vid in trusted_neighbors
        ]

        mean_source = float(np.mean(source_vals)) if source_vals else 0.0
        mean_trusted = float(np.mean(trusted_vals)) if trusted_vals else 0.0

        total_weight = float(sum(float(w) for w in neighbor_weights.values()))
        if total_weight > 0.0:
            weighted_trust = float(
                sum(
                    float(neighbor_weights.get(vid, 0.0))
                    * float(source_scores.get(vid, trust_scores.get(vid, 0.0)))
                    for vid in neighbor_weights.keys()
                )
                / total_weight
            )
        else:
            weighted_trust = 0.0

        return {
            "source_scores": source_scores,
            "mean_source_trust": mean_source,
            "mean_trusted_trust": mean_trusted,
            "weighted_neighbor_trust": weighted_trust,
        }

    def _calculate_paper_style_weights(
        self, trust_scores: Dict[int, float], neighbor_ids: List[int] = None
    ) -> WeightResult:
        """
        Paper-style summary weights for logging/global use.

        The actual observer update should use per-target
        `calculate_paper_weights_for_target(...)`.
        """
        trusted = self._get_trusted_neighbors(trust_scores, neighbor_ids)

        # Include anchor in summary mode
        include_anchor = True
        n_legitimate = len(trusted) + (1 if include_anchor else 0)

        # Paper normalization factor n = max(kappa, |LN| + 1[self])
        n_w = max(self.config.kappa, n_legitimate + 1)
        base_w = 1.0 / n_w

        new_weights = np.zeros(self.fleet_size + 1)
        neighbor_weights = {}

        if include_anchor:
            new_weights[0] = base_w

        for vid in trusted:
            new_weights[vid + 1] = base_w
            neighbor_weights[vid] = base_w

        # Residual self coefficient so total external influence stays <= 1
        new_weights[self.vehicle_id + 1] = max(
            0.0, 1.0 - new_weights[0] - sum(neighbor_weights.values())
        )

        # Optional smoothing
        if self.config.enable_smoothing and np.any(self.prev_weights > 0):
            eta = self.config.eta
            smoothed_weights = eta * new_weights + (1 - eta) * self.prev_weights
            smoothed_weights = smoothed_weights / np.sum(smoothed_weights)
            new_weights = smoothed_weights

        self.prev_weights = new_weights.copy()
        self.weights = new_weights
        trust_summary = self._summarize_trust_context(
            trust_scores=trust_scores,
            trusted_neighbors=trusted,
            neighbor_weights=neighbor_weights,
        )

        return WeightResult(
            weights=new_weights,
            trusted_neighbors=trusted,
            neighbor_weights=neighbor_weights,
            w0=new_weights[0],
            w_self=new_weights[self.vehicle_id + 1],
            total_neighbor_weight=sum(neighbor_weights.values()),
            trust_source_scores=trust_summary["source_scores"],
            mean_source_trust=trust_summary["mean_source_trust"],
            mean_trusted_trust=trust_summary["mean_trusted_trust"],
            weighted_neighbor_trust=trust_summary["weighted_neighbor_trust"],
        )
    
    def _calculate_equal_weights(self, trust_scores: Dict[int, float],
                                  neighbor_ids: List[int] = None) -> WeightResult:
        """
        Calculate equal weights for all neighbors (simple consensus)
        
        All available neighbors get equal weight: w = 1 / n_total
        Row-stochastic guarantee: Σ weights = 1.0
        
        Args:
            trust_scores: Dict of {vehicle_id: trust_score} (used for neighbor discovery)
            neighbor_ids: Optional list of neighbor IDs to consider
            
        Returns:
            WeightResult with equal weights
        """
        # Initialize
        new_weights = np.zeros(self.fleet_size + 1)
        neighbor_weights = {}
        
        # Get available neighbors (above threshold)
        trusted = self._get_trusted_neighbors(trust_scores, neighbor_ids)
        
        # Calculate equal weights
        n_total = len(trusted) + 1  # neighbors + virtual node (w0)
        equal_weight = 1.0 / n_total if n_total > 0 else 0.0
        
        # Assign equal weight to virtual node
        new_weights[0] = equal_weight
        
        # Assign equal weight to each neighbor
        for vid in trusted:
            new_weights[vid + 1] = equal_weight
            neighbor_weights[vid] = equal_weight
        
        # Self weight is 0 in pure equal consensus
        new_weights[self.vehicle_id + 1] = 0.0
        
        # Normalize (should already be 1.0, but ensure it)
        weight_sum = np.sum(new_weights)
        if weight_sum > 0:
            new_weights = new_weights / weight_sum
        
        # Apply EMA smoothing if enabled
        if self.config.enable_smoothing and np.any(self.prev_weights > 0):
            eta = self.config.eta
            smoothed_weights = eta * new_weights + (1 - eta) * self.prev_weights
            smoothed_weights = smoothed_weights / np.sum(smoothed_weights)
            new_weights = smoothed_weights
        
        # Store for next iteration
        self.prev_weights = new_weights.copy()
        self.weights = new_weights
        trust_summary = self._summarize_trust_context(
            trust_scores=trust_scores,
            trusted_neighbors=trusted,
            neighbor_weights=neighbor_weights,
        )

        # Build result
        result = WeightResult(
            weights=new_weights,
            trusted_neighbors=trusted,
            neighbor_weights=neighbor_weights,
            w0=new_weights[0],
            w_self=new_weights[self.vehicle_id + 1],
            total_neighbor_weight=sum(neighbor_weights.values()),
            trust_source_scores=trust_summary["source_scores"],
            mean_source_trust=trust_summary["mean_source_trust"],
            mean_trusted_trust=trust_summary["mean_trusted_trust"],
            weighted_neighbor_trust=trust_summary["weighted_neighbor_trust"],
        )
        
        return result
    
    def _calculate_trust_based_weights(self, trust_scores: Dict[int, float],
                                        neighbor_ids: List[int] = None) -> WeightResult:
        """
        Calculate trust-based weights for distributed observer
        
        Algorithm:
        1. Set fixed weights: w0 = 0.3, w_self = 0.2
        2. Calculate neighbor budget: 1.0 - w0 - w_self = 0.5
        3. Get trusted neighbors (trust > threshold)
        4. Distribute budget proportional to trust
        5. Apply influence cap (max 40% per neighbor)
        6. Apply EMA smoothing
        
        Args:
            trust_scores: Dict of {vehicle_id: trust_score}
            neighbor_ids: Optional list of neighbor IDs to consider
            
        Returns:
            WeightResult with weight array and metadata
        """
        # Initialize result
        new_weights = np.zeros(self.fleet_size + 1)
        neighbor_weights = {}
        
        # Step 1: Set fixed weights
        w0 = self.config.w0_fixed
        w_self = self.config.w_self_base
        new_weights[0] = w0
        new_weights[self.vehicle_id + 1] = w_self
        
        # Step 2: Calculate neighbor budget
        neighbor_budget = 1.0 - w0 - w_self
        
        # Step 3: Get trusted neighbors
        trusted = self._get_trusted_neighbors(trust_scores, neighbor_ids)
        
        # Step 4 & 5: Distribute budget and apply cap
        if trusted:
            # Calculate normalized trust weights
            trust_sum = sum(trust_scores.get(vid, 0.0) for vid in trusted)
            
            if trust_sum > 0:
                for vid in trusted:
                    trust = trust_scores.get(vid, 0.0)
                    raw_weight = (trust / trust_sum) * neighbor_budget
                    
                    # Apply influence cap
                    capped_weight = min(raw_weight, self.config.w_cap)
                    neighbor_weights[vid] = capped_weight
                    new_weights[vid + 1] = capped_weight
            else:
                # No trusted neighbors with positive trust
                # Add budget to self
                new_weights[self.vehicle_id + 1] += neighbor_budget
        else:
            # No trusted neighbors - add all budget to self
            new_weights[self.vehicle_id + 1] += neighbor_budget
        
        # Normalize to sum to 1.0
        weight_sum = np.sum(new_weights)
        if weight_sum > 0:
            new_weights = new_weights / weight_sum
        
        # Step 6: Apply EMA smoothing
        if self.config.enable_smoothing and np.any(self.prev_weights > 0):
            eta = self.config.eta
            smoothed_weights = eta * new_weights + (1 - eta) * self.prev_weights
            
            # Re-normalize after smoothing
            smoothed_weights = smoothed_weights / np.sum(smoothed_weights)
            new_weights = smoothed_weights
        
        # Store for next iteration
        self.prev_weights = new_weights.copy()
        self.weights = new_weights
        trust_summary = self._summarize_trust_context(
            trust_scores=trust_scores,
            trusted_neighbors=trusted,
            neighbor_weights=neighbor_weights,
        )

        # Build result
        result = WeightResult(
            weights=new_weights,
            trusted_neighbors=trusted,
            neighbor_weights=neighbor_weights,
            w0=new_weights[0],
            w_self=new_weights[self.vehicle_id + 1],
            total_neighbor_weight=sum(neighbor_weights.values()),
            trust_source_scores=trust_summary["source_scores"],
            mean_source_trust=trust_summary["mean_source_trust"],
            mean_trusted_trust=trust_summary["mean_trusted_trust"],
            weighted_neighbor_trust=trust_summary["weighted_neighbor_trust"],
        )
        
        return result
    
    @staticmethod
    def _clip_unit(value, default: float = 1.0) -> float:
        """Clip scalar inputs to [0, 1], with a safe default for NaN/None."""
        if value is None:
            return float(default)
        try:
            value = float(value)
        except (TypeError, ValueError):
            return float(default)
        if not np.isfinite(value):
            return float(default)
        return float(np.clip(value, 0.0, 1.0))

    def _resolve_flag_group_factors(self, trust_score) -> Tuple[float, float, float]:
        """
        Resolve raw-score multipliers for direct, self, and neighbor channels.

        Precedence mirrors the trust model semantics:
        target_attack > global_est_check > local_est_check.
        """
        direct_factor = 1.0
        self_factor = 1.0
        neighbor_factor = 1.0

        if trust_score is None:
            return direct_factor, self_factor, neighbor_factor

        if trust_score.flag_target_attack:
            # Local bad, global bad:
            # fall back to self/prediction and suppress all external channels.
            direct_factor = min(
                1.0, max(0.0, float(self.config.flag_w0_target_attack_factor))
            )
            neighbor_factor = direct_factor
            self_factor = 1.0 + (1.0 - direct_factor)
        elif trust_score.flag_global_est_check:
            # Local good, global bad:
            # trust the direct owner-target channel more than fleet consensus.
            direct_factor = max(
                1.0, float(self.config.flag_w0_global_est_check_factor)
            )
            neighbor_factor = max(0.0, 1.0 / direct_factor)
        elif trust_score.flag_local_est_check:
            # Local bad (broad warning, after the more specific cases above):
            # reduce direct and lean harder on trusted neighbors.
            direct_factor = min(
                1.0, max(0.0, float(self.config.flag_w0_local_est_check_factor))
            )
            neighbor_factor = 1.0 + (1.0 - direct_factor)

        return direct_factor, self_factor, neighbor_factor

    def _apply_neighbor_cap_for_target(
        self,
        neighbor_weights: Dict[int, float],
        neighbor_basis: Dict[int, float],
    ) -> Tuple[Dict[int, float], float]:
        """
        Enforce per-neighbor caps and move any irreducible overflow to self.
        """
        if not neighbor_weights:
            return {}, 0.0

        capped = {
            int(neighbor_id): float(weight)
            for neighbor_id, weight in neighbor_weights.items()
        }
        overflow_to_self = 0.0
        tol = 1e-12

        while True:
            over_cap = [
                neighbor_id
                for neighbor_id, weight in capped.items()
                if weight > self.config.w_cap + tol
            ]
            if not over_cap:
                break

            overflow = 0.0
            for neighbor_id in over_cap:
                overflow += capped[neighbor_id] - self.config.w_cap
                capped[neighbor_id] = self.config.w_cap

            uncapped = [
                neighbor_id
                for neighbor_id, weight in capped.items()
                if weight < self.config.w_cap - tol
            ]
            if overflow <= tol or not uncapped:
                overflow_to_self += overflow
                break

            basis_sum = sum(
                max(0.0, float(neighbor_basis.get(neighbor_id, 0.0)))
                for neighbor_id in uncapped
            )
            if basis_sum <= tol:
                overflow_to_self += overflow
                break

            for neighbor_id in uncapped:
                basis = max(0.0, float(neighbor_basis.get(neighbor_id, 0.0)))
                capped[neighbor_id] += overflow * (basis / basis_sum)

        return capped, overflow_to_self

    def calculate_weights_for_target(self, target_id: int,
                                      trust_scores: Dict[int, float],
                                      neighbor_fleet_estimates: Dict[int, Dict],
                                      direct_measurement: Optional[np.ndarray] = None,
                                      target_trust_obj=None) -> Dict[str, float]:
        """
        Calculate weights for estimating a specific target vehicle.

        For trust-based mode, the target row is built from raw group scores and
        normalized jointly:
        - direct channel depends on the target's local/direct reliability
        - neighbor channels depend on neighbor source trust
        - self channel absorbs residual confidence in the host prediction
        """
        if self.config.weight_type == "paper":
            # In paper mode, `trust_scores` is expected to be O_i(j) generalized trust
            # or direct trust when generalized trust is unavailable.
            target_local_trust = trust_scores.get(target_id, 0.0)
            return self.calculate_paper_weights_for_target(
                target_id=target_id,
                opinion_scores=trust_scores,
                target_local_trust=target_local_trust,
                neighbor_fleet_estimates=neighbor_fleet_estimates,
                direct_measurement=direct_measurement,
            )
        if self.config.weight_type == "equal":
            available_neighbors = []
            for neighbor_id, fleet_est in neighbor_fleet_estimates.items():
                if (
                    target_id in fleet_est
                    and neighbor_id != self.vehicle_id
                    and neighbor_id != target_id
                ):
                    trust = trust_scores.get(neighbor_id, 0.0)
                    if trust >= self.config.trust_threshold:
                        available_neighbors.append(int(neighbor_id))

            available_neighbors = available_neighbors[: self.config.kappa]

            channel_count = len(available_neighbors) + (
                1 if direct_measurement is not None else 0
            )
            if channel_count <= 0:
                return {"w0": 0.0, "neighbors": {}, "w_self": 1.0}

            equal_weight = 1.0 / float(channel_count)
            return {
                "w0": equal_weight if direct_measurement is not None else 0.0,
                "neighbors": {
                    neighbor_id: equal_weight for neighbor_id in available_neighbors
                },
                "w_self": 0.0,
            }

        # Get neighbors who have estimates for target
        available_neighbors = []
        for neighbor_id, fleet_est in neighbor_fleet_estimates.items():
            if (
                target_id in fleet_est
                and neighbor_id != self.vehicle_id
                and neighbor_id != target_id
            ):
                trust = trust_scores.get(neighbor_id, 0.0)
                if trust >= self.config.trust_threshold:
                    available_neighbors.append((neighbor_id, trust))

        # Sort by trust (highest first) and limit by kappa
        available_neighbors.sort(key=lambda x: x[1], reverse=True)
        available_neighbors = available_neighbors[:self.config.kappa]

        direct_factor, self_factor, neighbor_factor = (
            self._resolve_flag_group_factors(target_trust_obj)
        )

        direct_available = direct_measurement is not None
        if target_trust_obj is not None:
            local_trust = self._clip_unit(
                getattr(target_trust_obj, "local_trust_sample", None), default=1.0
            )
            # quality_factor = self._clip_unit(
            #     getattr(target_trust_obj, "quality_factor", None), default=1.0
            # )
        else:
            local_trust = 1.0
            # quality_factor = 1.0

        # direct_reliability = local_trust * quality_factor if direct_available else 0.0
        direct_reliability = local_trust if direct_available else 0.0
        anchor_gain = max(0.0, float(self.config.w0_fixed))
        self_gain = max(0.0, float(self.config.w_self_base))
        neighbor_gain = max(0.0, 1.0 - anchor_gain - self_gain)

        if (
            self.config.use_gamma_self_weight_adaptation
            and target_trust_obj is not None
        ):
            gamma_self = self._clip_unit(
                getattr(target_trust_obj, "gamma_self", None), default=1.0
            )
            gamma_self_floor = self._clip_unit(
                self.config.gamma_self_weight_floor, default=0.25
            )
            self_factor *= gamma_self_floor + (1.0 - gamma_self_floor) * gamma_self

        w0_raw = anchor_gain * direct_reliability * direct_factor
        w_self_raw = self_gain * self_factor

        neighbor_basis = {}
        trust_sum = sum(max(0.0, trust) for _, trust in available_neighbors)
        if trust_sum > 0.0:
            for neighbor_id, trust in available_neighbors:
                neighbor_basis[neighbor_id] = max(0.0, float(trust)) / trust_sum

        neighbor_raw = {
            neighbor_id: neighbor_gain * neighbor_factor * basis
            for neighbor_id, basis in neighbor_basis.items()
        }

        raw_total = w0_raw + w_self_raw + sum(neighbor_raw.values())
        if raw_total <= 0.0:
            return {"w0": 0.0, "neighbors": {}, "w_self": 1.0}

        weights = {
            "w0": w0_raw / raw_total,
            "neighbors": {
                neighbor_id: raw_weight / raw_total
                for neighbor_id, raw_weight in neighbor_raw.items()
            },
            "w_self": w_self_raw / raw_total,
        }

        if weights["neighbors"]:
            capped_neighbors, overflow_to_self = self._apply_neighbor_cap_for_target(
                weights["neighbors"], neighbor_basis
            )
            weights["neighbors"] = capped_neighbors
            weights["w_self"] += overflow_to_self

        used = weights["w0"] + weights["w_self"] + sum(weights["neighbors"].values())
        residual = 1.0 - used
        if abs(residual) > 1e-12:
            weights["w_self"] = max(0.0, weights["w_self"] + residual)

        return weights

    def calculate_startup_weights_for_target(
        self,
        target_id: int,
        neighbor_fleet_estimates: Dict[int, Dict],
        direct_measurement: Optional[np.ndarray] = None,
    ) -> Dict[str, object]:
        """Return stable non-trust target weights for the startup warmup."""
        available_neighbors = []
        for neighbor_id, fleet_est in neighbor_fleet_estimates.items():
            if (
                neighbor_id == self.vehicle_id
                or neighbor_id == target_id
                or target_id not in fleet_est
            ):
                continue
            available_neighbors.append(int(neighbor_id))

        return self._build_fixed_channel_weights(
            available_neighbors=available_neighbors,
            include_anchor=direct_measurement is not None,
        )

    def calculate_paper_weights_for_target(
        self,
        target_id: int,
        opinion_scores: Dict[int, float],
        target_local_trust: float,
        neighbor_fleet_estimates: Dict[int, Dict],
        direct_measurement: Optional[np.ndarray] = None,
    ) -> Dict[str, float]:
        """
        Paper-like per-target weight design:
        LN_i^(j) = {l in N_i | O_i(l) >= theta_min} U ({0} if LT_i,j >= theta_min)
        n = max{kappa, |LN_i^(j)| + 1}
        w_il^(j) = 1/n for l in LN_i^(j), else 0.

        Returned dictionary follows the estimator update interface.
        """
        theta_min = self.config.trust_threshold

        # Build candidate neighbors who actually provide target estimates
        candidates = []
        for neighbor_id, fleet_est in neighbor_fleet_estimates.items():
            if neighbor_id == self.vehicle_id:
                continue
            if target_id in fleet_est:
                candidates.append(neighbor_id)

        legitimate_neighbors = [
            vid
            for vid in candidates
            if opinion_scores.get(vid, 0.0) >= theta_min
        ]

        include_anchor = (
            target_local_trust >= theta_min and direct_measurement is not None
        )

        n_legitimate = len(legitimate_neighbors) + (1 if include_anchor else 0)
        n_w = max(self.config.kappa, n_legitimate + 1)  # +1 for self residual channel
        base_w = 1.0 / n_w

        weights = {
            "w0": base_w if include_anchor else 0.0,
            "neighbors": {vid: base_w for vid in legitimate_neighbors},
            "w_self": 0.0,
        }

        used = weights["w0"] + sum(weights["neighbors"].values())
        weights["w_self"] = max(0.0, 1.0 - used)
        return weights

    @staticmethod
    def apply_flag_adaptation(
        weights: Dict[str, object],
        trust_score,
        config: WeightConfig,
    ) -> Dict[str, object]:
        """
        
        Adjust per-target weights based on trust model attack flags.

        Flag logic:
        - flag_target_attack  (local OK, global BAD):  keep w₀ high
        - flag_global_est_check (local BAD, global OK): reduce w₀
        - flag_local_est_check  (local BAD):            reduce w₀

        Freed budget is absorbed by w_self (own dynamics prediction).
        """
        if trust_score is None:
            return weights

        w0_original = weights.get("w0", 0.0)

        if trust_score.flag_target_attack:
            # Local sensors say OK, fleet says BAD → trust your own sensor
            factor = config.flag_w0_target_attack_factor
            weights["w0"] = max(w0_original, w0_original * factor)
        elif trust_score.flag_global_est_check:
            # Local sensors BAD, fleet says OK → reduce local, lean on neighbors
            weights["w0"] = w0_original * config.flag_w0_global_est_check_factor
        elif trust_score.flag_local_est_check:
            # Local measurements unreliable
            weights["w0"] = w0_original * config.flag_w0_local_est_check_factor
        else:
            return weights  # no flag active, nothing to change

        # Rebalance: w_self absorbs the residual
        used = weights["w0"] + sum(weights.get("neighbors", {}).values())
        weights["w_self"] = max(0.0, 1.0 - used)
        return weights
    
    def _get_trusted_neighbors(self, trust_scores: Dict[int, float],
                                neighbor_ids: List[int] = None) -> List[int]:
        """
        Get list of trusted neighbors (trust above threshold)
        
        Args:
            trust_scores: Trust scores for vehicles
            neighbor_ids: Optional filter list
            
        Returns:
            List of trusted neighbor IDs, sorted by trust (descending)
        """
        trusted = []
        
        candidates = neighbor_ids if neighbor_ids else list(trust_scores.keys())
        
        for vid in candidates:
            if vid == self.vehicle_id:
                continue
            
            trust = trust_scores.get(vid, 0.0)
            if trust >= self.config.trust_threshold:
                trusted.append((vid, trust))
        
        # Sort by trust (highest first)
        trusted.sort(key=lambda x: x[1], reverse=True)
        
        # Apply kappa limit
        trusted = trusted[:self.config.kappa]
        
        return [vid for vid, _ in trusted]
    
    def generate_virtual_graph(self, physical_graph: np.ndarray = None) -> np.ndarray:
        """
        Generate virtual graph with extra node for local measurement.
        
        The virtual graph adds node 0 representing the local measurement,
        connected to the host vehicle and its neighbors.
        
        Args:
            physical_graph: Adjacency matrix of physical topology [N x N]
                           If None, uses self.graph
            
        Returns:
            Virtual graph adjacency matrix [(N+1) x (N+1)]
        """
        if physical_graph is None:
            physical_graph = self.graph
        
        n = physical_graph.shape[0]
        virtual_graph = np.zeros((n + 1, n + 1))
        
        # Copy physical connections (shifted by 1)
        virtual_graph[1:, 1:] = physical_graph
        
        # Set bidirectional edge between virtual node (0) and host vehicle
        virtual_graph[0, self.vehicle_id + 1] = 1
        virtual_graph[self.vehicle_id + 1, 0] = 1
        
        # Set edges between virtual node and host's neighbors
        neighbors = np.where(physical_graph[self.vehicle_id, :])[0]
        for neighbor in neighbors:
            virtual_graph[0, neighbor + 1] = physical_graph[self.vehicle_id, neighbor]
            virtual_graph[neighbor + 1, 0] = physical_graph[neighbor, self.vehicle_id]
        
        self.virtual_graph = virtual_graph
        return virtual_graph
    
    @staticmethod
    def generate_fully_connected_graph(n: int) -> np.ndarray:
        """
        Generate a fully connected adjacency matrix (all vehicles communicate).
        
        For V2V/P2P platoon where all vehicles can communicate with each other.
        Diagonal is 0 (no self-loops).
        
        Args:
            n: Number of vehicles
            
        Returns:
            n x n adjacency matrix with 1s everywhere except diagonal
        """
        graph = np.ones((n, n)) - np.eye(n)
        return graph
    
    def get_neighbors_from_graph(self, vehicle_idx: int = None) -> np.ndarray:
        """
        Get direct neighbors from adjacency matrix.
        
        Args:
            vehicle_idx: Vehicle index (default: self.vehicle_id)
            
        Returns:
            Array of neighbor vehicle indices
        """
        if vehicle_idx is None:
            vehicle_idx = self.vehicle_id
        
        return np.where(self.graph[vehicle_idx, :])[0]
    
    def get_trusted_neighbors_from_graph(self, trust_scores: Dict[int, float]) -> np.ndarray:
        """
        Get trusted neighbors using graph topology and trust scores.
        
        This matches the old implementation logic:
        1. Find direct neighbors from adjacency matrix
        2. Filter by trust threshold
        
        Args:
            trust_scores: Dict of {vehicle_id: trust_score}
            
        Returns:
            Array of trusted neighbor vehicle indices
        """
        # Convert trust_scores dict to array
        trust_array = np.zeros(self.fleet_size)
        for vid, score in trust_scores.items():
            if 0 <= vid < self.fleet_size:
                trust_array[vid] = score
        
        # Find direct neighbors from graph
        neighbors = self.get_neighbors_from_graph()
        
        # Filter by trust threshold
        trusted_mask = trust_array[neighbors] > self.config.trust_threshold
        trusted_neighbors = neighbors[trusted_mask]
        
        return trusted_neighbors
    
    def calculate_weights_with_graph(self, trust_scores: Dict[int, float],
                                      weight_type: str = "local") -> WeightResult:
        """
        Calculate weights using graph topology (matches old implementation).
        
        This method uses the adjacency matrix more explicitly:
        1. Generate virtual graph for this vehicle
        2. Find trusted neighbors from graph topology
        3. Distribute weights based on trust and topology
        
        Args:
            trust_scores: Dict of {vehicle_id: trust_score}
            weight_type: "local" (prioritize local measurement) or "distributed" (equal)
            
        Returns:
            WeightResult with weight array and metadata
        """
        # Generate virtual graph
        virtual_graph = self.generate_virtual_graph()
        num_nodes = virtual_graph.shape[0]
        
        # Get trusted neighbors from graph
        trusted_neighbors = self.get_trusted_neighbors_from_graph(trust_scores)
        
        # Initialize weights
        new_weights = np.zeros(num_nodes)
        neighbor_weights = {}
        
        # Calculate base weight using old formula
        # n_w_i = max(kappa, |trusted neighbors| + 1 (self) + 1 (virtual))
        n_w_i = max(self.config.kappa, len(trusted_neighbors) + 2)
        base_weight = 1.0 / n_w_i
        
        # Set self weight
        new_weights[self.vehicle_id + 1] = base_weight
        
        # Set neighbor weights
        for neighbor_id in trusted_neighbors:
            new_weights[neighbor_id + 1] = base_weight
            neighbor_weights[int(neighbor_id)] = base_weight
        
        # Set virtual node weight based on weight_type
        if weight_type == "local":
            # Prioritize local measurement: remainder goes to virtual node
            new_weights[0] = 1.0 - (len(trusted_neighbors) + 1) * base_weight
        else:
            # Distributed equally
            new_weights[0] = base_weight
        
        # Ensure non-negative and normalize
        new_weights = np.maximum(new_weights, 0.0)
        weight_sum = np.sum(new_weights)
        if weight_sum > 0:
            new_weights = new_weights / weight_sum
        
        # Apply EMA smoothing if enabled
        if self.config.enable_smoothing and np.any(self.prev_weights > 0):
            # Ensure same size
            if len(self.prev_weights) == len(new_weights):
                eta = self.config.eta
                new_weights = eta * new_weights + (1 - eta) * self.prev_weights
                new_weights = new_weights / np.sum(new_weights)
        
        # Store for next iteration
        self.prev_weights = new_weights.copy()
        self.weights = new_weights
        trust_summary = self._summarize_trust_context(
            trust_scores=trust_scores,
            trusted_neighbors=list(trusted_neighbors),
            neighbor_weights=neighbor_weights,
        )

        # Build result
        result = WeightResult(
            weights=new_weights,
            trusted_neighbors=list(trusted_neighbors),
            neighbor_weights=neighbor_weights,
            w0=new_weights[0],
            w_self=new_weights[self.vehicle_id + 1],
            total_neighbor_weight=sum(neighbor_weights.values()),
            trust_source_scores=trust_summary["source_scores"],
            mean_source_trust=trust_summary["mean_source_trust"],
            mean_trusted_trust=trust_summary["mean_trusted_trust"],
            weighted_neighbor_trust=trust_summary["weighted_neighbor_trust"],
        )
        
        return result
    
    def set_graph(self, graph: np.ndarray):
        """
        Set/update the adjacency matrix.
        
        Args:
            graph: New adjacency matrix (n x n)
        """
        self.graph = np.array(graph)
        self.fleet_size = graph.shape[0]
        
        # Reset weights for new graph size
        self.weights = np.zeros(self.fleet_size + 1)
        self.weights[0] = self.config.w0_fixed
        if self.vehicle_id < self.fleet_size:
            self.weights[self.vehicle_id + 1] = self.config.w_self_base
        self.prev_weights = self.weights.copy()
        self.virtual_graph = None
    
    def get_weights_array(self) -> np.ndarray:
        """Get current weights as numpy array"""
        return self.weights.copy()
    
    def get_neighbor_weight(self, neighbor_id: int) -> float:
        """Get weight for specific neighbor"""
        if 0 <= neighbor_id < self.fleet_size:
            return self.weights[neighbor_id + 1]
        return 0.0
    
    def update_fleet_size(self, new_fleet_size: int):
        """
        Expand weight arrays for new fleet size
        
        Args:
            new_fleet_size: New fleet size
        """
        if new_fleet_size <= self.fleet_size:
            return
        
        # Expand arrays
        new_weights = np.zeros(new_fleet_size + 1)
        new_weights[:len(self.weights)] = self.weights
        
        new_prev = np.zeros(new_fleet_size + 1)
        new_prev[:len(self.prev_weights)] = self.prev_weights
        
        self.weights = new_weights
        self.prev_weights = new_prev
        self.fleet_size = new_fleet_size
        
        if self.logger:
            self.logger.logger.info(
                f"WeightTrustModule: Expanded to fleet size {new_fleet_size}"
            )
    
    def reset(self):
        """Reset weights to initial state"""
        self.weights = np.zeros(self.fleet_size + 1)
        self.weights[0] = self.config.w0_fixed
        self.weights[self.vehicle_id + 1] = self.config.w_self_base
        self.prev_weights = self.weights.copy()


class AdaptiveWeightCalculator:
    """
    Advanced weight calculator with additional features:
    - Distance-based weighting
    - Anomaly-aware weight reduction
    - Consensus convergence tracking
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int,
                 config: WeightConfig = None, logger=None):
        """Initialize adaptive weight calculator"""
        self.vehicle_id = vehicle_id
        self.fleet_size = fleet_size
        self.config = config or WeightConfig()
        self.logger = logger
        
        # Base weight module
        self.base_module = WeightTrustModule(vehicle_id, fleet_size, config, logger)
        
        # Distance cache (vehicle_id -> distance)
        self.distances: Dict[int, float] = {}
        
        # Anomaly scores (vehicle_id -> anomaly_level)
        self.anomaly_scores: Dict[int, float] = {}
        
        # Convergence tracking
        self.convergence_history: List[float] = []
        self.max_convergence_history = 20
    
    def calculate_adaptive_weights(self, trust_scores: Dict[int, float],
                                    distances: Optional[Dict[int, float]] = None,
                                    anomaly_scores: Optional[Dict[int, float]] = None) -> WeightResult:
        """
        Calculate weights with distance and anomaly awareness
        
        Args:
            trust_scores: Trust scores from TriP model
            distances: Optional distance to each vehicle
            anomaly_scores: Optional anomaly detection scores
            
        Returns:
            WeightResult with adaptive weights
        """
        # Update caches
        if distances:
            self.distances.update(distances)
        if anomaly_scores:
            self.anomaly_scores.update(anomaly_scores)
        
        # Adjust trust scores based on anomaly
        adjusted_trust = {}
        for vid, trust in trust_scores.items():
            anomaly = self.anomaly_scores.get(vid, 0.0)
            # Reduce trust for anomalous vehicles
            adjusted_trust[vid] = trust * (1.0 - anomaly * 0.5)
        
        # Apply distance weighting if enabled
        if self.config.use_distance_weighting and self.distances:
            for vid in adjusted_trust:
                if vid in self.distances:
                    dist = self.distances[vid]
                    # Closer vehicles get higher weight
                    # distance_factor = exp(-dist/scale)
                    distance_factor = np.exp(-dist / 5.0)  # 5m scale
                    adjusted_trust[vid] *= distance_factor
        
        # Calculate weights with adjusted trust
        result = self.base_module.calculate_weights(adjusted_trust)
        
        return result
    
    def track_convergence(self, fleet_states: np.ndarray, 
                          neighbor_estimates: Dict[int, np.ndarray]):
        """
        Track consensus convergence over time
        
        Args:
            fleet_states: Current fleet state estimates
            neighbor_estimates: Neighbor estimates for comparison
        """
        if not neighbor_estimates:
            return
        
        # Calculate disagreement metric
        disagreements = []
        for neighbor_id, neighbor_states in neighbor_estimates.items():
            diff = np.linalg.norm(fleet_states - neighbor_states)
            disagreements.append(diff)
        
        if disagreements:
            avg_disagreement = np.mean(disagreements)
            self.convergence_history.append(avg_disagreement)
            
            if len(self.convergence_history) > self.max_convergence_history:
                self.convergence_history = self.convergence_history[-self.max_convergence_history:]
    
    def get_convergence_trend(self) -> Optional[float]:
        """
        Get convergence trend (-1 to 1, negative = converging)
        """
        if len(self.convergence_history) < 3:
            return None
        
        # Simple linear trend
        x = np.arange(len(self.convergence_history))
        y = np.array(self.convergence_history)
        
        slope = np.polyfit(x, y, 1)[0]
        
        # Normalize to [-1, 1]
        max_slope = np.std(y) if np.std(y) > 0 else 1.0
        normalized = np.clip(slope / max_slope, -1.0, 1.0)
        
        return float(normalized)
