"""
Run multiple fleet observers side by side on the same vehicle.

The outer VehicleObserver still sees one FleetStateEstimatorBase instance. This
wrapper fans out identical V2V data and update inputs to each child estimator,
then returns the configured primary estimator's fleet_states for controller use.
"""
import os
import re
from datetime import datetime
from typing import Dict

import numpy as np

from ..fleet_state_estimators import FleetStateEstimatorBase
from .classical_luenberger_observer import (
    ClassicalLuenbergerObserverEstimator,
    HighGainLuenbergerObserverEstimator,
)
from .classical_ekf_observer import ClassicalEKFObserverEstimator
from .leadering_observer import LeaderingObserverEstimator


class ParallelObserverEstimator(FleetStateEstimatorBase):
    """Wrapper that updates several observer estimators in the same experiment."""

    ESTIMATOR_CLASSES = {
        "leadering_observer": LeaderingObserverEstimator,
        "classical_luenberger_observer": ClassicalLuenbergerObserverEstimator,
        "classical_luenberge_observer": ClassicalLuenbergerObserverEstimator,
        "high_gain_luenberger_observer": HighGainLuenbergerObserverEstimator,
        "classical_ekf_observer": ClassicalEKFObserverEstimator,
    }

    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5,
                 config: Dict = None, logger=None):
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)

        self.primary_estimator = self.config.get("primary_estimator", "leadering_observer")
        self.experiment_output_dir = self._resolve_experiment_output_dir()
        observer_configs = self.config.get("observers") or self.config.get("parallel_observers")
        if observer_configs is None:
            observer_configs = {
                "leadering_observer": {"type": "leadering_observer"},
                "classical_luenberger_observer": {"type": "classical_luenberger_observer"},
            }

        self.estimators = {}
        self.results = {}
        for name, child_cfg in observer_configs.items():
            child_cfg = dict(child_cfg or {})
            estimator_type = child_cfg.pop("type", name)
            estimator_class = self.ESTIMATOR_CLASSES.get(estimator_type)
            if estimator_class is None:
                raise ValueError(
                    f"ParallelObserverEstimator unknown child type: {estimator_type}. "
                    f"Available: {list(self.ESTIMATOR_CLASSES.keys())}"
                )

            merged_child_cfg = self._child_config(name, child_cfg)
            self.estimators[name] = estimator_class(
                vehicle_id=vehicle_id,
                fleet_size=fleet_size,
                state_dim=state_dim,
                config=merged_child_cfg,
                logger=logger,
            )

        if self.primary_estimator not in self.estimators:
            self.primary_estimator = next(iter(self.estimators))

        self.debug_data = {}

    def _child_config(self, name: str, child_cfg: Dict) -> Dict:
        shared_cfg = dict(self.config)
        for key in ("observers", "parallel_observers", "primary_estimator"):
            shared_cfg.pop(key, None)
        shared_cfg.update(child_cfg)
        shared_cfg.setdefault("recorder_prefix", name)
        shared_cfg["debug_output_dir"] = self.experiment_output_dir
        return shared_cfg

    def _resolve_experiment_output_dir(self) -> str:
        base_dir = self.config.get("debug_output_dir", "observer_recordings")
        if not bool(self.config.get("recording_trial_subdir", True)):
            os.makedirs(base_dir, exist_ok=True)
            return base_dir

        explicit_dir = self.config.get("experiment_output_dir")
        if explicit_dir:
            os.makedirs(explicit_dir, exist_ok=True)
            return explicit_dir

        experiment_name = self.config.get("experiment_name")
        if not experiment_name:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            experiment_name = f"trial_v{self.vehicle_id}_parallel_observers_{timestamp}"
        experiment_name = self._sanitize_path_name(str(experiment_name))
        output_dir = os.path.join(base_dir, experiment_name)
        os.makedirs(output_dir, exist_ok=True)
        return output_dir

    @staticmethod
    def _sanitize_path_name(name: str) -> str:
        name = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip())
        return name.strip("._") or "trial"

    def update(self, local_state: np.ndarray, dt: float,
               current_time_ns: int, control: np.ndarray) -> np.ndarray:
        for name, estimator in self.estimators.items():
            self.results[name] = estimator.update(
                local_state=local_state,
                dt=dt,
                current_time_ns=current_time_ns,
                control=control,
            )

        primary_result = self.results.get(self.primary_estimator)
        if primary_result is None:
            return self.fleet_states.copy()

        self.fleet_states = primary_result.copy()
        self.debug_data = {
            name: estimator.get_debug_data()
            for name, estimator in self.estimators.items()
            if hasattr(estimator, "get_debug_data")
        }
        self._cleanup_old_data(current_time_ns)
        return self.fleet_states.copy()

    def add_received_local_state(self, sender_id: int, state: Dict, timestamp_ns: int) -> bool:
        success = super().add_received_local_state(sender_id, state, timestamp_ns)
        for estimator in self.estimators.values():
            success = estimator.add_received_local_state(sender_id, state, timestamp_ns) or success
        return success

    def add_received_fleet_state(self, sender_id: int, fleet_estimates: Dict, timestamp_ns: int) -> bool:
        success = super().add_received_fleet_state(sender_id, fleet_estimates, timestamp_ns)
        for estimator in self.estimators.values():
            success = estimator.add_received_fleet_state(sender_id, fleet_estimates, timestamp_ns) or success
        return success

    def add_received_observer_state(self, sender_id: int, observer_state: np.ndarray, timestamp_ns: int) -> bool:
        success = super().add_received_observer_state(sender_id, observer_state, timestamp_ns)
        for estimator in self.estimators.values():
            if hasattr(estimator, "add_received_observer_state"):
                success = estimator.add_received_observer_state(sender_id, observer_state, timestamp_ns) or success
        return success

    def get_debug_data(self) -> Dict:
        return self.debug_data

    def reset(self):
        super().reset()
        for estimator in self.estimators.values():
            estimator.reset()

    def stop_recording(self):
        stats = {}
        for name, estimator in self.estimators.items():
            if hasattr(estimator, "stop_recording"):
                stats[name] = estimator.stop_recording()
        return stats
