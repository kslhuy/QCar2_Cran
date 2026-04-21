"""
Trust and Weight Logger

Non-blocking I/O logger specifically for logging trust components,
final trust scores, and consensus weights for the TrustBasedFleetEstimator.
Plots can be visualized later. This file lives in TrustbasedDistributedObserver.
"""

import os
import csv
import math
import threading
import queue
from typing import Dict, Any, Iterable, List


class TrustWeightLogger:
    def __init__(self, output_dir: str = None, max_vehicles: int = 5):
        if output_dir is None:
            output_dir = os.path.dirname(os.path.abspath(__file__))
        self.output_dir = output_dir
        self.max_vehicles = max_vehicles
        self.recording = False

        self.queue = queue.Queue(maxsize=10000)
        self.thread = None
        self.file = None
        self.writer = None

        self.columns = self._build_columns(max_vehicles)

    @staticmethod
    def _nan() -> float:
        return float("nan")

    @staticmethod
    def _is_number(value: Any) -> bool:
        return isinstance(value, (int, float)) and not isinstance(value, bool)

    @classmethod
    def _to_float_or_nan(cls, value: Any) -> float:
        if cls._is_number(value):
            return float(value)
        return cls._nan()

    @classmethod
    def _normalize_vehicle_dict(cls, data: Any) -> Dict[int, Any]:
        if not isinstance(data, dict):
            return {}
        normalized: Dict[int, Any] = {}
        for key, value in data.items():
            try:
                vid = int(key)
            except (TypeError, ValueError):
                continue
            normalized[vid] = value
        return normalized

    @staticmethod
    def _to_csv_text(value: Any) -> str:
        if value is None:
            return ""
        if isinstance(value, (list, tuple, set)):
            return "|".join(str(v) for v in value)
        return str(value)

    @staticmethod
    def _nan_stats(values: Iterable[float]) -> Dict[str, float]:
        clean = [float(v) for v in values if isinstance(v, (int, float)) and not math.isnan(float(v))]
        if not clean:
            nan_val = float("nan")
            return {"mean": nan_val, "min": nan_val, "max": nan_val}
        return {
            "mean": float(sum(clean) / len(clean)),
            "min": float(min(clean)),
            "max": float(max(clean)),
        }

    def _build_columns(self, max_vehicles: int) -> List[str]:
        columns: List[str] = [
            "time",
            "w0",
            "w_self",
            "total_neighbor_weight",
            "trusted_neighbor_count",
            "active_vehicle_count",
            "mean_direct_trust",
            "mean_generalized_trust",
            "mean_gamma_host",
            "mean_gamma_local_peer",
            "mean_gamma_self",
            "mean_neighbor_weight",
            "weighted_neighbor_trust",
            "self_belief",
            "platoon_conf_mean",
            "platoon_conf_min",
            "platoon_conf_max",
            "prediction_mode_count",
            "rel_meas_used_global_count",
            "yolo_rel_meas_used_global_count",
            "is_turning",
            "host_steering",
            "v2v_attack_enabled",
            "v2v_attack_active",
            "v2v_attack_clock_s",
            "v2v_attack_scenario_count",
            "v2v_attack_active_count",
            "v2v_attack_types",
            "v2v_attack_names",
            "v2v_attack_data_types",
            "v2v_attack_start_s",
            "v2v_attack_end_s",
        ]

        for i in range(max_vehicles):
            columns.extend(
                [
                    f"vehicle_present_{i}",
                    f"trust_{i}",
                    f"gtrust_{i}",
                    f"local_trust_{i}",
                    f"global_trust_{i}",
                    f"gamma_host_{i}",
                    f"gamma_local_peer_{i}",
                    f"gamma_self_{i}",
                    f"rel_meas_used_global_{i}",
                    f"yolo_rel_meas_used_global_{i}",
                    f"rel_dist_meas_used_{i}",
                    f"rel_vel_meas_used_{i}",
                    f"y_local_distance_{i}",
                    f"y_true_distance_{i}",
                    f"yolo_true_rel_dist_error_{i}",
                    f"y_local_rel_velocity_{i}",
                    f"y_true_rel_velocity_{i}",
                    f"yolo_true_rel_vel_error_{i}",
                    f"yolo_rel_distance_{i}",
                    f"yolo_rel_velocity_{i}",
                    f"d_host_mean_{i}",
                    f"d_local_mean_{i}",
                    f"d_self_{i}",
                    f"mi_veh_id_{i}",
                    f"mi_dist_{i}",
                    f"mi_elem_idx_{i}",
                    f"mi_elem_val_{i}",
                    f"v_score_{i}",
                    f"d_score_{i}",
                    f"a_score_{i}",
                    f"h_score_{i}",
                    f"b_score_{i}",
                    f"q_factor_{i}",
                    f"w_neighbor_{i}",
                    f"est_conf_{i}",
                    f"pred_mode_{i}",
                    f"est_x_{i}",
                    f"est_y_{i}",
                    f"est_theta_{i}",
                    f"est_v_{i}",
                    f"est_a_{i}",
                    f"flag_attack_{i}",
                    f"flag_local_{i}",
                    f"flag_global_{i}",
                    f"inject_attack_active_{i}",
                    f"inject_attack_type_{i}",
                    f"inject_attack_name_{i}",
                    f"inject_attack_data_type_{i}",
                    f"inject_attack_modification_{i}",
                    f"inject_attack_fields_{i}",
                    f"inject_attack_start_{i}",
                    f"inject_attack_end_{i}",
                    f"inject_attack_attacker_{i}",
                ]
            )
            for k in range(max_vehicles):
                columns.extend(
                    [
                        f"g_dist_v{k}_{i}",
                        f"g_idx_v{k}_{i}",
                        f"g_val_v{k}_{i}",
                    ]
                )
        return columns

    def start(self, vehicle_id: int):
        if self.recording:
            return

        os.makedirs(self.output_dir, exist_ok=True)
        # Always overwrite the file for this vehicle
        filepath = os.path.join(self.output_dir, f"trust_weight_log_V{vehicle_id}.csv")

        try:
            self.file = open(filepath, "w", newline="", buffering=8192)
            self.writer = csv.DictWriter(self.file, fieldnames=self.columns)
            self.writer.writeheader()

            self.recording = True
            self.thread = threading.Thread(target=self._write_loop, daemon=True)
            self.thread.start()
        except Exception as e:
            print(f"[TrustLogger] Failed to open log file: {e}")
            self.recording = False

    def record(self, t: float, data: Dict[str, Any]):
        """
        Record a data sample without blocking.
        """
        if not self.recording:
            return

        # Flatten data for CSV
        nan_val = self._nan()
        neighbors = self._normalize_vehicle_dict(data.get("neighbors", {}))
        direct_trust = self._normalize_vehicle_dict(data.get("direct_trust", {}))
        generalized_trust = self._normalize_vehicle_dict(data.get("generalized_trust", {}))
        estimation_conf = self._normalize_vehicle_dict(data.get("estimation_confidence", {}))
        prediction_mode = self._normalize_vehicle_dict(data.get("prediction_mode", {}))
        fleet_estimates = self._normalize_vehicle_dict(data.get("fleet_estimates", {}))
        v2v_attack = data.get("v2v_attack", {})
        if not isinstance(v2v_attack, dict):
            v2v_attack = {}
        attack_by_vehicle = self._normalize_vehicle_dict(v2v_attack.get("by_vehicle", {}))

        row = {
            "time": float(t),
            "w0": self._to_float_or_nan(data.get("w0", 0.0)),
            "w_self": self._to_float_or_nan(data.get("w_self", 0.0)),
            "total_neighbor_weight": self._to_float_or_nan(
                data.get("total_neighbor_weight", nan_val)
            ),
            "trusted_neighbor_count": int(data.get("trusted_neighbor_count", 0) or 0),
            "active_vehicle_count": 0,
            "mean_direct_trust": nan_val,
            "mean_generalized_trust": nan_val,
            "mean_gamma_host": nan_val,
            "mean_gamma_local_peer": nan_val,
            "mean_gamma_self": nan_val,
            "mean_neighbor_weight": nan_val,
            "weighted_neighbor_trust": nan_val,
            "self_belief": self._to_float_or_nan(data.get("self_belief", nan_val)),
            "platoon_conf_mean": nan_val,
            "platoon_conf_min": nan_val,
            "platoon_conf_max": nan_val,
            "prediction_mode_count": 0,
            "rel_meas_used_global_count": 0,
            "yolo_rel_meas_used_global_count": 0,
            "is_turning": int(data.get("is_turning", 0)),
            "host_steering": self._to_float_or_nan(data.get("host_steering", nan_val)),
            "v2v_attack_enabled": int(bool(v2v_attack.get("enabled", False))),
            "v2v_attack_active": int(bool(v2v_attack.get("active", False))),
            "v2v_attack_clock_s": self._to_float_or_nan(
                v2v_attack.get("clock_s", nan_val)
            ),
            "v2v_attack_scenario_count": int(
                v2v_attack.get("scenario_count", 0) or 0
            ),
            "v2v_attack_active_count": int(
                v2v_attack.get("active_count", 0) or 0
            ),
            "v2v_attack_types": self._to_csv_text(v2v_attack.get("types")),
            "v2v_attack_names": self._to_csv_text(v2v_attack.get("names")),
            "v2v_attack_data_types": self._to_csv_text(
                v2v_attack.get("data_types")
            ),
            "v2v_attack_start_s": self._to_float_or_nan(
                v2v_attack.get("start_s", nan_val)
            ),
            "v2v_attack_end_s": self._to_float_or_nan(
                v2v_attack.get("end_s", nan_val)
            ),
        }

        active_vehicle_count = 0
        prediction_mode_count = 0
        direct_values: List[float] = []
        generalized_values: List[float] = []
        gamma_host_values: List[float] = []
        gamma_local_peer_values: List[float] = []
        gamma_self_values: List[float] = []
        neighbor_weights: List[float] = []
        conf_values: List[float] = []
        weighted_trust_num = 0.0
        weighted_trust_den = 0.0

        for i in range(self.max_vehicles):
            # Default/empty values use NaN so plots can easily ignore missing data.
            row[f"vehicle_present_{i}"] = 0
            row[f"trust_{i}"] = nan_val
            row[f"gtrust_{i}"] = nan_val
            row[f"local_trust_{i}"] = nan_val
            row[f"global_trust_{i}"] = nan_val
            row[f"gamma_host_{i}"] = nan_val
            row[f"gamma_local_peer_{i}"] = nan_val
            row[f"gamma_self_{i}"] = nan_val
            row[f"rel_meas_used_global_{i}"] = 0
            row[f"yolo_rel_meas_used_global_{i}"] = 0
            row[f"rel_dist_meas_used_{i}"] = 0
            row[f"rel_vel_meas_used_{i}"] = 0
            row[f"y_local_distance_{i}"] = nan_val
            row[f"y_true_distance_{i}"] = nan_val
            row[f"yolo_true_rel_dist_error_{i}"] = nan_val
            row[f"y_local_rel_velocity_{i}"] = nan_val
            row[f"y_true_rel_velocity_{i}"] = nan_val
            row[f"yolo_true_rel_vel_error_{i}"] = nan_val
            row[f"yolo_rel_distance_{i}"] = nan_val
            row[f"yolo_rel_velocity_{i}"] = nan_val
            row[f"d_host_mean_{i}"] = nan_val
            row[f"d_local_mean_{i}"] = nan_val
            row[f"d_self_{i}"] = nan_val
            row[f"mi_veh_id_{i}"] = nan_val
            row[f"mi_dist_{i}"] = nan_val
            row[f"mi_elem_idx_{i}"] = nan_val
            row[f"mi_elem_val_{i}"] = nan_val
            for k in range(self.max_vehicles):
                row[f"g_dist_v{k}_{i}"] = nan_val
                row[f"g_idx_v{k}_{i}"] = nan_val
                row[f"g_val_v{k}_{i}"] = nan_val
            row[f"v_score_{i}"] = nan_val
            row[f"d_score_{i}"] = nan_val
            row[f"a_score_{i}"] = nan_val
            row[f"h_score_{i}"] = nan_val
            row[f"b_score_{i}"] = nan_val
            row[f"q_factor_{i}"] = nan_val
            row[f"w_neighbor_{i}"] = nan_val
            row[f"est_conf_{i}"] = nan_val
            row[f"pred_mode_{i}"] = nan_val
            row[f"est_x_{i}"] = nan_val
            row[f"est_y_{i}"] = nan_val
            row[f"est_theta_{i}"] = nan_val
            row[f"est_v_{i}"] = nan_val
            row[f"est_a_{i}"] = nan_val
            row[f"flag_attack_{i}"] = 0
            row[f"flag_local_{i}"] = 0
            row[f"flag_global_{i}"] = 0
            row[f"inject_attack_active_{i}"] = 0
            row[f"inject_attack_type_{i}"] = ""
            row[f"inject_attack_name_{i}"] = ""
            row[f"inject_attack_data_type_{i}"] = ""
            row[f"inject_attack_modification_{i}"] = ""
            row[f"inject_attack_fields_{i}"] = ""
            row[f"inject_attack_start_{i}"] = nan_val
            row[f"inject_attack_end_{i}"] = nan_val
            row[f"inject_attack_attacker_{i}"] = nan_val

            present = False

            if i in attack_by_vehicle:
                attack_data = attack_by_vehicle[i]
                if isinstance(attack_data, dict):
                    row[f"inject_attack_active_{i}"] = int(
                        bool(attack_data.get("active", False))
                    )
                    row[f"inject_attack_type_{i}"] = self._to_csv_text(
                        attack_data.get("types")
                    )
                    row[f"inject_attack_name_{i}"] = self._to_csv_text(
                        attack_data.get("names")
                    )
                    row[f"inject_attack_data_type_{i}"] = self._to_csv_text(
                        attack_data.get("data_types")
                    )
                    row[f"inject_attack_modification_{i}"] = self._to_csv_text(
                        attack_data.get("modifications")
                    )
                    row[f"inject_attack_fields_{i}"] = self._to_csv_text(
                        attack_data.get("fields")
                    )
                    row[f"inject_attack_start_{i}"] = self._to_float_or_nan(
                        attack_data.get("start_s", nan_val)
                    )
                    row[f"inject_attack_end_{i}"] = self._to_float_or_nan(
                        attack_data.get("end_s", nan_val)
                    )
                    row[f"inject_attack_attacker_{i}"] = self._to_float_or_nan(
                        attack_data.get("attacker_id", nan_val)
                    )

            if i in direct_trust:
                trust_val = self._to_float_or_nan(direct_trust[i])
                row[f"trust_{i}"] = trust_val
                if not math.isnan(trust_val):
                    direct_values.append(trust_val)
                    present = True

            if i in generalized_trust:
                gtrust_val = self._to_float_or_nan(generalized_trust[i])
                row[f"gtrust_{i}"] = gtrust_val
                if not math.isnan(gtrust_val):
                    generalized_values.append(gtrust_val)
                    present = True

            if i in neighbors:
                ndata = neighbors[i]

                if math.isnan(row[f"trust_{i}"]):
                    trust_val = self._to_float_or_nan(ndata.get("trust_score", nan_val))
                    row[f"trust_{i}"] = trust_val
                    if not math.isnan(trust_val):
                        direct_values.append(trust_val)
                        present = True

                row[f"local_trust_{i}"] = self._to_float_or_nan(
                    ndata.get("local_trust", nan_val)
                )
                row[f"global_trust_{i}"] = self._to_float_or_nan(
                    ndata.get("global_trust", nan_val)
                )
                row[f"gamma_host_{i}"] = self._to_float_or_nan(
                    ndata.get("gamma_host", nan_val)
                )
                row[f"gamma_local_peer_{i}"] = self._to_float_or_nan(
                    ndata.get("gamma_local_peer", nan_val)
                )
                row[f"gamma_self_{i}"] = self._to_float_or_nan(
                    ndata.get("gamma_self", nan_val)
                )
                row[f"rel_meas_used_global_{i}"] = (
                    1 if bool(ndata.get("rel_meas_used_global", False)) else 0
                )
                row[f"yolo_rel_meas_used_global_{i}"] = (
                    1 if bool(ndata.get("yolo_rel_meas_used_global", False)) else 0
                )
                row[f"rel_dist_meas_used_{i}"] = (
                    1 if bool(ndata.get("rel_dist_meas_used", False)) else 0
                )
                row[f"rel_vel_meas_used_{i}"] = (
                    1 if bool(ndata.get("rel_vel_meas_used", False)) else 0
                )
                row[f"y_local_distance_{i}"] = self._to_float_or_nan(
                    ndata.get("y_local_distance", nan_val)
                )
                row[f"y_true_distance_{i}"] = self._to_float_or_nan(
                    ndata.get("y_true_distance", nan_val)
                )
                row[f"yolo_true_rel_dist_error_{i}"] = self._to_float_or_nan(
                    ndata.get("yolo_true_rel_dist_error", nan_val)
                )
                row[f"y_local_rel_velocity_{i}"] = self._to_float_or_nan(
                    ndata.get("y_local_rel_velocity", nan_val)
                )
                row[f"y_true_rel_velocity_{i}"] = self._to_float_or_nan(
                    ndata.get("y_true_rel_velocity", nan_val)
                )
                row[f"yolo_true_rel_vel_error_{i}"] = self._to_float_or_nan(
                    ndata.get("yolo_true_rel_vel_error", nan_val)
                )
                row[f"yolo_rel_distance_{i}"] = self._to_float_or_nan(
                    ndata.get("yolo_rel_distance", nan_val)
                )
                row[f"yolo_rel_velocity_{i}"] = self._to_float_or_nan(
                    ndata.get("yolo_rel_velocity", nan_val)
                )
                row[f"d_host_mean_{i}"] = self._to_float_or_nan(
                    ndata.get("d_host_mean", nan_val)
                )
                row[f"d_local_mean_{i}"] = self._to_float_or_nan(
                    ndata.get("d_local_mean", nan_val)
                )
                row[f"d_self_{i}"] = self._to_float_or_nan(
                    ndata.get("d_self", nan_val)
                )
                row[f"mi_veh_id_{i}"] = self._to_float_or_nan(
                    ndata.get("mi_veh_id", nan_val)
                )
                row[f"mi_dist_{i}"] = self._to_float_or_nan(
                    ndata.get("mi_dist", nan_val)
                )
                row[f"mi_elem_idx_{i}"] = self._to_float_or_nan(
                    ndata.get("mi_elem_idx", nan_val)
                )
                row[f"mi_elem_val_{i}"] = self._to_float_or_nan(
                    ndata.get("mi_elem_val", nan_val)
                )

                v2v = ndata.get("v2v_details", {})
                for k in range(self.max_vehicles):
                    if k in v2v:
                        kdata = v2v[k]
                        row[f"g_dist_v{k}_{i}"] = self._to_float_or_nan(kdata.get("dist", nan_val))
                        row[f"g_idx_v{k}_{i}"] = self._to_float_or_nan(kdata.get("idx", nan_val))
                        row[f"g_val_v{k}_{i}"] = self._to_float_or_nan(kdata.get("val", nan_val))

                if not math.isnan(row[f"gamma_host_{i}"]):
                    gamma_host_values.append(row[f"gamma_host_{i}"])
                if not math.isnan(row[f"gamma_local_peer_{i}"]):
                    gamma_local_peer_values.append(row[f"gamma_local_peer_{i}"])
                if not math.isnan(row[f"gamma_self_{i}"]):
                    gamma_self_values.append(row[f"gamma_self_{i}"])
                row[f"v_score_{i}"] = self._to_float_or_nan(
                    ndata.get("velocity_score", nan_val)
                )
                row[f"d_score_{i}"] = self._to_float_or_nan(
                    ndata.get("distance_score", nan_val)
                )
                row[f"a_score_{i}"] = self._to_float_or_nan(
                    ndata.get("acceleration_score", nan_val)
                )
                row[f"h_score_{i}"] = self._to_float_or_nan(
                    ndata.get("heading_score", nan_val)
                )
                row[f"b_score_{i}"] = self._to_float_or_nan(
                    ndata.get("beacon_score", nan_val)
                )
                row[f"q_factor_{i}"] = self._to_float_or_nan(
                    ndata.get("quality_factor", nan_val)
                )
                w_neighbor = self._to_float_or_nan(ndata.get("w_neighbor", nan_val))
                row[f"w_neighbor_{i}"] = w_neighbor
                if not math.isnan(w_neighbor):
                    neighbor_weights.append(w_neighbor)
                    trust_for_weight = row[f"trust_{i}"]
                    if not math.isnan(trust_for_weight):
                        weighted_trust_num += trust_for_weight * w_neighbor
                        weighted_trust_den += w_neighbor
                    present = True

                row[f"flag_attack_{i}"] = (
                    1 if ndata.get("flag_target_attack", False) else 0
                )
                row[f"flag_local_{i}"] = (
                    1 if ndata.get("flag_local_est_check", False) else 0
                )
                row[f"flag_global_{i}"] = (
                    1 if ndata.get("flag_global_est_check", False) else 0
                )
                row["rel_meas_used_global_count"] += int(row[f"rel_meas_used_global_{i}"])
                row["yolo_rel_meas_used_global_count"] += int(
                    row[f"yolo_rel_meas_used_global_{i}"]
                )

            if i in estimation_conf:
                conf_val = self._to_float_or_nan(estimation_conf[i])
                row[f"est_conf_{i}"] = conf_val
                if not math.isnan(conf_val):
                    conf_values.append(conf_val)
                    present = True

            if i in prediction_mode:
                pred_flag = 1 if bool(prediction_mode[i]) else 0
                row[f"pred_mode_{i}"] = pred_flag
                prediction_mode_count += pred_flag
                present = True

            if i in fleet_estimates:
                est = fleet_estimates[i]
                if isinstance(est, dict):
                    row[f"est_x_{i}"] = self._to_float_or_nan(est.get("x", nan_val))
                    row[f"est_y_{i}"] = self._to_float_or_nan(est.get("y", nan_val))
                    row[f"est_theta_{i}"] = self._to_float_or_nan(
                        est.get("theta", nan_val)
                    )
                    row[f"est_v_{i}"] = self._to_float_or_nan(
                        est.get("velocity", nan_val)
                    )
                    row[f"est_a_{i}"] = self._to_float_or_nan(
                        est.get("acceleration", nan_val)
                    )
                    present = True
                elif isinstance(est, (list, tuple)):
                    if len(est) > 0:
                        row[f"est_x_{i}"] = self._to_float_or_nan(est[0])
                    if len(est) > 1:
                        row[f"est_y_{i}"] = self._to_float_or_nan(est[1])
                    if len(est) > 2:
                        row[f"est_theta_{i}"] = self._to_float_or_nan(est[2])
                    if len(est) > 3:
                        row[f"est_v_{i}"] = self._to_float_or_nan(est[3])
                    if len(est) > 4:
                        row[f"est_a_{i}"] = self._to_float_or_nan(est[4])
                    present = True

            if present:
                row[f"vehicle_present_{i}"] = 1
                active_vehicle_count += 1

        row["active_vehicle_count"] = active_vehicle_count
        row["prediction_mode_count"] = int(
            data.get("prediction_mode_count", prediction_mode_count)
        )

        direct_stats = self._nan_stats(direct_values)
        generalized_stats = self._nan_stats(generalized_values)
        gamma_host_stats = self._nan_stats(gamma_host_values)
        gamma_local_peer_stats = self._nan_stats(gamma_local_peer_values)
        gamma_self_stats = self._nan_stats(gamma_self_values)
        neighbor_weight_stats = self._nan_stats(neighbor_weights)
        conf_stats = self._nan_stats(conf_values)

        row["mean_direct_trust"] = direct_stats["mean"]
        row["mean_generalized_trust"] = generalized_stats["mean"]
        row["mean_gamma_host"] = gamma_host_stats["mean"]
        row["mean_gamma_local_peer"] = gamma_local_peer_stats["mean"]
        row["mean_gamma_self"] = gamma_self_stats["mean"]
        row["mean_neighbor_weight"] = neighbor_weight_stats["mean"]
        row["platoon_conf_mean"] = conf_stats["mean"]
        row["platoon_conf_min"] = conf_stats["min"]
        row["platoon_conf_max"] = conf_stats["max"]

        if math.isnan(row["total_neighbor_weight"]):
            row["total_neighbor_weight"] = float(sum(neighbor_weights))
        if row["trusted_neighbor_count"] <= 0:
            row["trusted_neighbor_count"] = int(sum(1 for w in neighbor_weights if w > 0.0))
        if weighted_trust_den > 0:
            row["weighted_neighbor_trust"] = float(weighted_trust_num / weighted_trust_den)

        try:
            self.queue.put_nowait(row)
        except queue.Full:
            pass  # Drop if queue is full

    def _write_loop(self):
        writes_since_flush = 0
        while self.recording or not self.queue.empty():
            try:
                row = self.queue.get(timeout=0.1)
                self.writer.writerow(row)
                self.queue.task_done()
                
                # Flush occasionally to avoid leaving large chunks of unwritten data
                # that turn into null bytes upon unexpected process termination
                writes_since_flush += 1
                if writes_since_flush >= 50 and self.file:
                    self.file.flush()
                    writes_since_flush = 0
            except queue.Empty:
                continue
            except Exception as e:
                pass

    def stop(self):
        self.recording = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.file:
            try:
                self.file.flush()
                self.file.close()
            except Exception:
                pass
