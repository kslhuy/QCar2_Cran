"""
MATLAB-style mean attack batch experiment for the Python trust system.

This runner is intentionally simulation-only. It exercises the current
TrustBasedFleetEstimator with five synthetic vehicles in one process, applies
the six MATLAB Mix_test-style attacks, and writes summary metrics/plots.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml


QCAR_ROOT = Path(__file__).resolve().parents[1]
if str(QCAR_ROOT) not in sys.path:
    sys.path.insert(0, str(QCAR_ROOT))

from Observer.TrustbasedDistributedObserver import (  # noqa: E402
    trust_based_fleet_estimator as trust_estimator_module,
)
from Observer.TrustbasedDistributedObserver.trust_logger import TrustWeightLogger  # noqa: E402

TrustBasedFleetEstimator = trust_estimator_module.TrustBasedFleetEstimator


STATE_FIELDS = ("x", "y", "theta", "velocity", "acceleration")


@dataclass(frozen=True)
class AttackCase:
    case_number: int
    label: str
    fields: Tuple[str, ...]
    mode: str
    intensity: float
    probability: float = 1.0


MIX_TEST_CASES: Tuple[AttackCase, ...] = (
    AttackCase(1, "P Bias -5m", ("x",), "bias", -5.0),
    AttackCase(2, "P Faulty 10m", ("x",), "faulty", 10.0, probability=0.3),
    AttackCase(3, "V Bias -2m/s", ("velocity",), "bias", -2.0),
    AttackCase(4, "V Faulty 2.5m/s", ("velocity",), "faulty", 2.5, probability=0.3),
    AttackCase(
        5,
        "A Faulty 1.0m/s^2",
        ("acceleration",),
        "faulty",
        1.0,
        probability=0.3,
    ),
    AttackCase(6, "DoS Attack", tuple(), "drop", 0.0),
)


class NullLogger:
    """Small logger shim for estimator code paths that expect VehicleLogger."""

    def __init__(self) -> None:
        self.logger = self

    def debug(self, *args: Any, **kwargs: Any) -> None:
        pass

    def info(self, *args: Any, **kwargs: Any) -> None:
        pass

    def warning(self, *args: Any, **kwargs: Any) -> None:
        pass

    def error(self, *args: Any, **kwargs: Any) -> None:
        pass

    def log_warning(self, *args: Any, **kwargs: Any) -> None:
        pass

    def log_error(self, *args: Any, **kwargs: Any) -> None:
        pass

    def log_trust_weight(self, *args: Any, **kwargs: Any) -> None:
        pass


def wrap_angle(angle: float) -> float:
    return float(math.atan2(math.sin(angle), math.cos(angle)))


def state_to_dict(vehicle_id: int, state: np.ndarray) -> Dict[str, Any]:
    return {
        "vehicle_id": int(vehicle_id),
        "x": float(state[0]),
        "y": float(state[1]),
        "theta": float(state[2]),
        "velocity": float(state[3]),
        "acceleration": float(state[4]),
        "control_input": {
            "steering": 0.0,
            "throttle": float(state[4]),
        },
        "gps_valid": True,
        "source": "batch_simulation",
    }


def fleet_to_dict(fleet_states: np.ndarray, fleet_size: int) -> Dict[int, Dict[str, float]]:
    data: Dict[int, Dict[str, float]] = {}
    for vehicle_id in range(fleet_size):
        state = fleet_states[:, vehicle_id]
        data[vehicle_id] = {
            "x": float(state[0]),
            "y": float(state[1]),
            "theta": float(state[2]),
            "velocity": float(state[3]),
            "acceleration": float(state[4]) if state.shape[0] > 4 else 0.0,
            "confidence": 1.0,
        }
    return data


def load_trust_config(prediction_mode: str) -> Dict[str, Any]:
    """Load the trust child YAML and flatten it like VehicleObserver does."""
    config_path = (
        QCAR_ROOT
        / "Observer"
        / "TrustbasedDistributedObserver"
        / "config_trust_estimator.yaml"
    )
    with config_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    observer_cfg = dict(raw.get("observer", {}) or {})
    observer_cfg.pop("kalman", None)

    config: Dict[str, Any] = observer_cfg
    config["trust"] = dict(raw.get("trust", {}) or {})
    config["weight"] = dict(raw.get("weight", {}) or {})
    config["vehicle"] = dict(raw.get("vehicle", {}) or {})
    config["vehicle_models"] = dict(raw.get("vehicle_models", {}) or {})
    config["timestamp_alignment"] = dict(raw.get("timestamp_alignment", {}) or {})
    config["logging"] = dict(raw.get("logging", {}) or {})

    # Keep the batch experiment honest: do not use clean V2V as the prediction
    # source unless explicitly requested.
    config["dynamics_prediction_mode"] = prediction_mode
    config["vehicle"]["dynamics_prediction_mode"] = prediction_mode
    config["vehicle"]["longitudinal_model"] = "simple_acceleration"
    config["vehicle"]["max_velocity"] = max(float(config["vehicle"].get("max_velocity", 2.0)), 3.0)
    config["vehicle"]["max_acceleration"] = max(
        float(config["vehicle"].get("max_acceleration", 2.0)), 2.0
    )
    return config


def make_initial_states(num_vehicles: int) -> np.ndarray:
    """Return [vehicle, state] states similar to MATLAB platoon spacing."""
    states = np.zeros((num_vehicles, 5), dtype=float)
    initial_x = np.linspace(8.0, 8.0 - 2.0 * (num_vehicles - 1), num_vehicles)
    initial_v = np.array([0.9] + [1.0] * (num_vehicles - 1), dtype=float)
    states[:, 0] = initial_x
    states[:, 1] = 0.5
    states[:, 2] = 0.0
    states[:, 3] = initial_v
    states[:, 4] = 0.0
    return states


def acceleration_command(t: float, vehicle_id: int) -> float:
    """Small shared longitudinal excitation so velocity/acceleration checks matter."""
    phase = 0.15 * vehicle_id
    if 3.0 <= t <= 18.0:
        return float(0.12 * math.sin(0.55 * t + phase))
    return 0.0


def step_true_dynamics(states: np.ndarray, t: float, dt: float) -> None:
    for vehicle_id in range(states.shape[0]):
        a = acceleration_command(t, vehicle_id)
        states[vehicle_id, 4] = a
        states[vehicle_id, 3] = float(np.clip(states[vehicle_id, 3] + a * dt, 0.0, 2.0))
        states[vehicle_id, 0] += states[vehicle_id, 3] * math.cos(states[vehicle_id, 2]) * dt
        states[vehicle_id, 1] += states[vehicle_id, 3] * math.sin(states[vehicle_id, 2]) * dt


def apply_attack_to_state_dict(
    payload: Dict[str, Any],
    attack_case: AttackCase,
    rng: np.random.Generator,
) -> Optional[Dict[str, Any]]:
    if attack_case.mode == "drop":
        return None

    modified = dict(payload)
    modified["control_input"] = dict(payload.get("control_input", {}) or {})

    for field in attack_case.fields:
        if field not in modified:
            continue
        if attack_case.mode == "faulty" and rng.random() > attack_case.probability:
            continue
        original = float(modified[field])
        if attack_case.mode == "bias":
            modified[field] = original + attack_case.intensity
        elif attack_case.mode == "faulty":
            modified[field] = original + float(rng.normal(0.0, attack_case.intensity))

    if "acceleration" in attack_case.fields:
        modified["control_input"]["throttle"] = float(modified.get("acceleration", 0.0))
    return modified


def apply_attack_to_fleet_dict(
    payload: Dict[int, Dict[str, float]],
    attack_case: AttackCase,
    rng: np.random.Generator,
    fleet_target: str,
    attacker_id: int,
) -> Optional[Dict[int, Dict[str, float]]]:
    if attack_case.mode == "drop":
        return None

    modified = {int(vid): dict(state) for vid, state in payload.items()}
    target_ids: Iterable[int]
    if fleet_target == "attacker":
        target_ids = (attacker_id,)
    else:
        target_ids = tuple(modified.keys())

    for target_id in target_ids:
        if target_id not in modified:
            continue
        for field in attack_case.fields:
            if field not in modified[target_id]:
                continue
            if attack_case.mode == "faulty" and rng.random() > attack_case.probability:
                continue
            original = float(modified[target_id][field])
            if attack_case.mode == "bias":
                modified[target_id][field] = original + attack_case.intensity
            elif attack_case.mode == "faulty":
                modified[target_id][field] = original + float(
                    rng.normal(0.0, attack_case.intensity)
                )
    return modified


def build_attack_status(
    attack_case: AttackCase,
    t: float,
    t_start: float,
    t_end: float,
    attacker_id: int,
    data_type: str,
    active: bool,
) -> Dict[str, Any]:
    scenario = {
        "name": attack_case.label,
        "type": "Mix_test",
        "data_type": data_type,
        "modification": attack_case.mode,
        "target_fields": list(attack_case.fields),
        "t_start": float(t_start),
        "t_end": float(t_end),
        "attacker_id": int(attacker_id),
        "victim_ids": [-1],
    }
    return {
        "enabled": True,
        "attack_active": bool(active),
        "elapsed_time": float(t),
        "scenario_count": 1,
        "active_count": 1 if active else 0,
        "all_scenario_details": [scenario],
        "active_scenario_details": [scenario] if active else [],
    }


def create_estimators(
    num_vehicles: int,
    config: Dict[str, Any],
    initial_states: np.ndarray,
    trust_log_dir: Path,
) -> List[TrustBasedFleetEstimator]:
    estimators: List[TrustBasedFleetEstimator] = []
    logger = NullLogger()
    trust_log_dir.mkdir(parents=True, exist_ok=True)

    original_logger_cls = trust_estimator_module.TrustWeightLogger

    class BatchTrustWeightLogger(TrustWeightLogger):
        def __init__(self, output_dir: str = None, max_vehicles: int = 5):
            super().__init__(output_dir=str(trust_log_dir), max_vehicles=max_vehicles)

    trust_estimator_module.TrustWeightLogger = BatchTrustWeightLogger
    try:
        for vehicle_id in range(num_vehicles):
            estimator = TrustBasedFleetEstimator(
                vehicle_id=vehicle_id,
                fleet_size=num_vehicles,
                state_dim=5,
                config=config,
                logger=logger,
            )
            estimator.fleet_states[:, :] = initial_states.T
            estimators.append(estimator)
    finally:
        trust_estimator_module.TrustWeightLogger = original_logger_cls
    return estimators


def stop_estimators(estimators: Sequence[TrustBasedFleetEstimator]) -> None:
    for estimator in estimators:
        if hasattr(estimator, "trust_weight_logger"):
            estimator.trust_weight_logger.stop()


def compute_target_rmse(
    estimates: np.ndarray,
    truth: np.ndarray,
    observers: Sequence[int],
) -> np.ndarray:
    """Return [target, metric] average per-observer RMSE."""
    num_targets = truth.shape[1]
    rmse = np.full((num_targets, 4), np.nan, dtype=float)
    for target_id in range(num_targets):
        observer_values: List[np.ndarray] = []
        for host_id in observers:
            err = estimates[:, host_id, :, target_id] - truth[:, target_id, :]
            distance = np.hypot(err[:, 0], err[:, 1])
            theta = np.array([wrap_angle(v) for v in err[:, 2]], dtype=float)
            metrics = np.array(
                [
                    math.sqrt(float(np.mean(distance**2))),
                    math.sqrt(float(np.mean(theta**2))),
                    math.sqrt(float(np.mean(err[:, 3] ** 2))),
                    math.sqrt(float(np.mean(err[:, 4] ** 2))),
                ],
                dtype=float,
            )
            observer_values.append(metrics)
        if observer_values:
            rmse[target_id, :] = np.mean(np.vstack(observer_values), axis=0)
    return rmse


def trust_metrics(
    trust_trace: np.ndarray,
    times: np.ndarray,
    t_start: float,
    t_end: float,
    threshold: float,
) -> Tuple[float, float, float]:
    valid = np.isfinite(trust_trace)
    mean_trust = float(np.nanmean(trust_trace)) if np.any(valid) else float("nan")

    pre_mask = times < t_start
    during_mask = (times >= t_start) & (times <= t_end)
    pre_mean = float(np.nanmean(trust_trace[pre_mask])) if np.any(pre_mask) else float("nan")
    during_mean = (
        float(np.nanmean(trust_trace[during_mask])) if np.any(during_mask) else float("nan")
    )
    degradation = pre_mean - during_mean if np.isfinite(pre_mean) and np.isfinite(during_mean) else float("nan")

    detection_time = float("nan")
    attack_indices = np.where(during_mask & (trust_trace < threshold))[0]
    if attack_indices.size > 0:
        detection_time = float(times[int(attack_indices[0])])
    return mean_trust, degradation, detection_time


def run_case(
    attack_case: AttackCase,
    args: argparse.Namespace,
    config: Dict[str, Any],
    output_dir: Path,
) -> List[Dict[str, Any]]:
    rng = np.random.default_rng(args.seed + attack_case.case_number)
    states = make_initial_states(args.num_vehicles)
    case_log_dir = output_dir / f"case_{attack_case.case_number:02d}_trust_logs"
    estimators = create_estimators(args.num_vehicles, config, states, case_log_dir)
    non_attacker_ids = [vid for vid in range(args.num_vehicles) if vid != args.attacker_id]

    times: List[float] = []
    estimates_over_time: List[np.ndarray] = []
    truth_over_time: List[np.ndarray] = []
    trust_by_host: Dict[int, List[float]] = {vid: [] for vid in non_attacker_ids}

    num_steps = int(round(args.simulation_time / args.dt))

    try:
        for step in range(num_steps):
            t = step * args.dt
            current_time_ns = int(round(t * 1e9))
            active = args.t_start <= t <= args.t_end

            clean_local = {
                vid: state_to_dict(vid, states[vid]) for vid in range(args.num_vehicles)
            }
            clean_fleet = {
                vid: fleet_to_dict(estimators[vid].fleet_states, args.num_vehicles)
                for vid in range(args.num_vehicles)
            }

            local_payloads: Dict[int, Optional[Dict[str, Any]]] = {}
            fleet_payloads: Dict[int, Optional[Dict[int, Dict[str, float]]]] = {}

            for sender_id in range(args.num_vehicles):
                local_payloads[sender_id] = dict(clean_local[sender_id])
                fleet_payloads[sender_id] = {
                    int(k): dict(v) for k, v in clean_fleet[sender_id].items()
                }

                if active and sender_id == args.attacker_id:
                    if args.data_type in ("local", "both"):
                        local_payloads[sender_id] = apply_attack_to_state_dict(
                            local_payloads[sender_id],
                            attack_case,
                            rng,
                        )
                    if args.data_type in ("fleet", "both"):
                        fleet_payloads[sender_id] = apply_attack_to_fleet_dict(
                            fleet_payloads[sender_id],
                            attack_case,
                            rng,
                            fleet_target=args.fleet_target,
                            attacker_id=args.attacker_id,
                        )

            attack_status = build_attack_status(
                attack_case=attack_case,
                t=t,
                t_start=args.t_start,
                t_end=args.t_end,
                attacker_id=args.attacker_id,
                data_type=args.data_type,
                active=active,
            )

            for host_id, estimator in enumerate(estimators):
                if hasattr(estimator, "set_v2v_attack_status"):
                    estimator.set_v2v_attack_status(attack_status)

                for sender_id in range(args.num_vehicles):
                    if sender_id == host_id:
                        continue

                    local_payload = local_payloads[sender_id]
                    if local_payload is not None:
                        estimator.add_received_local_state(
                            sender_id=sender_id,
                            state=local_payload,
                            timestamp_ns=current_time_ns,
                        )
                        estimator.add_received_clean_local_state(
                            sender_id=sender_id,
                            state=clean_local[sender_id],
                            timestamp_ns=current_time_ns,
                        )

                    fleet_payload = fleet_payloads[sender_id]
                    if fleet_payload is not None:
                        estimator.add_received_fleet_state(
                            sender_id=sender_id,
                            fleet_estimates=fleet_payload,
                            timestamp_ns=current_time_ns,
                        )

                control = np.array([0.0, states[host_id, 4]], dtype=float)
                estimator.update(
                    local_state=states[host_id].copy(),
                    dt=args.dt,
                    current_time_ns=current_time_ns,
                    control=control,
                )

            estimates_over_time.append(
                np.stack([est.fleet_states.copy() for est in estimators], axis=0)
            )
            truth_over_time.append(states.copy())
            times.append(t)

            for host_id in non_attacker_ids:
                score = estimators[host_id].get_all_trust_scores().get(
                    args.attacker_id, float("nan")
                )
                trust_by_host[host_id].append(float(score))

            step_true_dynamics(states, t, args.dt)
    finally:
        stop_estimators(estimators)

    estimate_arr = np.stack(estimates_over_time, axis=0)
    truth_arr = np.stack(truth_over_time, axis=0)
    times_arr = np.asarray(times, dtype=float)
    mean_errors = compute_target_rmse(estimate_arr, truth_arr, observers=non_attacker_ids)

    rows: List[Dict[str, Any]] = []
    for vehicle_id in non_attacker_ids:
        trust_trace = np.asarray(trust_by_host[vehicle_id], dtype=float)
        mean_trust, degradation, detection_time = trust_metrics(
            trust_trace,
            times_arr,
            args.t_start,
            args.t_end,
            threshold=args.detection_threshold,
        )
        rows.append(
            {
                "AttackType": "Mix_test",
                "AttackerVehicle": f"V{args.attacker_id}",
                "Case": f"Case {attack_case.case_number}",
                "AttackLabel": attack_case.label,
                "Vehicle": f"V{vehicle_id}",
                "Mean_Distance": mean_errors[vehicle_id, 0],
                "Mean_Orientation": mean_errors[vehicle_id, 1],
                "Mean_Velocity": mean_errors[vehicle_id, 2],
                "Mean_Acceleration": mean_errors[vehicle_id, 3],
                "Mean_Trust_Score": mean_trust,
                "Trust_Degradation": degradation,
                "Attack_Detection_Time": detection_time,
            }
        )

    return rows


def write_summary(rows: Sequence[Dict[str, Any]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "AttackType",
        "AttackerVehicle",
        "Case",
        "AttackLabel",
        "Vehicle",
        "Mean_Distance",
        "Mean_Orientation",
        "Mean_Velocity",
        "Mean_Acceleration",
        "Mean_Trust_Score",
        "Trust_Degradation",
        "Attack_Detection_Time",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def make_plots(rows: Sequence[Dict[str, Any]], output_dir: Path) -> None:
    if not rows:
        return

    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"[plot] matplotlib unavailable, skipping plots: {exc}")
        return

    cases = sorted({int(str(row["Case"]).split()[-1]) for row in rows})
    vehicles = sorted({str(row["Vehicle"]) for row in rows})
    labels = [MIX_TEST_CASES[case_id - 1].label for case_id in cases]
    vehicle_ids = [int(v[1:]) for v in vehicles]
    case_to_idx = {case_id: idx for idx, case_id in enumerate(cases)}

    def matrix(metric: str) -> np.ndarray:
        data = np.full((len(vehicles), len(cases)), np.nan, dtype=float)
        for row in rows:
            v_idx = vehicles.index(str(row["Vehicle"]))
            c_idx = case_to_idx[int(str(row["Case"]).split()[-1])]
            data[v_idx, c_idx] = float(row[metric])
        return data

    dist = matrix("Mean_Distance")
    vel = matrix("Mean_Velocity")
    acc = matrix("Mean_Acceleration")
    trust = matrix("Mean_Trust_Score")

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), constrained_layout=True)
    for ax, data, title, cbar_label in (
        (axes[0], dist, "Distance Error", "m"),
        (axes[1], vel, "Velocity Error", "m/s"),
        (axes[2], acc, "Acceleration Error", "m/s^2"),
    ):
        im = ax.imshow(data, aspect="auto")
        ax.set_title(title)
        ax.set_xticks(range(len(labels)), labels, rotation=35, ha="right")
        ax.set_yticks(range(len(vehicles)), vehicles)
        for i in range(data.shape[0]):
            for j in range(data.shape[1]):
                ax.text(j, i, f"{data[i, j]:.3f}", ha="center", va="center", color="black")
        fig.colorbar(im, ax=ax, label=cbar_label)
    fig.savefig(output_dir / "mean_error_heatmaps.png", dpi=180)
    plt.close(fig)

    raw_combined = dist + vel + acc
    norm_components = []
    for data in (dist, vel, acc):
        finite = data[np.isfinite(data)]
        if finite.size == 0 or np.nanmax(data) == np.nanmin(data):
            norm_components.append(np.zeros_like(data))
        else:
            norm_components.append((data - np.nanmin(data)) / (np.nanmax(data) - np.nanmin(data)))
    impact = sum(norm_components) / len(norm_components)

    fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    im = ax.imshow(impact, aspect="auto")
    ax.set_title("Normalized Impact Color + Raw Combined Error")
    ax.set_xticks(range(len(labels)), labels, rotation=35, ha="right")
    ax.set_yticks(range(len(vehicles)), vehicles)
    for i in range(raw_combined.shape[0]):
        for j in range(raw_combined.shape[1]):
            ax.text(j, i, f"{raw_combined[i, j]:.3f}", ha="center", va="center", color="black")
    fig.colorbar(im, ax=ax, label="Normalized impact")
    fig.savefig(output_dir / "combined_impact_heatmap.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    im = ax.imshow(trust, aspect="auto", vmin=0.0, vmax=1.0)
    ax.set_title("Mean Trust Toward Attacker")
    ax.set_xticks(range(len(labels)), labels, rotation=35, ha="right")
    ax.set_yticks(range(len(vehicle_ids)), vehicles)
    for i in range(trust.shape[0]):
        for j in range(trust.shape[1]):
            ax.text(j, i, f"{trust[i, j]:.2f}", ha="center", va="center", color="black")
    fig.colorbar(im, ax=ax, label="Trust")
    fig.savefig(output_dir / "mean_trust_heatmap.png", dpi=180)
    plt.close(fig)


def parse_case_selection(raw: str) -> List[int]:
    if not raw:
        return [case.case_number for case in MIX_TEST_CASES]
    selected: List[int] = []
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start, end = part.split("-", 1)
            selected.extend(range(int(start), int(end) + 1))
        else:
            selected.append(int(part))
    valid = {case.case_number for case in MIX_TEST_CASES}
    return [case for case in selected if case in valid]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a 5-vehicle MATLAB-style Mix_test attack batch using the Python trust estimator."
    )
    parser.add_argument("--num-vehicles", type=int, default=5)
    parser.add_argument("--attacker-id", type=int, default=0)
    parser.add_argument("--simulation-time", type=float, default=25.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--t-start", type=float, default=10.0)
    parser.add_argument("--t-end", type=float, default=15.0)
    parser.add_argument(
        "--data-type",
        choices=("local", "fleet", "both"),
        default="fleet",
        help="MATLAB global data maps to Python fleet-state broadcasts.",
    )
    parser.add_argument(
        "--fleet-target",
        choices=("all", "attacker"),
        default="all",
        help="For fleet attacks, corrupt all entries or only the attacker's entry.",
    )
    parser.add_argument("--cases", default="1-6", help="Case list, e.g. 1,3,6 or 1-6.")
    parser.add_argument("--seed", type=int, default=10)
    parser.add_argument("--detection-threshold", type=float, default=0.7)
    parser.add_argument(
        "--prediction-mode",
        default="model",
        help="Fleet estimator dynamics_prediction_mode for this batch.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).resolve().parent / "results" / "mean_attack_batch"),
    )
    parser.add_argument("--no-plots", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.num_vehicles < 2:
        raise ValueError("--num-vehicles must be at least 2")
    if not 0 <= args.attacker_id < args.num_vehicles:
        raise ValueError("--attacker-id must be inside the vehicle ID range")

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    selected_cases = set(parse_case_selection(args.cases))
    config = load_trust_config(args.prediction_mode)

    all_rows: List[Dict[str, Any]] = []
    for attack_case in MIX_TEST_CASES:
        if attack_case.case_number not in selected_cases:
            continue
        print(
            f"[case {attack_case.case_number}] {attack_case.label} "
            f"attacker=V{args.attacker_id} data_type={args.data_type}"
        )
        all_rows.extend(run_case(attack_case, args, config, output_dir))

    summary_path = output_dir / f"Results_Mix_test_Attacker_V{args.attacker_id}.csv"
    write_summary(all_rows, summary_path)
    if not args.no_plots:
        make_plots(all_rows, output_dir)

    print(f"[done] Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
