"""Causal, ground-truth benchmark using the same MockQCar as the fake vehicle.

No target, fault label, or clean sensor is passed to an estimator. Faults are
injected before heading fusion, and each rollout has one initialization only.
"""
from __future__ import annotations

import contextlib
import copy
import io
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
QCAR = HERE.parents[2]
if str(QCAR) not in sys.path:
    sys.path.insert(0, str(QCAR))

SCENARIOS = (
    "clean", "gps_jump", "gps_ramp", "gps_freeze", "gps_dropout",
    "gps_noise", "heading_bias", "wheel_bias", "wheel_scale",
    "wheel_freeze", "imu_bias", "steering_bias", "gps_wheel",
)


def wrap(value):
    return (value + np.pi) % (2 * np.pi) - np.pi


def model_config():
    source = QCAR / "Observer/extra_configs/throttle_velocity_observer_model_qlabs.yaml"
    obs = yaml.safe_load(source.read_text(encoding="utf-8"))["observer_model"]
    return {
        "kin_wheelbase": 0.26, "wheelbase": 0.26,
        "longitudinal_model": "velocity_lag_lookup",
        "velocity_lag_lookup_model": obs["velocity_lag_lookup_model"],
        "velocity_lag_model": obs["velocity_lag_model"],
        "use_qcar_ekf": False, "gyro_heading_blend": 0.3,
        "max_velocity": 2.5, "max_acceleration": 2.0,
    }


def generate_drive(seed=1, duration=30.0, dt=0.02, gps_hz=10.0,
                   noise=True, mismatch=0.0, model_type="kinematic"):
    from simulation.mock_vehicle import MockQCar
    rng = np.random.default_rng(seed)
    cfg = yaml.safe_load((QCAR / "simulation/parameters.yaml").read_text(encoding="utf-8"))
    cfg["vehicle"].update(model_type=model_type, longitudinal_model="qlabs_velocity")
    cfg["electronics"]["enabled"] = False
    cfg["sensors"]["gps"]["has_noise"] = False
    cfg["initial_state"].update(x=0.0, y=0.0, theta=float(rng.uniform(-2, 2)), v=0.0)
    with contextlib.redirect_stdout(io.StringIO()):
        car = MockQCar(cfg)
    car.reset_pose([0., 0., cfg["initial_state"]["theta"]])
    # Plant-only perturbations; estimator calibration stays nominal.
    car.velocity_lag_lookup_tau *= 1 + mismatch
    car.velocity_lag_lookup_velocity_breakpoints *= 1 + 0.5 * mismatch
    phase = rng.uniform(0, 2 * np.pi, 4)
    samples, truth = [], []
    n = round(duration / dt)
    next_gps = 0.0
    initial = np.array([car.x, car.y, car.heading, car.velocity])
    for i in range(n):
        t = i * dt
        throttle = np.clip(0.105 + .033 * np.sin(.35*t + phase[0])
                           + .018 * np.sin(.87*t + phase[1]), .035, .16)
        # Stops and changing curvature, rather than one memorized circle.
        if 0.72 * duration < t < 0.78 * duration:
            throttle = 0.0
        steering = .22*np.sin(.43*t + phase[2]) + .09*np.sin(1.1*t + phase[3])
        car.write(float(throttle), float(steering))
        car.step(dt)
        target = np.array([car.x, car.y, wrap(car.heading), car.velocity])
        gps = None
        if t + 1e-9 >= next_gps:
            perturb = rng.normal(size=3) * ([.035, .035, .015] if noise else [0, 0, 0])
            gps = dict(x=car.x+perturb[0], y=car.y+perturb[1],
                       theta=wrap(car.heading+perturb[2]), valid=True,
                       position_valid=True, heading_valid=True, fresh=True,
                       hold_valid=True, age_sec=0.0)
            next_gps += 1 / gps_hz
        samples.append(dict(
            motor_tach=float(car.motorTach + (rng.normal(0, .012) if noise else 0)),
            steering=float(steering), throttle=float(throttle), dt=dt,
            gyro_z=float(car.gyroscope[2] + (rng.normal(0, .008) if noise else 0)),
            acceleration=np.asarray(car.accelerometer).copy()
                         + (rng.normal(0, .04, 3) if noise else 0),
            gps_data=gps,
        ))
        truth.append(target)
    return dict(samples=samples, truth=np.asarray(truth), initial=initial,
                seed=seed, dt=dt, duration=duration, gps_hz=gps_hz,
                noise=noise, mismatch=mismatch, model_type=model_type)


def attacked_samples(drive, scenario, seed=0):
    rng = np.random.default_rng(seed)
    samples = copy.deepcopy(drive["samples"])
    n = len(samples)
    mask = np.zeros(n, dtype=bool)
    # Two episodes with clean recovery, the second longer than training windows.
    episodes = [(int(.22*n), int(.45*n)), (int(.56*n), int(.70*n))]
    labels = np.zeros((n, 4), dtype=np.float32)  # untrusted [x,y,heading,v]
    if scenario == "clean":
        return samples, mask, labels
    for begin, end in episodes:
        mask[begin:end] = True
        frozen_gps = next((copy.deepcopy(s["gps_data"]) for s in samples[begin:end]
                           if s["gps_data"] is not None), None)
        frozen_v = samples[begin]["motor_tach"]
        sign = rng.choice([-1, 1])
        bias = sign * rng.uniform(.65, 1.4)
        for i in range(begin, end):
            s = samples[i]
            gps = s["gps_data"]
            elapsed = (i-begin) * drive["dt"]
            if scenario in ("gps_jump", "gps_wheel") and gps is not None:
                gps["x"] += bias
                gps["y"] -= .7 * bias
            elif scenario == "gps_ramp" and gps is not None:
                gps["x"] += sign * .12 * elapsed
                gps["y"] -= sign * .08 * elapsed
            elif scenario == "gps_freeze" and gps is not None:
                s["gps_data"] = copy.deepcopy(frozen_gps)
            elif scenario == "gps_dropout":
                s["gps_data"] = None
            elif scenario == "gps_noise" and gps is not None:
                gps["x"] += rng.normal(0, .65)
                gps["y"] += rng.normal(0, .65)
            elif scenario == "heading_bias" and gps is not None:
                gps["theta"] = wrap(gps["theta"] + sign * .9)
            if scenario in ("wheel_bias", "gps_wheel"):
                s["motor_tach"] += sign * .65
            elif scenario == "wheel_scale":
                s["motor_tach"] *= 1.9
            elif scenario == "wheel_freeze":
                s["motor_tach"] = frozen_v
            elif scenario == "imu_bias":
                s["gyro_z"] += sign * .8
                s["acceleration"][:2] += sign * 1.5
            elif scenario == "steering_bias":
                s["steering"] += sign * .3
            if scenario.startswith("gps_"):
                labels[i, :2] = 1
                if scenario == "gps_freeze":
                    labels[i, 2] = 1
            if scenario == "heading_bias":
                labels[i, 2] = 1
            if scenario.startswith("wheel_") or scenario == "gps_wheel":
                labels[i, 3] = 1
    return samples, mask, labels


def run_estimator(estimator, samples):
    states, elapsed = [], []
    for sample in samples:
        start = time.perf_counter_ns()
        if not estimator.update(**sample):
            raise RuntimeError(f"{type(estimator).__name__} failed at tick {len(states)}")
        elapsed.append((time.perf_counter_ns()-start) / 1e6)
        state = np.asarray(estimator.get_state())[:4]
        if not np.isfinite(state).all():
            raise ValueError("Non-finite estimate")
        states.append(state.copy())
    return np.asarray(states), np.asarray(elapsed)


def metrics(pred, truth, attack, elapsed):
    err = pred-truth
    err[:, 2] = wrap(err[:, 2])
    pos = np.linalg.norm(err[:, :2], axis=1)
    result = dict(position_rmse_m=float(np.sqrt(np.mean(pos**2))),
                  heading_rmse_rad=float(np.sqrt(np.mean(err[:, 2]**2))),
                  speed_rmse_mps=float(np.sqrt(np.mean(err[:, 3]**2))),
                  position_p95_m=float(np.quantile(pos, .95)),
                  position_max_m=float(pos.max()),
                  runtime_median_ms=float(np.median(elapsed)),
                  runtime_p95_ms=float(np.quantile(elapsed, .95)))
    if attack.any():
        result["attack_position_rmse_m"] = float(np.sqrt(np.mean(pos[attack]**2)))
        result["attack_heading_rmse_rad"] = float(np.sqrt(np.mean(err[attack, 2]**2)))
        result["attack_speed_rmse_mps"] = float(np.sqrt(np.mean(err[attack, 3]**2)))
    return result


def evaluate(seeds=(101, 102, 103), duration=30., dt=.02, checkpoint=None,
             legacy=False, output=None, scenarios=SCENARIOS, mismatch=0.,
             gps_hz=10., model_type="kinematic", noise=True):
    import torch
    torch.set_num_threads(1)
    from Observer.local_state_estimators import EKFStateEstimator
    from robustKLnet import RobustKalmanNetStateEstimator
    from innovation_trust import InnovationTrustFilter
    results, predictions = [], {}
    for seed in seeds:
        drive = generate_drive(seed, duration, dt, gps_hz, noise, mismatch, model_type)
        for scenario in scenarios:
            samples, attack, _ = attacked_samples(drive, scenario, seed+10000)
            cfg = model_config()
            estimators = {"ekf": EKFStateEstimator(drive["initial"][:3], cfg)}
            if checkpoint:
                robust_cfg = dict(cfg, estimator_backend="innovation_trust", model_path=str(Path(checkpoint).resolve()),
                                  device="cpu", publish_clean_reference_output=False,
                                  enable_ekf_comparator=False, enable_clean_reference_comparator=False,
                                  comparator_record_to_file=False)
                estimators["robust"] = RobustKalmanNetStateEstimator(drive["initial"][:3], robust_cfg)
            else:
                estimators["robust"] = InnovationTrustFilter(drive["initial"], cfg)
            if checkpoint:
                estimators["analytic_ablation"] = InnovationTrustFilter(drive["initial"], cfg)
            if legacy:
                old_path = HERE / "models/robust_kalmannet.best_robust.pt"
                old_cfg = torch.load(old_path, map_location="cpu", weights_only=False)["config"]
                old_cfg.update(model_path=str(old_path), device="cpu", sequence_length=1,
                               enable_ekf_comparator=False, publish_clean_reference_output=False,
                               comparator_record_to_file=False)
                # Also give the old network matched physics: report its best fair setup.
                old_cfg.update(cfg)
                estimators["legacy"] = RobustKalmanNetStateEstimator(drive["initial"][:3], old_cfg)
            for name, estimator in estimators.items():
                pred, elapsed = run_estimator(estimator, samples)
                row = dict(seed=seed, scenario=scenario, estimator=name,
                           **metrics(pred, drive["truth"], attack, elapsed))
                results.append(row)
                predictions[f"{seed}_{scenario}_{name}"] = pred
                print(f"{seed} {scenario:14s} {name:18s} pos={row['position_rmse_m']:.4f} "
                      f"psi={row['heading_rmse_rad']:.4f} v={row['speed_rmse_mps']:.4f} "
                      f"p95={row['runtime_p95_ms']:.3f}ms", flush=True)
            predictions[f"{seed}_{scenario}_truth"] = drive["truth"]
            predictions[f"{seed}_{scenario}_attack"] = attack
    payload = dict(protocol="continuous_causal_mockqcar_v1", seeds=list(seeds),
                   duration=duration, dt=dt, gps_hz=gps_hz, noise=noise,
                   mismatch=mismatch, model_type=model_type,
                   checkpoint=str(checkpoint), results=results)
    if checkpoint:
        with np.load(checkpoint) as data:
            training = json.loads(str(data["metadata"]))
        seen = set(training["train_seeds"]+training["val_seeds"])
        payload["partition"] = "development" if seen.intersection(seeds) else "held_out_test"
        payload["checkpoint_sha256"] = hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest()
    if output:
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        np.savez_compressed(output.with_suffix(".npz"), **predictions)
    return payload


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", nargs="+", type=int, default=[101, 102, 103])
    parser.add_argument("--duration", type=float, default=30.)
    parser.add_argument("--dt", type=float, default=.02)
    parser.add_argument("--gps-hz", type=float, default=10.)
    parser.add_argument("--no-noise", action="store_true")
    parser.add_argument("--mismatch", type=float, default=0.)
    parser.add_argument("--model-type", default="kinematic", choices=["kinematic", "dynamic"])
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--legacy", action="store_true")
    parser.add_argument("--scenarios", nargs="+", default=list(SCENARIOS), choices=SCENARIOS)
    parser.add_argument("--output", type=Path, default=HERE/"results/simulation_metrics.json")
    args = vars(parser.parse_args())
    args["noise"] = not args.pop("no_noise")
    evaluate(**args)


if __name__ == "__main__":
    main()
