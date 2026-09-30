"""MockQCar + repository Stanley/PID controllers driven by estimated state.

This is a deterministic headless control-loop test, not the networked ground
station UI. The plant trajectory changes in response to each estimator.
"""
import argparse
import contextlib
import io
import json
import hashlib
from pathlib import Path

import numpy as np
import torch
import yaml

from simulation_benchmark import HERE, QCAR, model_config, wrap, metrics
from simulation.mock_vehicle import MockQCar
from simulation.robust_estimator_config import simulation_estimator_params
from Observer.local_state_estimators import EKFStateEstimator
from Controller.lateral_controllers import StanleyController
from Controller.longitudinal_controllers import PIDVelocityController
from robustKLnet import RobustKalmanNetStateEstimator

CLOSED_LOOP_SCENARIOS = ("clean", "gps_jump", "gps_ramp", "gps_freeze", "gps_dropout",
                         "heading_bias", "wheel_bias", "wheel_freeze", "gps_wheel")


def run(seed, scenario, estimator_name, duration=40., dt=.02, checkpoint=None):
    rng = np.random.default_rng(seed)
    cfg = yaml.safe_load((QCAR/"simulation/parameters.yaml").read_text(encoding="utf-8"))
    cfg["vehicle"].update(model_type="kinematic", longitudinal_model="qlabs_velocity", steering_model="default")
    cfg["electronics"]["enabled"] = False
    with contextlib.redirect_stdout(io.StringIO()):
        car = MockQCar(cfg)
    radius = 2.
    initial = np.array([radius, 0., np.pi/2])
    car.reset_pose(initial)
    robust_params = simulation_estimator_params(car)
    if checkpoint is not None:
        robust_params["model_path"] = str(checkpoint)
    estimator = (RobustKalmanNetStateEstimator(initial, robust_params)
                 if estimator_name == "robust" else EKFStateEstimator(initial, model_config()))
    phi = np.linspace(0, 2*np.pi, 401)
    lateral = StanleyController(np.array([radius*np.cos(phi), radius*np.sin(phi)]),
                                 k_e=2., k_soft=.2, position_lookahead_offset=.13, max_steering=.5)
    longitudinal = PIDVelocityController(kp=.12, ki=.25, kd=0., max_throttle=.16, ff_gain=.16)
    predictions, truth, active, tracking, speeds = [], [], [], [], []
    frozen_gps, frozen_speed = None, None
    next_gps = 0.
    for i in range(round(duration/dt)):
        t = i*dt
        estimate = estimator.get_state()
        steering = lateral.update(estimate[:2], estimate[2], estimate[3])
        throttle = longitudinal.update(estimate[3], .65, dt)
        car.write(throttle, steering)
        car.step(dt)
        gps = None
        if t+1e-9 >= next_gps:
            z = np.array([car.x, car.y, car.heading])+rng.normal(size=3)*[.035, .035, .015]
            gps = dict(x=z[0], y=z[1], theta=wrap(z[2]), valid=True, fresh=True, position_valid=True)
            next_gps += .1
        sample = dict(motor_tach=car.motorTach+rng.normal(0, .012),
                      steering=steering, throttle=throttle, dt=dt,
                      gyro_z=car.gyroscope[2]+rng.normal(0, .008),
                      acceleration=car.accelerometer.copy()+rng.normal(0, .04, 3), gps_data=gps)
        attacked = scenario != "clean" and .25*duration <= t < .60*duration
        if attacked:
            if frozen_speed is None:
                frozen_speed = sample["motor_tach"]
            if gps is not None and frozen_gps is None:
                frozen_gps = gps.copy()
            if scenario in ("gps_jump", "gps_wheel") and gps is not None:
                gps["x"] += 1.
                gps["y"] -= .7
            elif scenario == "gps_ramp" and gps is not None:
                gps["x"] += .12*(t-.25*duration)
                gps["y"] -= .08*(t-.25*duration)
            elif scenario == "gps_freeze" and gps is not None:
                sample["gps_data"] = frozen_gps.copy()
            elif scenario == "gps_dropout":
                sample["gps_data"] = None
            elif scenario == "heading_bias" and gps is not None:
                gps["theta"] = wrap(gps["theta"]+.9)
            if scenario in ("wheel_bias", "gps_wheel"):
                sample["motor_tach"] += .65
            if scenario == "wheel_freeze":
                sample["motor_tach"] = frozen_speed
        if not estimator.update(**sample):
            raise RuntimeError(f"Estimator failed at {t}s")
        predictions.append(estimator.get_state()[:4])
        truth.append([car.x, car.y, car.heading, car.velocity])
        active.append(attacked)
        tracking.append(abs(np.hypot(car.x, car.y)-radius))
        speeds.append(car.velocity)
    result = metrics(np.asarray(predictions), np.asarray(truth), np.asarray(active), np.zeros(len(active)))
    result.pop("runtime_median_ms")
    result.pop("runtime_p95_ms")
    settled = np.arange(len(active))*dt > 3.
    result.update(seed=seed, scenario=scenario, estimator=estimator_name,
                  tracking_rmse_m=float(np.sqrt(np.mean(np.asarray(tracking)[settled]**2))),
                  tracking_max_m=float(np.max(tracking)),
                  speed_tracking_rmse_mps=float(np.sqrt(np.mean((np.asarray(speeds)[settled]-.65)**2))))
    return result, np.asarray(truth), np.asarray(predictions)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=[401, 402, 403])
    parser.add_argument("--duration", type=float, default=40.)
    parser.add_argument("--checkpoint", type=Path, default=HERE/"models/innovation_trust_sim.npz")
    parser.add_argument("--output", type=Path, default=HERE/"results/closed_loop.json")
    parser.add_argument("--scenarios", nargs="+", default=list(CLOSED_LOOP_SCENARIOS), choices=CLOSED_LOOP_SCENARIOS)
    args = parser.parse_args()
    if args.duration <= 3:
        parser.error("--duration must exceed the 3-second controller settling period")
    torch.set_num_threads(1)
    rows, arrays = [], {}
    for seed in args.seeds:
        for scenario in args.scenarios:
            for name in ("ekf", "robust"):
                row, truth, pred = run(seed, scenario, name, args.duration, checkpoint=args.checkpoint)
                rows.append(row)
                arrays[f"{seed}_{scenario}_{name}_truth"] = truth
                arrays[f"{seed}_{scenario}_{name}_estimate"] = pred
                tick_time = np.arange(len(truth)) * .02
                arrays[f"{seed}_{scenario}_attack"] = ((tick_time >= .25*args.duration) &
                    (tick_time < .60*args.duration) & (scenario != "clean"))
                print(f"{seed} {scenario:14s} {name:6s} tracking={row['tracking_rmse_m']:.4f} "
                      f"estimation={row['position_rmse_m']:.4f}", flush=True)
    destination = args.output
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(dict(protocol="headless_mockqcar_stanley_pid", duration=args.duration,
                                         dt=.02, seeds=args.seeds, results=rows,
                                         checkpoint=str(args.checkpoint.resolve()),
                                         checkpoint_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest()), indent=2), encoding="utf-8")
    np.savez_compressed(destination.with_suffix(".npz"), **arrays)


if __name__ == "__main__":
    main()
