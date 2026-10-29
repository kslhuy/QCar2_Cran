"""Headless verification of the actual local -> relative UIO -> trust pipeline.

Run from qcar: python -m simulation.observer_pipeline --duration 12
This exercises VehicleObserver with MockQCar plants and independent sensors.
It does not start Ground Station or substitute synthetic local estimates.
"""

# Keep direct-file launches working, including uploads into a renamed robot folder.
if __package__ != "qcar.simulation":
    import importlib.util as _import_util
    import sys as _sys
    from pathlib import Path as _Path
    _root = _Path(__file__).resolve().parents[1]
    if "qcar" not in _sys.modules:
        _spec = _import_util.spec_from_file_location("qcar", _root / "__init__.py", submodule_search_locations=[str(_root)])
        _pkg = _import_util.module_from_spec(_spec)
        _sys.modules["qcar"] = _pkg
        _spec.loader.exec_module(_pkg)
    __package__ = "qcar.simulation"

import argparse
from contextlib import ExitStack, redirect_stdout
import copy
import io
import json
import logging
from pathlib import Path
import random
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from qcar.Observer.VehicleObserverSimple import VehicleObserver
from qcar.Observer.TrustbasedDistributedObserver.estimator_config import CLEAN_PREDICTION_MODES
from qcar.Controller.config_controller_loader import ControllerConfig
from qcar.Controller.longitudinal_controllers import CACCLongitudinalController
from qcar.simulation.config import SimulationConfig
from qcar.simulation.mock_vehicle import MockQCar
from qcar.simulation.relative_sensor import LoopbackRelativeSensor, MockRelativeSensor
from qcar.simulation.robust_estimator_config import simulation_estimator_params, simulation_motion_params


def run_pipeline(duration=12.0, scenario="clean", seed=91, vehicles=3,
                 relative_source="external", prediction_mode="model", recovery_source="estimate",
                 force_clean_pose_anchor=False, closed_loop=False):
    """Run real estimator modules; truth is used only by sensors and metrics."""
    if scenario not in {"clean", "v2v_pose_bias", "relative_dropout", "gps_bias", "turning"}:
        raise ValueError(f"Unknown pipeline scenario: {scenario}")
    dt = 0.01
    if duration <= 0 or vehicles < 2:
        raise ValueError("Positive duration and at least two vehicles are required")
    if closed_loop and scenario == "turning":
        raise ValueError("The CACC loop is longitudinal; turning requires a lateral-control experiment")
    config = SimulationConfig.load_config()
    config["sensors"]["seed"] = seed * 1000
    clock_ns = [1_800_000_000_000_000_000]
    clock = SimpleNamespace(time=lambda: clock_ns[0] / 1e9, time_ns=lambda: clock_ns[0])
    logger = SimpleNamespace(logger=logging.getLogger("observer_pipeline"),
                             log_error=lambda *args: None, log_warning=lambda *args: None,
                             log_info=lambda *args: None)
    random_state = random.getstate()
    random.seed(seed)
    plants, observers, sensors = [], [], []
    local_errors, fleet_errors, sources, trust_trace = [], [], set(), []
    available_samples = unavailable_samples = relative_samples = 0
    controller_config = ControllerConfig() if closed_loop else None
    controllers = [CACCLongitudinalController(config=controller_config)
                   for _ in range(vehicles - 1)] if closed_loop else []
    controller_steps = missing_leader_stops = 0
    gap_errors, minimum_gap, max_command = [], float("inf"), 0.0
    attack_start, attack_end = duration * 0.3, duration * 0.65
    try:
        with ExitStack() as stack:
            for module in ("qcar.Observer.VehicleObserverSimple", "qcar.simulation.mock_sensors", "qcar.simulation.mock_vehicle"):
                stack.enter_context(patch(f"{module}.time", clock))
            for vehicle in range(vehicles):
                plant_config = copy.deepcopy(config)
                plant_config["vehicle"]["id"] = vehicle
                plant_config["initial_state"].update(x=vehicle * 1.0, y=0.0, theta=0.0, v=0.0)
                with redirect_stdout(io.StringIO()):
                    car = MockQCar(plant_config)
                car.reset_pose(np.array([vehicle * 1.0, 0.0, 0.0]))
                # The scheduler steps physics explicitly, so reading must not step it twice.
                car.read = lambda: None
                observer = VehicleObserver(vehicle, config={"observer": {"fleet_observer_rate": 25}}, logger=logger)
                observer.local_estimator_type = "robust_kalman_net"
                observer.initialize_local_estimator(gps=car.gps,
                    initial_pose=np.array([car.x, car.y, car.heading]),
                    estimator_params=simulation_estimator_params(car))
                fleet_defaults = observer.fleet_config_defaults[observer.fleet_estimator_type]
                fleet_defaults.setdefault("logging", {})["enabled"] = False
                fleet_defaults.update(relative_measurement_source=relative_source,
                                      dynamics_prediction_mode=prediction_mode,
                                      recovery_state_source=recovery_source,
                                      force_clean_pose_anchor=force_clean_pose_anchor)
                fleet_defaults.setdefault("vehicle", {}).update(simulation_motion_params(car))
                observer.reinitialize_fleet_estimation(vehicles, [peer for peer in range(vehicles) if peer != vehicle])
                sensor_config = dict(config["sensors"]["relative"], seed=seed * 100, drop_probability=0.0)
                plants.append(car)
                observers.append(observer)
                sensors.append(MockRelativeSensor(sensor_config, vehicle))
            for tick in range(round(duration / dt)):
                elapsed = tick * dt
                active = attack_start <= elapsed < attack_end
                controls = []
                for vehicle, car in enumerate(plants):
                    steering = 0.12 * np.sin(elapsed * 0.5) if scenario == "turning" else 0.0
                    throttle = 0.10 + 0.012 * np.sin(elapsed * 0.6)
                    if closed_loop and vehicle < vehicles - 1:
                        controller = controllers[vehicle]
                        leader = observers[vehicle].get_vehicle_state(vehicle + 1)
                        if leader is None:
                            controller.reset()
                            throttle = 0.0
                            missing_leader_stops += 1
                        else:
                            fields = ("x", "y", "theta", "velocity", "acceleration")
                            follower_state = dict(zip(fields, observers[vehicle].get_local_state()))
                            leader_state = dict(zip(fields, leader))
                            throttle = controller.compute_throttle(follower_state, leader_state, dt)
                            controller_steps += 1
                        if not np.isfinite(throttle) or abs(throttle) > controller.max_throttle + 1e-9:
                            raise RuntimeError("CACC produced an invalid or unbounded command")
                    max_command = max(max_command, abs(throttle))
                    controls.append(np.array([steering, throttle]))
                    car.write(float(throttle), float(steering))
                clock_ns[0] = 1_800_000_000_000_000_000 + round((tick + 1) * dt * 1e9)
                now_ns = clock_ns[0]
                for car in plants:
                    car.step(dt)
                if closed_loop:
                    for vehicle, controller in enumerate(controllers):
                        gap = plants[vehicle + 1].x - plants[vehicle].x
                        minimum_gap = min(minimum_gap, gap)
                        gap_errors.append(gap - controller.s0 - controller.h * plants[vehicle].velocity)
                world = {vehicle: (now_ns, LoopbackRelativeSensor.plant_state(car))
                         for vehicle, car in enumerate(plants)}
                # Snapshot all broadcasts before any host update (no order-dependent consensus).
                local_packets = [observer.get_local_state_for_broadcast() for observer in observers]
                fleet_packets = [observer.get_fleet_state_for_broadcast()["fleet_states"] for observer in observers]
                for host, (car, observer, sensor) in enumerate(zip(plants, observers, sensors)):
                    observer.update_sensor_data(car)
                    if scenario == "gps_bias" and host == 0 and active:
                        observer.sensor_data["gps_position"] = observer.sensor_data["gps_position"].copy()
                        observer.sensor_data["gps_position"][0] += 2.0
                    packets = sensor.sample(world[host][1], world, now_ns)
                    if not (scenario == "relative_dropout" and host == 0 and active):
                        for packet in packets:
                            observer.update_relative_measurement(
                                np.array([packet["distance"], packet["relative_velocity"]]),
                                target_id=packet["target_id"], timestamp_ns=packet["timestamp_ns"],
                                source=packet["source"], measurement_confidence=packet["confidence"],
                                relative_bearing=packet["relative_bearing"])
                            relative_samples += 1
                    for sender in range(vehicles):
                        if sender == host:
                            continue
                        payload = dict(local_packets[sender])
                        payload["control_input"] = {"steering": float(controls[sender][0]), "throttle": float(controls[sender][1])}
                        if scenario == "v2v_pose_bias" and sender == 1 and active:
                            payload["x"] += 5.0
                        observer.add_received_local_state(sender, payload, now_ns)
                        observer.add_received_clean_local_state(sender, local_packets[sender], now_ns)
                        observer.add_received_fleet_state(sender, fleet_packets[sender], now_ns)
                    observer.update_observer(dt, float(controls[host][0]), float(controls[host][1]))
                    truth = np.array([car.x, car.y])
                    local_errors.append(float(np.linalg.norm(observer.local_state[:2] - truth)))
                    for target in range(vehicles):
                        if target == host:
                            continue
                        state = observer.get_vehicle_state(target)
                        if state is None:
                            unavailable_samples += 1
                        else:
                            available_samples += 1
                            fleet_errors.append(float(np.linalg.norm(state[:2] - np.array([plants[target].x, plants[target].y]))))
                    for trust in observer.fleet_estimator.trust_model.trust_scores.values():
                        if trust.relative_measurement_source:
                            sources.add(trust.relative_measurement_source)
                    trust = observer.fleet_estimator.get_trust_score(1 if host == 0 else 0)
                    if host == 0 and trust is not None:
                        trust_trace.append(dict(time=elapsed, score=float(trust.final_score),
                            local=float(trust.local_trust_sample), flagged=bool(trust.flag_local_est_check)))
            return dict(scenario=scenario, duration_s=duration, vehicles=vehicles,
                        fleet_update_order="predict_then_correct",
                        native_parity=[observer.fleet_estimator.get_embedded_core_status()["parity_pass"]
                                       for observer in observers],
                        native_failures=[observer.fleet_estimator.get_embedded_core_status()["failures"]
                                         for observer in observers],
                        closed_loop=bool(closed_loop), controller_type="cacc" if closed_loop else None,
                        controller_steps=controller_steps, missing_leader_stops=missing_leader_stops,
                        minimum_center_gap_m=minimum_gap if closed_loop else None,
                        spacing_rmse_m=float(np.sqrt(np.mean(np.square(gap_errors)))) if gap_errors else None,
                        maximum_command=max_command,
                        reference_inputs=observers[0].fleet_estimator.get_reference_status(),
                        relative_samples=relative_samples, available_samples=available_samples,
                        unavailable_samples=unavailable_samples, relative_sources=sorted(sources),
                        local_position_rmse_m=float(np.sqrt(np.mean(np.square(local_errors)))),
                        fleet_position_rmse_m=float(np.sqrt(np.mean(np.square(fleet_errors)))) if fleet_errors else None,
                        local_backends=[observer.local_estimator.estimator_backend for observer in observers],
                        uio_targets=[sorted(observer.relative_estimators) for observer in observers],
                        trust_trace=trust_trace)
    finally:
        random.setstate(random_state)
        for observer in observers:
            observer.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--scenario", choices=["clean", "v2v_pose_bias", "relative_dropout", "gps_bias", "turning"], default="clean")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--relative-source", choices=["external", "clean_v2v"], default="external")
    parser.add_argument("--prediction-mode", choices=sorted(CLEAN_PREDICTION_MODES | {"model", "none", "dead_reckoning"}), default="model")
    parser.add_argument("--recovery-source", choices=["estimate", "direct_v2v", "clean_v2v"], default="estimate")
    parser.add_argument("--force-clean-pose-anchor", action="store_true")
    parser.add_argument("--closed-loop", action="store_true", help="Drive followers with the configured CACC and estimated leader states")
    args = parser.parse_args()
    result = run_pipeline(args.duration, args.scenario, relative_source=args.relative_source,
                          prediction_mode=args.prediction_mode, recovery_source=args.recovery_source,
                          force_clean_pose_anchor=args.force_clean_pose_anchor, closed_loop=args.closed_loop)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items() if key != "trust_trace"}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
