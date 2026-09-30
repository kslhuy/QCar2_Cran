"""Simulation-only RobustKLNet configuration, separate from real-car defaults."""
from pathlib import Path


def simulation_motion_params(car):
    """Same static calibration for robust and EKF simulation comparisons."""
    params = {
        "wheelbase": float(car.params.a+car.params.b),
        "kin_wheelbase": float(car.params.a+car.params.b),
        "v_lpf_alpha": 1., "accel_ema_alpha": 1., "local_output_lpf_alpha": 1.,
    }
    if car.longitudinal_model not in {"velocity_lag", "velocity_lag_lookup"}:
        return params
    params.update({
        "longitudinal_model": car.longitudinal_model,
        "positive_accel_switch": float(car.params.longitudinal.v_switch),
        "velocity_lag_model": {
            "tau": float(car.velocity_lag_tau), "velocity_gain": float(car.velocity_gain),
            "throttle_deadband": float(car.velocity_lag_deadband),
        },
        "velocity_lag_lookup_model": {
            "tau": float(car.velocity_lag_lookup_tau),
            "throttle_breakpoints": car.velocity_lag_lookup_throttle_breakpoints.tolist(),
            "steady_state_velocity_breakpoints": car.velocity_lag_lookup_velocity_breakpoints.tolist(),
        },
    })
    return params


def simulation_estimator_params(car):
    """Read static plant calibration, never its current state or clean sensors."""
    checkpoint = Path(__file__).resolve().parents[1] / "Observer/KalmaNet/Robust/models/innovation_trust_sim.npz"
    if car.longitudinal_model not in {"velocity_lag", "velocity_lag_lookup"}:
        raise ValueError("The simulation trust checkpoint requires a calibrated velocity_lag or velocity_lag_lookup plant")
    if car.steering_model != "default":
        raise ValueError("The simulation trust checkpoint is validated with the default steering actuator")
    return {
        **simulation_motion_params(car),
        "estimator_backend": "innovation_trust", "model_path": str(checkpoint),
        "device": "cpu", "streaming_inference": True,
        "publish_clean_reference_output": False,
        "enable_ekf_comparator": False, "enable_clean_reference_comparator": False,
        "comparator_record_to_file": False, "heading_filter_enabled": False,
        "sensor_failure_simulation": {"enabled": False},
    }
