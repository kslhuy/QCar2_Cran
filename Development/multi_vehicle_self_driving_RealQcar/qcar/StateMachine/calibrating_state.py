"""
Calibrating State - Active Vehicle Calibration Sequences

Dedicated state where the vehicle autonomously executes calibration sequences:
1) throttle_velocity   – staircase throttle, measure steady-state velocity
2) steering_curvature  – constant speed + steering sweep, measure yaw rate
3) throttle_acceleration – throttle step transitions, measure dynamics

The state takes full control of the vehicle (like ManualModeState).
Uses a non-blocking phase/step state machine inside update() — no time.sleep().

Transitions:
    WAITING_FOR_START / STOPPED  →  CALIBRATING  →  STOPPED
"""

import os
import sys
import time
import csv
from typing import Dict, Any, Tuple, Optional, List

import numpy as np

from .state_base import StateBase
from .vehicle_state import VehicleState, StateTransitionReason

# Import CommandType
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

try:
    from command_types import CommandType

    COMMAND_TYPE_AVAILABLE = True
except ImportError as e:
    print(f"ERROR: Cannot import CommandType: {e}")
    COMMAND_TYPE_AVAILABLE = False
    CommandType = None

# Try importing YAML for saving calibration results
try:
    import yaml

    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


# ========================================================================
# Internal calibration phases (non-blocking state machine)
# ========================================================================
class _Phase:
    IDLE = "idle"
    WARMUP = "warmup"          # Let vehicle reach initial conditions
    RUNNING = "running"        # Executing the current step
    SETTLING = "settling"      # Wait for transient to die out
    NEXT_STEP = "next_step"    # Advance to next step
    ANALYSING = "analysing"    # Fit model to collected data
    DONE = "done"              # Calibration complete


# ========================================================================
# Default calibration parameters
# ========================================================================
DEFAULT_THROTTLE_VELOCITY_PARAMS = {
    "throttle_levels": [0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16],
    "hold_time": 5.0,       # seconds at each throttle level
    "settle_time": 2.0,     # seconds to stabilise before measuring
    "warmup_time": 2.0,     # seconds warmup at first level
    "poly_degree": 3,       # degree of fitted polynomial
}

DEFAULT_STEERING_CURVATURE_PARAMS = {
    "steering_levels": [-0.5, -0.4, -0.3, -0.2, -0.1,
                        0.1, 0.2, 0.3, 0.4, 0.5],
    "cruise_throttle": 0.08,   # constant throttle during steering sweep
    "hold_time": 4.0,
    "settle_time": 2.0,
    "warmup_time": 3.0,
    "wheelbase": 0.256,        # nominal QCar2 wheelbase [m]
}

DEFAULT_THROTTLE_ACCEL_PARAMS = {
    "step_pairs": [
        (0.04, 0.08),   # (from_throttle, to_throttle)
        (0.06, 0.10),
        (0.08, 0.12),
        (0.10, 0.14),
    ],
    "pre_step_time": 3.0,   # seconds at 'from' throttle
    "step_time": 5.0,       # seconds at 'to' throttle (measure response)
    "settle_time": 2.0,
    "warmup_time": 2.0,
}


class CalibratingState(StateBase):
    """Handler for CALIBRATING state — active vehicle calibration."""

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def enter(self) -> bool:
        """Initialise calibration state."""
        super().enter()
        self.logger.logger.info("[CAL] Entering CALIBRATING state")

        # Read calibration request from vehicle_logic
        cal_request = getattr(self.vehicle_logic, "_calibration_request", {}) or {}
        self.calibration_type = str(
            cal_request.get("calibration_type", "throttle_velocity")
        )
        user_params = cal_request.get("params", {}) or {}

        # Merge user params with defaults
        if self.calibration_type == "throttle_velocity":
            self.params = {**DEFAULT_THROTTLE_VELOCITY_PARAMS, **user_params}
        elif self.calibration_type == "steering_curvature":
            self.params = {**DEFAULT_STEERING_CURVATURE_PARAMS, **user_params}
        elif self.calibration_type == "throttle_acceleration":
            self.params = {**DEFAULT_THROTTLE_ACCEL_PARAMS, **user_params}
        else:
            self.logger.log_error(
                f"[CAL] Unknown calibration_type: {self.calibration_type}"
            )
            return False

        # Internal state machine
        self._phase = _Phase.WARMUP
        self._step_index = 0
        self._phase_start_time = time.time()

        # Data recording
        self._records: List[Dict[str, float]] = []

        # Per-step aggregates (for analysis)
        self._step_results: List[Dict[str, Any]] = []

        # Build step sequence
        self._build_step_sequence()

        self.logger.logger.info(
            f"[CAL] Calibration type: {self.calibration_type}, "
            f"{len(self._steps)} steps, params={self.params}"
        )
        return True

    def exit(self):
        """Ensure vehicle is stopped when leaving calibration."""
        self.logger.logger.info(
            f"[CAL] Exiting CALIBRATING state after "
            f"{self.get_time_in_state():.1f}s, "
            f"{len(self._records)} samples recorded"
        )
        self._stop_vehicle()
        super().exit()

    # ------------------------------------------------------------------
    # update() — called every control loop tick
    # ------------------------------------------------------------------
    def update(
        self, dt: float, sensor_data: Dict[str, Any]
    ) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        now = time.time()
        elapsed_in_phase = now - self._phase_start_time

        # Determine commanded throttle / steering for this tick
        throttle = 0.0
        steering = 0.0

        # ---- WARMUP ----
        if self._phase == _Phase.WARMUP:
            throttle, steering = self._get_step_command(0)
            warmup_time = self.params.get("warmup_time", 2.0)
            if elapsed_in_phase >= warmup_time:
                self._phase = _Phase.RUNNING
                self._phase_start_time = now
                self._step_index = 0
                self.logger.logger.info("[CAL] Warmup complete → RUNNING step 0")

        # ---- RUNNING ----
        elif self._phase == _Phase.RUNNING:
            throttle, steering = self._get_step_command(self._step_index)
            self._record_sample(now, throttle, steering, sensor_data)

            hold_time = self._get_hold_time()
            if elapsed_in_phase >= hold_time:
                # Step complete → aggregate data for this step
                self._aggregate_step(self._step_index)
                self._phase = _Phase.SETTLING
                self._phase_start_time = now
                self.logger.logger.info(
                    f"[CAL] Step {self._step_index} done → SETTLING"
                )

        # ---- SETTLING ----
        elif self._phase == _Phase.SETTLING:
            # Keep last command during settling (or coast)
            throttle, steering = self._get_step_command(self._step_index)
            settle_time = self.params.get("settle_time", 2.0)
            if elapsed_in_phase >= settle_time:
                self._phase = _Phase.NEXT_STEP
                self._phase_start_time = now

        # ---- NEXT_STEP ----
        elif self._phase == _Phase.NEXT_STEP:
            self._step_index += 1
            if self._step_index < len(self._steps):
                self._phase = _Phase.RUNNING
                self._phase_start_time = now
                throttle, steering = self._get_step_command(self._step_index)
                self.logger.logger.info(
                    f"[CAL] Starting step {self._step_index}/{len(self._steps)}"
                )
            else:
                self._phase = _Phase.ANALYSING
                self._phase_start_time = now
                self.logger.logger.info("[CAL] All steps done → ANALYSING")

        # ---- ANALYSING ----
        elif self._phase == _Phase.ANALYSING:
            throttle, steering = 0.0, 0.0
            self._run_analysis()
            self._phase = _Phase.DONE
            self._phase_start_time = now

        # ---- DONE ----
        elif self._phase == _Phase.DONE:
            self._stop_vehicle()
            self.logger.logger.info("[CAL] Calibration DONE → transitioning to STOPPED")
            return 0.0, 0.0, (
                VehicleState.STOPPED,
                StateTransitionReason.CALIBRATION_COMPLETE,
            )

        # Send to hardware
        self._send_to_hardware(throttle, steering)

        return None, None, None  # Signal that hardware is handled internally

    # ------------------------------------------------------------------
    # Event handling
    # ------------------------------------------------------------------
    def handle_event(
        self, command_type, data: Dict[str, Any] = None
    ) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        data = data or {}
        if not COMMAND_TYPE_AVAILABLE:
            return super().handle_event(command_type, data)

        if command_type in (CommandType.STOP, CommandType.EMERGENCY_STOP):
            self.logger.logger.info(
                f"[CAL] {command_type.value} received — aborting calibration"
            )
            self._save_raw_data("aborted")
            return (VehicleState.STOPPED, StateTransitionReason.STOP_COMMAND)

        # Other commands handled by base class
        return super().handle_event(command_type, data)

    # ==================================================================
    # Step sequence builders
    # ==================================================================
    def _build_step_sequence(self):
        """Build the list of calibration steps based on calibration_type."""
        if self.calibration_type == "throttle_velocity":
            levels = self.params["throttle_levels"]
            self._steps = [
                {"throttle": float(t), "steering": 0.0} for t in levels
            ]

        elif self.calibration_type == "steering_curvature":
            levels = self.params["steering_levels"]
            throttle = self.params["cruise_throttle"]
            self._steps = [
                {"throttle": float(throttle), "steering": float(s)} for s in levels
            ]

        elif self.calibration_type == "throttle_acceleration":
            pairs = self.params["step_pairs"]
            self._steps = []
            for from_t, to_t in pairs:
                # Each "step" has two phases: pre-step then step
                self._steps.append({
                    "throttle_from": float(from_t),
                    "throttle_to": float(to_t),
                    "steering": 0.0,
                })
        else:
            self._steps = []

    def _get_step_command(self, step_idx: int) -> Tuple[float, float]:
        """Return (throttle, steering) for the given step index."""
        if step_idx >= len(self._steps):
            return 0.0, 0.0

        step = self._steps[step_idx]

        if self.calibration_type == "throttle_acceleration":
            # Two-phase step: pre_step_time at from_throttle, then to_throttle
            elapsed = time.time() - self._phase_start_time
            pre_step_time = self.params.get("pre_step_time", 3.0)
            if elapsed < pre_step_time:
                return step["throttle_from"], step["steering"]
            else:
                return step["throttle_to"], step["steering"]
        else:
            return step.get("throttle", 0.0), step.get("steering", 0.0)

    def _get_hold_time(self) -> float:
        """Total hold time for current step."""
        if self.calibration_type == "throttle_acceleration":
            return self.params.get("pre_step_time", 3.0) + self.params.get(
                "step_time", 5.0
            )
        return self.params.get("hold_time", 5.0)

    # ==================================================================
    # Data recording
    # ==================================================================
    def _record_sample(
        self,
        timestamp: float,
        throttle: float,
        steering: float,
        sensor_data: Dict[str, Any],
    ):
        """Record one sample from the observer."""
        observer = getattr(self.vehicle_logic, "vehicle_observer", None)
        if observer is None:
            return

        with observer.lock:
            velocity = float(observer.local_state[3]) if len(observer.local_state) > 3 else 0.0
            yaw_rate = float(observer.sensor_data.get("gyro_z", 0.0))
            accel = observer.sensor_data.get("accelerometer", np.zeros(3))
            motor_tach = float(observer.sensor_data.get("motor_tach", 0.0))

        self._records.append({
            "time": timestamp,
            "step_index": self._step_index,
            "throttle": throttle,
            "steering": steering,
            "velocity": velocity,
            "motor_tach": motor_tach,
            "yaw_rate": yaw_rate,
            "accel_x": float(accel[0]),
            "accel_y": float(accel[1]),
            "accel_z": float(accel[2]),
        })

    def _aggregate_step(self, step_idx: int):
        """Aggregate recorded data for the completed step."""
        step_data = [r for r in self._records if r["step_index"] == step_idx]
        if not step_data:
            return

        # Use the last 60% of samples (skip initial transient)
        n = len(step_data)
        steady_start = int(n * 0.4)
        steady = step_data[steady_start:]

        if not steady:
            return

        agg = {
            "step_index": step_idx,
            "command": self._steps[step_idx].copy(),
            "n_samples": len(step_data),
            "n_steady": len(steady),
            "mean_velocity": float(np.mean([s["velocity"] for s in steady])),
            "std_velocity": float(np.std([s["velocity"] for s in steady])),
            "mean_yaw_rate": float(np.mean([s["yaw_rate"] for s in steady])),
            "mean_accel_x": float(np.mean([s["accel_x"] for s in steady])),
        }
        self._step_results.append(agg)

        self.logger.logger.info(
            f"[CAL] Step {step_idx} aggregate: v={agg['mean_velocity']:.4f} m/s, "
            f"ω={agg['mean_yaw_rate']:.4f} rad/s, n_steady={agg['n_steady']}"
        )

    # ==================================================================
    # Analysis and model fitting
    # ==================================================================
    def _run_analysis(self):
        """Run the appropriate analysis based on calibration_type."""
        self.logger.logger.info(
            f"[CAL] Running {self.calibration_type} analysis with "
            f"{len(self._step_results)} steps"
        )

        if self.calibration_type == "throttle_velocity":
            self._analyse_throttle_velocity()
        elif self.calibration_type == "steering_curvature":
            self._analyse_steering_curvature()
        elif self.calibration_type == "throttle_acceleration":
            self._analyse_throttle_acceleration()

        # Always save raw data
        self._save_raw_data("complete")

    def _analyse_throttle_velocity(self):
        """Fit polynomial: velocity = f(throttle)."""
        if len(self._step_results) < 2:
            self.logger.log_warning("[CAL] Not enough steps for throttle-velocity fit")
            return

        throttles = np.array([s["command"]["throttle"] for s in self._step_results])
        velocities = np.array([s["mean_velocity"] for s in self._step_results])

        degree = self.params.get("poly_degree", 3)
        degree = min(degree, len(throttles) - 1)

        try:
            coeffs = np.polyfit(throttles, velocities, degree)
            poly_str = " + ".join(
                [f"{c:.6f}*t^{degree - i}" for i, c in enumerate(coeffs)]
            )
            self.logger.logger.info(
                f"[CAL] Throttle→Velocity polynomial (deg {degree}): {poly_str}"
            )

            # Save results
            result = {
                "calibration_type": "throttle_velocity",
                "poly_degree": int(degree),
                "poly_coefficients": [float(c) for c in coeffs],
                "throttle_values": [float(t) for t in throttles],
                "velocity_values": [float(v) for v in velocities],
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            }
            self._save_yaml(result, "throttle_velocity_poly.yaml")

        except Exception as e:
            self.logger.log_error("[CAL] Throttle-velocity polyfit failed", e)

    def _analyse_steering_curvature(self):
        """Fit curvature = f(steering) and estimate effective wheelbase."""
        if len(self._step_results) < 2:
            self.logger.log_warning("[CAL] Not enough steps for steering-curvature fit")
            return

        steerings = np.array([s["command"]["steering"] for s in self._step_results])
        velocities = np.array([s["mean_velocity"] for s in self._step_results])
        yaw_rates = np.array([s["mean_yaw_rate"] for s in self._step_results])

        # Curvature κ = ω / v (avoid division by zero)
        valid = np.abs(velocities) > 0.01
        curvatures = np.zeros_like(yaw_rates)
        curvatures[valid] = yaw_rates[valid] / velocities[valid]

        try:
            # Polynomial fit: curvature = f(steering)
            degree = min(3, len(steerings[valid]) - 1)
            if degree < 1:
                self.logger.log_warning("[CAL] Not enough valid points for curvature fit")
                return

            coeffs = np.polyfit(steerings[valid], curvatures[valid], degree)

            # Ackermann wheelbase estimate: κ = tan(δ) / L → L = tan(δ) / κ
            L_estimates = []
            for s, k in zip(steerings[valid], curvatures[valid]):
                if abs(k) > 1e-4 and abs(s) > 0.01:
                    L_est = abs(np.tan(s) / k)
                    if 0.05 < L_est < 1.0:  # sanity check
                        L_estimates.append(L_est)

            effective_wheelbase = float(np.median(L_estimates)) if L_estimates else self.params.get("wheelbase", 0.256)

            self.logger.logger.info(
                f"[CAL] Effective wheelbase: {effective_wheelbase:.4f} m "
                f"(from {len(L_estimates)} estimates)"
            )

            result = {
                "calibration_type": "steering_curvature",
                "poly_degree": int(degree),
                "poly_coefficients": [float(c) for c in coeffs],
                "effective_wheelbase": effective_wheelbase,
                "ackermann_model": "kappa = tan(delta_eff) / L_eff",
                "steering_values": [float(s) for s in steerings],
                "curvature_values": [float(k) for k in curvatures],
                "velocity_values": [float(v) for v in velocities],
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            }
            self._save_yaml(result, "steering_calibration.yaml")

        except Exception as e:
            self.logger.log_error("[CAL] Steering-curvature analysis failed", e)

    def _analyse_throttle_acceleration(self):
        """Identify first-order model parameters from step transitions."""
        if len(self._step_results) < 1:
            self.logger.log_warning("[CAL] No steps for throttle-acceleration analysis")
            return

        step_models = []
        for step_result in self._step_results:
            step_idx = step_result["step_index"]
            step_data = [r for r in self._records if r["step_index"] == step_idx]
            if len(step_data) < 10:
                continue

            step_cmd = self._steps[step_idx]
            t_from = step_cmd["throttle_from"]
            t_to = step_cmd["throttle_to"]
            pre_step_time = self.params.get("pre_step_time", 3.0)

            # Split into pre-step and step phases
            times = np.array([s["time"] for s in step_data])
            velocities = np.array([s["velocity"] for s in step_data])
            t0 = times[0]
            times_rel = times - t0

            # Find the transition point
            pre_mask = times_rel < pre_step_time
            step_mask = times_rel >= pre_step_time

            if np.sum(pre_mask) < 3 or np.sum(step_mask) < 3:
                continue

            v_initial = float(np.mean(velocities[pre_mask][-max(3, int(np.sum(pre_mask) * 0.3)):]))
            v_final = float(np.mean(velocities[step_mask][-max(3, int(np.sum(step_mask) * 0.3)):]))
            delta_v = v_final - v_initial

            if abs(delta_v) < 1e-4:
                continue

            # Estimate time constant τ (time to reach 63.2% of delta_v)
            target_v = v_initial + 0.632 * delta_v
            step_times = times_rel[step_mask] - pre_step_time
            step_vels = velocities[step_mask]

            tau = None
            for i in range(len(step_vels) - 1):
                if (delta_v > 0 and step_vels[i] <= target_v <= step_vels[i + 1]) or \
                   (delta_v < 0 and step_vels[i] >= target_v >= step_vels[i + 1]):
                    # Linear interpolation
                    frac = (target_v - step_vels[i]) / (step_vels[i + 1] - step_vels[i])
                    tau = step_times[i] + frac * (step_times[i + 1] - step_times[i])
                    break

            if tau is None:
                tau = float(step_times[-1] * 0.5)  # rough fallback

            # K = delta_v / delta_throttle (steady-state gain)
            delta_throttle = t_to - t_from
            K_local = delta_v / delta_throttle if abs(delta_throttle) > 1e-6 else 0.0

            model = {
                "throttle_from": float(t_from),
                "throttle_to": float(t_to),
                "v_initial": v_initial,
                "v_final": v_final,
                "delta_v": float(delta_v),
                "tau": float(tau),
                "K_local": float(K_local),
                "initial_accel": float(delta_v / tau) if tau > 1e-4 else 0.0,
            }
            step_models.append(model)
            self.logger.logger.info(
                f"[CAL] Step {step_idx}: τ={tau:.3f}s, K={K_local:.3f}, "
                f"v: {v_initial:.3f}→{v_final:.3f} m/s"
            )

        # Aggregate
        if step_models:
            avg_tau = float(np.mean([m["tau"] for m in step_models]))
            avg_K = float(np.mean([m["K_local"] for m in step_models]))

            result = {
                "calibration_type": "throttle_acceleration",
                "model_type": "first_order_lag",
                "avg_tau": avg_tau,
                "avg_K": avg_K,
                "step_models": step_models,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            }
            self._save_yaml(result, "throttle_accel_model.yaml")
            self.logger.logger.info(
                f"[CAL] Throttle-acceleration model: avg τ={avg_tau:.3f}s, avg K={avg_K:.3f}"
            )
        else:
            self.logger.log_warning("[CAL] No valid step models could be identified")

    # ==================================================================
    # File I/O
    # ==================================================================
    def _get_results_dir(self) -> str:
        """Get (and create) results directory."""
        base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        results_dir = os.path.join(
            base, "Calibration", "results", f"online_{self.calibration_type}"
        )
        os.makedirs(results_dir, exist_ok=True)
        return results_dir

    def _save_yaml(self, data: dict, filename: str):
        """Save calibration results to YAML."""
        if not YAML_AVAILABLE:
            self.logger.log_warning("[CAL] PyYAML not available, skipping YAML save")
            return

        filepath = os.path.join(self._get_results_dir(), filename)
        try:
            with open(filepath, "w") as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            self.logger.logger.info(f"[CAL] Results saved to {filepath}")
        except Exception as e:
            self.logger.log_error(f"[CAL] Failed to save YAML: {filepath}", e)

    def _save_raw_data(self, tag: str = ""):
        """Save raw recorded data to CSV."""
        if not self._records:
            return

        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        filename = f"raw_{self.calibration_type}_{tag}_{timestamp_str}.csv"
        filepath = os.path.join(self._get_results_dir(), filename)

        try:
            fieldnames = list(self._records[0].keys())
            with open(filepath, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self._records)
            self.logger.logger.info(
                f"[CAL] Raw data saved ({len(self._records)} samples): {filepath}"
            )
        except Exception as e:
            self.logger.log_error(f"[CAL] Failed to save raw CSV: {filepath}", e)

    # ==================================================================
    # Hardware helpers
    # ==================================================================
    def _send_to_hardware(self, throttle: float, steering: float):
        """Send commands directly to QCar hardware."""
        try:
            if (
                not hasattr(self.vehicle_logic, "qcar")
                or self.vehicle_logic.qcar is None
            ):
                return

            LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])  # Rear lights on during cal

            self.vehicle_logic.qcar.read_write_std(
                throttle=throttle, steering=steering, LEDs=LEDs
            )

            # Keep telemetry synced
            self.vehicle_logic._last_u = float(throttle)
            self.vehicle_logic._last_steering = float(steering)

        except Exception as e:
            if hasattr(self, "_hw_error_count"):
                self._hw_error_count += 1
            else:
                self._hw_error_count = 1
            if self._hw_error_count % 50 == 1:
                self.logger.log_error("[CAL] Hardware write error", e)

    def _stop_vehicle(self):
        """Zero all commands."""
        try:
            if hasattr(self.vehicle_logic, "qcar") and self.vehicle_logic.qcar is not None:
                self.vehicle_logic.qcar.read_write_std(
                    throttle=0.0, steering=0.0,
                    LEDs=np.array([0, 0, 0, 0, 0, 0, 0, 0]),
                )
                self.vehicle_logic._last_u = 0.0
                self.vehicle_logic._last_steering = 0.0
        except Exception:
            pass
