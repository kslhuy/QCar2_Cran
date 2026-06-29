import time
import math
import numpy as np
from pathlib import Path
from typing import Dict, Any, Optional, Tuple
from dataclasses import asdict, is_dataclass
from omegaconf import OmegaConf

# Internal imports
from .disturbances import DisturbanceGenerator
from .mock_sensors import MockQCarGPS

# QCar Project Imports (Assumes module path is set up)
from .vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from .vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from .vehiclemodels.vehicle_dynamics_qlpv import (
    vehicle_dynamics_qlpv, 
    get_tire_residuals,
    get_lateral_acceleration,
    compute_tire_forces_linear,
)
from .vehiclemodels.vehicle_parameters import setup_vehicle_parameters, VehicleParameters
from Observer.LocalNeuralObs.qlpv_vehicle_dynamics_obs import QLPVVehicleDynamicsObs

class MockQCar:
    """
    Refactored Mock QCar simulator.
    
    Supports:
    - Multiple Dynamics Models (Kinematic, Dynamic, qLPV, qLPV Matrix)
    - Generalized Disturbance Injection
    - Sensor Simulation (GPS, IMU)
    - Ground Truth Exposure for Observers
    """
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.vehicle_conf = config.get('vehicle', {})
        
        self.car_id = self.vehicle_conf.get('id', 0)
        self.model_type = self.vehicle_conf.get('model_type', 'kinematic')
        self.tire_model = self.vehicle_conf.get('tire_model', 'pacejka')
        print(f"Tire Model: {self.tire_model}")
        # Flags for optimization
        self.use_qlpv_matrix = (self.model_type == 'qlpv_matrix')
        self.use_qlpv_legacy = (self.model_type == 'qlpv_legacy')
        self.use_dynamic = (self.model_type == 'dynamic')
        
        # Disturbance Mode
        dist_conf = config.get('disturbances', {})
        self.disturbance_mode = dist_conf.get('mode', 'tire') # 'tire' or 'general'
        
        # Load Parameters
        self._load_parameters()
        
        # Initialize Components
        self.disturbance_gen = DisturbanceGenerator(config.get('disturbances'))
        self.gps = MockQCarGPS(initial_pose=[0.0, 0.0, 0.0], config=config['sensors']['gps']) # Will be updated
        
        # Initialize qLPV Observer Model with correct params and mode
        params_dict = asdict(self.params) if is_dataclass(self.params) else self.params
        self.qlpv_obs_model = QLPVVehicleDynamicsObs(
            vehicle_params=params_dict,
            disturbance_mode=self.disturbance_mode
        )
        self._load_longitudinal_plant_model()
        self._load_steering_plant_model()

        
        # State Initialization
        self._init_states()
        
        # Control Inputs
        self._throttle = 0.0
        self._steering = 0.0
        self.control_input = np.array([0.0, 0.0])
        self.current_steering_angle = 0.0 # Simulated actuator state
        
        # Mock Sensors Public Interface
        self.motorTach = 0.0
        self.gyroscope = np.array([0.0, 0.0, 0.0])
        self.accelerometer = np.array([0.0, 0.0, 9.81])
        
        # Public State Properties (synced)
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.lateral_velocity = 0.0
        
        # Ground Truth Disturbances
        self.true_disturbances = [0.0, 0.0, 0.0] # [d_vx, d_vy, d_r]
        self.w_r_true = 0.0
        self.w_f_true = 0.0
        
        # Pre-calculated tire info (computed in each update method)
        self._tire_info = {
            'Fyr_true': 0.0,
            'Fyf_true': 0.0,
            'Fyr_linear': 0.0,
            'Fyf_linear': 0.0,
            'alpha_r': 0.0,
            'alpha_f': 0.0,
        }
        
        # Last time read 
        self.last_time = time.time()

        print("="*70)
        print("[CAR] QCar Fake Vehicle with REAL VehicleLogic ")
        print("   Vehicle dynamics from vehiclemodels folder (CommonRoad models)")
        print(f"Car ID: {self.car_id}")
        print(f"Vehicle Model: {self.model_type}")
        print(f"Longitudinal Plant Model: {self.longitudinal_model}")
        if self.longitudinal_model_source_path:
            print(f"Longitudinal Model Source: {self.longitudinal_model_source_path}")
        print(f"Steering Plant Model: {self.steering_model}")
        if self.steering_model_source_path:
            print(f"Steering Model Source: {self.steering_model_source_path}")
        print(f"Tire Model: {self.tire_model}")
        print(f"Disturbance Mode: {self.disturbance_mode}")
        print("="*70)

    def _load_parameters(self):
        """Load vehicle parameters."""
        params_file = self.vehicle_conf.get('params_file', 'qcar')
        try:
            if params_file == 'qcar':
                self.params = setup_vehicle_parameters(vehicle_id='qcar')
            elif params_file.startswith('vehicle'):
                vid = int(params_file.replace('vehicle', ''))
                self.params = setup_vehicle_parameters(vehicle_id=vid)
            else:
                self.params = setup_vehicle_parameters(vehicle_id='qcar') # Fallback
        except Exception as e:
            print(f"Error loading parameters: {e}. Using QCar parameters.")
            self.params = setup_vehicle_parameters(vehicle_id='qcar')

    def _load_yaml_dict(self, path: Path) -> Dict[str, Any]:
        """Load a YAML file as a plain dictionary."""
        loaded = OmegaConf.to_object(OmegaConf.load(str(path)))
        return loaded if isinstance(loaded, dict) else {}

    def _resolve_longitudinal_source_path(self, profile: str) -> Optional[Path]:
        """Resolve a named fake-plant longitudinal profile to a YAML path."""
        qcar_dir = Path(__file__).resolve().parents[1]
        simulation_dir = Path(__file__).resolve().parent
        built_in_profiles = {
            "qcar_real": qcar_dir / "Observer" / "extra_configs" / "throttle_acceleration_observer_model_real.yaml",
            "qcar_sim": qcar_dir / "Observer" / "extra_configs" / "throttle_acceleration_observer_model_sim.yaml",
            "limo": qcar_dir / "Observer" / "extra_configs" / "throttle_acceleration_observer_model_limo.yaml",
            "real_car_61": qcar_dir / "Calibration" / "results" / "online_throttle_acceleration" / "calibration" / "61" / "throttle_acceleration_observer_model_latest.yaml",
            "qcar_real_61": qcar_dir / "Calibration" / "results" / "online_throttle_acceleration" / "calibration" / "61" / "throttle_acceleration_observer_model_latest.yaml",
            "qlabs_velocity": qcar_dir / "Observer" / "extra_configs" / "throttle_velocity_observer_model_qlabs.yaml",
        }

        source_cfg = self.vehicle_conf.get("longitudinal_model_source", {})
        profiles = source_cfg.get("profiles", {}) if isinstance(source_cfg, dict) else {}
        raw_path = None
        if isinstance(profiles, dict):
            profile_entry = profiles.get(profile)
            if isinstance(profile_entry, dict):
                raw_path = profile_entry.get("path", profile_entry.get("file"))
            elif profile_entry is not None:
                raw_path = profile_entry

        if raw_path is not None:
            path = Path(str(raw_path))
            if not path.is_absolute():
                path = simulation_dir / path
            return path

        if profile in built_in_profiles:
            return built_in_profiles[profile]

        if profile.lower().endswith((".yaml", ".yml")):
            path = Path(profile)
            if not path.is_absolute():
                path = simulation_dir / path
            return path

        return None

    @staticmethod
    def _normalized_longitudinal_profile(value: Any) -> str:
        """Normalize fake-plant longitudinal model aliases."""
        normalized = str(value or "default").strip().lower().replace("-", "_")
        aliases = {
            "": "default",
            "auto": "default",
            "legacy": "default",
            "direct": "default",
            "direct_accel": "default",
            "direct_acceleration": "default",
            "simple": "default",
            "simple_accel": "default",
            "simple_acceleration": "default",
            "real": "qcar_real",
            "qcar_real_61": "real_car_61",
            "real_car": "real_car_61",
            "qlabs": "qlabs_velocity",
            "qlab": "qlabs_velocity",
            "qlabs_velocity_lookup": "qlabs_velocity",
        }
        return aliases.get(normalized, normalized)

    def _load_longitudinal_plant_model(self):
        """Load optional calibrated fake-plant throttle-to-velocity behavior."""
        raw_profile = self.vehicle_conf.get(
            "longitudinal_model",
            self.vehicle_conf.get("longitudinal_model_profile", "default"),
        )
        source_cfg = self.vehicle_conf.get("longitudinal_model_source", {})
        if (
            isinstance(source_cfg, dict)
            and self._normalized_longitudinal_profile(raw_profile) == "default"
        ):
            raw_profile = source_cfg.get("profile", raw_profile)

        profile = self._normalized_longitudinal_profile(raw_profile)
        self.longitudinal_model_profile = profile
        self.longitudinal_model = "default"
        self.longitudinal_model_source_path = ""
        self.velocity_lag_tau = 0.301
        self.velocity_gain = 6.598
        self.velocity_lag_deadband = 0.0
        self.velocity_lag_lookup_tau = 0.301
        self.velocity_lag_lookup_throttle_breakpoints = np.asarray([], dtype=float)
        self.velocity_lag_lookup_velocity_breakpoints = np.asarray([], dtype=float)

        if profile == "default":
            return

        source_path = self._resolve_longitudinal_source_path(profile)
        observer_model: Dict[str, Any] = {}
        if source_path is not None:
            try:
                loaded = self._load_yaml_dict(source_path)
                self.longitudinal_model_source_path = str(source_path)
                observer_model = loaded.get("observer_model", loaded)
                if not isinstance(observer_model, dict):
                    observer_model = {}
            except Exception as exc:
                print(f"[WARN] Could not load longitudinal model '{source_path}': {exc}")

        vehicle_lag_cfg = self.vehicle_conf.get("velocity_lag_model")
        vehicle_lookup_cfg = self.vehicle_conf.get("velocity_lag_lookup_model")
        if isinstance(vehicle_lag_cfg, dict):
            observer_model["velocity_lag_model"] = vehicle_lag_cfg
        if isinstance(vehicle_lookup_cfg, dict):
            observer_model["velocity_lag_lookup_model"] = vehicle_lookup_cfg

        recommended = self._normalized_longitudinal_profile(
            observer_model.get("recommended_longitudinal_model", profile)
        )
        if recommended not in {"velocity_lag", "velocity_lag_lookup"}:
            recommended = profile if profile in {"velocity_lag", "velocity_lag_lookup"} else "default"

        lag_cfg = observer_model.get("velocity_lag_model", {})
        lookup_cfg = observer_model.get("velocity_lag_lookup_model", {})
        if not isinstance(lag_cfg, dict):
            lag_cfg = {}
        if not isinstance(lookup_cfg, dict):
            lookup_cfg = {}

        self.velocity_lag_tau = max(float(lag_cfg.get("tau", self.velocity_lag_tau)), 1e-6)
        self.velocity_gain = float(lag_cfg.get("velocity_gain", self.velocity_gain))
        self.velocity_lag_deadband = max(
            float(lag_cfg.get("throttle_deadband", self.velocity_lag_deadband)),
            0.0,
        )
        self.velocity_lag_lookup_tau = max(
            float(lookup_cfg.get("tau", self.velocity_lag_tau)),
            1e-6,
        )
        self.velocity_lag_lookup_throttle_breakpoints = np.asarray(
            lookup_cfg.get("throttle_breakpoints", []),
            dtype=float,
        ).reshape(-1)
        self.velocity_lag_lookup_velocity_breakpoints = np.asarray(
            lookup_cfg.get("steady_state_velocity_breakpoints", []),
            dtype=float,
        ).reshape(-1)

        has_lookup = (
            self.velocity_lag_lookup_throttle_breakpoints.size >= 2
            and self.velocity_lag_lookup_throttle_breakpoints.size
            == self.velocity_lag_lookup_velocity_breakpoints.size
        )
        if recommended == "velocity_lag_lookup" and has_lookup:
            self.longitudinal_model = "velocity_lag_lookup"
        elif recommended in {"velocity_lag", "velocity_lag_lookup"}:
            self.longitudinal_model = "velocity_lag"
        else:
            self.longitudinal_model = "default"

        if self.longitudinal_model == "default":
            print(
                f"[WARN] Longitudinal profile '{profile}' was not usable; "
                "falling back to default direct-acceleration fake plant"
            )

    def _effective_velocity_lag_throttle(self, throttle: float) -> float:
        """Apply the calibrated throttle deadband used by velocity_lag."""
        deadband = float(self.velocity_lag_deadband)
        if deadband <= 0.0:
            return float(throttle)
        return float(np.sign(throttle) * max(abs(float(throttle)) - deadband, 0.0))

    def _velocity_lag_lookup_target(self, throttle: float) -> float:
        """Lookup steady-state velocity for a throttle command."""
        if (
            self.velocity_lag_lookup_throttle_breakpoints.size >= 2
            and self.velocity_lag_lookup_throttle_breakpoints.size
            == self.velocity_lag_lookup_velocity_breakpoints.size
        ):
            return float(
                np.interp(
                    float(throttle),
                    self.velocity_lag_lookup_throttle_breakpoints,
                    self.velocity_lag_lookup_velocity_breakpoints,
                )
            )
        return self.velocity_gain * self._effective_velocity_lag_throttle(throttle)

    def _calibrated_longitudinal_accel(
        self, throttle: float, current_vx: float, fallback_accel: float
    ) -> float:
        """Return calibrated longitudinal acceleration, or the legacy fallback."""
        if self.longitudinal_model == "velocity_lag":
            v_target = self.velocity_gain * self._effective_velocity_lag_throttle(throttle)
            accel = (v_target - float(current_vx)) / self.velocity_lag_tau
        elif self.longitudinal_model == "velocity_lag_lookup":
            v_target = self._velocity_lag_lookup_target(throttle)
            accel = (v_target - float(current_vx)) / self.velocity_lag_lookup_tau
        else:
            return float(fallback_accel)

        if abs(float(throttle)) < 0.01 and abs(float(current_vx)) < 0.03:
            return 0.0
        return float(np.clip(accel, -5.0, 5.0))

    def _resolve_steering_source_path(self, profile: str) -> Optional[Path]:
        """Resolve a named fake-plant steering profile to a YAML path."""
        qcar_dir = Path(__file__).resolve().parents[1]
        simulation_dir = Path(__file__).resolve().parent
        built_in_profiles = {
            "qlabs_steering": qcar_dir / "Calibration" / "results" / "03_steering_calibration" / "steering_calibration_qlabs.yaml",
            "steering_qlabs": qcar_dir / "Calibration" / "results" / "03_steering_calibration" / "steering_calibration_qlabs.yaml",
        }

        source_cfg = self.vehicle_conf.get("steering_model_source", {})
        profiles = source_cfg.get("profiles", {}) if isinstance(source_cfg, dict) else {}
        raw_path = None
        if isinstance(profiles, dict):
            profile_entry = profiles.get(profile)
            if isinstance(profile_entry, dict):
                raw_path = profile_entry.get("path", profile_entry.get("file"))
            elif profile_entry is not None:
                raw_path = profile_entry

        if raw_path is not None:
            path = Path(str(raw_path))
            if not path.is_absolute():
                path = simulation_dir / path
            return path

        if profile in built_in_profiles:
            return built_in_profiles[profile]

        if profile.lower().endswith((".yaml", ".yml")):
            path = Path(profile)
            if not path.is_absolute():
                path = simulation_dir / path
            return path

        return None

    @staticmethod
    def _normalized_steering_profile(value: Any) -> str:
        """Normalize fake-plant steering model aliases."""
        normalized = str(value or "default").strip().lower().replace("-", "_")
        aliases = {
            "": "default",
            "auto": "default",
            "legacy": "default",
            "direct": "default",
            "direct_angle": "default",
            "qlabs": "qlabs_steering",
            "qlab": "qlabs_steering",
            "qlabs_curvature": "qlabs_steering",
            "steering_qlabs": "qlabs_steering",
        }
        return aliases.get(normalized, normalized)

    def _load_steering_plant_model(self):
        """Load optional calibrated fake-plant steering-to-curvature behavior."""
        raw_profile = self.vehicle_conf.get(
            "steering_model",
            self.vehicle_conf.get("steering_model_profile", "default"),
        )
        source_cfg = self.vehicle_conf.get("steering_model_source", {})
        if (
            isinstance(source_cfg, dict)
            and self._normalized_steering_profile(raw_profile) == "default"
        ):
            raw_profile = source_cfg.get("profile", raw_profile)

        profile = self._normalized_steering_profile(raw_profile)
        self.steering_model_profile = profile
        self.steering_model = "default"
        self.steering_model_source_path = ""
        self.steering_curvature_poly = np.asarray([], dtype=float)
        self.steering_curvature_poly_by_throttle: Dict[float, np.ndarray] = {}

        if profile == "default":
            return

        source_path = self._resolve_steering_source_path(profile)
        loaded: Dict[str, Any] = {}
        if source_path is not None:
            try:
                loaded = self._load_yaml_dict(source_path)
                self.steering_model_source_path = str(source_path)
            except Exception as exc:
                print(f"[WARN] Could not load steering model '{source_path}': {exc}")

        vehicle_steering_cfg = self.vehicle_conf.get("steering_curvature_model")
        if isinstance(vehicle_steering_cfg, dict):
            loaded.update(vehicle_steering_cfg)

        coeffs = loaded.get("poly_coefficients_global", loaded.get("poly_coefficients", []))
        try:
            self.steering_curvature_poly = np.asarray(coeffs, dtype=float).reshape(-1)
        except Exception:
            self.steering_curvature_poly = np.asarray([], dtype=float)

        per_throttle = loaded.get("poly_coefficients_per_throttle", {})
        if isinstance(per_throttle, dict):
            for raw_key, raw_coeffs in per_throttle.items():
                try:
                    throttle_key = float(str(raw_key).split("_")[-1])
                    coeff_array = np.asarray(raw_coeffs, dtype=float).reshape(-1)
                except Exception:
                    continue
                if coeff_array.size >= 2:
                    self.steering_curvature_poly_by_throttle[throttle_key] = coeff_array

        if self.steering_curvature_poly.size >= 2 or self.steering_curvature_poly_by_throttle:
            self.steering_model = "curvature_poly"
        else:
            print(
                f"[WARN] Steering profile '{profile}' was not usable; "
                "falling back to default direct-angle fake plant"
            )

    def _steering_curvature_coefficients(self, throttle: float) -> np.ndarray:
        """Return global or nearest-throttle steering curvature polynomial."""
        if self.steering_curvature_poly_by_throttle:
            nearest = min(
                self.steering_curvature_poly_by_throttle.keys(),
                key=lambda key: abs(float(key) - float(throttle)),
            )
            return self.steering_curvature_poly_by_throttle[nearest]
        return self.steering_curvature_poly

    def _calibrated_steering_target_angle(
        self, steering_cmd: float, throttle_cmd: float
    ) -> Optional[float]:
        """Convert calibrated steering command to an equivalent model steering angle."""
        if self.steering_model != "curvature_poly":
            return None

        coeffs = self._steering_curvature_coefficients(throttle_cmd)
        if coeffs.size < 2:
            return None

        curvature = float(np.polyval(coeffs, float(steering_cmd)))
        wheelbase = max(float(self.params.a + self.params.b), 1e-6)
        target_angle = math.atan(wheelbase * curvature)
        max_steering_angle = (
            float(self.qlpv_obs_model.params["steering"]["max"])
            if "steering" in self.qlpv_obs_model.params
            else 0.5
        )
        return float(np.clip(target_angle, -max_steering_angle, max_steering_angle))

    def _process_control_inputs(
        self,
        throttle_cmd: float,
        steering_cmd: float,
        current_state_obs: np.ndarray,
        current_steering_angle: float,
        dt: float,
    ) -> Tuple[float, float, float]:
        """Process steering normally and optionally replace longitudinal acceleration."""
        acc, steer_rate, new_steer = self.qlpv_obs_model.process_control_inputs(
            throttle_cmd,
            steering_cmd,
            current_state_obs,
            current_steering_angle,
            dt,
        )
        current_vx = float(np.asarray(current_state_obs).reshape(-1)[0])
        acc = self._calibrated_longitudinal_accel(throttle_cmd, current_vx, acc)
        calibrated_target_angle = self._calibrated_steering_target_angle(
            steering_cmd, throttle_cmd
        )
        if calibrated_target_angle is not None:
            max_steering_rate = (
                float(self.qlpv_obs_model.params["steering"]["v_max"])
                if "steering" in self.qlpv_obs_model.params
                else 5.0
            )
            steering_error = calibrated_target_angle - float(current_steering_angle)
            steering_rate = float(np.clip(10.0 * steering_error, -max_steering_rate, max_steering_rate))
            steer_rate = steering_rate
            new_steer = float(current_steering_angle) + steering_rate * float(dt)
            if abs(calibrated_target_angle - new_steer) < 1e-4:
                new_steer = calibrated_target_angle
        return acc, steer_rate, new_steer


    def _init_states(self):
        """Initialize state vectors based on configuration."""
        # Common initial state
        x0 = float(self.config.get('initial_state', {}).get('x', self.car_id * 2.0))
        y0 = float(self.config.get('initial_state', {}).get('y', 0.0))
        psi0 = float(self.config.get('initial_state', {}).get('theta', 0.0))
        v0 = float(self.config.get('initial_state', {}).get('v', 0.0))

        # Kinematic State: [x, y, δ, v, ψ]
        self.state_ks = np.array([x0, y0, 0.0, v0, psi0])
        
        # Dynamic State: [x, y, δ, v, ψ, r, β]
        self.state_st = np.array([x0, y0, 0.0, v0, psi0, 0.0, 0.0])
        
        # qLPV Legacy State: [X, Y, δ, v_x, ψ, r, v_y]
        self.state_qlpv = np.array([x0, y0, 0.0, v0, psi0, 0.0, 0.0])
        
        # qLPV Matrix State: [v_x, v_y, ψ, r, X, Y]
        self.state_obs = np.array([v0, 0.0, psi0, 0.0, x0, y0])

    def reset_pose(self, pose, velocity: Optional[float] = None):
        """Reset all model states, public state, and GPS to the same pose."""
        pose = np.asarray(pose, dtype=float).reshape(-1)
        if pose.size < 3:
            raise ValueError("pose must contain [x, y, theta]")

        x0, y0, psi0 = float(pose[0]), float(pose[1]), float(pose[2])
        if velocity is None:
            velocity = float(self.config.get('initial_state', {}).get('v', 0.0))
        v0 = float(velocity)

        self.state_ks = np.array([x0, y0, 0.0, v0, psi0])
        self.state_st = np.array([x0, y0, 0.0, v0, psi0, 0.0, 0.0])
        self.state_qlpv = np.array([x0, y0, 0.0, v0, psi0, 0.0, 0.0])
        self.state_obs = np.array([v0, 0.0, psi0, 0.0, x0, y0])

        self._throttle = 0.0
        self._steering = 0.0
        self.control_input = np.array([0.0, 0.0])
        self.current_steering_angle = 0.0
        self.true_disturbances = [0.0, 0.0, 0.0]
        self.w_r_true = 0.0
        self.w_f_true = 0.0
        self.last_time = time.time()

        self._sync_state()
        self.motorTach = self.velocity
        self.gyroscope[2] = self.angular_velocity
        self.accelerometer = np.array([0.0, 0.0, 9.81])
        self.gps.update_true_state(self.x, self.y, self.heading)
        self.gps.last_data = np.array([self.x, self.y, self.heading])
        self.gps.last_update_time = time.time()
        self.gps.new_data_ready = True

    def write(self, throttle: float, steering: float):
        """Set control inputs."""
        self._throttle = np.clip(throttle, -1.0, 1.0)
        self._steering = np.clip(steering, -1.0, 1.0)

    def step(self, dt: float):
        """Perform one simulation step."""
        # 1. Update Physics
        if self.use_qlpv_matrix:
            self._update_physics_qlpv_matrix(dt)
        elif self.use_qlpv_legacy:
            self._update_physics_qlpv(dt)
        elif self.use_dynamic:
            self._update_physics_dynamic(dt) # Was _update_physics in old code
        else:
            self._update_physics_kinematic(dt) # Was _update_physics in old code

        # 2. Sync Properties
        self._sync_state()
        
        # 3. Update Sensors
        self.motorTach = self.velocity
        self.gyroscope[2] = self.angular_velocity
        # Update GPS true state
        self.gps.update_true_state(self.x, self.y, self.heading)

    def get_ground_truth(self) -> Dict[str, Any]:
        """Expose ground truth for observers."""
        return {
            'vx_true': self.velocity,
            'vy_true': self.lateral_velocity,
            'r_true': self.angular_velocity,
            'psi_true': self.heading,
            'X_true': self.x,
            'Y_true': self.y,
            'd_vx_true': self.true_disturbances[0],
            'd_vy_true': self.true_disturbances[1],
            'd_r_true': self.true_disturbances[2],
            'w_r_true': self.w_r_true,
            'w_r_true': self.w_r_true,
            'w_f_true': self.w_f_true
        }

    def get_true_state(self) -> np.ndarray:
        """
        Get ground truth state vector [v_x, v_y, ψ, r, X, Y]
        Expected by NeuralLuenbergerEstimator
        """
        # [v_x, v_y, ψ, r, X, Y]
        return np.array([
            self.velocity,
            self.lateral_velocity,
            self.heading,
            self.angular_velocity,
            self.x,
            self.y
        ])

    def get_true_residuals(self) -> np.ndarray:
        """Get ground truth tire residuals [w_r, w_f]"""
        return np.array([self.w_r_true, self.w_f_true])

    def get_true_disturbances(self) -> np.ndarray:
        """Get ground truth general disturbances [d_vx, d_vy, d_r]"""
        return np.array([
            self.true_disturbances[0],
            self.true_disturbances[1],
            self.true_disturbances[2]
        ])

    def get_tire_info(self) -> Dict[str, float]:
        """
        Get pre-calculated tire information dictionary for recorder.
        
        Returns the tire info computed during the last physics update step.
        Each update method (_update_physics_*) calls _calculate_tire_info() internally.
        
        Returns:
            Dict with keys: Fyr_true, Fyf_true, Fyr_linear, Fyf_linear, alpha_r, alpha_f
        """
        return self._tire_info.copy()
    
    def _calculate_tire_info(self, vx: float, vy: float, r: float, delta: float , Cf: float = None , Cr: float = None):
        """
        Calculate and store tire information (ground truth and residuals).
        
        Called at the end of each physics update method to compute:
        - Slip angles (alpha_f, alpha_r)
        - Linear tire forces (Fyf_linear, Fyr_linear)
        - True tire forces (Fyf_true, Fyr_true)
        - Tire residuals (w_f, w_r) stored in self.w_f_true, self.w_r_true
        
        Args:
            vx: Longitudinal velocity
            vy: Lateral velocity  
            r: Yaw rate
            delta: Steering angle
        """
        lf = self.params.a
        lr = self.params.b
        if Cf is None:
            Cf = getattr(self.params, 'Cf', 120.0)
        if Cr is None:
            Cr = getattr(self.params, 'Cr', 120.0)
        
        # Clamp vx to avoid division by zero
        vx_safe = max(abs(vx), 0.1)
        
        # Compute slip angles (using arctan for large angle safety)
        # alpha_f = delta - math.atan((vy + lf * r) / vx_safe)
        # alpha_r = -math.atan((vy - lr * r) / vx_safe) # equivalent to atan((lr*r - vy)/vx)

        alpha_f = delta - ((vy + lf * r) / vx_safe)
        alpha_r = -((vy - lr * r) / vx_safe) # equivalent to atan((lr*r - vy)/vx)


        # # Compute linear tire forces (in observer, we use linear tire forces)
        Fyf_linear, Fyr_linear = compute_tire_forces_linear(alpha_f, alpha_r, Cf, Cr)
        
        # Get true tire residuals based on tire model
        w_r, w_f, Fyf_true, Fyr_true = get_tire_residuals(
            self.tire_model, alpha_f, alpha_r, Cf, Cr, self.params, vx_safe)

        # print ("Cf, Cr: ", Cf, Cr)    
        # print ("alpha_f, alpha_r: ", alpha_f, alpha_r)
        # print ("Fyf_cl, Fyr_cl: ", Cf * alpha_f, Cr * alpha_r)
        # print( "Fyf_true: ", Fyf_true , "Fyf_linear: ", Fyf_linear)
        # # -------------------------------------------------------------------------
        
        # Store residuals for observer access
        self.w_r_true = w_r
        self.w_f_true = w_f
        
        # Store complete tire info
        self._tire_info = {
            'Fyr_true': Fyr_true,
            'Fyf_true': Fyf_true,
            'Fyr_linear': Fyr_linear,
            'Fyf_linear': Fyf_linear,
            'alpha_r': alpha_r,
            'alpha_f': alpha_f,
        }
        # print(self.tire_model)
        # print(f"Tire Info -Fyf_true: {Fyf_true:.2f}, Fyr_true: {Fyr_true:.2f}, w_f: {w_f:.2f}, w_r: {w_r:.2f}")

    def _sync_state(self):
        """Synchronize public properties."""
        if self.use_qlpv_legacy:
            self.x, self.y = self.state_qlpv[0], self.state_qlpv[1]
            self.heading = self.state_qlpv[4]
            self.velocity = self.state_qlpv[3]
            self.angular_velocity = self.state_qlpv[5]
            self.lateral_velocity = self.state_qlpv[6]
        elif self.use_qlpv_matrix:
            self.velocity = self.state_obs[0]
            self.lateral_velocity = self.state_obs[1]
            self.heading = self.state_obs[2]
            self.angular_velocity = self.state_obs[3]
            self.x, self.y = self.state_obs[4], self.state_obs[5]
        elif self.use_dynamic:
            self.x, self.y = self.state_st[0], self.state_st[1]
            self.heading = self.state_st[4]
            self.velocity = self.state_st[3]
            self.angular_velocity = self.state_st[5]
            self.lateral_velocity = self.velocity * math.sin(self.state_st[6])
        else:
            self.x, self.y = self.state_ks[0], self.state_ks[1]
            self.heading = self.state_ks[4]
            self.velocity = self.state_ks[3]
            if abs(self.velocity) > 0.01:
                wb = self.params.a + self.params.b
                self.angular_velocity = (self.velocity / wb) * math.tan(self.state_ks[2])
            else:
                self.angular_velocity = 0.0
            self.lateral_velocity = 0.0

        # Wrap heading
        self.heading = (self.heading + math.pi) % (2 * math.pi) - math.pi
        
    def _update_physics_qlpv_matrix(self, dt: float):
        """
        Matrix-based qLPV update with smooth kinematic↔qLPV blending.
        
        Three velocity regions:
          |v| < V_LOW  : pure kinematic (qLPV singular at 1/vx)
          V_LOW ≤ |v| ≤ V_HIGH : blend kinematic + qLPV (smooth transition)
          |v| > V_HIGH : pure qLPV
        """
        if not self.qlpv_obs_model: return
        
        V_LOW = 0.05    # Below: pure kinematic
        V_HIGH = 0.4    # Above: pure qLPV
        current_vx = abs(self.state_obs[0])
        
        # --- Blend factor: 0 = kinematic, 1 = qLPV ---
        if current_vx < V_LOW:
            alpha = 0.0
        elif current_vx > V_HIGH:
            alpha = 1.0
        else:
            # Smooth S-curve (smoothstep) instead of linear for C1 continuity
            t = (current_vx - V_LOW) / (V_HIGH - V_LOW)
            alpha = t * t * (3.0 - 2.0 * t)  # smoothstep
        
        # --- Pure kinematic (skip qLPV entirely) ---
        if alpha == 0.0:
            self._kinematic_fallback_for_qlpv(dt)
            return
        
        # --- Control inputs (shared by both models) ---
        acc, steer_rate, new_steer = self._process_control_inputs(
            self._throttle, self._steering, self.state_obs, self.current_steering_angle, dt
        )
        self.current_steering_angle = new_steer
        u_vec = np.array([new_steer, acc])  # [delta, a]
        

        # Determine disturbance vector based on mode
        d = self.disturbance_gen.get_disturbance(time.time())
        if self.disturbance_mode == 'general':
            if abs(self.velocity) < 0.05 and abs(acc) < 0.01:
                d = [0.0, 0.0, 0.0]
            d_vec = np.array(d).reshape(3, 1)
            self.true_disturbances = d
            self.w_r_true = 0.0
            self.w_f_true = 0.0
        else:
            # Tire residual mode: compute actual tire residuals and inject through E matrix.
            # The A/B matrices use the OBSERVER's Cf/Cr (120), so the linear dynamics match.
            # The nonlinear tire effects enter as residuals: w = Fy_true - Fy_linear(obs_Cf).
            self.true_disturbances = [0.0, 0.0, 0.0]
            # Residuals are computed per-substep below (after scheduling params are known)
            d_vec = None  # Will be computed inside the integration loop
        
        # Use the SAME Cf/Cr as the observer for the A/B matrices.
        # This ensures the linear dynamics match between truth and observer.
        # The tire nonlinearity enters through E_c @ d_vec (residuals).
        Cf = self.params.Cf 
        Cr = self.params.Cr 
        # # Adaptive sub-stepping for Euler stability
        # current_vx_safe = max(current_vx, 0.1)
        # max_eigenvalue_est = (Cf + Cr) / (min(self.params.m, self.params.I_z) * current_vx_safe)
        # dt_sub = min(0.001, 0.5 / max(max_eigenvalue_est, 1.0))
        # n_steps = max(1, int(dt / dt_sub))
        # dt_step = dt / n_steps

        # Sub-stepping
        dt_sub = 0.001
        n_steps = max(1, int(dt / dt_sub))
        dt_step = dt / n_steps
        
        # Save pre-integration state for blending
        state_before = self.state_obs.copy()
        
        # print(self.tire_model)
        x_dot_final = np.zeros(6)
        for _ in range(n_steps):
            rho = self.qlpv_obs_model.compute_scheduling_params(self.state_obs, u_vec[0])
            A_c = self.qlpv_obs_model.compute_A_matrix(rho, Cf=Cf, Cr=Cr)
            B_c = self.qlpv_obs_model.compute_B_matrix(rho, Cf=Cf, Cr=Cr)
            E_c = self.qlpv_obs_model.compute_E_matrix(rho)
            
            # Compute tire residuals per-substep for tire mode
            if d_vec is None:
                vx_sub = self.state_obs[0]
                vy_sub = self.state_obs[1]
                r_sub = self.state_obs[3]
                delta_sub = u_vec[0]
                vx_safe_sub = max(abs(vx_sub), 0.1)
                
                alpha_f_sub = delta_sub - (vy_sub + self.params.a * r_sub) / vx_safe_sub
                alpha_r_sub = -(vy_sub - self.params.b * r_sub) / vx_safe_sub
                
                # Residuals relative to observer's Cf/Cr (120):
                # w = Fy_true(linear) - Fy_linear(Cf=120)
                
                w_r_sub, w_f_sub, _, _ = get_tire_residuals(
                    self.tire_model, alpha_f_sub, alpha_r_sub, 
                    Cf, Cr, self.params, vx_safe_sub)
                
                d_vec_step = np.array([w_r_sub, w_f_sub]).reshape(2, 1)
            else:
                d_vec_step = d_vec
            
            x_dot = (A_c @ self.state_obs.reshape(-1, 1) +
                     B_c @ u_vec.reshape(-1, 1) +
                     E_c @ d_vec_step).flatten()
            x_dot_final = x_dot
            self.state_obs += x_dot * dt_step
            self.state_obs[2] = (self.state_obs[2] + np.pi) % (2 * np.pi) - np.pi
        
        state_qlpv = self.state_obs.copy()
        
        # --- If blending, also compute kinematic result and interpolate ---
        if alpha < 1.0:
            vx, vy, psi, r, X, Y = state_before
            vx_k, vy_k, psi_k, r_k, X_k, Y_k = self._compute_kinematic_step(
                dt, vx, vy, psi, r, X, Y, acc, new_steer, self._throttle
            )
            state_kin = np.array([vx_k, vy_k, psi_k, r_k, X_k, Y_k])
            
            # Blend: state = (1-α)*kinematic + α*qLPV
            self.state_obs = (1.0 - alpha) * state_kin + alpha * state_qlpv
            
            # Blend heading carefully (handle wrap-around)
            psi_diff = state_qlpv[2] - state_kin[2]
            if psi_diff > math.pi: psi_diff -= 2 * math.pi
            if psi_diff < -math.pi: psi_diff += 2 * math.pi
            self.state_obs[2] = state_kin[2] + alpha * psi_diff
            self.state_obs[2] = (self.state_obs[2] + math.pi) % (2 * math.pi) - math.pi
            
            # Blend derivatives for IMU (kinematic: ax=acc, ay=0)
            x_dot_final = alpha * x_dot_final  # qLPV part fades in
        
        # --- IMU (after integration) ---
        vx_dot = x_dot_final[0]
        vy_dot = x_dot_final[1]
        r = self.state_obs[3]
        vx = self.state_obs[0]
        vy = self.state_obs[1]
        
        self.accelerometer[0] = vx_dot - r * vy + (1.0 - alpha) * acc
        self.accelerometer[1] = vy_dot + r * vx
        self.accelerometer[2] = 9.81
        
        # --- Tire info (blended: fades from zero in kinematic to full in qLPV) ---
        self._calculate_tire_info(vx, vy, r, u_vec[0], Cf, Cr)
        # Scale tire outputs by blend factor (kinematic has no lateral forces)
        self.w_r_true *= alpha
        self.w_f_true *= alpha
        for key in self._tire_info:
            self._tire_info[key] *= alpha

    def _update_physics_qlpv(self, dt: float):
        """Legacy qLPV update."""
        # Mapping ...
        # qLPV state: [X, Y, δ, v_x, ψ, r, v_y]
        current_obs_state = np.array([
            self.state_qlpv[3], self.state_qlpv[6], self.state_qlpv[4],
            self.state_qlpv[5], self.state_qlpv[0], self.state_qlpv[1]
        ])
        
        acc, steer_rate, new_steer = self._process_control_inputs(
            self._throttle, self._steering, current_obs_state, self.state_qlpv[2], dt
        )
        self.current_steering_angle = new_steer
        control = np.array([steer_rate, acc])
        
        # Sub-stepping
        dt_sub = 0.001
        n_steps = max(1, int(dt / dt_sub))
        dt_step = dt / n_steps
        
        d = self.disturbance_gen.get_disturbance(time.time())
        # Zero out additive disturbances if in 'tire' mode
        # Apply checks: only apply when command accel is active AND car is moving
        if self.disturbance_mode != 'general' or (abs(self.velocity) < 0.05 and abs(acc) < 0.01):
            d = [0.0, 0.0, 0.0]
        self.true_disturbances = d
        
        derivs = np.zeros(7)
        for _ in range(n_steps):
            derivs = vehicle_dynamics_qlpv(
                self.state_qlpv, control, self.params, 
                tire_mode=self.tire_model, disturbances=d
            )
            self.state_qlpv += derivs * dt_step
        
        # Update IMU
        # derivs: [X_dot, Y_dot, delta_dot, vx_dot, psi_dot, r_dot, vy_dot]
        vx_dot = derivs[3]
        vy_dot = derivs[6]
        vx = self.state_qlpv[3]
        r = self.state_qlpv[5]
        vy = self.state_qlpv[6]
        
        self.accelerometer[0] = vx_dot - r * vy
        self.accelerometer[1] = vy_dot + r * vx
        self.accelerometer[2] = 9.81
        
        # Calculate tire info at end of update
        delta = self.state_qlpv[2]
        self._calculate_tire_info(vx, vy, r, delta)
        

        
    def _update_physics_dynamic(self, dt: float):
        """
        Standard Dynamic ST model update. 
        Matches flow of _update_physics_qlpv_matrix: sub-stepping + shared control logic.
        """
        # 1. Process Control Inputs (Shared Logic)
        current_st_obs = np.array([
            self.state_st[3], 0.0, 0.0, 0.0, 0.0, 0.0
        ])
        
        acc, steer_rate, new_steer = self._process_control_inputs(
            self._throttle, self._steering, current_st_obs, self.state_st[2], dt
        )
        self.current_steering_angle = new_steer
        
        self.control_input[0] = steer_rate
        self.control_input[1] = acc
        
        # 2. Sub-stepping
        dt_sub = 0.001
        n_steps = max(1, int(dt / dt_sub))
        dt_step = dt / n_steps
        
        # 3. Disturbance Injection
        d = self.disturbance_gen.get_disturbance(time.time())
        if self.disturbance_mode == 'general':
            if abs(self.state_st[3]) < 0.05 and abs(acc) < 0.01:
                d = [0.0, 0.0, 0.0]
            self.true_disturbances = d
            self.w_r_true = 0.0
            self.w_f_true = 0.0
        else:
            d_tire = [0.0, 0.0]
            self.true_disturbances = [0.0, 0.0, 0.0]
            self.w_r_true = d_tire[0]
            self.w_f_true = d_tire[1]
            d = [0.0, 0.0, 0.0]
            
        # 4. Integration Loop
        derivatives = None
        for _ in range(n_steps):
            try:
                derivatives = np.array(vehicle_dynamics_st(self.state_st, self.control_input, self.params))
                
                if any(np.isnan(derivatives)) or any(np.isinf(derivatives)):
                    self._simple_kinematic_update(dt)
                    return

                # Apply disturbances
                # derivatives: [x_dot, y_dot, delta_dot, v_dot, psi_dot, r_dot, beta_dot]
                derivatives[3] += d[0] # v_dot
                derivatives[5] += d[2] # r_dot
                if abs(self.state_st[3]) > 0.1:
                    derivatives[6] += d[1] / self.state_st[3] # beta_dot approx

                # Euler integration
                self.state_st += derivatives * dt_step # State update
                
                # Safety clamps
                v_max = float(self.params.longitudinal.v_max)
                v_min = float(self.params.longitudinal.v_min)
                self.state_st[3] = np.clip(self.state_st[3], v_min, v_max)
                self.state_st[5] = np.clip(self.state_st[5], -5.0, 5.0) # r
                self.state_st[6] = np.clip(self.state_st[6], -0.5, 0.5) # beta
                
            except Exception as e:
                print(f"⚠️ Dynamic model error: {e}, falling back to kinematic")
                self._simple_kinematic_update(dt)
                return
                
        # 5. Update IMU
        if derivatives is not None:
            self._update_imu_sensors(derivatives, acc)
        
        # 6. Calculate tire info
        vx = max(abs(self.state_st[3]), 0.5)
        beta = self.state_st[6]
        vy = vx * math.tan(beta) if abs(beta) < 0.5 else vx * beta
        r = self.state_st[5]
        delta = self.state_st[2]
        self._calculate_tire_info(vx, vy, r, delta)

    def _update_physics_kinematic(self, dt: float):
        """
        Kinematic model update.
        Matches flow of _update_physics_qlpv_matrix.
        """
        # 1. Control Inputs
        current_ks_obs = np.array([
            self.state_ks[3], 0.0, 0.0, 0.0, 0.0, 0.0
        ])
        
        acc, steer_rate, new_steer = self._process_control_inputs(
            self._throttle, self._steering, current_ks_obs, self.state_ks[2], dt
        )
        self.current_steering_angle = new_steer
        
        self.control_input = np.array([steer_rate, acc])
        
        # 2. Sub-stepping
        dt_sub = 0.001
        n_steps = max(1, int(dt / dt_sub))
        dt_step = dt / n_steps
        
        # 3. Disturbance
        d = self.disturbance_gen.get_disturbance(time.time())
        if self.disturbance_mode == 'general':
            if abs(self.state_ks[3]) < 0.05 and abs(acc) < 0.01:
                d = [0.0, 0.0, 0.0]
            self.true_disturbances = d
            self.w_r_true = 0.0
            self.w_f_true = 0.0
        else:
            d = [0.0, 0.0, 0.0]
            self.true_disturbances = [0.0, 0.0, 0.0]
            self.w_r_true = 0.0
            self.w_f_true = 0.0

        # 4. Loop
        derivatives = None
        for _ in range(n_steps):
            derivatives = vehicle_dynamics_ks(self.state_ks, self.control_input, self.params)
            
            # Apply disturbances
            # KS derivs: [x_dot, y_dot, delta_dot, v_dot, psi_dot]
            derivatives[3] += d[0] # v_dot
            derivatives[4] += d[2] # psi_dot (approx yaw rate disturbance)
            
            for i in range(len(self.state_ks)):
                self.state_ks[i] += derivatives[i] * dt_step
                
            # Safety clamps
            v_max = float(self.params.longitudinal.v_max)
            v_min = float(self.params.longitudinal.v_min)
            self.state_ks[3] = np.clip(self.state_ks[3], v_min, v_max)
            
        # 5. IMU
        if derivatives is not None:
            self._update_imu_sensors(derivatives, acc)
        
        # # 6. Kinematic model: set tire info to zeros (no lateral dynamics)
        # self._tire_info = {
        #     'Fyr_true': 0.0,
        #     'Fyf_true': 0.0,
        #     'Fyr_linear': 0.0,
        #     'Fyf_linear': 0.0,
        #     'alpha_r': 0.0,
        #     'alpha_f': 0.0,
        # }
        # self.w_r_true = 0.0
        # self.w_f_true = 0.0

    def _simple_kinematic_update(self, dt: float):
        """Fallback simple kinematic update when dynamic model fails"""
        max_velocity = 3.0
        target_velocity = self._throttle * max_velocity
        
        # Apply friction and braking
        friction_coeff = 0.8
        if abs(self.velocity) > 0.01:
            friction_decel = -np.sign(self.velocity) * friction_coeff * dt
            self.velocity += friction_decel
        
        # Active braking
        if abs(self._throttle) < 0.01 and abs(self.velocity) > 0.01:
            brake_decel = -np.sign(self.velocity) * 4.0 * dt
            self.velocity += brake_decel
            
        if abs(self.velocity) < 0.05 and abs(self._throttle) < 0.01:
            self.velocity = 0.0
        else:
            velocity_error = target_velocity - self.velocity
            acceleration = 2.0 * velocity_error
            self.velocity += acceleration * dt
            
        self.velocity = np.clip(self.velocity, -max_velocity, max_velocity)
        
        self.x += self.velocity * math.cos(self.heading) * dt
        self.y += self.velocity * math.sin(self.heading) * dt
        
        if abs(self.velocity) > 0.1:
            wheelbase = 0.3
            self.angular_velocity = (self.velocity / wheelbase) * math.tan(self._steering * 0.3)
            self.heading += self.angular_velocity * dt
        else:
            self.angular_velocity = 0.0

    def _compute_kinematic_step(self, dt: float, 
                               vx: float, vy: float, psi: float, r: float, 
                               X: float, Y: float, 
                               acc: float, delta: float, 
                               throttle_input: float) -> Tuple[float, float, float, float, float, float]:
        """
        Common kinematic step: velocity → yaw → heading → position.
        
        When throttle is released, velocity decays exponentially to zero (smooth stop).
        When throttle is active, normal acceleration is applied.
        
        Returns: (vx_new, vy_new, psi_new, r_new, X_new, Y_new)
        """
        STOP_THRESHOLD = 0.05  # m/s — below this, snap to zero
        BRAKE_RATE = 20.0      # 1/s — exponential decay rate when no throttle
        
        # 1. Velocity: smooth braking when no throttle, otherwise integrate acceleration
        if self.longitudinal_model in {"velocity_lag", "velocity_lag_lookup"}:
            vx_new = vx + acc * dt
            if abs(throttle_input) < 0.01 and abs(vx_new) < STOP_THRESHOLD:
                vx_new = 0.0
        elif abs(throttle_input) < 0.01:
            # Exponential decay: vx * e^(-k*dt), snaps to 0 below threshold
            vx_new = vx * math.exp(-BRAKE_RATE * dt)
            if abs(vx_new) < STOP_THRESHOLD:
                vx_new = 0.0
        else:
            vx_new = vx + acc * dt
        
        # Clamp to vehicle limits
        v_max = float(self.params.longitudinal.v_max)
        v_min = float(self.params.longitudinal.v_min)
        vx_new = float(np.clip(vx_new, v_min, v_max))
        
        # 2. Kinematic yaw rate: r = v/L * tan(δ), zero when nearly stopped
        wheelbase = self.params.a + self.params.b
        r_new = (vx_new / wheelbase) * math.tan(delta) if abs(vx_new) > STOP_THRESHOLD else 0.0
        
        # 3. Heading & Position (midpoint integration for accuracy)
        psi_new = psi + r_new * dt
        psi_new = (psi_new + math.pi) % (2 * math.pi) - math.pi
        
        psi_mid = 0.5 * (psi + psi_new)
        X_new = X + vx_new * math.cos(psi_mid) * dt
        Y_new = Y + vx_new * math.sin(psi_mid) * dt
        
        # 4. Lateral velocity decay (kinematic = no sideslip)
        vy_new = vy * math.exp(-5.0 * dt) if abs(vy) > 0.01 else 0.0
            
        return vx_new, vy_new, psi_new, r_new, X_new, Y_new

    def _kinematic_fallback_for_qlpv(self, dt: float):
        """
        Kinematic fallback for qLPV matrix mode at low speeds.
        Uses common _compute_kinematic_step.
        """
        # 1. Control Inputs
        acc, steer_rate, new_steer = self._process_control_inputs(
            self._throttle, self._steering, self.state_obs, self.current_steering_angle, dt
        )
        self.current_steering_angle = new_steer
        delta = new_steer
        
        # 2. Current State
        vx, vy, psi, r, X, Y = self.state_obs
        
        # 3. Compute Common Kinematic Step
        vx_new, vy_new, psi_new, r_new, X_new, Y_new = self._compute_kinematic_step(
            dt, vx, vy, psi, r, X, Y, acc, delta, self._throttle
        )
        
        # 4. Update State [vx, vy, ψ, r, X, Y]
        self.state_obs = np.array([vx_new, vy_new, psi_new, r_new, X_new, Y_new])
        
        # 5. Set disturbances to zero in kinematic mode
        self.true_disturbances = [0.0, 0.0, 0.0]
        self.w_r_true = 0.0
        self.w_f_true = 0.0
        
        # 6. Update IMU (simple kinematic model)
        self.accelerometer[0] = acc  # ax ≈ commanded acceleration
        self.accelerometer[1] = 0.0  # ay ≈ 0 in kinematic
        self.accelerometer[2] = 9.81
        
        # 7. Zero tire info in kinematic mode (no lateral dynamics)
        self._tire_info = {
            'Fyr_true': 0.0,
            'Fyf_true': 0.0,
            'Fyr_linear': 0.0,
            'Fyf_linear': 0.0,
            'alpha_r': 0.0,
            'alpha_f': 0.0,
        }

    def _update_imu_sensors(self, derivatives: np.ndarray, commanded_accel: float):
        """Compute realistic IMU sensor readings."""
        g = 9.81
        vx = self.velocity
        vy = self.lateral_velocity
        r = self.angular_velocity
        
        if self.use_qlpv_legacy or self.use_qlpv_matrix:
            # qLPV derivatives: [Ẋ, Ẏ, δ̇, v̇_x, ψ̇, ṙ, v̇_y] -> indices depend on vector
            # This helper is mostly for ST/KS models here. 
            # For qLPV, derivatives logic in those functions handles IMU usually?
            # In mock_vehicle, qLPV updates don't call this yet.
            # We assume this is called by dynamic/kinematic.
            return

        if self.use_dynamic:
            # ST derivatives: [Ẋ, Ẏ, δ̇, v̇, ψ̇, ṙ, β̇]
            v_dot = derivatives[3]
            beta_dot = derivatives[6]
            vx_dot = v_dot 
            vy_dot = self.velocity * beta_dot 
        else:
            # KS derivatives: [Ẋ, Ẏ, δ̇, v̇, ψ̇]
            vx_dot = derivatives[3] if len(derivatives) > 3 else commanded_accel
            vy_dot = 0.0
            
        self.accelerometer[0] = float(vx_dot - r * vy)
        self.accelerometer[1] = float(vy_dot + r * vx)
        self.accelerometer[2] = g
        self.gyroscope[2] = float(r)

        
    def read(self):
        """Simulate reading sensors and update physics"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Limit dt to prevent instability from CPU pauses
        dt = min(max(dt, 0.001), 0.1)
        
        self.step(dt)
        
    
    
    def write(self, throttle: float, steering: float):
        """Receive control commands"""
        self._throttle = np.clip(throttle, -1.0, 1.0)
        self._steering = np.clip(steering, -1.0, 1.0)
    
    def read_write_std(self, throttle: float, steering: float, LEDs=None):
        """Receive control commands"""
        self._throttle = np.clip(throttle, -1.0, 1.0)
        self._steering = np.clip(steering, -1.0, 1.0)
    
