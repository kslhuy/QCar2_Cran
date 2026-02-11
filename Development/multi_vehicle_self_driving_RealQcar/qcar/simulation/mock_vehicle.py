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
        print(f"Tire Model: {self.tire_model}")
        print(f"Disturbance Mode: {self.disturbance_mode}")
        print("="*70)

    def _load_parameters(self):
        """Load vehicle parameters."""
        params_file = self.vehicle_conf.get('params_file', 'qcar')
        try:
            if params_file == 'qcar':
                # Load custom QCar parameters
                # vehiclemodels is now in the same directory as this file (qcar/simulation/vehiclemodels)
                base_dir = Path(__file__).parent / "vehiclemodels" / "parameters"
                qcar_conf = OmegaConf.load(str(base_dir / "parameters_qcar.yaml"))
                tire_conf = OmegaConf.load(str(base_dir / "parameters_tire.yaml"))
                structured_conf = OmegaConf.structured(VehicleParameters)
                self.params = OmegaConf.to_object(OmegaConf.merge(structured_conf, tire_conf, qcar_conf))
            elif params_file.startswith('vehicle'):
                vid = int(params_file.replace('vehicle', ''))
                self.params = setup_vehicle_parameters(vehicle_id=vid)
            else:
                self.params = setup_vehicle_parameters(vehicle_id=1) # Fallback
        except Exception as e:
            print(f"Error loading parameters: {e}. Using default.")
            self.params = setup_vehicle_parameters(vehicle_id=1)


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
        acc, steer_rate, new_steer = self.qlpv_obs_model.process_control_inputs(
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
        Cf = 120
        Cr = 120

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
        
        acc, steer_rate, new_steer = self.qlpv_obs_model.process_control_inputs(
            self._throttle, self._steering, current_obs_state, self.state_qlpv[2], dt
        )
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
        
        acc, steer_rate, new_steer = self.qlpv_obs_model.process_control_inputs(
            self._throttle, self._steering, current_st_obs, self.state_st[2], dt
        )
        
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
        
        acc, steer_rate, new_steer = self.qlpv_obs_model.process_control_inputs(
            self._throttle, self._steering, current_ks_obs, self.state_ks[2], dt
        )
        
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
        if abs(throttle_input) < 0.01:
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
        acc, steer_rate, new_steer = self.qlpv_obs_model.process_control_inputs(
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
    
