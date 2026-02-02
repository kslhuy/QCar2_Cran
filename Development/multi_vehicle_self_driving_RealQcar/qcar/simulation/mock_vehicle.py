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
    get_lateral_acceleration
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

    def get_observer_state(self) -> np.ndarray:
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
        """Matrix-based qLPV update."""
        if not self.qlpv_obs_model: return
        
        current_obs_state = self.state_obs
        acc, steer_rate, new_steer = self.qlpv_obs_model.process_control_inputs(
            self._throttle, self._steering, current_obs_state, self.current_steering_angle, dt
        )
        self.current_steering_angle = new_steer
        u_vec = np.array([new_steer, acc]) # [delta, a]
        
        # Sub-stepping
        dt_sub = 0.001
        n_steps = max(1, int(dt / dt_sub))
        dt_step = dt / n_steps
        
        # Determine disturbance vector based on mode
        d = self.disturbance_gen.get_disturbance(time.time())
        if self.disturbance_mode == 'general':
            # Check conditions for general disturbance (less aggressive gating)
            if abs(self.velocity) < 0.05 and abs(acc) < 0.01:
                d = [0.0, 0.0, 0.0]
            # d is already 3D for general mode
            d_vec = np.array(d).reshape(3, 1)
            self.true_disturbances = d
            self.w_r_true = 0.0
            self.w_f_true = 0.0
        else:
            # Tire mode: we don't inject external tire residuals in this simulation loop currently
            # Default to zero residuals (perfect model)
            d_tire = [0.0, 0.0] 
            d_vec = np.array(d_tire).reshape(2, 1)
            
            self.true_disturbances = [0.0, 0.0, 0.0]
            self.w_r_true = d_tire[0]
            self.w_f_true = d_tire[1]
        
        x_dot_final = np.zeros(6) # To store last derivative for IMU

        for _ in range(n_steps):
            rho = self.qlpv_obs_model.compute_scheduling_params(self.state_obs, u_vec[0])
            # A(rho), B(rho), E(rho) are continuous
            A_c = self.qlpv_obs_model.compute_A_matrix(rho)
            B_c = self.qlpv_obs_model.compute_B_matrix(rho) 
            E_c = self.qlpv_obs_model.compute_E_matrix(rho)
            
            # x_dot = A x + B u + E d
            x_dot = (A_c @ self.state_obs.reshape(-1,1) + 
                     B_c @ u_vec.reshape(-1,1) + 
                     E_c @ d_vec).flatten()
            
            x_dot_final = x_dot # Save for IMU

            # Disturbances are now included via E_c @ d_vec
            # No manual addition needed
            # state_obs is [v_x, v_y, ψ, r, X, Y].
            # Indices: 0:vx, 1:vy, 2:psi, 3:r.
            
            self.state_obs += x_dot * dt_step
            
            # Wrap yaw
            self.state_obs[2] = (self.state_obs[2] + np.pi) % (2*np.pi) - np.pi
        
        # Update IMU (Accelerometer)
        # x_dot = [v̇_x, v̇_y, ψ̇, ṙ, Ẋ, Ẏ]
        # a_x = v̇_x - r·v_y
        # a_y = v̇_y + r·v_x
        vx_dot = x_dot_final[0]
        vy_dot = x_dot_final[1]
        r = self.state_obs[3]
        vx = self.state_obs[0]
        vy = self.state_obs[1]
        
        self.accelerometer[0] = vx_dot - r * vy
        self.accelerometer[1] = vy_dot + r * vx
        self.accelerometer[2] = 9.81

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
    
