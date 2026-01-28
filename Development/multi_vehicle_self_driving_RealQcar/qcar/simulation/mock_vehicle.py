import time
import math
import numpy as np
from pathlib import Path
from typing import Dict, Any, Optional, Tuple
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
        
        # Load Parameters
        self._load_parameters()
        
        # Initialize Components
        self.disturbance_gen = DisturbanceGenerator(config.get('disturbances'))
        self.gps = MockQCarGPS(initial_pose=[0.0, 0.0, 0.0]) # Will be updated
        
        # Initialize qLPV Observer Model if needed
        self.qlpv_obs_model = None
        self._init_qlpv_obs_model()
        
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
        
        print(f"🔧 MockQCar {self.car_id}: Initialized with {self.model_type} model")

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

    def _init_qlpv_obs_model(self):
        """Initialize centralized qLPV model helper."""
        # Convert params to dict for QLPV model
        params_dict = {}
        try:
            # Basic conversion
            params_dict = {
                'steering': {'max': getattr(self.params.steering, 'max', 0.5), 
                             'v_max': getattr(self.params.steering, 'v_max', 5.0)},
                'longitudinal': {'a_max': getattr(self.params.longitudinal, 'a_max', 3.0),
                                 'v_max': getattr(self.params.longitudinal, 'v_max', 2.0),
                                 'v_min': getattr(self.params.longitudinal, 'v_min', -2.0)},
                # Physical params
                'a': getattr(self.params, 'a', 0.16),
                'b': getattr(self.params, 'b', 0.16),
                'm': getattr(self.params, 'm', 2.5),
                'lf': getattr(self.params, 'a', 0.16),
                'lr': getattr(self.params, 'b', 0.16),
                'Iz': getattr(self.params, 'I_z', 0.05),
                'Cf': getattr(self.params, 'Cf', 100),
                'Cr': getattr(self.params, 'Cr', 100),
            }
        except:
            pass
            
        self.qlpv_obs_model = QLPVVehicleDynamicsObs(vehicle_params=params_dict)

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
        
        # Get disturbances
        d = self.disturbance_gen.get_disturbance(time.time())
        self.true_disturbances = d # Store [d_vx, d_vy, d_r]
        d_vec = np.array(d).reshape(3, 1) # 3x1 vector
        
        for _ in range(n_steps):
            rho = self.qlpv_obs_model.compute_scheduling_params(self.state_obs, u_vec[0])
            # A(rho), B(rho) are continuous
            A_c = self.qlpv_obs_model.compute_A_matrix(rho)
            B_c = self.qlpv_obs_model.compute_B_matrix(rho) 
            
            # x_dot = A x + B u + E d
            # E map for disturbances: Identity to first 3 states (vx, vy, r)?
            # Assuming simple additive disturbances on derivatives
            # State: [v_x, v_y, ψ, r, X, Y]
            # dist: [d_vx, d_vy, d_r] -> add to index 0, 1, 3 (r)
            
            x_dot = A_c @ self.state_obs.reshape(-1,1) + B_c @ u_vec.reshape(-1,1)
            x_dot = x_dot.flatten()
            
            # Add disturbances
            x_dot[0] += d[0] # vx
            x_dot[1] += d[1] # vy
            x_dot[3] += d[2] # r (index 3 is r in this state vector)
            # state_obs is [v_x, v_y, ψ, r, X, Y].
            # Indices: 0:vx, 1:vy, 2:psi, 3:r.
            
            self.state_obs += x_dot * dt_step
            
            # Wrap yaw
            self.state_obs[2] = (self.state_obs[2] + np.pi) % (2*np.pi) - np.pi

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
        self.true_disturbances = d
        
        for _ in range(n_steps):
            derivs = vehicle_dynamics_qlpv(
                self.state_qlpv, control, self.params, 
                tire_mode=self.tire_model, disturbances=d
            )
            self.state_qlpv += derivs * dt_step
        

        
    def _update_physics_dynamic(self, dt: float):
        """Standard Dynamic ST model update."""
        # [x, y, δ, v, ψ, r, β]
        control_in = np.array([0.0, 0.0]) # [steer_rate, acc] needs computation
        # ... Simplified: just assume we track inputs
        # But vehicle_dynamics_st expects [steering_rate, acceleration]
        # We need the controller helper
        current_obs_state = np.array([
            self.state_st[3], self.state_st[3]*math.sin(self.state_st[6]),
            self.state_st[4], self.state_st[5], self.state_st[0], self.state_st[1]
        ])
        acc, steer_rate, _ = self.qlpv_obs_model.process_control_inputs(
            self._throttle, self._steering, current_obs_state, self.state_st[2], dt
        )
        control_in = np.array([steer_rate, acc])
        
        derivs = np.array(vehicle_dynamics_st(self.state_st, control_in, self.params))
        
        # Add disturbances? ST model doesn't support them natively usually.
        # We can add them to derivatives of v, r, beta?
        # d_vx -> add to v dot? d_r -> add to r dot?
        d = self.disturbance_gen.get_disturbance(time.time())
        self.true_disturbances = d
        
        # indices: 3:v, 5:r, 6:beta
        # v dot += d_vx (approx)
        derivs[3] += d[0]
        derivs[5] += d[2]
        # d_vy affects beta dot ~ d_vy / v
        if abs(self.state_st[3]) > 0.1:
            derivs[6] += d[1] / self.state_st[3]
            
        self.state_st += derivs * dt
        
    def _update_physics_kinematic(self, dt: float):
        """Kinematic update."""
        # [x, y, δ, v, ψ]
        # Simply set v and delta based on inputs? No, we integrate.
        
        # Use simple bicycle model logic from legacy code
        # ... kept simple for brevity
        max_v = 3.0
        target_v = self._throttle * max_v
        
        # Simple P control for velocity
        acc = 2.0 * (target_v - self.state_ks[3])
        
        # Steering is direct?
        self.state_ks[2] = self._steering * 0.5 # Max steer
        
        d = self.disturbance_gen.get_disturbance(time.time())
        self.true_disturbances = d
        acc += d[0] # additive vx disturbance
        
        self.state_ks[3] += acc * dt
        
        v = self.state_ks[3]
        delta = self.state_ks[2]
        psi = self.state_ks[4]
        wb = 0.3
        
        beta = math.atan(0.5 * math.tan(delta))
        
        x_dot = v * math.cos(psi + beta) + d[1] # additive vy disturbance effect?
        y_dot = v * math.sin(psi + beta)
        psi_dot = (v / wb) * math.cos(beta) * math.tan(delta) + d[2]
        
        self.state_ks[0] += x_dot * dt
        self.state_ks[1] += y_dot * dt  
        self.state_ks[4] += psi_dot * dt
        
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
    