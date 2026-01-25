"""
Fake Vehicle using REAL VehicleLogic Class - SIMPLIFIED
This creates a fake vehicle that uses the actual VehicleLogic class from vehicle_logic.py
with mock hardware components. Only the INITIALIZING state is replaced with a fake version.


Vehicle Models Available:
# Vehicle Models Available:
# - Kinematic Single-Track (ID 0): Simpler, faster, 5-state model
# - Single-Track Dynamic (ID 1): More realistic, includes tire dynamics, 7-state model
# - qLPV Legacy (ID 2): Matches observer dynamics exactly, 7-state model

Vehicle Parameter Sets:
- qcar (default): Quanser QCar 1:10 scale vehicle (0.38m, 1.5kg, v_max=2.0m/s)
- vehicle1: Ford Escort (4.3m, 1226kg, v_max=45.8m/s)
- vehicle2: BMW 320i (4.5m, 1093kg, v_max=50.8m/s)
- vehicle3: VW Vanagon (4.6m, 1587kg, v_max=41.7m/s)
- vehicle4: Truck with trailer (16.5m, v_max=22.2m/s)

Usage:
    python fake_vehicle_real_logic.py [car_id] [host_ip] [base_port] [use_dynamic] [vehicle_params]
    
Examples:
    python fake_vehicle_real_logic.py                              # Car 0, QCar params, kinematic
    python fake_vehicle_real_logic.py 1                            # Car 1, QCar params, kinematic
    python fake_vehicle_real_logic.py 0 127.0.0.1 5000 false qcar  # Car 0, QCar params, kinematic
    python fake_vehicle_real_logic.py 0 127.0.0.1 5000 true        # Car 0, QCar params, dynamic
    python fake_vehicle_real_logic.py 2 127.0.0.1 5000 false vehicle1  # Car 2, Ford Escort params
    python fake_vehicle_real_logic.py 0 127.0.0.1 5000 true vehicle2   # Car 0, BMW params, dynamic
"""
import sys
import os
import time
import socket
import json
import threading
import math
import random
import numpy as np
from typing import Dict, Any, Optional
from threading import Event

# Add parent directory to path to import qcar modules
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Import the REAL VehicleLogic and related classes
from vehicle_logic import VehicleLogic
from config_main import VehicleMainConfig
from StateMachine.vehicle_state_machine import VehicleStateMachine
from StateMachine.vehicle_state import VehicleState
from fake_initializing_state import FakeInitializingState

# Import vehicle dynamics models
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from vehiclemodels.vehicle_parameters import setup_vehicle_parameters
from pathlib import Path
from omegaconf import OmegaConf
from vehiclemodels.vehicle_parameters import VehicleParameters

# Import qLPV vehicle dynamics for observer testing
from vehiclemodels.vehicle_dynamics_qlpv import (
    vehicle_dynamics_qlpv, 
    get_tire_residuals,
    get_lateral_acceleration,
    compute_tire_forces_linear,
)

# Removed FakeVehicleStateMachine class - using real VehicleStateMachine instead
# We only need to replace the INITIALIZING state with fake version


class MockQCar:
    """Mock QCar hardware using proper vehicle dynamics from vehiclemodels folder
    
    Supports three dynamics models:
    - Kinematic Single-Track (default): Simpler, faster, 5-state model
    - Single-Track Dynamic: More realistic, includes tire dynamics, 7-state model
    - qLPV Model (for observer testing): Matches observer dynamics exactly, 7-state model
    """
    
    def __init__(self, car_id: int, dynamic_model_type: int = 0, 
                 vehicle_params: str = 'qcar'):
        self.car_id = car_id
        self.dynamic_model_type = dynamic_model_type  # 0=Kinematic, 1=Dynamic, 2=qLPV
        self.use_qlpv_model = (dynamic_model_type == 2)
        self.use_dynamic_model = (dynamic_model_type == 1)
        
        # Load vehicle parameters based on specified type
        # Options: 'qcar' (default), 'vehicle1' (Ford Escort), 'vehicle2' (BMW), 
        #          'vehicle3' (VW Vanagon), 'vehicle4' (Truck)
        if vehicle_params == 'qcar':
            # Load custom QCar parameters
            try:
                params_dir = Path(__file__).parent / "vehiclemodels" / "parameters"
                qcar_conf = OmegaConf.load(str(params_dir / "parameters_qcar.yaml"))
                tire_conf = OmegaConf.load(str(params_dir / "parameters_tire.yaml"))
                structured_conf = OmegaConf.structured(VehicleParameters)
                self.params = OmegaConf.to_object(OmegaConf.merge(structured_conf, qcar_conf, tire_conf))
            except Exception as e:
                print(f"⚠️  Warning: Could not load QCar parameters ({e}), using vehicle1 as fallback")
                # Fallback to vehicle1 if QCar params can't load
                self.params = setup_vehicle_parameters(vehicle_id=1)
        elif vehicle_params.startswith('vehicle'):
            # Load standard vehicle parameters (vehicle1-4)
            vehicle_id = int(vehicle_params.replace('vehicle', ''))
            self.params = setup_vehicle_parameters(vehicle_id=vehicle_id)


        # # Load parameters
        # # Officially use 'qcar' which now loads parameters_qcar.yaml via setup_vehicle_parameters
        # if vehicle_params:
        #     self.params = setup_vehicle_parameters(vehicle_id=vehicle_params)
        # else:
        #     self.params = setup_vehicle_parameters(vehicle_id='qcar')
        
        # Mock sensor data
        self.motorTach = 0.0
        self.gyroscope = np.array([0.0, 0.0, 0.0])
        self.accelerometer = np.array([0.0, 0.0, 9.81])  # NEW: Accelerometer [ax, ay, az]
        self.battery = np.array([12.0])  # Mock battery voltage
        
        # Mock actuator commands (normalized -1 to 1)
        self._throttle = 0.0
        self._steering = 0.0
        
        # Vehicle state vector for kinematic single-track model (5 states)
        # [x, y, steering_angle, velocity, yaw_angle]
        self.state_ks = np.array([
            car_id * 2.0,  # x position
            0.0,           # y position
            0.0,           # steering angle (rad)
            0.0,           # velocity (m/s)
            0.0            # yaw angle (rad)
        ])
        
        # Vehicle state vector for single-track dynamic model (7 states)
        # [x, y, steering_angle, velocity, yaw_angle, yaw_rate, slip_angle]
        self.state_st = np.array([
            car_id * 2.0,  # x position
            0.0,           # y position
            0.0,           # steering angle (rad)
            0.0,           # velocity (m/s)
            0.0,           # yaw angle (rad)
            0.0,           # yaw rate (rad/s)
            0.0            # slip angle at vehicle center (rad)
        ])
        
        # NEW: qLPV state vector for observer testing (7 states)
        # [X, Y, δ, v_x, ψ, r, v_y] - CommonRoad convention
        self.state_qlpv = np.array([
            car_id * 2.0,  # X position
            0.0,           # Y position
            0.0,           # δ steering angle (rad)
            0.0,           # v_x longitudinal velocity (m/s)
            0.0,           # ψ yaw angle (rad)
            0.0,           # r yaw rate (rad/s)
            0.0            # v_y lateral velocity (m/s)
        ])
        
        # NEW: True tire residuals for observer testing (ground truth)
        self.w_r = 0.0  # Rear tire residual [N]
        self.w_f = 0.0  # Front tire residual [N]
        
        # NEW: Lateral acceleration for observer measurement
        self.a_y = 0.0  # Lateral acceleration [m/s²]
        
        # NEW: Cornering stiffness from parameters (for residual computation)
        self.Cf = getattr(self.params, 'Cf', 50.0)
        self.Cr = getattr(self.params, 'Cr', 50.0)
        
        # Control input vector [steering_rate, acceleration]
        self.control_input = np.array([0.0, 0.0])
        
        # Timing
        self.last_time = time.time()
        
        # Physics properties (computed from state)
        self.x = self.state_ks[0]
        self.y = self.state_ks[1]
        self.heading = self.state_ks[4]
        self.velocity = self.state_ks[3]
        self.angular_velocity = 0.0
        self.lateral_velocity = 0.0  # NEW: For qLPV model
        
        # Model name for logging
        if self.use_qlpv_model:
            model_name = "qLPV (Observer-Compatible)"
        elif self.use_dynamic_model:
            model_name = "Single-Track Dynamic"
        else:
            model_name = "Kinematic Single-Track"
        print(f"🔧 MockQCar {car_id}: Initialized with {model_name} model")
        print(f"   Parameters: {vehicle_params}")
        print(f"   Vehicle: v_max={self.params.longitudinal.v_max:.2f} m/s, wheelbase={self.params.a + self.params.b:.3f}m")
        print(f"   Dimensions: L={self.params.l:.3f}m x W={self.params.w:.3f}m, mass={self.params.m:.2f}kg")
        print(f"   Position: ({self.x:.1f}, {self.y:.1f})")
    
    def read(self):
        """Simulate reading sensors and update physics"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Limit dt to prevent instability from CPU pauses
        dt = min(max(dt, 0.001), 0.1)
        
        # 1. Update physics based on selected model
        if self.use_qlpv_model:
            self._update_physics_qlpv(dt)
        else:
            self._update_physics(dt)
        
        # 2. Synchronize public properties from internal state
        self._sync_from_state()
        
        # 3. Update mock sensors for hardware compatibility
        # These match the real QCar sensor interface
        self.motorTach = self.velocity
        # Gyroscope and Accelerometer are updated within the physics update methods
        # but we ensure they are consistent here
        self.gyroscope[2] = self.angular_velocity
    
    def _sync_from_state(self):
        """Synchronize public properties from the active internal state model"""
        if self.use_qlpv_model:
            # qLPV state: [X, Y, δ, v_x, ψ, r, v_y]
            self.x = self.state_qlpv[0]
            self.y = self.state_qlpv[1]
            self.heading = self.state_qlpv[4]
            self.velocity = self.state_qlpv[3]
            self.angular_velocity = self.state_qlpv[5]
            self.lateral_velocity = self.state_qlpv[6]
        elif self.use_dynamic_model:
            # Dynamic state: [x, y, δ, v, ψ, r, β]
            self.x = self.state_st[0]
            self.y = self.state_st[1]
            self.heading = self.state_st[4]
            self.velocity = self.state_st[3]
            self.angular_velocity = self.state_st[5]
            # β is slip angle, not vy, but we can approximate vy = v * sin(β)
            self.lateral_velocity = self.velocity * math.sin(self.state_st[6])
        else:
            # Kinematic state: [x, y, δ, v, ψ]
            self.x = self.state_ks[0]
            self.y = self.state_ks[1]
            self.heading = self.state_ks[4]
            self.velocity = self.state_ks[3]
            
            # Compute yaw rate for kinematic model: r = (v/L) * tan(δ)
            if abs(self.velocity) > 0.01:
                wheelbase = self.params.a + self.params.b
                self.angular_velocity = (self.velocity / wheelbase) * math.tan(self.state_ks[2])
            else:
                self.angular_velocity = 0.0
            self.lateral_velocity = 0.0

        # Normalize heading to [-pi, pi]
        self.heading = (self.heading + math.pi) % (2 * math.pi) - math.pi
        
        # Update state vectors with normalized heading
        if self.use_qlpv_model: self.state_qlpv[4] = self.heading
        elif self.use_dynamic_model: self.state_st[4] = self.heading
        else: self.state_ks[4] = self.heading
    
    def write(self, throttle: float, steering: float):
        """Receive control commands"""
        self._throttle = np.clip(throttle, -1.0, 1.0)
        self._steering = np.clip(steering, -1.0, 1.0)
    
    def read_write_std(self, throttle: float, steering: float, LEDs=None):
        """Receive control commands"""
        self._throttle = np.clip(throttle, -1.0, 1.0)
        self._steering = np.clip(steering, -1.0, 1.0)
    
    def _update_physics(self, dt: float):
        """Physics simulation using Kinematic or Dynamic Single-Track models"""
        # Convert normalized commands to physical units
        max_steering = self.params.steering.max  
        target_steering = self._steering * max_steering
        max_accel = self.params.longitudinal.a_max 
        target_accel = self._throttle * max_accel
        
        # Determine which state to use for control logic
        current_state = self.state_st if self.use_dynamic_model else self.state_ks
        current_velocity = current_state[3]
        
        # --- Intelligent Friction & Braking ---
        if abs(self._throttle) > 0.01:
            # Driving mode
            c_rolling = 0.08
            c_air_drag = 0.1
            friction_val = c_rolling + (c_air_drag * abs(current_velocity))
            target_accel -= np.sign(current_velocity) * friction_val
        else:
            # Drag Braking mode (throttle is zero)
            brake_strength = 3.0
            if abs(current_velocity) > 0.05:
                target_accel = -np.sign(current_velocity) * brake_strength
            else:
                target_accel = 0.0
                current_state[3] = 0.0 # Force stop
        
        # Steering rate control (simulates servo dynamics)
        K_p_steering = 4.0
        steering_error = target_steering - current_state[2]
        steering_rate = K_p_steering * steering_error
        max_steering_v = float(self.params.steering.v_max)
        steering_rate = np.clip(steering_rate, -max_steering_v, max_steering_v)
        
        # Final control vector [steering_rate, acceleration]
        acceleration = np.clip(target_accel, -5.0, 5.0)
        self.control_input[0] = steering_rate
        self.control_input[1] = acceleration
        
        # Integrate model
        if self.use_dynamic_model:
            try:
                derivatives = vehicle_dynamics_st(self.state_st, self.control_input, self.params)
                
                if any(np.isnan(derivatives)) or any(np.isinf(derivatives)):
                    self._simple_kinematic_update(dt)
                    return
                
                # Euler integration
                for i in range(len(self.state_st)):
                    self.state_st[i] += np.clip(derivatives[i], -20.0, 20.0) * dt
                
                # Safety clamps
                v_max = float(self.params.longitudinal.v_max)
                v_min = float(self.params.longitudinal.v_min)
                self.state_st[3] = np.clip(self.state_st[3], v_min, v_max)
                self.state_st[5] = np.clip(self.state_st[5], -5.0, 5.0) # r
                self.state_st[6] = np.clip(self.state_st[6], -0.5, 0.5) # beta
                
                # Update IMU
                self._update_imu_sensors(derivatives, acceleration)
                
            except Exception as e:
                print(f"⚠️ Dynamic model error: {e}, falling back to kinematic")
                self._simple_kinematic_update(dt)
        else:
            # Kinematic model
            derivatives = vehicle_dynamics_ks(self.state_ks, self.control_input, self.params)
            for i in range(len(self.state_ks)):
                self.state_ks[i] += derivatives[i] * dt
            
            # Safety clamps
            v_max = float(self.params.longitudinal.v_max)
            v_min = float(self.params.longitudinal.v_min)
            self.state_ks[3] = np.clip(self.state_ks[3], v_min, v_max)
            
            # Update IMU sensors
            self._update_imu_sensors(derivatives, acceleration)
    
    def _simple_kinematic_update(self, dt: float):
        """Fallback simple kinematic update when dynamic model fails"""
        # Simple bicycle model as fallback
        max_velocity = 3.0
        target_velocity = self._throttle * max_velocity
        
        # Apply friction and braking
        friction_coeff = 0.8
        if abs(self.velocity) > 0.01:
            friction_decel = -np.sign(self.velocity) * friction_coeff * dt
            self.velocity += friction_decel
        
        # Active braking ONLY when throttle is near zero (not negative = backward)
        if abs(self._throttle) < 0.01 and abs(self.velocity) > 0.01:
            brake_decel = -np.sign(self.velocity) * 4.0 * dt
            self.velocity += brake_decel
        
        # Stop completely if velocity is very small and throttle near zero
        if abs(self.velocity) < 0.05 and abs(self._throttle) < 0.01:
            self.velocity = 0.0
        else:
            # Simple velocity control (allows forward AND backward motion)
            velocity_error = target_velocity - self.velocity
            acceleration = 2.0 * velocity_error
            self.velocity += acceleration * dt
        
        self.velocity = np.clip(self.velocity, -max_velocity, max_velocity)
        
        # Update position
        self.x += self.velocity * math.cos(self.heading) * dt
        self.y += self.velocity * math.sin(self.heading) * dt
        
        # Update heading based on steering
        if abs(self.velocity) > 0.1:
            wheelbase = 0.3
            self.angular_velocity = (self.velocity / wheelbase) * math.tan(self._steering * 0.3)
            self.heading += self.angular_velocity * dt
        else:
            self.angular_velocity = 0.0
    
    def _update_physics_qlpv(self, dt: float):
        """Physics simulation using qLPV dynamics (matches observer model)
         
        CONTROL INPUT NOTES:
        - Car receives: (steering_angle, throttle) in normalized [-1, 1] range
        - qLPV model expects: (steering_rate, acceleration)
        - We convert steering_angle → steering_rate using a P controller:
            steering_rate = K_p * (target_steering - current_steering)
        - This simulates how the real steering servo responds to angle commands"""
        
        # Convert normalized commands to physical units
        max_steering = self.params.steering.max  
        target_steering_angle = self._steering * max_steering
        max_accel = self.params.longitudinal.a_max
        acceleration = self._throttle * max_accel
        
        # Steering rate control (simulates servo dynamics)
        current_steering_angle = self.state_qlpv[2]
        K_p_steering = 4.0
        steering_rate = K_p_steering * (target_steering_angle - current_steering_angle)
        max_steering_rate = float(self.params.steering.v_max)
        steering_rate = np.clip(steering_rate, -max_steering_rate, max_steering_rate)
        
        # Control input for qLPV model: [steering_rate, acceleration]
        self.control_input = np.array([steering_rate, acceleration])
        
        # Integrate model
        try:
            derivatives = vehicle_dynamics_qlpv(
                self.state_qlpv, self.control_input, self.params, tire_mode='pacejka')
            
            for i in range(len(self.state_qlpv)):
                self.state_qlpv[i] += derivatives[i] * dt
            
            # Safety clamps
            v_max = float(self.params.longitudinal.v_max)
            v_min = float(self.params.longitudinal.v_min)
            self.state_qlpv[3] = np.clip(self.state_qlpv[3], v_min, v_max) # v_x
            self.state_qlpv[5] = np.clip(self.state_qlpv[5], -5.0, 5.0) # r
            self.state_qlpv[6] = np.clip(self.state_qlpv[6], -2.0, 2.0) # v_y
            
            # Update IMU and residuals
            self._update_imu_sensors(derivatives, acceleration)
            self._update_tire_residuals()
            
        except Exception as e:
            print(f"⚠️ qLPV model error: {e}, falling back to simple update")
            self._simple_kinematic_update(dt)
    
    def _update_imu_sensors(self, derivatives: np.ndarray, commanded_accel: float):
        """
        Compute realistic IMU sensor readings based on vehicle dynamics.
        
        Args:
            derivatives: State derivatives from the active model
            commanded_accel: Commanded acceleration for models without explicit v_dot
        """
        g = 9.81
        
        # Use shared properties synchronized from current model state
        vx = self.velocity
        vy = self.lateral_velocity
        r = self.angular_velocity
        
        # Determine acceleration derivatives based on model type
        if self.use_qlpv_model:
            # qLPV derivatives: [Ẋ, Ẏ, δ̇, v̇_x, ψ̇, ṙ, v̇_y]
            vx_dot = derivatives[3]
            vy_dot = derivatives[6]
        elif self.use_dynamic_model:
            # ST derivatives: [Ẋ, Ẏ, δ̇, v̇, ψ̇, ṙ, β̇]
            v_dot = derivatives[3]
            beta_dot = derivatives[6]
            # v_y = v * sin(beta) -> v_y_dot = v_dot * sin(beta) + v * cos(beta) * beta_dot
            # This is a bit complex, let's simplify for IMU
            vx_dot = v_dot # Approximation
            vy_dot = self.velocity * beta_dot # Approximation
        else:
            # KS derivatives: [Ẋ, Ẏ, δ̇, v̇, ψ̇]
            vx_dot = derivatives[3] if len(derivatives) > 3 else commanded_accel
            vy_dot = 0.0
        
        # Accelerometer (body frame):
        # a_x = v̇_x - r*v_y
        # a_y = v̇_y + r*v_x
        # In simulation, the IMU "measures" the inertial acceleration
        self.accelerometer[0] = float(vx_dot - r * vy)
        self.accelerometer[1] = float(vy_dot + r * vx)
        self.accelerometer[2] = g
        
        # Gyroscope
        self.gyroscope[2] = float(r)
        
        # Update lateral acceleration property used by observer
        self.a_y = self.accelerometer[1]

    
    def _update_tire_residuals(self):
        """Compute true tire residuals for observer testing"""
        vx = max(abs(self.state_qlpv[3]), 0.5)
        vy = self.state_qlpv[6]
        r = self.state_qlpv[5]
        delta = self.state_qlpv[2]
        
        lf = self.params.a
        lr = self.params.b
        
        # Slip angles
        alpha_f = delta - (vy + lf * r) / vx
        alpha_r = -(vy - lr * r) / vx
        
        # True tire residuals using Pacejka vs linear difference
        self.w_r, self.w_f = get_tire_residuals(
            alpha_f, alpha_r, self.Cf, self.Cr, self.params, vx)
        
        # Lateral acceleration
        Fyf_linear, Fyr_linear = compute_tire_forces_linear(
            alpha_f, alpha_r, self.Cf, self.Cr)
        Fyf_true = Fyf_linear + self.w_f
        Fyr_true = Fyr_linear + self.w_r
        self.a_y = get_lateral_acceleration(
            Fyf_true, Fyr_true, delta, self.params.m)
    
    def get_observer_measurement(self) -> np.ndarray:
        """
        Get measurement vector for observer [v_x, r, ψ, X, Y, a_y]
        
        Returns:
            Measurement vector (6D) for qLPV observer
        """
        # All models now share these properties via _sync_from_state
        return np.array([
            self.velocity,          # v_x (longitudinal)
            self.angular_velocity,   # r (yaw rate)
            self.heading,           # ψ (yaw)
            self.x,                 # X
            self.y,                 # Y
            getattr(self, 'a_y', 0.0) # a_y (lateral acceleration)
        ])
    
    def get_observer_state(self) -> np.ndarray:
        """
        Get true state in observer format [v_x, v_y, ψ, r, X, Y]
        
        Returns:
            True state vector (6D) for observer comparison
        """
        return np.array([
            self.velocity,          # v_x
            self.lateral_velocity,  # v_y
            self.heading,           # ψ
            self.angular_velocity,   # r
            self.x,                 # X
            self.y                  # Y
        ])
    
    def get_true_residuals(self) -> np.ndarray:
        """Get true tire residuals [w_r, w_f] for observer testing"""
        return np.array([self.w_r, self.w_f])

    def reset(self, pose: np.ndarray = None):
        """Reset vehicle state to initial or given pose"""
        if pose is not None:
            self.x = pose[0]
            self.y = pose[1]
            self.heading = pose[2]
        else:
            self.x = 0.0
            self.y = 0.0
            self.heading = 0.0

        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.lateral_velocity = 0.0

        # Reset qLPV state [X, Y, δ, v_x, ψ, r, v_y]
        self.state_qlpv = np.zeros(7)
        self.state_qlpv[0] = self.x
        self.state_qlpv[1] = self.y
        self.state_qlpv[2] = 0.0          # δ (steering)
        self.state_qlpv[3] = self.velocity # v_x
        self.state_qlpv[4] = self.heading  # ψ (yaw)
        self.state_qlpv[5] = self.angular_velocity # r (yaw rate)
        self.state_qlpv[6] = self.lateral_velocity # v_y

        # Reset Kinematic state [x, y, δ, v, ψ]
        self.state_ks = np.array([
            self.x,
            self.y,
            0.0,           # steering angle
            self.velocity,
            self.heading
        ])
        
        # Reset Dynamic Single-Track state [x, y, δ, v, ψ, r, β]
        self.state_st = np.array([
            self.x,
            self.y,
            0.0,           # steering angle
            self.velocity,
            self.heading,
            self.angular_velocity,
            0.0            # slip angle
        ])

        self.a_y = 0.0
        self.w_r = 0.0
        self.w_f = 0.0
        
        # Update existing GPS if it exists, otherwise create it
        if hasattr(self, 'gps') and self.gps is not None:
            self.gps.x = self.x
            self.gps.y = self.y
            self.gps.position = np.array([self.x, self.y, 0.0])
            self.gps.orientation = np.array([0.0, 0.0, self.heading])
        else:
            self.gps = MockQCarGPS(self)

class MockQCarGPS:
    """Mock GPS that provides position data"""
    
    def __init__(self, qcar: MockQCar, frequency: float = 200.0 , noise_enabled: bool = False):
        self.qcar = qcar
        self.x = qcar.x
        self.y = qcar.y
        self.latitude = 0.0
        self.longitude = 0.0
        self.valid = True
        self.noise_enabled = noise_enabled
        
        # Timing for frequency control
        self.last_update_time = 0.0
        self.frequency = frequency
        self.update_interval = 1.0 / self.frequency
        
        # Match real QCarGPS interface - position array [x, y, z]
        self.position = np.array([self.x, self.y, 0.0])
        # Match real QCarGPS interface - orientation array [roll, pitch, yaw]
        self.orientation = np.array([0.0, 0.0, qcar.heading])
        
        print(f"🛰️  MockGPS {qcar.car_id}: Initialized (Rate: {self.frequency}Hz)")
    
    def readGPS(self) -> bool:
        """
        Update GPS position from QCar physics.
        Returns True if a new reading was generated (based on frequency).
        """
        current_time = time.time()
        
        # Frequency control: check if enough time has passed
        if current_time - self.last_update_time < self.update_interval:
            return False
            
        self.last_update_time = current_time
        
        # 1. Update Position with optional noise
        # noise_std = 0.05 is ~5cm error, realistic for good GPS
        if getattr(self, 'noise_enabled', True):
            noise_x = random.gauss(0, 0.05)
            noise_y = random.gauss(0, 0.05)
        else:
            noise_x, noise_y = 0.0, 0.0
            
        self.x = self.qcar.x + noise_x
        self.y = self.qcar.y + noise_y
        
        # Update internal position array [x, y, z]
        self.position[0] = self.x
        self.position[1] = self.y
        self.position[2] = 0.0  # Altitude 0
        
        # 2. Update Orientation [roll, pitch, yaw]
        self.orientation[0] = 0.0
        self.orientation[1] = 0.0
        self.orientation[2] = self.qcar.heading
        
        # 3. Simulated Geodetic coordinates (using a fixed local origin)
        # 1 degree lat is ~111,000m. 0.00001 deg is ~1.1m
        lat_origin = 43.6532  # Toronto
        lon_origin = -79.3832
        self.latitude = lat_origin + (self.y * 0.000009)
        self.longitude = lon_origin + (self.x * 0.000012)
        
        return True






class FakeVehicleWithRealLogic:
    """Fake vehicle that uses the real VehicleLogic class"""
    
    def __init__(self, car_id: int, host_ip: str, base_port: int, dynamic_model_type: int = 0, vehicle_params: str = 'qcar'):
        self.car_id = car_id
        self.host_ip = host_ip
        self.base_port = base_port
        

        
        # Create mock hardware with proper vehicle dynamics
        self.mock_qcar = MockQCar(car_id, dynamic_model_type=dynamic_model_type, vehicle_params=vehicle_params)
        self.mock_gps = MockQCarGPS(self.mock_qcar)
        # No mock YOLO - it will be set to None and disabled
        
        # Create real configuration
        self.config = self._create_real_config()
        # self.mock_qcar
        # Create kill event
        self.kill_event = Event()
        
        # Create the REAL VehicleLogic
        self.vehicle_logic = VehicleLogic(self.config, self.kill_event)
        
        # Set a reference so the fake initialization state can access our mock hardware
        self.vehicle_logic._parent_fake_vehicle = self
        
        # Replace ONLY the INITIALIZING state with fake version AFTER starting
        # All other states remain real for complete system testing
        
        # Replace hardware with mocks AFTER VehicleLogic is created
        self._inject_mock_hardware()
        
        # Monkey patch _observer_update to inject ground truth provider into estimator
        self._patch_observer_update()

        # Initialize state for main loop
        self.running = True  # Start in running state
        self.ground_station_client = None  # Will be set by VehicleLogic
        
        # Statistics
        self.start_time = time.time()
        
        print(f"✅ Real VehicleLogic initialized for Car {car_id}")
        print(f"   Mock hardware injected successfully")
    
    def _create_real_config(self) -> VehicleMainConfig:
        """Create real configuration for VehicleLogic"""
        config = VehicleMainConfig()
        
        # Network configuration
        config.network.car_id = self.car_id
        config.network.host_ip = self.host_ip
        config.network.base_port = self.base_port
        
        # Enable telemetry for Ground Station visibility
        config.logging.enable_telemetry_logging = True
        
        # Set timing for better telemetry rate and V2V performance
        config.timing.controller_update_rate = 200  # 200 Hz (match vehicle_logic.py)
        config.timing.telemetry_send_rate = 20      # 20 Hz for V2V compatibility
        config.timing.tf = 300.0  # 5 minute experiment
        
        
        # Path configuration - don't set valid_nodes as it's a property
        # The default valid_nodes from PathPlanningConfig will be used
        
        return config
    
    def _replace_initialization_state_only(self):
        """Replace only the INITIALIZING state with fake version, keep everything else real"""
        try:
            # Wait for VehicleLogic to create the state machine
            import time
            start_time = time.time()
            while not hasattr(self.vehicle_logic, 'state_machine') and (time.time() - start_time) < 5.0:
                time.sleep(0.1)
            
            if not hasattr(self.vehicle_logic, 'state_machine'):
                print("🔧 State machine not found, VehicleLogic may not be fully initialized")
                return
            
            # Replace only the INITIALIZING state handler with our fake one
            from fake_initializing_state import FakeInitializingState
            fake_init_state = FakeInitializingState(self.vehicle_logic)
            self.vehicle_logic.state_machine.state_handlers[VehicleState.INITIALIZING] = fake_init_state
            
            # IMPORTANT: If state machine is already in INITIALIZING state, we need to call enter()
            # to trigger telemetry logging setup
            if self.vehicle_logic.state_machine.state == VehicleState.INITIALIZING:
                print(f"[!] State machine already in INITIALIZING - calling fake enter() now")
                fake_init_state.enter()
            
            # Path planning will be initialized by FakeInitializingState._check_components_ready()
            # This matches the real initialization flow where path planning happens during INITIALIZING state

            # print(f"✅ Car {self.car_id}: Replaced INITIALIZING state with fake version")
            # print(f"          All other states remain real for complete system testing")
            # print(f"          Path planning will be initialized by fake INITIALIZING state")
            
        except Exception as e:
            print(f"❌ Car {self.car_id}: Failed to replace initialization state: {e}")

    def _inject_mock_hardware(self):
        """Mock hardware injection will be done during state machine initialization"""
        # The fake initialization state will handle mock hardware injection
        print(f"🔧 Car {self.car_id}: Mock hardware injection deferred to initialization state")
    
    def _patch_observer_update(self):
        """Monkey patch VehicleLogic._observer_update to inject ground truth provider"""
        original_observer_update = self.vehicle_logic._observer_update
        
        def patched_observer_update(dt: float):
            # Call original method
            original_observer_update(dt)
            
            # Try to inject ground truth provider if not already set
            try:
                if hasattr(self.vehicle_logic, 'vehicle_observer'):
                    est = self.vehicle_logic.vehicle_observer.get_local_estimator()
                    # Check if it's a neural estimator and needs provider
                    if est and hasattr(est, 'set_ground_truth_provider') and getattr(est, 'ground_truth_provider', None) is None:
                        # Assuming NeuralLuenbergerEstimator or similar
                        print(f"✅ [SIM] Injecting MockQCar as Ground Truth Provider into Observer")
                        est.set_ground_truth_provider(self.mock_qcar)
            except Exception:
                pass
                
        # Apply the patch
        self.vehicle_logic._observer_update = patched_observer_update

    def start_simulation(self):
        """Start the fake vehicle simulation using real VehicleLogic"""
        print("\\n" + "="*60)
        print("[SIM] Starting Real VehicleLogic with Mock Hardware")
        print("      VehicleLogic handles ALL initialization including Ground Station")
        print("="*60)
        
        # Replace only the initialization state (keep everything else real)
        self._replace_initialization_state_only()
        
        print(f"✅ Car {self.car_id}: Real VehicleLogic ready with fake initialization")
        print(f"          ✅ Ground Station connection: Handled by VehicleLogic")
        print(f"          ✅ State machine: Real (except INITIALIZING state)")
        print(f"          ✅ All controllers: Real")
        print(f"          ✅ Hardware: Mock (injected during initialization)")
    
    
    def run(self):
        """Run the real VehicleLogic directly (no thread)"""
        print(f"🧠 Car {self.car_id}: Starting VehicleLogic.run()...")
        
        try:
            # Run the REAL VehicleLogic directly
            self.vehicle_logic.run()
            print(f"[RUN] Car {self.car_id}: VehicleLogic.run() completed normally")
        except Exception as e:
            print(f"❌ Car {self.car_id}: VehicleLogic error - {e}")
            import traceback
            traceback.print_exc()
        finally:
            print(f"🧠 Car {self.car_id}: VehicleLogic stopped")
            # Signal that we're shutting down
            self.running = False
            self.kill_event.set()
    

    
    def stop(self):
        """Stop the simulation"""
        print(f"🛑 Car {self.car_id}: Stopping Real VehicleLogic simulation...")
        
        # Signal shutdown
        self.running = False
        self.kill_event.set()
        
        # Close Ground Station client
        if self.ground_station_client:
            try:
                self.ground_station_client.close()
            except Exception as e:
                print(f"WARNING: Car {self.car_id}: Error closing Ground Station client - {e}")
            self.ground_station_client = None
        
        # Print final stats
        uptime = time.time() - self.start_time
        print(f"\n[STATS] Car {self.car_id} Real VehicleLogic Final Stats:")
        print(f"   Uptime: {uptime:.1f}s")
        print(f"   Final position: ({self.mock_qcar.x:.2f}, {self.mock_qcar.y:.2f})")
        print(f"   Final velocity: {self.mock_qcar.velocity:.2f} m/s")
        if hasattr(self.vehicle_logic, 'state_machine') and self.vehicle_logic.state_machine:
            print(f"   Final VehicleLogic state: {self.vehicle_logic.state_machine.state.name}")
        if hasattr(self.vehicle_logic, 'loop_counter'):
            print(f"   VehicleLogic iterations: {self.vehicle_logic.loop_counter}")
        if self.ground_station_client:
            stats = self.ground_station_client.get_statistics()
            print(f"   Network stats: Sent={stats.get('telemetry_sent', 0)}, Received={stats.get('commands_received', 0)}")


# Using real GroundStationClient instead of mock


def main():
    """Main entry point"""
    
    # Parse command line arguments
    car_id = 0
    host_ip = '127.0.0.1'
    base_port = 5000
    dynamic_model_type = 1 # Default to qLPV (2)
    vehicle_params = 'qcar'  # Default to QCar parameters
    
    # Smart argument parsing to allow flexible usage
    # Example: python fake_vehicle.py 0 qlpv
    args = sys.argv[1:]
    
    # 1. First argument is Car ID (if it's a small integer)
    if len(args) > 0 and args[0].isdigit() and int(args[0]) < 1000:
        car_id = int(args[0])
        args.pop(0)

    # 2. Parse remaining arguments by type/keyword
    for arg in args:
        val = arg.lower()
        
        # Model Type
        if val in ['0', 'kinematic', 'ks']:
            dynamic_model_type = 0
            continue
        if val in ['1', 'dynamic', 'st', 'true', 'yes']:
            dynamic_model_type = 1
            continue
        if val in ['2', 'qlpv']:
            dynamic_model_type = 2
            continue
            
        # Vehicle Params
        if val in ['qcar', 'vehicle1', 'vehicle2', 'vehicle3', 'vehicle4']:
            vehicle_params = val
            continue
            
        # Port (large integer)
        if arg.isdigit() and int(arg) > 1000:
            base_port = int(arg)
            continue
            
        # Host IP (has dot or localhost)
        if '.' in arg or val == 'localhost':
            host_ip = arg
            continue
    
    print("="*70)
    print("[CAR] QCar Fake Vehicle with REAL VehicleLogic ")
    # print("   Uses actual VehicleLogic + real StateMachine + real GroundStationClient")
    # print("   Only INITIALIZING state is fake for quick mock hardware injection")
    print("   Vehicle dynamics from vehiclemodels folder (CommonRoad models)")
    print("="*70)
    print(f"Car ID: {car_id}")
    print(f"Ground Station: {host_ip}:{base_port + car_id}")
    
    model_names = {0: "Kinematic Single-Track", 1: "Single-Track Dynamic", 2: "qLPV"}
    print(f"Vehicle Model: {model_names.get(dynamic_model_type, 'Unknown')} (ID: {dynamic_model_type})")
    
    print(f"Vehicle Parameters: {vehicle_params}")
    print(f"Approach: Real VehicleLogic + Fake initialization + Mock hardware")
    print("")
    
    # Create fake vehicle with real logic
    try:
        vehicle = FakeVehicleWithRealLogic(car_id, host_ip, base_port, 
                                          dynamic_model_type=dynamic_model_type,
                                          vehicle_params=vehicle_params)
        print(f"✅ Fake vehicle created successfully")
    except Exception as e:
        print(f"❌ Failed to create fake vehicle: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Start simulation (includes Ground Station connection)
    vehicle.start_simulation()
    

    
    try:
        # Run VehicleLogic directly (no thread)
        print(f"[MAIN] Car {car_id}: Starting VehicleLogic.run()...")
        vehicle.run()
        print(f"[MAIN] Car {car_id}: VehicleLogic.run() completed")
    
    except KeyboardInterrupt:
        print(f"\n🛑 Car {car_id}: Shutting down...")
    
    except Exception as e:
        print(f"❌ Car {car_id}: Unexpected error - {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        vehicle.stop()
        print(f"[CAR] Car {car_id}: Real VehicleLogic simulation ended")
        return 0


if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)