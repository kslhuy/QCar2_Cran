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
from ground_station_client import GroundStationClient
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
    QLPVVehicleModel,
    get_tire_residuals,
    get_lateral_acceleration,
    compute_tire_forces_linear,
    state_qlpv_to_observer
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
        
        if dt > 0.1:  # Skip large time steps
            dt = 0.02
        
        # Update physics based on selected model
        if self.use_qlpv_model:
            self._update_physics_qlpv(dt)
        else:
            self._update_physics(dt)
        
        # Update mock sensors from state
        if self.use_qlpv_model:
            # qLPV model: [X, Y, δ, v_x, ψ, r, v_y]
            self.motorTach = self.state_qlpv[3]  # v_x
            self.gyroscope[2] = self.state_qlpv[5]  # r (yaw rate)
            self.accelerometer[1] = self.a_y  # Lateral acceleration
            # TODO : more accurate longitudinal acceleration
            self.accelerometer[0] = 0.0  # Simplification
        elif self.use_dynamic_model:
            self.motorTach = self.state_st[3]  # velocity
            self.gyroscope[2] = self.state_st[5]  # yaw rate
        else:
            self.motorTach = self.state_ks[3]  # velocity
            # Compute yaw rate for kinematic model
            if abs(self.state_ks[3]) > 0.01:
                wheelbase = self.params.a + self.params.b
                self.gyroscope[2] = (self.state_ks[3] / wheelbase) * math.tan(self.state_ks[2])
            else:
                self.gyroscope[2] = 0.0
    
    def write(self, throttle: float, steering: float):
        """Receive control commands"""
        self._throttle = np.clip(throttle, -1.0, 1.0)
        self._steering = np.clip(steering, -1.0, 1.0)
    
    def read_write_std(self, throttle: float, steering: float, LEDs=None):
        """Receive control commands"""
        self._throttle = np.clip(throttle, -1.0, 1.0)
        self._steering = np.clip(steering, -1.0, 1.0)
    
    def _update_physics(self, dt: float):
        """Physics simulation using vehicle dynamics models from vehiclemodels folder"""
        # Convert normalized commands to physical units
        # Steering: -1 to 1 -> max steering angle from params (scaled down for QCar)
        max_steering = self.params.steering.max  # rad (~30 degrees) - more realistic for small vehicles
        target_steering = self._steering * max_steering
        
        # Throttle: -1 to 1 -> acceleration (scaled for QCar)
        max_accel = self.params.longitudinal.a_max  # m/s^2 - reasonable for small vehicle
        target_accel = self._throttle * max_accel
        
        # Apply friction/drag and braking forces
        current_state = self.state_st if self.use_dynamic_model else self.state_ks
        current_velocity = current_state[3]
        
        # Friction coefficient (rolling resistance + air drag simplified)


        # --- 2. INTELLIGENT FRICTION  ---
        
        # CONSTANTS
        # Friction when driving (MUST be lower than target_accel to move)
        c_rolling = 0.08   # Low resistance so motor wins
        c_air_drag = 0.1   # Wind resistance
        
        # Friction when throttle is zero (The "RC Drag Brake")
        brake_strength = 3.0 # High value = Instant stop
        
        # LOGIC
        if abs(self._throttle) > 0.01:
            # === STATE A: DRIVING ===
            # Apply normal physics: Motor Force - Small Friction
            friction_val = c_rolling + (c_air_drag * abs(current_velocity))
            friction_decel = -np.sign(current_velocity) * friction_val
            
            # Combine Motor + Friction
            target_accel = target_accel + friction_decel
            
        else:
            # === STATE B: DRAG BRAKING (Throttle is 0) ===
            # Ignore motor, apply massive braking force
            
            # Check if we are moving fast enough to need braking
            if abs(current_velocity) > 0.05:
                # Apply strong braking opposite to movement
                target_accel = -np.sign(current_velocity) * brake_strength
            else:
                # Snap to full stop if nearly stopped
                target_accel = 0.0
                current_state[3] = 0.0 # Force velocity state to 0
        
        # target_accel = self._throttle * max_accel + friction_decel


        # Stop completely if velocity is very small and throttle near zero
        if abs(current_velocity) < 0.05 and abs(self._throttle) < 0.01:
            target_accel = 0.0
            # Force velocity to zero in state
            current_state[3] = 0.0
        
        # Compute control inputs
        # steering_rate (rad/s) - simple P control
        K_p_steering = 4.0
        steering_error = target_steering - current_state[2]
        steering_rate = K_p_steering * steering_error  # P controller gain (reduced for stability)
        
        # Clip steering rate to reasonable values
        max_steering_v = float(self.params.steering.v_max)  # 3.0 rad/s from YAML
        steering_rate = np.clip(steering_rate, -max_steering_v, max_steering_v)
        
        # acceleration (m/s^2)
        acceleration = np.clip(target_accel, -5.0, 5.0)  # Clip to reasonable range
        
        self.control_input[0] = steering_rate
        self.control_input[1] = acceleration
        
        # Integrate using appropriate vehicle model
        if self.use_dynamic_model:
            # Use single-track dynamic model (more realistic)
            try:
                derivatives = vehicle_dynamics_st(self.state_st, self.control_input, self.params)
                
                # Check for numerical issues
                if any(np.isnan(derivatives)) or any(np.isinf(derivatives)):
                    print(f"⚠️  Warning: Numerical instability detected, resetting to safe state")
                    # Reset to safe kinematic update
                    self._simple_kinematic_update(dt)
                    return
                
                # Simple Euler integration with safety limits
                for i in range(len(self.state_st)):
                    # Limit derivative magnitude to prevent instability
                    if i >= 5:  # yaw_rate and slip_angle
                        derivatives[i] = np.clip(derivatives[i], -10.0, 10.0)
                    self.state_st[i] += derivatives[i] * dt
                
                v_max = float(self.params.longitudinal.v_max)  # 2.0
                v_min = float(self.params.longitudinal.v_min)  # -1.0
                # Clamp velocity to reasonable range
                self.state_st[3] = np.clip(self.state_st[3], v_min, v_max)
                
                # Clamp yaw rate to prevent explosion
                self.state_st[5] = np.clip(self.state_st[5], -5.0, 5.0)
                
                # Clamp slip angle to reasonable range
                self.state_st[6] = np.clip(self.state_st[6], -0.5, 0.5)
                
                # Update public properties
                self.x = self.state_st[0]
                self.y = self.state_st[1]
                self.heading = self.state_st[4]
                self.velocity = self.state_st[3]
                self.angular_velocity = self.state_st[5]
                
            except Exception as e:
                print(f"⚠️  Dynamic model error: {e}, using simple kinematic update")
                self._simple_kinematic_update(dt)
            
        else:
            # Use kinematic single-track model (simpler, faster)
            derivatives = vehicle_dynamics_ks(self.state_ks, self.control_input, self.params)
            
            # Simple Euler integration
            for i in range(len(self.state_ks)):
                self.state_ks[i] += derivatives[i] * dt
            
            # Clamp velocity to reasonable range
            v_max = float(self.params.longitudinal.v_max)  # 2.0
            v_min = float(self.params.longitudinal.v_min)  # -1.0
            self.state_ks[3] = np.clip(self.state_ks[3], v_min, v_max)
            
            # Update public properties
            self.x = self.state_ks[0]
            self.y = self.state_ks[1]
            self.heading = self.state_ks[4]
            self.velocity = self.state_ks[3]
            # print(f"_steering={self._steering:.3f}, target_steering={target_steering:.3f}, "
            #     f"state_steer={current_state[2]:.6f}, err={steering_error:.6f}, "
            #     f"Kp={K_p_steering}, steering_rate={steering_rate:.6f}")
            # print(f"Velocity: {self.velocity}, Heading: {self.heading} , control input: {self.control_input}")

            # Compute angular velocity
            if abs(self.velocity) > 0.01:
                wheelbase = self.params.a + self.params.b
                self.angular_velocity = (self.velocity / wheelbase) * math.tan(self.state_ks[2])
            else:
                self.angular_velocity = 0.0
        
        # Normalize heading
        while self.heading > math.pi:
            self.heading -= 2 * math.pi
        while self.heading < -math.pi:
            self.heading += 2 * math.pi
    
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
        """
        Physics simulation using qLPV dynamics (matches observer model)
        
        State: [X, Y, δ, v_x, ψ, r, v_y]
        This model matches the observer's dynamics exactly for proper testing.
        
        CONTROL INPUT NOTES:
        - Car receives: (steering_angle, throttle) in normalized [-1, 1] range
        - qLPV model expects: (steering_rate, acceleration)
        - We convert steering_angle → steering_rate using a P controller:
            steering_rate = K_p * (target_steering - current_steering)
        - This simulates how the real steering servo responds to angle commands
        """
        # Convert normalized commands to physical units
        # _steering: normalized [-1, 1] steering ANGLE command
        # _throttle: normalized [-1, 1] throttle command
        max_steering = self.params.steering.max  # rad
        target_steering_angle = self._steering * max_steering
        
        # Throttle → Acceleration (direct mapping)
        max_accel = self.params.longitudinal.a_max
        acceleration = self._throttle * max_accel
        
        # Current steering angle from state
        current_steering_angle = self.state_qlpv[2]  # δ
        
        # Compute steering rate: P controller to track target steering angle
        # This simulates the steering servo dynamics
        K_p_steering = 4.0  # Increased from 1.0 to 4.0 for better tracking
        steering_rate = K_p_steering * (target_steering_angle - current_steering_angle)
        
        # Clamp steering rate to physical limits
        max_steering_rate = float(self.params.steering.v_max)
        steering_rate = np.clip(steering_rate, -max_steering_rate, max_steering_rate)
        
        # Control input for qLPV model: [steering_rate, acceleration]
        control = np.array([steering_rate, acceleration])
        
        # Store control input for observer use
        self.control_input = control.copy()
        
        # Use qLPV dynamics (with Pacejka for true tire forces)
        try:
            derivatives = vehicle_dynamics_qlpv(
                self.state_qlpv, control, self.params,tire_mode='dynamic_linear' )
            
            # Euler integration
            for i in range(len(self.state_qlpv)):
                self.state_qlpv[i] += derivatives[i] * dt
            
            # Clamp velocity to physical limits
            v_max = float(self.params.longitudinal.v_max)
            v_min = float(self.params.longitudinal.v_min)
            self.state_qlpv[3] = np.clip(self.state_qlpv[3], v_min, v_max)
            
            # Clamp yaw rate and lateral velocity for stability
            self.state_qlpv[5] = np.clip(self.state_qlpv[5], -5.0, 5.0)
            self.state_qlpv[6] = np.clip(self.state_qlpv[6], -2.0, 2.0)
            
        except Exception as e:
            print(f"⚠️ qLPV model error: {e}, using simple update")
            self._simple_kinematic_update(dt)
            return
        
        # Update public properties from qLPV state
        self.x = self.state_qlpv[0]  # X
        self.y = self.state_qlpv[1]  # Y
        self.heading = self.state_qlpv[4]  # ψ
        self.velocity = self.state_qlpv[3]  # v_x
        self.angular_velocity = self.state_qlpv[5]  # r
        self.lateral_velocity = self.state_qlpv[6]  # v_y
        
        # Normalize heading
        while self.heading > math.pi:
            self.heading -= 2 * math.pi
        while self.heading < -math.pi:
            self.heading += 2 * math.pi
        self.state_qlpv[4] = self.heading
        
        # Update IMU sensors (realistic accelerometer and gyroscope)
        self._update_imu_sensors(derivatives, acceleration)
        
        # Update tire residuals (ground truth for observer testing)
        self._update_tire_residuals()
    
    def _update_imu_sensors(self, derivatives: list, commanded_accel: float):
        """
        Compute realistic IMU sensor readings based on vehicle dynamics.
        
        Accelerometer (body frame): [a_x, a_y, a_z]
            - a_x: longitudinal acceleration = dv_x/dt - r*v_y (inertial + centripetal)
            - a_y: lateral acceleration = dv_y/dt + r*v_x (inertial + centripetal)
            - a_z: vertical = gravity (assuming flat ground)
        
        Gyroscope (body frame): [ω_x, ω_y, ω_z]
            - ω_x: roll rate (assumed ~0 for bicycle model)
            - ω_y: pitch rate (assumed ~0 for bicycle model)
            - ω_z: yaw rate = r
        
        Args:
            derivatives: State derivatives from qLPV model
            commanded_accel: Commanded acceleration from control input
        """
        g = 9.81  # gravity [m/s²]
        
        # Extract states
        vx = self.state_qlpv[3]  # longitudinal velocity
        vy = self.state_qlpv[6]  # lateral velocity
        r = self.state_qlpv[5]   # yaw rate
        
        # State derivatives: [Ẋ, Ẏ, δ̇, v̇_x, ψ̇, ṙ, v̇_y]
        # derivatives[3] = v̇_x, derivatives[6] = v̇_y
        vx_dot = derivatives[3] if len(derivatives) > 3 else commanded_accel
        vy_dot = derivatives[6] if len(derivatives) > 6 else 0.0
        
        # Accelerometer readings (body frame)
        # IMU measures: a_measured = a_inertial - g (gravity subtracted in real IMU)
        # But for simulation, we compute what the IMU would read:
        
        # Longitudinal acceleration: v̇_x - r*v_y (minus centripetal term)
        # Note: In body frame, centripetal effect adds r*v_y to apparent acceleration
        a_x_body = vx_dot + r * vy  # includes centripetal correction
        
        # Lateral acceleration: v̇_y + r*v_x (plus centripetal term)
        # This is what the IMU measures as lateral acceleration
        a_y_body = vy_dot + r * vx  # includes centripetal acceleration
        
        # Vertical acceleration (gravity in body frame, assuming flat ground)
        a_z_body = g
        
        # Update accelerometer array
        self.accelerometer[0] = float(a_x_body)
        self.accelerometer[1] = float(a_y_body)
        self.accelerometer[2] = float(a_z_body)
        
        # Gyroscope readings (body frame angular rates)
        # For bicycle model: roll and pitch rates are approximately zero
        self.gyroscope[0] = 0.0   # ω_x (roll rate) - assumed zero
        self.gyroscope[1] = 0.0   # ω_y (pitch rate) - assumed zero
        self.gyroscope[2] = float(r)  # ω_z (yaw rate)
        
        # Also update the lateral acceleration for observer (a_y in sensor)
        self.a_y = float(a_y_body)
    
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
        if self.use_qlpv_model:
            return np.array([
                self.state_qlpv[3],  # v_x
                self.state_qlpv[5],  # r
                self.state_qlpv[4],  # ψ
                self.state_qlpv[0],  # X
                self.state_qlpv[1],  # Y
                self.a_y,            # a_y
            ])
        else:
            # For non-qLPV models, approximate the measurement
            return np.array([
                self.velocity,       # v_x
                self.angular_velocity,  # r
                self.heading,        # ψ
                self.x,              # X
                self.y,              # Y
                0.0,                 # a_y (not computed)
            ])
    
    def get_observer_state(self) -> np.ndarray:
        """
        Get true state in observer format [v_x, v_y, ψ, r, X, Y]
        
        Returns:
            True state vector (6D) for observer comparison
        """
        if self.use_qlpv_model:
            return state_qlpv_to_observer(self.state_qlpv)
        else:
            return np.array([
                self.velocity,
                0.0,  # v_y unknown for non-qLPV models
                self.heading,
                self.angular_velocity,
                self.x,
                self.y,
            ])
    
    def get_true_residuals(self) -> np.ndarray:
        """Get true tire residuals [w_r, w_f] for observer testing"""
        return np.array([self.w_r, self.w_f])

    def reset(self , pose: np.ndarray = None):
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

        self.state_qlpv = np.zeros(7)
        self.state_qlpv[0] = self.x
        self.state_qlpv[1] = self.y
        self.state_qlpv[2] = self.heading
        self.state_qlpv[3] = self.velocity
        self.state_qlpv[4] = self.heading
        self.state_qlpv[5] = self.angular_velocity
        self.state_qlpv[6] = 0.0

        self.state_ks = np.array([
            self.x,  # x position
            self.y,           # y position
            0.0,           # steering angle (rad)
            0.0,           # velocity (m/s)
            self.heading            # yaw angle (rad)
        ])
        
        # Vehicle state vector for single-track dynamic model (7 states)
        # [x, y, steering_angle, velocity, yaw_angle, yaw_rate, slip_angle]
        self.state_st = np.array([
            self.x,  # x position
            self.y,           # y position
            0.0,           # steering angle (rad)
            0.0,           # velocity (m/s)
            self.heading,           # yaw angle (rad)
            0.0,           # yaw rate (rad/s)
            0.0            # slip angle at vehicle center (rad)
        ])
        self.a_y = 0.0
        self.w_r = 0.0
        self.w_f = 0.0
        self.gps = MockQCarGPS(self)

class MockQCarGPS:
    """Mock GPS that provides position data"""
    
    def __init__(self, qcar: MockQCar):
        self.qcar = qcar
        self.x = qcar.x
        self.y = qcar.y
        self.latitude = 0.0
        self.longitude = 0.0
        self.valid = True
        
        # Match real QCarGPS interface - position array [x, y, z]
        self.position = np.array([self.x, self.y, 0.0])
        # Match real QCarGPS interface - orientation array [roll, pitch, yaw]
        self.orientation = np.array([0.0, 0.0, qcar.heading])
        
        print(f"🛰️  MockGPS {qcar.car_id}: Initialized")
    
    def readGPS(self):
        """Update GPS position from QCar physics - returns True on success"""
        # Add some noise to simulate real GPS
        noise_std = 0.05  # 5cm standard deviation
        self.x = self.qcar.x + random.gauss(0, noise_std)
        self.y = self.qcar.y + random.gauss(0, noise_std)
        
        # Update position array (match real QCarGPS interface)
        self.position[0] = self.x
        self.position[1] = self.y
        self.position[2] = 0.0  # z (altitude)
        
        # Update orientation array (match real QCarGPS interface)
        self.orientation[0] = 0.0  # roll
        self.orientation[1] = 0.0  # pitch
        self.orientation[2] = self.qcar.heading  # yaw
        
        # Convert to lat/lon (fake conversion)
        self.latitude = self.y * 0.00001 + 43.0  # Fake latitude around 43°N
        self.longitude = self.x * 0.00001 + -80.0  # Fake longitude around 80°W
        
        # Return True to indicate successful reading (match real QCarGPS)
        return True


# class MockYOLOReceiver:
#     """Mock YOLO receiver that provides detection data"""
    
#     def __init__(self):
#         # YOLO detection arrays (same format as real YOLO)
#         self.stopSign = np.zeros(7, dtype=np.float64)
#         self.trafficlight = np.zeros(7, dtype=np.float64) 
#         self.cars = np.zeros(7, dtype=np.float64)
#         self.yieldSign = np.zeros(7, dtype=np.float64)
#         self.person = np.zeros(7, dtype=np.float64)
        
#         print("👁️  MockYOLO: Initialized (no detections)")
    
#     def read(self):
#         """Simulate YOLO detections - always empty for safe testing"""
#         # Always reset all detections to zero (no objects detected)
#         self.stopSign.fill(0.0)
#         self.trafficlight.fill(0.0)
#         self.cars.fill(0.0)
#         self.yieldSign.fill(0.0)
#         self.person.fill(0.0)
        
#         # No fake detections to avoid triggering emergency stops
    
#     def terminate(self):
#         """Mock terminate method"""
#         pass


class MockStateEstimator:
    """Mock State Estimator that provides position from mock GPS"""
    
    def __init__(self, qcar: MockQCar, gps: MockQCarGPS):
        self.qcar = qcar
        self.gps = gps
        self.state_valid = True
        
        print(f"🧭 MockStateEstimator {qcar.car_id}: Initialized")
    
    def update(self, motor_tach: float, steering: float, dt: float, gyro_z: float):
        """Update state estimate (mock implementation)"""
        # Update GPS position
        self.gps.readGPS()
        
        # Add some small random movement for testing telemetry
        if random.random() < 0.01:  # 1% chance per update
            self.qcar.x += random.uniform(-0.05, 0.05)
            self.qcar.y += random.uniform(-0.05, 0.05)
            self.qcar.heading += random.uniform(-0.02, 0.02)
    
    def get_state(self):
        """Get current state estimate"""
        return (
            self.qcar.x,      # x position
            self.qcar.y,      # y position  
            self.qcar.heading,  # theta (heading)
            self.qcar.velocity, # velocity
            self.state_valid   # state validity
        )


# class MockYOLODrive:
#     """Mock YOLO Drive system for obstacle detection and response"""
    
#     def __init__(self, car_id: int):
#         self.car_id = car_id
#         self.yolo_gain = 1.0  # Default gain
#         self.carDist = 100.0  # Safe distance - no cars detected
#         self.personDist = 100.0  # Safe distance - no persons detected
#         print(f"🤖 MockYOLODrive {car_id}: Initialized (safe distances)")
    
#     def check_yolo(self, stop_sign, traffic_light, cars, yield_sign, person):
#         """Mock YOLO obstacle detection - always returns safe values for testing"""
#         # For testing, always report safe distances to avoid emergency stops
#         self.carDist = 100.0  # Always safe distance
#         self.personDist = 100.0  # Always safe distance
        
#         # Always return normal speed gain (no obstacles)
#         self.yolo_gain = 1.0
#         return 1.0


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
    
    if len(sys.argv) > 1:
        car_id = int(sys.argv[1])
    if len(sys.argv) > 2:
        host_ip = sys.argv[2]
    if len(sys.argv) > 3:
        base_port = int(sys.argv[3])
    if len(sys.argv) > 4:
        # Accept integer ID or string names
        arg = sys.argv[4].lower()
        if arg in ['0', 'kinematic', 'ks']:
            dynamic_model_type = 0
        elif arg in ['1', 'dynamic', 'st']:
            dynamic_model_type = 1
        elif arg in ['2', 'qlpv']:
            dynamic_model_type = 2
        elif arg in ['true', 'yes']: # Backwards compatibility
            dynamic_model_type = 1
        else:
            try:
                dynamic_model_type = int(arg)
            except:
                print(f"⚠️ Unknown model type '{arg}', defaulting to Kinematic (0)")
                dynamic_model_type = 0
            
    if len(sys.argv) > 5:
        vehicle_params = sys.argv[5]  # qcar, vehicle1, vehicle2, vehicle3, vehicle4
    
    print("="*70)
    print("[CAR] QCar Fake Vehicle with REAL VehicleLogic - SIMPLIFIED")
    print("   Uses actual VehicleLogic + real StateMachine + real GroundStationClient")
    print("   Only INITIALIZING state is fake for quick mock hardware injection")
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