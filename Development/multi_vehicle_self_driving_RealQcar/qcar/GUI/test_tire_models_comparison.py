"""
Comparison of Tire Models in qLPV Vehicle Dynamics
1. Static Linear Tire Model
2. Dynamic Linear Tire Model (with Load Transfer)
3. Pacejka Magic Formula (Nonlinear)
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# Add parent directory to path
parent_dir = os.path.dirname(os.path.abspath(__file__))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from vehiclemodels.vehicle_dynamics_qlpv import (
    vehicle_dynamics_qlpv,
    compute_tire_forces_linear,
    compute_tire_forces_load_transfer,
    compute_tire_forces_pacejka
)
from vehiclemodels.vehicle_parameters import setup_vehicle_parameters
from vehiclemodels.utils.steering_constraints import steering_constraints
from vehiclemodels.utils.acceleration_constraints import acceleration_constraints

def run_simulation(scenario_name, throttle_profile, steering_profile, dt=0.001, duration=5.0):
    """Run simulation for a specific scenario using 3 different tire configurations"""
    print(f"Running Scenario: {scenario_name}")
    
    p = setup_vehicle_parameters(vehicle_id='qcar')
    mass = getattr(p, 'm', 1.5) or 1.5
    h_cg = getattr(p, 'h_cg', 0.07) or 0.07
    print(f"   [+] Loaded QCar parameters: mass={mass}kg, h_cg={h_cg}m")
    
    t_span = np.arange(0, duration, dt)
    num_steps = len(t_span)
    
    # State: [X, Y, delta, vx, psi, r, vy]
    models = {
        'Static Linear': {'state': np.zeros(7)},
        'Dynamic Linear': {'state': np.zeros(7)},
        'Pacejka': {'state': np.zeros(7)}
    }
    
    results = {name: {
        'x': [], 'y': [], 'vx': [], 'vy': [], 'psi': [], 'r': [],
        'Fyf': [], 'Fyr': [], 'alpha_f': [], 'alpha_r': []
    } for name in models}

    # Logging at 0.01s interval
    log_interval = max(1, int(0.01 / dt))

    for name, config in models.items():
        state = config['state'].copy()
        
        for i in range(num_steps):
            throttle = throttle_profile[i]
            steering_rate = steering_profile[i]
            u_raw = [steering_rate, throttle * 3.0] # Scale throttle to acceleration
            
            # Extract current state for force calculation
            delta = state[2]
            vx = state[3]
            psi = state[4]
            r = state[5]
            vy = state[6]
            
            # Use same vx_safe as vehicle_dynamics_qlpv
            vx_safe = max(abs(vx), 0.1)
            alpha_f = delta - (vy + p.a * r) / vx_safe
            alpha_r = -(vy - p.b * r) / vx_safe
            
            # Manually compute forces for logging/manual simulation
            if name == 'Static Linear':
                # Force the use of the basic linear model for this baseline
                Fyf, Fyr = compute_tire_forces_linear(alpha_f, alpha_r, p.Cf, p.Cr)
                
                # Apply constraints and kinematic/dynamic logic similar to core model
                steer_rate = steering_constraints(delta, u_raw[0], p.steering)
                accel = acceleration_constraints(vx, u_raw[1], p.longitudinal)
                
                if abs(vx) < 0.1:
                    # Kinematic model
                    lwb = p.a + p.b
                    beta = np.arctan2(p.b * np.tan(delta), lwb) if abs(delta) > 0.001 else 0.0
                    vx_dot = accel - (9.81 * 0.015) * np.sign(vx) if abs(vx) > 0.01 else 0.0
                    f = [
                        vx * np.cos(psi + beta),
                        vx * np.sin(psi + beta),
                        steer_rate,
                        vx_dot,
                        vx / lwb * np.tan(delta) * np.cos(beta) if abs(vx) > 0.01 else 0.0,
                        0.0, 0.0
                    ]
                else:
                    # Dynamic model
                    vx_dot = accel - (9.81 * 0.015) * np.sign(vx) + r*vy - (Fyf * np.sin(delta))/p.m
                    vy_dot = (Fyr + Fyf * np.cos(delta))/p.m - r*vx
                    psi_dot = r
                    r_dot = (p.a * Fyf * np.cos(delta) - p.b * Fyr) / p.I_z
                    X_dot = vx * np.cos(psi) - vy * np.sin(psi)
                    Y_dot = vx * np.sin(psi) + vy * np.cos(psi)
                    f = [X_dot, Y_dot, steer_rate, vx_dot, psi_dot, r_dot, vy_dot]
            elif name == 'Dynamic Linear':
                Fyf, Fyr = compute_tire_forces_load_transfer(alpha_f, alpha_r, p, u_raw[1])
                f = vehicle_dynamics_qlpv(state, u_raw, p, use_pacejka=False)
            else: # Pacejka
                Fyf, Fyr = compute_tire_forces_pacejka(alpha_f, alpha_r, p, vx, u_raw[1])
                f = vehicle_dynamics_qlpv(state, u_raw, p, use_pacejka=True)
            
            # Step
            state += np.array(f) * dt
            
            if np.isnan(state).any():
                print(f"   [!] Simulation diverged for {name} at t={i*dt:.3f}s")
                break
                
            # Log at 100Hz
            if i % log_interval == 0:
                results[name]['x'].append(state[0])
                results[name]['y'].append(state[1])
                results[name]['vx'].append(state[3])
                results[name]['vy'].append(state[6])
                results[name]['psi'].append(state[4])
                results[name]['r'].append(state[5])
                results[name]['Fyf'].append(Fyf)
                results[name]['Fyr'].append(Fyr)
                results[name]['alpha_f'].append(alpha_f)
                results[name]['alpha_r'].append(alpha_r)

    # Convert to numpy and handle early stoppage
    t_log = np.arange(0, duration, 0.01)
    for name in results:
        for key in results[name]:
            data = results[name][key]
            if len(data) < len(t_log):
                results[name][key] = np.pad(data, (0, len(t_log) - len(data)), constant_values=np.nan)
            else:
                results[name][key] = np.array(data[:len(t_log)])

    return t_log, results

def plot_comparison(t, results, title):
    fig, axs = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(title, fontsize=16)
    
    # Trajectory
    for name, data in results.items():
        axs[0, 0].plot(data['x'], data['y'], label=name)
    axs[0, 0].set_title('Trajectory (X-Y)')
    axs[0, 0].set_xlabel('X [m]')
    axs[0, 0].set_ylabel('Y [m]')
    axs[0, 0].legend()
    axs[0, 0].grid(True)
    axs[0, 0].axis('equal')
    
    # Velocity
    for name, data in results.items():
        axs[0, 1].plot(t, data['vx'], label=name)
    axs[0, 1].set_title('Longitudinal Velocity (vx)')
    axs[0, 1].set_xlabel('Time [s]')
    axs[0, 1].set_ylabel('vx [m/s]')
    axs[0, 1].legend()
    axs[0, 1].grid(True)
    
    # Front Tire Force
    for name, data in results.items():
        axs[1, 0].plot(t, data['Fyf'], label=name)
    axs[1, 0].set_title('Front Lateral Tire Force (Fyf)')
    axs[1, 0].set_xlabel('Time [s]')
    axs[1, 0].set_ylabel('Force [N]')
    axs[1, 0].legend()
    axs[1, 0].grid(True)
    
    # Rear Tire Force
    for name, data in results.items():
        axs[1, 1].plot(t, data['Fyr'], label=name)
    axs[1, 1].set_title('Rear Lateral Tire Force (Fyr)')
    axs[1, 1].set_xlabel('Time [s]')
    axs[1, 1].set_ylabel('Force [N]')
    axs[1, 1].legend()
    axs[1, 1].grid(True)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Save plot
    filename = title.lower().replace(' ', '_').replace('-', '_') + ".png"
    plt.savefig(filename)
    print(f"Saved plot to {filename}")
    plt.show()

if __name__ == "__main__":
    duration = 5.0
    dt = 0.001
    steps = int(duration / dt)
    
    # --- Scenario 1: Steady Turn ---
    throttle_1 = np.ones(steps) * 0.1
    throttle_1[:int(0.5/dt)] = 0.5 # Initial push
    steering_1 = np.zeros(steps)
    steering_1[int(0.5/dt):int(1.0/dt)] = 0.4 # Slowly start turning
    
    t1, res1 = run_simulation("Steady Turn", throttle_1, steering_1, dt, duration)
    plot_comparison(t1, res1, "Tire Model Comparison - Steady Turn")
    
    # --- Scenario 2: Acceleration Turn ---
    # Accelerating while mid-turn to see load transfer effect (reducing front grip, increasing rear)
    throttle_2 = np.ones(steps) * 0.3
    throttle_2[int(1.5/dt):int(3.5/dt)] = 0.8 # Sudden acceleration mid-turn
    steering_2 = np.zeros(steps)
    steering_2[int(0.5/dt):int(1.5/dt)] = 0.4 # Start turn
    steering_2[int(1.5/dt):] = 0.4 # Keep steering constant to see path drift
    
    t2, res2 = run_simulation("Acceleration Turn", throttle_2, steering_2, dt, duration)
    plot_comparison(t2, res2, "Tire Model Comparison - Acceleration Turn")
