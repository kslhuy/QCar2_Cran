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

def run_simulation(scenario_name, throttle_profile, steering_profile, dt=0.001, duration=5.0):
    """Run simulation for a specific scenario using 3 different tire configurations"""
    print(f"Running Scenario: {scenario_name}")
    
    p = setup_vehicle_parameters(vehicle_id='qcar')
    mass = getattr(p, 'm', 1.5) or 1.5
    h_cg = getattr(p, 'h_cg', 0.07) or 0.07
    print(f"   [+] Loaded QCar parameters: mass={mass}kg, h_cg={h_cg}m")
    print(f"       I_z={getattr(p, 'I_z', 'N/A')}, L={getattr(p, 'l', 'N/A')}, W={getattr(p, 'w', 'N/A')}")
    print(f"       Cf={getattr(p, 'Cf', 'N/A')}, Cr={getattr(p, 'Cr', 'N/A')}")
    if hasattr(p, 'tire'):
        print(f"       Tire Params: p_ky1={getattr(p.tire, 'p_ky1', 'N/A')}, p_dy1={getattr(p.tire, 'p_dy1', 'N/A')}")
        print(f"                    p_cy1={getattr(p.tire, 'p_cy1', 'N/A')}, p_ey1={getattr(p.tire, 'p_ey1', 'N/A')}")
    
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
            
            # Use same dynamics framework for all models, only tire force calculation differs
            if name == 'Static Linear':
                Fyf, Fyr = compute_tire_forces_linear(alpha_f, alpha_r, p.Cf, p.Cr)
                f = vehicle_dynamics_qlpv(state, u_raw, p, tire_mode='static_linear')
            elif name == 'Dynamic Linear':
                Fyf, Fyr = compute_tire_forces_load_transfer(alpha_f, alpha_r, p, u_raw[1])
                f = vehicle_dynamics_qlpv(state, u_raw, p, tire_mode='dynamic_linear')
            else:  # Pacejka
                Fyf, Fyr = compute_tire_forces_pacejka(alpha_f, alpha_r, p, vx, u_raw[1])
                f = vehicle_dynamics_qlpv(state, u_raw, p, tire_mode='pacejka')
            
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
    fig, axs = plt.subplots(3, 2, figsize=(16, 14))
    fig.suptitle(title, fontsize=16)
    
    # Trajectory
    for name, data in results.items():
        axs[0, 0].plot(data['x'], data['y'], label=name, linewidth=2)
    axs[0, 0].set_title('Trajectory (X-Y)')
    axs[0, 0].set_xlabel('X [m]')
    axs[0, 0].set_ylabel('Y [m]')
    axs[0, 0].legend()
    axs[0, 0].grid(True)
    axs[0, 0].axis('equal')
    
    # Longitudinal Velocity
    for name, data in results.items():
        axs[0, 1].plot(t, data['vx'], label=name, linewidth=2)
    axs[0, 1].set_title('Longitudinal Velocity (vx)')
    axs[0, 1].set_xlabel('Time [s]')
    axs[0, 1].set_ylabel('vx [m/s]')
    axs[0, 1].legend()
    axs[0, 1].grid(True)
    
    # Lateral Velocity
    for name, data in results.items():
        axs[1, 0].plot(t, data['vy'], label=name, linewidth=2)
    axs[1, 0].set_title('Lateral Velocity (vy)')
    axs[1, 0].set_xlabel('Time [s]')
    axs[1, 0].set_ylabel('vy [m/s]')
    axs[1, 0].legend()
    axs[1, 0].grid(True)
    
    # Slip Angles
    for name, data in results.items():
        axs[1, 1].plot(t, np.rad2deg(data['alpha_f']), label=f'{name} - Front', linewidth=2)
        axs[1, 1].plot(t, np.rad2deg(data['alpha_r']), label=f'{name} - Rear', linewidth=2, linestyle='--')
    axs[1, 1].set_title('Slip Angles (αf, αr)')
    axs[1, 1].set_xlabel('Time [s]')
    axs[1, 1].set_ylabel('Slip Angle [deg]')
    axs[1, 1].legend()
    axs[1, 1].grid(True)
    
    # Front Tire Force
    for name, data in results.items():
        axs[2, 0].plot(t, data['Fyf'], label=name, linewidth=2)
    axs[2, 0].set_title('Front Lateral Tire Force (Fyf)')
    axs[2, 0].set_xlabel('Time [s]')
    axs[2, 0].set_ylabel('Force [N]')
    axs[2, 0].legend()
    axs[2, 0].grid(True)
    
    # Rear Tire Force
    for name, data in results.items():
        axs[2, 1].plot(t, data['Fyr'], label=name, linewidth=2)
    axs[2, 1].set_title('Rear Lateral Tire Force (Fyr)')
    axs[2, 1].set_xlabel('Time [s]')
    axs[2, 1].set_ylabel('Force [N]')
    axs[2, 1].legend()
    axs[2, 1].grid(True)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.97])
    
    # Save plot
    filename = title.lower().replace(' ', '_').replace('-', '_') + ".png"
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"Saved plot to {filename}")
    plt.show()

def plot_combined_comparison(scenarios_data):
    """Plot all scenarios in a single comparison figure"""
    fig, axs = plt.subplots(4, 4, figsize=(24, 20))
    fig.suptitle('Complete Tire Model Comparison - All Scenarios', fontsize=18)
    
    colors = {'Static Linear': 'blue', 'Dynamic Linear': 'red', 'Pacejka': 'green'}
    
    for i, (scenario_name, (t, results)) in enumerate(scenarios_data.items()):
        row = i
        
        # Trajectory
        for name, data in results.items():
            axs[row, 0].plot(data['x'], data['y'], label=name, color=colors[name], linewidth=2)
        axs[row, 0].set_title(f'{scenario_name} - Trajectory')
        axs[row, 0].set_xlabel('X [m]')
        axs[row, 0].set_ylabel('Y [m]')
        if row == 0:
            axs[row, 0].legend()
        axs[row, 0].grid(True)
        axs[row, 0].axis('equal')
        
        # Longitudinal Velocity
        for name, data in results.items():
            axs[row, 1].plot(t, data['vx'], label=name, color=colors[name], linewidth=2)
        axs[row, 1].set_title(f'{scenario_name} - Long. Velocity')
        axs[row, 1].set_xlabel('Time [s]')
        axs[row, 1].set_ylabel('vx [m/s]')
        if row == 0:
            axs[row, 1].legend()
        axs[row, 1].grid(True)
        
        # Lateral Velocity
        for name, data in results.items():
            axs[row, 2].plot(t, data['vy'], label=name, color=colors[name], linewidth=2)
        axs[row, 2].set_title(f'{scenario_name} - Lat. Velocity')
        axs[row, 2].set_xlabel('Time [s]')
        axs[row, 2].set_ylabel('vy [m/s]')
        if row == 0:
            axs[row, 2].legend()
        axs[row, 2].grid(True)
        
        # Slip Angles
        for name, data in results.items():
            axs[row, 3].plot(t, np.rad2deg(data['alpha_f']), label=f'{name} - αf', color=colors[name], linewidth=2)
            axs[row, 3].plot(t, np.rad2deg(data['alpha_r']), label=f'{name} - αr', color=colors[name], linewidth=2, linestyle='--')
        axs[row, 3].set_title(f'{scenario_name} - Slip Angles')
        axs[row, 3].set_xlabel('Time [s]')
        axs[row, 3].set_ylabel('Slip Angle [deg]')
        if row == 0:
            axs[row, 3].legend()
        axs[row, 3].grid(True)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.97])
    plt.savefig("complete_tire_model_comparison.png", dpi=150, bbox_inches='tight')
    print("Saved combined comparison to complete_tire_model_comparison.png")
    plt.show()

if __name__ == "__main__":
    duration = 12.0
    dt = 0.001
    steps = int(duration / dt)
    
    # Store all scenarios for combined plot
    scenarios_data = {}
    
    print("="*60)
    print("ENHANCED TIRE MODEL COMPARISON - HIGH SPEED TESTS")
    print("="*60)
    
    # --- Scenario 1: High-Speed Steady Turn ---
    throttle_1 = np.ones(steps) * 0.2  # Much higher base throttle
    throttle_1[:int(1.0/dt)] = 1     # Strong initial acceleration
    steering_1 = np.zeros(steps)
    steering_1[int(1.0/dt):int(2.0/dt)] = 0.4  # Sharper turn
    steering_1[int(2.0/dt):] = 0.1             # Maintain turn
    
    t1, res1 = run_simulation("High-Speed Steady Turn", throttle_1, steering_1, dt, duration)
    plot_comparison(t1, res1, "Tire Model Comparison - High-Speed Steady Turn")
    scenarios_data["High-Speed Steady Turn"] = (t1, res1)
    
    # --- Scenario 2: Aggressive Acceleration Turn ---
    # Very aggressive acceleration while turning to stress tire models
    throttle_2 = np.ones(steps) * 0.4
    throttle_2[:int(0.8/dt)] = 1.0    # Very strong initial acceleration
    throttle_2[int(2.0/dt):int(4.0/dt)] = 1.0  # Extreme acceleration mid-turn
    steering_2 = np.zeros(steps)
    steering_2[int(0.8/dt):int(2.0/dt)] = 0.5  # Sharp turn
    steering_2[int(2.0/dt):] = 0.2             # Maintain sharp turn during acceleration
    
    t2, res2 = run_simulation("Aggressive Acceleration Turn", throttle_2, steering_2, dt, duration)
    plot_comparison(t2, res2, "Tire Model Comparison - Aggressive Acceleration Turn")
    scenarios_data["Aggressive Acceleration Turn"] = (t2, res2)
    
    # --- Scenario 3: Successive Lane Changes ---
    # Accelerate to speed, then perform multiple lane changes
    throttle_3 = np.zeros(steps)
    throttle_3[:int(2.0/dt)] = 1.0     # Accelerate for 2s
    throttle_3[int(2.0/dt):] = 0.2     # Maintain speed (cruise)
    
    steering_3 = np.zeros(steps)
    # Lane change 1 at t=3s
    # Sine wave pulse for lane change
    period = 1.0 # 1 second duration
    start_idx = int(3.0/dt)
    end_idx = int(4.0/dt)
    t_pulse = np.linspace(0, 2*np.pi, end_idx - start_idx)
    steering_3[start_idx:end_idx] = 0.5 * np.sin(t_pulse)
    
    # Lane change 2 (back) at t=6s
    start_idx = int(6.0/dt)
    end_idx = int(7.0/dt)
    steering_3[start_idx:end_idx] = -0.5 * np.sin(t_pulse)

    # Lane change 3 (again) at t=9s
    start_idx = int(9.0/dt)
    end_idx = int(10.0/dt)
    steering_3[start_idx:end_idx] = 0.5 * np.sin(t_pulse)
    
    t3, res3 = run_simulation("Successive Lane Changes", throttle_3, steering_3, dt, duration)
    plot_comparison(t3, res3, "Tire Model Comparison - Successive Lane Changes")
    scenarios_data["Successive Lane Changes"] = (t3, res3)
    
    # --- Scenario 4: Extreme Lateral Maneuvers (REMOVED) ---
    # throttle_4 = np.ones(steps) * 0.4
    # throttle_4[:int(0.5/dt)] = 1.8     # Very strong acceleration
    # throttle_4[int(2.5/dt):int(3.5/dt)] = -0.5  # Braking during turn
    # throttle_4[int(4.5/dt):int(5.5/dt)] = -0.8  # Heavy braking during opposite turn
    
    # steering_4 = np.zeros(steps)
    # # Create extreme alternating sharp turns
    # steering_4[int(1.0/dt):int(2.5/dt)] = 1.0    # Maximum right turn
    # steering_4[int(2.5/dt):int(4.0/dt)] = -1.0   # Maximum left turn  
    # steering_4[int(4.0/dt):int(5.5/dt)] = 1.2    # Even sharper right turn
    
    # t4, res4 = run_simulation("Extreme Lateral Maneuvers", throttle_4, steering_4, dt, duration)
    # plot_comparison(t4, res4, "Tire Model Comparison - Extreme Lateral Maneuvers")
    # scenarios_data["Extreme Lateral Maneuvers"] = (t4, res4)
    
    # Create combined comparison plot
    print("\n" + "="*60)
    print("GENERATING COMBINED COMPARISON PLOT...")
    print("="*60)
    plot_combined_comparison(scenarios_data)
    
    print("\n" + "="*60)
    print("ANALYSIS COMPLETE!")
    print("- All individual scenario plots saved")
    print("- Combined comparison plot saved as 'complete_tire_model_comparison.png'")
    print("- Check for significant differences in high-speed scenarios")
    print("="*60)
