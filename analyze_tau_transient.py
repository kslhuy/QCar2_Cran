"""
Analyze tau parameter using TRANSIENT phase (startup acceleration) only
在稳态时，加速度≈0是正常的（驱动力=阻力）。只有在过渡阶段才能看到τ的效果。
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Read data
csv_file = r"j:\Qcar_Development\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar\qcar\observer_recordings\dist_luenberger_v1_20260211_115706.csv"
df = pd.read_csv(csv_file)

print("Analyzing τ parameter from transient response")
print("="*80)

# Extract data
time = df['time'].values - df['time'].values[0]  # Relative time
accel = df['true_acceleration_0'].values
vel = df['true_velocity_0'].values

dt = np.mean(np.diff(time + df['time'].values[0]))

print(f"Data: {len(time)} samples, dt={dt:.4f} s")

# Identify startup transient phase
# Look for when velocity starts increasing from near-zero
# Find the first significant acceleration event

# Method: Find where velocity increases significantly
vel_diff = np.diff(vel)
vel_diff = np.append(vel_diff, vel_diff[-1])  # Pad to same length

# Find startup: velocity goes from near-zero to ~1 m/s
# Strategy: find the first point where velocity starts rising and doesn't drop back
startup_candidates = np.where((vel > 0.01) & (vel_diff > 0.001))[0]

if len(startup_candidates) > 0:
    # Find continuous acceleration phase
    startup_idx = startup_candidates[0]
    
    # Find when velocity reaches plateau (around 1 m/s)
    plateau_candidates = np.where(vel > 0.95)[0]
    if len(plateau_candidates) > 0:
        plateau_idx = plateau_candidates[0]
    else:
        plateau_idx = min(startup_idx + 100, len(vel) - 1)
    
    #Use a bit before startup for initial condition
    start_idx = max(0, startup_idx - 10)
    end_idx = min(len(time), plateau_idx + 20)
    
    print(f"\nTransient phase identified:")
    print(f"  Time range: {time[start_idx]:.2f} - {time[end_idx]:.2f} s")
    print(f"  Duration: {time[end_idx] - time[start_idx]:.2f} s")
    print(f"  Samples: {end_idx - start_idx}")
    
    # Extract transient data
    time_trans = time[start_idx:end_idx]
    accel_trans = accel[start_idx:end_idx]
    vel_trans = vel[start_idx:end_idx]
    
    print(f"\nTransient data statistics:")
    print(f"  Velocity: {vel_trans[0]:.4f} → {vel_trans[-1]:.4f} m/s")
    print(f"  Acceleration: min={np.min(accel_trans):.4f}, max={np.max(accel_trans):.4f} m/s²")
    
    # Check if this is a good transient phase
    if vel_trans[-1] - vel_trans[0] < 0.5:
        print("\n⚠ Warning: Velocity change is too small, may not be a good transient phase")
        print("  Trying alternative detection method...")
        
        # Alternative: find largest acceleration peak
        accel_peak_idx = np.argmax(accel)
        start_idx = max(0, accel_peak_idx - 30)
        end_idx = min(len(time), accel_peak_idx + 50)
        time_trans = time[start_idx:end_idx]
        accel_trans = accel[start_idx:end_idx]
        vel_trans = vel[start_idx:end_idx]
        
        print(f"\n  Alternative phase:")
        print(f"    Time range: {time[start_idx]:.2f} - {time[end_idx]:.2f} s")
        print(f"    Velocity: {vel_trans[0]:.4f} → {vel_trans[-1]:.4f} m/s")
        print(f"    Acceleration: min={np.min(accel_trans):.4f}, max={np.max(accel_trans):.4f} m/s²")
    
    # Method 1: Fit first-order response model
    # For step input u=0.15 at t=0, with initial conditions v(0)≈0, a(0)≈0:
    # The acceleration response to step input should be:
    # a(t) = a_ss * (1 - exp(-t/τ))
    # where a_ss is steady-state acceleration
    
    # But in reality, steady-state a ≈ 0 due to drag forces
    # Better model: During startup with constant throttle u:
    # ȧ + a/τ = u/τ
    # Solution: a(t) = u*(1 - exp(-t/τ))
    
    # Fit exponential rise
    def accel_response(t, tau, a_max):
        """First-order response: a(t) = a_max * (1 - exp(-t/tau))"""
        return a_max * (1 - np.exp(-t / tau))
    
    try:
        # Fit using curve_fit
        t_fit = time_trans - time_trans[0]  # Start from 0
        
        # Initial guess
        p0 = [0.2, np.max(accel_trans)]  # tau=0.2s, a_max=peak
        
        # Fit
        popt, pcov = curve_fit(accel_response, t_fit, accel_trans, p0=p0,
                               bounds=([0.01, 0.1], [2.0, 5.0]),
                               maxfev=5000)
        
        tau_fit, a_max_fit = popt
        
        print(f"\n{'='*80}")
        print(f"CURVE FITTING RESULT (First-Order Response Model)")
        print(f"{'='*80}")
        print(f"  τ (time constant):        {tau_fit:.4f} s")
        print(f"  a_max (peak acceleration): {a_max_fit:.4f} m/s²")
        
        # Calculate R²
        a_pred = accel_response(t_fit, tau_fit, a_max_fit)
        ss_res = np.sum((accel_trans - a_pred)**2)
        ss_tot = np.sum((accel_trans - np.mean(accel_trans))**2)
        r_squared = 1 - (ss_res / ss_tot)
        print(f"  R² score:                  {r_squared:.4f}")
        
        # Standard errors
        perr = np.sqrt(np.diag(pcov))
        print(f"  τ standard error:          {perr[0]:.4f} s")
        
        # Plot
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        
        # Plot 1: Full data
        axes[0].plot(time, vel, 'b-', label='Velocity', linewidth=1.5)
        axes[0].axvspan(time[start_idx], time[end_idx], alpha=0.2, color='yellow', label='Transient Phase')
        axes[0].set_ylabel('Velocity [m/s]')
        axes[0].set_title('Full Dataset - Velocity Profile')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        
        # Plot 2: Acceleration during transient
        axes[1].plot(t_fit, accel_trans, 'go', label='Measured', markersize=4, alpha=0.6)
        axes[1].plot(t_fit, a_pred, 'r-', label=f'Fitted (τ={tau_fit:.3f}s)', linewidth=2)
        axes[1].set_ylabel('Acceleration [m/s²]')
        axes[1].set_xlabel('Time [s]')
        axes[1].set_title(f'Transient Phase - Acceleration Response (R²={r_squared:.4f})')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        # Plot 3: Full acceleration
        axes[2].plot(time, accel, 'g-', linewidth=1, alpha=0.7)
        axes[2].axvspan(time[start_idx], time[end_idx], alpha=0.2, color='yellow')
        axes[2].set_ylabel('Acceleration [m/s²]')
        axes[2].set_xlabel('Time [s]')
        axes[2].set_title('Full Dataset - Acceleration Profile')
        axes[2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        output_path = r'j:\Qcar_Development\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar\qcar\observer_recordings\tau_transient_analysis.png'
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"\nPlot saved to: {output_path}")
        plt.close()  # Close instead of show
        
        # Final recommendation
        print(f"\n{'='*80}")
        print(f"RECOMMENDATION")
        print(f"{'='*80}")
        current_tau = 0.16
        print(f"  Current τ in code:  {current_tau:.4f} s")
        print(f"  Identified τ:       {tau_fit:.4f} s")
        print(f"  Difference:         {abs(tau_fit - current_tau):.4f} s")
        
        if r_squared > 0.7:
            if abs(tau_fit - current_tau) > 0.05:
                print(f"\n  ✓ Good fit (R²={r_squared:.2f}). Consider updating to τ = {tau_fit:.4f} s")
            else:
                print(f"\n  ✓ Current value is adequate (difference < 0.05 s)")
        else:
            print(f"\n  ⚠ Warning: Low R² ({r_squared:.2f}), fit quality is questionable")
            print(f"    Consider using τ ≈ {tau_fit:.4f} s but with caution")
        
        # plt.show()  # Uncomment for interactive display
        
    except Exception as e:
        print(f"Error in curve fitting: {e}")
        print("Could not determine τ from transient response")
        
else:
    print("Could not identify clear transient phase in the data")
    print("Transient phase is needed to estimate τ parameter")
