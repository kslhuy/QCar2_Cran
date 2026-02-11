"""
Diagnose the tau parameter fitting issue
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read data
csv_file = r"j:\Qcar_Development\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar\qcar\observer_recordings\dist_luenberger_v1_20260211_115706.csv"
df = pd.read_csv(csv_file)

print(f"Data diagnosis")
print("="*80)

# Extract relevant data
time = df['time'].values
accel_0 = df['true_acceleration_0'].values
vel_0 = df['true_velocity_0'].values if 'true_velocity_0' in df.columns else None

# Calculate time step
dt = np.mean(np.diff(time))

# Calculate jerk
jerk_0 = np.gradient(accel_0, dt)

print(f"\nVehicle 0 Statistics:")
print(f"  Time range: {time[0]:.2f} to {time[-1]:.2f} s")
print(f"  Velocity - min: {np.nanmin(vel_0):.4f}, max: {np.nanmax(vel_0):.4f} m/s")
print(f"  Acceleration - min: {np.nanmin(accel_0):.4f}, max: {np.nanmax(accel_0):.4f} m/s²")
print(f"  Jerk - min: {np.nanmin(jerk_0):.4f}, max: {np.nanmax(jerk_0):.4f} m/s³")

# Create diagnostic plots
fig, axes = plt.subplots(4, 1, figsize=(12, 12))

# Plot 1: Velocity
if vel_0 is not None:
    axes[0].plot(time - time[0], vel_0, 'b-', linewidth=1)
    axes[0].set_ylabel('Velocity [m/s]')
    axes[0].set_title('Vehicle 0 - Velocity over Time')
    axes[0].grid(True, alpha=0.3)

# Plot 2: Acceleration
axes[1].plot(time - time[0], accel_0, 'g-', linewidth=1)
axes[1].axhline(y=0, color='r', linestyle='--', alpha=0.5)
axes[1].set_ylabel('Acceleration [m/s²]')
axes[1].set_title('Vehicle 0 - Acceleration over Time')
axes[1].grid(True, alpha=0.3)

# Plot 3: Jerk
axes[2].plot(time - time[0], jerk_0, 'r-', linewidth=0.5, alpha=0.7)
axes[2].axhline(y=0, color='k', linestyle='--', alpha=0.5)
axes[2].set_ylabel('Jerk [m/s³]')
axes[2].set_title('Vehicle 0 - Jerk (ȧ) over Time')
axes[2].grid(True, alpha=0.3)

# Plot 4: Phase diagram - model relationship
# According to model: ȧ = -1/τ·a + 1/τ·u
# With fixed u=0.15: ȧ = -1/τ·a + 0.15/τ
# This should show: ȧ vs a should be roughly linear with negative slope -1/τ
axes[3].scatter(accel_0, jerk_0, alpha=0.3, s=10)
axes[3].set_xlabel('Acceleration a [m/s²]')
axes[3].set_ylabel('Jerk ȧ [m/s³]')
axes[3].set_title('Phase Diagram: ȧ vs a (should show linear trend with negative slope)')
axes[3].grid(True, alpha=0.3)
axes[3].axhline(y=0, color='k', linestyle='--', alpha=0.3)
axes[3].axvline(x=0, color='k', linestyle='--', alpha=0.3)

plt.tight_layout()
plt.savefig(r'j:\Qcar_Development\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar\qcar\observer_recordings\data_diagnosis.png', 
            dpi=150, bbox_inches='tight')
print("\nDiagnostic plot saved")

# Analyze the steady-state behavior
# When platoon is moving at constant speed, velocity should be about 1 m/s
# and acceleration should be close to 0
print(f"\nSteady-state analysis:")
if vel_0 is not None:
    # Find periods where velocity > 0.5 m/s (moving)
    moving_mask = vel_0 > 0.5
    if np.any(moving_mask):
        print(f"  During moving period (v > 0.5 m/s):")
        print(f"    Mean velocity: {np.mean(vel_0[moving_mask]):.4f} m/s")
        print(f"    Mean acceleration: {np.mean(accel_0[moving_mask]):.4f} m/s²")
        print(f"    Std acceleration: {np.std(accel_0[moving_mask]):.4f} m/s²")
    
    # Find steady state (low variation in velocity)
    window_size = 20
    vel_std_rolling = pd.Series(vel_0).rolling(window=window_size).std().values
    steady_mask = vel_std_rolling < 0.05  # Low variation
    if np.any(steady_mask):
        print(f"  During steady-state period (low velocity variation):")
        print(f"    Mean acceleration: {np.mean(accel_0[steady_mask]):.4f} m/s²")
        print(f"    Std acceleration: {np.std(accel_0[steady_mask]):.4f} m/s²")

# Check the model assumption
# For fixed control input u=0.15 and steady state (ȧ≈0), we should have: a ≈ u = 0.15
print(f"\nModel validation:")
print(f"  Expected steady-state acceleration (a ≈ u): 0.15 m/s²")
print(f"  Actual mean acceleration: {np.mean(accel_0):.4f} m/s²")
print(f"  Discrepancy: The model ȧ = -1/τ·a + 1/τ·u might not match the actual dynamics")
print(f"\nPossible issues:")
print(f"  1. The actual control input might not be constant 0.15")
print(f"  2. There might be additional dynamics (friction, air resistance)")
print(f"  3. The model might need to include additional terms")

plt.show()
