"""
Analyze tau parameter from specific CSV file
"""
import sys
sys.path.append(r'j:\Qcar_Development\QCar2_Cran')

from analyze_tau_parameter import calculate_acceleration_derivative, fit_tau_least_squares
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

# Specify the CSV file to analyze
csv_file = r"j:\Qcar_Development\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar\qcar\observer_recordings\dist_luenberger_v1_20260211_115706.csv"

print(f"Analyzing file: {csv_file}")
print("="*80)

# Read data
df = pd.read_csv(csv_file)
print(f"Data rows: {len(df)}")
print(f"Columns: {len(df.columns)}")

# Calculate dt
time_col = df['time'].values
valid_times = time_col[~np.isnan(time_col)]
dt = np.mean(np.diff(valid_times))
print(f"Time step dt: {dt:.6f} s")
print(f"Total duration: {valid_times[-1] - valid_times[0]:.2f} s")

# Use vehicle 0 data
if 'true_acceleration_0' in df.columns:
    print(f"\nFound true_acceleration_0 column")
    acceleration = df['true_acceleration_0'].values
    
    # Vehicle 0 uses FIXED control input = 0.15
    control_input = np.full_like(acceleration, 0.15)
    
    print(f"\nData statistics:")
    print(f"  Acceleration - min: {np.nanmin(acceleration):.6f} m/s²")
    print(f"  Acceleration - max: {np.nanmax(acceleration):.6f} m/s²")
    print(f"  Acceleration - mean: {np.nanmean(acceleration):.6f} m/s²")
    print(f"  Acceleration - std: {np.nanstd(acceleration):.6f} m/s²")
    print(f"  Control input - fixed: 0.15 (throttle)")
    
    # Calculate jerk (acceleration derivative)
    jerk = calculate_acceleration_derivative(acceleration, dt)
    print(f"\nJerk statistics:")
    print(f"  Jerk - min: {np.nanmin(jerk):.6f} m/s³")
    print(f"  Jerk - max: {np.nanmax(jerk):.6f} m/s³")
    print(f"  Jerk - std: {np.nanstd(jerk):.6f} m/s³")
    
    # Fit tau parameter
    print(f"\n{'='*80}")
    print("FITTING τ PARAMETER")
    print(f"{'='*80}")
    tau, fig = fit_tau_least_squares(
        acceleration, jerk, control_input, dt,
        plot=True,
        title="Vehicle 0 (Leader) - τ Parameter Identification\n(Fixed Control Input u = 0.15)"
    )
    
    if tau is not None:
        print(f"\n{'='*80}")
        print(f"FINAL RESULT: τ = {tau:.4f} s")
        print(f"{'='*80}")
        
        # Compare with current value
        current_tau = 0.16
        print(f"\nComparison:")
        print(f"  Current τ in code: {current_tau:.4f} s")
        print(f"  Identified τ:      {tau:.4f} s")
        print(f"  Difference:        {abs(tau - current_tau):.4f} s ({abs(tau - current_tau)/current_tau*100:.1f}%)")
        
        if abs(tau - current_tau) > 0.02:
            print(f"\nRecommendation: Consider updating τ to {tau:.4f} s for better accuracy")
        else:
            print(f"\nCurrent τ value is close to the identified value, no change needed")
    
    if fig is not None:
        output_path = os.path.join(os.path.dirname(csv_file), 'tau_identification_vehicle0.png')
        fig.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"\nPlot saved to: {output_path}")
        plt.show()
        
else:
    print("Error: true_acceleration_0 column not found")
    print(f"Available columns: {df.columns.tolist()}")
