"""
τ (tau) parameter identification script

This script analyzes observer recording data to identify the time constant τ 
from the longitudinal model: ȧ = -1/τ·a + 1/τ·u

Method: Least squares fitting
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy import signal
import glob
import os

def calculate_acceleration_derivative(acceleration, dt):
    """
    Calculate acceleration derivative (jerk) using finite differences
    
    Args:
        acceleration: array of acceleration values
        dt: time step
        
    Returns:
        jerk: acceleration derivative (ȧ)
    """
    jerk = np.gradient(acceleration, dt)
    return jerk

def fit_tau_least_squares(acceleration, jerk, control_input, dt, plot=True, title=""):
    """
    Fit τ parameter using least squares: ȧ = -1/τ·a + 1/τ·u
    
    Rearrange: ȧ = -1/τ·a + 1/τ·u
    Or: ȧ·τ = -a + u
    Or: ȧ = (-a + u)/τ
    
    Define: b1 = -1/τ, b2 = 1/τ
    Then: ȧ = b1·a + b2·u
    
    We use least squares to find b1, b2, then recover τ
    """
    
    # Remove NaN values
    mask = ~(np.isnan(jerk) | np.isnan(acceleration) | np.isnan(control_input))
    
    # Also filter out some extreme values that might be noise
    # Keep data within reasonable ranges
    valid_accel = np.abs(acceleration[mask]) < 5  # m/s^2
    valid_jerk = np.abs(jerk[mask]) < 5  # m/s^3
    valid_control = np.abs(control_input[mask]) < 1  # normalized
    
    mask = mask.copy()
    mask[mask] = valid_accel & valid_jerk & valid_control
    
    if np.sum(mask) < 10:
        print(f"Warning: Too few valid samples ({np.sum(mask)})")
        return None
    
    a = acceleration[mask]
    j = jerk[mask]
    u = control_input[mask]
    
    # Build regression matrix: [a | u]
    X = np.column_stack([a, u])
    y = j
    
    # Solve least squares: (X^T X)^{-1} X^T y
    try:
        beta = np.linalg.lstsq(X, y, rcond=None)[0]
        b1, b2 = beta[0], beta[1]
        
        # Recover τ from b1 = -1/τ and b2 = 1/τ
        # Should have b1 ≈ -b2
        if b1 >= 0:
            print(f"Warning: b1 = {b1:.6f} >= 0 (should be negative)")
        
        tau_from_b1 = -1.0 / b1 if b1 != 0 else None
        tau_from_b2 = 1.0 / b2 if b2 != 0 else None
        
        print(f"\n{title}")
        print(f"Regression coefficients: b1={b1:.6f}, b2={b2:.6f}")
        print(f"τ from b1 (-1/τ): {tau_from_b1:.4f} s")
        print(f"τ from b2 (1/τ):  {tau_from_b2:.4f} s")
        
        # Average the two estimates
        if tau_from_b1 is not None and tau_from_b2 is not None:
            tau = (tau_from_b1 + tau_from_b2) / 2.0
            print(f"Average τ: {tau:.4f} s")
        elif tau_from_b1 is not None:
            tau = tau_from_b1
            print(f"Using τ from b1: {tau:.4f} s")
        else:
            tau = tau_from_b2
            print(f"Using τ from b2: {tau:.4f} s")
        
        # Calculate R² (coefficient of determination)
        y_pred = X @ beta
        ss_res = np.sum((y - y_pred) ** 2)
        ss_tot = np.sum((y - np.mean(y)) ** 2)
        r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0
        print(f"R² score: {r_squared:.6f}")
        
        # Plot if requested
        if plot:
            fig, axes = plt.subplots(3, 1, figsize=(12, 10))
            
            # Plot 1: Actual vs predicted jerk
            time = np.arange(len(a)) * dt
            axes[0].plot(time, j, 'b-', label='Actual jerk (measured)', linewidth=1.5)
            axes[0].plot(time, y_pred, 'r--', label='Predicted jerk (model)', linewidth=1.5, alpha=0.8)
            axes[0].set_ylabel('Jerk [m/s³]')
            axes[0].set_title(f'{title} - Jerk Comparison (R² = {r_squared:.4f})')
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)
            
            # Plot 2: Control input and acceleration
            axes[1].plot(time, a, 'g-', label='Acceleration a', linewidth=1.5)
            axes[1].plot(time, u, 'm-', label='Control input u', linewidth=1.5, alpha=0.8)
            axes[1].set_ylabel('Acceleration / Control')
            axes[1].set_title('Inputs to the Model')
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)
            
            # Plot 3: Error distribution
            residual = y - y_pred
            axes[2].scatter(y_pred, residual, alpha=0.5, s=10)
            axes[2].axhline(y=0, color='r', linestyle='--')
            axes[2].set_xlabel('Predicted Jerk')
            axes[2].set_ylabel('Residual (Actual - Predicted)')
            axes[2].set_title('Residual Plot')
            axes[2].grid(True, alpha=0.3)
            
            plt.tight_layout()
            return tau, fig
        
        return tau
        
    except np.linalg.LinAlgError:
        print("Error in least squares fit")
        return None

def analyze_csv_file(csv_path, vehicle_id=0):
    """
    Analyze a single CSV file to identify τ for a specific vehicle
    
    Args:
        csv_path: Path to CSV file
        vehicle_id: Vehicle ID to analyze (default 0 for leader)
    """
    print(f"\n{'='*60}")
    print(f"Analyzing: {os.path.basename(csv_path)}")
    print(f"Focus on Vehicle {vehicle_id}")
    print(f"{'='*60}")
    
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return None
    
    # Check available columns
    print(f"\nAvailable columns ({len(df.columns)}): ")
    print(f"{df.columns.tolist()[:10]}...")  # Print first 10 columns
    
    results = {}
    
    # Get dt (time step)
    time_col = df['time'].values
    # Convert to float if needed
    try:
        time_col = time_col.astype(float)
    except:
        print(f"Warning: Could not convert time column to float")
    
    # Filter out NaN values
    valid_times = time_col[~np.isnan(time_col)]
    if len(valid_times) < 2:
        print(f"Error: Not enough valid time values")
        return None
    
    dt = np.mean(np.diff(valid_times))
    
    print(f"Time step dt: {dt:.4f} s")
    print(f"Number of samples: {len(df)}")
    
    # Analyze vehicle 0 (leader with fixed control input)
    print(f"\n--- Vehicle {vehicle_id} (Leader with fixed control input = 0.15) ---")
    
    # Look for acceleration column for vehicle 0
    # Try different column name patterns
    accel_candidates = [
        f'true_acceleration_{vehicle_id}',
        f'fleet_a_{vehicle_id}',
        f'dynamics_a{vehicle_id}',
        f'acceleration_{vehicle_id}'
    ]
    
    accel_col = None
    
    for col in accel_candidates:
        if col in df.columns:
            accel_col = col
            print(f"Found acceleration column: {accel_col}")
            break
    
    if accel_col is None:
        print(f"Error: Could not find acceleration column")
        print(f"  Acceleration candidates: {accel_candidates}")
        return None
    
    acceleration = df[accel_col].values
    
    # Vehicle 0 uses FIXED control input = 0.15 (triggered after platoon starts)
    # Not from CSV control_input column (which is for observer's own vehicle)
    control_input = np.full_like(acceleration, 0.15)
    print(f"Using FIXED control input: 0.15 (constant for vehicle 0)")
    
    print(f"Data statistics:")
    print(f"  Acceleration - min: {np.nanmin(acceleration):.4f}, max: {np.nanmax(acceleration):.4f}")
    print(f"  Control input - min: {np.nanmin(control_input):.4f}, max: {np.nanmax(control_input):.4f}")
    
    # Calculate jerk
    jerk = calculate_acceleration_derivative(acceleration, dt)
    
    # Fit τ
    tau = fit_tau_least_squares(acceleration, jerk, control_input, dt, 
                               plot=False, title=f"Vehicle {vehicle_id} (Leader)")
    
    if tau is not None:
        results[f'vehicle_{vehicle_id}'] = tau
    
    return results

def analyze_all_recordings():
    """
    Analyze all recording files in observer_recordings directory
    Focus on vehicle 0 (leader) which has fixed control input
    """
    # Check multiple possible recording directories
    recording_dirs = [
        r"j:\Qcar_Development\QCar2_Cran\observer_recordings",
        r"j:\Qcar_Development\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar\qcar\observer_recordings"
    ]
    
    csv_files = []
    for recording_dir in recording_dirs:
        if os.path.exists(recording_dir):
            csv_files.extend(glob.glob(os.path.join(recording_dir, "*.csv")))
    
    if not csv_files:
        print(f"No CSV files found in {recording_dir}")
        return
    
    all_results = {}
    
    # Analyze last 3 files
    for csv_file in sorted(csv_files)[-3:]:
        results = analyze_csv_file(csv_file, vehicle_id=0)
        if results:
            all_results[os.path.basename(csv_file)] = results
    
    # Summary
    print(f"\n{'='*60}")
    print("SUMMARY OF τ ESTIMATES (Vehicle 0 - Leader)")
    print(f"{'='*60}")
    
    all_tau_values = []
    for filename, vehicles in all_results.items():
        print(f"\n{filename}")
        for vehicle, tau in vehicles.items():
            print(f"  {vehicle}: τ = {tau:.4f} s")
            all_tau_values.append(tau)
    
    if all_tau_values:
        print(f"\n{'='*60}")
        print(f"Statistical Summary:")
        print(f"  Mean:   {np.mean(all_tau_values):.4f} s")
        print(f"  Median: {np.median(all_tau_values):.4f} s")
        print(f"  Std:    {np.std(all_tau_values):.4f} s")
        print(f"  Min:    {np.min(all_tau_values):.4f} s")
        print(f"  Max:    {np.max(all_tau_values):.4f} s")
        print(f"\nRecommendation: Use τ = {np.median(all_tau_values):.4f} s")
        print(f"{'='*60}")
    
    return all_results

if __name__ == "__main__":
    all_results = analyze_all_recordings()
    
    # Example: detailed analysis of single file with plotting
    print("\n\nDetailed analysis of latest recording with plots:")
    
    # Check multiple possible recording directories
    recording_dirs = [
        r"j:\Qcar_Development\QCar2_Cran\observer_recordings",
        r"j:\Qcar_Development\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar\qcar\observer_recordings"
    ]
    
    csv_files = []
    for recording_dir in recording_dirs:
        if os.path.exists(recording_dir):
            csv_files.extend(glob.glob(os.path.join(recording_dir, "*.csv")))
    
    if not csv_files:
        print("No CSV files found in any recording directory")
    else:
        latest_file = sorted(csv_files)[-1]
        print(f"Latest file: {os.path.basename(latest_file)}")
        print(f"Full path: {latest_file}")
        
        df = pd.read_csv(latest_file)
        print(f"Data rows: {len(df)}")
        
        if len(df) == 0:
            print("Warning: CSV file has no data!")
        else:
            # Get dt
            time_col = df['time'].values
            # Convert to float if needed
            try:
                time_col = time_col.astype(float)
            except:
                print(f"Warning: Could not convert time column to float")
            
            valid_times = time_col[~np.isnan(time_col)]
            if len(valid_times) >= 2:
                dt = np.mean(np.diff(valid_times))
            else:
                dt = 0.01  # Default fallback
            
            print(f"Time step dt: {dt:.6f} s")
            print(f"Available columns for vehicle 0: ")
            accel_col = None
            
            # Find acceleration column for vehicle 0
            for col in df.columns:
                if 'acceleration' in col.lower() and '0' in col:
                    print(f"  {col}")
            
            # Try to find the right acceleration column
            for col in ['true_acceleration_0', 'fleet_a_0', 'dynamics_a0']:
                if col in df.columns:
                    accel_col = col
                    break
            
            if accel_col:
                print(f"\nUsing acceleration column: {accel_col}")
                print(f"Using FIXED control input: 0.15 (constant for vehicle 0)")
                acceleration = df[accel_col].values
                # Vehicle 0 uses FIXED control input = 0.15
                control_input = np.full_like(acceleration, 0.15)
                jerk = calculate_acceleration_derivative(acceleration, dt)
                
                tau, fig = fit_tau_least_squares(acceleration, jerk, control_input, dt, 
                                                plot=True, title="Vehicle 0 (Leader, Fixed u=0.15) - Detailed Analysis")
                
                if fig is not None:
                    output_dir = os.path.dirname(latest_file)
                    fig.savefig(os.path.join(output_dir, 'tau_analysis_vehicle0.png'), dpi=150)
                    print(f"\nPlot saved to: {os.path.join(output_dir, 'tau_analysis_vehicle0.png')}")
                    plt.show()
            else:
                print(f"Could not find acceleration column in the data")
                print(f"Available columns: {df.columns.tolist()}")
