#!/usr/bin/env python3
"""
Analyze throttle to velocity relationship from ThrottleSequenceController test run

Steps:
1. Read vehicle 0 velocity data from CSV file
2. Extract speed and timestamp from throttle sequence test
3. Calculate steady-state velocity for each throttle segment
4. Fit throttle = F(velocity) relationship
5. Save results to new CSV file
"""

import pandas as pd
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from scipy.optimize import curve_fit
import warnings
warnings.filterwarnings('ignore')


class ThrottleVelocityAnalyzer:
    """Analyze throttle to velocity mapping from test data"""
    
    def __init__(self, csv_file, output_dir=None):
        """
        Initialize analyzer
        
        Args:
            csv_file: Path to CSV file from test run
            output_dir: Output directory for results (default: same as csv_file)
        """
        self.csv_file = Path(csv_file)
        self.output_dir = Path(output_dir) if output_dir else self.csv_file.parent
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # ThrottleSequenceController configuration from config file
        # Vehicle 0 throttle sequence: [0.05, 0.06, 0.07, 0.8, 0.9, 0.1, 0.11, 0.12, 0.13, 0.15, 0.16, 0.17, 0.18]
        # Each throttle value lasts 5 seconds
        # Note: 0.8 and 0.9 are typos - should be 0.08 and 0.09
        # self.throttle_sequence = [0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.11, 0.12, 0.13, 0.15, 0.16, 0.17, 0.18]
        self.throttle_sequence = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.11, 0.12, 0.13, 0.15, 0.16, 0.17, 0.18, 0.19, 0.2, 0.21, 0.22, 0.23, 0.24, 0.25]
        self.segment_duration = 15.0  # seconds
        
        # Auto-correction for common typos in throttle sequence
        self.auto_correct_throttle = True
        
        self.data = None
        self.segments_data = []
        self.calibration_points = []
        
    def load_data(self):
        """Load CSV file and extract vehicle 0 velocity"""
        print(f"\n[1] Loading CSV file: {self.csv_file}")
        
        self.data = pd.read_csv(self.csv_file)
        print(f"    [OK] Loaded {len(self.data)} samples")
        print(f"    [OK] Columns: {list(self.data.columns)[:10]}...")
        
        # Check for vehicle 0 velocity column
        if 'true_velocity_0' in self.data.columns:
            print(f"    [OK] Found 'true_velocity_0' column")
        elif 'fleet_v_0' in self.data.columns:
            print(f"    [OK] Found 'fleet_v_0' column (using as vehicle 0 velocity)")
            self.data['true_velocity_0'] = self.data['fleet_v_0']
        else:
            print(f"    [ERROR] Cannot find vehicle 0 velocity column!")
            print(f"    Available columns: {list(self.data.columns)}")
            raise ValueError("Vehicle 0 velocity column not found")
        
        # Check for time column
        if 'time' not in self.data.columns:
            # Estimate time from index with assumed sample rate
            sample_rate = 50  # Hz (from config)
            self.data['time'] = np.arange(len(self.data)) / sample_rate
            print(f"    [OK] Generated time column (assuming {sample_rate}Hz sample rate)")
        else:
            print(f"    [OK] Found 'time' column")
        
        # Remove any NaN values
        nan_count = self.data['true_velocity_0'].isna().sum()
        if nan_count > 0:
            print(f"    [WARN] Found {nan_count} NaN values in velocity, removing them")
            self.data = self.data.dropna(subset=['true_velocity_0'])
        
        return self
    
    def segment_data(self):
        """Detect throttle segments either via markers or velocity jumps."""
        print("\n[2] Segmenting data by detected velocity transitions")
        time_col = self.data['time'].values
        velocity_col = self.data['true_velocity_0'].values

        marker_column = None
        for candidate in ['segment_marker', 'marker', 'transition_flag']:
            if candidate in self.data.columns:
                marker_column = candidate
                break

        segments_bounds = []
        if marker_column:
            markers = self.data[marker_column].to_numpy()
            value_to_start = {}
            for idx, val in enumerate(markers):
                if val not in value_to_start:
                    value_to_start[val] = idx
            for val in sorted(value_to_start.keys()):
                if val + 1 in value_to_start:
                    segments_bounds.append((value_to_start[val], value_to_start[val + 1], val))
            detection_note = f"manual marker ranges from '{marker_column}'"
        else:
            dt = np.diff(time_col, prepend=time_col[0])
            # use acceleration change rate (jerk) to detect throttle transitions
            accel = np.gradient(velocity_col, time_col)
            da = np.diff(accel, prepend=accel[0])
            jerk = np.zeros_like(da)
            valid_dt = dt > 0
            jerk[valid_dt] = np.abs(da[valid_dt] / dt[valid_dt])

            threshold = max(np.mean(jerk) + 2 * np.std(jerk), 10)
            abrupt_idxs = np.where(jerk > threshold)[0]

            boundaries = [0]
            for idx in abrupt_idxs:
                if idx - boundaries[-1] < 5:
                    continue
                boundaries.append(idx)
            boundaries.append(len(time_col))
            detection_note = f"automatic (threshold {threshold:.3f})"
            for seg_idx in range(len(boundaries) - 1):
                segments_bounds.append((boundaries[seg_idx], boundaries[seg_idx + 1], seg_idx))

        self.segments_data = []
        print(f"\n    Detected {len(segments_bounds)} segments ({detection_note})")
        print("    {'Seg':<4} {'Throttle':<10} {'Time Range':<25} {'Samples':<10} {'Steady V (m/s)':<10}")
        print("    {'-'*65}")

        for start_idx, end_idx, seg_idx in segments_bounds:
            if end_idx - start_idx < 5:
                continue

            velocities = velocity_col[start_idx:end_idx]
            times = time_col[start_idx:end_idx]

            seg_idx_int = int(seg_idx)
            throttle_val = self.throttle_sequence[seg_idx_int] if 0 <= seg_idx_int < len(self.throttle_sequence) else self.throttle_sequence[-1]

            mid_count = end_idx - start_idx
            mid_start = start_idx + int(0.25 * mid_count)
            mid_end = start_idx + int(0.75 * mid_count)
            steady_segment = velocity_col[mid_start:mid_end] if mid_end > mid_start else velocities

            steady_state_v_mean = np.mean(steady_segment)
            steady_state_v_std = np.std(steady_segment)

            seg_record = {
                'segment_idx': seg_idx_int,
                'throttle': throttle_val,
                'times': times.tolist(),
                'velocities': velocities.tolist(),
                'steady_state_velocity': steady_state_v_mean,
                'steady_state_velocity_std': steady_state_v_std,
                'duration': times[-1] - times[0],
                'n_samples': len(velocities)
            }

            print(f"    {seg_idx:<4} {throttle_val:<10.4f} [{times[0]:<8.2f}, {times[-1]:<8.2f}] {len(velocities):<10} {steady_state_v_mean:<10.4f}")
            self.segments_data.append(seg_record)
            self.calibration_points.append({
                'throttle': throttle_val,
                'velocity': steady_state_v_mean,
                'velocity_std': steady_state_v_std,
                'segment_idx': seg_idx_int
            })

        print(f"\n    [OK] Created {len(self.segments_data)} segments")
        return self

    def fit_model(self):
        """Fit throttle = F(velocity) relationship for feedforward control"""
        print(f"\n[3] Fitting throttle = F(velocity) model (for feedforward control)")
        
        throttles = np.array([p['throttle'] for p in self.calibration_points])
        velocities = np.array([p['velocity'] for p in self.calibration_points])
        velocities_std = np.array([p['velocity_std'] for p in self.calibration_points])
        
        # For feedforward: given target velocity, what throttle to apply?
        # So we fit: throttle = F(velocity), i.e., throttle is y, velocity is x
        print(f"    Data points for fitting: {len(throttles)}")
        print(f"    Throttle (output) range: [{throttles.min():.4f}, {throttles.max():.4f}]")
        print(f"    Velocity (input) range: [{velocities.min():.6f}, {velocities.max():.6f}] m/s")
        
        # Try different fitting models: throttle = F(velocity)
        results = {}
        
        # Model 1: Linear fit (throttle = a*velocity + b)
        try:
            z = np.polyfit(velocities, throttles, 1)  # x=velocity, y=throttle
            p_linear = np.poly1d(z)
            predictions = p_linear(velocities)
            rmse = np.sqrt(np.mean((predictions - throttles) ** 2))
            r_squared = 1 - np.sum((throttles - predictions) ** 2) / np.sum((throttles - np.mean(throttles)) ** 2)
            
            results['linear'] = {
                'model': p_linear,
                'coeffs': z,
                'rmse': rmse,
                'r_squared': r_squared,
                'equation': f'throttle = {z[0]:.6f} * v + {z[1]:.6f}'
            }
            print(f"    Linear (throttle = a*v + b):")
            print(f"      - RMSE: {rmse:.8f}")
            print(f"      - R2: {r_squared:.6f}")
            print(f"      - {results['linear']['equation']}")
        except Exception as e:
            print(f"    Linear fit failed: {e}")
        
        # Model 2: Quadratic fit (throttle = a*velocity^2 + b*velocity + c)
        try:
            z = np.polyfit(velocities, throttles, 2)  # x=velocity, y=throttle
            p_quad = np.poly1d(z)
            predictions = p_quad(velocities)
            rmse = np.sqrt(np.mean((predictions - throttles) ** 2))
            r_squared = 1 - np.sum((throttles - predictions) ** 2) / np.sum((throttles - np.mean(throttles)) ** 2)
            
            results['quadratic'] = {
                'model': p_quad,
                'coeffs': z,
                'rmse': rmse,
                'r_squared': r_squared,
                'equation': f'throttle = {z[0]:.6f} * v^2 + {z[1]:.6f} * v + {z[2]:.6f}'
            }
            print(f"    Quadratic (throttle = a*v^2 + b*v + c):")
            print(f"      - RMSE: {rmse:.8f}")
            print(f"      - R2: {r_squared:.6f}")
            print(f"      - {results['quadratic']['equation']}")
        except Exception as e:
            print(f"    Quadratic fit failed: {e}")
        
        # Model 3: Exponential fit (throttle = a * (1 - exp(-b*velocity)))
        try:
            def exp_model(v, a, b):
                return a * (1 - np.exp(-b * v))
            
            # Initial guess
            popt, pcov = curve_fit(exp_model, velocities, throttles, p0=[0.2, 5.0], maxfev=5000)
            predictions = exp_model(velocities, *popt)
            rmse = np.sqrt(np.mean((predictions - throttles) ** 2))
            r_squared = 1 - np.sum((throttles - predictions) ** 2) / np.sum((throttles - np.mean(throttles)) ** 2)
            
            results['exponential'] = {
                'model': lambda v: exp_model(v, *popt),
                'coeffs': popt,
                'rmse': rmse,
                'r_squared': r_squared,
                'equation': f'throttle = {popt[0]:.6f} * (1 - exp(-{popt[1]:.6f} * v))'
            }
            print(f"    Exponential (throttle = a*(1-exp(-b*v))):")
            print(f"      - RMSE: {rmse:.8f}")
            print(f"      - R2: {r_squared:.6f}")
            print(f"      - {results['exponential']['equation']}")
        except Exception as e:
            print(f"    Exponential fit failed: {e}")
        
        # Select best model
        best_model_name = max(results.keys(), key=lambda k: results[k]['r_squared'])
        self.best_model_info = results[best_model_name]
        self.best_model_name = best_model_name
        
        print(f"\n    [OK] Best model: {best_model_name}")
        print(f"      - {self.best_model_info['equation']}")
        print(f"      - R² = {self.best_model_info['r_squared']:.6f}")
        
        self.fitting_results = results
        return self
    
    def save_results(self):
        """Save analysis results to CSV files"""
        print(f"\n[4] Saving results")
        
        # Save velocity data segments
        velocity_file = self.output_dir / f"{self.csv_file.stem}_vehicle0_velocity.csv"
        
        # Create velocity dataframe with time, velocity, and segment info
        velocity_data = []
        for seg_data in self.segments_data:
            for t, v in zip(seg_data['times'], seg_data['velocities']):
                velocity_data.append({
                    'time': t,
                    'velocity': v,
                    'throttle': seg_data['throttle'],
                    'segment_index': seg_data['segment_idx'],
                    'segment_duration': seg_data['duration']
                })
        
        df_velocity = pd.DataFrame(velocity_data)
        df_velocity.to_csv(velocity_file, index=False)
        print(f"    [OK] Saved velocity data: {velocity_file}")
        print(f"      - {len(df_velocity)} samples")
        
        # Save calibration points (throttle-velocity pairs)
        calibration_file = self.output_dir / f"{self.csv_file.stem}_calibration_points.csv"
        df_calibration = pd.DataFrame(self.calibration_points)
        df_calibration.to_csv(calibration_file, index=False)
        print(f"    [OK] Saved calibration points: {calibration_file}")
        print(f"      - {len(df_calibration)} calibration points")
        
        # Save fitting results
        results_file = self.output_dir / f"{self.csv_file.stem}_fitting_results.txt"
        with open(results_file, 'w', encoding='utf-8') as f:
            f.write("=" * 80 + "\n")
            f.write("Velocity to Throttle Fitting Results (for Feedforward Control)\n")
            f.write("=" * 80 + "\n\n")
            
            f.write(f"CSV File: {self.csv_file}\n")
            f.write(f"Test Duration: {self.data['time'].iloc[-1] - self.data['time'].iloc[0]:.2f} seconds\n")
            f.write(f"Total Samples: {len(self.data)}\n\n")
            
            f.write("Throttle Sequence Configuration:\n")
            f.write(f"  - Values: {self.throttle_sequence}\n")
            f.write(f"  - Duration per value: {self.segment_duration}s\n")
            f.write(f"  - Total sequence duration: {len(self.throttle_sequence) * self.segment_duration}s\n\n")
            
            f.write("Calibration Data Points:\n")
            f.write("  Segment | Throttle | Velocity (m/s) | Std Dev\n")
            f.write("  " + "-" * 50 + "\n")
            for point in self.calibration_points:
                f.write(f"  {point['segment_idx']:7d} | {point['throttle']:8.4f} | {point['velocity']:14.6f} | {point['velocity_std']:.6f}\n")
            
            f.write("\n" + "=" * 80 + "\n")
            f.write("Fitting Models:\n")
            f.write("=" * 80 + "\n\n")
            
            for model_name, result in self.fitting_results.items():
                f.write(f"{model_name.upper()}:\n")
                f.write(f"  Equation: {result['equation']}\n")
                f.write(f"  RMSE: {result['rmse']:.8f} m/s\n")
                f.write(f"  R²: {result['r_squared']:.6f}\n")
                if 'coeffs' in result:
                    f.write(f"  Coefficients: {result['coeffs']}\n")
                f.write("\n")
            
            f.write("=" * 80 + "\n")
            f.write(f"BEST MODEL: {self.best_model_name.upper()}\n")
            f.write("=" * 80 + "\n")
            f.write(f"\nEquation: {self.best_model_info['equation']}\n")
            f.write(f"R²: {self.best_model_info['r_squared']:.6f}\n")
            f.write(f"RMSE: {self.best_model_info['rmse']:.8f} m/s\n")
        
        print(f"    [OK] Saved fitting results: {results_file}")
        
        return self
    
    def plot_results(self):
        """Generate plots of fitting results"""
        print(f"\n[5] Generating plots")        
        # Use non-interactive backend for headless environment
        import matplotlib
        matplotlib.use('Agg')
        
        # Extract calibration data
        throttles = np.array([p['throttle'] for p in self.calibration_points])
        velocities = np.array([p['velocity'] for p in self.calibration_points])
        velocities_std = np.array([p['velocity_std'] for p in self.calibration_points])
        
        # Create figure with 2 subplots
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        
        # Plot 1: Calibration points and fitting models (throttle = F(velocity))
        # X-axis: velocity (input), Y-axis: throttle (output)
        ax1.errorbar(velocities, throttles, xerr=velocities_std, fmt='ko', markersize=8, 
                     capsize=5, capthick=2, label='Measured Data', linewidth=2)
        
        # Generate smooth curve for best model
        v_smooth = np.linspace(velocities.min(), velocities.max(), 100)
        t_smooth = self.best_model_info['model'](v_smooth)
        ax1.plot(v_smooth, t_smooth, 'b-', linewidth=2, label=f"Best fit ({self.best_model_name})")
        
        ax1.set_xlabel('Velocity (m/s)', fontsize=12)
        ax1.set_ylabel('Throttle Command', fontsize=12)
        ax1.set_title('Velocity to Throttle Mapping (Feedforward)', fontsize=13, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=10)
        
        # Add equation to plot
        textstr = f"throttle = f(v)\n{self.best_model_info['equation']}\nR2 = {self.best_model_info['r_squared']:.4f}"
        ax1.text(0.05, 0.95, textstr, transform=ax1.transAxes, fontsize=10,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Plot 2: Time series of velocity colored by throttle
        colors = plt.cm.viridis(np.linspace(0, 1, len(self.segments_data)))
        
        for i, seg_data in enumerate(self.segments_data):
            times = seg_data['times']
            velocities_seg = seg_data['velocities']
            throttle_val = seg_data['throttle']
            
            ax2.plot(times, velocities_seg, 'o-', markersize=4, linewidth=1, 
                    color=colors[i], label=f'Throttle={throttle_val:.2f}')
            ax2.axhline(y=seg_data['steady_state_velocity'], color=colors[i], 
                       linestyle='--', linewidth=1, alpha=0.7)
        
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('Velocity (m/s)', fontsize=12)
        ax2.set_title('Velocity vs Time (colored by throttle)', fontsize=13, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=9, loc='best', ncol=2)
        
        plt.tight_layout()
        plot_file = self.output_dir / f"{self.csv_file.stem}_fitting_analysis.png"
        plt.savefig(plot_file, dpi=150, bbox_inches='tight')
        print(f"    [OK] Saved plot: {plot_file}")
        plt.close()
        
        return self
    
    def analyze(self):
        """Run complete analysis pipeline"""
        print("\n" + "=" * 80)
        print("Velocity to Throttle Analysis (for Feedforward Control)")
        print("=" * 80)
        
        self.load_data()
        self.segment_data()
        self.fit_model()
        self.save_results()
        self.plot_results()
        
        print("\n" + "=" * 80)
        print("Analysis Complete!")
        print("=" * 80)
        print(f"\nOutput files saved to: {self.output_dir}")
        print(f"  1. Vehicle 0 velocity: {self.csv_file.stem}_vehicle0_velocity.csv")
        print(f"  2. Calibration points: {self.csv_file.stem}_calibration_points.csv")
        print(f"  3. Fitting results: {self.csv_file.stem}_fitting_results.txt")
        print(f"  4. Analysis plot: {self.csv_file.stem}_fitting_analysis.png")
        print("=" * 80)

if __name__ == '__main__':
    import sys
    
    # Get CSV file path
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        # Try to find latest CSV file in observer_recordings directory
        recordings_dir = Path(__file__).parent.parent.parent / "observer_recordings"
        csv_files = sorted(recordings_dir.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
        
        if csv_files:
            csv_file = csv_files[0]
            print(f"Found latest recording: {csv_file.name}")
        else:
            print("Usage: python get_throttle_2_velocity.py <csv_file>")
            print(f"No CSV files found in {recordings_dir}")
            sys.exit(1)
    
    # Run analysis
    analyzer = ThrottleVelocityAnalyzer(csv_file)
    analyzer.analyze()

