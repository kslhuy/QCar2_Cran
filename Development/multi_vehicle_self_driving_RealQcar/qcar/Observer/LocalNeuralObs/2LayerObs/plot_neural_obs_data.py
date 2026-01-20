"""
Neural Observer Data Plotter

Visualization tool for recorded neural observer data.
Creates multi-panel plots showing:
- State estimation vs measurements
- Neural network tire residual outputs
- Estimation errors
- Training loss
- X-Y trajectory

Usage:
    python plot_neural_obs_data.py [path/to/recording.csv]
    
If no path is provided, opens a file dialog or uses the most recent recording.
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from typing import Optional, Dict

# Try to import pandas for CSV loading
try:
    import pandas as pd
    PANDAS_AVAILABLE = True
except ImportError:
    PANDAS_AVAILABLE = False


def load_data(filepath: str) -> Dict[str, np.ndarray]:
    """
    Load recorded data from CSV file.
    
    Args:
        filepath: Path to CSV file
        
    Returns:
        Dictionary with column names as keys and numpy arrays as values
    """
    if PANDAS_AVAILABLE:
        df = pd.read_csv(filepath)
        return {col: df[col].values for col in df.columns}
    else:
        # Fallback using numpy
        data = np.genfromtxt(filepath, delimiter=',', names=True)
        return {name: data[name] for name in data.dtype.names}


def find_latest_recording(base_dir: str = "neural_obs_recordings") -> Optional[str]:
    """Find the most recent recording file in the output directory."""
    if not os.path.exists(base_dir):
        return None
    
    csv_files = list(Path(base_dir).glob("*.csv"))
    if not csv_files:
        return None
    
    # Sort by modification time, newest first
    csv_files.sort(key=lambda x: x.stat().st_mtime, reverse=True)
    return str(csv_files[0])


def plot_neural_obs_data(data: Dict[str, np.ndarray], 
                         title: str = "Neural Observer Analysis",
                         save_path: Optional[str] = None):
    """
    Create comprehensive plots for neural observer data.
    
    Args:
        data: Dictionary of recorded data arrays
        title: Overall title for the figure
        save_path: Optional path to save the figure
    """
    time = data.get('time', np.arange(len(list(data.values())[0])))
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(title, fontsize=14, fontweight='bold')
    
    # Use GridSpec for flexible layout
    gs = fig.add_gridspec(4, 3, hspace=0.35, wspace=0.3)
    
    # =========================================================================
    # Row 1: Velocity and Yaw Rate Estimation
    # =========================================================================
    
    # Plot 1: Longitudinal Velocity
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(time, data.get('vx_est', []), 'b-', label='Estimated', linewidth=1.5)
    ax1.plot(time, data.get('vx_meas', []), 'r--', label='Measured', linewidth=1)
    if 'vx_uio' in data:
        ax1.plot(time, data['vx_uio'], 'g:', label='UIO', linewidth=1)
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('$v_x$ [m/s]')
    ax1.set_title('Longitudinal Velocity')
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Lateral Velocity
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(time, data.get('vy_est', []), 'b-', label='Estimated', linewidth=1.5)
    if 'vy_uio' in data:
        ax2.plot(time, data['vy_uio'], 'g:', label='UIO', linewidth=1)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('$v_y$ [m/s]')
    ax2.set_title('Lateral Velocity')
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Yaw Rate
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(time, data.get('r_est', []), 'b-', label='Estimated', linewidth=1.5)
    ax3.plot(time, data.get('r_meas', []), 'r--', label='Measured', linewidth=1)
    if 'r_uio' in data:
        ax3.plot(time, data['r_uio'], 'g:', label='UIO', linewidth=1)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('$r$ [rad/s]')
    ax3.set_title('Yaw Rate')
    ax3.legend(loc='upper right', fontsize=8)
    ax3.grid(True, alpha=0.3)
    
    # =========================================================================
    # Row 2: Position and Heading
    # =========================================================================
    
    # Plot 4: X Position
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(time, data.get('X_est', []), 'b-', label='Estimated', linewidth=1.5)
    ax4.plot(time, data.get('X_meas', []), 'r--', label='GPS', linewidth=1)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('X [m]')
    ax4.set_title('X Position')
    ax4.legend(loc='upper right', fontsize=8)
    ax4.grid(True, alpha=0.3)
    
    # Plot 5: Y Position
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(time, data.get('Y_est', []), 'b-', label='Estimated', linewidth=1.5)
    ax5.plot(time, data.get('Y_meas', []), 'r--', label='GPS', linewidth=1)
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Y [m]')
    ax5.set_title('Y Position')
    ax5.legend(loc='upper right', fontsize=8)
    ax5.grid(True, alpha=0.3)
    
    # Plot 6: Yaw Angle
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.plot(time, np.rad2deg(data.get('psi_est', [])), 'b-', label='Estimated', linewidth=1.5)
    ax6.plot(time, np.rad2deg(data.get('psi_meas', [])), 'r--', label='GPS', linewidth=1)
    ax6.set_xlabel('Time [s]')
    ax6.set_ylabel('$\\psi$ [deg]')
    ax6.set_title('Yaw Angle')
    ax6.legend(loc='upper right', fontsize=8)
    ax6.grid(True, alpha=0.3)
    
    # =========================================================================
    # Row 3: Neural Network Outputs and Control
    # =========================================================================
    
    # Plot 7: Tire Residuals (NN Output)
    ax7 = fig.add_subplot(gs[2, 0])
    ax7.plot(time, data.get('w_r', []), 'b-', label='$w_r$ (rear)', linewidth=1.5)
    ax7.plot(time, data.get('w_f', []), 'r-', label='$w_f$ (front)', linewidth=1.5)
    ax7.set_xlabel('Time [s]')
    ax7.set_ylabel('Tire Residual [N]')
    ax7.set_title('Neural Network Output (Tire Residuals)')
    ax7.legend(loc='upper right', fontsize=8)
    ax7.grid(True, alpha=0.3)
    
    # Plot 8: Control Inputs
    ax8 = fig.add_subplot(gs[2, 1])
    ax8.plot(time, np.rad2deg(data.get('steering', [])), 'b-', label='Steering', linewidth=1.5)
    ax8_twin = ax8.twinx()
    ax8_twin.plot(time, data.get('throttle', []), 'r-', label='Throttle', linewidth=1.5)
    ax8.set_xlabel('Time [s]')
    ax8.set_ylabel('Steering [deg]', color='b')
    ax8_twin.set_ylabel('Throttle', color='r')
    ax8.set_title('Control Inputs')
    ax8.grid(True, alpha=0.3)
    
    # Plot 9: Training Loss
    ax9 = fig.add_subplot(gs[2, 2])
    loss = data.get('loss', [])
    if len(loss) > 0:
        # Filter out zero values (no GPS updates)
        gps_valid = data.get('gps_valid', np.ones_like(loss))
        valid_mask = np.array(gps_valid, dtype=bool)
        if np.any(valid_mask):
            ax9.plot(time[valid_mask], np.abs(loss[valid_mask]) + 1e-10, 'b-', linewidth=1)
            ax9.set_yscale('log')
    ax9.set_xlabel('Time [s]')
    ax9.set_ylabel('Loss')
    ax9.set_title('Training Loss (log scale)')
    ax9.grid(True, alpha=0.3)
    
    # =========================================================================
    # Row 4: Trajectory and Estimation Errors
    # =========================================================================
    
    # Plot 10: X-Y Trajectory
    ax10 = fig.add_subplot(gs[3, 0:2])
    X_est = data.get('X_est', [])
    Y_est = data.get('Y_est', [])
    X_meas = data.get('X_meas', [])
    Y_meas = data.get('Y_meas', [])
    
    if len(X_est) > 0:
        ax10.plot(X_est, Y_est, 'b-', label='Estimated', linewidth=2)
        ax10.plot(X_meas, Y_meas, 'r--', label='GPS Measured', linewidth=1, alpha=0.7)
        ax10.plot(X_est[0], Y_est[0], 'go', markersize=10, label='Start')
        ax10.plot(X_est[-1], Y_est[-1], 'rs', markersize=10, label='End')
    ax10.set_xlabel('X [m]')
    ax10.set_ylabel('Y [m]')
    ax10.set_title('X-Y Trajectory')
    ax10.legend(loc='best', fontsize=8)
    ax10.axis('equal')
    ax10.grid(True, alpha=0.3)
    
    # Plot 11: Position Error
    ax11 = fig.add_subplot(gs[3, 2])
    X_err = np.array(data.get('X_est', [])) - np.array(data.get('X_meas', []))
    Y_err = np.array(data.get('Y_est', [])) - np.array(data.get('Y_meas', []))
    pos_err = np.sqrt(X_err**2 + Y_err**2)
    if len(pos_err) > 0:
        ax11.plot(time, pos_err * 100, 'b-', linewidth=1.5)  # Convert to cm
        ax11.axhline(y=np.mean(pos_err) * 100, color='r', linestyle='--', 
                     label=f'Mean: {np.mean(pos_err)*100:.1f} cm')
    ax11.set_xlabel('Time [s]')
    ax11.set_ylabel('Position Error [cm]')
    ax11.set_title('Position Estimation Error')
    ax11.legend(loc='upper right', fontsize=8)
    ax11.grid(True, alpha=0.3)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Figure saved to: {save_path}")
    
    plt.show()


def main():
    """Main entry point for the plotting tool."""
    parser = argparse.ArgumentParser(description='Plot neural observer recorded data')
    parser.add_argument('file', nargs='?', help='Path to CSV recording file')
    parser.add_argument('--save', '-s', help='Path to save the figure')
    parser.add_argument('--dir', '-d', default='neural_obs_recordings',
                        help='Directory to search for recordings')
    
    args = parser.parse_args()
    
    # Determine file path
    if args.file:
        filepath = args.file
    else:
        filepath = find_latest_recording(args.dir)
        if filepath is None:
            print(f"No recording files found in '{args.dir}'")
            print("Usage: python plot_neural_obs_data.py [path/to/recording.csv]")
            sys.exit(1)
        print(f"Using latest recording: {filepath}")
    
    if not os.path.exists(filepath):
        print(f"File not found: {filepath}")
        sys.exit(1)
    
    # Load and plot data
    print(f"Loading data from: {filepath}")
    data = load_data(filepath)
    print(f"Loaded {len(data.get('time', []))} samples")
    
    title = f"Neural Observer Analysis - {Path(filepath).stem}"
    plot_neural_obs_data(data, title=title, save_path=args.save)


if __name__ == '__main__':
    main()
