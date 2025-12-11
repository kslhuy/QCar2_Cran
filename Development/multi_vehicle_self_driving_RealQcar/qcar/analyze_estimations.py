"""
Example: Analyzing Received Fleet and Local Estimations
Demonstrates how to use the new estimation logging features
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path


def analyze_fleet_consensus(run_dir: str, observer_vehicle_id: int):
    """
    Analyze fleet consensus by examining fleet estimations received by one vehicle
    
    Args:
        run_dir: Directory containing the log files (e.g., 'data_logs/run_20250110_143000')
        observer_vehicle_id: ID of the vehicle that received the estimations
    """
    # Load fleet estimations
    file_path = Path(run_dir) / f"received_fleet_estimations_vehicle_{observer_vehicle_id}.csv"
    
    if not file_path.exists():
        print(f"Error: File not found: {file_path}")
        return
    
    df = pd.read_csv(file_path)
    
    print(f"\n{'='*80}")
    print(f"Fleet Consensus Analysis - Vehicle {observer_vehicle_id}")
    print(f"{'='*80}")
    print(f"\nDataset Info:")
    print(f"  Total records: {len(df)}")
    print(f"  Senders: {sorted(df['sender_id'].unique())}")
    print(f"  Vehicles tracked: {sorted(df['vehicle_id'].unique())}")
    print(f"  Time range: {df['timestamp'].min():.2f}s to {df['timestamp'].max():.2f}s")
    print(f"  Duration: {df['timestamp'].max() - df['timestamp'].min():.2f}s")
    
    # Analyze consensus for each vehicle
    for vehicle_id in sorted(df['vehicle_id'].unique()):
        vehicle_data = df[df['vehicle_id'] == vehicle_id]
        print(f"\n  Vehicle {vehicle_id} estimates:")
        print(f"    Records: {len(vehicle_data)}")
        print(f"    From senders: {sorted(vehicle_data['sender_id'].unique())}")
        print(f"    Position range: X=[{vehicle_data['x'].min():.2f}, {vehicle_data['x'].max():.2f}], "
              f"Y=[{vehicle_data['y'].min():.2f}, {vehicle_data['y'].max():.2f}]")
        print(f"    Mean confidence: {vehicle_data['confidence'].mean():.3f}")
    
    # Plot trajectories from different senders
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    
    # Plot 1: All trajectories
    ax = axes[0, 0]
    for vehicle_id in sorted(df['vehicle_id'].unique()):
        vehicle_data = df[df['vehicle_id'] == vehicle_id]
        ax.plot(vehicle_data['x'], vehicle_data['y'], 'o-', alpha=0.6, markersize=2, label=f'Vehicle {vehicle_id}')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title(f'Fleet Trajectories (Received by Vehicle {observer_vehicle_id})')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # Plot 2: Consensus comparison for one vehicle
    if len(df['vehicle_id'].unique()) > 0:
        target_vehicle = sorted(df['vehicle_id'].unique())[0]
        ax = axes[0, 1]
        vehicle_data = df[df['vehicle_id'] == target_vehicle]
        
        for sender_id in sorted(vehicle_data['sender_id'].unique()):
            sender_data = vehicle_data[vehicle_data['sender_id'] == sender_id]
            relative_time = sender_data['timestamp'] - sender_data['timestamp'].min()
            ax.plot(relative_time, sender_data['x'], 'o-', alpha=0.7, markersize=3, label=f'From Vehicle {sender_id}')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('X Position (m)')
        ax.set_title(f'Consensus Comparison - Vehicle {target_vehicle} X Position')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    # Plot 3: Confidence over time
    ax = axes[1, 0]
    for vehicle_id in sorted(df['vehicle_id'].unique()):
        vehicle_data = df[df['vehicle_id'] == vehicle_id]
        relative_time = vehicle_data['timestamp'] - vehicle_data['timestamp'].min()
        ax.plot(relative_time, vehicle_data['confidence'], 'o', alpha=0.5, markersize=3, label=f'Vehicle {vehicle_id}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Confidence')
    ax.set_title('Estimation Confidence Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 4: Message reception rate
    ax = axes[1, 1]
    for sender_id in sorted(df['sender_id'].unique()):
        sender_data = df[df['sender_id'] == sender_id]
        # Count messages per second
        time_bins = np.arange(sender_data['timestamp'].min(), sender_data['timestamp'].max(), 1.0)
        counts, _ = np.histogram(sender_data['timestamp'], bins=time_bins)
        ax.plot(time_bins[:-1], counts, 'o-', alpha=0.7, label=f'From Vehicle {sender_id}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Messages per Second')
    ax.set_title('Fleet Estimation Reception Rate')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(Path(run_dir) / f'fleet_consensus_analysis_vehicle_{observer_vehicle_id}.png', dpi=150)
    print(f"\n✓ Plot saved: {run_dir}/fleet_consensus_analysis_vehicle_{observer_vehicle_id}.png")
    plt.show()


def analyze_v2v_latency(run_dir: str, observer_vehicle_id: int):
    """
    Analyze V2V communication latency from local state messages
    
    Args:
        run_dir: Directory containing the log files
        observer_vehicle_id: ID of the vehicle that received the messages
    """
    file_path = Path(run_dir) / f"received_local_estimations_vehicle_{observer_vehicle_id}.csv"
    
    if not file_path.exists():
        print(f"Error: File not found: {file_path}")
        return
    
    df = pd.read_csv(file_path)
    
    # Calculate latency
    df['latency_ms'] = (df['receive_time'] - df['timestamp']) * 1000
    
    print(f"\n{'='*80}")
    print(f"V2V Communication Latency Analysis - Vehicle {observer_vehicle_id}")
    print(f"{'='*80}")
    print(f"\nDataset Info:")
    print(f"  Total messages: {len(df)}")
    print(f"  Senders: {sorted(df['sender_id'].unique())}")
    print(f"  Time range: {df['timestamp'].min():.2f}s to {df['timestamp'].max():.2f}s")
    
    print(f"\nLatency Statistics by Sender:")
    for sender_id in sorted(df['sender_id'].unique()):
        sender_data = df[df['sender_id'] == sender_id]
        print(f"\n  Vehicle {sender_id} → Vehicle {observer_vehicle_id}:")
        print(f"    Messages: {len(sender_data)}")
        print(f"    Mean latency: {sender_data['latency_ms'].mean():.2f} ms")
        print(f"    Median latency: {sender_data['latency_ms'].median():.2f} ms")
        print(f"    Max latency: {sender_data['latency_ms'].max():.2f} ms")
        print(f"    Std deviation: {sender_data['latency_ms'].std():.2f} ms")
        print(f"    99th percentile: {sender_data['latency_ms'].quantile(0.99):.2f} ms")
    
    # Plot latency analysis
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    # Plot 1: Latency over time
    ax = axes[0, 0]
    for sender_id in sorted(df['sender_id'].unique()):
        sender_data = df[df['sender_id'] == sender_id]
        relative_time = sender_data['timestamp'] - sender_data['timestamp'].min()
        ax.plot(relative_time, sender_data['latency_ms'], 'o', alpha=0.6, markersize=2, label=f'Vehicle {sender_id}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Latency (ms)')
    ax.set_title('V2V Communication Latency Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 2: Latency histogram
    ax = axes[0, 1]
    for sender_id in sorted(df['sender_id'].unique()):
        sender_data = df[df['sender_id'] == sender_id]
        ax.hist(sender_data['latency_ms'], bins=50, alpha=0.5, label=f'Vehicle {sender_id}')
    ax.set_xlabel('Latency (ms)')
    ax.set_ylabel('Count')
    ax.set_title('Latency Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 3: Message reception rate
    ax = axes[1, 0]
    for sender_id in sorted(df['sender_id'].unique()):
        sender_data = df[df['sender_id'] == sender_id]
        time_bins = np.arange(sender_data['receive_time'].min(), sender_data['receive_time'].max(), 1.0)
        counts, _ = np.histogram(sender_data['receive_time'], bins=time_bins)
        ax.plot(time_bins[:-1], counts, 'o-', alpha=0.7, label=f'From Vehicle {sender_id}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Messages per Second')
    ax.set_title('Message Reception Rate')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 4: Trajectory comparison
    ax = axes[1, 1]
    for sender_id in sorted(df['sender_id'].unique()):
        sender_data = df[df['sender_id'] == sender_id]
        ax.plot(sender_data['x'], sender_data['y'], 'o-', alpha=0.6, markersize=2, label=f'Vehicle {sender_id}')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Trajectories from Local State Messages')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.tight_layout()
    plt.savefig(Path(run_dir) / f'v2v_latency_analysis_vehicle_{observer_vehicle_id}.png', dpi=150)
    print(f"\n✓ Plot saved: {run_dir}/v2v_latency_analysis_vehicle_{observer_vehicle_id}.png")
    plt.show()


def compare_self_vs_consensus(run_dir: str, observer_vehicle_id: int, target_vehicle_id: int):
    """
    Compare self-reported states vs fleet consensus estimates
    
    Args:
        run_dir: Directory containing the log files
        observer_vehicle_id: Vehicle that received the messages
        target_vehicle_id: Vehicle to analyze
    """
    local_path = Path(run_dir) / f"received_local_estimations_vehicle_{observer_vehicle_id}.csv"
    fleet_path = Path(run_dir) / f"received_fleet_estimations_vehicle_{observer_vehicle_id}.csv"
    
    if not local_path.exists() or not fleet_path.exists():
        print(f"Error: Missing log files in {run_dir}")
        return
    
    # Load data
    local_df = pd.read_csv(local_path)
    fleet_df = pd.read_csv(fleet_path)
    
    # Filter for target vehicle
    self_reports = local_df[local_df['sender_id'] == target_vehicle_id].copy()
    consensus_estimates = fleet_df[fleet_df['vehicle_id'] == target_vehicle_id].copy()
    
    if len(self_reports) == 0 or len(consensus_estimates) == 0:
        print(f"Error: No data found for vehicle {target_vehicle_id}")
        return
    
    print(f"\n{'='*80}")
    print(f"Self-Report vs Consensus - Vehicle {target_vehicle_id}")
    print(f"{'='*80}")
    print(f"\nSelf-reports: {len(self_reports)} messages")
    print(f"Consensus estimates: {len(consensus_estimates)} messages")
    print(f"  From senders: {sorted(consensus_estimates['sender_id'].unique())}")
    
    # Plot comparison
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    # Plot 1: Trajectory comparison
    ax = axes[0, 0]
    ax.plot(self_reports['x'], self_reports['y'], 'b-', linewidth=2, label='Self-reported', alpha=0.8)
    
    for sender_id in sorted(consensus_estimates['sender_id'].unique()):
        sender_data = consensus_estimates[consensus_estimates['sender_id'] == sender_id]
        ax.plot(sender_data['x'], sender_data['y'], 'o--', alpha=0.6, markersize=3, label=f'Consensus from V{sender_id}')
    
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title(f'Vehicle {target_vehicle_id}: Self-Report vs Fleet Consensus')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # Plot 2: X position over time
    ax = axes[0, 1]
    self_time = self_reports['timestamp'] - self_reports['timestamp'].min()
    ax.plot(self_time, self_reports['x'], 'b-', linewidth=2, label='Self-reported', alpha=0.8)
    
    for sender_id in sorted(consensus_estimates['sender_id'].unique()):
        sender_data = consensus_estimates[consensus_estimates['sender_id'] == sender_id]
        sender_time = sender_data['timestamp'] - self_reports['timestamp'].min()
        ax.plot(sender_time, sender_data['x'], 'o--', alpha=0.6, markersize=3, label=f'From V{sender_id}')
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('X Position (m)')
    ax.set_title('X Position Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 3: Y position over time
    ax = axes[1, 0]
    ax.plot(self_time, self_reports['y'], 'b-', linewidth=2, label='Self-reported', alpha=0.8)
    
    for sender_id in sorted(consensus_estimates['sender_id'].unique()):
        sender_data = consensus_estimates[consensus_estimates['sender_id'] == sender_id]
        sender_time = sender_data['timestamp'] - self_reports['timestamp'].min()
        ax.plot(sender_time, sender_data['y'], 'o--', alpha=0.6, markersize=3, label=f'From V{sender_id}')
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Y Position Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 4: Velocity comparison
    ax = axes[1, 1]
    ax.plot(self_time, self_reports['v'], 'b-', linewidth=2, label='Self-reported', alpha=0.8)
    
    for sender_id in sorted(consensus_estimates['sender_id'].unique()):
        sender_data = consensus_estimates[consensus_estimates['sender_id'] == sender_id]
        sender_time = sender_data['timestamp'] - self_reports['timestamp'].min()
        ax.plot(sender_time, sender_data['v'], 'o--', alpha=0.6, markersize=3, label=f'From V{sender_id}')
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Velocity Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(Path(run_dir) / f'self_vs_consensus_vehicle_{target_vehicle_id}.png', dpi=150)
    print(f"\n✓ Plot saved: {run_dir}/self_vs_consensus_vehicle_{target_vehicle_id}.png")
    plt.show()


if __name__ == "__main__":
    # Example usage - update with your actual run directory
    run_dir = "data_logs/run_20250110_143000"  # Update this path
    
    print("\n" + "="*80)
    print("Vehicle Estimation Logging - Analysis Examples")
    print("="*80)
    
    # Check if directory exists
    if not Path(run_dir).exists():
        print(f"\n⚠ Error: Directory not found: {run_dir}")
        print("\nPlease update the 'run_dir' variable with the path to your data logs.")
        print("Example: run_dir = 'data_logs/run_20250110_143000'")
        print("\nAvailable runs:")
        data_logs_dir = Path("data_logs")
        if data_logs_dir.exists():
            for d in sorted(data_logs_dir.iterdir()):
                if d.is_dir():
                    print(f"  - {d.name}")
        else:
            print(f"  No data_logs directory found")
        exit(1)
    
    # Run analysis examples
    print("\n[1] Fleet Consensus Analysis")
    analyze_fleet_consensus(run_dir, observer_vehicle_id=1)
    
    print("\n[2] V2V Communication Latency Analysis")
    analyze_v2v_latency(run_dir, observer_vehicle_id=1)
    
    print("\n[3] Self-Report vs Consensus Comparison")
    compare_self_vs_consensus(run_dir, observer_vehicle_id=1, target_vehicle_id=0)
    
    print("\n" + "="*80)
    print("Analysis Complete!")
    print("="*80)
