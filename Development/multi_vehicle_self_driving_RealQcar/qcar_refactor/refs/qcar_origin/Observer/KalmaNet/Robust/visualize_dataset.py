import numpy as np
import matplotlib.pyplot as plt
import json
import sys
from pathlib import Path

'''
# python .\visualize_dataset.py datasets\your_specific_file.npz
'''


def visualize_npz(file_path):
    print(f"Loading dataset from: {file_path}\n")
    
    # 1. Load the NPZ file
    try:
        data = np.load(file_path, allow_pickle=True)
    except Exception as e:
        print(f"Failed to load file: {e}")
        return

    # 2. Print available keys
    print("--- Available Arrays (Keys) ---")
    print(", ".join(data.files))
    print("\n")

    # 3. Decode metadata
    if "metadata_json" in data:
        # Depending on how it's saved, it might be a scalar string or a 0-d array
        val = data["metadata_json"]
        meta_str = str(val.tolist()) if hasattr(val, "tolist") else str(val)
        metadata = json.loads(meta_str)
        print("--- Metadata ---")
        print(json.dumps(metadata, indent=2))
        print("\n")

    # 4. Extract arrays for visualization
    timestamps = data.get("timestamps", [])
    if len(timestamps) == 0:
        print("No timestamp data found to plot!")
        return

    x_gt = data.get("x_gt", np.zeros((len(timestamps), 5))) # [x, y, theta, v, w]
    z = data.get("z", np.zeros((len(timestamps), 5)))       # [x, y, theta, v, w]
    steering = data.get("steering", np.zeros(len(timestamps)))
    throttle = data.get("throttle", np.zeros(len(timestamps)))

    t_rel = timestamps - timestamps[0]

    # 5. Visualize
    fig, axs = plt.subplots(3, 1, figsize=(10, 12))

    # Plot Trajectory (X vs Y)
    axs[0].plot(x_gt[:, 0], x_gt[:, 1], label="Ground Truth / Observer (x_gt)", linestyle='--', color='red')
    axs[0].plot(z[:, 0], z[:, 1], label="Measurements (z)", alpha=0.7, color='blue')
    axs[0].set_title("Vehicle Trajectory (X vs Y)")
    axs[0].set_xlabel("X (m)")
    axs[0].set_ylabel("Y (m)")
    axs[0].legend()
    axs[0].grid(True)
    axs[0].axis('equal') # Keep aspect ratio square for spatial paths

    # Plot Speed over time
    axs[1].plot(t_rel, x_gt[:, 3], label="Target Velocity (v)", color='red')
    axs[1].plot(t_rel, z[:, 3], label="Measured Velocity", alpha=0.7, color='blue')
    axs[1].set_title("Velocity over Time")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Velocity (m/s)")
    axs[1].legend()
    axs[1].grid(True)

    # Plot Steering and Throttle
    axs[2].plot(t_rel, steering, label="Steering")
    axs[2].plot(t_rel, throttle, label="Throttle", alpha=0.7)
    axs[2].set_title("Control Inputs")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Command Value")
    axs[2].legend()
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Default to the most recent dataset in the folder if no argument provided
    target_dir = Path(__file__).parent / "datasets"
    
    if len(sys.argv) > 1:
        target_file = sys.argv[1]
    else:
        # Find the latest npz file
        npz_files = list(target_dir.glob("*.npz"))
        if not npz_files:
            print(f"No .npz files found in {target_dir}")
            sys.exit(1)
            
        # Sort by modification time and pick the most recent
        target_file = max(npz_files, key=lambda f: f.stat().st_mtime)
        
    visualize_npz(target_file)
