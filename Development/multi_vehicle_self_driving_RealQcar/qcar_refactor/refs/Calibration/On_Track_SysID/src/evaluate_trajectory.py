import os
import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt

SRC_DIR = os.path.dirname(os.path.abspath(__file__))
if SRC_DIR not in sys.path:
    sys.path.append(SRC_DIR)

from helpers.load_model import get_dotdict
from helpers.runtime import get_package_path
from helpers.vehicle_dynamics_stown import vehicle_dynamics_st


def evaluate_trajectory(racecar_version, save_LUT_name, package_path=None):
    package_root = get_package_path(package_path=package_path)
    csv_path = os.path.join(
        package_root, "models", racecar_version, f"{save_LUT_name}_training_data.csv"
    )

    if not os.path.exists(csv_path):
        print(
            f"Error: Could not find training data CSV at {csv_path}. Make sure you run the training first."
        )
        return

    print(f"Loading data from {csv_path}...")
    # Skip header
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)

    # Data columns: v_x, v_y, omega, delta
    v_x_actual = data[:, 0]
    v_y_actual = data[:, 1]
    omega_actual = data[:, 2]
    delta_actual = data[:, 3]

    timesteps = len(data)
    dt = 0.02  # default sample_dt
    time = np.arange(timesteps) * dt

    print(f"Loading identified model {racecar_version}_pacejka...")
    model = get_dotdict(f"{racecar_version}_pacejka", package_path=package_path)

    # Initialize simulated states
    x_sim = 0.0
    y_sim = 0.0
    yaw_sim = 0.0
    v_y_sim = v_y_actual[0]
    omega_sim = omega_actual[0]

    # Initialize actual states (just integrating for position comparison)
    x_act = 0.0
    y_act = 0.0
    yaw_act = 0.0

    # History arrays for simulated trajectory
    x_sim_hist = np.zeros(timesteps)
    y_sim_hist = np.zeros(timesteps)
    yaw_sim_hist = np.zeros(timesteps)
    v_y_sim_hist = np.zeros(timesteps)
    omega_sim_hist = np.zeros(timesteps)

    # History arrays for actual trajectory (integrated from sensors)
    x_act_hist = np.zeros(timesteps)
    y_act_hist = np.zeros(timesteps)
    yaw_act_hist = np.zeros(timesteps)

    print("Running simulation using actual v_x and delta...")
    for t in range(timesteps):
        # Save actual states
        x_act_hist[t] = x_act
        y_act_hist[t] = y_act
        yaw_act_hist[t] = yaw_act

        # Integrate actual path using Euler integration
        x_act += (
            v_x_actual[t] * np.cos(yaw_act) - v_y_actual[t] * np.sin(yaw_act)
        ) * dt
        y_act += (
            v_x_actual[t] * np.sin(yaw_act) + v_y_actual[t] * np.cos(yaw_act)
        ) * dt
        yaw_act += omega_actual[t] * dt

        # Save simulated states
        x_sim_hist[t] = x_sim
        y_sim_hist[t] = y_sim
        yaw_sim_hist[t] = yaw_sim
        v_y_sim_hist[t] = v_y_sim
        omega_sim_hist[t] = omega_sim

        # Simulate next step using actual v_x and actual delta
        state_sim = [x_sim, y_sim, yaw_sim, v_x_actual[t], v_y_sim, omega_sim]
        # u[1] is acceleration, but we treat it as 0 since we forcibly overwrite v_x with tracked data
        u_sim = [delta_actual[t], 0.0]

        f = vehicle_dynamics_st(state_sim, u_sim, model, "pacejka")

        # Integrate simulated path
        x_sim += f[0] * dt
        y_sim += f[1] * dt
        yaw_sim += f[2] * dt
        # f[3] represents v_x derivative based on acceleration, we skip it since we use measured v_x
        v_y_sim += f[4] * dt
        omega_sim += f[5] * dt

    print("Plotting results...")
    plt.figure(figsize=(15, 10))

    # Plot V_y
    plt.subplot(2, 2, 1)
    plt.plot(time, v_y_actual, "k-", label="Actual")
    plt.plot(time, v_y_sim_hist, "r--", label="Simulated")
    plt.xlabel("Time (s)")
    plt.ylabel("v_y (m/s)")
    plt.title("Lateral Velocity")
    plt.legend()
    plt.grid(True)

    # Plot Omega
    plt.subplot(2, 2, 2)
    plt.plot(time, omega_actual, "k-", label="Actual")
    plt.plot(time, omega_sim_hist, "r--", label="Simulated")
    plt.xlabel("Time (s)")
    plt.ylabel("omega (rad/s)")
    plt.title("Yaw Rate")
    plt.legend()
    plt.grid(True)

    # Plot Trajectory (X-Y)
    plt.subplot(2, 2, 3)
    plt.plot(x_act_hist, y_act_hist, "k-", label="Actual Path")
    plt.plot(x_sim_hist, y_sim_hist, "r--", label="Simulated Path")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("2D Trajectory")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")

    # Plot Inputs (V_x and Delta)
    plt.subplot(2, 2, 4)
    plt.plot(time, v_x_actual, "b-", label="v_x (m/s)")
    plt.plot(time, delta_actual, "g-", label="delta (rad)")
    plt.xlabel("Time (s)")
    plt.ylabel("Recorded Inputs")
    plt.title("Inputs Fed into Simulation")
    plt.legend()
    plt.grid(True)

    plt.suptitle(f"Pacejka Model Track Evaluation ({racecar_version})", fontsize=16)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Evaluate identified Pacejka model against recorded data."
    )
    parser.add_argument(
        "--racecar-version",
        type=str,
        default="SIM",
        help="Version of the racecar (e.g., SIM, QCar2)",
    )
    parser.add_argument(
        "--save-lut-name",
        type=str,
        default="online_sysid",
        help="Name prefix used when training",
    )
    args = parser.parse_args()

    evaluate_trajectory(args.racecar_version, args.save_lut_name)
