"""
Plot LeaderingObserver recordings.

Usage:
    python plot_leadering_observer.py --vehicle-id 1
    python plot_leadering_observer.py --csv path/to/leadering_observer_v1_*.csv
"""
import argparse
import glob
import os
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def default_output_dir() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(script_dir, "..", "..", "observer_recordings"))


def find_latest_file(output_dir: str, vehicle_id: int) -> Optional[str]:
    pattern = os.path.join(output_dir, f"leadering_observer_v{vehicle_id}_*.csv")
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def _series(df: pd.DataFrame, name: str) -> Optional[pd.Series]:
    if name not in df.columns:
        return None
    data = pd.to_numeric(df[name], errors="coerce")
    if data.notna().sum() == 0:
        return None
    return data


def plot_leadering_observer(csv_path: str, save: bool = True) -> Optional[str]:
    df = pd.read_csv(csv_path)
    if "time" not in df.columns:
        raise ValueError("CSV file must contain a 'time' column.")

    t = pd.to_numeric(df["time"], errors="coerce")
    t = t - t.min()

    fig, axes = plt.subplots(4, 2, figsize=(13, 10), sharex=True)
    axs = axes.ravel()

    state_specs = [
        ("x", "Position [m]", "true_leader_x", "zeta_hat_x", "err_x"),
        ("v", "Velocity [m/s]", "true_leader_v", "zeta_hat_v", "err_v"),
        ("a", "Acceleration [m/s^2]", "true_leader_a", "zeta_hat_a", "err_a"),
    ]

    for idx, (name, ylabel, true_col, hat_col, err_col) in enumerate(state_specs):
        true_data = _series(df, true_col)
        hat_data = _series(df, hat_col)
        err_data = _series(df, err_col)

        if true_data is not None:
            axs[idx].plot(t, true_data, label=f"leader true {name}", linewidth=2)
        if hat_data is not None:
            axs[idx].plot(t, hat_data, label=f"estimated {name}_hat", linestyle="--", linewidth=2)
        axs[idx].set_ylabel(ylabel)
        axs[idx].set_title(f"Leader true state vs zeta_hat {name}")
        axs[idx].legend()

        if err_data is None and true_data is not None and hat_data is not None:
            err_data = hat_data - true_data
        if err_data is not None:
            axs[idx + 4].plot(t, err_data, label=f"{name}_hat - true {name}", color="#d62728")
            rmse = float(np.sqrt(np.nanmean(np.asarray(err_data) ** 2)))
            axs[idx + 4].set_title(f"Estimation error {name}, RMSE={rmse:.4g}")
        else:
            axs[idx + 4].set_title(f"Estimation error {name}")
        axs[idx + 4].set_ylabel("Error")
        axs[idx + 4].legend()

    zeta0 = _series(df, "zeta_hat_0")
    y_zeta = _series(df, "y_zeta")
    if y_zeta is not None:
        axs[3].plot(t, y_zeta, label="filtered delayed output y_zeta", linewidth=2)
    if zeta0 is not None:
        axs[3].plot(t, zeta0, label="zeta_hat_0", linestyle="--", linewidth=2)
    axs[3].set_ylabel("Integral output")
    axs[3].set_title("Output state reconstruction")
    axs[3].legend()

    hat_tau = _series(df, "hat_tau")
    if hat_tau is not None:
        axs[7].plot(t, hat_tau, label="hat_tau", color="#9467bd", linewidth=2)
    axs[7].set_ylabel("Delay [s]")
    axs[7].set_title("Estimated communication delay hat_tau(t)")
    axs[7].legend()

    for ax in axs:
        ax.grid(True, alpha=0.3)
    for ax in axs[-2:]:
        ax.set_xlabel("Time [s]")

    fig.suptitle(
        "Leadering Observer: leader state reconstruction and delay estimation\n"
        "Shows the PDF contribution: compensating V2V delay with an online hat_tau integral observer.",
        fontsize=12,
    )
    fig.tight_layout(rect=[0, 0.03, 1, 0.94])

    figure_path = None
    if save:
        base = os.path.splitext(os.path.basename(csv_path))[0]
        figure_path = os.path.join(os.path.dirname(csv_path), f"{base}_leadering_observer.png")
        fig.savefig(figure_path, dpi=200, bbox_inches="tight")
        print(f"Saved figure: {figure_path}")

    plt.show(block=False)
    return figure_path


def main():
    parser = argparse.ArgumentParser(description="Plot LeaderingObserverEstimator recordings.")
    parser.add_argument("--csv", type=str, help="Path to a leadering_observer_v*.csv file.")
    parser.add_argument("--vehicle-id", type=int, default=1, help="Observer vehicle ID. Default: 1.")
    parser.add_argument("--output-dir", type=str, default=None, help="Directory containing observer recordings.")
    parser.add_argument("--no-save", action="store_true", help="Do not save the figure PNG.")
    args = parser.parse_args()

    csv_path = args.csv
    output_dir = os.path.abspath(args.output_dir) if args.output_dir else default_output_dir()
    if csv_path is None:
        csv_path = find_latest_file(output_dir, args.vehicle_id)
        if csv_path is None:
            print(f"No leadering observer recording found in {output_dir}")
            return

    print(f"Using CSV file: {csv_path}")
    plot_leadering_observer(csv_path, save=not args.no_save)


if __name__ == "__main__":
    main()
