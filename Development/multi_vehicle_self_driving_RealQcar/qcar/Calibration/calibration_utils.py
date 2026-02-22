"""
calibration_utils.py  –  Shared helpers for the QCar calibration suite.

Usage (from any calibration script):
    from calibration_utils import CSVLogger, fit_first_order, plot_step_response
"""

import os
import csv
import time
import yaml
import numpy as np
from datetime import datetime
from typing import Optional, Tuple, List

# ─── Optional heavy deps – gracefully degraded when unavailable ───────────────
try:
    import matplotlib

    matplotlib.use("Agg")  # headless-safe backend
    import matplotlib.pyplot as plt

    MPL_AVAILABLE = True
except (ImportError, Exception):
    # Catching Exception too because broken matplotlib/numpy installs can raise AttributeError/RuntimeError
    MPL_AVAILABLE = False
    plt = None

try:
    from scipy.optimize import curve_fit

    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    curve_fit = None

# ─── Paths ───────────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(_HERE, "results")
os.makedirs(RESULTS_DIR, exist_ok=True)


# ─── CSV Logger ──────────────────────────────────────────────────────────────


class CSVLogger:
    """
    Simple CSV logger with automatic timestamp column.

    Usage::

        with CSVLogger("my_experiment.csv", ["time", "throttle", "velocity"]) as log:
            log.write(0.0, 0.05, 0.12)
    """

    def __init__(
        self, filename: str, columns: List[str], results_dir: str = RESULTS_DIR
    ):
        self.path = os.path.join(results_dir, filename)
        self.columns = columns
        self._file = None
        self._writer = None

    def __enter__(self):
        self._file = open(self.path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.columns)
        return self

    def write(self, *values):
        """Write one row.  Values must match the column order."""
        self._writer.writerow(
            [f"{v:.6f}" if isinstance(v, float) else v for v in values]
        )

    def __exit__(self, *args):
        if self._file:
            self._file.close()
        print(f"[CSVLogger] Saved → {self.path}")

    def flush(self):
        if self._file:
            self._file.flush()


# ─── First-order model fitting ───────────────────────────────────────────────


def _first_order_response(t: np.ndarray, tau: float, K: float) -> np.ndarray:
    """v(t) = K_u · (1 - exp(-t/tau))  with  K = v_ss / u"""
    return K * (1.0 - np.exp(-t / tau))


def fit_first_order(
    t: np.ndarray,
    y: np.ndarray,
    y_ss: Optional[float] = None,
) -> Tuple[float, float]:
    """
    Fit the first-order step response  y(t) ≈ K*(1 - exp(-t/τ)).

    Parameters
    ----------
    t     : time vector (seconds), starting from 0.
    y     : measured output (velocity in m/s), same length as t.
    y_ss  : optional known steady-state value; if None, uses y[-1].

    Returns
    -------
    (tau, K) – time constant [s] and steady-state gain [m/s per unit input].

    Raises
    ------
    RuntimeError if scipy is not available or fitting fails.
    """
    if not SCIPY_AVAILABLE:
        raise RuntimeError(
            "scipy is required for model fitting. Install with: pip install scipy"
        )

    if y_ss is None:
        y_ss = float(y[-1])

    # Normalise: fit y_norm(t) = 1 - exp(-t/tau)
    eps = 1e-9
    y_norm = np.clip(y / (y_ss + eps), 0.0, 1.0 + eps)

    try:
        (tau_fit,), _ = curve_fit(
            lambda t, tau: 1.0 - np.exp(-t / tau),
            t,
            y_norm,
            p0=[1.0],
            bounds=(1e-3, 30.0),
            maxfev=5000,
        )
    except Exception as exc:
        raise RuntimeError(f"Curve fitting failed: {exc}") from exc

    return float(tau_fit), float(y_ss)


# ─── Plotting ─────────────────────────────────────────────────────────────────


def plot_step_response(
    t: np.ndarray,
    v_measured: np.ndarray,
    v_cmd: float,
    v_fitted: Optional[np.ndarray] = None,
    tau: Optional[float] = None,
    K: Optional[float] = None,
    title: str = "Step Response",
    filename: str = "step_response.png",
) -> str:
    """
    Save a step-response plot to the results directory.

    Returns the path to the saved file.
    """
    if not MPL_AVAILABLE:
        print("[plot_step_response] matplotlib not available – skipping plot.")
        return ""

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(t, v_measured, label="Measured velocity", color="royalblue", linewidth=1.8)
    if v_fitted is not None:
        ax.plot(
            t,
            v_fitted,
            "--",
            label=f"Fit (τ={tau:.3f}s, K={K:.3f})",
            color="tomato",
            linewidth=1.6,
        )
    ax.axhline(
        v_cmd,
        color="gray",
        linestyle=":",
        linewidth=1.2,
        label=f"Command v_ref={v_cmd:.3f} m/s",
    )

    # Annotate 63% rise time (t = tau)
    if tau is not None:
        v63 = 0.632 * K if K is not None else 0.632 * v_cmd
        ax.axvline(tau, color="orange", linestyle="--", linewidth=1.0)
        ax.annotate(
            f"τ={tau:.2f}s",
            xy=(tau, v63),
            xytext=(tau + 0.1, v63 * 0.85),
            arrowprops=dict(arrowstyle="->", color="orange"),
            color="orange",
        )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Velocity [m/s]")
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    path = os.path.join(RESULTS_DIR, filename)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"[plot] Saved → {path}")
    return path


def plot_calibration_map(
    x: np.ndarray,
    y: np.ndarray,
    x_label: str = "Command",
    y_label: str = "Response",
    poly_deg: int = 2,
    title: str = "Calibration Map",
    filename: str = "calibration_map.png",
) -> Tuple[str, np.ndarray]:
    """
    Scatter + polynomial fit plot.

    Returns (path, poly_coeffs).
    """
    coeffs = np.polyfit(x, y, poly_deg)
    x_fit = np.linspace(x.min(), x.max(), 200)
    y_fit = np.polyval(coeffs, x_fit)

    if MPL_AVAILABLE:
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.scatter(x, y, s=60, zorder=5, label="Measured", color="royalblue")
        ax.plot(
            x_fit,
            y_fit,
            "--",
            color="tomato",
            linewidth=1.8,
            label=f"Poly-{poly_deg} fit",
        )
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        path = os.path.join(RESULTS_DIR, filename)
        fig.savefig(path, dpi=130)
        plt.close(fig)
        print(f"[plot] Saved → {path}")
    else:
        path = ""

    return path, coeffs


# ─── YAML helpers ─────────────────────────────────────────────────────────────


def save_yaml(data: dict, filename: str, results_dir: str = RESULTS_DIR) -> str:
    """Save a dict as YAML to the results directory."""
    path = os.path.join(results_dir, filename)
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    print(f"[YAML] Saved → {path}")
    return path


def load_yaml(filename: str, results_dir: str = RESULTS_DIR) -> dict:
    """Load a YAML file from the results directory."""
    path = os.path.join(results_dir, filename)
    if not os.path.exists(path):
        raise FileNotFoundError(f"YAML not found: {path}")
    with open(path) as f:
        return yaml.safe_load(f)


# ─── Simulation helpers ───────────────────────────────────────────────────────


class FirstOrderMotorSim:
    """
    Lightweight simulation of a first-order motor model:
        τ · dv/dt + v = K · u
    Used in --sim mode so scripts work without physical hardware.

    Default: τ=0.8 s, K=5.0 m/s (realistic for QCar at max ~0.5 m/s throttle 0.10)
    """

    def __init__(self, tau: float = 0.8, K: float = 5.0, noise_std: float = 0.01):
        self.tau = tau
        self.K = K
        self.noise_std = noise_std
        self.v = 0.0  # current simulated velocity

    def step(self, u: float, dt: float) -> float:
        """Advance simulation by dt and return new velocity."""
        # Euler integration of τ·v̇ = K·u - v
        dvdt = (self.K * u - self.v) / self.tau
        self.v += dvdt * dt
        # Add small Gaussian noise
        return float(self.v + np.random.randn() * self.noise_std)

    def reset(self):
        self.v = 0.0


class BicycleSteeringSim:
    """
    Kinematic bicycle model for steering calibration in --sim mode.

        x_dot   = v * cos(θ)
        y_dot   = v * sin(θ)
        θ_dot   = v / L * tan(δ)

    where L = wheelbase, δ = steering angle.
    """

    def __init__(self, wheelbase: float = 0.256, noise_std: float = 0.002):
        self.L = wheelbase
        self.noise_std = noise_std
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def step(self, v: float, delta: float, dt: float) -> Tuple[float, float, float]:
        """Step the bicycle model and return (x, y, theta)."""
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += (v / self.L) * np.tan(delta) * dt
        # Small heading noise
        self.theta += np.random.randn() * self.noise_std
        return self.x, self.y, self.theta

    def reset(self):
        self.x = self.y = self.theta = 0.0


# ─── PID Gain Formulae ────────────────────────────────────────────────────────


def pid_gains_ziegler_nichols(K: float, tau: float, L: float) -> dict:
    """
    Ziegler-Nichols open-loop (step-response) tuning.
    L = dead time (use 0.1*tau if unknown).

    Returns dict with Kp, Ki, Kd.
    """
    L = max(L, 1e-6)
    Kp = 1.2 * tau / (K * L)
    Ki = Kp / (2.0 * L)
    Kd = 0.5 * Kp * L
    return {"Kp": round(Kp, 4), "Ki": round(Ki, 4), "Kd": round(Kd, 4)}


def pid_gains_imc(K: float, tau: float, lambda_: Optional[float] = None) -> dict:
    """
    Internal Model Control (IMC) tuning.
    Lambda (closed-loop time constant) defaults to 0.5 * tau (moderate).
    Gives smoother response than ZN, good for real car.
    """
    if lambda_ is None:
        lambda_ = 0.5 * tau
    lambda_ = max(lambda_, 1e-6)
    Kp = tau / (K * lambda_)
    Ki = Kp / tau
    Kd = 0.0  # IMC PI only for first-order
    return {"Kp": round(Kp, 4), "Ki": round(Ki, 4), "Kd": round(Kd, 4)}


def pid_gains_itae(K: float, tau: float, L: float) -> dict:
    """
    ITAE (Integral of Time-weighted Absolute Error) optimal for step disturbance.
    Coefficients from standard ITAE tables for first-order process.
    """
    L = max(L, 1e-6)
    ratio = L / tau
    Kp = (0.586 / K) * (ratio**-0.916)
    Ki = Kp / (1.03 - 0.165 * ratio) / tau
    Kd = 0.0
    return {"Kp": round(Kp, 4), "Ki": round(Ki, 4), "Kd": round(Kd, 4)}


# ─── Misc ─────────────────────────────────────────────────────────────────────


def timestamp_tag() -> str:
    """Return a short timestamp string for file naming, e.g. '20260219_223500'."""
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def print_section(title: str):
    """Print a formatted section header."""
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)
