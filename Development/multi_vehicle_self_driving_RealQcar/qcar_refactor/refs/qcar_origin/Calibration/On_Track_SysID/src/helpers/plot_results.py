import multiprocessing
import numpy as np


def _run_plot(
    model,
    v_x,
    v_y,
    omega,
    delta,
    C_Pf_identified,
    C_Pr_identified,
    iteration,
    training_data=None,
):
    import matplotlib.pyplot as plt
    from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Calibration.On_Track_SysID.src.helpers.pacejka_formula import pacejka_formula

    C_Pf_model = model["C_Pf_model"]
    C_Pr_model = model["C_Pr_model"]

    C_Pf_ref = model["C_Pf_ref"]
    C_Pr_ref = model["C_Pr_ref"]

    l_f = model["l_f"]
    l_r = model["l_r"]
    l_wb = model["l_wb"]
    m = model["m"]
    g_ = 9.81

    # Compute vertical forces on front and rear tires
    F_zf = m * g_ * l_r / l_wb
    F_zr = m * g_ * l_f / l_wb

    # Compute lateral forces on front and rear tires
    F_yf = m * l_r * v_x * omega / ((l_r + l_f) * np.cos(delta))
    F_yr = m * l_f * v_x * omega / (l_r + l_f)

    # Compute slip angles for front and rear tires
    alpha_f = -np.arctan((v_y + l_f * omega) / v_x) + delta
    alpha_r = -np.arctan((v_y - l_r * omega) / v_x)

    # Plotting
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
    ax1.scatter(alpha_f, F_yf, color="Green", alpha=1, label="Generated Data", s=1)
    ax2.scatter(alpha_r, F_yr, color="Green", alpha=1, label="Generated Data", s=1)

    # Plot reference, identified, and prior models
    alpha_space = np.linspace(-0.2, 0.2, 100)
    fit_f = pacejka_formula(C_Pf_ref, alpha_space, F_zf)
    fit_r = pacejka_formula(C_Pr_ref, alpha_space, F_zr)

    ax1.plot(alpha_space, fit_f, "Black", label="Reference Model")
    ax2.plot(alpha_space, fit_r, "Black", label="Reference Model")

    fit_f = pacejka_formula(C_Pf_identified, alpha_space, F_zf)
    fit_r = pacejka_formula(C_Pr_identified, alpha_space, F_zr)

    ax1.plot(alpha_space, fit_f, "Blue", label="Identified Model")
    ax2.plot(alpha_space, fit_r, "Blue", label="Identified Model")

    fit_f = pacejka_formula(C_Pf_model, alpha_space, F_zf)
    fit_r = pacejka_formula(C_Pr_model, alpha_space, F_zr)

    ax1.plot(alpha_space, fit_f, "Red", label="Prior Model")
    ax2.plot(alpha_space, fit_r, "Red", label="Prior Model")

    # Formatting
    ax1.set_title("Front tires")
    ax2.set_title("Rear tires")
    fig.suptitle(
        f"System Identification Results After Iteration {iteration}", fontsize=16
    )
    for ax in [ax1, ax2]:
        ax.set_xlabel(r"$\alpha$ [rad]")
        ax.set_ylabel(r"$F_y$ [N]")
        ax.grid()
        ax.legend(loc="best")
        ax.set_xlim([-0.2, 0.2])
        ax.set_ylim([-15, 15])
    plt.tight_layout()

    if training_data is not None:
        fig2, axs = plt.subplots(4, 1, figsize=(8, 8), sharex=True)
        t_ax = np.arange(len(training_data)) * 0.02

        axs[0].plot(t_ax, training_data[:, 0], color="blue", label="v_x")
        axs[0].set_ylabel("v_x [m/s]")
        axs[0].grid(True)
        axs[0].legend(loc="upper right")

        axs[1].plot(t_ax, training_data[:, 1], color="red", label="v_y")
        axs[1].set_ylabel("v_y [m/s]")
        axs[1].grid(True)
        axs[1].legend(loc="upper right")

        axs[2].plot(t_ax, training_data[:, 2], color="green", label="omega")
        axs[2].set_ylabel("omega [rad/s]")
        axs[2].grid(True)
        axs[2].legend(loc="upper right")

        axs[3].plot(t_ax, training_data[:, 3], color="black", label="delta")
        axs[3].set_ylabel("delta [rad]")
        axs[3].set_xlabel("Time [s]")
        axs[3].grid(True)
        axs[3].legend(loc="upper right")

        fig2.suptitle("Received Raw Training Data", fontsize=16)
        plt.tight_layout()

    plt.show()


def plot_results(
    model,
    v_x,
    v_y,
    omega,
    delta,
    C_Pf_identified,
    C_Pr_identified,
    iteration,
    training_data=None,
):
    """
    Plot system identification results.

    Plots the system identification results for front and rear tires after each iteration of the training.
    Uses multiprocessing to spawn a separate process for the matplotlib GUI to prevent
    issues when called from a background thread (e.g. ZMQ workers).
    """
    p = multiprocessing.Process(
        target=_run_plot,
        args=(
            model,
            v_x,
            v_y,
            omega,
            delta,
            C_Pf_identified,
            C_Pr_identified,
            iteration,
            training_data,
        ),
    )
    p.start()
    p.join()


# =====================================================================
# Combined multi-iteration plot
# =====================================================================

def _run_plot_all_iterations(model, all_iterations_data, training_data=None):
    """Render all iteration results in a single figure (runs in subprocess)."""
    import matplotlib.pyplot as plt
    from matplotlib import colormaps
    from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Calibration.On_Track_SysID.src.helpers.pacejka_formula import pacejka_formula

    C_Pf_ref = model["C_Pf_ref"]
    C_Pr_ref = model["C_Pr_ref"]
    C_Pf_prior = all_iterations_data[0]["model_snapshot"]["C_Pf_model"]
    C_Pr_prior = all_iterations_data[0]["model_snapshot"]["C_Pr_model"]

    l_f = model["l_f"]
    l_r = model["l_r"]
    l_wb = model["l_wb"]
    m = model["m"]
    g_ = 9.81

    F_zf = m * g_ * l_r / l_wb
    F_zr = m * g_ * l_f / l_wb

    alpha_space = np.linspace(-0.2, 0.2, 100)

    num_iters = len(all_iterations_data)
    cmap = colormaps.get_cmap("cool")

    # ------ Figure 1: Pacejka curves for all iterations ------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    # Reference and prior (constant across iterations)
    ax1.plot(alpha_space, pacejka_formula(C_Pf_ref, alpha_space, F_zf),
             "k-", linewidth=2, label="Reference Model")
    ax2.plot(alpha_space, pacejka_formula(C_Pr_ref, alpha_space, F_zr),
             "k-", linewidth=2, label="Reference Model")

    ax1.plot(alpha_space, pacejka_formula(C_Pf_prior, alpha_space, F_zf),
             "r--", linewidth=2, label="Prior Model")
    ax2.plot(alpha_space, pacejka_formula(C_Pr_prior, alpha_space, F_zr),
             "r--", linewidth=2, label="Prior Model")

    for idx, it_data in enumerate(all_iterations_data):
        iteration = it_data["iteration"]
        v_x = it_data["v_x"]
        v_y = it_data["v_y"]
        omega = it_data["omega"]
        delta = it_data["delta"]
        C_Pf_id = it_data["C_Pf_identified"]
        C_Pr_id = it_data["C_Pr_identified"]
        color = cmap(idx / max(num_iters - 1, 1))

        # Scatter: generated data for this iteration
        F_yf = m * l_r * v_x * omega / ((l_r + l_f) * np.cos(delta))
        F_yr = m * l_f * v_x * omega / (l_r + l_f)
        alpha_f = -np.arctan((v_y + l_f * omega) / v_x) + delta
        alpha_r = -np.arctan((v_y - l_r * omega) / v_x)

        ax1.scatter(alpha_f, F_yf, color=color, alpha=0.3, s=1)
        ax2.scatter(alpha_r, F_yr, color=color, alpha=0.3, s=1)

        # Identified curve
        fit_f = pacejka_formula(C_Pf_id, alpha_space, F_zf)
        fit_r = pacejka_formula(C_Pr_id, alpha_space, F_zr)
        ax1.plot(alpha_space, fit_f, color=color, linewidth=1.5,
                 label=f"Iter {iteration}")
        ax2.plot(alpha_space, fit_r, color=color, linewidth=1.5,
                 label=f"Iter {iteration}")

    ax1.set_title("Front tires")
    ax2.set_title("Rear tires")
    fig.suptitle("System Identification — All Iterations", fontsize=16)
    for ax in [ax1, ax2]:
        ax.set_xlabel(r"$\alpha$ [rad]")
        ax.set_ylabel(r"$F_y$ [N]")
        ax.grid(True)
        ax.legend(loc="best", fontsize=7, ncol=2)
        ax.set_xlim([-0.2, 0.2])
        ax.set_ylim([-15, 15])
    plt.tight_layout()

    # ------ Figure 2: Pacejka coefficient convergence ------
    iterations = [d["iteration"] for d in all_iterations_data]
    C_Pf_all = np.array([d["C_Pf_identified"] for d in all_iterations_data])
    C_Pr_all = np.array([d["C_Pr_identified"] for d in all_iterations_data])
    num_coeffs = C_Pf_all.shape[1] if C_Pf_all.ndim == 2 else 1

    coeff_labels = ["B", "C", "D", "E"] if num_coeffs == 4 else [
        f"c{j}" for j in range(num_coeffs)
    ]

    fig2, axs2 = plt.subplots(num_coeffs, 2, figsize=(12, 3 * num_coeffs),
                               sharex=True, squeeze=False)
    for j in range(num_coeffs):
        axs2[j, 0].plot(iterations, C_Pf_all[:, j], "o-", color="blue")
        axs2[j, 0].set_ylabel(f"Front {coeff_labels[j]}")
        axs2[j, 0].grid(True)

        axs2[j, 1].plot(iterations, C_Pr_all[:, j], "o-", color="green")
        axs2[j, 1].set_ylabel(f"Rear {coeff_labels[j]}")
        axs2[j, 1].grid(True)

    axs2[-1, 0].set_xlabel("Iteration")
    axs2[-1, 1].set_xlabel("Iteration")
    fig2.suptitle("Pacejka Coefficient Convergence", fontsize=16)
    plt.tight_layout()

    # ------ Figure 3: Raw training data (if provided) ------
    if training_data is not None:
        fig3, axs3 = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
        t_ax = np.arange(len(training_data)) * 0.02

        labels_units = [
            ("v_x", "v_x [m/s]", "blue"),
            ("v_y", "v_y [m/s]", "red"),
            ("omega", "omega [rad/s]", "green"),
            ("delta", "delta [rad]", "black"),
        ]
        for k, (lbl, ylabel, clr) in enumerate(labels_units):
            axs3[k].plot(t_ax, training_data[:, k], color=clr, label=lbl)
            axs3[k].set_ylabel(ylabel)
            axs3[k].grid(True)
            axs3[k].legend(loc="upper right")

        axs3[-1].set_xlabel("Time [s]")
        fig3.suptitle("Received Raw Training Data", fontsize=16)
        plt.tight_layout()

    plt.show()


def plot_all_iterations(model, all_iterations_data, training_data=None):
    """
    Plot combined results from ALL training iterations.

    Shows every identified Pacejka curve overlaid on the same axes (colour-coded
    by iteration), a coefficient convergence chart, and optionally the raw
    training data.  Uses multiprocessing to keep the matplotlib GUI out of the
    calling thread.
    """
    p = multiprocessing.Process(
        target=_run_plot_all_iterations,
        args=(model, all_iterations_data, training_data),
    )
    p.start()
    p.join()
