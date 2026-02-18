import os
import sys

import numpy as np

# # Ensure this repository's hal package is used first.
# THIS_DIR = os.path.dirname(os.path.abspath(__file__))
# HAL_LIB_DIR = os.path.normpath(
#     os.path.join(THIS_DIR, "..", "..", "..", "0_libraries", "python")
# )
# if HAL_LIB_DIR not in sys.path:
#     sys.path.insert(0, HAL_LIB_DIR)

from path_rich import LocalObstacle, RichSDCSPlanner, SpeedZone


def main():
    # Choose a valid node route for right-hand traffic full map.
    node_sequence = [10, 2, 4, 14, 20, 22, 9, 7, 0]

    planner = RichSDCSPlanner(
        leftHandTraffic=False,
        useSmallMap=False,
        sample_ds=0.02,
        is_cyclic=False,
    )
    base_route = planner.set_route(node_sequence)
    if base_route is None:
        print("No path found for the provided node sequence.")
        return

    base_ds = np.hypot(np.diff(base_route[0, :]), np.diff(base_route[1, :]))
    route_length = float(np.sum(base_ds))
    desired_speed = 0.65
    max_speed = 0.90
    hard_turn_kappa = 0.85
    hard_turn_speed = 0.32

    planner.set_speed_zones(
        [
            SpeedZone(
                s_start=0.18 * route_length,
                s_end=0.30 * route_length,
                speed_scale=1.20,
                max_speed=0.90,
            ),
            SpeedZone(
                s_start=0.50 * route_length,
                s_end=0.72 * route_length,
                speed_scale=0.68,
                max_speed=0.45,
            ),
        ]
    )

    # Simulated real-time replanning loop with moving obstacles.
    current_pose = np.array([base_route[0, 0], base_route[1, 0], 0.0], dtype=float)
    route_start = np.array([base_route[0, 0], base_route[1, 0]], dtype=float)
    route_end = np.array([base_route[0, -1], base_route[1, -1]], dtype=float)
    local_trajs = []
    obstacle_trace = []

    n_steps = 60
    for k in range(n_steps):
        obstacles = [
            LocalObstacle(
                x=0.20 + 0.010 * k,
                y=0.55 + 0.15 * np.sin(0.12 * k),
                radius=0.18,
                influence=0.70,
                clearance=0.12,
                vx=0.10,
                vy=0.00,
            ),
            LocalObstacle(
                x=-0.60 + 0.004 * k,
                y=0.95 - 0.12 * np.cos(0.10 * k),
                radius=0.15,
                influence=0.60,
                clearance=0.10,
                vx=0.04,
                vy=0.00,
            ),
        ]

        traj_rt = planner.update_realtime(
            current_pose=current_pose,
            obstacles=obstacles,
            lookahead_distance=3.2,
            behind_distance=0.5,
            prediction_time=0.30,
            desired_speed=desired_speed,
            min_speed=0.20,
            kappa_speed_gain=2.2,
            max_speed=max_speed,
            hard_turn_kappa=hard_turn_kappa,
            hard_turn_speed=hard_turn_speed,
        )
        if traj_rt is None or traj_rt.shape[0] < 5:
            break

        local_trajs.append(traj_rt)
        obstacle_trace.append([(obs.x, obs.y) for obs in obstacles])

        # Advance ego pose along the latest local trajectory.
        center_idx = int(np.ceil(0.5 / max(planner.sample_ds, 1e-6)))
        advance_idx = int(np.ceil(0.35 / max(planner.sample_ds, 1e-6)))
        step_idx = min(center_idx + advance_idx, traj_rt.shape[0] - 1)
        current_pose = np.array(
            [traj_rt[step_idx, 1], traj_rt[step_idx, 2], traj_rt[step_idx, 3]],
            dtype=float,
        )

    if not local_trajs:
        print("Realtime planner failed to produce any local trajectory.")
        return

    traj_with_obs = local_trajs[-1]
    traj_no_obs = planner.path_to_rich_trajectory(
        base_route,
        desired_speed=desired_speed,
        min_speed=0.20,
        kappa_speed_gain=2.2,
        max_speed=max_speed,
        hard_turn_kappa=hard_turn_kappa,
        hard_turn_speed=hard_turn_speed,
    )
    if traj_no_obs is None:
        print("Failed to produce baseline trajectory.")
        return

    print("Trajectory columns: [s, x, y, heading, curvature, vx, ax]")
    print("Realtime replans:", len(local_trajs))
    print("No-obstacle trajectory shape:", traj_no_obs.shape)
    print("Latest local trajectory shape:", traj_with_obs.shape)
    print("First row sample (latest local):", np.round(traj_with_obs[0, :], 4))
    print(
        "Speed settings:",
        {
            "desired_speed": desired_speed,
            "max_speed": max_speed,
            "hard_turn_kappa": hard_turn_kappa,
            "hard_turn_speed": hard_turn_speed,
        },
    )

    if "--plot" not in sys.argv:
        print("Plot disabled by default. Run with '--plot' to visualize.")
        return

    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print("Matplotlib is unavailable in this environment; skipping plots.")
        print("Reason:", exc)
        return

    fig, (ax_map, ax_speed) = plt.subplots(1, 2, figsize=(12, 5))

    ax_map.plot(traj_no_obs[:, 1], traj_no_obs[:, 2], "g--", linewidth=1.2, label="Base path")
    sampled = local_trajs[::8]
    for idx, tr in enumerate(sampled):
        alpha = 0.20 + 0.70 * (idx + 1) / max(len(sampled), 1)
        ax_map.plot(tr[:, 1], tr[:, 2], color="tab:red", alpha=alpha, linewidth=1.0)
    ax_map.plot(traj_with_obs[:, 1], traj_with_obs[:, 2], "r", linewidth=2.0, label="Latest local path")

    start_x, start_y = float(route_start[0]), float(route_start[1])
    end_x, end_y = float(route_end[0]), float(route_end[1])
    ax_map.plot(
        start_x,
        start_y,
        marker="o",
        markersize=8,
        markeredgecolor="k",
        markerfacecolor="lime",
        linestyle="None",
        label="Start",
    )
    ax_map.plot(
        end_x,
        end_y,
        marker="X",
        markersize=9,
        markeredgecolor="k",
        markerfacecolor="gold",
        linestyle="None",
        label="Goal",
    )

    for obs_xy in obstacle_trace:
        for ox, oy in obs_xy:
            ax_map.plot(ox, oy, "b.", markersize=2, alpha=0.35)
    for ox, oy in obstacle_trace[-1]:
        circle = plt.Circle((ox, oy), 0.18, color="tab:blue", fill=False, linewidth=1.5)
        ax_map.add_patch(circle)
        ax_map.plot(ox, oy, "bo", markersize=3)

    ax_map.set_title("Real-Time Local Replanning (Simulated Obstacles)")
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.axis("equal")
    ax_map.grid(True, alpha=0.3)
    ax_map.legend(loc="best")

    ax_speed.plot(traj_with_obs[:, 0], traj_with_obs[:, 5], "k", linewidth=1.6, label="vx [m/s]")
    ax_speed.plot(traj_with_obs[:, 0], traj_with_obs[:, 6], "tab:orange", linewidth=1.2, label="ax [m/s^2]")
    ax_speed.set_title("Rich Trajectory Speed Terms")
    ax_speed.set_xlabel("s [m]")
    ax_speed.grid(True, alpha=0.3)
    ax_speed.legend(loc="best")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
