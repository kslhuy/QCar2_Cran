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
    # node_sequence = [10, 2, 4, 14, 20, 22, 9, 7, 0]
    node_sequence =  [0, 2, 4, 6, 13, 19, 17, 15, 6, 0]

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
    desired_speed = 2.0  # m/s, nominal speed for the trajectory (not a hard limit)
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
    # derive time axis from s and vx for both trajectories; dt = ds/vx
    def compute_time_axis(traj):
        s = traj[:,0]
        vx = traj[:,5]
        t = np.zeros_like(s)
        for i in range(1, len(s)):
            ds = s[i] - s[i-1]
            v = max(vx[i], 1e-3)
            t[i] = t[i-1] + ds / v
        return t
    time_no_obs = compute_time_axis(traj_no_obs)
    time_with_obs = compute_time_axis(traj_with_obs)
    if traj_no_obs is None:
        print("Failed to produce baseline trajectory.")
        return

    # The planner returns a "rich" trajectory array whose columns are:
    #   s        : accumulated path distance [m], measured along the center
    #              line of the route.  Think of it as the arc‑length coordinate;
    #              every point increments s by the Euclidean distance from the
    #              previous point.  This axis is convenient for plotting and for
    #              applying speed zones, since everything is specified in meters
    #              along the path rather than global x/y.
    #   x, y     : world coordinates of each sample.
    #   heading  : path heading (rad) measured in the global frame.  Zero is
    #               along +x, positive turns toward +y.
    #   curvature: path curvature (1/m) = d(heading)/ds.  Large magnitude
    #               indicates a tight turn.  Sign convention corresponds to
    #               left/right curvature.
    #   vx       : planned speed at each sample point [m/s].
    #   ax       : derivative of vx w.r.t. s (i.e. speed change per meter).
    #
    # The values above are all computed in the *Frenet frame*, which is the
    # coordinate system attached to the route: s along the path and d lateral
    # offset.  In this example we don’t project anything into Frenet; the
    # planner uses the base path for calculations, and you can obtain Frenet
    # coordinates via `RichSDCSPlanner._map_path_to_base_s` or the utilities in
    # `pp_map_utils`.  Frenet coordinates are useful on the car because they
    # separate longitudinal and lateral errors and make it easy to specify
    # speed/curvature constraints relative to progress along the route.
    #
    # Note: `ax` is not a measured vehicle acceleration; it is derived purely
    # from the planner's speed profile.  Values can appear large (10–15 m/s²)
    # because they represent how quickly the *planned* speed changes along the
    # path, not the car's actual longitudinal acceleration.  In a real car the
    # controller would smooth/limit these when converting to throttle/brake
    # commands.
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
    # show actual planned speed range to illustrate why vx never reaches desired_speed
    print(
        f"planned vx min/max (no obs): {traj_no_obs[:,5].min():.3f}/{traj_no_obs[:,5].max():.3f}"
    )
    print("(Note: max_speed parameter and curvature scaling limit the output speed.)")

    # --- demonstration of PP waypoint conversion and v_ref scaling ---
    try:
        from StateMachine.following_path.pp_map_utils import build_pp_waypoint_array, update_pp_runtime_speed_profile
    except ImportError:
        # add qcar package root (one level up) to sys.path
        import os, sys

        HERE = os.path.abspath(os.path.dirname(__file__))
        # qcar/PathPlanner -> go up one directory to qcar
        QCAR_ROOT = os.path.normpath(os.path.join(HERE, ".."))
        if QCAR_ROOT not in sys.path:
            sys.path.insert(0, QCAR_ROOT)
        try:
            from StateMachine.following_path.pp_map_utils import build_pp_waypoint_array, update_pp_runtime_speed_profile
        except ImportError:
            # fallback: also try workspace root
            WS_ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
            if WS_ROOT not in sys.path:
                sys.path.insert(0, WS_ROOT)
            from StateMachine.following_path.pp_map_utils import build_pp_waypoint_array, update_pp_runtime_speed_profile

    rich_path = traj_no_obs  # baseline rich trajectory
    # convert to PP-style waypoint array (N,7)
    pp_waypoints, speed_profile_base, s_axis, track_length, design_speed = build_pp_waypoint_array(
        rich_planner=planner,
        waypoint_sequence=base_route,
        params={"desired_speed": desired_speed, "min_speed":0.2, "kappa_speed_gain":2.2, "max_speed":max_speed,
                "hard_turn_kappa":hard_turn_kappa, "hard_turn_speed":hard_turn_speed},
    )
    print("PP waypoint array shape:", pp_waypoints.shape)

    # simulate changing v_ref at runtime and scale profile
    for vref in [0.3, 0.5, 0.7, 1.0]:
        scale = update_pp_runtime_speed_profile(
            pp_waypoint_array=pp_waypoints,
            pp_speed_profile_base=speed_profile_base,
            profile_design_speed=design_speed,
            v_ref_runtime=vref,
        )
        print(f"v_ref={vref:.2f} -> speed_scale={scale:.3f}, min/max speed={pp_waypoints[:,2].min():.3f}/{pp_waypoints[:,2].max():.3f}")

    # save offline data
    out_dir = os.path.join(os.getcwd(), "path_rich_output")
    os.makedirs(out_dir, exist_ok=True)
    np.savetxt(os.path.join(out_dir, "traj_no_obs.csv"), traj_no_obs, delimiter=",")
    np.savetxt(os.path.join(out_dir, "last_local_traj.csv"), traj_with_obs, delimiter=",")
    np.savetxt(os.path.join(out_dir, "pp_waypoints.csv"), pp_waypoints, delimiter=",")
    print(f"All trajectories and waypoint data saved to {out_dir}")

    if "--plot" not in sys.argv:
        print("Plot disabled by default. Run with '--plot' to visualize.")
        return

    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print("Matplotlib is unavailable in this environment; skipping plots.")
        print("Reason:", exc)
        return

    # create 2x2 grid: map, speed profile, heading, curvature
    # create single figure with 3 rows x 2 columns
    # layout:
    #  row0: map | speed+accel
    #  row1: heading | curvature
    #  row2: vx-only | time-domain speed/accel
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    ax_map = axes[0, 0]
    ax_speed = axes[0, 1]
    ax_head = axes[1, 0]
    ax_curv = axes[1, 1]
    ax_vxonly = axes[2, 0]
    ax_time = axes[2, 1]

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

    # speed/accel plot
    # The speed curve here comes directly from the planner call above and is
    # based only on the `desired_speed` parameter.  It does **not** automatically
    # incorporate a separate runtime `v_ref` value (which would typically come
    # from the vehicle manager).  Note that the planner also respects the
    # `max_speed` parameter and reduces vx in tight turns (via curvature gain),
    # so even if desired_speed is high the output will never exceed max_speed
    # and will usually be lower when the curvature is nonzero.
    # If you want to visualize a hypothetical v_ref on the same plot, draw a
    # horizontal line as shown below (using one of the test v_ref values).
    ax_speed.plot(traj_with_obs[:, 0], traj_with_obs[:, 5], "k", linewidth=1.6, label="vx [m/s]")
    ax_speed.plot(traj_with_obs[:, 0], traj_with_obs[:, 6], "tab:orange", linewidth=1.2, label="ax [m/s^2]")
    # show desired_speed and max_speed limits
    ax_speed.axhline(desired_speed, color="tab:blue", linestyle=":",
                     label=f"desired_speed={desired_speed:.1f}")
    ax_speed.axhline(max_speed, color="tab:red", linestyle=":",
                     label=f"max_speed={max_speed:.1f}")
    # overlay an example v_ref line (choose any of the vrefs tested earlier)
    example_vref = 0.7
    ax_speed.axhline(example_vref, color="tab:gray", linestyle="--",
                     label=f"v_ref={example_vref:.2f} m/s")
    ax_speed.set_title("Rich Trajectory Speed Terms")
    ax_speed.set_xlabel("s [m]")
    ax_speed.grid(True, alpha=0.3)
    ax_speed.legend(loc="best")

    # heading plot
    ax_head.plot(traj_with_obs[:, 0], traj_with_obs[:, 3], "tab:green", linewidth=1.2)
    ax_head.set_title("Heading vs. s")
    ax_head.set_xlabel("s [m]")
    ax_head.set_ylabel("heading [rad]")
    ax_head.grid(True, alpha=0.3)

    # curvature plot
    ax_curv.plot(traj_with_obs[:, 0], traj_with_obs[:, 4], "tab:purple", linewidth=1.2)
    ax_curv.set_title("Curvature vs. s")
    ax_curv.set_xlabel("s [m]")
    ax_curv.set_ylabel("curvature [1/m]")
    ax_curv.grid(True, alpha=0.3)

    # vx-only subplot (row2,col0)
    ax_vxonly.plot(traj_with_obs[:,0], traj_with_obs[:,5], "k", linewidth=1.6)
    ax_vxonly.set_title("Speed vx vs. s")
    ax_vxonly.set_xlabel("s [m]")
    ax_vxonly.set_ylabel("vx [m/s]")
    ax_vxonly.grid(True, alpha=0.3)

    # time-domain subplot (row2,col1): use twin y-axis for vx and ax
    ax_time.plot(time_with_obs, traj_with_obs[:,5], "k", label="vx")
    ax_time.set_title("vx & ax vs. time")
    ax_time.set_xlabel("time [s]")
    ax_time.set_ylabel("vx [m/s]")
    ax_time.grid(True, alpha=0.3)
    ax2 = ax_time.twinx()
    ax2.plot(time_with_obs, traj_with_obs[:,6], "tab:orange", label="ax")
    ax2.set_ylabel("ax [m/s^2]")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
