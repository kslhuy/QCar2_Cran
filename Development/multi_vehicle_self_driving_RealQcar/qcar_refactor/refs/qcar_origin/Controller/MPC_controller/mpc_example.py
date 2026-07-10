"""
Example Usage of CasADi MPC Controller

Demonstrates how to use the MPC controller for vehicle control:
1. As a standalone combined controller
2. With existing controller interfaces (wrappers)
3. With different vehicle models (kinematic vs dynamic)
"""
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Add project root to path to allow importing qcar package
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if project_root not in sys.path:
    sys.path.append(project_root)

try:
    from qcar.simulation.vehiclemodels.vehicle_parameters import get_qcar_parameters
except ImportError:
    # When running directly from Controller/ directory, resolve via relative path
    _sim_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'simulation')
    if _sim_dir not in sys.path:
        sys.path.insert(0, _sim_dir)
    from vehiclemodels.vehicle_parameters import get_qcar_parameters

def example_standalone_mpc():
    """Example: Using MPC as a standalone combined controller."""
    print("=" * 60)
    print("Example 1: Standalone MPC Controller")
    print("=" * 60)
    
    from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Controller.MPC_controller.mpc_controller import CasADiMPCController
    
    # Load QCar parameters
    qcar_params = get_qcar_parameters()
    
    # Create MPC controller with QCar parameters
    mpc = CasADiMPCController(
        params=qcar_params,
        horizon=10,
        dt_mpc=0.05,
        desired_spacing=0.5,
        time_headway=0.5,
    )
    
    # Simulation parameters
    dt = 0.05
    T = 10.0
    steps = int(T / dt)
    
    # Initial state
    x, y, theta, v = 0.0, 0.0, 0.0, 0.5
    
    # Leader state (moving ahead)
    leader_x, leader_y, leader_theta, leader_v = 2.0, 0.0, 0.0, 0.5
    
    # Storage for plotting
    trajectory_x = [x]
    trajectory_y = [y]
    leader_traj_x = [leader_x]
    leader_traj_y = [leader_y]
    throttle_history = []
    steering_history = []
    
    print(f"Running simulation for {T} seconds...")
    
    for i in range(steps):
        # Create state dictionaries
        follower_state = {
            'x': x, 'y': y, 'theta': theta, 'velocity': v,
            'target_velocity': 0.5
        }
        
        leader_state = {
            'x': leader_x, 'y': leader_y, 
            'theta': leader_theta, 'velocity': leader_v
        }
        
        # Compute control
        throttle, steering = mpc.compute_control(follower_state, leader_state, dt)
        
        # Store history
        throttle_history.append(throttle)
        steering_history.append(steering)
        
        # Simple kinematic update for follower
        # Note: Using hardcoded kinematic model for simulation loop itself 
        # (independent of controller model)
        acc = throttle * 2.0  # Max acceleration (approx)
        v = max(0.0, v + acc * dt)
        beta = np.arctan(qcar_params.a / (qcar_params.a + qcar_params.b) * np.tan(steering))
        x += v * np.cos(theta + beta) * dt
        y += v * np.sin(theta + beta) * dt
        theta += v / qcar_params.a * np.sin(beta) * dt
        
        # Simple update for leader (constant velocity, slight turn)
        if i > steps // 2:
            leader_theta += 0.01  # Gentle turn
        leader_x += leader_v * np.cos(leader_theta) * dt
        leader_y += leader_v * np.sin(leader_theta) * dt
        
        trajectory_x.append(x)
        trajectory_y.append(y)
        leader_traj_x.append(leader_x)
        leader_traj_y.append(leader_y)
    
    # Plot results
    # (Plotting code remains same, omitted for brevity in replacement if not changed)
    # Re-including plotting code to ensure continuity
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # Trajectory plot
    axes[0, 0].plot(trajectory_x, trajectory_y, 'b-', label='Follower')
    axes[0, 0].plot(leader_traj_x, leader_traj_y, 'r--', label='Leader')
    axes[0, 0].plot(trajectory_x[0], trajectory_y[0], 'go', markersize=10, label='Start')
    axes[0, 0].set_xlabel('X [m]')
    axes[0, 0].set_ylabel('Y [m]')
    axes[0, 0].set_title('Vehicle Trajectories')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].axis('equal')
    
    # Throttle plot
    time = np.arange(len(throttle_history)) * dt
    axes[0, 1].plot(time, throttle_history, 'b-')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Throttle')
    axes[0, 1].set_title('Throttle Command')
    axes[0, 1].grid(True)
    
    # Steering plot
    axes[1, 0].plot(time, np.rad2deg(steering_history), 'g-')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Steering Angle [deg]')
    axes[1, 0].set_title('Steering Command')
    axes[1, 0].grid(True)
    
    # Distance to leader
    distances = [np.sqrt((lx - fx)**2 + (ly - fy)**2) 
                 for fx, fy, lx, ly in zip(trajectory_x, trajectory_y, 
                                           leader_traj_x, leader_traj_y)]
    time_dist = np.arange(len(distances)) * dt
    axes[1, 1].plot(time_dist, distances, 'r-')
    axes[1, 1].axhline(y=0.5, color='k', linestyle='--', label='Desired spacing')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Distance [m]')
    axes[1, 1].set_title('Distance to Leader')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig('mpc_example_standalone.png', dpi=150)
    plt.show()
    
    print("Simulation complete!")
    print(f"Final distance to leader: {distances[-1]:.3f} m")


def example_with_factory():
    """Example: Using MPC via the factory (simplest approach)."""
    print("\n" + "=" * 60)
    print("Example 2: MPC via Factory (Standalone)")
    print("=" * 60)
    
    from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Controller.MPC_controller.mpc_controller import MPCControllerFactory
    
    # Create MPC controller via factory - auto-loads QCar vehicle params
    mpc = MPCControllerFactory.create('mpc', params={
        'horizon': 15,
        'dt_mpc': 0.05,
        'Q_pos': 15.0,
        'Q_heading': 80.0,
    })
    
    # Test with a simple scenario
    follower_state = {
        'x': 0.0, 'y': 0.5, 'theta': 0.0, 'velocity': 0.5,
        'target_velocity': 0.5
    }
    
    leader_state = {
        'x': 1.5, 'y': 0.0, 'theta': 0.0, 'velocity': 0.5
    }
    
    throttle, steering = mpc.compute_control(follower_state, leader_state, dt=0.05)
    
    print(f"Input states:")
    print(f"  Follower: x={follower_state['x']}, y={follower_state['y']}, "
          f"theta={follower_state['theta']}, v={follower_state['velocity']}")
    print(f"  Leader: x={leader_state['x']}, y={leader_state['y']}, "
          f"theta={leader_state['theta']}, v={leader_state['velocity']}")
    print(f"\nMPC Output:")
    print(f"  Throttle: {throttle:.4f}")
    print(f"  Steering: {steering:.4f} rad ({np.rad2deg(steering):.2f} deg)")
    
    # Reset and test again
    mpc.reset()
    throttle2, steering2 = mpc.compute_control(follower_state, leader_state, dt=0.05)
    print(f"\nAfter reset:")
    print(f"  Throttle: {throttle2:.4f}")
    print(f"  Steering: {steering2:.4f} rad ({np.rad2deg(steering2):.2f} deg)")


def example_path_following():
    """Example: Using MPC for waypoint path following (like Stanley controller)."""
    print("\n" + "=" * 60)
    print("Example 3: MPC Waypoint Path Following")
    print("=" * 60)
    
    from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Controller.MPC_controller.mpc_controller import CasADiMPCController
    
    # Load parameters
    qcar_params = get_qcar_parameters()
    
    # Create MPC controller
    # Anti-corner-cutting tuning: Q (tracking) >>> R (input penalty)
    # + decaying Q_pos over horizon + compressed lookahead
    mpc = CasADiMPCController(
        params=qcar_params,
        horizon=12,
        dt_mpc=0.05,
        Q_pos=500.0,        # Very high: stay ON the path
        Q_heading=300.0,    # Very high: align with path heading
        Q_vel=10.0,         # Moderate: reach target speed
        R_delta=1.0,        # Very low: don't penalize steering
        R_acc=1.0,          # Very low: don't penalize acceleration
        R_delta_rate=3.0,   # Very low: allow fast steering changes
        R_acc_rate=3.0,
        max_steering_rate=3.0,
        path_lookahead_scale=0.4,  # Compress lookahead to reduce curve preview
    )
    
    # # Create a figure-8 waypoint path (shape [2, N])
    # t = np.linspace(0, 2 * np.pi, 200)
    # scale = 2.0
    # wp_x = scale * np.sin(t)
    # wp_y = scale * np.sin(t) * np.cos(t)
    # waypoints = np.vstack([wp_x, wp_y])  # [2, 200]

    # Create an oval waypoint path (shape [2, N]) — easier to track than figure-8
    t = np.linspace(0, 2 * np.pi, 300, endpoint=False)
    rx, ry = 3.0, 1.5   # semi-axes of the oval
    wp_x = rx * np.cos(t)
    wp_y = ry * np.sin(t)
    waypoints = np.vstack([wp_x, wp_y])  # [2, 300]
    
    # Set waypoints on MPC (same interface as StanleyController)
    mpc.set_waypoints(waypoints, cyclic=True)
    
    # Simulation
    dt = 0.05
    T = 30.0
    steps = int(T / dt)
    
    # Start at the first waypoint with correct tangent heading
    x, y = waypoints[0, 0], waypoints[1, 0]
    dx = waypoints[0, 1] - waypoints[0, 0]
    dy = waypoints[1, 1] - waypoints[1, 0]
    theta = np.arctan2(dy, dx)  # Align with path direction
    v = 0.0
    target_v = 0.4
    
    # Storage
    trajectory_x = [x]
    trajectory_y = [y]
    
    print(f"Running waypoint path following for {T} seconds ({waypoints.shape[1]} waypoints)...")
    
    for i in range(steps):
        follower_state = {
            'x': x, 'y': y, 'theta': theta, 'velocity': v,
            'target_velocity': target_v
        }
        
        # No leader - MPC follows waypoints automatically
        throttle, steering = mpc.compute_control(follower_state, None, dt)
        
        # Update state (simple kinematic bicycle model)
        acc = throttle * qcar_params.longitudinal.a_max
        v = max(0.0, min(qcar_params.longitudinal.v_max, v + acc * dt))
        beta = np.arctan(qcar_params.b / (qcar_params.a + qcar_params.b) * np.tan(steering))
        x += v * np.cos(theta + beta) * dt
        y += v * np.sin(theta + beta) * dt
        theta += v / qcar_params.b * np.sin(beta) * dt
        
        trajectory_x.append(x)
        trajectory_y.append(y)
    
    # Plot
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # Trajectory vs waypoints
    axes[0].plot(waypoints[0], waypoints[1], 'r--', linewidth=1, alpha=0.5, label='Waypoints')
    axes[0].plot(trajectory_x, trajectory_y, 'b-', linewidth=2, label='MPC trajectory')
    axes[0].plot(trajectory_x[0], trajectory_y[0], 'go', markersize=10, label='Start')
    axes[0].set_xlabel('X [m]')
    axes[0].set_ylabel('Y [m]')
    axes[0].set_title('MPC Waypoint Path Following')
    axes[0].legend()
    axes[0].grid(True)
    axes[0].axis('equal')
    
    # Cross-track error over time
    cte = []
    for tx, ty in zip(trajectory_x, trajectory_y):
        dists = np.sqrt((waypoints[0] - tx)**2 + (waypoints[1] - ty)**2)
        cte.append(np.min(dists))
    time_vec = np.arange(len(cte)) * dt
    axes[1].plot(time_vec, cte, 'r-')
    axes[1].set_xlabel('Time [s]')
    axes[1].set_ylabel('Cross-Track Error [m]')
    axes[1].set_title('Tracking Error')
    axes[1].grid(True)
    
    plt.tight_layout()
    plt.savefig('mpc_example_path_following.png', dpi=150)
    plt.show()
    
    print(f"Simulation complete!")
    print(f"Final position: ({x:.3f}, {y:.3f})")
    print(f"Mean cross-track error: {np.mean(cte):.4f} m")
    print(f"Max cross-track error: {np.max(cte):.4f} m")


def example_dynamic_model():
    """Example: Using dynamic bicycle model MPC."""
    print("\n" + "=" * 60)
    print("Example 4: Dynamic Bicycle Model MPC")
    print("=" * 60)
    
    from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Controller.MPC_controller.mpc_controller import DynamicBicycleMPCController
    
    # Load parameters
    qcar_params = get_qcar_parameters()
    
    # Create dynamic MPC controller
    mpc = DynamicBicycleMPCController(
        params=qcar_params,
        horizon=10,
        dt_mpc=0.02,
    )
    
    # Test
    follower_state = {
        'x': 0.0, 'y': 0.0, 'theta': 0.0, 'velocity': 0.5,
        'y_dot': 0.0, 'psi_dot': 0.0,
        'target_velocity': 0.8
    }
    
    leader_state = {
        'x': 1.5, 'y': 0.2, 'theta': 0.1, 'velocity': 0.6
    }
    
    throttle, steering = mpc.compute_control(follower_state, leader_state, dt=0.02)
    
    print(f"Dynamic MPC Output:")
    print(f"  Throttle: {throttle:.4f}")
    print(f"  Steering: {steering:.4f} rad ({np.rad2deg(steering):.2f} deg)")


def example_factory_usage():
    """Example: Using the MPC controller factory."""
    print("\n" + "=" * 60)
    print("Example 5: Using MPC Controller Factory")
    print("=" * 60)
    
    from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Controller.MPC_controller.mpc_controller import MPCControllerFactory
    
    # Create kinematic MPC (only pass valid MPC config keys, not vehicle params)
    mpc_kinematic = MPCControllerFactory.create(
        'casadi_kinematic',
        params={'horizon': 10, 'max_steering_rate': 0.4}
    )
    print(f"Created kinematic MPC: {type(mpc_kinematic).__name__}")
    
    # Create dynamic MPC (vehicle params loaded automatically from YAML)
    mpc_dynamic = MPCControllerFactory.create(
        'casadi_dynamic',
        params={'horizon': 8}
    )
    print(f"Created dynamic MPC: {type(mpc_dynamic).__name__}")
    
    # Use default alias
    mpc_default = MPCControllerFactory.create('mpc')
    print(f"Created default MPC: {type(mpc_default).__name__}")


if __name__ == "__main__":
    print("CasADi MPC Controller Examples")
    print("=" * 60)
    
    # Check if casadi is available
    try:
        import casadi as ca
        print(f"CasADi version: {ca.__version__}")
    except ImportError:
        print("ERROR: CasADi not installed!")
        print("Install with: pip install casadi")
        exit(1)
    
    # Run examples
    try:
        example_factory_usage()
        example_with_factory()
        example_path_following()
        example_dynamic_model()
        example_standalone_mpc()
    except Exception as e:
        print(f"\nError running examples: {e}")
        import traceback
        traceback.print_exc()
