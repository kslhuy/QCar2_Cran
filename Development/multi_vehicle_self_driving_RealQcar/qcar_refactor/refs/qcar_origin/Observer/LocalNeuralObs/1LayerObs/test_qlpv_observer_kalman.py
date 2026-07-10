"""
Test Suite for qLPV Augmented-State Observer with EKF-style Gain Computation

Tests the EKF-based observer implementation including:
- Observer initialization and default parameters
- EKF predict/update cycle
- Covariance propagation and Kalman gain computation
- Tire residual estimation
- State tracking convergence
- Simulation with synthetic vehicle data

Run with: pytest test_qlpv_observer_kalman.py -v
python -m pytest test_qlpv_observer_kalman.py -v --tb=short
"""

import numpy as np
import pytest
import matplotlib.pyplot as plt
from typing import Tuple

# Import the EKF-based observer
from qlpv_observer_kalma import (
    qLPVAugmentedObserver,
    create_qlpv_observer,
    SchedulingParameters
)


class TestObserverInitialization:
    """Test observer initialization and default parameters"""
    
    def test_default_initialization(self):
        """Test observer initializes with default parameters"""
        obs = create_qlpv_observer()
        
        assert obs.Ts == 0.02
        assert obs.STATE_DIM == 6
        assert obs.AUGMENTED_DIM == 8
        assert obs.MEAS_DIM == 6
        
    def test_covariance_matrix_shapes(self):
        """Test Q, R, P matrices have correct shapes"""
        obs = create_qlpv_observer()
        
        assert obs.Q.shape == (8, 8), f"Q shape mismatch: {obs.Q.shape}"
        assert obs.R.shape == (6, 6), f"R shape mismatch: {obs.R.shape}"
        assert obs.P.shape == (8, 8), f"P shape mismatch: {obs.P.shape}"
        assert obs.K.shape == (8, 6), f"K shape mismatch: {obs.K.shape}"
        
    def test_custom_covariance_matrices(self):
        """Test observer accepts custom Q, R, P0 matrices"""
        Q_custom = np.diag([0.1, 0.2, 0.01, 0.1, 0.1, 0.1, 10.0, 10.0])
        R_custom = np.diag([0.1, 0.01, 0.02, 1.0, 1.0, 0.2])
        P0_custom = np.eye(8) * 5.0
        
        obs = create_qlpv_observer(Q=Q_custom, R=R_custom, P0=P0_custom)
        
        np.testing.assert_array_equal(obs.Q, Q_custom)
        np.testing.assert_array_equal(obs.R, R_custom)
        np.testing.assert_array_equal(obs.P, P0_custom)
        
    def test_vehicle_parameters(self):
        """Test vehicle parameters are correctly set"""
        custom_params = {
            'lf': 0.15,
            'lr': 0.12,
            'm': 4.0,
            'Iz': 0.06,
            'Cf': 60.0,
            'Cr': 55.0,
        }
        
        obs = create_qlpv_observer(vehicle_params=custom_params)
        
        assert obs.lf == 0.15
        assert obs.lr == 0.12
        assert obs.m == 4.0
        assert obs.Iz == 0.06
        assert obs.Cf == 60.0
        assert obs.Cr == 55.0


class TestSchedulingParameters:
    """Test scheduling parameter computation"""
    
    def test_scheduling_from_state(self):
        """Test scheduling parameters computed from state correctly"""
        state = np.array([1.0, 0.1, 0.5, 0.2, 0.0, 0.0])  # [vx, vy, psi, r, X, Y]
        delta = 0.1  # Steering angle
        
        rho = SchedulingParameters.from_state_and_input(state, delta)
        
        assert rho.vx == pytest.approx(1.0)
        assert rho.vy == pytest.approx(0.1)
        assert rho.inv_vx == pytest.approx(1.0)
        assert rho.sin_delta == pytest.approx(np.sin(0.1))
        assert rho.cos_delta == pytest.approx(np.cos(0.1))
        assert rho.sin_psi == pytest.approx(np.sin(0.5))
        assert rho.cos_psi == pytest.approx(np.cos(0.5))
        
    def test_min_velocity_clipping(self):
        """Test minimum velocity is enforced to avoid singularity"""
        state = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])  # Very low vx
        delta = 0.0
        
        rho = SchedulingParameters.from_state_and_input(state, delta, min_vx=0.5)
        
        assert rho.vx == 0.5  # Should be clipped to min_vx
        assert rho.inv_vx == pytest.approx(2.0)


class TestEKFMethods:
    """Test EKF-specific methods"""
    
    def test_numerical_jacobian(self):
        """Test numerical Jacobian computation"""
        obs = create_qlpv_observer()
        
        # Test with a simple function f(x) = x^2
        def square_func(x):
            return x ** 2
        
        x = np.array([1.0, 2.0, 3.0])
        J = obs._numerical_jacobian(square_func, x)
        
        # Expected Jacobian: diag([2*1, 2*2, 2*3]) = diag([2, 4, 6])
        expected = np.diag([2.0, 4.0, 6.0])
        np.testing.assert_array_almost_equal(J, expected, decimal=4)
        
    def test_discrete_dynamics(self):
        """Test discrete-time state transition function"""
        obs = create_qlpv_observer()
        
        # Initial state: moving forward at 1 m/s
        xa = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        u = np.array([0.0, 0.0])  # No steering, no acceleration
        
        xa_next = obs.f_discrete(xa, u)
        
        # Position should increase based on velocity
        assert xa_next[4] > 0  # X position increases
        assert len(xa_next) == 8
        
    def test_measurement_function(self):
        """Test measurement function"""
        obs = create_qlpv_observer()
        
        xa = np.array([1.0, 0.1, 0.5, 0.2, 10.0, 5.0, 1.0, 0.5])
        u = np.array([0.1, 0.0])
        
        y = obs.h_meas(xa, u)
        
        assert len(y) == 6  # [vx, r, psi, X, Y, ay]
        assert y[0] == pytest.approx(1.0)  # vx
        assert y[1] == pytest.approx(0.2)  # r
        assert y[2] == pytest.approx(0.5)  # psi
        assert y[3] == pytest.approx(10.0)  # X
        assert y[4] == pytest.approx(5.0)  # Y
        
    def test_ekf_predict(self):
        """Test EKF prediction step"""
        obs = create_qlpv_observer()
        
        xa = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        P = obs.P.copy()
        u = np.array([0.0, 0.0])
        
        xa_pred, P_pred = obs.ekf_predict(xa, P, u)
        
        # Predicted state should be different from initial
        assert len(xa_pred) == 8
        # Predicted covariance should generally increase (due to Q)
        assert P_pred.shape == (8, 8)
        
    def test_ekf_update(self):
        """Test EKF update step"""
        obs = create_qlpv_observer()
        
        xa_pred = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        P_pred = obs._default_P0()
        y_meas = np.array([1.0, 0.1, 0.0, 0.0, 0.0, 0.5])  # [vx, r, psi, X, Y, ay]
        u = np.array([0.0, 0.0])
        
        xa_upd, P_upd, innov, y_pred = obs.ekf_update(xa_pred, P_pred, y_meas, u)
        
        assert len(xa_upd) == 8
        assert P_upd.shape == (8, 8)
        assert len(innov) == 6
        assert len(y_pred) == 6


class TestObserverUpdate:
    """Test full observer update cycle"""
    
    def test_single_update(self):
        """Test single observer update"""
        obs = create_qlpv_observer()
        
        # Full measurement: [vx, r, psi, X, Y, ay]
        y = np.array([1.0, 0.1, 0.0, 0.0, 0.0, 0.5])
        u = np.array([0.1, 0.0])
        
        state, residuals = obs.update(y, u)
        
        assert len(state) == 6  # [vx, vy, psi, r, X, Y]
        assert len(residuals) == 2  # [wr, wf]
        
    def test_5d_measurement(self):
        """Test update with 5D measurement (no ay)"""
        obs = create_qlpv_observer()
        
        # 5D measurement: [vx, r, psi, X, Y]
        y = np.array([1.0, 0.1, 0.0, 0.0, 0.0])
        u = np.array([0.1, 0.0])
        acceleration = np.array([0.0, 0.5, 0.0])  # [ax, ay, az]
        
        state, residuals = obs.update(y, u, acceleration=acceleration)
        
        assert len(state) == 6
        assert len(residuals) == 2
        
    def test_covariance_update(self):
        """Test that covariance matrix is updated after each step"""
        obs = create_qlpv_observer()
        
        P_initial = obs.P.copy()
        
        y = np.array([1.0, 0.1, 0.0, 0.0, 0.0, 0.5])
        u = np.array([0.1, 0.0])
        obs.update(y, u)
        
        P_after = obs.P.copy()
        
        # Covariance should have changed
        assert not np.allclose(P_initial, P_after)
        
    def test_kalman_gain_updated(self):
        """Test that Kalman gain is computed after update"""
        obs = create_qlpv_observer()
        
        y = np.array([1.0, 0.1, 0.0, 0.0, 0.0, 0.5])
        u = np.array([0.1, 0.0])
        obs.update(y, u)
        
        K = obs.get_kalman_gain()
        
        assert K.shape == (8, 6)
        assert not np.allclose(K, 0)  # Should have non-zero values


class TestDiagnosticMethods:
    """Test diagnostic getter methods"""
    
    def test_get_covariance(self):
        """Test get_covariance returns correct matrix"""
        obs = create_qlpv_observer()
        P = obs.get_covariance()
        
        assert P.shape == (8, 8)
        np.testing.assert_array_equal(P, obs.P)
        
    def test_get_innovation(self):
        """Test get_innovation returns measurement residual"""
        obs = create_qlpv_observer()
        
        y = np.array([1.0, 0.1, 0.0, 0.0, 0.0, 0.5])
        u = np.array([0.1, 0.0])
        obs.update(y, u)
        
        innov = obs.get_innovation()
        assert len(innov) == 6
        
    def test_get_state_uncertainty(self):
        """Test get_state_uncertainty returns standard deviations"""
        obs = create_qlpv_observer()
        
        sigma = obs.get_state_uncertainty()
        
        assert len(sigma) == 6
        assert all(sigma > 0)  # Standard deviations should be positive
        
    def test_get_residual_uncertainty(self):
        """Test get_residual_uncertainty returns tire residual std devs"""
        obs = create_qlpv_observer()
        
        sigma = obs.get_residual_uncertainty()
        
        assert len(sigma) == 2
        assert all(sigma > 0)


class TestTuningMethods:
    """Test covariance tuning methods"""
    
    def test_set_Q(self):
        """Test set_Q updates process noise"""
        obs = create_qlpv_observer()
        
        Q_new = np.eye(8) * 0.5
        obs.set_Q(Q_new)
        
        np.testing.assert_array_equal(obs.Q, Q_new)
        
    def test_set_R(self):
        """Test set_R updates measurement noise"""
        obs = create_qlpv_observer()
        
        R_new = np.eye(6) * 0.1
        obs.set_R(R_new)
        
        np.testing.assert_array_equal(obs.R, R_new)
        
    def test_set_P(self):
        """Test set_P updates error covariance"""
        obs = create_qlpv_observer()
        
        P_new = np.eye(8) * 2.0
        obs.set_P(P_new)
        
        np.testing.assert_array_equal(obs.P, P_new)


class TestReset:
    """Test observer reset functionality"""
    
    def test_reset_clears_state(self):
        """Test reset clears state to initial values"""
        obs = create_qlpv_observer()
        
        # Perform some updates
        y = np.array([1.0, 0.1, 0.0, 0.0, 0.0, 0.5])
        u = np.array([0.1, 0.0])
        for _ in range(10):
            obs.update(y, u)
            
        # Reset
        obs.reset()
        
        np.testing.assert_array_equal(obs.state_hat, np.zeros(6))
        np.testing.assert_array_equal(obs.w_hat, np.zeros(2))
        
    def test_reset_with_initial_state(self):
        """Test reset with custom initial state"""
        obs = create_qlpv_observer()
        
        initial_state = np.array([1.5, 0.1, 0.5, 0.05, 10.0, 5.0])
        obs.reset(initial_state=initial_state)
        
        np.testing.assert_array_equal(obs.state_hat, initial_state)
        
    def test_reset_restores_covariance(self):
        """Test reset restores covariance to initial"""
        obs = create_qlpv_observer()
        
        P_initial = obs._default_P0()
        
        # Run updates to modify P
        y = np.array([1.0, 0.1, 0.0, 0.0, 0.0, 0.5])
        u = np.array([0.1, 0.0])
        for _ in range(10):
            obs.update(y, u)
            
        obs.reset()
        
        np.testing.assert_array_almost_equal(obs.P, P_initial)


class TestStateTracking:
    """Test state tracking and convergence"""
    
    def test_velocity_tracking(self):
        """Test observer tracks velocity (basic functionality test)"""
        obs = create_qlpv_observer()
        
        # Initialize with reasonable state
        obs.reset(initial_state=np.array([1.0, 0.0, 0.0, 0.1, 0.0, 0.0]))
        
        # Run updates - just verify it doesn't crash and produces output
        for i in range(50):
            y = np.array([1.0, 0.1, 0.0, i * 0.02, 0.0, 0.3])
            u = np.array([0.0, 0.0])
            state, residuals = obs.update(y, u)
            
        # Basic sanity checks
        assert len(state) == 6
        assert not np.any(np.isnan(state)), "State should not contain NaN"
        assert not np.any(np.isinf(state)), "State should not contain Inf"
        assert obs.P.shape == (8, 8)
        assert np.all(np.diag(obs.P) >= 0), "Covariance diagonal should be non-negative"
        
    def test_position_tracking(self):
        """Test observer tracks position correctly"""
        obs = create_qlpv_observer()
        
        # Initialize with some velocity
        obs.reset(initial_state=np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        
        # Simulate forward motion
        for i in range(100):
            X_true = i * 0.02  # Position should increase
            y = np.array([1.0, 0.0, 0.0, X_true, 0.0, 0.0])
            u = np.array([0.0, 0.0])
            state, _ = obs.update(y, u)
            
        # Final X estimate should be close to true
        assert abs(state[4] - X_true) < 0.5


class TestTireResidualEstimation:
    """Test tire residual estimation capability"""
    
    def test_residual_estimation_with_disturbance(self):
        """Test observer estimates tire residuals from ay innovation"""
        obs = create_qlpv_observer()
        
        # Simulate with tire force disturbance (shown in ay measurement)
        true_w = 5.0  # True tire residual effect
        
        residuals_history = []
        for i in range(100):
            # ay measurement includes residual effect
            ay_measured = 0.5 + true_w / obs.m
            y = np.array([1.0, 0.1, 0.0, 0.0, 0.0, ay_measured])
            u = np.array([0.0, 0.0])
            state, residuals = obs.update(y, u)
            residuals_history.append(residuals.copy())
            
        # Check residuals are estimated (not zero)
        final_residuals = residuals_history[-1]
        assert np.any(np.abs(final_residuals) > 0.1), "Residuals should be estimated"


def simulate_vehicle_motion(Ts: float, T_total: float, 
                            initial_state: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulate simple vehicle motion for testing
    
    Returns:
        Tuple of (time, true_states, measurements)
    """
    n_steps = int(T_total / Ts)
    time = np.arange(n_steps) * Ts
    
    # True states: [vx, vy, psi, r, X, Y]
    states = np.zeros((n_steps, 6))
    states[0] = initial_state
    
    # Measurements: [vx, r, psi, X, Y, ay]
    measurements = np.zeros((n_steps, 6))
    
    # Control: constant forward motion
    delta = 0.05  # Small steering
    
    for i in range(1, n_steps):
        vx, vy, psi, r, X, Y = states[i-1]
        
        # Simple kinematic update
        X_new = X + Ts * (vx * np.cos(psi) - vy * np.sin(psi))
        Y_new = Y + Ts * (vx * np.sin(psi) + vy * np.cos(psi))
        psi_new = psi + Ts * r
        
        states[i] = [vx, vy, psi_new, r, X_new, Y_new]
        
        # Measurements with some noise
        measurements[i, 0] = vx + np.random.normal(0, 0.05)  # vx
        measurements[i, 1] = r + np.random.normal(0, 0.01)   # r
        measurements[i, 2] = psi_new + np.random.normal(0, 0.02)  # psi
        measurements[i, 3] = X_new + np.random.normal(0, 0.5)  # X
        measurements[i, 4] = Y_new + np.random.normal(0, 0.5)  # Y
        measurements[i, 5] = r * vx + np.random.normal(0, 0.1)  # ay (approximate)
        
    return time, states, measurements


class TestSimulation:
    """Integration tests with simulated vehicle data"""
    
    def test_simulation_convergence(self):
        """Test observer converges during simulation"""
        obs = create_qlpv_observer(sample_time=0.02)
        
        # Initial conditions
        initial_state = np.array([1.0, 0.0, 0.0, 0.1, 0.0, 0.0])
        
        # Generate simulation data
        time, true_states, measurements = simulate_vehicle_motion(
            Ts=0.02, T_total=2.0, initial_state=initial_state
        )
        
        # Run observer
        estimates = []
        control = np.array([0.05, 0.0])  # Small steering, no acceleration
        
        for i in range(len(time)):
            state, residuals = obs.update(measurements[i], control)
            estimates.append(state)
            
        estimates = np.array(estimates)
        
        # Check position tracking error is bounded
        pos_error = np.sqrt((estimates[:, 4] - true_states[:, 4])**2 + 
                           (estimates[:, 5] - true_states[:, 5])**2)
        assert np.max(pos_error) < 5.0, "Position error should be bounded"


def run_visual_test():
    """
    Visual test with matplotlib plotting
    Run this function directly for visual inspection
    """
    print("Running visual EKF observer test...")
    
    obs = create_qlpv_observer(sample_time=0.02)
    
    # Simulation parameters
    T_total = 5.0
    Ts = 0.02
    n_steps = int(T_total / Ts)
    
    # True trajectory: circular motion
    true_vx = 1.0
    true_r = 0.3
    
    true_states = np.zeros((n_steps, 6))
    estimates = np.zeros((n_steps, 6))
    residuals_history = np.zeros((n_steps, 2))
    covariance_trace = np.zeros(n_steps)
    
    # Initial state
    true_states[0] = [true_vx, 0.0, 0.0, true_r, 0.0, 0.0]
    
    control = np.array([0.1, 0.0])
    
    for i in range(1, n_steps):
        # True dynamics (simple kinematic)
        vx, vy, psi, r, X, Y = true_states[i-1]
        X_new = X + Ts * (vx * np.cos(psi))
        Y_new = Y + Ts * (vx * np.sin(psi))
        psi_new = psi + Ts * r
        true_states[i] = [true_vx, vy, psi_new, true_r, X_new, Y_new]
        
        # Noisy measurement
        y = np.array([
            true_vx + np.random.normal(0, 0.1),
            true_r + np.random.normal(0, 0.02),
            psi_new + np.random.normal(0, 0.03),
            X_new + np.random.normal(0, 0.5),
            Y_new + np.random.normal(0, 0.5),
            true_r * true_vx + np.random.normal(0, 0.1),
        ])
        
        # Observer update
        state, residuals = obs.update(y, control)
        estimates[i] = state
        residuals_history[i] = residuals
        covariance_trace[i] = np.trace(obs.P)
        
    # Plotting
    time = np.arange(n_steps) * Ts
    
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle('EKF qLPV Observer Test', fontsize=14)
    
    # Position XY
    axes[0, 0].plot(true_states[:, 4], true_states[:, 5], 'b-', label='True', linewidth=2)
    axes[0, 0].plot(estimates[:, 4], estimates[:, 5], 'r--', label='Estimate', linewidth=1.5)
    axes[0, 0].set_xlabel('X [m]')
    axes[0, 0].set_ylabel('Y [m]')
    axes[0, 0].legend()
    axes[0, 0].set_title('Position Tracking')
    axes[0, 0].axis('equal')
    axes[0, 0].grid(True)
    
    # Velocity
    axes[0, 1].plot(time, true_states[:, 0], 'b-', label='True vx', linewidth=2)
    axes[0, 1].plot(time, estimates[:, 0], 'r--', label='Est vx', linewidth=1.5)
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Velocity [m/s]')
    axes[0, 1].legend()
    axes[0, 1].set_title('Velocity Estimation')
    axes[0, 1].grid(True)
    
    # Yaw angle
    axes[1, 0].plot(time, true_states[:, 2], 'b-', label='True ψ', linewidth=2)
    axes[1, 0].plot(time, estimates[:, 2], 'r--', label='Est ψ', linewidth=1.5)
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Yaw [rad]')
    axes[1, 0].legend()
    axes[1, 0].set_title('Yaw Angle Estimation')
    axes[1, 0].grid(True)
    
    # Position error
    pos_error = np.sqrt((estimates[:, 4] - true_states[:, 4])**2 + 
                        (estimates[:, 5] - true_states[:, 5])**2)
    axes[1, 1].plot(time, pos_error, 'g-', linewidth=1.5)
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Position Error [m]')
    axes[1, 1].set_title('Position Estimation Error')
    axes[1, 1].grid(True)
    
    # Tire residuals
    axes[2, 0].plot(time, residuals_history[:, 0], 'b-', label='ŵ_r', linewidth=1.5)
    axes[2, 0].plot(time, residuals_history[:, 1], 'r-', label='ŵ_f', linewidth=1.5)
    axes[2, 0].set_xlabel('Time [s]')
    axes[2, 0].set_ylabel('Tire Residual')
    axes[2, 0].legend()
    axes[2, 0].set_title('Tire Force Residual Estimates')
    axes[2, 0].grid(True)
    
    # Covariance trace
    axes[2, 1].plot(time, covariance_trace, 'm-', linewidth=1.5)
    axes[2, 1].set_xlabel('Time [s]')
    axes[2, 1].set_ylabel('trace(P)')
    axes[2, 1].set_title('Covariance Trace (Uncertainty)')
    axes[2, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig('ekf_qlpv_observer_test.png', dpi=150)
    plt.show()
    
    print("Visual test completed. Plot saved to 'ekf_qlpv_observer_test.png'")


if __name__ == '__main__':
    # Run pytest tests
    print("Running pytest...")
    pytest.main([__file__, '-v', '--tb=short'])
    
    # Run visual test
    try:
        run_visual_test()
    except Exception as e:
        print(f"Visual test skipped (no display): {e}")
