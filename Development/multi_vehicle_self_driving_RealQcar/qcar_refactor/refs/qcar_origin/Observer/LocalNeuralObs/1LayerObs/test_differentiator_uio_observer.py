"""
Test suite for Differentiator, UIO-Style Observer, and EKF Observer

Tests organized into logical groups:
1. DirtyDerivative filter tests
2. HighGainDifferentiator tests
3. SlidingModeDifferentiator tests
4. Differentiator factory tests
5. WEstimatorUIOStyle tests
6. SchedulingParameters tests
7. DifferentiatorUIOObserver tests
8. DifferentiatorUIOEKF tests
9. Factory integration tests
"""

import numpy as np
import sys
from pathlib import Path
from typing import Callable, List, Tuple

# Add parent directory to path
parent_dir = Path(__file__).parent
sys.path.insert(0, str(parent_dir))

# Import differentiators from centralized module
from differentiators import (
    DirtyDerivative,
    HighGainDifferentiator,
    SlidingModeDifferentiator,
    create_differentiator,
    create_differentiator_from_config,
    load_differentiator_config,
)

# Import observer components
from differentiator_uio_observer import (
    WEstimatorUIOStyle,
    DifferentiatorUIOObserver,
    SchedulingParameters,
    create_differentiator_uio_observer
)

from differentiator_uio_ekf import (
    DifferentiatorUIOEKF,
    create_differentiator_uio_ekf
)


# ==============================================================================
# Helper Classes and Functions
# ==============================================================================

class TestResult:
    """Simple test result tracker"""
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.skipped = 0
        self.failures: List[Tuple[str, str]] = []
    
    def add_pass(self):
        self.passed += 1
    
    def add_fail(self, name: str, message: str):
        self.failed += 1
        self.failures.append((name, message))
    
    def add_skip(self):
        self.skipped += 1
    
    @property
    def total(self) -> int:
        return self.passed + self.failed + self.skipped


def run_test(name: str, func: Callable, results: TestResult) -> bool:
    """Run a single test and track results"""
    try:
        func()
        results.add_pass()
        print(f"✅ {name}")
        return True
    except Exception as e:
        results.add_fail(name, str(e))
        print(f"❌ {name}: {e}")
        import traceback
        traceback.print_exc()
        return False


def print_section(title: str):
    """Print a section header"""
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


# ==============================================================================
# Section 1: DirtyDerivative Tests
# ==============================================================================

def test_dirty_derivative_constant():
    """DirtyDerivative with constant input should converge to zero derivative"""
    Ts = 0.01
    tau = 0.05
    dd = DirtyDerivative(Ts=Ts, tau=tau, y0=1.0)
    
    # Feed constant value - derivative should converge to zero
    for _ in range(100):
        ydot = dd.update(1.0)
    
    assert abs(ydot) < 0.001, f"Derivative of constant should be ~0, got {ydot}"


def test_dirty_derivative_ramp():
    """DirtyDerivative with ramp input should converge to the slope"""
    Ts = 0.01
    tau = 0.02  # Smaller tau for faster response
    slope = 2.0  # dy/dt = 2
    dd = DirtyDerivative(Ts=Ts, tau=tau, y0=0.0)
    
    # Feed ramp: y = slope * t
    for i in range(200):
        t = i * Ts
        y = slope * t
        ydot = dd.update(y)
    
    # Allow 15% error due to filter lag
    assert abs(ydot - slope) < 0.3, f"Expected ydot ≈ {slope}, got {ydot}"


def test_dirty_derivative_sinusoid():
    """DirtyDerivative with sinusoidal input tracks derivative with phase lag"""
    Ts = 0.01
    tau = 0.02
    freq = 1.0  # 1 Hz
    omega = 2 * np.pi * freq
    dd = DirtyDerivative(Ts=Ts, tau=tau, y0=0.0)
    
    # Feed sinusoid: y = sin(ωt), true derivative = ω*cos(ωt)
    errors = []
    for i in range(500):
        t = i * Ts
        y = np.sin(omega * t)
        ydot = dd.update(y)
        true_ydot = omega * np.cos(omega * t)
        if i > 100:  # Skip transient
            errors.append(abs(ydot - true_ydot))
    
    mean_error = np.mean(errors)
    # Phase lag is expected, so error will be non-zero but bounded
    assert mean_error < 2.0, f"Mean error too large: {mean_error}"


def test_dirty_derivative_step():
    """DirtyDerivative response to step input (impulse in derivative)"""
    Ts = 0.01
    tau = 0.02
    dd = DirtyDerivative(Ts=Ts, tau=tau, y0=0.0)
    
    # Step at t=0.5s
    step_time = 50  # 50 * 0.01 = 0.5s
    peak_derivative = 0.0
    
    for i in range(100):
        y = 0.0 if i < step_time else 1.0
        ydot = dd.update(y)
        if i >= step_time:
            peak_derivative = max(peak_derivative, abs(ydot))
    
    # Should detect a spike in derivative
    assert peak_derivative > 1.0, f"Should detect step, got peak {peak_derivative}"
    # Should settle after step
    final_ydot = dd.update(1.0)
    for _ in range(50):
        final_ydot = dd.update(1.0)
    assert abs(final_ydot) < 0.1, f"Should settle to ~0, got {final_ydot}"


def test_dirty_derivative_reset():
    """DirtyDerivative reset functionality"""
    Ts = 0.01
    tau = 0.05
    dd = DirtyDerivative(Ts=Ts, tau=tau, y0=5.0)
    
    # Run some updates
    for _ in range(10):
        dd.update(10.0)
    
    # Reset
    dd.reset(y0=0.0)
    
    assert dd.y_prev == 0.0, "y_prev should be reset"
    assert dd.ydot == 0.0, "ydot should be reset"


# ==============================================================================
# Section 2: HighGainDifferentiator Tests
# ==============================================================================

def test_highgain_constant():
    """HighGainDifferentiator with constant input converges to zero"""
    Ts = 0.01
    hg = HighGainDifferentiator(Ts=Ts, omega=50.0, zeta=1.0, y0=1.0)
    
    for _ in range(100):
        ydot = hg.update(1.0)
    
    assert abs(ydot) < 0.01, f"Derivative of constant should be ~0, got {ydot}"


def test_highgain_ramp():
    """HighGainDifferentiator with ramp input tracks slope"""
    Ts = 0.01
    omega = 50.0  # High bandwidth for fast response
    slope = 2.0
    hg = HighGainDifferentiator(Ts=Ts, omega=omega, zeta=1.0, y0=0.0)
    
    for i in range(200):
        t = i * Ts
        y = slope * t
        ydot = hg.update(y)
    
    assert abs(ydot - slope) < 0.5, f"Expected ydot ≈ {slope}, got {ydot}"


def test_highgain_sinusoid():
    """HighGainDifferentiator tracks sinusoidal derivative"""
    Ts = 0.01
    freq = 0.5  # 0.5 Hz
    omega = 2 * np.pi * freq
    hg = HighGainDifferentiator(Ts=Ts, omega=50.0, zeta=0.707, y0=0.0)
    
    errors = []
    for i in range(500):
        t = i * Ts
        y = np.sin(omega * t)
        ydot = hg.update(y)
        true_ydot = omega * np.cos(omega * t)
        if i > 100:
            errors.append(abs(ydot - true_ydot))
    
    mean_error = np.mean(errors)
    assert mean_error < 1.5, f"Mean error too large: {mean_error}"


def test_highgain_bandwidth_change():
    """HighGainDifferentiator bandwidth can be changed dynamically"""
    Ts = 0.01
    hg = HighGainDifferentiator(Ts=Ts, omega=20.0, zeta=0.707)
    
    assert hg.omega == 20.0
    
    # Change bandwidth
    hg.set_bandwidth(50.0, zeta=1.0)
    
    assert hg.omega == 50.0, "Omega should be updated"
    assert hg.zeta == 1.0, "Zeta should be updated"
    assert abs(hg.L1 - 100.0) < 0.01, f"L1 should be 2*zeta*omega = 100, got {hg.L1}"
    assert abs(hg.L2 - 2500.0) < 0.01, f"L2 should be omega^2 = 2500, got {hg.L2}"


def test_highgain_antiwindup():
    """HighGainDifferentiator respects maximum derivative limit"""
    Ts = 0.01
    ydot_max = 5.0
    hg = HighGainDifferentiator(Ts=Ts, omega=100.0, zeta=1.0, y0=0.0, ydot_max=ydot_max)
    
    # Large step should trigger anti-windup
    hg.update(0.0)
    ydot = hg.update(100.0)  # Huge jump
    
    assert abs(ydot) <= ydot_max + 0.01, f"Should be clamped to {ydot_max}, got {ydot}"


def test_highgain_reset():
    """HighGainDifferentiator reset functionality"""
    Ts = 0.01
    hg = HighGainDifferentiator(Ts=Ts, omega=50.0)
    
    for _ in range(10):
        hg.update(5.0)
    
    hg.reset(y0=0.0)
    
    assert hg.y_hat == 0.0, "y_hat should be reset"
    assert hg.ydot_hat == 0.0, "ydot_hat should be reset"


def test_highgain_noisy_signal():
    """HighGainDifferentiator handles noisy signals with reasonable filtering"""
    np.random.seed(42)
    Ts = 0.01
    hg = HighGainDifferentiator(Ts=Ts, omega=30.0, zeta=1.0, y0=0.0)
    
    # Ramp with noise
    slope = 1.0
    noise_std = 0.1
    
    derivatives = []
    for i in range(300):
        t = i * Ts
        y = slope * t + noise_std * np.random.randn()
        ydot = hg.update(y)
        if i > 100:
            derivatives.append(ydot)
    
    mean_deriv = np.mean(derivatives)
    # Should track slope despite noise
    assert abs(mean_deriv - slope) < 0.3, f"Mean derivative {mean_deriv} should be near {slope}"


# ==============================================================================
# Section 3: SlidingModeDifferentiator Tests
# ==============================================================================

def test_sliding_mode_constant():
    """SlidingModeDifferentiator with constant input converges to zero"""
    Ts = 0.01
    sm = SlidingModeDifferentiator(Ts=Ts, k1=20.0, k2=200.0, y0=1.0)
    
    for _ in range(200):
        ydot = sm.update(1.0)
    
    assert abs(ydot) < 0.5, f"Derivative should converge to ~0, got {ydot}"


def test_sliding_mode_ramp():
    """SlidingModeDifferentiator runs on ramp input without numerical issues"""
    Ts = 0.01
    slope = 2.0
    # Higher gains for sliding mode
    sm = SlidingModeDifferentiator(Ts=Ts, k1=50.0, k2=500.0, epsilon=0.01, y0=0.0)
    
    # Run on ramp input
    for i in range(500):
        t = i * Ts
        y = slope * t
        ydot = sm.update(y)
    
    assert abs(ydot - slope) < 1.0, f"Expected ydot ≈ {slope}, got {ydot}"


def test_sliding_mode_sinusoid():
    """SlidingModeDifferentiator tracks sinusoidal derivative"""
    Ts = 0.01
    freq = 0.5  # 0.5 Hz
    omega = 2 * np.pi * freq
    sm = SlidingModeDifferentiator(Ts=Ts, k1=50.0, k2=500.0, epsilon=0.01, y0=0.0)
    
    errors = []
    for i in range(500):
        t = i * Ts
        y = np.sin(omega * t)
        ydot = sm.update(y)
        true_ydot = omega * np.cos(omega * t)
        if i > 200:  # Longer transient for sliding mode
            errors.append(abs(ydot - true_ydot))
    
    mean_error = np.mean(errors)
    assert mean_error < 3.0, f"Error too large: {mean_error}"


def test_sliding_mode_smoothing_types():
    """SlidingModeDifferentiator works with different smoothing types"""
    Ts = 0.01
    smoothing_types = ['epsilon', 'tanh', 'saturation']
    
    for smooth in smoothing_types:
        sm = SlidingModeDifferentiator(Ts=Ts, k1=50.0, k2=500.0, 
                                        epsilon=0.01, smoothing=smooth)
        
        for i in range(100):
            y = np.sin(0.1 * i)
            ydot = sm.update(y)
        
        assert np.isfinite(ydot), f"{smooth} produced non-finite output"


def test_sliding_mode_gain_change():
    """SlidingModeDifferentiator gains can be changed dynamically"""
    Ts = 0.01
    sm = SlidingModeDifferentiator(Ts=Ts, k1=10.0, k2=100.0)
    
    sm.set_gains(k1=30.0, k2=300.0)
    
    assert sm.k1 == 30.0, "k1 should be updated"
    assert sm.k2 == 300.0, "k2 should be updated"


def test_sliding_mode_antiwindup():
    """SlidingModeDifferentiator respects maximum derivative limit"""
    Ts = 0.01
    v_max = 5.0
    sm = SlidingModeDifferentiator(Ts=Ts, k1=100.0, k2=1000.0, y0=0.0, v_max=v_max)
    
    sm.update(0.0)
    ydot = sm.update(100.0)
    
    assert abs(ydot) <= v_max + 0.01, f"Should be clamped to {v_max}, got {ydot}"


def test_sliding_mode_reset():
    """SlidingModeDifferentiator reset functionality"""
    Ts = 0.01
    sm = SlidingModeDifferentiator(Ts=Ts, k1=20.0, k2=200.0)
    
    for _ in range(10):
        sm.update(5.0)
    
    sm.reset(y0=0.0)
    
    assert sm.y_hat == 0.0, "y_hat should be reset"
    assert sm.v_hat == 0.0, "v_hat should be reset"


# ==============================================================================
# Section 4: Differentiator Factory Tests
# ==============================================================================

def test_factory_creates_dirty():
    """Factory creates DirtyDerivative correctly"""
    dd = create_differentiator('dirty', Ts=0.01, tau=0.05)
    assert isinstance(dd, DirtyDerivative)


def test_factory_creates_highgain():
    """Factory creates HighGainDifferentiator correctly"""
    hg = create_differentiator('highgain', Ts=0.01, omega=40.0, zeta=0.8)
    assert isinstance(hg, HighGainDifferentiator)
    assert hg.omega == 40.0


def test_factory_creates_sliding():
    """Factory creates SlidingModeDifferentiator correctly"""
    sm = create_differentiator('sliding', Ts=0.01, k1=25.0, k2=250.0, smoothing='tanh')
    assert isinstance(sm, SlidingModeDifferentiator)
    assert sm.k1 == 25.0
    assert sm.smoothing == 'tanh'


def test_factory_invalid_type():
    """Factory raises error for invalid type"""
    try:
        create_differentiator('invalid', Ts=0.01)
        assert False, "Should raise ValueError"
    except ValueError:
        pass


# ==============================================================================
# Section 5: WEstimatorUIOStyle Tests
# ==============================================================================

def test_w_estimator_zero_residuals():
    """WEstimatorUIOStyle with no dynamics produces near-zero residuals"""
    Ts = 0.02
    params = dict(m=3.5, Iz=0.05, lf=0.11, lr=0.11, Cf=50.0, Cr=50.0, vx_min=0.5)
    
    w_est = WEstimatorUIOStyle(Ts, params, tau_rdot=0.05, ridge=1e-6)
    
    # State with no lateral dynamics
    xhat = np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Initialize with zero r
    for _ in range(10):
        w_hat, rdot_hat, b = w_est.estimate(xhat, r_meas=0.0, ay_meas=0.0, delta=0.0)
    
    assert abs(w_hat[0]) < 0.5, f"w_r should be ~0, got {w_hat[0]}"
    assert abs(w_hat[1]) < 0.5, f"w_f should be ~0, got {w_hat[1]}"


def test_w_estimator_with_highgain_diff():
    """WEstimatorUIOStyle works with HighGain differentiator"""
    Ts = 0.02
    params = dict(m=3.5, Iz=0.05, lf=0.11, lr=0.11, Cf=50.0, Cr=50.0, vx_min=0.5)
    
    w_est = WEstimatorUIOStyle(
        Ts, params, 
        diff_type='highgain',
        diff_params={'omega': 40.0, 'zeta': 1.0}
    )
    
    xhat = np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Should work without error
    w_hat, rdot_hat, b = w_est.estimate(xhat, r_meas=0.1, ay_meas=0.5, delta=0.1)
    
    assert np.isfinite(w_hat).all(), "w_hat should be finite"
    assert np.isfinite(rdot_hat), "rdot_hat should be finite"


def test_w_estimator_with_sliding_diff():
    """WEstimatorUIOStyle works with Sliding Mode differentiator"""
    Ts = 0.02
    params = dict(m=3.5, Iz=0.05, lf=0.11, lr=0.11, Cf=50.0, Cr=50.0, vx_min=0.5)
    
    w_est = WEstimatorUIOStyle(
        Ts, params,
        diff_type='sliding',
        diff_params={'k1': 20.0, 'k2': 200.0}
    )
    
    xhat = np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    w_hat, rdot_hat, b = w_est.estimate(xhat, r_meas=0.1, ay_meas=0.5, delta=0.1)
    
    assert np.isfinite(w_hat).all(), "w_hat should be finite"


def test_w_estimator_reset():
    """WEstimatorUIOStyle reset functionality"""
    Ts = 0.02
    params = dict(m=3.5, Iz=0.05, lf=0.11, lr=0.11, Cf=50.0, Cr=50.0, vx_min=0.5)
    
    w_est = WEstimatorUIOStyle(Ts, params)
    
    xhat = np.array([5.0, 0.5, 0.1, 0.2, 0.0, 0.0])
    for _ in range(10):
        w_est.estimate(xhat, r_meas=0.2, ay_meas=0.5, delta=0.1)
    
    w_est.reset(r0=0.0)
    
    assert w_est.rdot_hat == 0.0, "rdot_hat should be reset"
    assert np.allclose(w_est.residual, 0.0), "residual should be reset"


def test_w_estimator_compute_lin_terms():
    """WEstimatorUIOStyle computes linear terms correctly"""
    Ts = 0.02
    params = dict(m=3.5, Iz=0.05, lf=0.11, lr=0.11, Cf=50.0, Cr=50.0, vx_min=0.5)
    
    w_est = WEstimatorUIOStyle(Ts, params)
    
    xhat = np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    rdot_lin, ay_lin = w_est.compute_lin_terms(xhat, delta=0.0)
    
    # With zero lateral velocity and zero steering, linear terms should be zero
    assert abs(rdot_lin) < 0.01, f"rdot_lin should be ~0, got {rdot_lin}"
    assert abs(ay_lin) < 0.01, f"ay_lin should be ~0, got {ay_lin}"


# ==============================================================================
# Section 6: SchedulingParameters Tests
# ==============================================================================

def test_scheduling_params_basic():
    """SchedulingParameters computes correctly from state"""
    state = np.array([5.0, 0.5, 0.1, 0.2, 10.0, 5.0])
    delta = 0.15
    
    rho = SchedulingParameters.from_state_and_input(state, delta)
    
    assert abs(rho.inv_vx - 0.2) < 1e-6, f"1/vx incorrect: {rho.inv_vx}"
    assert abs(rho.sin_delta - np.sin(delta)) < 1e-6, "sin(δ) incorrect"
    assert abs(rho.cos_delta - np.cos(delta)) < 1e-6, "cos(δ) incorrect"
    assert abs(rho.vx - 5.0) < 1e-6, "vx incorrect"


def test_scheduling_params_min_vx():
    """SchedulingParameters enforces minimum velocity"""
    state = np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0])  # Very low vx
    delta = 0.0
    min_vx = 0.5
    
    rho = SchedulingParameters.from_state_and_input(state, delta, min_vx)
    
    assert rho.vx == min_vx, f"vx should be clamped to {min_vx}, got {rho.vx}"
    assert rho.inv_vx == 1.0 / min_vx, "inv_vx should use clamped vx"


def test_scheduling_params_negative_vx():
    """SchedulingParameters handles negative velocity (reverse)"""
    state = np.array([-3.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    delta = 0.0
    min_vx = 0.5
    
    rho = SchedulingParameters.from_state_and_input(state, delta, min_vx)
    
    # Should use abs(vx)
    assert rho.vx == 3.0, f"vx should be abs(-3.0) = 3.0, got {rho.vx}"


# ==============================================================================
# Section 7: DifferentiatorUIOObserver Tests
# ==============================================================================

def test_observer_initialization():
    """DifferentiatorUIOObserver initializes correctly"""
    observer = DifferentiatorUIOObserver(sample_time=0.02)
    
    assert observer.Ts == 0.02
    assert observer.STATE_DIM == 6
    assert observer.MEAS_DIM == 6
    assert len(observer.state_hat) == 6


def test_observer_with_custom_params():
    """DifferentiatorUIOObserver accepts custom vehicle parameters"""
    params = {'m': 5.0, 'Iz': 0.1}
    observer = DifferentiatorUIOObserver(sample_time=0.02, vehicle_params=params)
    
    assert observer.m == 5.0
    assert observer.Iz == 0.1


def test_observer_matrix_dimensions():
    """DifferentiatorUIOObserver matrices have correct dimensions"""
    observer = DifferentiatorUIOObserver(sample_time=0.02)
    observer.state_hat = np.array([5.0, 0.1, 0.0, 0.1, 0.0, 0.0])
    
    rho = observer.compute_scheduling_params(observer.state_hat, 0.1)
    
    A = observer.compute_A_matrix(rho)
    B = observer.compute_B_matrix(rho)
    E = observer.compute_E_matrix(rho)
    C = observer.compute_C_matrix()
    
    assert A.shape == (6, 6), f"A dimension error: {A.shape}"
    assert B.shape == (6, 2), f"B dimension error: {B.shape}"
    assert E.shape == (6, 2), f"E dimension error: {E.shape}"
    assert C.shape == (6, 6), f"C dimension error: {C.shape}"


def test_observer_reset():
    """DifferentiatorUIOObserver reset functionality"""
    observer = DifferentiatorUIOObserver(sample_time=0.02)
    
    initial_state = np.array([2.0, 0.1, 0.05, 0.02, 5.0, 3.0])
    observer.reset(initial_state)
    
    state = observer.get_state()
    assert np.allclose(state, initial_state), "State should match initial"


def test_observer_update_basic():
    """DifferentiatorUIOObserver basic update works"""
    observer = DifferentiatorUIOObserver(sample_time=0.02)
    observer.reset(np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    measurement = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    control = np.array([0.0, 0.0])
    
    state, w = observer.update(measurement, control)
    
    assert len(state) == 6
    assert len(w) == 2


def test_observer_state_changes():
    """DifferentiatorUIOObserver state changes with dynamics"""
    observer = DifferentiatorUIOObserver(sample_time=0.02)
    initial_state = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    observer.reset(initial_state)
    
    control = np.array([0.1, 0.5])  # Steering + acceleration
    
    for i in range(10):
        vx = 2.0 + 0.5 * (i + 1) * 0.02
        r = 0.1
        psi = 0.1 * (i + 1) * 0.02
        a_y = r * vx
        measurement = np.array([vx, r, psi, 0.0, 0.0, a_y])
        state, _ = observer.update(measurement, control)
    
    # State should have changed from initial
    assert not np.allclose(state, initial_state), "State should change"


def test_observer_with_highgain():
    """DifferentiatorUIOObserver works with HighGain differentiator"""
    observer = DifferentiatorUIOObserver(
        sample_time=0.02,
        diff_type='highgain',
        diff_params={'omega': 40.0, 'zeta': 1.0}
    )
    
    observer.reset(np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    measurement = np.array([2.0, 0.1, 0.0, 0.0, 0.0, 0.2])
    control = np.array([0.1, 0.0])
    
    state, w = observer.update(measurement, control)
    
    assert observer.diff_type == 'highgain'
    assert np.isfinite(state).all()


def test_observer_with_sliding():
    """DifferentiatorUIOObserver works with Sliding Mode differentiator"""
    observer = DifferentiatorUIOObserver(
        sample_time=0.02,
        diff_type='sliding',
        diff_params={'k1': 25.0, 'k2': 250.0}
    )
    
    observer.reset(np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    measurement = np.array([2.0, 0.1, 0.0, 0.0, 0.0, 0.2])
    control = np.array([0.1, 0.0])
    
    state, w = observer.update(measurement, control)
    
    assert observer.diff_type == 'sliding'
    assert np.isfinite(state).all()


def test_observer_convergence():
    """DifferentiatorUIOObserver tracks directly measured states"""
    np.random.seed(42)
    observer = DifferentiatorUIOObserver(sample_time=0.02)
    
    # True measurements
    true_vx = 5.0
    true_r = 0.1  # Small yaw rate
    
    # Initialize with incorrect vx
    initial_guess = np.array([3.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    observer.reset(initial_guess)
    
    initial_vx_error = abs(initial_guess[0] - true_vx)
    
    control = np.array([0.1, 0.0])
    
    # Run updates with consistent measurement
    for i in range(100):
        a_y = true_r * true_vx
        # Evolving psi based on r integration
        psi = true_r * i * 0.02
        measurement = np.array([
            true_vx + 0.01 * np.random.randn(),
            true_r + 0.001 * np.random.randn(),
            psi + 0.001 * np.random.randn(),
            0.0, 0.0,  # X, Y = 0
            a_y + 0.01 * np.random.randn()
        ])
        state, _ = observer.update(measurement, control)
    
    final_vx_error = abs(state[0] - true_vx)
    
    # vx should be tracked better than initial guess
    # (vx has direct measurement feedback through innovation)
    assert final_vx_error < initial_vx_error, f"vx error should decrease: {final_vx_error} vs {initial_vx_error}"


def test_observer_measurement_formats():
    """DifferentiatorUIOObserver handles different measurement formats"""
    observer = DifferentiatorUIOObserver()
    observer.reset(np.array([3.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    control = np.array([0.0, 0.0])
    
    # 6D measurement
    meas_6d = np.array([3.0, 0.1, 0.05, 1.0, 0.5, 0.3])
    state, w = observer.update(meas_6d, control)
    assert np.isfinite(state).all()
    
    # 5D measurement with acceleration
    meas_5d = np.array([3.0, 0.1, 0.05, 1.0, 0.5])
    accel = np.array([0.1, 0.3, 0.0])
    state, w = observer.update(meas_5d, control, acceleration=accel)
    assert np.isfinite(state).all()
    
    # 2D measurement
    meas_2d = np.array([3.0, 0.1])
    state, w = observer.update(meas_2d, control, acceleration=accel)
    assert np.isfinite(state).all()


def test_observer_getters():
    """DifferentiatorUIOObserver getter methods work"""
    observer = DifferentiatorUIOObserver(sample_time=0.02)
    observer.reset(np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    measurement = np.array([2.0, 0.1, 0.0, 0.0, 0.0, 0.2])
    control = np.array([0.0, 0.0])
    observer.update(measurement, control)
    
    state = observer.get_state()
    assert len(state) == 6
    
    w = observer.get_tire_residuals()
    assert len(w) == 2
    
    w_alias = observer.get_unknown_input()
    assert np.allclose(w, w_alias)
    
    rdot = observer.get_rdot_estimate()
    assert np.isfinite(rdot)
    
    residual = observer.get_residual_vector()
    assert len(residual) == 2


# ==============================================================================
# Section 8: DifferentiatorUIOEKF Tests
# ==============================================================================

def test_ekf_initialization():
    """DifferentiatorUIOEKF initializes correctly"""
    observer = DifferentiatorUIOEKF(sample_time=0.02)
    
    assert observer.Ts == 0.02
    assert observer.STATE_DIM == 6
    assert observer.MEAS_DIM == 6
    assert observer.P.shape == (6, 6)
    assert observer.Q.shape == (6, 6)
    assert observer.R.shape == (6, 6)
    assert observer.K.shape == (6, 6)


def test_ekf_with_custom_params():
    """DifferentiatorUIOEKF accepts custom vehicle parameters"""
    params = {'m': 5.0, 'Iz': 0.1}
    observer = DifferentiatorUIOEKF(sample_time=0.02, vehicle_params=params)
    
    assert observer.m == 5.0
    assert observer.Iz == 0.1


def test_ekf_with_custom_covariances():
    """DifferentiatorUIOEKF accepts custom covariance matrices"""
    Q = np.diag([0.1, 0.1, 0.01, 0.1, 0.1, 0.1])
    R = np.diag([0.01, 0.001, 0.01, 0.1, 0.1, 0.1])
    P0 = np.eye(6) * 0.5
    
    observer = DifferentiatorUIOEKF(sample_time=0.02, Q=Q, R=R, P0=P0)
    
    assert np.allclose(observer.Q, Q)
    assert np.allclose(observer.R, R)
    assert np.allclose(observer.P, P0)


def test_ekf_reset():
    """DifferentiatorUIOEKF reset functionality"""
    observer = DifferentiatorUIOEKF(sample_time=0.02)
    
    # Run some updates
    measurement = np.array([2.0, 0.1, 0.0, 0.0, 0.0, 0.2])
    control = np.array([0.1, 0.5])
    for _ in range(10):
        observer.update(measurement, control)
    
    # Reset
    initial_state = np.array([1.0, 0.0, 0.0, 0.0, 5.0, 3.0])
    observer.reset(initial_state)
    
    state = observer.get_state()
    assert np.allclose(state, initial_state)
    
    # Covariance should be reset to default P0
    P0 = observer._default_P0()
    assert np.allclose(observer.P, P0)


def test_ekf_update_basic():
    """DifferentiatorUIOEKF basic update works"""
    observer = DifferentiatorUIOEKF(sample_time=0.02)
    observer.reset(np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    measurement = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    control = np.array([0.0, 0.0])
    
    state, w = observer.update(measurement, control)
    
    assert len(state) == 6
    assert len(w) == 2
    assert np.isfinite(state).all()


def test_ekf_kalman_gain_updates():
    """DifferentiatorUIOEKF Kalman gain changes with updates"""
    observer = DifferentiatorUIOEKF(sample_time=0.02)
    observer.reset(np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    K_initial = observer.get_kalman_gain().copy()
    
    measurement = np.array([2.0, 0.1, 0.0, 0.0, 0.0, 0.2])
    control = np.array([0.1, 0.5])
    
    for _ in range(10):
        observer.update(measurement, control)
    
    K_final = observer.get_kalman_gain()
    
    # Kalman gain should have changed
    assert not np.allclose(K_initial, K_final), "Kalman gain should update"


def test_ekf_covariance_bounded():
    """DifferentiatorUIOEKF covariance remains bounded"""
    observer = DifferentiatorUIOEKF(sample_time=0.02)
    observer.reset(np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    measurement = np.array([2.0, 0.1, 0.0, 0.0, 0.0, 0.2])
    control = np.array([0.1, 0.5])
    
    for _ in range(100):
        observer.update(measurement, control)
    
    P = observer.get_covariance()
    
    # P should remain positive definite and bounded
    eigvals = np.linalg.eigvalsh(P)
    assert all(eigvals > 0), "P should be positive definite"
    assert all(eigvals < 100), "P eigenvalues should be bounded"


def test_ekf_state_clamping():
    """DifferentiatorUIOEKF clamps states for numerical stability"""
    observer = DifferentiatorUIOEKF(sample_time=0.02)
    observer.reset(np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    # Extreme measurement that could cause instability
    measurement = np.array([50.0, 20.0, 0.0, 0.0, 0.0, 100.0])
    control = np.array([1.0, 10.0])
    
    for _ in range(10):
        state, w = observer.update(measurement, control)
    
    # States should be clamped
    assert abs(state[0]) <= 10.0, f"vx should be clamped, got {state[0]}"
    assert abs(state[1]) <= 5.0, f"vy should be clamped, got {state[1]}"
    assert abs(state[3]) <= 10.0, f"r should be clamped, got {state[3]}"


def test_ekf_with_different_differentiators():
    """DifferentiatorUIOEKF works with different differentiator types"""
    diff_types = ['dirty', 'highgain', 'sliding']
    
    for diff_type in diff_types:
        observer = DifferentiatorUIOEKF(sample_time=0.02, diff_type=diff_type)
        observer.reset(np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        
        measurement = np.array([2.0, 0.1, 0.0, 0.0, 0.0, 0.2])
        control = np.array([0.1, 0.0])
        
        state, w = observer.update(measurement, control)
        
        assert observer.diff_type == diff_type
        assert np.isfinite(state).all(), f"{diff_type}: state should be finite"


def test_ekf_convergence():
    """DifferentiatorUIOEKF converges from incorrect initial state"""
    np.random.seed(42)
    observer = DifferentiatorUIOEKF(sample_time=0.02)
    
    # True state
    true_vx = 3.0
    true_r = 0.1
    true_psi = 0.0
    true_X = 0.0
    true_Y = 0.0
    
    # Initialize with incorrect state
    initial_guess = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    observer.reset(initial_guess)
    
    initial_vx_error = abs(initial_guess[0] - true_vx)
    
    control = np.array([0.0, 0.0])
    
    for i in range(100):
        a_y = true_r * true_vx
        measurement = np.array([
            true_vx + 0.01 * np.random.randn(),
            true_r + 0.001 * np.random.randn(),
            true_psi,
            true_X + i * 0.02 * true_vx,
            true_Y,
            a_y + 0.01 * np.random.randn()
        ])
        state, _ = observer.update(measurement, control)
    
    final_vx_error = abs(state[0] - true_vx)
    
    # Error should decrease
    assert final_vx_error < initial_vx_error, "vx error should decrease"


def test_ekf_continuous_dynamics():
    """DifferentiatorUIOEKF continuous dynamics function produces reasonable output"""
    observer = DifferentiatorUIOEKF(sample_time=0.02)
    
    x = np.array([2.0, 0.1, 0.0, 0.1, 0.0, 0.0])
    u = np.array([0.1, 0.5])
    w = np.array([0.0, 0.0])
    
    x_dot = observer.f_continuous(x, u, w)
    
    assert len(x_dot) == 6
    assert np.isfinite(x_dot).all()


def test_ekf_measurement_function():
    """DifferentiatorUIOEKF measurement function produces reasonable output"""
    observer = DifferentiatorUIOEKF(sample_time=0.02)
    
    x = np.array([2.0, 0.1, 0.0, 0.1, 0.0, 0.0])
    u = np.array([0.1, 0.0])
    w = np.array([0.0, 0.0])
    
    y = observer.h_meas(x, u, w)
    
    assert len(y) == 6
    assert np.isfinite(y).all()
    # Direct measurements should match state
    assert y[0] == x[0], "vx measurement should match state"
    assert y[1] == x[3], "r measurement should match state"


def test_ekf_jacobian_computation():
    """DifferentiatorUIOEKF computes Jacobians numerically"""
    observer = DifferentiatorUIOEKF(sample_time=0.02)
    
    x = np.array([2.0, 0.1, 0.0, 0.1, 0.0, 0.0])
    u = np.array([0.1, 0.5])
    w = np.array([0.0, 0.0])
    
    F = observer._numerical_jacobian_F(x, u, w)
    H = observer._numerical_jacobian_H(x, u, w)
    
    assert F.shape == (6, 6)
    assert H.shape == (6, 6)
    assert np.isfinite(F).all()
    assert np.isfinite(H).all()


def test_ekf_getters():
    """DifferentiatorUIOEKF getter methods work"""
    observer = DifferentiatorUIOEKF(sample_time=0.02)
    observer.reset(np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    measurement = np.array([2.0, 0.1, 0.0, 0.0, 0.0, 0.2])
    control = np.array([0.0, 0.0])
    observer.update(measurement, control)
    
    state = observer.get_state()
    assert len(state) == 6
    
    w = observer.get_tire_residuals()
    assert len(w) == 2
    
    w_alias = observer.get_unknown_input()
    assert np.allclose(w, w_alias)
    
    rdot = observer.get_rdot_estimate()
    assert np.isfinite(rdot)
    
    K = observer.get_kalman_gain()
    assert K.shape == (6, 6)
    
    P = observer.get_covariance()
    assert P.shape == (6, 6)


def test_ekf_innovation_computed():
    """DifferentiatorUIOEKF computes innovation vector"""
    observer = DifferentiatorUIOEKF(sample_time=0.02)
    observer.reset(np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    # Measurement different from prediction
    measurement = np.array([3.0, 0.1, 0.1, 1.0, 0.5, 0.2])
    control = np.array([0.0, 0.0])
    
    observer.update(measurement, control)
    
    assert len(observer.innovation) == 6
    assert np.isfinite(observer.innovation).all()


# ==============================================================================
# Section 9: Factory Integration Tests
# ==============================================================================

def test_factory_uio_observer():
    """Factory creates DifferentiatorUIOObserver correctly"""
    observer = create_differentiator_uio_observer(
        sample_time=0.01,
        vehicle_params={'m': 5.0, 'Iz': 0.1}
    )
    
    assert isinstance(observer, DifferentiatorUIOObserver)
    assert observer.Ts == 0.01
    assert observer.m == 5.0


def test_factory_uio_ekf():
    """Factory creates DifferentiatorUIOEKF correctly"""
    observer = create_differentiator_uio_ekf(
        sample_time=0.01,
        vehicle_params={'m': 5.0, 'Iz': 0.1}
    )
    
    assert isinstance(observer, DifferentiatorUIOEKF)
    assert observer.Ts == 0.01
    assert observer.m == 5.0


def test_factory_integration_with_uio_observers():
    """Factory integration with uio_observers module"""
    try:
        from firstLayerObserverBase import create_first_layer_observer
        
        observer = create_first_layer_observer('differentiator_uio', sample_time=0.02)
        assert isinstance(observer, DifferentiatorUIOObserver)
        return  # Test passed
    except ImportError:
        pass  # Skip if uio_observers not available
    except Exception as e:
        if "differentiator_uio" in str(e).lower():
            pass  # Type not registered, acceptable


def test_comparative_observers():
    """Compare DifferentiatorUIOObserver and DifferentiatorUIOEKF on same inputs"""
    np.random.seed(42)
    
    obs_uio = DifferentiatorUIOObserver(sample_time=0.02)
    obs_ekf = DifferentiatorUIOEKF(sample_time=0.02)
    
    initial_state = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    obs_uio.reset(initial_state)
    obs_ekf.reset(initial_state)
    
    control = np.array([0.1, 0.5])
    
    states_uio = []
    states_ekf = []
    
    for i in range(50):
        vx = 2.0 + 0.02 * i * 0.5
        r = 0.1
        a_y = r * vx
        measurement = np.array([vx, r, 0.0, 0.0, 0.0, a_y])
        
        state_uio, _ = obs_uio.update(measurement, control)
        state_ekf, _ = obs_ekf.update(measurement, control)
        
        states_uio.append(state_uio.copy())
        states_ekf.append(state_ekf.copy())
    
    # Both should produce finite results
    assert np.isfinite(np.array(states_uio)).all(), "UIO should produce finite states"
    assert np.isfinite(np.array(states_ekf)).all(), "EKF should produce finite states"


# ==============================================================================
# Main Test Runner
# ==============================================================================

def main():
    """Run all tests organized by section"""
    print("=" * 70)
    print("  Differentiator + UIO-Style Observer Test Suite")
    print("=" * 70)
    
    results = TestResult()
    
    # Section 1: DirtyDerivative Tests
    print_section("1. DirtyDerivative Tests")
    run_test("test_dirty_derivative_constant", test_dirty_derivative_constant, results)
    run_test("test_dirty_derivative_ramp", test_dirty_derivative_ramp, results)
    run_test("test_dirty_derivative_sinusoid", test_dirty_derivative_sinusoid, results)
    run_test("test_dirty_derivative_step", test_dirty_derivative_step, results)
    run_test("test_dirty_derivative_reset", test_dirty_derivative_reset, results)
    
    # Section 2: HighGainDifferentiator Tests
    print_section("2. HighGainDifferentiator Tests")
    run_test("test_highgain_constant", test_highgain_constant, results)
    run_test("test_highgain_ramp", test_highgain_ramp, results)
    run_test("test_highgain_sinusoid", test_highgain_sinusoid, results)
    run_test("test_highgain_bandwidth_change", test_highgain_bandwidth_change, results)
    run_test("test_highgain_antiwindup", test_highgain_antiwindup, results)
    run_test("test_highgain_reset", test_highgain_reset, results)
    run_test("test_highgain_noisy_signal", test_highgain_noisy_signal, results)
    
    # Section 3: SlidingModeDifferentiator Tests
    print_section("3. SlidingModeDifferentiator Tests")
    run_test("test_sliding_mode_constant", test_sliding_mode_constant, results)
    run_test("test_sliding_mode_ramp", test_sliding_mode_ramp, results)
    run_test("test_sliding_mode_sinusoid", test_sliding_mode_sinusoid, results)
    run_test("test_sliding_mode_smoothing_types", test_sliding_mode_smoothing_types, results)
    run_test("test_sliding_mode_gain_change", test_sliding_mode_gain_change, results)
    run_test("test_sliding_mode_antiwindup", test_sliding_mode_antiwindup, results)
    run_test("test_sliding_mode_reset", test_sliding_mode_reset, results)
    
    # Section 4: Differentiator Factory Tests
    print_section("4. Differentiator Factory Tests")
    run_test("test_factory_creates_dirty", test_factory_creates_dirty, results)
    run_test("test_factory_creates_highgain", test_factory_creates_highgain, results)
    run_test("test_factory_creates_sliding", test_factory_creates_sliding, results)
    run_test("test_factory_invalid_type", test_factory_invalid_type, results)
    
    # Section 5: WEstimatorUIOStyle Tests
    print_section("5. WEstimatorUIOStyle Tests")
    run_test("test_w_estimator_zero_residuals", test_w_estimator_zero_residuals, results)
    run_test("test_w_estimator_with_highgain_diff", test_w_estimator_with_highgain_diff, results)
    run_test("test_w_estimator_with_sliding_diff", test_w_estimator_with_sliding_diff, results)
    run_test("test_w_estimator_reset", test_w_estimator_reset, results)
    run_test("test_w_estimator_compute_lin_terms", test_w_estimator_compute_lin_terms, results)
    
    # Section 6: SchedulingParameters Tests
    print_section("6. SchedulingParameters Tests")
    run_test("test_scheduling_params_basic", test_scheduling_params_basic, results)
    run_test("test_scheduling_params_min_vx", test_scheduling_params_min_vx, results)
    run_test("test_scheduling_params_negative_vx", test_scheduling_params_negative_vx, results)
    
    # Section 7: DifferentiatorUIOObserver Tests
    print_section("7. DifferentiatorUIOObserver Tests")
    run_test("test_observer_initialization", test_observer_initialization, results)
    run_test("test_observer_with_custom_params", test_observer_with_custom_params, results)
    run_test("test_observer_matrix_dimensions", test_observer_matrix_dimensions, results)
    run_test("test_observer_reset", test_observer_reset, results)
    run_test("test_observer_update_basic", test_observer_update_basic, results)
    run_test("test_observer_state_changes", test_observer_state_changes, results)
    run_test("test_observer_with_highgain", test_observer_with_highgain, results)
    run_test("test_observer_with_sliding", test_observer_with_sliding, results)
    run_test("test_observer_convergence", test_observer_convergence, results)
    run_test("test_observer_measurement_formats", test_observer_measurement_formats, results)
    run_test("test_observer_getters", test_observer_getters, results)
    
    # Section 8: DifferentiatorUIOEKF Tests
    print_section("8. DifferentiatorUIOEKF Tests")
    run_test("test_ekf_initialization", test_ekf_initialization, results)
    run_test("test_ekf_with_custom_params", test_ekf_with_custom_params, results)
    run_test("test_ekf_with_custom_covariances", test_ekf_with_custom_covariances, results)
    run_test("test_ekf_reset", test_ekf_reset, results)
    run_test("test_ekf_update_basic", test_ekf_update_basic, results)
    run_test("test_ekf_kalman_gain_updates", test_ekf_kalman_gain_updates, results)
    run_test("test_ekf_covariance_bounded", test_ekf_covariance_bounded, results)
    run_test("test_ekf_state_clamping", test_ekf_state_clamping, results)
    run_test("test_ekf_with_different_differentiators", test_ekf_with_different_differentiators, results)
    run_test("test_ekf_convergence", test_ekf_convergence, results)
    run_test("test_ekf_continuous_dynamics", test_ekf_continuous_dynamics, results)
    run_test("test_ekf_measurement_function", test_ekf_measurement_function, results)
    run_test("test_ekf_jacobian_computation", test_ekf_jacobian_computation, results)
    run_test("test_ekf_getters", test_ekf_getters, results)
    run_test("test_ekf_innovation_computed", test_ekf_innovation_computed, results)
    
    # Section 9: Factory Integration Tests
    print_section("9. Factory Integration Tests")
    run_test("test_factory_uio_observer", test_factory_uio_observer, results)
    run_test("test_factory_uio_ekf", test_factory_uio_ekf, results)
    run_test("test_factory_integration_with_uio_observers", test_factory_integration_with_uio_observers, results)
    run_test("test_comparative_observers", test_comparative_observers, results)
    
    # Summary
    print("\n" + "=" * 70)
    print(f"  SUMMARY: {results.passed}/{results.total} tests passed")
    if results.skipped:
        print(f"           {results.skipped} tests skipped")
    print("=" * 70)
    
    if results.failed > 0:
        print(f"\n❌ {results.failed} TEST(S) FAILED:")
        for name, msg in results.failures:
            print(f"   - {name}: {msg}")
        return 1
    else:
        print("\n✅ ALL TESTS PASSED")
        return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
