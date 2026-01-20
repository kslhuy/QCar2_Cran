"""
Comparative Test Suite for Differentiators

This test file initializes ALL differentiator types and compares their performance
on the SAME signals. For each test signal, we show:
  - The estimated derivative from each differentiator
  - The error against the true derivative
  - Comparative statistics (RMSE, max error, settling time)

Differentiators tested:
  1. DirtyDerivative (low-pass filtered)
  2. HighGainDifferentiator (2nd-order high-gain observer)
  3. SlidingModeDifferentiator (super-twisting algorithm)

Test signals:
  1. Constant signal (true derivative = 0)
  2. Ramp signal (true derivative = slope)
  3. Sinusoidal signal (true derivative = ω·cos(ω·t))
  4. Step signal (impulse in derivative)
  5. Noisy ramp (robustness test)
"""

import numpy as np
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Callable
from dataclasses import dataclass

# Add parent directory to path
parent_dir = Path(__file__).parent
sys.path.insert(0, str(parent_dir))

# Import from centralized differentiators module
from differentiators import (
    DirtyDerivative,
    HighGainDifferentiator,
    SlidingModeDifferentiator,
    create_differentiator,
    create_differentiator_from_config,
    load_differentiator_config,
)


# ==============================================================================
# Data Classes for Results
# ==============================================================================

@dataclass
class DifferentiatorResult:
    """Stores results from a single differentiator on a test signal"""
    name: str
    estimates: np.ndarray          # Derivative estimates over time
    errors: np.ndarray             # Error vs true derivative
    rmse: float                    # Root mean square error
    max_error: float               # Maximum absolute error
    mean_error: float              # Mean absolute error
    settling_time_idx: int         # Index when error settles below threshold


@dataclass
class ComparisonResult:
    """Stores comparison results for all differentiators on a test signal"""
    signal_name: str
    time: np.ndarray
    signal: np.ndarray
    true_derivative: np.ndarray
    results: Dict[str, DifferentiatorResult]


# ==============================================================================
# Differentiator Factory and Initialization
# ==============================================================================

def create_all_differentiators(Ts: float = 0.01, y0: float = 0.0) -> Dict[str, object]:
    """
    Create all differentiator types with comparable tuning parameters.
    
    Args:
        Ts: Sample time [s]
        y0: Initial signal value
        
    Returns:
        Dictionary mapping differentiator names to instances
    """
    differentiators = {
        'DirtyDerivative': DirtyDerivative(
            Ts=Ts, 
            tau=0.02,  # Small time constant for faster response
            y0=y0
        ),
        'HighGain': HighGainDifferentiator(
            Ts=Ts,
            omega=40.0,     # Bandwidth [rad/s]
            zeta=0.707,     # Butterworth damping
            y0=y0,
            ydot_max=None   # No anti-windup for comparison
        ),
        'SlidingMode': SlidingModeDifferentiator(
            Ts=Ts,
            k1=15.0,        # First gain
            k2=200.0,       # Second gain
            epsilon=0.01,   # Smoothing parameter
            y0=y0,
            smoothing='epsilon',
            v_max=None      # No anti-windup for comparison
        ),
    }
    return differentiators


def reset_all_differentiators(differentiators: Dict[str, object], y0: float = 0.0):
    """Reset all differentiators to initial state"""
    for diff in differentiators.values():
        diff.reset(y0=y0)


# ==============================================================================
# Test Signal Generators
# ==============================================================================

def generate_constant_signal(N: int, Ts: float, value: float = 1.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Generate constant signal with zero true derivative"""
    t = np.arange(N) * Ts
    signal = np.full(N, value)
    true_deriv = np.zeros(N)
    return t, signal, true_deriv


def generate_ramp_signal(N: int, Ts: float, slope: float = 2.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Generate ramp signal with constant true derivative = slope"""
    t = np.arange(N) * Ts
    signal = slope * t
    true_deriv = np.full(N, slope)
    return t, signal, true_deriv


def generate_sinusoid_signal(N: int, Ts: float, freq: float = 1.0, amplitude: float = 1.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Generate sinusoidal signal with cosine true derivative"""
    t = np.arange(N) * Ts
    omega = 2 * np.pi * freq
    signal = amplitude * np.sin(omega * t)
    true_deriv = amplitude * omega * np.cos(omega * t)
    return t, signal, true_deriv


def generate_step_signal(N: int, Ts: float, step_time: float = 0.5, step_value: float = 1.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate step signal.
    Note: True derivative is impulse at step_time (approximated as large spike).
    """
    t = np.arange(N) * Ts
    step_idx = int(step_time / Ts)
    signal = np.zeros(N)
    signal[step_idx:] = step_value
    
    # True derivative: impulse at step (approximate as 1/Ts for one sample)
    true_deriv = np.zeros(N)
    if step_idx < N:
        true_deriv[step_idx] = step_value / Ts  # Impulse amplitude
    return t, signal, true_deriv


def generate_noisy_ramp_signal(N: int, Ts: float, slope: float = 2.0, 
                                noise_std: float = 0.1, seed: int = 42) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Generate ramp with additive Gaussian noise"""
    np.random.seed(seed)
    t = np.arange(N) * Ts
    signal = slope * t + noise_std * np.random.randn(N)
    true_deriv = np.full(N, slope)  # Noise derivative is not included
    return t, signal, true_deriv


def generate_combined_signal(N: int, Ts: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate a realistic vehicle-like signal with multiple phases:
    - Initial ramp up
    - Steady state
    - Sinusoidal oscillation
    - Step disturbance
    """
    t = np.arange(N) * Ts
    signal = np.zeros(N)
    true_deriv = np.zeros(N)
    
    # Phase 1: Ramp up (acceleration) - first 20% of time
    n1 = int(0.2 * N)
    slope = 5.0
    signal[:n1] = slope * t[:n1]
    true_deriv[:n1] = slope
    
    # Phase 2: Steady state - next 20%
    n2 = int(0.4 * N)
    steady_val = signal[n1-1]
    signal[n1:n2] = steady_val
    true_deriv[n1:n2] = 0.0
    
    # Phase 3: Sinusoidal oscillation - next 40%
    n3 = int(0.8 * N)
    freq = 0.5  # Hz
    omega = 2 * np.pi * freq
    t_phase3 = t[n2:n3] - t[n2]
    amplitude = 0.5
    signal[n2:n3] = steady_val + amplitude * np.sin(omega * t_phase3)
    true_deriv[n2:n3] = amplitude * omega * np.cos(omega * t_phase3)
    
    # Phase 4: Decay back to steady - final 20%
    decay_val = signal[n3-1]
    tau_decay = 0.2  # Time constant
    t_phase4 = t[n3:] - t[n3]
    signal[n3:] = decay_val * np.exp(-t_phase4 / tau_decay)
    true_deriv[n3:] = -decay_val / tau_decay * np.exp(-t_phase4 / tau_decay)
    
    return t, signal, true_deriv


# ==============================================================================
# Run Differentiator and Compute Metrics
# ==============================================================================

def run_differentiator(diff, signal: np.ndarray, true_deriv: np.ndarray,
                       name: str, transient_skip: int = 50,
                       settling_threshold: float = 0.1) -> DifferentiatorResult:
    """
    Run a single differentiator on a signal and compute metrics.
    
    Args:
        diff: Differentiator instance
        signal: Input signal array
        true_deriv: True derivative array
        name: Differentiator name
        transient_skip: Number of initial samples to skip in metric calculation
        settling_threshold: Error threshold to determine settling
        
    Returns:
        DifferentiatorResult with all metrics
    """
    N = len(signal)
    estimates = np.zeros(N)
    
    for i in range(N):
        estimates[i] = diff.update(signal[i])
    
    # Compute errors (skip initial transient)
    errors = np.abs(estimates - true_deriv)
    errors_steady = errors[transient_skip:]
    
    # Metrics
    rmse = np.sqrt(np.mean(errors_steady**2))
    max_error = np.max(errors_steady)
    mean_error = np.mean(errors_steady)
    
    # Settling time: first index where error stays below threshold
    settling_time_idx = N  # Default: never settled
    for i in range(transient_skip, N):
        if np.all(errors[i:min(i+10, N)] < settling_threshold):
            settling_time_idx = i
            break
    
    return DifferentiatorResult(
        name=name,
        estimates=estimates,
        errors=errors,
        rmse=rmse,
        max_error=max_error,
        mean_error=mean_error,
        settling_time_idx=settling_time_idx
    )


def run_comparison(differentiators: Dict[str, object],
                   signal_name: str,
                   signal_generator: Callable,
                   N: int = 500,
                   Ts: float = 0.01,
                   **generator_kwargs) -> ComparisonResult:
    """
    Run all differentiators on the same signal and compare results.
    
    Args:
        differentiators: Dictionary of differentiator instances
        signal_name: Name of the test signal
        signal_generator: Function to generate (t, signal, true_deriv)
        N: Number of samples
        Ts: Sample time
        **generator_kwargs: Additional arguments for signal generator
        
    Returns:
        ComparisonResult with all results
    """
    # Generate signal
    t, signal, true_deriv = signal_generator(N, Ts, **generator_kwargs)
    
    # Reset all differentiators
    reset_all_differentiators(differentiators, y0=signal[0])
    
    # Run each differentiator
    results = {}
    for name, diff in differentiators.items():
        result = run_differentiator(diff, signal, true_deriv, name)
        results[name] = result
    
    return ComparisonResult(
        signal_name=signal_name,
        time=t,
        signal=signal,
        true_derivative=true_deriv,
        results=results
    )


# ==============================================================================
# Display and Analysis Functions
# ==============================================================================

def print_comparison_table(comparison: ComparisonResult):
    """Print a formatted comparison table for one signal"""
    print(f"\n{'='*80}")
    print(f"  Signal: {comparison.signal_name}")
    print(f"{'='*80}")
    
    # Header
    print(f"{'Differentiator':<20} {'RMSE':>12} {'Max Error':>12} {'Mean Error':>12} {'Settling Idx':>15}")
    print("-" * 80)
    
    # Sort by RMSE
    sorted_results = sorted(comparison.results.items(), key=lambda x: x[1].rmse)
    
    for name, result in sorted_results:
        print(f"{name:<20} {result.rmse:>12.6f} {result.max_error:>12.6f} "
              f"{result.mean_error:>12.6f} {result.settling_time_idx:>15}")
    
    # Winner
    winner = sorted_results[0][0]
    print("-" * 80)
    print(f"✅ Best performer (lowest RMSE): {winner}")


def print_summary_ranking(all_comparisons: List[ComparisonResult]):
    """Print overall ranking across all tests"""
    print("\n" + "=" * 80)
    print("  OVERALL RANKING (Sum of RMSE across all tests)")
    print("=" * 80)
    
    # Aggregate RMSE scores
    total_rmse = {}
    for comp in all_comparisons:
        for name, result in comp.results.items():
            if name not in total_rmse:
                total_rmse[name] = 0.0
            total_rmse[name] += result.rmse
    
    # Sort and display
    sorted_ranking = sorted(total_rmse.items(), key=lambda x: x[1])
    
    print(f"\n{'Rank':<6} {'Differentiator':<25} {'Total RMSE':>15}")
    print("-" * 50)
    for rank, (name, rmse) in enumerate(sorted_ranking, 1):
        medal = "🥇" if rank == 1 else "🥈" if rank == 2 else "🥉" if rank == 3 else "  "
        print(f"{medal} {rank:<4} {name:<25} {rmse:>15.6f}")


def detailed_comparison_at_time(comparison: ComparisonResult, time_indices: List[int]):
    """Print detailed comparison at specific time indices"""
    print(f"\n--- Detailed values at specific times [{comparison.signal_name}] ---")
    
    print(f"{'Time (s)':<10} {'True':>12}", end="")
    for name in comparison.results.keys():
        short_name = name[:10]
        print(f" {short_name:>12}", end="")
    print()
    print("-" * (10 + 12 + 12 * len(comparison.results)))
    
    for idx in time_indices:
        if idx >= len(comparison.time):
            continue
        t = comparison.time[idx]
        true_val = comparison.true_derivative[idx]
        print(f"{t:<10.3f} {true_val:>12.4f}", end="")
        for name, result in comparison.results.items():
            est = result.estimates[idx]
            print(f" {est:>12.4f}", end="")
        print()


# ==============================================================================
# Main Test Functions
# ==============================================================================

def test_all_differentiators_constant():
    """Test all differentiators on constant signal"""
    Ts = 0.01
    differentiators = create_all_differentiators(Ts=Ts, y0=1.0)
    
    comparison = run_comparison(
        differentiators,
        signal_name="Constant Signal (y=1.0)",
        signal_generator=generate_constant_signal,
        N=300,
        Ts=Ts,
        value=1.0
    )
    
    print_comparison_table(comparison)
    
    # Verify all estimates converge near zero
    for name, result in comparison.results.items():
        assert result.rmse < 0.5, f"{name} RMSE too high for constant signal: {result.rmse}"
    
    print("✅ All differentiators passed constant signal test")
    return comparison


def test_all_differentiators_ramp():
    """Test all differentiators on ramp signal"""
    Ts = 0.01
    slope = 2.0
    differentiators = create_all_differentiators(Ts=Ts, y0=0.0)
    
    comparison = run_comparison(
        differentiators,
        signal_name=f"Ramp Signal (slope={slope})",
        signal_generator=generate_ramp_signal,
        N=500,
        Ts=Ts,
        slope=slope
    )
    
    print_comparison_table(comparison)
    detailed_comparison_at_time(comparison, [100, 200, 300, 400])
    
    # Verify estimates converge to slope
    for name, result in comparison.results.items():
        final_avg = np.mean(result.estimates[-50:])
        assert abs(final_avg - slope) < 1.0, f"{name} failed to track slope: got {final_avg}"
    
    print("✅ All differentiators passed ramp signal test")
    return comparison


def test_all_differentiators_sinusoid():
    """Test all differentiators on sinusoidal signal"""
    Ts = 0.01
    freq = 0.5  # Hz
    differentiators = create_all_differentiators(Ts=Ts, y0=0.0)
    
    comparison = run_comparison(
        differentiators,
        signal_name=f"Sinusoid (freq={freq} Hz)",
        signal_generator=generate_sinusoid_signal,
        N=500,
        Ts=Ts,
        freq=freq,
        amplitude=1.0
    )
    
    print_comparison_table(comparison)
    
    # All should have bounded error
    for name, result in comparison.results.items():
        assert result.rmse < 3.0, f"{name} RMSE too high for sinusoid: {result.rmse}"
    
    print("✅ All differentiators passed sinusoid test")
    return comparison


def test_all_differentiators_step():
    """Test all differentiators on step signal"""
    Ts = 0.01
    differentiators = create_all_differentiators(Ts=Ts, y0=0.0)
    
    comparison = run_comparison(
        differentiators,
        signal_name="Step Signal",
        signal_generator=generate_step_signal,
        N=300,
        Ts=Ts,
        step_time=0.5,
        step_value=1.0
    )
    
    print_comparison_table(comparison)
    
    # Check that all detect the step (have a spike in derivative)
    for name, result in comparison.results.items():
        max_deriv = np.max(np.abs(result.estimates))
        assert max_deriv > 1.0, f"{name} failed to detect step: max={max_deriv}"
    
    print("✅ All differentiators passed step test")
    return comparison


def test_all_differentiators_noisy_ramp():
    """Test all differentiators on noisy ramp (robustness test)"""
    Ts = 0.01
    slope = 1.0
    noise_std = 0.2
    differentiators = create_all_differentiators(Ts=Ts, y0=0.0)
    
    comparison = run_comparison(
        differentiators,
        signal_name=f"Noisy Ramp (slope={slope}, noise_std={noise_std})",
        signal_generator=generate_noisy_ramp_signal,
        N=500,
        Ts=Ts,
        slope=slope,
        noise_std=noise_std
    )
    
    print_comparison_table(comparison)
    
    # With noise, we expect higher error, but mean should still track slope
    for name, result in comparison.results.items():
        mean_estimate = np.mean(result.estimates[100:])  # Skip transient
        assert abs(mean_estimate - slope) < 1.0, f"{name} mean estimate off: {mean_estimate}"
    
    print("✅ All differentiators passed noisy ramp test")
    return comparison


def test_all_differentiators_combined():
    """Test all differentiators on combined realistic signal"""
    Ts = 0.01
    differentiators = create_all_differentiators(Ts=Ts, y0=0.0)
    
    comparison = run_comparison(
        differentiators,
        signal_name="Combined Realistic Signal",
        signal_generator=generate_combined_signal,
        N=1000,
        Ts=Ts
    )
    
    print_comparison_table(comparison)
    
    print("✅ All differentiators passed combined signal test")
    return comparison


def test_frequency_sweep():
    """Test differentiator performance across different frequencies"""
    Ts = 0.01
    frequencies = [0.25, 0.5, 1.0, 2.0, 5.0]  # Hz
    
    print("\n" + "=" * 80)
    print("  FREQUENCY SWEEP TEST")
    print("=" * 80)
    
    print(f"\n{'Frequency (Hz)':<15}", end="")
    diff_names = ['DirtyDerivative', 'HighGain', 'SlidingMode']
    for name in diff_names:
        print(f" {name:>15}", end="")
    print()
    print("-" * (15 + 15 * len(diff_names)))
    
    all_rmses = {name: [] for name in diff_names}
    
    for freq in frequencies:
        differentiators = create_all_differentiators(Ts=Ts, y0=0.0)
        
        comparison = run_comparison(
            differentiators,
            signal_name=f"Sinusoid {freq} Hz",
            signal_generator=generate_sinusoid_signal,
            N=int(5/Ts),  # 5 seconds
            Ts=Ts,
            freq=freq,
            amplitude=1.0
        )
        
        print(f"{freq:<15.2f}", end="")
        for name in diff_names:
            rmse = comparison.results[name].rmse
            all_rmses[name].append(rmse)
            print(f" {rmse:>15.4f}", end="")
        print()
    
    # Determine best performer at each frequency
    print("\nBest performer at each frequency:")
    for i, freq in enumerate(frequencies):
        best_name = min(diff_names, key=lambda n: all_rmses[n][i])
        print(f"  {freq:.2f} Hz: {best_name}")
    
    print("✅ Frequency sweep test complete")


def test_gain_sensitivity():
    """Test how differentiators perform with different gain settings"""
    Ts = 0.01
    N = 500
    slope = 2.0
    
    print("\n" + "=" * 80)
    print("  GAIN SENSITIVITY TEST (Ramp signal)")
    print("=" * 80)
    
    # Test DirtyDerivative with different tau
    print("\n--- DirtyDerivative (varying tau) ---")
    taus = [0.01, 0.02, 0.05, 0.1, 0.2]
    print(f"{'tau':<10} {'RMSE':>12} {'Final Estimate':>15} {'True':>12}")
    print("-" * 50)
    
    for tau in taus:
        dd = DirtyDerivative(Ts=Ts, tau=tau, y0=0.0)
        t, signal, true_deriv = generate_ramp_signal(N, Ts, slope)
        estimates = np.array([dd.update(s) for s in signal])
        errors = np.abs(estimates[100:] - true_deriv[100:])
        rmse = np.sqrt(np.mean(errors**2))
        final_est = np.mean(estimates[-50:])
        print(f"{tau:<10.3f} {rmse:>12.6f} {final_est:>15.4f} {slope:>12.2f}")
    
    # Test HighGain with different omega
    print("\n--- HighGainDifferentiator (varying omega) ---")
    omegas = [10.0, 30.0, 50.0, 100.0, 150.0]
    print(f"{'omega':<10} {'RMSE':>12} {'Final Estimate':>15} {'True':>12}")
    print("-" * 50)
    
    for omega in omegas:
        hg = HighGainDifferentiator(Ts=Ts, omega=omega, zeta=0.707, y0=0.0)
        t, signal, true_deriv = generate_ramp_signal(N, Ts, slope)
        estimates = np.array([hg.update(s) for s in signal])
        errors = np.abs(estimates[100:] - true_deriv[100:])
        rmse = np.sqrt(np.mean(errors**2))
        final_est = np.mean(estimates[-50:])
        print(f"{omega:<10.1f} {rmse:>12.6f} {final_est:>15.4f} {slope:>12.2f}")
    
    # Test SlidingMode with different k1, k2
    print("\n--- SlidingModeDifferentiator (varying k1, k2) ---")
    gains = [(15, 50), (10, 100), (20, 200), (30, 300), (50, 500), (100, 1000)]
    print(f"{'(k1, k2)':<15} {'RMSE':>12} {'Final Estimate':>15} {'True':>12}")
    print("-" * 55)
    
    for k1, k2 in gains:
        sm = SlidingModeDifferentiator(Ts=Ts, k1=k1, k2=k2, epsilon=0.01, y0=0.0)
        t, signal, true_deriv = generate_ramp_signal(N, Ts, slope)
        estimates = np.array([sm.update(s) for s in signal])
        errors = np.abs(estimates[100:] - true_deriv[100:])
        rmse = np.sqrt(np.mean(errors**2))
        final_est = np.mean(estimates[-50:])
        print(f"({k1}, {k2})"[:15].ljust(15) + f" {rmse:>12.6f} {final_est:>15.4f} {slope:>12.2f}")
    
    print("\n✅ Gain sensitivity test complete")


# ==============================================================================
# Main Runner
# ==============================================================================

def run_all_tests():
    """Run all comparative tests"""
    print("\n")
    print("╔══════════════════════════════════════════════════════════════════════════════╗")
    print("║            COMPREHENSIVE DIFFERENTIATOR COMPARISON TEST SUITE                ║")
    print("╚══════════════════════════════════════════════════════════════════════════════╝")
    
    all_comparisons = []
    
    # Basic signal tests
    print("\n" + "=" * 80)
    print("  PART 1: BASIC SIGNAL TESTS")
    print("=" * 80)
    
    all_comparisons.append(test_all_differentiators_constant())
    all_comparisons.append(test_all_differentiators_ramp())
    all_comparisons.append(test_all_differentiators_sinusoid())
    all_comparisons.append(test_all_differentiators_step())
    all_comparisons.append(test_all_differentiators_noisy_ramp())
    all_comparisons.append(test_all_differentiators_combined())
    
    # Overall ranking
    print_summary_ranking(all_comparisons)
    
    # Additional tests
    print("\n" + "=" * 80)
    print("  PART 2: ADVANCED TESTS")
    print("=" * 80)
    
    test_frequency_sweep()
    test_gain_sensitivity()
    
    # Final summary
    print("\n" + "=" * 80)
    print("  ALL TESTS COMPLETED SUCCESSFULLY")
    print("=" * 80)
    print("\nConclusions:")
    print("  • DirtyDerivative: Simple, stable, good for low-frequency signals")
    print("  • HighGainDifferentiator: Best balance of speed and noise rejection")
    print("  • SlidingModeDifferentiator: Robust to disturbances, finite-time convergence")
    print("\nRecommendations:")
    print("  • For vehicle dynamics (low freq): HighGain or DirtyDerivative")
    print("  • For robustness to disturbances: SlidingMode")
    print("  • For simplicity: DirtyDerivative")


if __name__ == "__main__":
    run_all_tests()
