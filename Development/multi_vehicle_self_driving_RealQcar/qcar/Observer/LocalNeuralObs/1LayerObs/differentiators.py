"""
Differentiator Module

Provides multiple differentiator implementations for signal derivative estimation:
1. DirtyDerivative - Low-pass filtered differentiator (Tustin bilinear)
2. HighGainDifferentiator - 2nd-order high-gain observer
3. SlidingModeDifferentiator - Super-twisting sliding mode

All differentiators can be configured via YAML config file or directly.

Usage:
    # Method 1: Direct creation
    from differentiators import create_differentiator
    diff = create_differentiator('highgain', Ts=0.01, omega=50.0)
    
    # Method 2: From config file
    from differentiators import create_differentiator_from_config
    diff = create_differentiator_from_config('highgain', Ts=0.01)
    
    # Method 3: Load all params from config
    from differentiators import load_differentiator_config, DirtyDerivative
    config = load_differentiator_config()
    diff = DirtyDerivative(Ts=0.01, **config['dirty_derivative'])
"""

import numpy as np
from typing import Optional, Dict, Union
from pathlib import Path
import yaml


# ==============================================================================
# Configuration Loading
# ==============================================================================

def get_default_config_path() -> Path:
    """Get the default path to the differentiator config file"""
    return Path(__file__).parent / "config_differentiators.yaml"


def load_differentiator_config(config_path: Optional[Union[str, Path]] = None) -> Dict:
    """
    Load differentiator configuration from YAML file.
    
    Args:
        config_path: Path to config file. If None, uses default path.
        
    Returns:
        Dictionary with differentiator configurations
    """
    if config_path is None:
        config_path = get_default_config_path()
    
    config_path = Path(config_path)
    
    if not config_path.exists():
        # Return default config if file doesn't exist
        return get_default_config()
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    return config


def get_default_config() -> Dict:
    """Return default differentiator configuration"""
    return {
        'default_type': 'highgain',
        'dirty_derivative': {
            'tau': 0.02,
        },
        'highgain': {
            'omega': 50.0,
            'zeta': 0.707,
            'ydot_max': None,
        },
        'sliding_mode': {
            'k1': 30.0,
            'k2': 300.0,
            'epsilon': 0.01,
            'smoothing': 'epsilon',
            'v_max': None,
        },
    }


# ==============================================================================
# Differentiator Classes
# ==============================================================================

class DirtyDerivative:
    """
    Low-pass differentiator using Tustin (bilinear) discretization.
    
    Continuous transfer function:
        ydot = (s / (τs + 1)) * y
    
    Discretized with Tustin (bilinear transform):
        ydot_k = α·ydot_{k-1} + β·(y_k - y_{k-1})
    
    where:
        α = (2τ - Ts) / (2τ + Ts)
        β = 2 / (2τ + Ts)
    
    Args:
        Ts: Sample time [s]
        tau: Filter time constant [s] (larger = more smoothing)
        y0: Initial value of signal
    """
    
    def __init__(self, Ts: float, tau: float = 0.02, y0: float = 0.0):
        self.Ts = Ts
        self.tau = tau
        
        # Tustin discretization coefficients
        self.alpha = (2*tau - Ts) / (2*tau + Ts)
        self.beta = 2.0 / (2*tau + Ts)
        
        # State variables
        self.y_prev = float(y0)
        self.ydot = 0.0
    
    def update(self, y: float) -> float:
        """
        Update differentiator with new measurement
        
        Args:
            y: Current signal value
            
        Returns:
            Filtered derivative estimate
        """
        y = float(y)
        self.ydot = self.alpha * self.ydot + self.beta * (y - self.y_prev)
        self.y_prev = y
        return self.ydot
    
    def reset(self, y0: float = 0.0):
        """Reset differentiator state"""
        self.y_prev = float(y0)
        self.ydot = 0.0
    
    def get_derivative(self) -> float:
        """Get current derivative estimate"""
        return self.ydot


class HighGainDifferentiator:
    """
    2nd-order High-Gain Observer for differentiation.
    
    State-space form:
        x1 = y_hat (signal estimate)
        x2 = ydot_hat (derivative estimate)
    
    Dynamics (continuous):
        ẋ1 = x2 + L1·(y - x1)
        ẋ2 = L2·(y - x1)
    
    Discretized with Euler:
        x1_{k+1} = x1_k + Ts·(x2_k + L1·(y - x1_k))
        x2_{k+1} = x2_k + Ts·L2·(y - x1_k)
    
    Gain design (pole placement at -ω with damping ζ):
        L1 = 2·ζ·ω
        L2 = ω²
    
    Args:
        Ts: Sample time [s]
        omega: Observer bandwidth [rad/s] (higher = faster but more noise)
        zeta: Damping ratio (0.707 = Butterworth, 1.0 = critically damped)
        y0: Initial signal value
        ydot_max: Maximum derivative magnitude for anti-windup (None = no limit)
    """
    
    def __init__(self, Ts: float = 0.01, omega: float = 50.0, zeta: float = 0.707,
                 y0: float = 0.0, ydot_max: Optional[float] = None):
        self.Ts = Ts
        self.omega = float(omega)
        self.zeta = float(zeta)
        self.ydot_max = ydot_max
        
        # Observer gains (pole placement)
        self.L1 = 2.0 * self.zeta * self.omega
        self.L2 = self.omega ** 2
        
        # State variables
        self.y_hat = float(y0)
        self.ydot_hat = 0.0
    
    def update(self, y: float) -> float:
        """
        Update differentiator with new measurement
        
        Args:
            y: Current signal value
            
        Returns:
            Derivative estimate
        """
        y = float(y)
        e = y - self.y_hat
        
        # Discrete update (Euler)
        self.y_hat = self.y_hat + self.Ts * (self.ydot_hat + self.L1 * e)
        self.ydot_hat = self.ydot_hat + self.Ts * self.L2 * e
        
        # Anti-windup saturation
        if self.ydot_max is not None:
            self.ydot_hat = np.clip(self.ydot_hat, -self.ydot_max, self.ydot_max)
        
        return self.ydot_hat
    
    def reset(self, y0: float = 0.0):
        """Reset differentiator state"""
        self.y_hat = float(y0)
        self.ydot_hat = 0.0
    
    def get_derivative(self) -> float:
        """Get current derivative estimate"""
        return self.ydot_hat
    
    def set_bandwidth(self, omega: float, zeta: Optional[float] = None):
        """
        Update observer bandwidth dynamically
        
        Args:
            omega: New bandwidth [rad/s]
            zeta: New damping ratio (optional, keeps current if None)
        """
        self.omega = float(omega)
        if zeta is not None:
            self.zeta = float(zeta)
        self.L1 = 2.0 * self.zeta * self.omega
        self.L2 = self.omega ** 2


class SlidingModeDifferentiator:
    """
    Super-Twisting Sliding Mode Differentiator (Levant's robust differentiator).
    
    Provides finite-time convergence and robustness to bounded noise/disturbances.
    
    Continuous-time form:
        ẏ_hat = v_hat + k1·|e|^(1/2)·sign(e)
        v̇_hat = k2·sign(e)
        e = y - y_hat
    
    where v_hat converges to ydot.
    
    Discretized with semi-implicit Euler for improved stability:
        y_hat_{k+1} = y_hat_k + Ts·(v_hat_k + k1·|e_k|^0.5·sgn(e_k))
        v_hat_{k+1} = v_hat_k + Ts·k2·sgn(e_k)
    
    Smoothing options for sgn():
        - 'epsilon': sgn_ε(e) = e / (|e| + ε)
        - 'tanh': tanh(e / ε)
        - 'saturation': sat(e / ε)
    
    Gain tuning guidelines:
        - For noise bound L and Lipschitz constant M:
          k1 ≥ 1.5·sqrt(M), k2 ≥ 1.1·M
        - Higher k1, k2 = faster convergence but more noise amplification
    
    Args:
        Ts: Sample time [s]
        k1: First gain (affects convergence rate)
        k2: Second gain (affects robustness)
        epsilon: Smoothing parameter (smaller = more aggressive)
        y0: Initial signal value
        smoothing: Smoothing type ('epsilon', 'tanh', 'saturation')
        v_max: Maximum derivative magnitude for anti-windup
    """
    
    def __init__(self, Ts: float = 0.01, k1: float = 30.0, k2: float = 300.0,
                 epsilon: float = 0.01, y0: float = 0.0, 
                 smoothing: str = 'epsilon', v_max: Optional[float] = None):
        self.Ts = Ts
        self.k1 = float(k1)
        self.k2 = float(k2)
        self.epsilon = float(epsilon)
        self.smoothing = smoothing
        self.v_max = v_max
        
        # State variables
        self.y_hat = float(y0)
        self.v_hat = 0.0  # This is ydot_hat
    
    def _smooth_sign(self, e: float) -> float:
        """
        Smoothed sign function to reduce chattering
        
        Args:
            e: Error signal
            
        Returns:
            Smoothed sign value in [-1, 1]
        """
        if self.smoothing == 'tanh':
            return np.tanh(e / self.epsilon)
        elif self.smoothing == 'saturation':
            return np.clip(e / self.epsilon, -1.0, 1.0)
        else:  # 'epsilon' (default)
            return e / (abs(e) + self.epsilon)
    
    def update(self, y: float) -> float:
        """
        Update differentiator with new measurement
        
        Args:
            y: Current signal value
            
        Returns:
            Derivative estimate
        """
        y = float(y)
        e = y - self.y_hat
        
        # Smooth sign function
        s = self._smooth_sign(e)
        
        # Super-twisting injection terms
        sqrt_e = np.sqrt(abs(e) + 1e-12)  # Small offset for numerical stability
        inj1 = self.k1 * sqrt_e * s
        inj2 = self.k2 * s
        
        # Semi-implicit Euler discretization (better stability)
        self.y_hat = self.y_hat + self.Ts * (self.v_hat + inj1)
        self.v_hat = self.v_hat + self.Ts * inj2
        
        # Anti-windup saturation
        if self.v_max is not None:
            self.v_hat = np.clip(self.v_hat, -self.v_max, self.v_max)
        
        return self.v_hat
    
    def reset(self, y0: float = 0.0):
        """Reset differentiator state"""
        self.y_hat = float(y0)
        self.v_hat = 0.0
    
    def get_derivative(self) -> float:
        """Get current derivative estimate"""
        return self.v_hat
    
    def set_gains(self, k1: Optional[float] = None, k2: Optional[float] = None):
        """
        Update gains dynamically
        
        Args:
            k1: New first gain (optional)
            k2: New second gain (optional)
        """
        if k1 is not None:
            self.k1 = float(k1)
        if k2 is not None:
            self.k2 = float(k2)


# ==============================================================================
# Factory Functions
# ==============================================================================

def create_differentiator(diff_type: str = 'highgain', Ts: float = 0.01, 
                          y0: float = 0.0, **kwargs) -> Union[DirtyDerivative, HighGainDifferentiator, SlidingModeDifferentiator]:
    """
    Factory function to create differentiator instances
    
    Args:
        diff_type: Type of differentiator:
            - 'dirty': Low-pass filtered (DirtyDerivative)
            - 'highgain': High-gain observer (HighGainDifferentiator)
            - 'sliding': Super-twisting sliding mode (SlidingModeDifferentiator)
        Ts: Sample time [s]
        y0: Initial signal value
        **kwargs: Additional parameters for specific differentiator type
        
    Returns:
        Differentiator instance
    """
    if diff_type == 'dirty':
        tau = kwargs.get('tau', 0.02)
        return DirtyDerivative(Ts=Ts, tau=tau, y0=y0)
    elif diff_type == 'highgain':
        omega = kwargs.get('omega', 50.0)
        zeta = kwargs.get('zeta', 0.707)
        ydot_max = kwargs.get('ydot_max', None)
        return HighGainDifferentiator(Ts=Ts, omega=omega, zeta=zeta, 
                                       y0=y0, ydot_max=ydot_max)
    elif diff_type == 'sliding':
        k1 = kwargs.get('k1', 30.0)
        k2 = kwargs.get('k2', 300.0)
        epsilon = kwargs.get('epsilon', 0.01)
        smoothing = kwargs.get('smoothing', 'epsilon')
        v_max = kwargs.get('v_max', None)
        return SlidingModeDifferentiator(Ts=Ts, k1=k1, k2=k2, epsilon=epsilon,
                                          y0=y0, smoothing=smoothing, v_max=v_max)
    else:
        raise ValueError(f"Unknown differentiator type: {diff_type}. "
                        f"Available: 'dirty', 'highgain', 'sliding'")


def create_differentiator_from_config(diff_type: Optional[str] = None,
                                       Ts: float = 0.01,
                                       y0: float = 0.0,
                                       config_path: Optional[Union[str, Path]] = None,
                                       **overrides) -> Union[DirtyDerivative, HighGainDifferentiator, SlidingModeDifferentiator]:
    """
    Create a differentiator using parameters from YAML config file.
    
    Args:
        diff_type: Type of differentiator. If None, uses default from config.
        Ts: Sample time [s]
        y0: Initial signal value
        config_path: Path to config file. If None, uses default path.
        **overrides: Override specific parameters from config
        
    Returns:
        Configured differentiator instance
    """
    config = load_differentiator_config(config_path)
    
    # Get type from config if not specified
    if diff_type is None:
        diff_type = config.get('default_type', 'highgain')
    
    # Get parameters for this type
    type_map = {
        'dirty': 'dirty_derivative',
        'highgain': 'highgain',
        'sliding': 'sliding_mode',
    }
    config_key = type_map.get(diff_type, diff_type)
    
    params = config.get(config_key, {}).copy()
    
    # Apply overrides
    params.update(overrides)
    
    return create_differentiator(diff_type=diff_type, Ts=Ts, y0=y0, **params)


def get_differentiator_types() -> list:
    """Get list of available differentiator types"""
    return ['dirty', 'highgain', 'sliding']


# ==============================================================================
# Test / Demo
# ==============================================================================

if __name__ == '__main__':
    print("=" * 60)
    print("Differentiator Module Test")
    print("=" * 60)
    
    # Load config
    config = load_differentiator_config()
    print(f"\nDefault type: {config.get('default_type')}")
    print(f"Config keys: {list(config.keys())}")
    
    # Create each type
    Ts = 0.01
    
    for diff_type in get_differentiator_types():
        print(f"\nTesting {diff_type}:")
        diff = create_differentiator_from_config(diff_type, Ts=Ts)
        
        # Simple ramp test
        slope = 2.0
        for i in range(100):
            y = slope * i * Ts
            ydot = diff.update(y)
        
        print(f"  Final estimate: {ydot:.4f} (true: {slope})")
    
    print("\n✅ All differentiators working!")
