"""
LocalNeuralObs - Neural Network-Enhanced Local Observer

This module provides neural network-based state estimation for vehicle dynamics,
learning unknown tire forces and disturbances online.
"""

__version__ = '1.0.0'

# Lazy imports to avoid requiring torch if not using neural estimator
__all__ = [
    'NeuralObserverNet',
    'NeuralLuenbergerEstimator',
    'GradientSolver',
    'LearningBatch',
    'ModelQueue'
]


def __getattr__(name):
    """Lazy import to avoid torch dependency unless needed"""
    if name == 'NeuralObserverNet' or name == 'LearningBatch' or name == 'ModelQueue':
        from .neural_network import NeuralObserverNet, LearningBatch, ModelQueue
        return globals()[name]
    elif name == 'NeuralLuenbergerEstimator':
        from .neural_state_estimator import NeuralLuenbergerEstimator
        return NeuralLuenbergerEstimator
    elif name == 'GradientSolver':
        from .gradient_solver import GradientSolver
        return GradientSolver
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
