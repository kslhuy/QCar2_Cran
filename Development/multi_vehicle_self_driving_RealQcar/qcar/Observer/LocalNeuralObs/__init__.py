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
    import importlib
    if name == 'NeuralObserverNet' or name == 'LearningBatch' or name == 'ModelQueue':
        # 2LayerObs starts with number, use importlib
        module = importlib.import_module('.2LayerObs.neural_network', package=__name__)
        return getattr(module, name)
    elif name == 'NeuralLuenbergerEstimator':
        module = importlib.import_module('.2LayerObs.neural_state_estimator', package=__name__)
        return module.NeuralLuenbergerEstimator
    elif name == 'GradientSolver':
        module = importlib.import_module('.2LayerObs.gradient_solver', package=__name__)
        return module.GradientSolver
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
