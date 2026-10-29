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
    'NeuralObsRecorder',
    'GradientSolver',
    'LearningBatch',
    'ModelQueue'
]


# Symbols crossing numeric layer directories share one canonical module identity.
_LAYER_EXPORTS = {
    '_layer_1LayerObs_differentiator_uio_ekf_DifferentiatorUIOEKF': ('1LayerObs.differentiator_uio_ekf', 'DifferentiatorUIOEKF'),
    '_layer_1LayerObs_differentiator_uio_observer_DifferentiatorUIOObserver': ('1LayerObs.differentiator_uio_observer', 'DifferentiatorUIOObserver'),
    '_layer_1LayerObs_firstLayerObserverBase_FirstLayerObserverBase': ('1LayerObs.firstLayerObserverBase', 'FirstLayerObserverBase'),
    '_layer_1LayerObs_firstLayerObserverBase_create_first_layer_observer': ('1LayerObs.firstLayerObserverBase', 'create_first_layer_observer'),
    '_layer_1LayerObs_qlpv_observer_kalma_qLPVAugmentedObserver': ('1LayerObs.qlpv_observer_kalma', 'qLPVAugmentedObserver'),
    '_layer_1LayerObs_qlpv_observer_kalma_qLPVKalmanObserver': ('1LayerObs.qlpv_observer_kalma', 'qLPVKalmanObserver'),
    '_layer_1LayerObs_qlpv_observer_qLPVAugmentedObserver': ('1LayerObs.qlpv_observer', 'qLPVAugmentedObserver'),
    '_layer_1LayerObs_tire_estimator_simple_TireEstimatorSimple': ('1LayerObs.tire_estimator_simple', 'TireEstimatorSimple'),
    '_layer_2LayerObs_Design_LMI_neural_NeuralPolytopicVertex': ('2LayerObs.Design_LMI_neural', 'NeuralPolytopicVertex'),
    '_layer_2LayerObs_Design_LMI_neural_NeuralQLPVGainScheduler': ('2LayerObs.Design_LMI_neural', 'NeuralQLPVGainScheduler'),
    '_layer_2LayerObs_Design_LMI_neural_compute_discrete_contraction_lmi': ('2LayerObs.Design_LMI_neural', 'compute_discrete_contraction_lmi'),
    '_layer_2LayerObs_Design_LMI_neural_compute_discrete_h2_lmi_observer_gain': ('2LayerObs.Design_LMI_neural', 'compute_discrete_h2_lmi_observer_gain'),
    '_layer_2LayerObs_Design_LMI_neural_compute_discrete_hinf_lmi_observer_gain': ('2LayerObs.Design_LMI_neural', 'compute_discrete_hinf_lmi_observer_gain'),
    '_layer_2LayerObs_Design_LMI_neural_compute_discrete_l2_lmi_observer_gain': ('2LayerObs.Design_LMI_neural', 'compute_discrete_l2_lmi_observer_gain'),
    '_layer_2LayerObs_Design_LMI_neural_discretize_system_zoh': ('2LayerObs.Design_LMI_neural', 'discretize_system_zoh'),
    '_layer_2LayerObs_Design_LMI_neural_validate_discrete_observer_gain': ('2LayerObs.Design_LMI_neural', 'validate_discrete_observer_gain'),
    '_layer_2LayerObs_neural_state_estimator_NeuralLuenbergerEstimator': ('2LayerObs.neural_state_estimator', 'NeuralLuenbergerEstimator'),
}


def __getattr__(name):
    """Lazy import to avoid torch dependency unless needed"""
    import importlib
    if name in _LAYER_EXPORTS:
        module_name, attribute = _LAYER_EXPORTS[name]
        module = importlib.import_module('.' + module_name, package=__name__)
        return getattr(module, attribute)
    if name == 'NeuralObserverNet' or name == 'LearningBatch' or name == 'ModelQueue':
        # 2LayerObs starts with number, use importlib
        module = importlib.import_module('.2LayerObs.neural_network', package=__name__)
        return getattr(module, name)
    elif name == 'NeuralLuenbergerEstimator':
        module = importlib.import_module('.2LayerObs.neural_state_estimator', package=__name__)
        return module.NeuralLuenbergerEstimator
    elif name == 'NeuralObsRecorder':
        # Import unified recorder (supports both 1-layer and 2-layer)
        from .neural_obs_recorder import NeuralObsRecorder
        return NeuralObsRecorder
    elif name == 'GradientSolver':
        module = importlib.import_module('.2LayerObs.gradient_solver', package=__name__)
        return module.GradientSolver
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
