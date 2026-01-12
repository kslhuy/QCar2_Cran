"""
Trust-Based Distributed Observer Package

This package provides a trust-aware distributed state estimation framework
for multi-vehicle coordination in potentially adversarial environments.

Components:
- trust_model: TriP (Trust-based Intelligent Platoon) trust evaluation
- weight_trust_module: Adaptive weight calculation based on trust scores  
- trust_based_fleet_estimator: Fleet estimators with trust-weighted consensus

Usage:
    from Observer.TrustbasedDistributedObserver import (
        TrustBasedFleetEstimator,
        TrustBasedKalmanEstimator,
        TriPTrustModel,
        WeightTrustModule,
        create_trust_based_estimator
    )
    
    # Create estimator
    estimator = create_trust_based_estimator(
        estimator_type='trust_consensus',
        vehicle_id=0,
        fleet_size=3,
        config={'trust': {'trust_threshold': 0.5}}
    )

See README.md for detailed documentation.
"""

from Observer.TrustbasedDistributedObserver.trust_model import (
    TriPTrustModel,
    TrustConfig,
    TrustScore,
    VehicleData
)

from Observer.TrustbasedDistributedObserver.weight_trust_module import (
    WeightTrustModule,
    WeightConfig,
    WeightResult,
    AdaptiveWeightCalculator
)

from Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator import (
    TrustBasedFleetEstimator,
    TrustBasedKalmanEstimator,
    create_trust_based_estimator
)

__all__ = [
    # Trust Model
    'TriPTrustModel',
    'TrustConfig', 
    'TrustScore',
    'VehicleData',
    
    # Weight Module
    'WeightTrustModule',
    'WeightConfig',
    'WeightResult',
    'AdaptiveWeightCalculator',
    
    # Fleet Estimators
    'TrustBasedFleetEstimator',
    'TrustBasedKalmanEstimator',
    'create_trust_based_estimator',
]

__version__ = '1.0.0'
__author__ = 'QCar Research Team'
