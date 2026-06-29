# Controller module

# Longitudinal controllers
from .longitudinal_controllers import (
    LongitudinalControllerBase,
    PIDVelocityController,
    QCar2SpeedController,
    CACCLongitudinalController,
    SA_ACCController,
    FixConstantController,
    LeaderCruiseController,
    ControllerFactory,
)

# Lateral controllers
from .lateral_controllers import (
    LateralControllerBase,
    PurePursuitController,
    StanleyController,
    LookaheadController,
    FusionLateralController,
    DummyLateralController,
    LateralControllerFactory,
    wrap_to_pi,
)


# MPC controllers (optional - requires casadi)
try:
    from .mpc_controller import (
        MPCControllerBase,
        CasADiMPCController,
        DynamicBicycleMPCController,
        MPCControllerFactory,
    )
    from .mpc_wrappers import (
        MPCLongitudinalWrapper,
        MPCLateralWrapper,
        MPCCombinedController,
    )

    MPC_AVAILABLE = True
except ImportError:
    MPC_AVAILABLE = False

__all__ = [
    # Longitudinal
    "LongitudinalControllerBase",
    "PIDVelocityController",
    "QCar2SpeedController",
    "CACCLongitudinalController",
    "SA_ACCController",
    "FixConstantController",
    "LeaderCruiseController",
    "ControllerFactory",
    # Lateral
    "LateralControllerBase",
    "PurePursuitController",
    "StanleyController",
    "LookaheadController",
    "HybridLateralController",
    "FusionLateralController",
    "DummyLateralController",
    "LateralControllerFactory",
    "wrap_to_pi",
    # MPC (if available)
    "MPC_AVAILABLE",
]

if MPC_AVAILABLE:
    __all__.extend(
        [
            "MPCControllerBase",
            "CasADiMPCController",
            "DynamicBicycleMPCController",
            "MPCControllerFactory",
            "MPCLongitudinalWrapper",
            "MPCLateralWrapper",
            "MPCCombinedController",
        ]
    )
