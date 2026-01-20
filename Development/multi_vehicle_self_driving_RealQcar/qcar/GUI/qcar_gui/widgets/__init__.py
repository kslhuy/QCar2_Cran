"""
Widgets package for QCar Fleet Controller.

This package contains all reusable UI components for the application.
"""

from .base import (
    BaseWidget,
    ThemedButton,
    ThemedEntry,
    ThemedLabel,
    ThemedLabelFrame,
    ExpandablePanel,
    StatusIndicator,
    ScrollableFrame,
    FormRow,
)

from .car_panel import (
    CarState,
    CarPanelCallbacks,
    CarPanelWidget,
    TelemetryDisplay,
    ControlButtons,
    ManualControlPanel,
    VelocityControl,
    PathControl,
    PlatoonControl,
)

from .fleet_controls import (
    FleetControlCallbacks,
    FleetControlsWidget,
)

from .status_panels import (
    FleetStatus,
    StatusPanelWidget,
    LogPanelWidget,
    HeaderWidget,
)

__all__ = [
    # Base widgets
    'BaseWidget',
    'ThemedButton',
    'ThemedEntry',
    'ThemedLabel',
    'ThemedLabelFrame',
    'ExpandablePanel',
    'StatusIndicator',
    'ScrollableFrame',
    'FormRow',
    
    # Car panel
    'CarState',
    'CarPanelCallbacks',
    'CarPanelWidget',
    'TelemetryDisplay',
    'ControlButtons',
    'ManualControlPanel',
    'VelocityControl',
    'PathControl',
    'PlatoonControl',
    
    # Fleet controls
    'FleetControlCallbacks',
    'FleetControlsWidget',
    
    # Status panels
    'FleetStatus',
    'StatusPanelWidget',
    'LogPanelWidget',
    'HeaderWidget',
]
