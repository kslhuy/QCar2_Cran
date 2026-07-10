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

from .car_panel import CarPanelWidget
from .car_components import CarState, CarPanelCallbacks

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

from .vehicle_connection_panel import (
    VehicleConnectionConfig,
    ConnectionCallbacks,
    VehicleConnectionPanel,
    FleetConnectionPanel,
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
    
    # Fleet controls
    'FleetControlCallbacks',
    'FleetControlsWidget',
    
    # Status panels
    'FleetStatus',
    'StatusPanelWidget',
    'LogPanelWidget',
    'HeaderWidget',
    
    # Vehicle connection
    'VehicleConnectionConfig',
    'ConnectionCallbacks',
    'VehicleConnectionPanel',
    'FleetConnectionPanel',
]
