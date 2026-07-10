"""
QCar Fleet Controller GUI Package.

A modular, production-quality GUI for controlling QCar vehicle fleets.

Example Usage:
    from qcar_gui import create_app, main
    
    # Simple launch
    main()
    
    # Or with custom configuration
    app = create_app(num_cars=5, host_ip='0.0.0.0', base_port=5000)
    app.root.mainloop()
"""

from .config import (
    AppConfig,
    NetworkConfig,
    GUIConfig,
    VehicleConfig,
    ManualControlConfig,
    TELEMETRY_FIELDS,
    DEFAULT_PATHS,
)

from .theme import (
    Theme,
    ColorScheme,
    FontScheme,
    DEFAULT_THEME,
)

from .widgets import (
    # Base widgets
    BaseWidget,
    ThemedButton,
    ThemedEntry,
    ThemedLabel,
    ThemedLabelFrame,
    ExpandablePanel,
    StatusIndicator,
    ScrollableFrame,
    FormRow,
    
    # Car panel
    CarState,
    CarPanelCallbacks,
    CarPanelWidget,
    
    # Fleet controls
    FleetControlCallbacks,
    FleetControlsWidget,
    
    # Status panels
    FleetStatus,
    StatusPanelWidget,
    LogPanelWidget,
    HeaderWidget,
)

from .controllers import (
    QCarRemoteController,
    CommandType,
    CommandValidator,
    CarConnection,
    TelemetryStats,
    ControllerStats,
    ManualInputController,
    KeyboardController,
    SteeringWheelController,
    ManualControlState,
)

from .app import (
    QCarFleetController,
    create_app,
    main,
)


__version__ = '2.0.0'
__author__ = 'QCar Team'


__all__ = [
    # Main application
    'QCarFleetController',
    'create_app',
    'main',
    
    # Configuration
    'AppConfig',
    'NetworkConfig',
    'GUIConfig',
    'VehicleConfig',
    'ManualControlConfig',
    'TELEMETRY_FIELDS',
    'DEFAULT_PATHS',
    
    # Theme
    'Theme',
    'ColorScheme',
    'FontScheme',
    'DEFAULT_THEME',
    
    # Widgets - Base
    'BaseWidget',
    'ThemedButton',
    'ThemedEntry',
    'ThemedLabel',
    'ThemedLabelFrame',
    'ExpandablePanel',
    'StatusIndicator',
    'ScrollableFrame',
    'FormRow',
    
    # Widgets - Car
    'CarState',
    'CarPanelCallbacks',
    'CarPanelWidget',
    
    # Widgets - Fleet
    'FleetControlCallbacks',
    'FleetControlsWidget',
    
    # Widgets - Status
    'FleetStatus',
    'StatusPanelWidget',
    'LogPanelWidget',
    'HeaderWidget',
    
    # Controllers
    'QCarRemoteController',
    'CommandType',
    'CommandValidator',
    'CarConnection',
    'TelemetryStats',
    'ControllerStats',
    'ManualInputController',
    'KeyboardController',
    'SteeringWheelController',
    'ManualControlState',
]
