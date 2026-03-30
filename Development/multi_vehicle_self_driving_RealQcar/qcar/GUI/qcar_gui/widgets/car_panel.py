"""
Car Panel Widget for QCar Fleet Controller.

This module contains the CarPanelWidget class that displays
individual vehicle controls, telemetry, and status information.
"""

import tkinter as tk
from typing import Optional

from ..theme import Theme
from .base import BaseWidget, ExpandablePanel, StatusIndicator

from .car_components import (
    CarState,
    CarPanelCallbacks,
    TelemetryDisplay,
    ControlButtons,
    ManualAndVelocityControl,
    NavigationControl,
    PerceptionControl,
    ScopesControl,
    PlatoonControl,
    ControllerTuningControl,
    RuntimeSwitchingControl,
    OnlineSysidControl,
    CalibrationControl,
    RobustDatasetControl,
)

class CarPanelWidget(BaseWidget):
    """Complete car control panel widget."""

    def __init__(
        self,
        parent: tk.Widget,
        car_id: int,
        callbacks: CarPanelCallbacks = None,
        config: dict = None,
        expanded: bool = True,
        theme: Theme = None,
    ):
        self.car_id = car_id
        self.callbacks = callbacks or CarPanelCallbacks()
        self.config = config or {}
        self._expanded = expanded

        # Child widgets
        self._expandable: Optional[ExpandablePanel] = None
        self._telemetry: Optional[TelemetryDisplay] = None
        self._control_buttons: Optional[ControlButtons] = None
        
        self._manual_and_velocity: Optional[ManualAndVelocityControl] = None
        self._navigation: Optional[NavigationControl] = None
        
        self._perception_control: Optional[PerceptionControl] = None
        self._scopes_control: Optional[ScopesControl] = None
        self._runtime_switching: Optional[RuntimeSwitchingControl] = None
        
        self._platoon_control: Optional[PlatoonControl] = None
        self._sysid_control: Optional[OnlineSysidControl] = None
        self._calibration_control: Optional[CalibrationControl] = None
        self._robust_dataset_control: Optional[RobustDatasetControl] = None
        self._tuning_control: Optional[ControllerTuningControl] = None

        # Status indicators
        self._conn_indicator: Optional[StatusIndicator] = None
        self._v2v_indicator: Optional[tk.Label] = None
        self._platoon_indicator: Optional[tk.Label] = None
        self._state_label: Optional[tk.Label] = None

        super().__init__(parent, theme)

    def _build(self) -> None:
        """Build the car panel widget."""
        c = self.theme.colors

        # Extract vehicle type from config (e.g. Qcar, Limo)
        vehicle_type = self.config.get("vehicle_type", "Car")
        if vehicle_type.lower() == "qcar":
            vehicle_type = "QCar"
            
        # Create expandable panel
        self._expandable = ExpandablePanel(
            self.parent,
            title=f"🚗 {vehicle_type} {self.car_id}",
            expanded=self._expanded,
            theme=self.theme,
        )
        self.frame = self._expandable.frame

        # Add state label to header
        self._build_header_indicators()

        # Build content sections
        self._build_content()

    def _build_header_indicators(self) -> None:
        """Build header status indicators."""
        c = self.theme.colors
        header = self._expandable.header

        # Connection indicator
        self._conn_indicator = StatusIndicator(
            header,
            status="connected",
            theme=self.theme,
            font=self.theme.fonts.heading(),
            padx=12,
        )
        self._conn_indicator.pack(side="right", padx=12, pady=12)

        # V2V indicator
        self._v2v_indicator = tk.Label(
            header,
            text="📡 V2V: OFF",
            bg=c.bg_header,
            fg=c.fg_muted,
            font=self.theme.fonts.small(),
            padx=8,
        )
        self._v2v_indicator.pack(side="right", padx=(0, 6), pady=12)

        # Platoon indicator
        self._platoon_indicator = tk.Label(
            header,
            text="🚗 Solo",
            bg=c.bg_header,
            fg=c.fg_muted,
            font=self.theme.fonts.small(),
            padx=8,
        )
        self._platoon_indicator.pack(side="right", padx=(0, 6), pady=12)

        # Add state label under title in header
        title_frame = tk.Frame(self._expandable.header, bg=c.bg_header)
        title_frame.pack(side="left", fill="y", expand=True, padx=10)

        self._state_label = tk.Label(
            title_frame,
            text="State: Unknown",
            bg=c.bg_header,
            fg=c.fg_muted,
            font=self.theme.fonts.small(),
            cursor="hand2",
        )
        self._state_label.pack(anchor="w")

    def _build_content(self) -> None:
        """Build the content area."""
        content = self._expandable.content
        left_width = 190
        right_width = 120

        # Main layout - left and right sections
        main_layout = tk.Frame(content, bg=self.theme.colors.bg_medium)
        main_layout.pack(fill="x", pady=(0, 6))

        ### Left section
        left_section = tk.Frame(
            main_layout, bg=self.theme.colors.bg_medium, width=left_width
        )
        left_section.pack(side="left", fill="both", expand=True, padx=(0, 4))

        self._telemetry = TelemetryDisplay(left_section, theme=self.theme)
        self._telemetry.pack(fill="x", pady=(0, 5))

        self._control_buttons = ControlButtons(
            left_section, self.car_id, self.callbacks, theme=self.theme
        )
        self._control_buttons.pack(fill="x", pady=(3, 0))

        # Runtime switching control (observer/controller selection)
        self._runtime_switching = RuntimeSwitchingControl(
            left_section,
            self.car_id,
            self.callbacks,
            config=self.config,
            theme=self.theme,
        )
        self._runtime_switching.pack(fill="x", pady=(5, 0))
        
        self._perception_control = PerceptionControl(
            left_section, self.car_id, self.callbacks, theme=self.theme
        )
        self._perception_control.pack(fill="x", pady=(5, 0))

        # Scopes control (estimation visualization)
        self._scopes_control = ScopesControl(
            left_section, self.car_id, self.callbacks, theme=self.theme
        )
        self._scopes_control.pack(fill="x", pady=(5, 0))

        

        ### Right section
        right_section = tk.Frame(
            main_layout, bg=self.theme.colors.bg_medium, width=right_width
        )
        right_section.pack(side="left", fill="both", expand=True, padx=(4, 0))
        
        # Manual & Velocity combined
        self._manual_and_velocity = ManualAndVelocityControl(
            right_section, self.car_id, self.callbacks, theme=self.theme
        )
        self._manual_and_velocity.pack(fill="x", pady=(0, 4))
        
        # Navigation & Taxi combined
        self._navigation = NavigationControl(
            right_section, self.car_id, self.callbacks, theme=self.theme
        )
        self._navigation.pack(fill="x", pady=(0, 4))

        self._platoon_control = PlatoonControl(
            right_section, self.car_id, self.callbacks, theme=self.theme
        )
        self._platoon_control.pack(fill="x", pady=(0, 4))

        # Controller tuning control
        self._tuning_control = ControllerTuningControl(
            right_section, self.car_id, self.callbacks, theme=self.theme
        )
        self._tuning_control.pack(fill="x", pady=(0, 4))

        # Online SysID control
        self._sysid_control = OnlineSysidControl(
            right_section, self.car_id, self.callbacks, theme=self.theme
        )
        self._sysid_control.pack(fill="x", pady=(5, 0))

        self._robust_dataset_control = RobustDatasetControl(
            right_section, self.car_id, self.callbacks, theme=self.theme
        )
        self._robust_dataset_control.pack(fill="x", pady=(5, 0))

        # Calibration control
        self._calibration_control = CalibrationControl(
            right_section, self.car_id, self.callbacks, theme=self.theme
        )
        self._calibration_control.pack(fill="x")

    def update_state(self, state: CarState) -> None:
        """Update the panel with current car state."""
        # Update telemetry
        if self._telemetry:
            self._telemetry.update(state)

        # Update state label
        if self._state_label:
            self._state_label.config(text=f"State: {state.state}")

        # Update V2V indicator
        if self._v2v_indicator:
            if state.v2v_active and state.v2v_peers > 0:
                self._v2v_indicator.config(
                    text=f"📡 V2V: ON ({state.v2v_peers})",
                    fg=self.theme.colors.accent_green,
                )
            else:
                self._v2v_indicator.config(
                    text="📡 V2V: OFF", fg=self.theme.colors.fg_muted
                )

        # Update platoon indicator
        if self._platoon_indicator:
            if state.platoon_enabled and state.platoon_position is not None:
                if state.platoon_is_leader:
                    self._platoon_indicator.config(
                        text="🚗 LEADER", fg=self.theme.colors.platoon_leader
                    )
                else:
                    self._platoon_indicator.config(
                        text=f"🚗 FOLLOWER-{state.platoon_position}",
                        fg=self.theme.colors.platoon_follower,
                    )
            else:
                self._platoon_indicator.config(
                    text="🚗 Solo", fg=self.theme.colors.fg_muted
                )

        # Update manual mode and gear
        if self._manual_and_velocity:
            self._manual_and_velocity.set_manual_active(state.manual_mode)
            self._manual_and_velocity.set_gear(state.gear)

        # Update perception status
        if self._perception_control:
            self._perception_control.set_perception_active(state.perception_active)

        # Update scopes status
        if self._scopes_control:
            self._scopes_control.set_scopes_active(state.scopes_active)

        # Update Online SysID status
        if self._sysid_control:
            self._sysid_control.update_status(state.online_sysid_status)

        if self._robust_dataset_control:
            self._robust_dataset_control.update_status(
                state.robust_kalmannet_dataset_status
            )

        # Update Calibration status
        if self._calibration_control:
            self._calibration_control.update_status(state.online_calibration_status)

        # Update Taxi Mode status
        if self._navigation:
            self._navigation.set_taxi_mode_active(state.state == "Taxi Mode")

    def set_connected(self, connected: bool) -> None:
        """Update connection status."""
        if self._conn_indicator:
            self._conn_indicator.set_status(
                "connected" if connected else "disconnected"
            )

    def set_manual_keyboard_profile(self, profile: dict) -> None:
        """Update the manual keyboard profile shown in the panel."""
        if self._manual_and_velocity:
            self._manual_and_velocity.set_manual_profile(profile)

    @property
    def platoon_position(self) -> int:
        """Get the configured platoon position."""
        return (
            self._platoon_control.position if self._platoon_control else self.car_id + 1
        )

    @property
    def control_type(self) -> str:
        """Get the selected manual control type."""
        return self._manual_and_velocity.control_type if self._manual_and_velocity else "keyboard"
