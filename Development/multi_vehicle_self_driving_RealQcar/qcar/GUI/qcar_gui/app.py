"""
Main Application for QCar Fleet Controller.

This module contains the main application class that orchestrates
all components of the fleet controller GUI.
"""

import tkinter as tk
from tkinter import messagebox, ttk
import threading
import time
from typing import Dict, Optional, Set, Any
from dataclasses import dataclass

from .config import AppConfig, VehicleConfig
from .theme import Theme, DEFAULT_THEME
from .widgets import (
    CarState,
    CarPanelCallbacks,
    CarPanelWidget,
    FleetControlCallbacks,
    FleetControlsWidget,
    FleetStatus,
    StatusPanelWidget,
    LogPanelWidget,
    HeaderWidget,
    ScrollableFrame,
    ThemedLabel,
    VehicleConnectionConfig,
    ConnectionCallbacks,
    VehicleConnectionPanel,
    FleetConnectionPanel,
)
from .controllers import (
    QCarRemoteController,
    ManualInputController,
    VehicleConnector,
    RemoteConfig,
    GroundStationConfig,
)
from .config_loader import get_available_config

from .app_components import DeploymentMixin, FleetCommandsMixin, VehicleCommandsMixin


@dataclass
class PlatoonConfig:
    """Configuration for platoon formation."""

    formation: Dict[int, int] = None  # car_id -> position
    leader_id: Optional[int] = None
    setup_complete: bool = False


class QCarFleetController(DeploymentMixin, FleetCommandsMixin, VehicleCommandsMixin):
    """
    Main application class for QCar Fleet Controller.

    This class manages the GUI, vehicle connections, and all user interactions.
    """

    def __init__(self, root: tk.Tk, config: AppConfig = None, ws_port: int = 8080):
        """
        Initialize the fleet controller.

        Args:
            root: Tkinter root window
            config: Application configuration
        """
        self.root = root
        self.config = config or AppConfig.default()
        self.theme = DEFAULT_THEME

        # Initialize controllers
        self._remote = QCarRemoteController(
            host_ip=self.config.network.host_ip,
            base_port=self.config.network.base_port,
            telemetry_buffer_size=self.config.network.telemetry_buffer_size,
        )
        self._remote.websocket_port = ws_port
        self._input = ManualInputController(self.config.manual_control)

        # Load dynamic configurations
        self._dynamic_config = get_available_config(controller_mode="sim")

        # Set callbacks
        self._remote.gui_controller = self  # For backward compatibility
        self._remote.set_config_data(self._dynamic_config)

        # Initialize vehicle connector for deployment
        self._vehicle_connector = VehicleConnector(
            remote_config=RemoteConfig(
                username=self.config.deployment.ssh_username,
                password=self.config.deployment.ssh_password,
                remote_path=self.config.deployment.remote_path,
                timeout=self.config.deployment.ssh_timeout,
            ),
            ground_station_config=GroundStationConfig(
                local_ip=self.config.network.host_ip
                if self.config.network.host_ip != "0.0.0.0"
                else "192.168.2.200",
                base_port=self.config.network.base_port,
            ),
            log_callback=lambda msg, lvl: self.log(msg, lvl),
        )

        # State tracking
        self._connected_cars: Set[int] = set()
        self._car_panels: Dict[int, CarPanelWidget] = {}
        self._manual_mode_active: Dict[int, bool] = {}
        self._v2v_status: Dict[int, Dict] = {}
        self._platoon_config = PlatoonConfig()

        # Statistics
        self._commands_sent_gui = 0
        self._commands_failed_gui = 0
        self._start_time = time.time()

        # UI components
        self._header: Optional[HeaderWidget] = None
        self._scrollable: Optional[ScrollableFrame] = None
        self._fleet_controls: Optional[FleetControlsWidget] = None
        self._status_panel: Optional[StatusPanelWidget] = None
        self._log_panel: Optional[LogPanelWidget] = None
        self._no_cars_label: Optional[tk.Label] = None

        # Deployment tab components
        self._notebook: Optional[ttk.Notebook] = None
        self._deployment_panels: Dict[int, VehicleConnectionPanel] = {}
        self._deployment_scrollable: Optional[ScrollableFrame] = None

        # Threading
        self._running = True
        self._update_thread: Optional[threading.Thread] = None

        # Setup
        self._setup_window()
        self._build_gui()
        self._start_server()
        self._start_update_loop()

        # Bind close handler
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

    # ========== Setup Methods ==========

    def _setup_window(self) -> None:
        """Configure the main window."""
        gui_cfg = self.config.gui

        self.root.title(gui_cfg.window_title)
        self.root.geometry(f"{gui_cfg.window_width}x{gui_cfg.window_height}")
        self.theme.apply_to_root(self.root)
        self.theme.configure_ttk_styles()

    def _build_gui(self) -> None:
        """Build the main GUI layout."""
        c = self.theme.colors

        # Header
        self._header = HeaderWidget(
            self.root, title="QCar Fleet Controller", theme=self.theme
        )
        self._header.pack(fill="x")

        # Main content area
        main_frame = tk.Frame(self.root, bg=c.bg_dark)
        main_frame.pack(fill="both", expand=True, padx=5, pady=5)

        # Left panel - Car controls with tabs

        
        left_panel = tk.Frame(main_frame, bg=c.bg_dark)
        left_panel.pack(side="left", fill="both", expand=True, padx=(0, 5))

        # Create notebook (tabs) for switching between connected vehicles and deployment
        style = ttk.Style()
        style.configure("Dark.TNotebook", background=c.bg_dark)
        style.configure(
            "Dark.TNotebook.Tab",
            background=c.bg_medium,
            foreground=c.fg_primary,
            padding=[8, 4],
        )
        style.map(
            "Dark.TNotebook.Tab",
            background=[("selected", c.bg_light)],
            foreground=[("selected", c.fg_primary)],
        )

        self._notebook = ttk.Notebook(left_panel, style="Dark.TNotebook")
        self._notebook.pack(fill="both", expand=True, pady=(10, 0))

        # Tab 1: Connected Vehicles
        connected_tab = tk.Frame(self._notebook, bg=c.bg_dark)
        self._notebook.add(connected_tab, text="📡 Connected Vehicles")

        # Scrollable car panels in connected tab
        self._scrollable = ScrollableFrame(connected_tab, theme=self.theme)
        self._scrollable.pack(fill="both", expand=True)

        self._show_waiting_message()

        # Tab 2: Deploy Vehicles
        deploy_tab = tk.Frame(self._notebook, bg=c.bg_dark)
        self._notebook.add(deploy_tab, text="🚀 Deploy Vehicles")

        # Build deployment tab content from DeploymentMixin
        self._build_deployment_tab(deploy_tab)

        # Fleet controls
        fleet_callbacks = FleetControlCallbacks(
            on_start_all=self._start_all_cars,
            on_stop_all=self._stop_all_cars,
            on_setup_platoon=self._setup_platoon,
            on_trigger_platoon=self._trigger_platoon,
            on_disable_all_platoons=self._disable_all_platoons,
            on_activate_v2v=self._activate_v2v,
            on_disable_v2v=self._disable_v2v,
            on_activate_perception=self._activate_perception,
            on_disable_perception=self._disable_perception,
        )

        self._fleet_controls = FleetControlsWidget(
            left_panel, callbacks=fleet_callbacks, theme=self.theme
        )
        self._fleet_controls.pack(fill="x", pady=(10, 5))

        # Right panel - Status and log
        right_panel = tk.Frame(main_frame, bg=c.bg_dark, width=400)
        right_panel.pack(side="right", fill="both", padx=(5, 0))
        right_panel.pack_propagate(False)

        # Status panel
        self._status_panel = StatusPanelWidget(right_panel, theme=self.theme)
        self._status_panel.pack(fill="x", pady=(0, 10))

        # Log panel
        self._log_panel = LogPanelWidget(right_panel, theme=self.theme)
        self._log_panel.pack(fill="both", expand=True)

        # Bind keyboard for manual control
        self._input.bind_keyboard(self.root)

    def _show_waiting_message(self) -> None:
        """Show waiting for vehicles message."""
        if self._scrollable:
            self._no_cars_label = ThemedLabel(
                self._scrollable.content,
                text="⏳ Waiting for vehicles to connect...\n\nNo vehicles currently connected",
                style="muted",
                theme=self.theme,
                justify="center",
                pady=50,
            )
            self._no_cars_label.pack(fill="both", expand=True)

    def _hide_waiting_message(self) -> None:
        """Hide waiting for vehicles message."""
        if self._no_cars_label:
            self._no_cars_label.destroy()
            self._no_cars_label = None

    def _start_server(self) -> None:
        """Start the remote controller server."""
        self._remote.start_server(self.config.gui.default_num_cars)
        self.log(
            f"Server started on {self.config.network.host_ip}:{self.config.network.base_port}",
            "SUCCESS",
        )

    def _start_update_loop(self) -> None:
        """Start the background update loop."""
        self._update_loop()

    # ========== Car Panel Management ==========

    def _on_car_disconnected(self, car_id: int) -> None:
        """Handle car disconnection - clear all state associated with this car."""
        if car_id in self._manual_mode_active:
            del self._manual_mode_active[car_id]
            if hasattr(self, "_input") and self._input._active_car_id == car_id:
                self._input.stop()

        if car_id in self._v2v_status:
            del self._v2v_status[car_id]

        if hasattr(self, "_probing_processes") and car_id in self._probing_processes:
            self._stop_probing_car(car_id)

        if self._platoon_config.formation and car_id in self._platoon_config.formation:
            del self._platoon_config.formation[car_id]
            if car_id == self._platoon_config.leader_id:
                self._platoon_config.leader_id = None
                self._platoon_config.setup_complete = False
                self.log(
                    f"Platoon leader (Car {car_id}) disconnected - Platoon setup invalidated",
                    "WARNING",
                )

        self.log(f"Car {car_id} disconnected - Cleared all associated state", "WARNING")

    def _create_car_panel_callbacks(self, car_id: int) -> CarPanelCallbacks:
        """Create callbacks for a car panel."""
        return CarPanelCallbacks(
            on_start=self._start_car,
            on_stop=self._stop_car,
            on_calibrate=self._calibrate_car,
            on_set_velocity=self._set_velocity,
            on_set_path=self._set_path,
            on_set_initial_position=self._set_initial_position,
            on_toggle_manual=self._toggle_manual_mode,
            on_update_control_type=self._update_control_type,
            on_platoon_position_change=self._update_platoon_position,
            on_toggle_perception=self._toggle_perception_car,
            on_toggle_probing=self._toggle_probing_car,
            on_toggle_scopes=self._toggle_scopes_car,
            on_toggle_remote_plot_local=self._toggle_remote_plot_local,
            on_toggle_remote_plot_fleet=self._toggle_remote_plot_fleet,
            on_set_local_observer=self._set_local_observer,
            on_set_fleet_observer=self._set_fleet_observer,
            on_set_controller=self._set_controller,
            on_set_controller_params=self._set_controller_params,
            on_set_online_sysid=self._set_online_sysid,
            on_set_robust_kalmannet_dataset=self._set_robust_kalmannet_dataset,
            on_start_online_calibration=self._start_online_calibration,
            on_stop_online_calibration=self._stop_online_calibration,
            on_trigger_online_analysis=self._trigger_online_analysis,
            on_clear_online_calibration=self._clear_online_calibration,
            on_trigger_active_calibration=self._trigger_active_calibration,
            on_set_gear=self._set_gear_car,
            on_enable_taxi_mode=self._enable_taxi_mode,
            on_disable_taxi_mode=self._disable_taxi_mode,
            on_set_taxi_trip=self._set_taxi_trip,
        )

    def _update_car_panels(self) -> None:
        """Update car panels based on connection status."""
        current_connected = set()
        for car_id in range(self.config.gui.default_num_cars):
            if self._remote.is_car_connected(car_id):
                current_connected.add(car_id)

        for car_id in list(self._car_panels.keys()):
            if car_id not in current_connected:
                self._car_panels[car_id].destroy()
                del self._car_panels[car_id]
                self.log(f"Car {car_id} disconnected - Panel removed", "WARNING")

        for car_id in current_connected:
            if car_id not in self._car_panels:
                self._hide_waiting_message()
                callbacks = self._create_car_panel_callbacks(car_id)
                panel = CarPanelWidget(
                    self._scrollable.content,
                    car_id=car_id,
                    callbacks=callbacks,
                    config=self._dynamic_config,
                    theme=self.theme,
                )
                panel.pack(fill="x", pady=(0, 15))
                self._car_panels[car_id] = panel
                self.log(f"Car {car_id} connected - New panel created", "SUCCESS")

        if not current_connected:
            if not self._no_cars_label:
                self._show_waiting_message()
            if self._connected_cars:
                self._cleanup_global_state()

        self._connected_cars = current_connected

    def _cleanup_global_state(self) -> None:
        """Clean up all global state when no cars are connected."""
        self.log("No cars connected - Cleaning up all global state", "WARNING")
        self._platoon_config = PlatoonConfig()
        self._v2v_status.clear()

    def _update_car_states(self) -> None:
        """Update car panel states from telemetry."""
        for car_id, panel in self._car_panels.items():
            telemetry = self._remote.get_telemetry(car_id)
            if telemetry:
                state = CarState(
                    car_id=car_id,
                    connected=True,
                    state=telemetry.get("state", "Unknown"),
                    position=(telemetry.get("x", 0), telemetry.get("y", 0)),
                    velocity=telemetry.get("v", 0),
                    heading=telemetry.get("th", 0),
                    throttle=telemetry.get("u", 0),
                    steering=telemetry.get("delta", 0),
                    v2v_active=telemetry.get("v2v_active", False),
                    v2v_peers=telemetry.get("v2v_peers", 0),
                    platoon_enabled=telemetry.get("platoon_enabled", False),
                    platoon_is_leader=telemetry.get("platoon_is_leader", False),
                    platoon_position=telemetry.get("platoon_position"),
                    manual_mode=self._manual_mode_active.get(car_id, False),
                    perception_active=telemetry.get("perception_active", False),
                    scopes_active=telemetry.get("scopes_active", False),
                    local_observer_type=telemetry.get("local_observer_type", "unknown"),
                    fleet_observer_type=telemetry.get("fleet_observer_type", "unknown"),
                    path_long_ctrl=telemetry.get("path_long_ctrl", "unknown"),
                    path_lat_ctrl=telemetry.get("path_lat_ctrl", "unknown"),
                    leader_long_ctrl=telemetry.get("leader_long_ctrl", "unknown"),
                    leader_lat_ctrl=telemetry.get("leader_lat_ctrl", "unknown"),
                    gear=telemetry.get("operational_status", {}).get("gear", "DRIVE_1"),
                    online_sysid_status=telemetry.get("online_sysid_status", {}),
                    online_calibration_status=telemetry.get("online_calibration_status", {}),
                    robust_kalmannet_dataset_status=telemetry.get(
                        "robust_kalmannet_dataset_status", {}
                    ),
                )
                panel.update_state(state)

        self._check_fleet_plot_availability()

    def _check_fleet_plot_availability(self) -> None:
        """Check if fleet plotting should be enabled (all connected cars have V2V)."""
        if not self._connected_cars or len(self._connected_cars) < 2:
            all_v2v_active = False
        else:
            all_v2v_active = True
            for car_id in self._connected_cars:
                telemetry = self._remote.get_telemetry(car_id)
                if not telemetry or not telemetry.get("v2v_active", False):
                    all_v2v_active = False
                    break

        for panel in self._car_panels.values():
            if hasattr(panel, "_scopes_control"):
                panel._scopes_control.set_fleet_button_enabled(all_v2v_active)

    def on_close(self) -> None:
        """Cleanup resources on application close."""
        self.log("Shutting down...", "INFO")
        self._running = False

        if hasattr(self, "_probing_processes"):
            for car_id in list(self._probing_processes.keys()):
                self._stop_probing_car(car_id)

        self._stop_all_cars()

        if self._remote:
            self._remote.stop()

    def _update_loop(self) -> None:
        """UI update loop."""
        if not self._running:
            return

        update_interval_ms = int(1000.0 / self.config.gui.update_rate_hz)

        try:
            self._update_car_panels()
            self._update_car_states()
            self._update_status()
            self._update_header_stats()
        except Exception as e:
            print(f"Update loop error: {e}")

        if self._running and hasattr(self, 'root'):
            self.root.after(update_interval_ms, self._update_loop)

    def _update_status(self) -> None:
        """Update status panel."""
        if not self._status_panel:
            return

        fleet = self._remote.get_fleet_status()

        status = FleetStatus(
            total_cars=self.config.gui.default_num_cars,
            connected_cars=len(self._connected_cars),
            commands_sent=self._commands_sent_gui + fleet["commands_sent_total"],
            commands_failed=self._commands_failed_gui + fleet["commands_failed_total"],
            success_rate=fleet["success_rate"],
            uptime_seconds=time.time() - self._start_time,
            avg_telemetry_rate=fleet["avg_telemetry_rate_hz"],
            host_ip=self.config.network.host_ip,
            base_port=self.config.network.base_port,
        )
        self._status_panel.update(status)

    def _update_header_stats(self) -> None:
        """Update header statistics."""
        if not self._header:
            return

        fleet = self._remote.get_fleet_status()
        self._header.update_stats(
            commands_sent=self._commands_sent_gui + fleet["commands_sent_total"],
            commands_failed=self._commands_failed_gui + fleet["commands_failed_total"],
            uptime=time.time() - self._start_time,
            telemetry_rate=fleet["avg_telemetry_rate_hz"],
        )

    def log(self, message: str, level: str = "INFO") -> None:
        """Log a message safely from any thread."""
        log_panel = getattr(self, "_log_panel", None)
        if log_panel and hasattr(self, 'root'):
            self.root.after(0, lambda: log_panel.log(message, level))

    def _on_closing(self) -> None:
        """Handle window close."""
        self._running = False
        for car_id in list(self._manual_mode_active.keys()):
            if self._manual_mode_active.get(car_id):
                self._remote.disable_manual_mode(car_id)

        self._input.cleanup()
        if self._vehicle_connector:
            self._vehicle_connector.cleanup()

        self._remote.close()
        self.root.destroy()


def create_app(
    num_cars: int = 5,
    host_ip: str = "0.0.0.0",
    base_port: int = 5000,
    ws_port: int = 8080,
) -> QCarFleetController:
    """
    Factory function to create the application.
    """
    from .config import NetworkConfig, GUIConfig, AppConfig

    root = tk.Tk()

    config = AppConfig(
        gui=GUIConfig(default_num_cars=num_cars),
        network=NetworkConfig(host_ip=host_ip, base_port=base_port),
    )

    app = QCarFleetController(root, config, ws_port=ws_port)
    return app


def main():
    """Main entry point."""
    NUM_CARS = 5
    HOST_IP = "0.0.0.0"
    BASE_PORT = 5000

    app = create_app(num_cars=NUM_CARS, host_ip=HOST_IP, base_port=BASE_PORT)

    app.log("QCar Fleet Controller started", "SUCCESS")
    app.log(f"Listening on ports {BASE_PORT}-{BASE_PORT + NUM_CARS - 1}", "INFO")
    app.log(
        "Features: Command validation, platoon control, V2V, manual control", "INFO"
    )

    app.root.mainloop()


if __name__ == "__main__":
    main()
