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


@dataclass
class PlatoonConfig:
    """Configuration for platoon formation."""

    formation: Dict[int, int] = None  # car_id -> position
    leader_id: Optional[int] = None
    setup_complete: bool = False


class QCarFleetController:
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
        # We assume simulation mode for the GUI default unless we know it's a real car.
        # Here we just load the sim config as default, and we can switch it later if needed.
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
        main_frame.pack(fill="both", expand=True, padx=15, pady=10)

        # Left panel - Car controls with tabs
        left_panel = tk.Frame(main_frame, bg=c.bg_dark)
        left_panel.pack(side="left", fill="both", expand=True, padx=(0, 10))

        # Create notebook (tabs) for switching between connected vehicles and deployment
        style = ttk.Style()
        style.configure("Dark.TNotebook", background=c.bg_dark)
        style.configure(
            "Dark.TNotebook.Tab",
            background=c.bg_medium,
            foreground=c.fg_primary,
            padding=[15, 8],
        )
        style.map(
            "Dark.TNotebook.Tab",
            background=[("selected", c.bg_light)],
            foreground=[("selected", c.fg_primary)],
        )

        self._notebook = ttk.Notebook(left_panel, style="Dark.TNotebook")
        self._notebook.pack(fill="both", expand=True, pady=(10, 0))

        # Tab 1: Connected Vehicles (existing functionality)
        connected_tab = tk.Frame(self._notebook, bg=c.bg_dark)
        self._notebook.add(connected_tab, text="📡 Connected Vehicles")

        # Scrollable car panels in connected tab
        self._scrollable = ScrollableFrame(connected_tab, theme=self.theme)
        self._scrollable.pack(fill="both", expand=True)

        # Initial "waiting for cars" message
        self._show_waiting_message()

        # Tab 2: Deploy Vehicles (new functionality)
        deploy_tab = tk.Frame(self._notebook, bg=c.bg_dark)
        self._notebook.add(deploy_tab, text="🚀 Deploy Vehicles")

        # Build deployment tab content
        self._build_deployment_tab(deploy_tab)

        # Fleet controls (below tabs)
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
        right_panel = tk.Frame(main_frame, bg=c.bg_dark, width=450)
        right_panel.pack(side="right", fill="both", padx=(10, 0))
        right_panel.pack_propagate(False)

        # Status panel
        self._status_panel = StatusPanelWidget(right_panel, theme=self.theme)
        self._status_panel.pack(fill="x", pady=(0, 10))

        # Log panel
        self._log_panel = LogPanelWidget(right_panel, theme=self.theme)
        self._log_panel.pack(fill="both", expand=True)

        # Bind keyboard for manual control
        self._input.bind_keyboard(self.root)

    def _build_deployment_tab(self, parent: tk.Frame) -> None:
        """Build the vehicle deployment tab content."""
        c = self.theme.colors
        from .widgets.base import (
            ThemedLabelFrame,
            ThemedButton,
            ThemedEntry,
            ThemedLabel,
        )

        # Master controls frame
        master_frame = ThemedLabelFrame(
            parent, text="🎛️ Fleet Deployment Controls", theme=self.theme
        )
        master_frame.pack(fill="x", padx=10, pady=10)

        master_content = tk.Frame(master_frame, bg=c.bg_medium)
        master_content.pack(fill="x", padx=8, pady=8)

        # Ground Station IP configuration
        gs_row = tk.Frame(master_content, bg=c.bg_medium)
        gs_row.pack(fill="x", pady=(0, 5))

        ThemedLabel(
            gs_row, text="Ground Station IP:", style="muted", theme=self.theme
        ).pack(side="left", padx=(0, 10))
        self._gs_ip_entry = ThemedEntry(gs_row, width=15, theme=self.theme)
        default_ip = (
            self.config.network.host_ip
            if self.config.network.host_ip != "0.0.0.0"
            else "192.168.2.200"
        )
        self._gs_ip_entry.insert(0, default_ip)
        self._gs_ip_entry.pack(side="left", padx=(0, 15))

        ThemedLabel(gs_row, text="Base Port:", style="muted", theme=self.theme).pack(
            side="left", padx=(0, 5)
        )
        self._gs_port_entry = ThemedEntry(gs_row, width=6, theme=self.theme)
        self._gs_port_entry.insert(0, str(self.config.network.base_port))
        self._gs_port_entry.pack(side="left")

        # Batch operations row
        batch_row = tk.Frame(master_content, bg=c.bg_medium)
        batch_row.pack(fill="x", pady=(5, 0))

        ThemedButton(
            batch_row,
            text="➕ Add Vehicle",
            button_type="command",
            command=self._add_deployment_panel,
            padx=10,
            pady=3,
        ).pack(side="left", padx=(0, 5))

        ThemedButton(
            batch_row,
            text="🔌 Connect All",
            button_type="command",
            command=self._connect_all_vehicles,
            padx=10,
            pady=3,
        ).pack(side="left", padx=5)

        ThemedButton(
            batch_row,
            text="📤 Upload All",
            button_type="warning",
            command=self._upload_all_vehicles,
            padx=10,
            pady=3,
        ).pack(side="left", padx=5)

        ThemedButton(
            batch_row,
            text="▶️ Start All",
            button_type="start",
            command=self._start_all_deployed_vehicles,
            padx=10,
            pady=3,
        ).pack(side="left", padx=5)

        ThemedButton(
            batch_row,
            text="⬛ Stop All",
            button_type="stop",
            command=self._stop_all_deployed_vehicles,
            padx=10,
            pady=3,
        ).pack(side="left", padx=5)

        # Scrollable area for vehicle deployment panels
        self._deployment_scrollable = ScrollableFrame(parent, theme=self.theme)
        self._deployment_scrollable.pack(
            fill="both", expand=True, padx=10, pady=(0, 10)
        )

        # Add initial deployment panels (2 vehicles by default)
        for i in range(2):
            self._add_deployment_panel()

    def _add_deployment_panel(self) -> None:
        """Add a new vehicle deployment panel."""
        car_id = len(self._deployment_panels)

        # Create callbacks for this panel
        callbacks = ConnectionCallbacks(
            on_connect=lambda cfg: self._on_vehicle_connect(cfg),
            on_disconnect=lambda cid: self._on_vehicle_disconnect(cid),
            on_upload_files=lambda cfg: self._on_vehicle_upload(cfg),
            on_start_vehicle=lambda cfg: self._on_vehicle_start(cfg),
            on_stop_vehicle=lambda cid, ip, sq, sh: self._on_vehicle_stop(
                cid, ip, sq, sh
            ),
            on_calibrate=lambda cfg, dist: self._on_vehicle_calibrate(cfg, dist),
            on_test_connection=lambda ip: self._on_test_connection(ip),
        )

        # Default config
        default_config = VehicleConnectionConfig(
            car_id=car_id,
            ip=f"{self.config.deployment.default_ip_prefix}{100 + car_id}",
            vehicle_type=self.config.deployment.default_vehicle_type,
            initial_v_ref=self.config.deployment.default_velocity,
        )

        panel = VehicleConnectionPanel(
            self._deployment_scrollable.content,
            car_id=car_id,
            callbacks=callbacks,
            default_config=default_config,
            theme=self.theme,
        )
        panel.pack(fill="x", pady=(0, 10))
        self._deployment_panels[car_id] = panel

        self.log(f"Added deployment slot for Vehicle {car_id}", "INFO")

    # ========== Deployment Callbacks ==========

    def _update_vehicle_connector_config(self) -> None:
        """Update vehicle connector with current Ground Station settings."""
        try:
            gs_ip = self._gs_ip_entry.get().strip()
            gs_port = int(self._gs_port_entry.get().strip())
            self._vehicle_connector.set_ground_station_config(gs_ip, gs_port)
        except (ValueError, AttributeError):
            pass

    def _on_test_connection(self, ip: str) -> None:
        """Handle test connection request."""
        success, message = self._vehicle_connector.test_connection(ip)
        self.log(
            f"Connection test to {ip}: {message}", "SUCCESS" if success else "ERROR"
        )

    def _on_vehicle_connect(self, config: VehicleConnectionConfig) -> None:
        """Handle vehicle connection request."""
        self._update_vehicle_connector_config()

        success, message = self._vehicle_connector.connect(
            config.car_id, config.ip, config.vehicle_type
        )

        # Update panel status (thread-safe)
        if config.car_id in self._deployment_panels:
            panel = self._deployment_panels[config.car_id]
            panel.set_connected(success, message)

    def _on_vehicle_disconnect(self, car_id: int) -> None:
        """Handle vehicle disconnection request."""
        self._vehicle_connector.disconnect(car_id)
        self.log(f"Vehicle {car_id}: Disconnected", "INFO")

    def _on_vehicle_upload(self, config: VehicleConnectionConfig) -> None:
        """Handle file upload request."""
        self._update_vehicle_connector_config()

        # Update progress
        if config.car_id in self._deployment_panels:
            panel = self._deployment_panels[config.car_id]
            panel.set_progress("Uploading files...")

        success, message = self._vehicle_connector.upload_files(
            config.car_id, config.ip, config.vehicle_type
        )

        # Update panel status
        if config.car_id in self._deployment_panels:
            panel = self._deployment_panels[config.car_id]
            panel.set_upload_complete(success, message)

    def _on_vehicle_start(self, config: VehicleConnectionConfig) -> None:
        """Handle vehicle start request."""
        self._update_vehicle_connector_config()

        success, message = self._vehicle_connector.start_vehicle(
            car_id=config.car_id,
            ip=config.ip,
            vehicle_type=config.vehicle_type,
            path_number=config.path_number,
            calibrate=config.calibrate,
            left_hand_traffic=config.left_hand_traffic,
            initial_v_ref=config.initial_v_ref,
            enable_logs=True,
        )

        # Update panel status
        if config.car_id in self._deployment_panels:
            panel = self._deployment_panels[config.car_id]
            panel.set_vehicle_running(success, message)

        if success:
            # Switch to Connected Vehicles tab after successful start
            self.root.after(2000, lambda: self._notebook.select(0))

    def _on_vehicle_stop(
        self, car_id: int, ip: str, stop_quarc: bool = True, stop_hardware: bool = True
    ) -> None:
        """Handle vehicle stop request (enhanced - like stop_enhanced.bat)."""
        self.log(
            f"Car {car_id}: Stopping (QUARC={stop_quarc}, Hardware={stop_hardware})...",
            "INFO",
        )

        success, message = self._vehicle_connector.stop_vehicle(
            car_id, ip, stop_quarc=stop_quarc, stop_hardware=stop_hardware
        )

        # Update panel status
        if car_id in self._deployment_panels:
            panel = self._deployment_panels[car_id]
            panel.set_vehicle_running(False, message)
            # Re-enable stop button
            panel._stop_btn.config(state="normal")

    def _on_vehicle_calibrate(
        self, config: VehicleConnectionConfig, distribute_to_all: bool = True
    ) -> None:
        """Handle vehicle calibration request (LiDAR calibration like calibrate.bat)."""
        car_id = config.car_id
        ip = config.ip

        self.log(
            f"Car {car_id}: Starting LiDAR calibration (distribute={distribute_to_all})...",
            "INFO",
        )

        # Set progress callback
        panel = self._deployment_panels.get(car_id)
        if panel:
            self._vehicle_connector.progress_callback = panel.set_progress

        # Gather other vehicle IPs for distribution (only connected vehicles)
        distribute_ips = []
        if distribute_to_all:
            for other_id, other_panel in self._deployment_panels.items():
                if other_id != car_id and other_panel.is_connected:
                    other_ip = other_panel.get_ip()
                    if other_ip:
                        distribute_ips.append(other_ip)

            if distribute_ips:
                self.log(
                    f"Will distribute calibration results to {len(distribute_ips)} connected vehicles: {distribute_ips}",
                    "INFO",
                )
            else:
                self.log("No other connected vehicles to distribute to", "WARNING")

        # Run calibration
        success, message = self._vehicle_connector.calibrate_vehicle(
            car_id=car_id,
            ip=ip,
            distribute_ips=distribute_ips if distribute_to_all else None,
        )

        # Update panel status
        if panel:
            panel.set_calibration_complete(success, message)
            self._vehicle_connector.progress_callback = None

        if success:
            self.log(f"Car {car_id}: {message}", "SUCCESS")
        else:
            self.log(f"Car {car_id}: Calibration failed - {message}", "ERROR")

    def _connect_all_vehicles(self) -> None:
        """Connect to all vehicles in deployment panels."""
        self._update_vehicle_connector_config()
        self.log("Connecting to all vehicles...", "INFO")

        for car_id, panel in self._deployment_panels.items():
            config = panel._get_current_config()
            if config.ip:
                import threading

                threading.Thread(
                    target=self._on_vehicle_connect, args=(config,), daemon=True
                ).start()

    def _upload_all_vehicles(self) -> None:
        """Upload files to all connected vehicles."""
        self.log("Uploading files to all connected vehicles...", "INFO")

        for car_id, panel in self._deployment_panels.items():
            if panel.is_connected:
                config = panel._get_current_config()
                import threading

                threading.Thread(
                    target=self._on_vehicle_upload, args=(config,), daemon=True
                ).start()

    def _start_all_deployed_vehicles(self) -> None:
        """Start all connected vehicles."""
        self.log("Starting all connected vehicles...", "INFO")

        for car_id, panel in self._deployment_panels.items():
            if panel.is_connected:
                config = panel._get_current_config()
                import threading

                threading.Thread(
                    target=self._on_vehicle_start, args=(config,), daemon=True
                ).start()

    def _stop_all_deployed_vehicles(self) -> None:
        """Stop all running vehicles (with QUARC and hardware stopping)."""
        self.log("Stopping all deployed vehicles...", "INFO")

        for car_id, panel in self._deployment_panels.items():
            if panel.is_running:
                ip = panel.get_ip()
                # Get stop options from panel checkboxes
                stop_quarc = (
                    panel._stop_quarc_var.get()
                    if hasattr(panel, "_stop_quarc_var")
                    else True
                )
                stop_hardware = (
                    panel._stop_hardware_var.get()
                    if hasattr(panel, "_stop_hardware_var")
                    else True
                )
                import threading

                threading.Thread(
                    target=self._on_vehicle_stop,
                    args=(car_id, ip, stop_quarc, stop_hardware),
                    daemon=True,
                ).start()

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

    # ========== Deployment Tab Methods ==========

    def _build_deployment_tab(self, parent: tk.Frame) -> None:
        """Build the vehicle deployment tab content."""
        from .widgets.base import ThemedButton, ThemedLabelFrame

        c = self.theme.colors

        # Master controls frame
        master_frame = ThemedLabelFrame(
            parent, text="🎛️ Deployment Controls", theme=self.theme
        )
        master_frame.pack(fill="x", padx=10, pady=10)

        master_content = tk.Frame(master_frame, bg=c.bg_medium)
        master_content.pack(fill="x", padx=8, pady=8)

        # Info row
        info_row = tk.Frame(master_content, bg=c.bg_medium)
        info_row.pack(fill="x", pady=(0, 5))

        self._deployment_count_label = tk.Label(
            info_row,
            text=f"Vehicles: {self.config.gui.default_num_cars}",
            bg=c.bg_medium,
            fg=c.fg_primary,
            font=self.theme.fonts.body(),
        )
        self._deployment_count_label.pack(side="left", padx=(0, 20))

        ThemedButton(
            info_row,
            text="➕ Add Vehicle",
            button_type="command",
            command=self._add_deployment_vehicle,
            padx=10,
            pady=3,
        ).pack(side="left", padx=(0, 5))

        ThemedButton(
            info_row,
            text="➖ Remove Last",
            button_type="warning",
            command=self._remove_deployment_vehicle,
            padx=10,
            pady=3,
        ).pack(side="left")

        # Batch operation buttons
        batch_row = tk.Frame(master_content, bg=c.bg_medium)
        batch_row.pack(fill="x", pady=(5, 0))

        ThemedButton(
            batch_row,
            text="🔌 Connect All",
            button_type="command",
            command=self._connect_all_vehicles,
            padx=12,
            pady=4,
        ).pack(side="left", expand=True, fill="x", padx=(0, 3))

        ThemedButton(
            batch_row,
            text="📤 Upload All",
            button_type="warning",
            command=self._upload_all_vehicles,
            padx=12,
            pady=4,
        ).pack(side="left", expand=True, fill="x", padx=3)

        ThemedButton(
            batch_row,
            text="▶️ Start All",
            button_type="start",
            command=self._start_all_deployed_vehicles,
            padx=12,
            pady=4,
        ).pack(side="left", expand=True, fill="x", padx=3)

        ThemedButton(
            batch_row,
            text="⬛ Stop All",
            button_type="stop",
            command=self._stop_all_deployed_vehicles,
            padx=12,
            pady=4,
        ).pack(side="left", expand=True, fill="x", padx=(3, 0))

        # Scrollable area for vehicle connection panels
        self._deployment_scrollable = ScrollableFrame(parent, theme=self.theme)
        self._deployment_scrollable.pack(
            fill="both", expand=True, padx=10, pady=(0, 10)
        )

        # Create initial vehicle connection panels
        for i in range(self.config.gui.default_num_cars):
            self._add_deployment_panel(i)

    def _add_deployment_panel(self, car_id: int) -> None:
        """Add a vehicle deployment panel."""
        callbacks = ConnectionCallbacks(
            on_connect=lambda cfg: self._on_vehicle_connect(cfg),
            on_disconnect=lambda cid: self._on_vehicle_disconnect(cid),
            on_upload_files=lambda cfg: self._on_vehicle_upload(cfg),
            on_start_vehicle=lambda cfg: self._on_vehicle_start(cfg),
            on_stop_vehicle=lambda cid, ip, sq, sh: self._on_vehicle_stop(
                cid, ip, sq, sh
            ),
            on_calibrate=lambda cfg, dist: self._on_vehicle_calibrate(cfg, dist),
            on_test_connection=lambda ip: self._on_test_connection(ip, car_id),
        )

        # Default config with IP
        default_config = VehicleConnectionConfig(
            car_id=car_id,
            ip=f"{self.config.deployment.default_ip_prefix}{100 + car_id}",
            vehicle_type=self.config.deployment.default_vehicle_type,
            initial_v_ref=self.config.deployment.default_velocity,
        )

        panel = VehicleConnectionPanel(
            self._deployment_scrollable.content,
            car_id=car_id,
            callbacks=callbacks,
            default_config=default_config,
            theme=self.theme,
        )
        panel.pack(fill="x", pady=(0, 10))
        self._deployment_panels[car_id] = panel

    def _add_deployment_vehicle(self) -> None:
        """Add a new vehicle deployment slot."""
        new_id = len(self._deployment_panels)
        self._add_deployment_panel(new_id)
        self._update_deployment_count()
        self.log(f"Added Vehicle {new_id} deployment slot", "INFO")

    def _remove_deployment_vehicle(self) -> None:
        """Remove the last vehicle deployment slot."""
        if len(self._deployment_panels) > 1:
            last_id = len(self._deployment_panels) - 1
            if last_id in self._deployment_panels:
                self._deployment_panels[last_id].destroy()
                del self._deployment_panels[last_id]
            self._update_deployment_count()
            self.log(f"Removed Vehicle {last_id} deployment slot", "INFO")

    def _update_deployment_count(self) -> None:
        """Update the deployment count label."""
        if hasattr(self, "_deployment_count_label"):
            self._deployment_count_label.config(
                text=f"Vehicles: {len(self._deployment_panels)}"
            )

    # ========== Vehicle Connection Callbacks ==========

    def _on_test_connection(self, ip: str, car_id: int) -> None:
        """Handle test connection request."""
        panel = self._deployment_panels.get(car_id)
        vehicle_type = None
        if panel:
            vehicle_type = panel._get_current_config().vehicle_type

        success, message = self._vehicle_connector.test_connection(
            ip, car_id=car_id, vehicle_type=vehicle_type
        )

        if panel:
            panel.set_connected(success, message)

    def _on_vehicle_connect(self, config: VehicleConnectionConfig) -> None:
        """Handle vehicle connect request."""
        success, message = self._vehicle_connector.connect(
            config.car_id, config.ip, config.vehicle_type
        )

        panel = self._deployment_panels.get(config.car_id)
        if panel:
            panel.set_connected(success, message)

    def _on_vehicle_disconnect(self, car_id: int) -> None:
        """Handle vehicle disconnect request."""
        self._vehicle_connector.disconnect(car_id)

        panel = self._deployment_panels.get(car_id)
        if panel:
            panel.set_connected(False, "Disconnected")

    def _on_vehicle_upload(self, config: VehicleConnectionConfig) -> None:
        """Handle file upload request."""
        # Set progress callback
        panel = self._deployment_panels.get(config.car_id)
        if panel:
            self._vehicle_connector.progress_callback = panel.set_progress

        success, message = self._vehicle_connector.upload_files(
            config.car_id, config.ip, config.vehicle_type
        )

        if panel:
            panel.set_upload_complete(success, message)
            self._vehicle_connector.progress_callback = None

    def _on_vehicle_start(self, config: VehicleConnectionConfig) -> None:
        """Handle vehicle start request."""
        panel = self._deployment_panels.get(config.car_id)
        if panel:
            self._vehicle_connector.progress_callback = panel.set_progress

        success, message = self._vehicle_connector.start_vehicle(
            car_id=config.car_id,
            ip=config.ip,
            vehicle_type=config.vehicle_type,
            path_number=config.path_number,
            calibrate=config.calibrate,
            left_hand_traffic=config.left_hand_traffic,
            initial_v_ref=config.initial_v_ref,
            enable_logs=True,
        )

        if panel:
            panel.set_vehicle_running(success, message)
            self._vehicle_connector.progress_callback = None

        if success:
            # Switch to Connected Vehicles tab after starting
            self.log(
                f"Vehicle {config.car_id} started - waiting for connection...", "INFO"
            )

    def _on_vehicle_stop(
        self, car_id: int, ip: str, stop_quarc: bool = True, stop_hardware: bool = True
    ) -> None:
        """Handle vehicle stop request (enhanced with QUARC/hardware options)."""
        self.log(
            f"Car {car_id}: Stopping (QUARC={stop_quarc}, Hardware={stop_hardware})...",
            "INFO",
        )

        success, message = self._vehicle_connector.stop_vehicle(
            car_id, ip, stop_quarc=stop_quarc, stop_hardware=stop_hardware
        )

        panel = self._deployment_panels.get(car_id)
        if panel:
            panel.set_vehicle_running(
                False, message if success else "Stop command sent"
            )
            panel._stop_btn.config(state="normal")

    # ========== Batch Deployment Operations ==========

    def _connect_all_vehicles(self) -> None:
        """Connect to all vehicles."""
        self.log("🔌 Connecting to all vehicles...", "INFO")

        for car_id, panel in self._deployment_panels.items():
            config = panel._get_current_config()
            if config.ip:
                panel._on_connect()

    def _upload_all_vehicles(self) -> None:
        """Upload files to all connected vehicles."""
        self.log("📤 Uploading files to all connected vehicles...", "INFO")

        for car_id, panel in self._deployment_panels.items():
            if panel.is_connected:
                panel._on_upload()

    def _start_all_deployed_vehicles(self) -> None:
        """Start all connected vehicles."""
        self.log("▶️ Starting all deployed vehicles...", "INFO")

        for car_id, panel in self._deployment_panels.items():
            if panel.is_connected and not panel.is_running:
                panel._on_start()

    def _stop_all_deployed_vehicles(self) -> None:
        """Stop all running vehicles."""
        self.log("⬛ Stopping all deployed vehicles...", "INFO")

        for car_id, panel in self._deployment_panels.items():
            if panel.is_running:
                panel._on_stop()
        self.log(
            f"Waiting for {self.config.gui.default_num_cars} cars to connect...", "INFO"
        )

    def _start_update_loop(self) -> None:
        """Start the background update loop."""
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()

    # ========== Car Panel Management ==========

    def _on_car_disconnected(self, car_id: int) -> None:
        """Handle car disconnection - clear all state associated with this car."""
        # 1. Clear manual mode state
        if car_id in self._manual_mode_active:
            del self._manual_mode_active[car_id]
            # Stop manual input loop if this car was being controlled
            if hasattr(self, "_input") and self._input._active_car_id == car_id:
                self._input.stop()

        # 2. Clear V2V status
        if car_id in self._v2v_status:
            del self._v2v_status[car_id]

        # 3. Stop probing if active
        if hasattr(self, "_probing_processes") and car_id in self._probing_processes:
            self._stop_probing_car(car_id)

        # 4. Remove from platoon configuration
        if self._platoon_config.formation and car_id in self._platoon_config.formation:
            del self._platoon_config.formation[car_id]
            # If this car was crucial (leader), invalidate setup
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
            # Runtime switching callbacks
            on_set_local_observer=self._set_local_observer,
            on_set_fleet_observer=self._set_fleet_observer,
            on_set_controller=self._set_controller,
            on_set_controller_params=self._set_controller_params,
            on_set_online_sysid=self._set_online_sysid,
            on_set_gear=self._set_gear_car,
            on_enable_taxi_mode=self._enable_taxi_mode,
            on_disable_taxi_mode=self._disable_taxi_mode,
            on_set_taxi_trip=self._set_taxi_trip,
        )

    def _update_car_panels(self) -> None:
        """Update car panels based on connection status."""
        # Get currently connected cars
        current_connected = set()
        for car_id in range(self.config.gui.default_num_cars):
            if self._remote.is_car_connected(car_id):
                current_connected.add(car_id)

        # Remove panels for disconnected cars
        for car_id in list(self._car_panels.keys()):
            if car_id not in current_connected:
                self._car_panels[car_id].destroy()
                del self._car_panels[car_id]
                self.log(f"Car {car_id} disconnected - Panel removed", "WARNING")

        # Add panels for newly connected cars
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

        # Show waiting message if no cars connected
        if not current_connected:
            if not self._no_cars_label:
                self._show_waiting_message()

            # Global cleanup when NO cars are connected
            if self._connected_cars:  # If we HAD cars before
                self._cleanup_global_state()

        self._connected_cars = current_connected

    def _cleanup_global_state(self) -> None:
        """Clean up all global state when no cars are connected."""
        self.log("No cars connected - Cleaning up all global state", "WARNING")

        # Clear platoon config
        self._platoon_config = PlatoonConfig()

        # Clear V2V status
        self._v2v_status.clear()

        # Reset any fleet control buttons if needed
        if self._fleet_controls:
            # Assuming fleet controls might need resetting, but they mostly trigger actions
            pass

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
                    # Observer and Controller types
                    local_observer_type=telemetry.get("local_observer_type", "unknown"),
                    fleet_observer_type=telemetry.get("fleet_observer_type", "unknown"),
                    path_long_ctrl=telemetry.get("path_long_ctrl", "unknown"),
                    path_lat_ctrl=telemetry.get("path_lat_ctrl", "unknown"),
                    leader_long_ctrl=telemetry.get("leader_long_ctrl", "unknown"),
                    leader_lat_ctrl=telemetry.get("leader_lat_ctrl", "unknown"),
                    gear=telemetry.get("operational_status", {}).get("gear", "DRIVE_1"),
                    online_sysid_status=telemetry.get("online_sysid_status", {}),
                )

                # Check stream status to keep button in sync
                is_streaming_local = self._remote.is_scope_streaming(car_id)
                # Note: We can't distinguish local/fleet stream easily from just boolean,
                # but we can check the presets if needed. For now simpler check:

                # Update panel
                panel.update_state(state)

        # Check fleet plot availability (needs V2V active on all vehicles)
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

        # Enable/disable fleet plot button for all panels
        for panel in self._car_panels.values():
            if hasattr(panel, "_scopes_control"):
                panel._scopes_control.set_fleet_button_enabled(all_v2v_active)

    def on_close(self) -> None:
        """Cleanup resources on application close."""
        self.log("Shutting down...", "INFO")
        self._running = False

        # Stop all probing processes
        if hasattr(self, "_probing_processes"):
            for car_id in list(self._probing_processes.keys()):
                self._stop_probing_car(car_id)

        # Stop all cars
        self._stop_all_cars()

        # Stop remote controller (closes sockets and viewers)
        if self._remote:
            self._remote.stop()

    # ========== Individual Car Commands ==========

    def _start_car(self, car_id: int) -> None:
        """Start a car."""
        if self._remote.start_car(car_id):
            self._commands_sent_gui += 1
            self.log(f"✅ Started Car {car_id}", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Failed to start Car {car_id}", "ERROR")

    def _stop_car(self, car_id: int) -> None:
        """Stop a car."""
        if self._remote.stop_car(car_id):
            self._commands_sent_gui += 1
            self.log(f"🛑 Stopped Car {car_id}", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Failed to stop Car {car_id}", "ERROR")

    def _calibrate_car(self, car_id: int) -> None:
        """Calibrate GPS for a car."""
        if self._remote.send_command(car_id, {"type": "calibrate"}):
            self._commands_sent_gui += 1
            self.log(f"📍 GPS Calibration started for Car {car_id}", "INFO")
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Failed to calibrate Car {car_id}", "ERROR")

    def _set_velocity(self, car_id: int, velocity: float) -> None:
        """Set velocity for a car."""
        cfg = self.config.vehicle
        if cfg.min_velocity <= velocity <= cfg.max_velocity:
            if self._remote.set_velocity(car_id, velocity):
                self._commands_sent_gui += 1
                self.log(
                    f"🎯 Set Car {car_id} velocity to {velocity:.2f} m/s", "SUCCESS"
                )
            else:
                self._commands_failed_gui += 1
                self.log(f"❌ Failed to set velocity for Car {car_id}", "ERROR")
        else:
            self.log(
                f"❌ Invalid velocity {velocity:.2f} (must be {cfg.min_velocity}-{cfg.max_velocity} m/s)",
                "ERROR",
            )

    def _set_gear_car(self, car_id: int, gear: str) -> None:
        """Set gear for a car."""
        if self._remote.set_gear(car_id, gear):
            self._commands_sent_gui += 1
            self.log(f"⚙️ Set Car {car_id} gear to {gear}", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Failed to set gear for Car {car_id}", "ERROR")

    def _set_path(self, car_id: int, nodes: list) -> None:
        """Set path for a car."""
        if len(nodes) >= 2:
            if self._remote.set_path(car_id, nodes):
                self._commands_sent_gui += 1
                self.log(f"🛤️ Set Car {car_id} path: {nodes}", "SUCCESS")
            else:
                self._commands_failed_gui += 1
                self.log(f"❌ Failed to set path for Car {car_id}", "ERROR")
        else:
            self.log("❌ Path must have at least 2 nodes", "ERROR")

    def _set_initial_position(
        self, car_id: int, x: float, y: float, theta: float, calibrate: bool
    ) -> None:
        """Set initial position for a car."""
        if self._remote.set_initial_position(car_id, x, y, theta, calibrate):
            self._commands_sent_gui += 1
            mode = "with GPS calibration" if calibrate else "without GPS calibration"
            self.log(
                f"📍 Set initial position for Car {car_id}: ({x:.2f}, {y:.2f}, θ={theta:.2f}) {mode}",
                "SUCCESS",
            )
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Failed to set initial position for Car {car_id}", "ERROR")

    # ========== Taxi Operations ==========

    def _enable_taxi_mode(self, car_id: int) -> None:
        """Enable taxi mode for a car."""
        if self._remote.enable_taxi_mode(car_id):
            self._commands_sent_gui += 1
            self.log(f"🚕 Car {car_id}: Taxi mode ENABLED", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Car {car_id}: Failed to enable taxi mode", "ERROR")

    def _disable_taxi_mode(self, car_id: int) -> None:
        """Disable taxi mode for a car."""
        if self._remote.disable_taxi_mode(car_id):
            self._commands_sent_gui += 1
            self.log(f"🚕 Car {car_id}: Taxi mode DISABLED", "INFO")
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Car {car_id}: Failed to disable taxi mode", "ERROR")

    def _set_taxi_trip(self, car_id: int, nodes: list) -> None:
        """Set taxi trip for a car."""
        if len(nodes) > 0:
            if self._remote.set_taxi_trip(car_id, nodes):
                self._commands_sent_gui += 1
                self.log(f"🚕 Set Car {car_id} taxi trip to nodes: {nodes}", "SUCCESS")
            else:
                self._commands_failed_gui += 1
                self.log(f"❌ Failed to set taxi trip for Car {car_id}", "ERROR")
        else:
            self.log("❌ Taxi trip must have at least 1 node", "ERROR")

    # ========== Manual Control ==========

    def _toggle_manual_mode(self, car_id: int) -> None:
        """Toggle manual mode for a car."""
        is_active = self._manual_mode_active.get(car_id, False)

        if is_active:
            self._disable_manual_mode(car_id)
        else:
            self._enable_manual_mode(car_id)

    def _enable_manual_mode(self, car_id: int) -> None:
        """Enable manual mode for a car."""
        panel = self._car_panels.get(car_id)
        control_type = panel.control_type if panel else "keyboard"

        if self._remote.enable_manual_mode(car_id, control_type):
            self._manual_mode_active[car_id] = True

            # Set input controller type
            if not self._input.set_control_type(control_type):
                self.log(
                    f"Warning: Could not initialize {control_type} controller",
                    "WARNING",
                )

            # Start input loop
            self._input.start(car_id, self._send_manual_control)

            self._commands_sent_gui += 1
            self.log(
                f"🎮 Car {car_id}: Manual mode ENABLED ({control_type.upper()})",
                "SUCCESS",
            )
            self.log(self._input.get_help_text(), "INFO")
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Car {car_id}: Failed to enable manual mode", "ERROR")

    def _disable_manual_mode(self, car_id: int) -> None:
        """Disable manual mode for a car."""
        if self._remote.disable_manual_mode(car_id):
            self._manual_mode_active[car_id] = False
            self._input.stop()

            self._commands_sent_gui += 1
            self.log(f"🎮 Car {car_id}: Manual mode DISABLED", "INFO")
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Car {car_id}: Failed to disable manual mode", "ERROR")

    def _send_manual_control(
        self, car_id: int, throttle: float, steering: float
    ) -> None:
        """Send manual control command."""
        self._remote.send_manual_control(car_id, throttle, steering)

    def _update_control_type(self, car_id: int, control_type: str) -> None:
        """Update control type for a car."""
        self.log(
            f"Car {car_id}: Manual control type set to {control_type.upper()}", "CONFIG"
        )

    # ========== Individual Perception Control ==========

    def _toggle_perception_car(self, car_id: int) -> None:
        """Toggle perception for a specific car."""
        # Check current state from telemetry
        telemetry = self._remote.get_telemetry(car_id)
        is_active = telemetry.get("perception_active", False) if telemetry else False

        if is_active:
            self._disable_perception_car(car_id)
        else:
            self._activate_perception_car(car_id)

    def _activate_perception_car(self, car_id: int) -> None:
        """Activate perception for a specific car."""
        if self._remote.activate_perception(car_id):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Perception activation sent", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to activate perception", "ERROR")

    def _disable_perception_car(self, car_id: int) -> None:
        """Disable perception for a specific car."""
        if self._remote.disable_perception(car_id):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Perception disabled", "INFO")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to disable perception", "ERROR")

    # ========== Individual Probing Control ==========

    def _toggle_probing_car(self, car_id: int) -> None:
        """Toggle probing for a specific car - opens observer window to see YOLO stream."""
        # Check if probing process exists for this car
        if not hasattr(self, "_probing_processes"):
            self._probing_processes = {}

        if (
            car_id in self._probing_processes
            and self._probing_processes[car_id] is not None
        ):
            # Stop probing
            self._stop_probing_car(car_id)
        else:
            # Start probing
            self._start_probing_car(car_id)

    def _start_probing_car(self, car_id: int) -> None:
        """Start probing for a specific car - runs multi_probing.py."""
        import subprocess
        import os

        if not hasattr(self, "_probing_processes"):
            self._probing_processes = {}

        try:
            # Path to multi_probing.py
            # app.py is at qcar/GUI/qcar_gui/app.py
            # multi_probing.py is at python/multi_probing.py (under multi_vehicle_self_driving_RealQcar)
            # So we need to go up 3 levels: qcar_gui -> GUI -> qcar -> multi_vehicle_self_driving_RealQcar
            base_path = os.path.dirname(
                os.path.dirname(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                )
            )
            probing_script = os.path.join(base_path, "python", "multi_probing.py")

            if not os.path.exists(probing_script):
                self.log(
                    f"Car {car_id}: Probing script not found at {probing_script}",
                    "ERROR",
                )
                return

            # --- Resolve the car IP dynamically ---
            # Priority 1: Live TCP peer address captured when the car connected to our server.
            #              This is the most reliable source because it comes from the active socket.
            ip = self._remote.get_car_ip(car_id)
            ip_source = "live TCP connection"

            # Priority 2: IP typed in the deployment panel (what the user entered before connecting).
            if not ip and car_id in self._deployment_panels:
                panel_ip = self._deployment_panels[car_id].get_ip()
                if panel_ip:
                    ip = panel_ip
                    ip_source = "deployment panel"

            if not ip:
                ip = "localhost"
                ip_source = "default (localhost)"

            self.log(f"Car {car_id}: Probing → IP={ip} (source: {ip_source})", "INFO")

            # Start the probing process
            cmd = ["python", probing_script, "--car", str(car_id), "--ip", ip]
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                creationflags=subprocess.CREATE_NEW_CONSOLE,  # Opens in new window on Windows
            )

            self._probing_processes[car_id] = process
            self.log(
                f"Car {car_id}: Probing started (IP={ip}) - observer window opened",
                "SUCCESS",
            )

            # Update button state in car panel
            if car_id in self._car_panels:
                panel = self._car_panels[car_id]
                if hasattr(panel, "_perception_ctrl") and panel._perception_ctrl:
                    panel._perception_ctrl.set_probing_active(True)

        except Exception as e:
            self.log(f"Car {car_id}: Failed to start probing - {e}", "ERROR")

    def _stop_probing_car(self, car_id: int) -> None:
        """Stop probing for a specific car."""
        if not hasattr(self, "_probing_processes"):
            return

        if (
            car_id in self._probing_processes
            and self._probing_processes[car_id] is not None
        ):
            try:
                self._probing_processes[car_id].terminate()
                self._probing_processes[car_id] = None
                self.log(f"Car {car_id}: Probing stopped", "INFO")

                # Update button state in car panel
                if car_id in self._car_panels:
                    panel = self._car_panels[car_id]
                    if hasattr(panel, "_perception_ctrl") and panel._perception_ctrl:
                        panel._perception_ctrl.set_probing_active(False)

            except Exception as e:
                self.log(f"Car {car_id}: Error stopping probing - {e}", "ERROR")

    # ========== Individual Scopes Control ==========

    def _toggle_scopes_car(self, car_id: int) -> None:
        """Toggle estimation scopes for a specific car."""
        # Check current state from telemetry
        telemetry = self._remote.get_telemetry(car_id)
        is_active = telemetry.get("scopes_active", False) if telemetry else False

        if is_active:
            self._disable_scopes_car(car_id)
        else:
            self._activate_scopes_car(car_id)

    def _activate_scopes_car(self, car_id: int) -> None:
        """Activate estimation scopes for a specific car."""
        command = {
            "type": "activate_scopes",
            "preset_names": ["local_state", "local_control"],
        }
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Scopes activation sent", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to activate scopes", "ERROR")

    def _disable_scopes_car(self, car_id: int) -> None:
        """Disable estimation scopes for a specific car."""
        if self._remote.send_command(car_id, {"type": "disable_scopes"}):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Scopes disabled", "INFO")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to disable scopes", "ERROR")

    # ========== Remote Plotting Control ==========

    def _toggle_remote_plot_local(self, car_id: int) -> None:
        """Toggle remote local scope streaming and visualization for a car."""
        is_streaming = self._remote.is_scope_streaming(car_id)

        if is_streaming:
            self._disable_remote_plot_local(car_id)
        else:
            self._enable_remote_plot_local(car_id)

    def _toggle_remote_plot_fleet(self, car_id: int) -> None:
        """Toggle remote fleet scope streaming and visualization for a car."""
        is_streaming = self._remote.is_scope_streaming(car_id)

        if is_streaming:
            self._disable_remote_plot_fleet(car_id)
        else:
            self._enable_remote_plot_fleet(car_id)

    def _enable_remote_plot_local(self, car_id: int) -> None:
        """Enable local scope streaming from vehicle and open viewer."""
        preset_names = ["local_state", "local_control"]

        # Send command to vehicle to start streaming
        success = self._remote.enable_scope_streaming(
            car_id, preset_names=preset_names, stream_rate=50.0
        )

        if success:
            # Open viewer window on Ground Station
            self._remote.open_scope_viewer(car_id, preset_names)

            # Update button state
            if car_id in self._car_panels:
                panel = self._car_panels[car_id]
                if hasattr(panel, "_scopes_control"):
                    panel._scopes_control.set_remote_local_active(True)

            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Local remote plot enabled", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to enable local remote plot", "ERROR")

    def _disable_remote_plot_local(self, car_id: int) -> None:
        """Disable local scope streaming from vehicle and close viewer."""
        # Close viewer window
        self._remote.close_scope_viewer(car_id)

        # Send command to vehicle to stop streaming
        success = self._remote.disable_scope_streaming(car_id)

        if success:
            # Update button state
            if car_id in self._car_panels:
                panel = self._car_panels[car_id]
                if hasattr(panel, "_scopes_control"):
                    panel._scopes_control.set_remote_local_active(False)

            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Local remote plot disabled", "INFO")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to disable local remote plot", "ERROR")

    def _enable_remote_plot_fleet(self, car_id: int) -> None:
        """Enable fleet scope streaming from vehicle and open viewer."""
        preset_names = ["fleet_state", "fleet_consensus"]

        # Send command to vehicle to start streaming
        success = self._remote.enable_scope_streaming(
            car_id, preset_names=preset_names, stream_rate=50.0
        )

        if success:
            # Open viewer window on Ground Station
            self._remote.open_scope_viewer(car_id, preset_names)

            # Update button state
            if car_id in self._car_panels:
                panel = self._car_panels[car_id]
                if hasattr(panel, "_scopes_control"):
                    panel._scopes_control.set_remote_fleet_active(True)

            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Fleet remote plot enabled", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to enable fleet remote plot", "ERROR")

    def _disable_remote_plot_fleet(self, car_id: int) -> None:
        """Disable fleet scope streaming from vehicle and close viewer."""
        # Close viewer window
        self._remote.close_scope_viewer(car_id)

        # Send command to vehicle to stop streaming
        success = self._remote.disable_scope_streaming(car_id)

        if success:
            # Update button state
            if car_id in self._car_panels:
                panel = self._car_panels[car_id]
                if hasattr(panel, "_scopes_control"):
                    panel._scopes_control.set_remote_fleet_active(False)

            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Fleet remote plot disabled", "INFO")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to disable fleet remote plot", "ERROR")

    # ========== Runtime Switching Commands ==========

    def _set_local_observer(self, car_id: int, observer_type: str) -> None:
        """Set local observer type for a specific car."""
        command = {"type": "set_local_observer", "observer_type": observer_type}
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Local observer → {observer_type}", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to set local observer", "ERROR")

    def _set_fleet_observer(self, car_id: int, observer_type: str) -> None:
        """Set fleet observer type for a specific car."""
        command = {"type": "set_fleet_observer", "observer_type": observer_type}
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Fleet observer → {observer_type}", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to set fleet observer", "ERROR")

    def _set_controller(
        self,
        car_id: int,
        category: str,
        controller_type: str,
        state_context: str = "path",
    ) -> None:
        """Set controller type for a specific car.

        Args:
            car_id: Vehicle identifier
            category: 'longitudinal' or 'lateral'
            controller_type: Controller type name
            state_context: 'path' or 'leader'
        """
        command = {
            "type": "set_controller",
            "category": category,  # 'longitudinal' or 'lateral'
            "controller_type": controller_type,
            "state_context": state_context,
        }
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(
                f"Car {car_id}: {state_context.capitalize()} {category} controller → {controller_type}",
                "SUCCESS",
            )
        else:
            self._commands_failed_gui += 1
            self.log(
                f"Car {car_id}: Failed to set {state_context} {category} controller",
                "ERROR",
            )

    def _set_controller_params(
        self,
        car_id: int,
        category: str,
        params: dict,
        state_context: str = "path",
    ) -> None:
        """Send a SET_PARAMS command to a vehicle."""
        command = {
            "type": "set_params",
            "category": category,
            "params": params,
            "state_context": state_context,
        }
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            params_str = ", ".join(f"{k}={v}" for k, v in params.items())
            self.log(
                f"Car {car_id}: Set {state_context} {category} params ({params_str})",
                "SUCCESS",
            )
        else:
            self._commands_failed_gui += 1
            self.log(
                f"Car {car_id}: Failed to pass {state_context} {category} params",
                "ERROR",
            )

    def _set_online_sysid(self, car_id: int, action: str, params: dict = None) -> None:
        """Send an online_sysid command to a vehicle."""
        if params is None:
            params = {}
        params["action"] = action
        command = {
            "type": "set_params",
            "category": "online_sysid",
            "params": params,
        }
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Online SysID [{action}] command sent", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(
                f"Car {car_id}: Failed to send Online SysID [{action}] command", "ERROR"
            )

    # ========== Fleet Commands ==========

    def _start_all_cars(self) -> None:
        """Start all connected cars."""
        results = self._remote.start_all_cars()
        successes = sum(1 for s in results.values() if s)
        self._commands_sent_gui += successes
        self._commands_failed_gui += len(results) - successes
        self.log(f"▶️ Start all: {successes}/{len(results)} cars started", "INFO")

    def _stop_all_cars(self) -> None:
        """Stop all connected cars."""
        results = self._remote.stop_all_cars()
        successes = sum(1 for s in results.values() if s)
        self._commands_sent_gui += successes
        self._commands_failed_gui += len(results) - successes
        self.log(f"⬛ Stop all: {successes}/{len(results)} cars stopped", "INFO")

    # ========== Platoon Commands ==========

    def _update_platoon_position(self, car_id: int, position: int) -> None:
        """Update platoon position for a car."""
        if self._platoon_config.formation is None:
            self._platoon_config.formation = {}

        self._platoon_config.formation[car_id] = position
        role = "LEADER" if position == 1 else "FOLLOWER"
        self.log(f"Car {car_id} platoon config: Position {position} ({role})", "CONFIG")

    def _setup_platoon(self) -> None:
        """Setup platoon formation."""
        # Collect positions from car panels
        formation = {}
        for car_id, panel in self._car_panels.items():
            formation[car_id] = panel.platoon_position

        if not formation:
            self.log("❌ No vehicles available for platoon", "ERROR")
            return

        # Validate formation
        positions = sorted(formation.values())
        if not positions or positions[0] != 1:
            self.log("❌ Position 1 (leader) must be assigned", "ERROR")
            return

        # Find leader
        leader_id = next((cid for cid, pos in formation.items() if pos == 1), None)

        # Send formation
        self.log(f"📊 Setting up platoon formation: {formation}", "INFO")
        self.log(f"👑 Leader: Car {leader_id}", "INFO")

        results = self._remote.setup_global_platoon_formation(formation)

        success_count = sum(1 for s in results.values() if s)
        for car_id, success in results.items():
            position = formation.get(car_id, 0)
            role = "LEADER" if position == 1 else f"FOLLOWER (pos {position})"
            if success:
                self.log(f"✅ Car {car_id}: Formation configured as {role}", "SUCCESS")
            else:
                self.log(f"❌ Car {car_id}: Failed to configure {role}", "ERROR")

        if success_count == len(formation):
            self._platoon_config.formation = formation
            self._platoon_config.leader_id = leader_id
            self._platoon_config.setup_complete = True
            self.log(f"🎉 Platoon formation setup complete!", "SUCCESS")
        else:
            self._platoon_config.setup_complete = False
            self.log(
                f"⚠️ Partial setup: {success_count}/{len(formation)} configured",
                "WARNING",
            )

    def _trigger_platoon(self) -> None:
        """Trigger platoon start."""
        if not self._platoon_config.setup_complete:
            self.log("❌ Platoon not set up - run Setup Platoon first", "ERROR")
            return

        formation = self._platoon_config.formation
        leader_id = self._platoon_config.leader_id

        self.log(f"🚀 Triggering platoon start with formation: {formation}", "INFO")

        success_count = 0
        for car_id in formation.keys():
            result = self._remote.start_platoon_mode(car_id, leader_id)
            if result.get("status") == "success":
                role = "LEADER" if car_id == leader_id else "FOLLOWER"
                self.log(f"✅ Car {car_id}: Platoon started ({role})", "SUCCESS")
                success_count += 1
            else:
                self.log(f"❌ Car {car_id}: {result.get('message', 'Failed')}", "ERROR")

        if success_count == len(formation):
            self.log(
                f"🎉 Platoon started successfully! {success_count}/{len(formation)} vehicles active",
                "SUCCESS",
            )
        else:
            self.log(
                f"⚠️ Partial start: {success_count}/{len(formation)} vehicles started",
                "WARNING",
            )

    def _disable_all_platoons(self) -> None:
        """Disable all platoons."""
        results = self._remote.disable_all_platoons()
        successes = sum(1 for s in results.values() if s)
        self._platoon_config.setup_complete = False
        self.log(f"🚗 Disabled platoons: {successes}/{len(results)} cars", "INFO")

    # ========== V2V Commands ==========

    def _activate_v2v(self) -> None:
        """Activate V2V communication."""
        if len(self._connected_cars) < 2:
            self.log("❌ V2V requires at least 2 connected vehicles", "ERROR")
            messagebox.showwarning(
                "V2V Error", "V2V requires at least 2 connected vehicles"
            )
            return

        self._fleet_controls.set_v2v_activating(True)
        self.log("📡 Activating V2V communication...", "INFO")

        success_count = 0
        connected_list = list(self._connected_cars)

        for car_id in connected_list:
            peers = [cid for cid in connected_list if cid != car_id]

            # Get peer IPs
            vehicle_ips = []
            for peer_id in peers:
                status = self._remote.get_car_status(peer_id)
                if status and status.get("address"):
                    vehicle_ips.append(status["address"][0])
                else:
                    vehicle_ips.append(f"192.168.1.{100 + peer_id}")

            command = {
                "command": "activate_v2v",
                "peer_vehicles": peers,
                "peer_ips": vehicle_ips,
                "my_id": car_id,
            }

            if self._remote.send_command(car_id, command):
                success_count += 1
                self._commands_sent_gui += 1
            else:
                self._commands_failed_gui += 1

        if success_count > 0:
            self.log(
                f"V2V activation sent to {success_count}/{len(connected_list)} vehicles",
                "SUCCESS",
            )
            # Set timeout to reset button
            self.root.after(10000, self._v2v_activation_timeout)
        else:
            self.log("Failed to send V2V activation", "ERROR")
            self._fleet_controls.set_v2v_activating(False)

    def _v2v_activation_timeout(self) -> None:
        """Handle V2V activation timeout."""
        # Check if still waiting for V2V
        if self._fleet_controls:
            self._fleet_controls.set_v2v_activating(False)
            self.log("⏰ V2V activation timeout - button re-enabled", "WARNING")

    def _disable_v2v(self) -> None:
        """Disable V2V communication."""
        self.log("📡 Disabling V2V communication...", "INFO")

        success_count = 0
        for car_id in self._connected_cars:
            if self._remote.send_command(car_id, {"command": "disable_v2v"}):
                success_count += 1
                self._commands_sent_gui += 1
            else:
                self._commands_failed_gui += 1

        if success_count > 0:
            self.log(f"✅ V2V disabled for {success_count} vehicles", "SUCCESS")
            self._fleet_controls.reset_v2v_buttons()
            self._v2v_status.clear()
            self._v2v_network_established = (
                False  # Reset so message can be logged again
            )
        else:
            self.log("❌ Failed to disable V2V", "ERROR")

    def _activate_perception(self) -> None:
        """Activate perception system (YOLO) for all connected vehicles."""
        self.log("👁️ Activating perception systems...", "INFO")

        success_count = 0
        failed_cars = []

        for car_id in self._connected_cars:
            if self._remote.activate_perception(car_id):
                success_count += 1
                self._commands_sent_gui += 1
                self.log(
                    f"✅ Car {car_id}: Perception activation command sent", "SUCCESS"
                )
            else:
                failed_cars.append(car_id)
                self._commands_failed_gui += 1

        if success_count > 0:
            self.log(
                f"✅ Perception activated for {success_count}/{len(self._connected_cars)} vehicles",
                "SUCCESS",
            )

        if failed_cars:
            self.log(
                f"❌ Failed to activate perception for cars: {failed_cars}", "ERROR"
            )

    def _disable_perception(self) -> None:
        """Disable perception system for all connected vehicles."""
        self.log("👁️ Disabling perception systems...", "INFO")

        success_count = 0
        for car_id in self._connected_cars:
            if self._remote.disable_perception(car_id):
                success_count += 1
                self._commands_sent_gui += 1
            else:
                self._commands_failed_gui += 1

        if success_count > 0:
            self.log(f"✅ Perception disabled for {success_count} vehicles", "SUCCESS")
        else:
            self.log("❌ Failed to disable perception", "ERROR")

    # ========== Callback Handlers (for backward compatibility) ==========

    def process_v2v_status(self, car_id: int, v2v_data: dict) -> None:
        """Process V2V status from a vehicle."""
        status = v2v_data.get("status", "unknown")

        if status == "connected":
            peers = v2v_data.get("connected_peers", 0)

            # Check for change before logging to avoid spam
            # Check for change before logging to avoid spam
            current_status = self._v2v_status.get(car_id, {})

            # Update status
            self._v2v_status[car_id] = {"status": "connected", "peers": peers}

            # ONLY log if we weren't connected before (debounce updates)
            if current_status.get("status") != "connected":
                self.log(f"📡 Car {car_id}: V2V connected ({peers} peers)", "SUCCESS")
            # Else: just updated peer count silently

            # Check if all vehicles connected
            self._check_v2v_network()

        elif status == "failed":
            error = v2v_data.get("error", "unknown")
            self._v2v_status[car_id] = {"status": "failed", "error": error}
            self.log(f"❌ Car {car_id}: V2V failed - {error}", "ERROR")

        elif status == "disconnected":
            self._v2v_status[car_id] = {"status": "disconnected"}

            # self.log(f"📡 Car {car_id}: V2V disconnected", 'WARNING')

    def _check_v2v_network(self) -> None:
        """Check if V2V network is fully established."""
        if len(self._connected_cars) < 2:
            return

        connected = [
            cid
            for cid, status in self._v2v_status.items()
            if status.get("status") == "connected"
        ]

        expected_peers = len(self._connected_cars) - 1
        fully_connected = all(
            self._v2v_status.get(cid, {}).get("peers", 0) >= expected_peers
            for cid in connected
        )

        if len(connected) == len(self._connected_cars) and fully_connected:
            self._fleet_controls.set_v2v_connected(True)
            # Only log once when network becomes established
            if not getattr(self, "_v2v_network_established", False):
                self._v2v_network_established = True
                self.log("✅ V2V Network Fully Established", "SUCCESS")

    def process_platoon_setup_confirmation(
        self, car_id: int, platoon_data: dict
    ) -> None:
        """Process platoon setup confirmation from a vehicle."""
        position = platoon_data.get("position")
        is_leader = platoon_data.get("is_leader", False)

        role = "LEADER" if is_leader else f"FOLLOWER-{position}"
        self.log(f"✅ Car {car_id} platoon confirmed: {role}", "SUCCESS")

    # ========== Update Loop ==========

    def _update_loop(self) -> None:
        """Background update loop."""
        update_interval = 1.0 / self.config.gui.update_rate_hz

        while self._running:
            try:
                # Update car panels
                self._update_car_panels()

                # Update car states
                self._update_car_states()

                # Update status panel
                self._update_status()

                # Update header stats
                self._update_header_stats()

                time.sleep(update_interval)

            except Exception as e:
                print(f"Update loop error: {e}")
                time.sleep(0.1)

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

    # ========== Logging ==========

    def log(self, message: str, level: str = "INFO") -> None:
        """Log a message."""
        log_panel = getattr(self, "_log_panel", None)
        if log_panel:
            log_panel.log(message, level)

    # ========== Cleanup ==========

    def _on_closing(self) -> None:
        """Handle window close."""
        self._running = False

        # Disable manual modes
        for car_id in list(self._manual_mode_active.keys()):
            if self._manual_mode_active.get(car_id):
                self._remote.disable_manual_mode(car_id)

        # Cleanup input controller
        self._input.cleanup()

        # Cleanup vehicle connector
        if self._vehicle_connector:
            self._vehicle_connector.cleanup()

        # Close remote controller
        self._remote.close()

        # Destroy window
        self.root.destroy()


def create_app(
    num_cars: int = 5,
    host_ip: str = "0.0.0.0",
    base_port: int = 5000,
    ws_port: int = 8080,
) -> QCarFleetController:
    """
    Factory function to create the application.

    Args:
        num_cars: Number of cars to support
        host_ip: IP address to listen on
        base_port: Base port number
        ws_port: WebSocket port number for web bridge
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
    # Configuration
    NUM_CARS = 5
    HOST_IP = "0.0.0.0"
    BASE_PORT = 5000

    app = create_app(num_cars=NUM_CARS, host_ip=HOST_IP, base_port=BASE_PORT)

    # Log startup
    app.log("QCar Fleet Controller started", "SUCCESS")
    app.log(f"Listening on ports {BASE_PORT}-{BASE_PORT + NUM_CARS - 1}", "INFO")
    app.log(
        "Features: Command validation, platoon control, V2V, manual control", "INFO"
    )

    app.root.mainloop()


if __name__ == "__main__":
    main()
