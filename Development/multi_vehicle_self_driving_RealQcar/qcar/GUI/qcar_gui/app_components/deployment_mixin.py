import tkinter as tk
import threading
from tkinter import messagebox

from ..widgets.base import ThemedButton, ThemedLabelFrame, ThemedEntry, ThemedLabel
from ..widgets import VehicleConnectionPanel, VehicleConnectionConfig, ConnectionCallbacks, ScrollableFrame

class DeploymentMixin:
    """Mixin for deployment tab functionality."""

    def _build_deployment_tab(self, parent: tk.Frame) -> None:
        """Build the vehicle deployment tab content."""
        c = self.theme.colors

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

        self._deployment_count_label = tk.Label(
            batch_row,
            text=f"Vehicles: {self.config.gui.default_num_cars}",
            bg=c.bg_medium,
            fg=c.fg_primary,
            font=self.theme.fonts.body(),
        )
        self._deployment_count_label.pack(side="left", padx=(0, 10))

        ThemedButton(
            batch_row,
            text="➕ Add",
            button_type="command",
            command=self._add_deployment_vehicle,
            padx=10,
            pady=3,
        ).pack(side="left", padx=(0, 5))

        ThemedButton(
            batch_row,
            text="➖ Remove",
            button_type="warning",
            command=self._remove_deployment_vehicle,
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

        # Add initial deployment panels
        for i in range(self.config.gui.default_num_cars):
            self._add_deployment_panel(i)

    def _add_deployment_panel(self, car_id: int = None) -> None:
        """Add a new vehicle deployment panel."""
        if car_id is None:
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
            on_test_connection=lambda ip: self._on_test_connection(ip, car_id),
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

    def _update_vehicle_connector_config(self) -> None:
        """Update vehicle connector with current Ground Station settings."""
        try:
            gs_ip = self._gs_ip_entry.get().strip()
            gs_port = int(self._gs_port_entry.get().strip())
            self._vehicle_connector.set_ground_station_config(gs_ip, gs_port)
        except (ValueError, AttributeError):
            pass

    def _on_test_connection(self, ip: str, car_id: int = None) -> None:
        """Handle test connection request."""
        panel = self._deployment_panels.get(car_id) if car_id is not None else None
        vehicle_type = panel._get_current_config().vehicle_type if panel else None

        success, message = self._vehicle_connector.test_connection(
            ip, car_id=car_id, vehicle_type=vehicle_type
        )
        if panel:
            panel.set_connected(success, message)
        else:
            self.log(f"Connection test to {ip}: {message}", "SUCCESS" if success else "ERROR")

    def _on_vehicle_connect(self, config: VehicleConnectionConfig) -> None:
        """Handle vehicle connection request."""
        self._update_vehicle_connector_config()

        success, message = self._vehicle_connector.connect(
            config.car_id, config.ip, config.vehicle_type
        )

        panel = self._deployment_panels.get(config.car_id)
        if panel:
            panel.set_connected(success, message)

    def _on_vehicle_disconnect(self, car_id: int) -> None:
        """Handle vehicle disconnection request."""
        self._vehicle_connector.disconnect(car_id)
        self.log(f"Vehicle {car_id}: Disconnected", "INFO")
        panel = self._deployment_panels.get(car_id)
        if panel:
            panel.set_connected(False, "Disconnected")

    def _on_vehicle_upload(self, config: VehicleConnectionConfig) -> None:
        """Handle file upload request."""
        self._update_vehicle_connector_config()

        panel = self._deployment_panels.get(config.car_id)
        if panel:
            panel.set_progress("Uploading files...")
            self._vehicle_connector.progress_callback = panel.set_progress

        success, message = self._vehicle_connector.upload_files(
            config.car_id, config.ip, config.vehicle_type, config.folders_to_upload, config.upload_root_files
        )

        if panel:
            panel.set_upload_complete(success, message)
            self._vehicle_connector.progress_callback = None

    def _on_vehicle_start(self, config: VehicleConnectionConfig) -> None:
        """Handle vehicle start request."""
        self._update_vehicle_connector_config()
        
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
            self.log(f"Vehicle {config.car_id} started - waiting for connection...", "INFO")
            # Switch to Connected Vehicles tab after successful start
            if hasattr(self, "_notebook"):
                self.root.after(2000, lambda: self._notebook.select(0))

    def _on_vehicle_stop(
        self, car_id: int, ip: str, stop_quarc: bool = True, stop_hardware: bool = True
    ) -> None:
        """Handle vehicle stop request."""
        self.log(
            f"Car {car_id}: Stopping (QUARC={stop_quarc}, Hardware={stop_hardware})...",
            "INFO",
        )

        success, message = self._vehicle_connector.stop_vehicle(
            car_id, ip, stop_quarc=stop_quarc, stop_hardware=stop_hardware
        )

        panel = self._deployment_panels.get(car_id)
        if panel:
            panel.set_vehicle_running(False, message if success else "Stop command sent")
            panel._stop_btn.config(state="normal")

    def _on_vehicle_calibrate(
        self, config: VehicleConnectionConfig, distribute_to_all: bool = True
    ) -> None:
        """Handle vehicle calibration request (LiDAR calibration)."""
        car_id = config.car_id
        ip = config.ip
        self.log(f"Car {car_id}: Starting LiDAR calibration (distribute={distribute_to_all})...", "INFO")

        panel = self._deployment_panels.get(car_id)
        if panel:
            self._vehicle_connector.progress_callback = panel.set_progress

        distribute_ips = []
        if distribute_to_all:
            for other_id, other_panel in self._deployment_panels.items():
                if other_id != car_id and other_panel.is_connected:
                    other_ip = other_panel.get_ip()
                    if other_ip:
                        distribute_ips.append(other_ip)

        success, message = self._vehicle_connector.calibrate_vehicle(
            car_id=car_id, ip=ip, distribute_ips=distribute_ips if distribute_to_all else None
        )

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
        self.log("🔌 Connecting to all vehicles...", "INFO")

        for car_id, panel in self._deployment_panels.items():
            config = panel._get_current_config()
            if config.ip:
                threading.Thread(
                    target=self._on_vehicle_connect, args=(config,), daemon=True
                ).start()

    def _upload_all_vehicles(self) -> None:
        """Upload files to all connected vehicles."""
        self.log("📤 Uploading files to all connected vehicles...", "INFO")

        for car_id, panel in self._deployment_panels.items():
            if panel.is_connected:
                config = panel._get_current_config()
                threading.Thread(
                    target=self._on_vehicle_upload, args=(config,), daemon=True
                ).start()

    def _start_all_deployed_vehicles(self) -> None:
        """Start all connected vehicles."""
        self.log("▶️ Starting all connected vehicles...", "INFO")

        for car_id, panel in self._deployment_panels.items():
            if panel.is_connected and not panel.is_running:
                config = panel._get_current_config()
                threading.Thread(
                    target=self._on_vehicle_start, args=(config,), daemon=True
                ).start()

    def _stop_all_deployed_vehicles(self) -> None:
        """Stop all running vehicles."""
        self.log("⬛ Stopping all deployed vehicles...", "INFO")

        for car_id, panel in self._deployment_panels.items():
            if panel.is_running:
                ip = panel.get_ip()
                stop_quarc = panel._stop_quarc_var.get() if hasattr(panel, "_stop_quarc_var") else True
                stop_hardware = panel._stop_hardware_var.get() if hasattr(panel, "_stop_hardware_var") else True
                
                threading.Thread(
                    target=self._on_vehicle_stop, args=(car_id, ip, stop_quarc, stop_hardware), daemon=True
                ).start()
        self.log(f"Waiting for {self.config.gui.default_num_cars} cars to connect...", "INFO")
