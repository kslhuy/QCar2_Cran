"""
Vehicle Connection Panel Widget for QCar Fleet Controller.

This module provides a GUI panel for connecting to real QCar vehicles
via SSH, uploading files, and starting vehicle control programs.
Similar functionality to start_enhanced.bat but with a graphical interface.
"""

import tkinter as tk
from tkinter import ttk, messagebox
from typing import Optional, Callable, Dict, Any, List
from dataclasses import dataclass, field
import threading

from ..theme import Theme, DEFAULT_THEME
from .base import (
    BaseWidget, ThemedButton, ThemedEntry, ThemedLabel,
    ThemedLabelFrame, StatusIndicator
)


@dataclass
class VehicleConnectionConfig:
    """Configuration for a vehicle connection."""
    car_id: int = 0
    ip: str = ""
    vehicle_type: str = "Qcar"  # "Qcar" or "Limo"
    enabled: bool = True
    probing: bool = False
    calibrate: bool = False
    path_number: int = 0
    left_hand_traffic: bool = False
    initial_v_ref: float = 0.6
    description: str = ""


@dataclass
class ConnectionCallbacks:
    """Callbacks for connection panel actions."""
    on_connect: Callable[[VehicleConnectionConfig], None] = None
    on_disconnect: Callable[[int], None] = None
    on_upload_files: Callable[[VehicleConnectionConfig], None] = None
    on_start_vehicle: Callable[[VehicleConnectionConfig], None] = None
    on_stop_vehicle: Callable[[int, str, bool, bool], None] = None  # car_id, ip, stop_quarc, stop_hardware
    on_test_connection: Callable[[str], None] = None
    on_config_changed: Callable[[VehicleConnectionConfig], None] = None


class VehicleConnectionPanel(BaseWidget):
    """
    Widget for configuring and connecting to a real QCar vehicle.
    
    Provides GUI controls for:
    - IP address configuration
    - SSH connection testing
    - File upload (Python scripts, YAML configs, folders)
    - Starting/stopping vehicle control programs
    - Vehicle-specific settings (path, velocity, calibration, etc.)
    """
    
    def __init__(self, parent: tk.Widget, car_id: int,
                 callbacks: ConnectionCallbacks = None,
                 default_config: VehicleConnectionConfig = None,
                 theme: Theme = None):
        self.car_id = car_id
        self.callbacks = callbacks or ConnectionCallbacks()
        self.default_config = default_config or VehicleConnectionConfig(car_id=car_id)
        
        # State tracking
        self._connection_status = "disconnected"  # disconnected, connecting, connected, error
        self._upload_status = "idle"  # idle, uploading, done, error
        self._vehicle_running = False
        
        # UI components
        self._ip_entry: Optional[ThemedEntry] = None
        self._vehicle_type_var: Optional[tk.StringVar] = None
        self._path_entry: Optional[ThemedEntry] = None
        self._velocity_entry: Optional[ThemedEntry] = None
        self._probing_var: Optional[tk.BooleanVar] = None
        self._calibrate_var: Optional[tk.BooleanVar] = None
        self._left_hand_var: Optional[tk.BooleanVar] = None
        
        self._status_indicator: Optional[StatusIndicator] = None
        self._status_label: Optional[tk.Label] = None
        self._connect_btn: Optional[tk.Button] = None
        self._upload_btn: Optional[tk.Button] = None
        self._start_btn: Optional[tk.Button] = None
        self._stop_btn: Optional[tk.Button] = None
        
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the connection panel widget."""
        c = self.theme.colors
        
        # Main frame with border
        self.frame = tk.Frame(
            self.parent,
            bg=c.bg_panel,
            highlightbackground=c.bg_light,
            highlightthickness=1
        )
        
        # Header
        self._build_header()
        
        # Connection settings section
        self._build_connection_section()
        
        # Vehicle settings section
        self._build_vehicle_settings()
        
        # Action buttons section
        self._build_action_buttons()
        
        # Status section
        self._build_status_section()
    
    def _build_header(self) -> None:
        """Build the header with title and status indicator."""
        c = self.theme.colors
        
        header = tk.Frame(self.frame, bg=c.bg_header, height=40)
        header.pack(fill='x')
        header.pack_propagate(False)
        
        # Title
        title_label = tk.Label(
            header,
            text=f"🚗 Vehicle {self.car_id} Connection",
            bg=c.bg_header,
            fg=c.fg_primary,
            font=self.theme.fonts.heading()
        )
        title_label.pack(side='left', padx=12, pady=8)
        
        # Status indicator
        self._status_indicator = StatusIndicator(
            header,
            status='disconnected',
            theme=self.theme
        )
        self._status_indicator.pack(side='right', padx=12, pady=8)
    
    def _build_connection_section(self) -> None:
        """Build the connection settings section."""
        c = self.theme.colors
        
        conn_frame = ThemedLabelFrame(
            self.frame,
            text="📡 Connection Settings",
            theme=self.theme
        )
        conn_frame.pack(fill='x', padx=10, pady=(10, 5))
        
        content = tk.Frame(conn_frame, bg=c.bg_medium)
        content.pack(fill='x', padx=8, pady=6)
        
        # IP Address row
        ip_row = tk.Frame(content, bg=c.bg_medium)
        ip_row.pack(fill='x', pady=2)
        
        ThemedLabel(ip_row, text="IP Address:", style='muted', theme=self.theme).pack(side='left', padx=(0, 10))
        
        self._ip_entry = ThemedEntry(ip_row, width=18, theme=self.theme)
        self._ip_entry.insert(0, self.default_config.ip or f"192.168.2.{100 + self.car_id}")
        self._ip_entry.pack(side='left', padx=(0, 10))
        
        # Test connection button
        ThemedButton(
            ip_row,
            text="🔍 Test",
            button_type='command',
            command=self._test_connection,
            padx=8,
            pady=2
        ).pack(side='left')
        
        # Vehicle type row
        type_row = tk.Frame(content, bg=c.bg_medium)
        type_row.pack(fill='x', pady=2)
        
        ThemedLabel(type_row, text="Vehicle Type:", style='muted', theme=self.theme).pack(side='left', padx=(0, 10))
        
        self._vehicle_type_var = tk.StringVar(value=self.default_config.vehicle_type)
        
        for text, value in [("🚙 QCar", "Qcar"), ("🚗 Limo", "Limo")]:
            tk.Radiobutton(
                type_row,
                text=text,
                variable=self._vehicle_type_var,
                value=value,
                bg=c.bg_medium,
                fg=c.fg_primary,
                selectcolor=c.bg_light,
                font=self.theme.fonts.small()
            ).pack(side='left', padx=(0, 15))
    
    def _build_vehicle_settings(self) -> None:
        """Build the vehicle settings section."""
        c = self.theme.colors
        
        settings_frame = ThemedLabelFrame(
            self.frame,
            text="⚙️ Vehicle Settings",
            theme=self.theme
        )
        settings_frame.pack(fill='x', padx=10, pady=5)
        
        content = tk.Frame(settings_frame, bg=c.bg_medium)
        content.pack(fill='x', padx=8, pady=6)
        
        # Path and Velocity row
        path_vel_row = tk.Frame(content, bg=c.bg_medium)
        path_vel_row.pack(fill='x', pady=2)
        
        ThemedLabel(path_vel_row, text="Path #:", style='muted', theme=self.theme).pack(side='left', padx=(0, 5))
        self._path_entry = ThemedEntry(path_vel_row, width=4, theme=self.theme)
        self._path_entry.insert(0, str(self.default_config.path_number))
        self._path_entry.pack(side='left', padx=(0, 15))
        
        ThemedLabel(path_vel_row, text="Initial Velocity:", style='muted', theme=self.theme).pack(side='left', padx=(0, 5))
        self._velocity_entry = ThemedEntry(path_vel_row, width=6, theme=self.theme)
        self._velocity_entry.insert(0, str(self.default_config.initial_v_ref))
        self._velocity_entry.pack(side='left', padx=(0, 5))
        ThemedLabel(path_vel_row, text="m/s", style='muted', theme=self.theme).pack(side='left')
        
        # Checkboxes row
        checkbox_row = tk.Frame(content, bg=c.bg_medium)
        checkbox_row.pack(fill='x', pady=(5, 2))
        
        self._probing_var = tk.BooleanVar(value=self.default_config.probing)
        tk.Checkbutton(
            checkbox_row,
            text="🎥 Probing (YOLO)",
            variable=self._probing_var,
            bg=c.bg_medium,
            fg=c.fg_primary,
            selectcolor=c.bg_light,
            font=self.theme.fonts.small()
        ).pack(side='left', padx=(0, 15))
        
        self._calibrate_var = tk.BooleanVar(value=self.default_config.calibrate)
        tk.Checkbutton(
            checkbox_row,
            text="📍 Calibrate GPS",
            variable=self._calibrate_var,
            bg=c.bg_medium,
            fg=c.fg_primary,
            selectcolor=c.bg_light,
            font=self.theme.fonts.small()
        ).pack(side='left', padx=(0, 15))
        
        self._left_hand_var = tk.BooleanVar(value=self.default_config.left_hand_traffic)
        tk.Checkbutton(
            checkbox_row,
            text="🚦 Left-Hand Traffic",
            variable=self._left_hand_var,
            bg=c.bg_medium,
            fg=c.fg_primary,
            selectcolor=c.bg_light,
            font=self.theme.fonts.small()
        ).pack(side='left')
    
    def _build_action_buttons(self) -> None:
        """Build the action buttons section."""
        c = self.theme.colors
        
        action_frame = ThemedLabelFrame(
            self.frame,
            text="🎬 Actions",
            theme=self.theme
        )
        action_frame.pack(fill='x', padx=10, pady=5)
        
        content = tk.Frame(action_frame, bg=c.bg_medium)
        content.pack(fill='x', padx=8, pady=6)
        
        # Button row 1: Connect and Upload
        row1 = tk.Frame(content, bg=c.bg_medium)
        row1.pack(fill='x', pady=2)
        
        self._connect_btn = ThemedButton(
            row1,
            text="🔌 Connect",
            button_type='command',
            command=self._on_connect,
            padx=12,
            pady=4
        )
        self._connect_btn.pack(side='left', expand=True, fill='x', padx=(0, 5))
        
        self._upload_btn = ThemedButton(
            row1,
            text="📤 Upload Files",
            button_type='warning',
            command=self._on_upload,
            padx=12,
            pady=4
        )
        self._upload_btn.pack(side='left', expand=True, fill='x', padx=(5, 0))
        self._upload_btn.config(state='disabled')
        
        # Button row 2: Start and Stop
        row2 = tk.Frame(content, bg=c.bg_medium)
        row2.pack(fill='x', pady=(5, 2))
        
        self._start_btn = ThemedButton(
            row2,
            text="▶️ Start Vehicle",
            button_type='start',
            command=self._on_start,
            padx=12,
            pady=4
        )
        self._start_btn.pack(side='left', expand=True, fill='x', padx=(0, 5))
        self._start_btn.config(state='disabled')
        
        self._stop_btn = ThemedButton(
            row2,
            text="⬛ Stop (Full)",
            button_type='stop',
            command=self._on_stop,
            padx=12,
            pady=4
        )
        self._stop_btn.pack(side='left', expand=True, fill='x', padx=(5, 0))
        self._stop_btn.config(state='disabled')
        
        # Button row 3: Stop options (checkboxes)
        row3 = tk.Frame(content, bg=c.bg_medium)
        row3.pack(fill='x', pady=(3, 2))
        
        ThemedLabel(row3, text="Stop options:", style='muted', theme=self.theme).pack(side='left', padx=(0, 10))
        
        self._stop_quarc_var = tk.BooleanVar(value=True)
        tk.Checkbutton(
            row3,
            text="🔧 QUARC",
            variable=self._stop_quarc_var,
            bg=c.bg_medium,
            fg=c.fg_primary,
            selectcolor=c.bg_light,
            font=self.theme.fonts.tiny()
        ).pack(side='left', padx=(0, 10))
        
        self._stop_hardware_var = tk.BooleanVar(value=True)
        tk.Checkbutton(
            row3,
            text="🔌 Hardware",
            variable=self._stop_hardware_var,
            bg=c.bg_medium,
            fg=c.fg_primary,
            selectcolor=c.bg_light,
            font=self.theme.fonts.tiny()
        ).pack(side='left')
    
    def _build_status_section(self) -> None:
        """Build the status section."""
        c = self.theme.colors
        
        status_frame = tk.Frame(self.frame, bg=c.bg_panel)
        status_frame.pack(fill='x', padx=10, pady=(5, 10))
        
        self._status_label = tk.Label(
            status_frame,
            text="Status: Not connected",
            bg=c.bg_panel,
            fg=c.fg_muted,
            font=self.theme.fonts.small(),
            anchor='w'
        )
        self._status_label.pack(fill='x')
        
        # Progress label for operations
        self._progress_label = tk.Label(
            status_frame,
            text="",
            bg=c.bg_panel,
            fg=c.fg_secondary,
            font=self.theme.fonts.tiny(),
            anchor='w'
        )
        self._progress_label.pack(fill='x')
    
    # ========== Event Handlers ==========
    
    def _test_connection(self) -> None:
        """Test SSH connection to the vehicle."""
        ip = self._ip_entry.get().strip()
        if not ip:
            messagebox.showwarning("Warning", "Please enter an IP address")
            return
        
        self._set_status("testing", "Testing connection...")
        
        if self.callbacks.on_test_connection:
            # Run in background thread
            threading.Thread(
                target=self.callbacks.on_test_connection,
                args=(ip,),
                daemon=True
            ).start()
    
    def _on_connect(self) -> None:
        """Handle connect button click."""
        config = self._get_current_config()
        
        if not config.ip:
            messagebox.showwarning("Warning", "Please enter an IP address")
            return
        
        self._set_status("connecting", f"Connecting to {config.ip}...")
        
        if self.callbacks.on_connect:
            threading.Thread(
                target=self.callbacks.on_connect,
                args=(config,),
                daemon=True
            ).start()
    
    def _on_upload(self) -> None:
        """Handle upload button click."""
        config = self._get_current_config()
        
        self._set_status("uploading", "Uploading files...")
        self._upload_btn.config(state='disabled')
        
        if self.callbacks.on_upload_files:
            threading.Thread(
                target=self.callbacks.on_upload_files,
                args=(config,),
                daemon=True
            ).start()
    
    def _on_start(self) -> None:
        """Handle start vehicle button click."""
        config = self._get_current_config()
        
        self._set_status("starting", "Starting vehicle control program...")
        self._start_btn.config(state='disabled')
        
        if self.callbacks.on_start_vehicle:
            threading.Thread(
                target=self.callbacks.on_start_vehicle,
                args=(config,),
                daemon=True
            ).start()
    
    def _on_stop(self) -> None:
        """Handle stop vehicle button click (enhanced - like stop_enhanced.bat)."""
        ip = self._ip_entry.get().strip()
        stop_quarc = self._stop_quarc_var.get()
        stop_hardware = self._stop_hardware_var.get()
        
        # Build status message based on what will be stopped
        steps = ["Python processes"]
        if stop_quarc:
            steps.append("QUARC models")
        if stop_hardware:
            steps.append("Hardware")
        
        self._set_status("stopping", f"Stopping: {', '.join(steps)}...")
        self._stop_btn.config(state='disabled')
        
        if self.callbacks.on_stop_vehicle:
            threading.Thread(
                target=self.callbacks.on_stop_vehicle,
                args=(self.car_id, ip, stop_quarc, stop_hardware),
                daemon=True
            ).start()
    
    # ========== State Management ==========
    
    def _get_current_config(self) -> VehicleConnectionConfig:
        """Get the current configuration from UI inputs."""
        return VehicleConnectionConfig(
            car_id=self.car_id,
            ip=self._ip_entry.get().strip(),
            vehicle_type=self._vehicle_type_var.get(),
            enabled=True,
            probing=self._probing_var.get(),
            calibrate=self._calibrate_var.get(),
            path_number=int(self._path_entry.get() or 0),
            left_hand_traffic=self._left_hand_var.get(),
            initial_v_ref=float(self._velocity_entry.get() or 0.6),
            description=f"Vehicle {self.car_id}"
        )
    
    def _set_status(self, status: str, message: str) -> None:
        """Update the status display."""
        c = self.theme.colors
        
        status_colors = {
            "disconnected": c.fg_muted,
            "connecting": c.accent_orange,
            "connected": c.accent_green,
            "testing": c.accent_blue,
            "uploading": c.accent_orange,
            "uploaded": c.accent_green,
            "starting": c.accent_orange,
            "running": c.accent_green,
            "stopping": c.accent_orange,
            "error": c.accent_red,
        }
        
        if self._status_label:
            self._status_label.config(
                text=f"Status: {message}",
                fg=status_colors.get(status, c.fg_muted)
            )
        
        self._connection_status = status
    
    def set_progress(self, message: str) -> None:
        """Update progress message (thread-safe)."""
        if self._progress_label:
            self._progress_label.after(0, lambda: self._progress_label.config(text=message))
    
    def set_connected(self, success: bool, message: str = "") -> None:
        """Update connection status after connection attempt (thread-safe)."""
        def _update():
            c = self.theme.colors
            
            if success:
                self._connection_status = "connected"
                self._status_indicator.set_status('connected')
                self._set_status("connected", message or "Connected successfully")
                
                # Enable buttons
                self._upload_btn.config(state='normal')
                self._start_btn.config(state='normal')
                self._stop_btn.config(state='normal')
                self._connect_btn.config(text="🔄 Reconnect")
            else:
                self._connection_status = "error"
                self._status_indicator.set_status('disconnected')
                self._set_status("error", message or "Connection failed")
                
                # Disable buttons
                self._upload_btn.config(state='disabled')
                self._start_btn.config(state='disabled')
                self._stop_btn.config(state='disabled')
        
        # Schedule on main thread
        if self._status_label:
            self._status_label.after(0, _update)
    
    def set_upload_complete(self, success: bool, message: str = "") -> None:
        """Update after file upload (thread-safe)."""
        def _update():
            if success:
                self._upload_status = "done"
                self._set_status("uploaded", message or "Files uploaded successfully")
            else:
                self._upload_status = "error"
                self._set_status("error", message or "Upload failed")
            
            self._upload_btn.config(state='normal')
        
        if self._status_label:
            self._status_label.after(0, _update)
    
    def set_vehicle_running(self, running: bool, message: str = "") -> None:
        """Update vehicle running status (thread-safe)."""
        def _update():
            self._vehicle_running = running
            
            if running:
                self._set_status("running", message or "Vehicle program running")
                self._start_btn.config(state='disabled', text="✅ Running")
                self._stop_btn.config(state='normal')
            else:
                self._set_status("connected", message or "Vehicle stopped")
                self._start_btn.config(state='normal', text="▶️ Start Vehicle")
        
        if self._status_label:
            self._status_label.after(0, _update)
    
    def get_ip(self) -> str:
        """Get the current IP address."""
        return self._ip_entry.get().strip() if self._ip_entry else ""
    
    def set_ip(self, ip: str) -> None:
        """Set the IP address."""
        if self._ip_entry:
            self._ip_entry.delete(0, tk.END)
            self._ip_entry.insert(0, ip)
    
    @property
    def is_connected(self) -> bool:
        """Check if the vehicle is connected."""
        return self._connection_status == "connected"
    
    @property
    def is_running(self) -> bool:
        """Check if the vehicle program is running."""
        return self._vehicle_running


class FleetConnectionPanel(BaseWidget):
    """
    Panel for managing multiple vehicle connections.
    
    Provides a master control panel with:
    - Add/remove vehicle slots
    - Batch operations (connect all, upload all, start all)
    - Fleet overview status
    """
    
    def __init__(self, parent: tk.Widget, 
                 num_vehicles: int = 2,
                 callbacks: ConnectionCallbacks = None,
                 theme: Theme = None):
        self.num_vehicles = num_vehicles
        self.callbacks = callbacks or ConnectionCallbacks()
        
        self._vehicle_panels: Dict[int, VehicleConnectionPanel] = {}
        self._master_controls_frame: Optional[tk.Frame] = None
        
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the fleet connection panel."""
        c = self.theme.colors
        
        self.frame = tk.Frame(self.parent, bg=c.bg_dark)
        
        # Master controls at top
        self._build_master_controls()
        
        # Scrollable area for vehicle panels
        self._build_vehicle_list()
    
    def _build_master_controls(self) -> None:
        """Build master control buttons."""
        c = self.theme.colors
        
        self._master_controls_frame = ThemedLabelFrame(
            self.frame,
            text="🎛️ Fleet Connection Controls",
            theme=self.theme
        )
        self._master_controls_frame.pack(fill='x', padx=10, pady=10)
        
        content = tk.Frame(self._master_controls_frame, bg=c.bg_medium)
        content.pack(fill='x', padx=8, pady=8)
        
        # Row 1: Add vehicle and fleet info
        row1 = tk.Frame(content, bg=c.bg_medium)
        row1.pack(fill='x', pady=(0, 5))
        
        ThemedLabel(
            row1,
            text=f"Vehicles: {self.num_vehicles}",
            style='normal',
            theme=self.theme
        ).pack(side='left', padx=(0, 20))
        
        ThemedButton(
            row1,
            text="➕ Add Vehicle",
            button_type='command',
            command=self._add_vehicle,
            padx=10,
            pady=3
        ).pack(side='left', padx=(0, 5))
        
        ThemedButton(
            row1,
            text="➖ Remove Last",
            button_type='warning',
            command=self._remove_vehicle,
            padx=10,
            pady=3
        ).pack(side='left')
        
        # Row 2: Batch operations
        row2 = tk.Frame(content, bg=c.bg_medium)
        row2.pack(fill='x', pady=(5, 0))
        
        ThemedButton(
            row2,
            text="🔌 Connect All",
            button_type='command',
            command=self._connect_all,
            padx=12,
            pady=4
        ).pack(side='left', expand=True, fill='x', padx=(0, 3))
        
        ThemedButton(
            row2,
            text="📤 Upload All",
            button_type='warning',
            command=self._upload_all,
            padx=12,
            pady=4
        ).pack(side='left', expand=True, fill='x', padx=3)
        
        ThemedButton(
            row2,
            text="▶️ Start All",
            button_type='start',
            command=self._start_all,
            padx=12,
            pady=4
        ).pack(side='left', expand=True, fill='x', padx=3)
        
        ThemedButton(
            row2,
            text="⬛ Stop All",
            button_type='stop',
            command=self._stop_all,
            padx=12,
            pady=4
        ).pack(side='left', expand=True, fill='x', padx=(3, 0))
    
    def _build_vehicle_list(self) -> None:
        """Build the scrollable list of vehicle panels."""
        c = self.theme.colors
        
        # Create canvas for scrolling
        canvas_frame = tk.Frame(self.frame, bg=c.bg_dark)
        canvas_frame.pack(fill='both', expand=True, padx=10, pady=(0, 10))
        
        canvas = tk.Canvas(canvas_frame, bg=c.bg_dark, highlightthickness=0)
        scrollbar = ttk.Scrollbar(canvas_frame, orient='vertical', command=canvas.yview)
        
        self._vehicles_frame = tk.Frame(canvas, bg=c.bg_dark)
        
        canvas.create_window((0, 0), window=self._vehicles_frame, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')
        
        # Configure scrolling
        def _on_frame_configure(event):
            canvas.configure(scrollregion=canvas.bbox("all"))
        
        self._vehicles_frame.bind("<Configure>", _on_frame_configure)
        
        # Mouse wheel scrolling
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        
        # Store reference for adding vehicles
        self._canvas = canvas
        
        # Add initial vehicle panels
        for i in range(self.num_vehicles):
            self._add_vehicle_panel(i)
    
    def _add_vehicle_panel(self, car_id: int) -> None:
        """Add a vehicle connection panel."""
        panel = VehicleConnectionPanel(
            self._vehicles_frame,
            car_id=car_id,
            callbacks=self.callbacks,
            theme=self.theme
        )
        panel.pack(fill='x', pady=(0, 10))
        self._vehicle_panels[car_id] = panel
    
    def _add_vehicle(self) -> None:
        """Add a new vehicle slot."""
        new_id = len(self._vehicle_panels)
        self._add_vehicle_panel(new_id)
        self.num_vehicles += 1
    
    def _remove_vehicle(self) -> None:
        """Remove the last vehicle slot."""
        if self.num_vehicles > 1:
            last_id = self.num_vehicles - 1
            if last_id in self._vehicle_panels:
                self._vehicle_panels[last_id].destroy()
                del self._vehicle_panels[last_id]
            self.num_vehicles -= 1
    
    def _connect_all(self) -> None:
        """Connect to all vehicles."""
        for panel in self._vehicle_panels.values():
            panel._on_connect()
    
    def _upload_all(self) -> None:
        """Upload files to all connected vehicles."""
        for panel in self._vehicle_panels.values():
            if panel.is_connected:
                panel._on_upload()
    
    def _start_all(self) -> None:
        """Start all connected vehicles."""
        for panel in self._vehicle_panels.values():
            if panel.is_connected:
                panel._on_start()
    
    def _stop_all(self) -> None:
        """Stop all running vehicles."""
        for panel in self._vehicle_panels.values():
            if panel.is_running:
                panel._on_stop()
    
    def get_panel(self, car_id: int) -> Optional[VehicleConnectionPanel]:
        """Get a specific vehicle panel."""
        return self._vehicle_panels.get(car_id)
    
    def get_all_configs(self) -> List[VehicleConnectionConfig]:
        """Get configurations from all vehicle panels."""
        return [panel._get_current_config() for panel in self._vehicle_panels.values()]
