"""
Main Application for QCar Fleet Controller.

This module contains the main application class that orchestrates
all components of the fleet controller GUI.
"""

import tkinter as tk
from tkinter import messagebox
import threading
import time
import subprocess
import sys
from pathlib import Path
from typing import Dict, Optional, Set, Any
from dataclasses import dataclass

from .config import AppConfig, VehicleConfig
from .theme import Theme, DEFAULT_THEME
from .widgets import (
    CarState, CarPanelCallbacks, CarPanelWidget,
    FleetControlCallbacks, FleetControlsWidget,
    FleetStatus, StatusPanelWidget, LogPanelWidget, HeaderWidget,
    ScrollableFrame, ThemedLabel,
)
from .controllers import QCarRemoteController, ManualInputController


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
            telemetry_buffer_size=self.config.network.telemetry_buffer_size
        )
        self._remote.websocket_port = ws_port
        self._input = ManualInputController(self.config.manual_control)
        
        # Set callbacks
        self._remote.gui_controller = self  # For backward compatibility
        
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
        self._plot_all_process: Optional[subprocess.Popen] = None
        
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
            self.root,
            title="QCar Fleet Controller",
            theme=self.theme
        )
        self._header.pack(fill='x')
        
        # Main content area
        main_frame = tk.Frame(self.root, bg=c.bg_dark)
        main_frame.pack(fill='both', expand=True, padx=15, pady=10)
        
        # Left panel - Car controls
        left_panel = tk.Frame(main_frame, bg=c.bg_dark)
        left_panel.pack(side='left', fill='both', expand=True, padx=(0, 10))
        
        # Scrollable car panels
        self._scrollable = ScrollableFrame(left_panel, theme=self.theme)
        self._scrollable.pack(fill='both', expand=True, pady=(10, 0))
        
        # Initial "waiting for cars" message
        self._show_waiting_message()
        
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
            left_panel,
            callbacks=fleet_callbacks,
            theme=self.theme
        )
        self._fleet_controls.pack(fill='x', pady=(10, 5))
        
        # Right panel - Status and log
        right_panel = tk.Frame(main_frame, bg=c.bg_dark, width=450)
        right_panel.pack(side='right', fill='both', padx=(10, 0))
        right_panel.pack_propagate(False)

        # Quick access button for live distributed observer plotting
        plot_btn = tk.Button(
            right_panel,
            text="📈 Plot All Observer",
            command=self._launch_plot_all_observer_viewer,
            bg=self.theme.colors.accent_blue,
            fg='white',
            activebackground=self.theme.colors.accent_green,
            relief='flat',
            padx=10,
            pady=6,
        )
        plot_btn.pack(fill='x', pady=(0, 10))
        
        # Status panel
        self._status_panel = StatusPanelWidget(right_panel, theme=self.theme)
        self._status_panel.pack(fill='x', pady=(0, 10))
        
        # Log panel
        self._log_panel = LogPanelWidget(right_panel, theme=self.theme)
        self._log_panel.pack(fill='both', expand=True)
        
        # Bind keyboard for manual control
        self._input.bind_keyboard(self.root)
    
    def _show_waiting_message(self) -> None:
        """Show waiting for vehicles message."""
        if self._scrollable:
            self._no_cars_label = ThemedLabel(
                self._scrollable.content,
                text="⏳ Waiting for vehicles to connect...\n\nNo vehicles currently connected",
                style='muted',
                theme=self.theme,
                justify='center',
                pady=50
            )
            self._no_cars_label.pack(fill='both', expand=True)
    
    def _hide_waiting_message(self) -> None:
        """Hide waiting for vehicles message."""
        if self._no_cars_label:
            self._no_cars_label.destroy()
            self._no_cars_label = None
    
    def _start_server(self) -> None:
        """Start the remote controller server."""
        self._remote.start_server(self.config.gui.default_num_cars)
        self.log(f"Server started on {self.config.network.host_ip}:{self.config.network.base_port}", 'SUCCESS')
        self.log(f"Waiting for {self.config.gui.default_num_cars} cars to connect...", 'INFO')
    
    def _start_update_loop(self) -> None:
        """Start the background update loop."""
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()
    
    # ========== Car Panel Management ==========
    
    def _on_car_disconnected(self, car_id: int) -> None:
        """Handle car disconnection - clear manual mode state."""
        if car_id in self._manual_mode_active:
            del self._manual_mode_active[car_id]
            self.log(f"Car {car_id} disconnected - cleared manual mode state", 'WARNING')
    

    
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
            on_toggle_scopes=self._toggle_scopes_car,
            on_toggle_remote_plot_local=self._toggle_remote_plot_local,
            on_toggle_remote_plot_fleet=self._toggle_remote_plot_fleet,
            # Runtime switching callbacks
            on_set_local_observer=self._set_local_observer,
            on_set_fleet_observer=self._set_fleet_observer,
            on_set_controller=self._set_controller,
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
                self.log(f"Car {car_id} disconnected", 'WARNING')
        
        # Add panels for newly connected cars
        for car_id in current_connected:
            if car_id not in self._car_panels:
                self._hide_waiting_message()
                
                callbacks = self._create_car_panel_callbacks(car_id)
                panel = CarPanelWidget(
                    self._scrollable.content,
                    car_id=car_id,
                    callbacks=callbacks,
                    theme=self.theme
                )
                panel.pack(fill='x', pady=(0, 15))
                self._car_panels[car_id] = panel
                
                self.log(f"Car {car_id} connected", 'SUCCESS')
        
        # Show waiting message if no cars connected
        if not current_connected and not self._no_cars_label:
            self._show_waiting_message()
        
        self._connected_cars = current_connected
    
    def _update_car_states(self) -> None:
        """Update car panel states from telemetry."""
        for car_id, panel in self._car_panels.items():
            telemetry = self._remote.get_telemetry(car_id)
            
            if telemetry:
                state = CarState(
                    car_id=car_id,
                    connected=True,
                    state=telemetry.get('state', 'Unknown'),
                    position=(telemetry.get('x', 0), telemetry.get('y', 0)),
                    velocity=telemetry.get('v', 0),
                    heading=telemetry.get('th', 0),
                    throttle=telemetry.get('u', 0),
                    steering=telemetry.get('delta', 0),
                    v2v_active=telemetry.get('v2v_active', False),
                    v2v_peers=telemetry.get('v2v_peers', 0),
                    platoon_enabled=telemetry.get('platoon_enabled', False),
                    platoon_is_leader=telemetry.get('platoon_is_leader', False),
                    platoon_position=telemetry.get('platoon_position'),
                    manual_mode=self._manual_mode_active.get(car_id, False),
                    perception_active=telemetry.get('perception_active', False),
                    scopes_active=telemetry.get('scopes_active', False),
                    # Observer and Controller types
                    local_observer_type=telemetry.get('local_observer_type', 'unknown'),
                    fleet_observer_type=telemetry.get('fleet_observer_type', 'unknown'),
                    longitudinal_ctrl_type=telemetry.get('longitudinal_ctrl_type', 'unknown'),
                    lateral_ctrl_type=telemetry.get('lateral_ctrl_type', 'unknown'),
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
                if not telemetry or not telemetry.get('v2v_active', False):
                    all_v2v_active = False
                    break
        
        # Enable/disable fleet plot button for all panels
        for panel in self._car_panels.values():
            if hasattr(panel, '_scopes_control'):
                panel._scopes_control.set_fleet_button_enabled(all_v2v_active)

    def on_close(self) -> None:
        """Cleanup resources on application close."""
        self.log("Shutting down...", 'INFO')
        self._running = False
        
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
            self.log(f"✅ Started Car {car_id}", 'SUCCESS')
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Failed to start Car {car_id}", 'ERROR')
    
    def _stop_car(self, car_id: int) -> None:
        """Stop a car."""
        if self._remote.stop_car(car_id):
            self._commands_sent_gui += 1
            self.log(f"🛑 Stopped Car {car_id}", 'SUCCESS')
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Failed to stop Car {car_id}", 'ERROR')
    
    def _calibrate_car(self, car_id: int) -> None:
        """Calibrate GPS for a car."""
        if self._remote.send_command(car_id, {'type': 'calibrate'}):
            self._commands_sent_gui += 1
            self.log(f"📍 GPS Calibration started for Car {car_id}", 'INFO')
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Failed to calibrate Car {car_id}", 'ERROR')
    
    def _set_velocity(self, car_id: int, velocity: float) -> None:
        """Set velocity for a car."""
        cfg = self.config.vehicle
        if cfg.min_velocity <= velocity <= cfg.max_velocity:
            if self._remote.set_velocity(car_id, velocity):
                self._commands_sent_gui += 1
                self.log(f"🎯 Set Car {car_id} velocity to {velocity:.2f} m/s", 'SUCCESS')
            else:
                self._commands_failed_gui += 1
                self.log(f"❌ Failed to set velocity for Car {car_id}", 'ERROR')
        else:
            self.log(f"❌ Invalid velocity {velocity:.2f} (must be {cfg.min_velocity}-{cfg.max_velocity} m/s)", 'ERROR')
    
    def _set_path(self, car_id: int, nodes: list) -> None:
        """Set path for a car."""
        if len(nodes) >= 2:
            if self._remote.set_path(car_id, nodes):
                self._commands_sent_gui += 1
                self.log(f"🛤️ Set Car {car_id} path: {nodes}", 'SUCCESS')
            else:
                self._commands_failed_gui += 1
                self.log(f"❌ Failed to set path for Car {car_id}", 'ERROR')
        else:
            self.log("❌ Path must have at least 2 nodes", 'ERROR')
    
    def _set_initial_position(self, car_id: int, x: float, y: float, 
                               theta: float, calibrate: bool) -> None:
        """Set initial position for a car."""
        if self._remote.set_initial_position(car_id, x, y, theta, calibrate):
            self._commands_sent_gui += 1
            mode = "with GPS calibration" if calibrate else "without GPS calibration"
            self.log(f"📍 Set initial position for Car {car_id}: ({x:.2f}, {y:.2f}, θ={theta:.2f}) {mode}", 'SUCCESS')
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Failed to set initial position for Car {car_id}", 'ERROR')
    
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
        control_type = panel.control_type if panel else 'keyboard'
        
        if self._remote.enable_manual_mode(car_id, control_type):
            self._manual_mode_active[car_id] = True
            
            # Set input controller type
            if not self._input.set_control_type(control_type):
                self.log(f"Warning: Could not initialize {control_type} controller", 'WARNING')
            
            # Start input loop
            self._input.start(car_id, self._send_manual_control)
            
            self._commands_sent_gui += 1
            self.log(f"🎮 Car {car_id}: Manual mode ENABLED ({control_type.upper()})", 'SUCCESS')
            self.log(self._input.get_help_text(), 'INFO')
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Car {car_id}: Failed to enable manual mode", 'ERROR')
    
    def _disable_manual_mode(self, car_id: int) -> None:
        """Disable manual mode for a car."""
        if self._remote.disable_manual_mode(car_id):
            self._manual_mode_active[car_id] = False
            self._input.stop()
            
            self._commands_sent_gui += 1
            self.log(f"🎮 Car {car_id}: Manual mode DISABLED", 'INFO')
        else:
            self._commands_failed_gui += 1
            self.log(f"❌ Car {car_id}: Failed to disable manual mode", 'ERROR')
    
    def _send_manual_control(self, car_id: int, throttle: float, steering: float) -> None:
        """Send manual control command."""
        self._remote.send_manual_control(car_id, throttle, steering)
    
    def _update_control_type(self, car_id: int, control_type: str) -> None:
        """Update control type for a car."""
        self.log(f"Car {car_id}: Manual control type set to {control_type.upper()}", 'CONFIG')
    
    # ========== Individual Perception Control ==========
    
    def _toggle_perception_car(self, car_id: int) -> None:
        """Toggle perception for a specific car."""
        # Check current state from telemetry
        telemetry = self._remote.get_telemetry(car_id)
        is_active = telemetry.get('perception_active', False) if telemetry else False
        
        if is_active:
            self._disable_perception_car(car_id)
        else:
            self._activate_perception_car(car_id)
    
    def _activate_perception_car(self, car_id: int) -> None:
        """Activate perception for a specific car."""
        if self._remote.activate_perception(car_id):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Perception activation sent", 'SUCCESS')
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to activate perception", 'ERROR')
    
    def _disable_perception_car(self, car_id: int) -> None:
        """Disable perception for a specific car."""
        if self._remote.disable_perception(car_id):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Perception disabled", 'INFO')
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to disable perception", 'ERROR')
    
    # ========== Individual Scopes Control ==========
    
    def _toggle_scopes_car(self, car_id: int) -> None:
        """Toggle estimation scopes for a specific car."""
        # Check current state from telemetry
        telemetry = self._remote.get_telemetry(car_id)
        is_active = telemetry.get('scopes_active', False) if telemetry else False
        
        if is_active:
            self._disable_scopes_car(car_id)
        else:
            self._activate_scopes_car(car_id)
    
    def _activate_scopes_car(self, car_id: int) -> None:
        """Activate estimation scopes for a specific car."""
        command = {
            'type': 'activate_scopes',
            'preset_names': ['local_state', 'local_control']
        }
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Scopes activation sent", 'SUCCESS')
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to activate scopes", 'ERROR')
    
    def _disable_scopes_car(self, car_id: int) -> None:
        """Disable estimation scopes for a specific car."""
        if self._remote.send_command(car_id, {'type': 'disable_scopes'}):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Scopes disabled", 'INFO')
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to disable scopes", 'ERROR')
    
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
        preset_names = ['local_state', 'local_control']
        
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
                if hasattr(panel, '_scopes_control'):
                    panel._scopes_control.set_remote_local_active(True)
            
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Local remote plot enabled", 'SUCCESS')
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to enable local remote plot", 'ERROR')
    
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
                if hasattr(panel, '_scopes_control'):
                    panel._scopes_control.set_remote_local_active(False)
            
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Local remote plot disabled", 'INFO')
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to disable local remote plot", 'ERROR')
    
    def _enable_remote_plot_fleet(self, car_id: int) -> None:
        """Enable fleet scope streaming from vehicle and open viewer."""
        preset_names = ['fleet_state', 'fleet_consensus']
        
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
                if hasattr(panel, '_scopes_control'):
                    panel._scopes_control.set_remote_fleet_active(True)
            
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Fleet remote plot enabled", 'SUCCESS')
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to enable fleet remote plot", 'ERROR')
    
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
                if hasattr(panel, '_scopes_control'):
                    panel._scopes_control.set_remote_fleet_active(False)
            
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Fleet remote plot disabled", 'INFO')
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to disable fleet remote plot", 'ERROR')
    
    # ========== Runtime Switching Commands ==========
    
    def _set_local_observer(self, car_id: int, observer_type: str) -> None:
        """Set local observer type for a specific car."""
        command = {
            'type': 'set_local_observer',
            'observer_type': observer_type
        }
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Local observer → {observer_type}", 'SUCCESS')
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to set local observer", 'ERROR')
    
    def _set_fleet_observer(self, car_id: int, observer_type: str) -> None:
        """Set fleet observer type for a specific car."""
        command = {
            'type': 'set_fleet_observer',
            'observer_type': observer_type
        }
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Fleet observer → {observer_type}", 'SUCCESS')
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to set fleet observer", 'ERROR')
    
    def _set_controller(self, car_id: int, category: str, controller_type: str) -> None:
        """Set controller type for a specific car."""
        command = {
            'type': 'set_controller',
            'category': category,  # 'longitudinal' or 'lateral'
            'controller_type': controller_type
        }
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: {category.capitalize()} controller → {controller_type}", 'SUCCESS')
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to set {category} controller", 'ERROR')
    
    # ========== Fleet Commands ==========
    
    def _start_all_cars(self) -> None:
        """Start all connected cars."""
        results = self._remote.start_all_cars()
        successes = sum(1 for s in results.values() if s)
        self._commands_sent_gui += successes
        self._commands_failed_gui += len(results) - successes
        self.log(f"▶️ Start all: {successes}/{len(results)} cars started", 'INFO')
    
    def _stop_all_cars(self) -> None:
        """Stop all connected cars."""
        results = self._remote.stop_all_cars()
        successes = sum(1 for s in results.values() if s)
        self._commands_sent_gui += successes
        self._commands_failed_gui += len(results) - successes
        self.log(f"⬛ Stop all: {successes}/{len(results)} cars stopped", 'INFO')
    
    # ========== Platoon Commands ==========
    
    def _update_platoon_position(self, car_id: int, position: int) -> None:
        """Update platoon position for a car."""
        if self._platoon_config.formation is None:
            self._platoon_config.formation = {}
        
        self._platoon_config.formation[car_id] = position
        role = "LEADER" if position == 1 else "FOLLOWER"
        self.log(f"Car {car_id} platoon config: Position {position} ({role})", 'CONFIG')
    
    def _setup_platoon(self) -> None:
        """Setup platoon formation."""
        # Collect positions from car panels
        formation = {}
        for car_id, panel in self._car_panels.items():
            formation[car_id] = panel.platoon_position
        
        if not formation:
            self.log("❌ No vehicles available for platoon", 'ERROR')
            return
        
        # Validate formation
        positions = sorted(formation.values())
        if not positions or positions[0] != 1:
            self.log("❌ Position 1 (leader) must be assigned", 'ERROR')
            return
        
        # Find leader
        leader_id = next((cid for cid, pos in formation.items() if pos == 1), None)
        
        # Send formation
        self.log(f"📊 Setting up platoon formation: {formation}", 'INFO')
        self.log(f"👑 Leader: Car {leader_id}", 'INFO')
        
        results = self._remote.setup_global_platoon_formation(formation)
        
        success_count = sum(1 for s in results.values() if s)
        for car_id, success in results.items():
            position = formation.get(car_id, 0)
            role = "LEADER" if position == 1 else f"FOLLOWER (pos {position})"
            if success:
                self.log(f"✅ Car {car_id}: Formation configured as {role}", 'SUCCESS')
            else:
                self.log(f"❌ Car {car_id}: Failed to configure {role}", 'ERROR')
        
        if success_count == len(formation):
            self._platoon_config.formation = formation
            self._platoon_config.leader_id = leader_id
            self._platoon_config.setup_complete = True
            self.log(f"🎉 Platoon formation setup complete!", 'SUCCESS')
        else:
            self._platoon_config.setup_complete = False
            self.log(f"⚠️ Partial setup: {success_count}/{len(formation)} configured", 'WARNING')
    
    def _trigger_platoon(self) -> None:
        """Trigger platoon start."""
        if not self._platoon_config.setup_complete:
            self.log("❌ Platoon not set up - run Setup Platoon first", 'ERROR')
            return
        
        formation = self._platoon_config.formation
        leader_id = self._platoon_config.leader_id
        
        self.log(f"🚀 Triggering platoon start with formation: {formation}", 'INFO')
        
        success_count = 0
        for car_id in formation.keys():
            result = self._remote.start_platoon_mode(car_id, leader_id)
            if result.get('status') == 'success':
                role = "LEADER" if car_id == leader_id else "FOLLOWER"
                self.log(f"✅ Car {car_id}: Platoon started ({role})", 'SUCCESS')
                success_count += 1
            else:
                self.log(f"❌ Car {car_id}: {result.get('message', 'Failed')}", 'ERROR')
        
        if success_count == len(formation):
            self.log(f"🎉 Platoon started successfully! {success_count}/{len(formation)} vehicles active", 'SUCCESS')
        else:
            self.log(f"⚠️ Partial start: {success_count}/{len(formation)} vehicles started", 'WARNING')
    
    def _disable_all_platoons(self) -> None:
        """Disable all platoons."""
        results = self._remote.disable_all_platoons()
        successes = sum(1 for s in results.values() if s)
        self._platoon_config.setup_complete = False
        self.log(f"🚗 Disabled platoons: {successes}/{len(results)} cars", 'INFO')
    
    # ========== V2V Commands ==========
    
    def _activate_v2v(self) -> None:
        """Activate V2V communication."""
        if len(self._connected_cars) < 2:
            self.log("❌ V2V requires at least 2 connected vehicles", 'ERROR')
            messagebox.showwarning("V2V Error", "V2V requires at least 2 connected vehicles")
            return
        
        self._fleet_controls.set_v2v_activating(True)
        self.log("📡 Activating V2V communication...", 'INFO')
        
        success_count = 0
        connected_list = list(self._connected_cars)
        
        for car_id in connected_list:
            peers = [cid for cid in connected_list if cid != car_id]
            
            # Get peer IPs
            vehicle_ips = []
            for peer_id in peers:
                status = self._remote.get_car_status(peer_id)
                if status and status.get('address'):
                    vehicle_ips.append(status['address'][0])
                else:
                    vehicle_ips.append(f"192.168.1.{100 + peer_id}")
            
            command = {
                'command': 'activate_v2v',
                'peer_vehicles': peers,
                'peer_ips': vehicle_ips,
                'my_id': car_id
            }
            
            if self._remote.send_command(car_id, command):
                success_count += 1
                self._commands_sent_gui += 1
            else:
                self._commands_failed_gui += 1
        
        if success_count > 0:
            self.log(f"V2V activation sent to {success_count}/{len(connected_list)} vehicles", 'SUCCESS')
            # Set timeout to reset button
            self.root.after(10000, self._v2v_activation_timeout)
        else:
            self.log("Failed to send V2V activation", 'ERROR')
            self._fleet_controls.set_v2v_activating(False)
    
    def _v2v_activation_timeout(self) -> None:
        """Handle V2V activation timeout."""
        # Check if still waiting for V2V
        if self._fleet_controls:
            self._fleet_controls.set_v2v_activating(False)
            self.log("⏰ V2V activation timeout - button re-enabled", 'WARNING')
    
    def _disable_v2v(self) -> None:
        """Disable V2V communication."""
        self.log("📡 Disabling V2V communication...", 'INFO')
        
        success_count = 0
        for car_id in self._connected_cars:
            if self._remote.send_command(car_id, {'command': 'disable_v2v'}):
                success_count += 1
                self._commands_sent_gui += 1
            else:
                self._commands_failed_gui += 1
        
        if success_count > 0:
            self.log(f"✅ V2V disabled for {success_count} vehicles", 'SUCCESS')
            self._fleet_controls.reset_v2v_buttons()
            self._v2v_status.clear()
            self._v2v_network_established = False  # Reset so message can be logged again
        else:
            self.log("❌ Failed to disable V2V", 'ERROR')
    
    def _activate_perception(self) -> None:
        """Activate perception system (YOLO) for all connected vehicles."""
        self.log("👁️ Activating perception systems...", 'INFO')
        
        success_count = 0
        failed_cars = []
        
        for car_id in self._connected_cars:
            if self._remote.activate_perception(car_id):
                success_count += 1
                self._commands_sent_gui += 1
                self.log(f"✅ Car {car_id}: Perception activation command sent", 'SUCCESS')
            else:
                failed_cars.append(car_id)
                self._commands_failed_gui += 1
        
        if success_count > 0:
            self.log(f"✅ Perception activated for {success_count}/{len(self._connected_cars)} vehicles", 'SUCCESS')
        
        if failed_cars:
            self.log(f"❌ Failed to activate perception for cars: {failed_cars}", 'ERROR')
    
    def _disable_perception(self) -> None:
        """Disable perception system for all connected vehicles."""
        self.log("👁️ Disabling perception systems...", 'INFO')
        
        success_count = 0
        for car_id in self._connected_cars:
            if self._remote.disable_perception(car_id):
                success_count += 1
                self._commands_sent_gui += 1
            else:
                self._commands_failed_gui += 1
        
        if success_count > 0:
            self.log(f"✅ Perception disabled for {success_count} vehicles", 'SUCCESS')
        else:
            self.log("❌ Failed to disable perception", 'ERROR')
    
    # ========== Callback Handlers (for backward compatibility) ==========
    
    def process_v2v_status(self, car_id: int, v2v_data: dict) -> None:
        """Process V2V status from a vehicle."""
        status = v2v_data.get('status', 'unknown')
        
        if status == 'connected':
            peers = v2v_data.get('connected_peers', 0)
            
            # Check for change before logging to avoid spam
            # Check for change before logging to avoid spam
            current_status = self._v2v_status.get(car_id, {})
            
            # Update status
            self._v2v_status[car_id] = {'status': 'connected', 'peers': peers}
            
            # ONLY log if we weren't connected before (debounce updates)
            if current_status.get('status') != 'connected':
                self.log(f"📡 Car {car_id}: V2V connected ({peers} peers)", 'SUCCESS')
            # Else: just updated peer count silently
            
            # Check if all vehicles connected
            self._check_v2v_network()
            
        elif status == 'failed':
            error = v2v_data.get('error', 'unknown')
            self._v2v_status[car_id] = {'status': 'failed', 'error': error}
            self.log(f"❌ Car {car_id}: V2V failed - {error}", 'ERROR')
            
        elif status == 'disconnected':
            self._v2v_status[car_id] = {'status': 'disconnected'}
            
            # self.log(f"📡 Car {car_id}: V2V disconnected", 'WARNING')
    
    def _check_v2v_network(self) -> None:
        """Check if V2V network is fully established."""
        if len(self._connected_cars) < 2:
            return
        
        connected = [
            cid for cid, status in self._v2v_status.items()
            if status.get('status') == 'connected'
        ]
        
        expected_peers = len(self._connected_cars) - 1
        fully_connected = all(
            self._v2v_status.get(cid, {}).get('peers', 0) >= expected_peers
            for cid in connected
        )
        
        if len(connected) == len(self._connected_cars) and fully_connected:
            self._fleet_controls.set_v2v_connected(True)
            # Only log once when network becomes established
            if not getattr(self, '_v2v_network_established', False):
                self._v2v_network_established = True
                self.log("✅ V2V Network Fully Established", 'SUCCESS')
    
    def process_platoon_setup_confirmation(self, car_id: int, platoon_data: dict) -> None:
        """Process platoon setup confirmation from a vehicle."""
        position = platoon_data.get('position')
        is_leader = platoon_data.get('is_leader', False)
        
        role = "LEADER" if is_leader else f"FOLLOWER-{position}"
        self.log(f"✅ Car {car_id} platoon confirmed: {role}", 'SUCCESS')
    
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
            commands_sent=self._commands_sent_gui + fleet['commands_sent_total'],
            commands_failed=self._commands_failed_gui + fleet['commands_failed_total'],
            success_rate=fleet['success_rate'],
            uptime_seconds=time.time() - self._start_time,
            avg_telemetry_rate=fleet['avg_telemetry_rate_hz'],
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
            commands_sent=self._commands_sent_gui + fleet['commands_sent_total'],
            commands_failed=self._commands_failed_gui + fleet['commands_failed_total'],
            uptime=time.time() - self._start_time,
            telemetry_rate=fleet['avg_telemetry_rate_hz']
        )

    def _launch_plot_all_observer_viewer(self) -> None:
        """Launch the standalone live observer plotting tool."""
        try:
            if self._plot_all_process and self._plot_all_process.poll() is None:
                self.log("Plot All Observer viewer is already running", 'INFO')
                return

            # Search upwards from this file to find the repo root script.
            script_path = None
            repo_root = None
            for parent in Path(__file__).resolve().parents:
                candidate = parent / 'plot_all_observer_viewer.py'
                if candidate.exists():
                    script_path = candidate
                    repo_root = parent
                    break

            if script_path is None or repo_root is None:
                self.log("Plot script not found: plot_all_observer_viewer.py", 'ERROR')
                return

            self._plot_all_process = subprocess.Popen(
                [sys.executable, str(script_path)],
                cwd=str(repo_root),
                creationflags=getattr(subprocess, 'CREATE_NEW_PROCESS_GROUP', 0),
            )
            self.log("Started Plot All Observer viewer", 'SUCCESS')

        except Exception as e:
            self.log(f"Failed to launch Plot All Observer viewer: {e}", 'ERROR')
    
    # ========== Logging ==========
    
    def log(self, message: str, level: str = 'INFO') -> None:
        """Log a message."""
        if self._log_panel:
            self._log_panel.log(message, level)
    
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
        
        # Close remote controller
        self._remote.close()

        # Stop external plot viewer if started by this GUI
        if self._plot_all_process and self._plot_all_process.poll() is None:
            try:
                self._plot_all_process.terminate()
            except Exception:
                pass
        
        # Destroy window
        self.root.destroy()


def create_app(num_cars: int = 5, host_ip: str = '0.0.0.0', base_port: int = 5000, ws_port: int = 8080) -> QCarFleetController:
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
        network=NetworkConfig(host_ip=host_ip, base_port=base_port)
    )
    
    app = QCarFleetController(root, config, ws_port=ws_port)
    
    return app


def main():
    """Main entry point."""
    # Configuration
    NUM_CARS = 5
    HOST_IP = '0.0.0.0'
    BASE_PORT = 5000
    
    app = create_app(num_cars=NUM_CARS, host_ip=HOST_IP, base_port=BASE_PORT)
    
    # Log startup
    app.log("QCar Fleet Controller started", 'SUCCESS')
    app.log(f"Listening on ports {BASE_PORT}-{BASE_PORT + NUM_CARS - 1}", 'INFO')
    app.log("Features: Command validation, platoon control, V2V, manual control", 'INFO')
    
    app.root.mainloop()


if __name__ == '__main__':
    main()
