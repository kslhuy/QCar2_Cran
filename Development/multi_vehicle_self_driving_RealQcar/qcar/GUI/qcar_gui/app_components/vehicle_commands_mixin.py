class VehicleCommandsMixin:
    """Mixin for individual vehicle remote commands."""

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
                self.log(f"🎯 Set Car {car_id} velocity to {velocity:.2f} m/s", "SUCCESS")
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
            if not getattr(self, "_input").set_control_type(control_type):
                self.log(f"Warning: Could not initialize {control_type} controller", "WARNING")

            # Start input loop
            self._input.start(car_id, self._send_manual_control)

            self._commands_sent_gui += 1
            self.log(f"🎮 Car {car_id}: Manual mode ENABLED ({control_type.upper()})", "SUCCESS")
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

    def _send_manual_control(self, car_id: int, throttle: float, steering: float) -> None:
        """Send manual control command."""
        self._remote.send_manual_control(car_id, throttle, steering)

    def _update_control_type(self, car_id: int, control_type: str) -> None:
        """Update control type for a car."""
        self.log(f"Car {car_id}: Manual control type set to {control_type.upper()}", "CONFIG")

    def _toggle_perception_car(self, car_id: int) -> None:
        """Toggle perception for a specific car."""
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

    def _toggle_probing_car(self, car_id: int) -> None:
        """Toggle probing for a specific car - opens observer window to see YOLO stream."""
        if not hasattr(self, "_probing_processes"):
            self._probing_processes = {}

        if car_id in self._probing_processes and self._probing_processes[car_id] is not None:
            self._stop_probing_car(car_id)
        else:
            self._start_probing_car(car_id)

    def _start_probing_car(self, car_id: int) -> None:
        """Start probing for a specific car - runs multi_probing.py."""
        import subprocess
        import os

        if not hasattr(self, "_probing_processes"):
            self._probing_processes = {}

        try:
            base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            probing_script = os.path.join(base_path, "python", "multi_probing.py")

            if not os.path.exists(probing_script):
                self.log(f"Car {car_id}: Probing script not found at {probing_script}", "ERROR")
                return

            ip = self._remote.get_car_ip(car_id)
            ip_source = "live TCP connection"

            if not ip and car_id in getattr(self, "_deployment_panels", {}):
                panel_ip = self._deployment_panels[car_id].get_ip()
                if panel_ip:
                    ip = panel_ip
                    ip_source = "deployment panel"

            if not ip:
                ip = "localhost"
                ip_source = "default (localhost)"

            self.log(f"Car {car_id}: Probing → IP={ip} (source: {ip_source})", "INFO")

            cmd = ["python", probing_script, "--car", str(car_id), "--ip", ip]
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                creationflags=subprocess.CREATE_NEW_CONSOLE,
            )

            self._probing_processes[car_id] = process
            self.log(f"Car {car_id}: Probing started (IP={ip}) - observer window opened", "SUCCESS")

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

        if car_id in self._probing_processes and self._probing_processes[car_id] is not None:
            try:
                self._probing_processes[car_id].terminate()
                self._probing_processes[car_id] = None
                self.log(f"Car {car_id}: Probing stopped", "INFO")

                if car_id in self._car_panels:
                    panel = self._car_panels[car_id]
                    if hasattr(panel, "_perception_ctrl") and panel._perception_ctrl:
                        panel._perception_ctrl.set_probing_active(False)

            except Exception as e:
                self.log(f"Car {car_id}: Error stopping probing - {e}", "ERROR")

    def _toggle_scopes_car(self, car_id: int) -> None:
        """Toggle estimation scopes for a specific car."""
        telemetry = self._remote.get_telemetry(car_id)
        is_active = telemetry.get("scopes_active", False) if telemetry else False

        if is_active:
            self._disable_scopes_car(car_id)
        else:
            self._activate_scopes_car(car_id)

    def _activate_scopes_car(self, car_id: int) -> None:
        """Activate estimation scopes for a specific car."""
        command = {"type": "activate_scopes", "preset_names": ["local_state", "local_control"]}
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
        success = self._remote.enable_scope_streaming(car_id, preset_names=preset_names, stream_rate=50.0)

        if success:
            self._remote.open_scope_viewer(car_id, preset_names)
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
        self._remote.close_scope_viewer(car_id)
        success = self._remote.disable_scope_streaming(car_id)

        if success:
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
        success = self._remote.enable_scope_streaming(car_id, preset_names=preset_names, stream_rate=50.0)

        if success:
            self._remote.open_scope_viewer(car_id, preset_names)
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
        self._remote.close_scope_viewer(car_id)
        success = self._remote.disable_scope_streaming(car_id)

        if success:
            if car_id in self._car_panels:
                panel = self._car_panels[car_id]
                if hasattr(panel, "_scopes_control"):
                    panel._scopes_control.set_remote_fleet_active(False)

            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Fleet remote plot disabled", "INFO")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to disable fleet remote plot", "ERROR")

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
        self, car_id: int, category: str, controller_type: str, state_context: str = "path"
    ) -> None:
        """Set controller type for a specific car."""
        command = {
            "type": "set_controller",
            "category": category,
            "controller_type": controller_type,
            "state_context": state_context,
        }
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: {state_context.capitalize()} {category} controller → {controller_type}", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to set {state_context} {category} controller", "ERROR")

    def _set_controller_params(
        self, car_id: int, category: str, params: dict, state_context: str = "path"
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
            self.log(f"Car {car_id}: Set {state_context} {category} params ({params_str})", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to pass {state_context} {category} params", "ERROR")

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
            self.log(f"Car {car_id}: Failed to send Online SysID [{action}] command", "ERROR")

    def _set_robust_kalmannet_dataset(
        self, car_id: int, action: str, params: dict = None
    ) -> None:
        """Send a Robust KalmanNet offline dataset command to a vehicle."""
        if params is None:
            params = {}
        params["action"] = action
        command = {
            "type": "set_params",
            "category": "robust_kalmannet_dataset",
            "params": params,
        }
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(
                f"Car {car_id}: RKNet dataset [{action}] command sent",
                "SUCCESS",
            )
        else:
            self._commands_failed_gui += 1
            self.log(
                f"Car {car_id}: Failed to send RKNet dataset [{action}] command",
                "ERROR",
            )

    def _start_online_calibration(self, car_id: int) -> None:
        """Start passive online calibration."""
        if self._remote.send_command(car_id, {"type": "enable_online_calibration", "config": {}}):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Started passive calibration collection", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to start passive calibration", "ERROR")

    def _stop_online_calibration(self, car_id: int) -> None:
        """Stop passive online calibration."""
        if self._remote.send_command(car_id, {"type": "disable_online_calibration"}):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Stopped passive calibration collection", "INFO")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to stop passive calibration", "ERROR")

    def _trigger_online_analysis(
        self, car_id: int, calibration_type: str, options: dict
    ) -> None:
        """Trigger analysis for passive online calibration."""
        command = {
            "type": "set_params",
            "category": "online_calibration",
            "params": {"action": "analyse", "calibration_type": calibration_type, "options": options},
        }
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Triggered passive analysis for {calibration_type}", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to trigger passive analysis", "ERROR")

    def _clear_online_calibration(self, car_id: int) -> None:
        """Clear passive online calibration buffer."""
        command = {
            "type": "set_params",
            "category": "online_calibration",
            "params": {"action": "clear"},
        }
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Cleared passive calibration buffer", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to clear passive calibration buffer", "ERROR")

    def _trigger_active_calibration(
        self, car_id: int, calibration_type: str, params: dict
    ) -> None:
        """Trigger active calibration sequence."""
        command = {
            "type": "enable_calibration_mode",
            "calibration_type": calibration_type,
            "params": params,
        }
        if self._remote.send_command(car_id, command):
            self._commands_sent_gui += 1
            self.log(f"Car {car_id}: Triggered active calibration for {calibration_type}", "SUCCESS")
        else:
            self._commands_failed_gui += 1
            self.log(f"Car {car_id}: Failed to trigger active calibration", "ERROR")
