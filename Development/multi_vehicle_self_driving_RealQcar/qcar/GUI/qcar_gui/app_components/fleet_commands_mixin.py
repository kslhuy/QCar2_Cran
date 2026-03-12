from tkinter import messagebox

class FleetCommandsMixin:
    """Mixin for fleet-related commands."""

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
            if hasattr(self, 'root'):
                self.root.after(10000, self._v2v_activation_timeout)
        else:
            self.log("Failed to send V2V activation", "ERROR")
            self._fleet_controls.set_v2v_activating(False)

    def _v2v_activation_timeout(self) -> None:
        """Handle V2V activation timeout."""
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
            self._v2v_network_established = False
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
                self.log(f"✅ Car {car_id}: Perception activation command sent", "SUCCESS")
            else:
                failed_cars.append(car_id)
                self._commands_failed_gui += 1

        if success_count > 0:
            self.log(
                f"✅ Perception activated for {success_count}/{len(self._connected_cars)} vehicles",
                "SUCCESS",
            )

        if failed_cars:
            self.log(f"❌ Failed to activate perception for cars: {failed_cars}", "ERROR")

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

    def process_v2v_status(self, car_id: int, v2v_data: dict) -> None:
        """Process V2V status from a vehicle."""
        status = v2v_data.get("status", "unknown")

        if status == "connected":
            peers = v2v_data.get("connected_peers", 0)
            current_status = self._v2v_status.get(car_id, {})

            self._v2v_status[car_id] = {"status": "connected", "peers": peers}

            if current_status.get("status") != "connected":
                self.log(f"📡 Car {car_id}: V2V connected ({peers} peers)", "SUCCESS")

            self._check_v2v_network()

        elif status == "failed":
            error = v2v_data.get("error", "unknown")
            self._v2v_status[car_id] = {"status": "failed", "error": error}
            self.log(f"❌ Car {car_id}: V2V failed - {error}", "ERROR")

        elif status == "disconnected":
            self._v2v_status[car_id] = {"status": "disconnected"}

    def _check_v2v_network(self) -> None:
        """Check if V2V network is fully established."""
        if len(self._connected_cars) < 2:
            return

        connected = [
            cid for cid, status in self._v2v_status.items()
            if status.get("status") == "connected"
        ]

        expected_peers = len(self._connected_cars) - 1
        fully_connected = all(
            self._v2v_status.get(cid, {}).get("peers", 0) >= expected_peers
            for cid in connected
        )

        if len(connected) == len(self._connected_cars) and fully_connected:
            self._fleet_controls.set_v2v_connected(True)
            if not getattr(self, "_v2v_network_established", False):
                self._v2v_network_established = True
                self.log("✅ V2V Network Fully Established", "SUCCESS")

    def process_platoon_setup_confirmation(self, car_id: int, platoon_data: dict) -> None:
        """Process platoon setup confirmation from a vehicle."""
        position = platoon_data.get("position")
        is_leader = platoon_data.get("is_leader", False)

        role = "LEADER" if is_leader else f"FOLLOWER-{position}"
        self.log(f"✅ Car {car_id} platoon confirmed: {role}", "SUCCESS")
