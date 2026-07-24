"""Protocol, vehicle bridge, server registry, and terminal dashboard tests."""

import os
import socket
import sys
import time
import unittest

import msgpack

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.commands import CommandOutcome, CommandResult, CommandType, VehicleCommand
from core.types import ControllerReference, VehicleStateEstimate
from utils.ground_station.monitoring import MonitoringSnapshot
from extra.ground_station.command_handler import GroundStationCommandHandler
from extra.ground_station.dashboard import GroundStationDashboard
from extra.ground_station.server import GroundStationServer
from utils.ground_station.bridge_tcp import GroundStationClientBridge
from utils.ground_station.bridge_base import NullGroundStationBridge
from utils.ground_station.protocol import FrameDecoder, FrameType, ProtocolError, decode_frame, encode_frame
from utils.ground_station.runtime_facade import GroundStationRuntimeFacade


def _wait_for(predicate, timeout_s: float = 3.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def _snapshot(vehicle_id: int) -> MonitoringSnapshot:
    return MonitoringSnapshot(
        vehicle_id=vehicle_id,
        runtime_state="RUNNING",
        fleet_phase="disabled",
        estimate_valid=True,
        x_m=1.25,
        y_m=-0.5,
        heading_rad=0.1,
        velocity_mps=0.6,
        io_healthy=True,
        observer_healthy=True,
        v2v_status={"messages_received": 4, "packets_dropped": 1},
        fleet_summary={"peer_count": 0, "peer_health": []},
        control_reference={
            "target_x_m": 5.0,
            "target_y_m": 1.0,
            "target_heading_rad": 0.2,
            "target_velocity_mps": 0.7,
            "is_finished": False,
        },
    )


def _bridge_config(port: int, **overrides) -> dict:
    config = {
        "implementation": "tcp_client",
        "enabled": True,
        "server_host": "127.0.0.1",
        "server_port": port,
        "connect_timeout_s": 0.5,
        "reconnect_interval_s": 0.05,
        "command_queue_size": 4,
        "outbound_queue_size": 8,
        "max_frame_bytes": 4096,
        "monitoring_rate_hz": 100.0,
        "command_batch_size": 4,
    }
    config.update(overrides)
    return config


class TestGroundStationProtocol(unittest.TestCase):
    def test_incremental_frame_decoder_handles_fragmented_tcp_reads(self):
        encoded = encode_frame(FrameType.REGISTER, {"vehicle_id": 2, "session_id": "session"})
        decoder = FrameDecoder()

        self.assertEqual(decoder.feed(encoded[:3]), [])
        frames = decoder.feed(encoded[3:])

        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].frame_type, FrameType.REGISTER)
        self.assertEqual(frames[0].payload["vehicle_id"], 2)

    def test_decoder_rejects_oversized_length_prefix(self):
        decoder = FrameDecoder(max_frame_bytes=64)
        with self.assertRaisesRegex(ProtocolError, "frame size"):
            decoder.feed((65).to_bytes(4, byteorder="big"))

    def test_protocol_rejects_an_unsupported_version(self):
        encoded = msgpack.packb({"version": 2, "type": "REGISTER", "payload": {}}, use_bin_type=True)
        with self.assertRaisesRegex(ProtocolError, "protocol version"):
            decode_frame(encoded)

    def test_cli_parser_builds_only_typed_vehicle_commands(self):
        request = GroundStationCommandHandler().parse("set-velocity 3 0.8")

        self.assertEqual(request.vehicle_id, 3)
        self.assertEqual(request.action, "set-velocity")
        self.assertEqual(request.command.command_type, CommandType.SET_VELOCITY)
        self.assertEqual(request.command.payload["velocity"], 0.8)

    def test_cli_parser_builds_manual_input_command(self):
        request = GroundStationCommandHandler().parse("manual 3 0.2 -0.1")

        self.assertEqual(request.command.command_type, CommandType.MANUAL_INPUT)
        self.assertAlmostEqual(request.command.payload["throttle"], 0.2)
        self.assertAlmostEqual(request.command.payload["steering"], -0.1)

    def test_cli_parser_handles_read_only_status_request(self):
        request = GroundStationCommandHandler().parse("status 3")

        self.assertEqual(request.vehicle_id, 3)
        self.assertIsNone(request.command)
        self.assertEqual(request.action, "status")

    def test_command_handler_routes_only_typed_vehicle_commands(self):
        class _Server:
            def __init__(self):
                self.sent = None

            def send_command(self, vehicle_id, command):
                self.sent = (vehicle_id, command)
                return "sent"

        handler = GroundStationCommandHandler()
        request = handler.parse("start 4")
        server = _Server()

        self.assertEqual(handler.route(server, request), "sent")
        self.assertEqual(server.sent[0], 4)
        self.assertEqual(server.sent[1].command_type, CommandType.START)

    def test_null_bridge_has_no_transport_side_effects(self):
        bridge = NullGroundStationBridge(vehicle_id=2)

        bridge.start()
        bridge.publish_snapshot(_snapshot(2))
        self.assertEqual(bridge.drain_commands(2), [])
        self.assertFalse(bridge.get_status()["enabled"])
        bridge.stop()

    def test_runtime_facade_owns_command_pumping_acknowledgement_and_monitoring(self):
        class _Bridge:
            def __init__(self):
                self.commands = [VehicleCommand(CommandType.START, source="ground_station")]
                self.acks = []
                self.snapshots = []

            def start(self):
                return None

            def stop(self):
                return None

            def drain_commands(self, limit):
                commands, self.commands = self.commands[:limit], self.commands[limit:]
                return commands

            def publish_ack(self, result):
                self.acks.append(result)

            def publish_snapshot(self, snapshot):
                self.snapshots.append(snapshot)

            def get_status(self):
                return {"enabled": True}

        bridge = _Bridge()
        facade = GroundStationRuntimeFacade(bridge, command_batch_size=2)
        facade.process_pending(
            lambda command: CommandResult(command.command_id, 6, CommandOutcome.APPLIED, "RUNNING")
        )
        facade.publish_monitoring(
            vehicle_id=6,
            runtime_state="RUNNING",
            estimate=VehicleStateEstimate(1.0, 2.0, 3.0, 0.1, 0.4, 0.0, True),
            control_reference=ControllerReference(4.0, 5.0, 0.2, 0.6),
        )

        self.assertEqual(len(bridge.acks), 1)
        self.assertEqual(facade.last_command_result, bridge.acks[0])
        self.assertEqual(bridge.snapshots[0].runtime_state, "RUNNING")
        self.assertEqual(bridge.snapshots[0].control_reference["target_x_m"], 4.0)
        self.assertEqual(bridge.snapshots[0].last_command_result, bridge.acks[0])

    def test_manual_input_acknowledgements_are_suppressed_for_high_rate_transport(self):
        class _Bridge:
            def __init__(self):
                self.acks = []

            def publish_ack(self, result):
                self.acks.append(result)

        bridge = _Bridge()
        facade = GroundStationRuntimeFacade(bridge)
        command = VehicleCommand(
            CommandType.MANUAL_INPUT,
            {"throttle": 0.1, "steering": 0.0},
            source="ground_station",
        )
        result = CommandResult(command.command_id, 2, CommandOutcome.APPLIED, "RUNNING")

        facade.record_command_result(command, result)

        self.assertEqual(bridge.acks, [])
        self.assertEqual(facade.last_command_result, result)


class TestGroundStationBridgeAndServer(unittest.TestCase):
    def setUp(self):
        self.server = GroundStationServer("127.0.0.1", 0, max_frame_bytes=4096)
        self.server.start()
        self.bridge = GroundStationClientBridge(_bridge_config(self.server.port), vehicle_id=7)
        self.bridge.start()
        self.assertTrue(_wait_for(lambda: len(self.server.session_rows()) == 1), "vehicle bridge did not register")

    def tearDown(self):
        self.bridge.stop()
        self.server.stop()

    def test_command_snapshot_and_ack_flow_without_runtime_calls_from_network_thread(self):
        delivery = self.server.send_command(7, VehicleCommand(CommandType.START, command_id="start-7"))
        self.assertTrue(delivery.accepted, delivery.reason)
        self.assertTrue(_wait_for(lambda: self.bridge.get_status()["commands_received"] == 1))

        commands = self.bridge.drain_commands(4)
        self.assertEqual(len(commands), 1)
        self.assertEqual(commands[0].command_type, CommandType.START)
        self.assertEqual(commands[0].target_vehicle_id, 7)

        self.bridge.publish_snapshot(_snapshot(7))
        self.bridge.publish_ack(
            CommandResult("start-7", 7, CommandOutcome.APPLIED, "RUNNING")
        )
        self.assertTrue(
            _wait_for(
                lambda: (
                    self.server.session_rows()[0]["snapshot"] is not None
                    and self.server.session_rows()[0]["last_command_result"].get("command_id") == "start-7"
                )
            )
        )
        row = self.server.session_rows()[0]
        self.assertEqual(row["snapshot"]["runtime_state"], "RUNNING")
        self.assertEqual(row["last_command_result"]["outcome"], "applied")

    def test_live_duplicate_vehicle_id_is_rejected(self):
        duplicate = socket.create_connection(("127.0.0.1", self.server.port), timeout=1.0)
        try:
            duplicate.sendall(
                encode_frame(
                    FrameType.REGISTER,
                    {"vehicle_id": 7, "session_id": "duplicate", "capabilities": {}},
                    max_frame_bytes=4096,
                )
            )
            data = duplicate.recv(4096)
            frames = FrameDecoder(max_frame_bytes=4096).feed(data)
            self.assertEqual(frames[0].frame_type, FrameType.REGISTER_ACK)
            self.assertFalse(frames[0].payload["accepted"])
            self.assertEqual(frames[0].payload["reason"], "duplicate_live_vehicle_id")
        finally:
            duplicate.close()

    def test_dashboard_renders_latest_snapshot_and_stale_status(self):
        self.bridge.publish_snapshot(_snapshot(7))
        self.assertTrue(_wait_for(lambda: self.server.session_rows()[0]["snapshot"] is not None))
        dashboard = GroundStationDashboard(stale_after_s=0.1)
        rendered = dashboard.render(self.server.session_rows(), now_monotonic=time.monotonic())

        self.assertIn("RUNNING", rendered)
        self.assertIn("4/1", rendered)
        self.assertIn("5.0, 1.0; 0.70", rendered)
        self.assertIn("unavailable", rendered)
        stale = dashboard.render(self.server.session_rows(), now_monotonic=time.monotonic() + 1.0)
        self.assertIn("stale", stale)

    def test_client_reconnects_to_a_restarted_single_listener(self):
        port = self.server.port
        self.server.stop()
        self.server = GroundStationServer("127.0.0.1", port, max_frame_bytes=4096)
        self.server.start()

        self.assertTrue(
            _wait_for(
                lambda: (
                    len(self.server.session_rows()) == 1
                    and self.server.session_rows()[0]["connection_state"] == "connected"
                )
            ),
            "vehicle bridge did not reconnect and register",
        )

    def test_dashboard_retains_disconnected_vehicle_diagnostic_row(self):
        self.bridge.stop()

        self.assertTrue(
            _wait_for(
                lambda: (
                    len(self.server.session_rows()) == 1
                    and self.server.session_rows()[0]["connection_state"] == "disconnected"
                )
            )
        )
        self.assertIn("disconnected", GroundStationDashboard().render(self.server.session_rows()))

    def test_command_queue_bound_rejects_the_excess_request(self):
        bridge = GroundStationClientBridge(_bridge_config(self.server.port, command_queue_size=1), vehicle_id=9)
        first = VehicleCommand(CommandType.START, command_id="queue-first", source="ground_station", target_vehicle_id=9)
        second = VehicleCommand(CommandType.STOP, command_id="queue-second", source="ground_station", target_vehicle_id=9)

        bridge._queue_command(first.to_mapping())
        bridge._queue_command(second.to_mapping())

        self.assertEqual([item.command_id for item in bridge.drain_commands(2)], ["queue-first"])
        self.assertEqual(bridge.get_status()["commands_rejected"], 1)

    def test_manual_inputs_are_coalesced_to_the_newest_value(self):
        bridge = GroundStationClientBridge(_bridge_config(self.server.port), vehicle_id=9)
        first = VehicleCommand(
            CommandType.MANUAL_INPUT,
            {"throttle": 0.1, "steering": 0.0},
            source="ground_station",
            target_vehicle_id=9,
        )
        latest = VehicleCommand(
            CommandType.MANUAL_INPUT,
            {"throttle": 0.2, "steering": 0.1},
            source="ground_station",
            target_vehicle_id=9,
        )

        bridge._queue_command(first.to_mapping())
        bridge._queue_command(latest.to_mapping())

        drained = bridge.drain_commands(2)
        self.assertEqual([item.command_id for item in drained], [latest.command_id])
        self.assertEqual(bridge.get_status()["manual_inputs_coalesced"], 1)


if __name__ == "__main__":
    unittest.main()
