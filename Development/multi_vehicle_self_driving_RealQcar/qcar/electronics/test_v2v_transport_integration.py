"""Integration tests using the real V2V MsgPack communication class."""

import logging
import unittest

import msgpack

from electronics import ElectronicsDigitalTwin, ElectronicsV2VBridge, VehiclePlantSample
from V2V.v2v_communication import V2VCommunication, V2VMessage


class _Clock:
    now_ns = 0

    def __call__(self):
        return self.now_ns


class _Socket:
    def __init__(self):
        self.sent = []

    def sendto(self, payload, address):
        self.sent.append((bytes(payload), address))


def _board_config():
    return {
        "nav_protocol": "sensor_frame_v2",
        "nav_com_interface": "spi",
        "vehicle_interface": "uart",
        "mcu": {"nav_boot_time_s": 0.0, "com_boot_time_s": 0.0},
        "sensors": {
            "imu_rate_hz": 0.0,
            "gnss_rate_hz": 0.0,
            "magnetometer_rate_hz": 0.0,
        },
        "firmware": {"publish_rate_hz": 0.0},
    }


def _plant(timestamp_ns):
    return VehiclePlantSample(timestamp_ns, 0, 0, 0, 0, 0, 0)


class V2VCommunicationElectronicsTests(unittest.TestCase):
    def _system(self):
        clock = _Clock()
        twin = ElectronicsDigitalTwin(0, _board_config())
        twin.step(_plant(10_000_000), 0.01)
        bridge = ElectronicsV2VBridge(
            twin, {"mode": "firmware", "faults": {}}, clock_ns=clock
        )
        communication = V2VCommunication(
            0,
            logging.getLogger("electronics-v2v-test"),
            base_port=8000,
            send_intervals={"local_state": 0},
        )
        socket = _Socket()
        communication.send_socket = socket
        communication._is_active = True
        communication.peer_vehicles = [1]
        communication.peer_ips = {1: "peer-one"}
        communication.peer_ports = {1: 8001}
        communication.set_datagram_adapter(bridge)
        return clock, twin, bridge, communication, socket

    @staticmethod
    def _advance(clock, twin, communication, start=2, stop=12):
        for step in range(start, stop):
            timestamp_ns = step * 10_000_000
            clock.now_ns = timestamp_ns
            twin.step(_plant(timestamp_ns), 0.01)
            communication.pump_transport()

    def test_outbound_msgpack_keeps_existing_schema_after_com_and_radio(self):
        clock, twin, _, communication, socket = self._system()
        payload = {
            "vehicle_id": 0,
            "x": 1.0,
            "y": 2.0,
            "theta": 0.1,
            "velocity": 0.4,
            "timestamp_ref_ns": 123,
        }
        self.assertTrue(communication.send_message("local_state", payload))
        self._advance(clock, twin, communication)
        self.assertEqual(len(socket.sent), 1)
        decoded = msgpack.unpackb(socket.sent[0][0], strict_map_key=False)
        self.assertEqual(decoded["sender_id"], 0)
        self.assertEqual(decoded["message_type"], "local_state")
        self.assertEqual(decoded["data"], payload)

    def test_inbound_msgpack_reaches_registered_handler_after_com(self):
        clock, twin, bridge, communication, _ = self._system()
        received = []
        communication.register_message_handler(
            "local_state", lambda message: received.append(message)
        )
        incoming = V2VMessage(
            sender_id=1,
            message_type="local_state",
            data={"vehicle_id": 1, "timestamp_ref_ns": 321},
            seq_id=8,
            send_time_ns=99,
        )
        bridge.inbound(
            msgpack.packb(incoming.to_dict()),
            ("peer-one", 8001),
            communication._process_udp_message,
        )
        self._advance(clock, twin, communication)
        self.assertEqual(len(received), 1)
        self.assertEqual(received[0].sender_id, 1)
        self.assertEqual(received[0].seq_id, 8)
        self.assertEqual(received[0].data["timestamp_ref_ns"], 321)


if __name__ == "__main__":
    unittest.main()
