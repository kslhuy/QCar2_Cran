"""
Unit tests for the minimal V2V utilities.

Run from the qcar_refactor directory:
    python -m unittest test.unit_test_v2v
"""

import os
import socket
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.types import V2VState, VehicleStateEstimate
from utils.v2v import V2VNull, V2VUdp


def _state(timestamp=None, x=1.0, y=2.0, theta=0.3, velocity=0.4):
    return VehicleStateEstimate(
        timestamp=time.time() if timestamp is None else float(timestamp),
        x=float(x),
        y=float(y),
        theta=float(theta),
        velocity=float(velocity),
        acceleration=0.0,
        gps_valid=True,
    )


def _free_base_port(vehicle_count: int = 2) -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("127.0.0.1", 0))
        start_port = max(20000, probe.getsockname()[1])
    finally:
        probe.close()

    for base_port in range(start_port, 65000 - vehicle_count):
        sockets = []
        try:
            for offset in range(vehicle_count):
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind(("127.0.0.1", base_port + offset))
                sockets.append(sock)
            return base_port
        except OSError:
            continue
        finally:
            for sock in sockets:
                sock.close()

    raise RuntimeError("could not find free consecutive UDP ports")


def _wait_for(condition, timeout_s=1.0, interval_s=0.02):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if condition():
            return True
        time.sleep(interval_s)
    return condition()


def _build_v2v_fleet(vehicle_count: int, base_port: int):
    vehicles = []
    for vehicle_id in range(vehicle_count):
        peers = [
            {"vehicle_id": peer_id, "ip": "127.0.0.1"}
            for peer_id in range(vehicle_count)
            if peer_id != vehicle_id
        ]
        vehicles.append(
            V2VUdp(
                {
                    "vehicle_id": vehicle_id,
                    "bind_ip": "127.0.0.1",
                    "base_port": base_port,
                    "broadcast_rate_hz": 1000,
                    "peer_timeout_s": 2.0,
                    "peers": peers,
                }
            )
        )
    return vehicles


def _stop_all(vehicles):
    for vehicle in vehicles:
        vehicle.stop()


def _process_all(vehicles):
    for vehicle in vehicles:
        vehicle.process_received_messages()


class TestNullV2V(unittest.TestCase):
    def test_null_v2v_status_is_safe(self):
        config = {"enabled": False}
        v2v = V2VNull(config, vehicle_id=3)

        v2v.start()
        self.assertEqual(v2v._config, config)
        self.assertEqual(v2v._vehicle_id, 3)
        self.assertFalse(v2v.broadcast_local_state(_state()))
        self.assertEqual(v2v.get_peer_states(), {})
        self.assertEqual(
            v2v.get_status(),
            {"enabled": False, "active": False, "peer_count": 0},
        )
        v2v.stop()


class TestUdpV2V(unittest.TestCase):
    def test_udp_v2v_one_vehicle_starts_with_zero_peers(self):
        base_port = _free_base_port()
        v2v = V2VUdp(
            {
                "vehicle_id": 0,
                "bind_ip": "127.0.0.1",
                "base_port": base_port,
                "peers": [],
            }
        )

        try:
            v2v.start()
            status = v2v.get_status()

            self.assertTrue(status["active"])
            self.assertEqual(status["peer_count"], 0)
            self.assertEqual(v2v.get_peer_states(), {})
            self.assertFalse(v2v.broadcast_local_state(_state()))
        finally:
            v2v.stop()

    def test_udp_v2v_uses_config_vehicle_id_when_not_overridden(self):
        base_port = _free_base_port(3)
        v2v = V2VUdp(
            {
                "vehicle_id": 2,
                "bind_ip": "127.0.0.1",
                "base_port": base_port,
                "peers": [],
            }
        )

        self.assertEqual(v2v.get_status()["vehicle_id"], 2)
        self.assertEqual(v2v.get_status()["local_port"], base_port + 2)

    def test_udp_v2v_two_instances_exchange_state_on_localhost(self):
        base_port = _free_base_port()
        v0 = V2VUdp(
            {
                "vehicle_id": 0,
                "bind_ip": "127.0.0.1",
                "base_port": base_port,
                "broadcast_rate_hz": 1000,
                "peers": [{"vehicle_id": 1, "ip": "127.0.0.1"}],
            }
        )
        v1 = V2VUdp(
            {
                "vehicle_id": 1,
                "bind_ip": "127.0.0.1",
                "base_port": base_port,
                "broadcast_rate_hz": 1000,
                "peers": [{"vehicle_id": 0, "ip": "127.0.0.1"}],
            }
        )

        try:
            v0.start()
            v1.start()
            self.assertTrue(v0.broadcast_local_state(_state(x=1.2, y=0.3, theta=0.1, velocity=0.5)))

            received = _wait_for(
                lambda: (v1.process_received_messages() or 0 in v1.get_peer_states()),
                timeout_s=1.0,
            )

            self.assertTrue(received)
            peer_state = v1.get_peer_states()[0]
            self.assertIsInstance(peer_state, V2VState)
            self.assertEqual(peer_state.vehicle_id, 0)
            self.assertAlmostEqual(peer_state.x, 1.2)
            self.assertAlmostEqual(peer_state.y, 0.3)
            self.assertAlmostEqual(peer_state.theta, 0.1)
            self.assertAlmostEqual(peer_state.velocity, 0.5)
        finally:
            v0.stop()
            v1.stop()

    def test_udp_v2v_one_sender_reaches_multiple_peers_on_localhost(self):
        vehicle_count = 4
        base_port = _free_base_port(vehicle_count)
        vehicles = _build_v2v_fleet(vehicle_count, base_port)

        try:
            for vehicle in vehicles:
                vehicle.start()

            self.assertTrue(
                vehicles[0].broadcast_local_state(
                    _state(x=10.0, y=20.0, theta=0.4, velocity=0.8)
                )
            )

            received = _wait_for(
                lambda: (
                    _process_all(vehicles)
                    or all(0 in vehicles[i].get_peer_states() for i in range(1, vehicle_count))
                ),
                timeout_s=1.0,
            )

            self.assertTrue(received)
            self.assertEqual(vehicles[0].get_peer_states(), {})
            for vehicle_id in range(1, vehicle_count):
                peer_state = vehicles[vehicle_id].get_peer_states()[0]
                self.assertEqual(peer_state.vehicle_id, 0)
                self.assertAlmostEqual(peer_state.x, 10.0)
                self.assertAlmostEqual(peer_state.y, 20.0)
                self.assertAlmostEqual(peer_state.theta, 0.4)
                self.assertAlmostEqual(peer_state.velocity, 0.8)
        finally:
            _stop_all(vehicles)

    def test_udp_v2v_four_instances_exchange_all_to_all_on_localhost(self):
        vehicle_count = 4
        base_port = _free_base_port(vehicle_count)
        vehicles = _build_v2v_fleet(vehicle_count, base_port)

        try:
            for vehicle in vehicles:
                vehicle.start()

            for vehicle_id, vehicle in enumerate(vehicles):
                self.assertTrue(
                    vehicle.broadcast_local_state(
                        _state(
                            x=vehicle_id + 0.1,
                            y=vehicle_id + 0.2,
                            theta=vehicle_id + 0.3,
                            velocity=vehicle_id + 0.4,
                        )
                    )
                )

            expected_peer_ids = {
                vehicle_id: set(range(vehicle_count)) - {vehicle_id}
                for vehicle_id in range(vehicle_count)
            }
            received = _wait_for(
                lambda: (
                    _process_all(vehicles)
                    or all(
                        set(vehicle.get_peer_states().keys()) == expected_peer_ids[vehicle_id]
                        for vehicle_id, vehicle in enumerate(vehicles)
                    )
                ),
                timeout_s=1.0,
            )

            self.assertTrue(received)
            for vehicle_id, vehicle in enumerate(vehicles):
                peer_states = vehicle.get_peer_states()
                self.assertEqual(set(peer_states.keys()), expected_peer_ids[vehicle_id])
                for peer_id, peer_state in peer_states.items():
                    self.assertEqual(peer_state.vehicle_id, peer_id)
                    self.assertAlmostEqual(peer_state.x, peer_id + 0.1)
                    self.assertAlmostEqual(peer_state.y, peer_id + 0.2)
                    self.assertAlmostEqual(peer_state.theta, peer_id + 0.3)
                    self.assertAlmostEqual(peer_state.velocity, peer_id + 0.4)
        finally:
            _stop_all(vehicles)

    def test_udp_v2v_ignores_own_messages(self):
        base_port = _free_base_port()
        v2v = V2VUdp(
            {
                "vehicle_id": 0,
                "bind_ip": "127.0.0.1",
                "base_port": base_port,
                "broadcast_rate_hz": 1000,
                "peers": [{"vehicle_id": 1, "ip": "127.0.0.1"}],
            }
        )

        try:
            v2v.start()
            raw = (
                b'{"sender_id":0,"timestamp":1.0,"message_type":"STATE",'
                b'"payload":{"x":9.0,"y":9.0,"theta":0.0,"velocity":0.0}}'
            )
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.sendto(raw, ("127.0.0.1", base_port))
            finally:
                sock.close()

            time.sleep(0.1)
            v2v.process_received_messages()
            self.assertEqual(v2v.get_peer_states(), {})
        finally:
            v2v.stop()

    def test_udp_v2v_peer_timeout_removes_stale_peer(self):
        base_port = _free_base_port()
        v0 = V2VUdp(
            {
                "vehicle_id": 0,
                "bind_ip": "127.0.0.1",
                "base_port": base_port,
                "broadcast_rate_hz": 1000,
                "peer_timeout_s": 0.1,
                "peers": [{"vehicle_id": 1, "ip": "127.0.0.1"}],
            }
        )
        v1 = V2VUdp(
            {
                "vehicle_id": 1,
                "bind_ip": "127.0.0.1",
                "base_port": base_port,
                "broadcast_rate_hz": 1000,
                "peer_timeout_s": 0.1,
                "peers": [{"vehicle_id": 0, "ip": "127.0.0.1"}],
            }
        )

        try:
            v0.start()
            v1.start()
            v0.broadcast_local_state(_state())
            self.assertTrue(
                _wait_for(lambda: (v1.process_received_messages() or 0 in v1.get_peer_states()))
            )
            time.sleep(0.15)
            v1.process_received_messages()
            self.assertEqual(v1.get_peer_states(), {})
        finally:
            v0.stop()
            v1.stop()

    def test_udp_v2v_stop_closes_thread(self):
        base_port = _free_base_port()
        v2v = V2VUdp(
            {
                "vehicle_id": 0,
                "bind_ip": "127.0.0.1",
                "base_port": base_port,
                "peers": [],
            }
        )

        v2v.start()
        self.assertTrue(v2v.get_status()["thread_alive"])
        v2v.stop()

        self.assertFalse(v2v.get_status()["active"])
        self.assertFalse(v2v.get_status()["thread_alive"])


if __name__ == "__main__":
    unittest.main()
