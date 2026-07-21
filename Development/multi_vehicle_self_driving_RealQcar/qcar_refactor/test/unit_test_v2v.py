"""Unit tests for generic UDP V2V transport."""

import os
import socket
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils.v2v import V2VNull, V2VUdp


def _free_udp_ports(count=2):
    sockets = []
    try:
        for _ in range(count):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("127.0.0.1", 0))
            sockets.append(sock)
        return [sock.getsockname()[1] for sock in sockets]
    finally:
        for sock in sockets:
            sock.close()


def _wait_for_messages(transport, timeout_s=1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        messages = transport.drain_received()
        if messages:
            return messages
        time.sleep(0.01)
    return transport.drain_received()


def _wait_for_message_count(transport, expected_count, timeout_s=1.0):
    deadline = time.monotonic() + timeout_s
    messages = []
    while time.monotonic() < deadline and len(messages) < expected_count:
        messages.extend(transport.drain_received())
        if len(messages) < expected_count:
            time.sleep(0.01)
    return messages


def _transport(vehicle_id, local_port, peer_id, peer_port):
    return V2VUdp(
        {
            "bind_ip": "127.0.0.1",
            "local_port": local_port,
            "peers": [{"vehicle_id": peer_id, "ip": "127.0.0.1", "port": peer_port}],
        },
        vehicle_id=vehicle_id,
    )


class TestGenericV2VTransport(unittest.TestCase):
    def test_null_transport_is_safe(self):
        transport = V2VNull({"enabled": False}, vehicle_id=3)
        transport.start()
        self.assertFalse(transport.publish("STATE", {"x": 1.0}))
        self.assertEqual(transport.drain_received(), [])
        self.assertEqual(transport.get_status()["peer_count"], 0)
        transport.stop()

    def test_udp_transports_exchange_generic_payload_and_metadata(self):
        first_port, second_port = _free_udp_ports()
        first = _transport(1, first_port, 2, second_port)
        second = _transport(2, second_port, 1, first_port)
        try:
            first.start()
            second.start()
            self.assertTrue(first.publish("STATE", {"x": 1.2, "valid": True}))
            messages = _wait_for_messages(second)

            self.assertEqual(len(messages), 1)
            message = messages[0]
            self.assertEqual(message.sender_id, 1)
            self.assertEqual(message.message_type, "STATE")
            self.assertEqual(message.payload, {"x": 1.2, "valid": True})
            self.assertEqual(message.sequence, 0)
            self.assertGreater(message.received_at_perf_counter_ns, message.sent_at_perf_counter_ns)
            self.assertEqual(second.get_status()["messages_received"], 1)
        finally:
            first.stop()
            second.stop()

    def test_udp_drops_invalid_and_unknown_sender_packets(self):
        first_port, second_port = _free_udp_ports()
        receiver = _transport(2, second_port, 1, first_port)
        try:
            receiver.start()
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sender.sendto(b'{"sender_id":1,"message_type":"STATE"}', ("127.0.0.1", second_port))
                sender.sendto(
                    b'{"sender_id":9,"message_type":"STATE","payload":{},"sequence":0,'
                    b'"sent_at_monotonic":1.0,"sent_at_perf_counter_ns":1}',
                    ("127.0.0.1", second_port),
                )
            finally:
                sender.close()
            time.sleep(0.05)
            self.assertEqual(receiver.drain_received(), [])
            self.assertGreaterEqual(receiver.get_status()["packets_dropped"], 1)
        finally:
            receiver.stop()

    def test_udp_estimates_packet_loss_from_generic_sequence_gaps(self):
        first_port, second_port = _free_udp_ports()
        receiver = _transport(2, second_port, 1, first_port)
        try:
            receiver.start()
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                for sequence in (3, 5):
                    packet = (
                        '{"sender_id":1,"message_type":"STATE","payload":{},'
                        f'"sequence":{sequence},"sent_at_monotonic":1.0,'
                        '"sent_at_perf_counter_ns":1}'
                    ).encode("ascii")
                    sender.sendto(packet, ("127.0.0.1", second_port))
            finally:
                sender.close()
            messages = _wait_for_message_count(receiver, expected_count=2)
            self.assertEqual([message.sequence for message in messages], [3, 5])
            self.assertEqual(receiver.get_status()["estimated_packets_lost"], 1)
        finally:
            receiver.stop()

    def test_udp_targets_only_selected_configured_peers(self):
        first_port, second_port, third_port = _free_udp_ports(3)
        first = V2VUdp(
            {
                "bind_ip": "127.0.0.1",
                "local_port": first_port,
                "peers": [
                    {"vehicle_id": 2, "ip": "127.0.0.1", "port": second_port},
                    {"vehicle_id": 3, "ip": "127.0.0.1", "port": third_port},
                ],
            },
            vehicle_id=1,
        )
        second = _transport(2, second_port, 1, first_port)
        third = _transport(3, third_port, 1, first_port)
        try:
            first.start()
            second.start()
            third.start()
            self.assertTrue(first.publish("WARNING", {"level": "low"}, target_vehicle_ids=[2]))
            self.assertEqual([message.sender_id for message in _wait_for_messages(second)], [1])
            time.sleep(0.05)
            self.assertEqual(third.drain_received(), [])
            with self.assertRaisesRegex(ValueError, "not a configured peer"):
                first.publish("WARNING", {}, target_vehicle_ids=[9])
        finally:
            first.stop()
            second.stop()
            third.stop()

    def test_udp_applies_optional_message_rate_limit_and_reports_rate(self):
        first_port, second_port = _free_udp_ports()
        sender = V2VUdp(
            {
                "bind_ip": "127.0.0.1",
                "local_port": first_port,
                "peers": [{"vehicle_id": 2, "ip": "127.0.0.1", "port": second_port}],
                "message_rate_limits_hz": {"STATE": 5},
            },
            vehicle_id=1,
        )
        receiver = _transport(2, second_port, 1, first_port)
        try:
            sender.start()
            receiver.start()
            self.assertTrue(sender.publish("STATE", {"x": 1.0}))
            self.assertFalse(sender.publish("STATE", {"x": 2.0}))
            self.assertEqual(sender.get_status()["message_rate_limits_hz"], {"STATE": 5.0})
            self.assertGreaterEqual(sender.get_status()["publish_rate_hz"], 1.0)
            self.assertEqual(len(_wait_for_messages(receiver)), 1)
        finally:
            sender.stop()
            receiver.stop()

    def test_udp_rejects_datagrams_larger_than_2048_bytes(self):
        first_port, second_port = _free_udp_ports()
        sender = _transport(1, first_port, 2, second_port)
        try:
            sender.start()
            with self.assertRaisesRegex(ValueError, "exceeds 2048 bytes"):
                sender.publish("STATE", {"blob": "x" * 4096})
            status = sender.get_status()
            self.assertEqual(status["max_datagram_bytes"], 2048)
            self.assertEqual(status["send_buffer_bytes"], 16384)
            self.assertEqual(status["receive_buffer_bytes"], 32768)
        finally:
            sender.stop()

    def test_stop_is_idempotent_and_closes_receive_thread(self):
        first_port, second_port = _free_udp_ports()
        transport = _transport(1, first_port, 2, second_port)
        transport.start()
        transport.stop()
        transport.stop()
        self.assertFalse(transport.get_status()["active"])
        self.assertFalse(transport.get_status()["thread_alive"])


if __name__ == "__main__":
    unittest.main()
