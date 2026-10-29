"""PCB UART tests use a fake serial device; no hardware is opened."""

import os
import queue
import time
import unittest
from unittest.mock import patch

from pcb_connection import PcbConnectionMonitor


class FakeSerial:
    def __init__(self):
        self.incoming = queue.Queue()
        self.closed = False

    @property
    def in_waiting(self):
        return 4096

    def read(self, size):
        try:
            value = self.incoming.get(timeout=0.02)
        except queue.Empty:
            return b""
        if isinstance(value, Exception):
            raise value
        return value

    def close(self):
        self.closed = True


class PcbConnectionTests(unittest.TestCase):
    def wait_for(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate():
            if time.monotonic() > deadline:
                self.fail("UART monitor did not reach the expected state")
            time.sleep(0.005)

    def monitor(self, **kwargs):
        device = FakeSerial()
        monitor = PcbConnectionMonitor("TEST_PORT", serial_factory=lambda **_: device, **kwargs)
        self.addCleanup(monitor.close)
        self.wait_for(lambda: monitor.get_status()["state"] == "waiting")
        return monitor, device

    def test_no_configuration_does_not_open_a_port(self):
        factory = unittest.mock.Mock()
        monitor = PcbConnectionMonitor(serial_factory=factory)
        self.assertEqual(monitor.get_status()["state"], "not_configured")
        factory.assert_not_called()

    def test_open_port_and_unrelated_data_do_not_confirm_a_pcb(self):
        monitor, device = self.monitor()
        device.incoming.put(b"hello\n{\"healthy\":true}\n")
        self.wait_for(device.incoming.empty)
        self.assertEqual(monitor.get_status()["state"], "waiting")

    def test_fragmented_uart_message_timeout_and_recovery(self):
        now = [10.0]
        monitor, device = self.monitor(clock=lambda: now[0])
        device.incoming.put(b"[UART 42] st=0x01 AX=1 AY=2")
        device.incoming.put(b" AZ=3 GX=0 GY=0 GZ=0 T=20\r\n")
        self.wait_for(lambda: monitor.get_status()["state"] == "connected")
        now[0] += 4
        self.assertEqual(monitor.get_status()["state"], "stale")
        device.incoming.put(b"[43] IMU  status=0x01\r\n")
        self.wait_for(lambda: monitor.get_status()["state"] == "connected")
        self.assertEqual(monitor.get_status()["messages_received"], 2)

    def test_current_wifi_probe_firmware_confirms_uart_even_if_wifi_faults(self):
        monitor, device = self.monitor()
        device.incoming.put(b"[WiFi] RDSR0=0x42 RDSR1=0x02 RDSR2=0x00 HOST_IRQ=0\r\n")
        device.incoming.put(b"[WiFi] SPI4 status-register read failed\r\n")
        self.wait_for(lambda: monitor.get_status()["messages_received"] == 2)
        self.assertEqual(monitor.get_status()["state"], "connected")

    def test_unplug_clears_connected_state_and_reports_error(self):
        monitor, device = self.monitor(retry_s=10)
        device.incoming.put(b"COM H753 - IMU SPI Receiver\r\n")
        self.wait_for(lambda: monitor.get_status()["state"] == "connected")
        device.incoming.put(OSError("USB adapter unplugged"))
        self.wait_for(lambda: monitor.get_status()["state"] == "error")
        self.assertIn("unplugged", monitor.get_status()["last_error"])
        self.assertIsNone(monitor.get_status()["last_receive_age_s"])
        self.wait_for(lambda: device.closed)

    def test_failed_port_retries_and_recovers_without_restarting_vehicle(self):
        device = FakeSerial()
        attempts = []

        def factory(**kwargs):
            attempts.append(kwargs)
            if len(attempts) == 1:
                raise OSError("Port unavailable")
            return device

        device.incoming.put(b"COM H753 - WiFi nRF7002 Probe\n")
        monitor = PcbConnectionMonitor("TEST_PORT", serial_factory=factory, retry_s=0.01)
        self.addCleanup(monitor.close)
        self.wait_for(lambda: monitor.get_status()["state"] == "connected")
        self.assertEqual(len(attempts), 2)
        self.assertIsNone(monitor.get_status()["last_error"])

    def test_environment_is_per_vehicle_and_invalid_baud_is_reported(self):
        with patch.dict(os.environ, {"QCAR_PCB_PORT_1": "COM7", "QCAR_PCB_BAUD_1": "oops"}, clear=True):
            self.assertEqual(PcbConnectionMonitor.from_environment(0).get_status()["state"], "not_configured")
            status = PcbConnectionMonitor.from_environment(1).get_status()
            self.assertEqual(status["state"], "error")
            self.assertIn("QCAR_PCB_BAUD_1", status["last_error"])


if __name__ == "__main__":
    unittest.main()
