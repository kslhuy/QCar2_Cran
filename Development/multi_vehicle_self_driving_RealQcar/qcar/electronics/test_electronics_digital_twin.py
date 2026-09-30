"""Deterministic tests for the electronics digital-twin vertical slice."""

import json
import unittest

import numpy as np

from electronics import (
    BusFaultProfile,
    CANBus,
    ElectronicsDataGateway,
    ElectronicsDigitalTwin,
    ElectronicsSensorFrame,
    ElectronicsV2VBridge,
    MCUState,
    MockQCarElectronicsAdapter,
    NavImuPacketCodecV1,
    NavSensorPacketCodecV2,
    UARTBus,
    VehiclePlantSample,
)
from electronics.types import FrameStatus
from electronics.v2v_protocol import (
    V2VEnvelopeError,
    decode_host_v2v,
    encode_host_v2v,
)


def sensor_frame(sequence: int = 7, timestamp_ns: int = 20_000_000):
    return ElectronicsSensorFrame(
        sequence=sequence,
        timestamp_ns=timestamp_ns,
        acceleration_mps2=(1.0, -2.0, 9.81),
        angular_rate_rps=(0.01, -0.02, 0.3),
        magnetic_field_ut=(20.0, 5.0, 40.0),
        position_m=(3.5, -1.2),
        heading_rad=0.25,
        temperature_c=31.5,
    )


class BusModelTests(unittest.TestCase):
    def test_uart_includes_serialization_time(self):
        bus = UARTBus("uart", baud_rate=1000.0, faults=BusFaultProfile())
        bus.transmit(
            b"0123456789", source="board", destination="vehicle", requested_at_ns=0
        )
        self.assertEqual(bus.advance(0), [])
        self.assertEqual(bus.advance(99_999_999), [])
        deliveries = bus.advance(100_000_000)
        self.assertEqual(len(deliveries), 1)
        self.assertEqual(deliveries[0].status, FrameStatus.OK)

    def test_can_crc_rejects_bit_corruption(self):
        bus = CANBus(
            "can",
            bitrate=1_000_000.0,
            faults=BusFaultProfile(bit_error_rate=1.0),
        )
        bus.transmit(
            b"abc",
            source="board",
            destination="vehicle",
            requested_at_ns=0,
            metadata={"arbitration_id": 0x123},
        )
        bus.advance(0)
        deliveries = bus.advance(1_000_000)
        self.assertEqual(len(deliveries), 1)
        self.assertEqual(deliveries[0].status, FrameStatus.DROPPED)
        self.assertEqual(deliveries[0].reason, "crc_error")


class CodecTests(unittest.TestCase):
    def test_current_firmware_v1_packet_roundtrip(self):
        encoded = NavImuPacketCodecV1.encode(sensor_frame())
        self.assertEqual(len(encoded), 18)
        decoded = NavImuPacketCodecV1.decode(
            encoded, sequence=7, timestamp_ns=20_000_000
        )
        self.assertTrue(decoded.imu_valid)
        self.assertAlmostEqual(decoded.angular_rate_rps[2], 0.3, places=2)
        self.assertFalse(decoded.gnss_valid)

    def test_full_v2_packet_roundtrip_and_crc(self):
        encoded = NavSensorPacketCodecV2.encode(sensor_frame())
        decoded = NavSensorPacketCodecV2.decode(encoded)
        self.assertEqual(decoded.sequence, 7)
        self.assertAlmostEqual(decoded.position_m[0], 3.5)
        damaged = bytearray(encoded)
        damaged[8] ^= 0x01
        with self.assertRaises(ValueError):
            NavSensorPacketCodecV2.decode(bytes(damaged))

    def test_v2v_host_envelope_preserves_routes_and_payload(self):
        encoded = encode_host_v2v(b"msgpack-payload", [1, 7])
        targets, payload, mirror_only = decode_host_v2v(encoded)
        self.assertEqual(targets, (1, 7))
        self.assertEqual(payload, b"msgpack-payload")
        self.assertFalse(mirror_only)
        with self.assertRaises(V2VEnvelopeError):
            decode_host_v2v(b"invalid")


class BoardTwinTests(unittest.TestCase):
    @staticmethod
    def _config():
        return {
            "nav_protocol": "sensor_frame_v2",
            "nav_com_interface": "spi",
            "vehicle_interface": "uart",
            "mcu": {"nav_boot_time_s": 0.0, "com_boot_time_s": 0.0},
            "sensors": {
                "imu_rate_hz": 200.0,
                "gnss_rate_hz": 200.0,
                "magnetometer_rate_hz": 200.0,
                "accel_noise_std_mps2": 0.0,
                "gyro_noise_std_rps": 0.0,
                "magnetometer_noise_std_ut": 0.0,
                "gnss_position_noise_std_m": 0.0,
                "gnss_heading_noise_std_rad": 0.0,
                "accel_bias_walk_std_mps2_sqrt_s": 0.0,
                "gyro_bias_walk_std_rps_sqrt_s": 0.0,
                "accel_quantum_mps2": 0.0,
                "gyro_quantum_rps": 0.0,
                "magnetometer_quantum_ut": 0.0,
            },
            "firmware": {"publish_rate_hz": 200.0},
        }

    def test_board_boots_and_moves_sensor_data_through_buses(self):
        twin = ElectronicsDigitalTwin(2, self._config())
        for step in range(1, 8):
            now_ns = step * 10_000_000
            twin.step(
                VehiclePlantSample(
                    timestamp_ns=now_ns,
                    x_m=1.0,
                    y_m=2.0,
                    heading_rad=0.1,
                    velocity_x_mps=0.5,
                    velocity_y_mps=0.0,
                    yaw_rate_rps=0.2,
                    acceleration_x_mps2=0.3,
                    acceleration_y_mps2=0.1,
                ),
                0.01,
            )
        snapshot = twin.get_snapshot()
        self.assertEqual(snapshot.nav_mcu_state, MCUState.RUNNING)
        self.assertEqual(snapshot.com_mcu_state, MCUState.RUNNING)
        self.assertIsNotNone(snapshot.com_sensor_frame)
        self.assertGreater(snapshot.bus_statistics["nav_com"]["delivered"], 0)
        deliveries = twin.pop_vehicle_deliveries()
        self.assertTrue(deliveries)
        payload = json.loads(deliveries[-1].payload.decode("utf-8"))
        self.assertEqual(payload["type"], "electronics_sensor")

    def test_brownout_resets_both_mcus(self):
        twin = ElectronicsDigitalTwin(0, self._config())
        twin.set_input_voltage(2.0)
        snapshot = twin.step(
            VehiclePlantSample(10_000_000, 0, 0, 0, 0, 0, 0), 0.01
        )
        self.assertTrue(snapshot.power.brownout)
        self.assertEqual(snapshot.nav_mcu_state, MCUState.BROWNOUT)
        self.assertEqual(snapshot.com_mcu_state, MCUState.BROWNOUT)

    def test_adapter_uses_plant_state_not_vehicle_sensor_cache(self):
        class Plant:
            x = 0.0
            y = 0.0
            heading = 0.0
            velocity = 1.0
            lateral_velocity = 0.0
            angular_velocity = 0.2
            accelerometer = np.array([999.0, 999.0, 999.0])

        twin = ElectronicsDigitalTwin(0, self._config())
        adapter = MockQCarElectronicsAdapter(twin)
        adapter(Plant(), 0.01)
        adapter(Plant(), 0.01)
        frame = twin.get_snapshot().nav_sensor_frame
        self.assertIsNotNone(frame)
        self.assertNotEqual(frame.acceleration_mps2[0], 999.0)
        self.assertAlmostEqual(frame.acceleration_mps2[1], 0.2)


class FusionPolicyTests(unittest.TestCase):
    def test_auxiliary_mode_never_overwrites_vehicle_sensors(self):
        twin = ElectronicsDigitalTwin(0, BoardTwinTests._config())
        twin.step(VehiclePlantSample(10_000_000, 4, 5, 0.2, 0, 0, 0), 0.01)
        twin.step(VehiclePlantSample(20_000_000, 4, 5, 0.2, 0, 0, 0), 0.01)
        gateway = ElectronicsDataGateway("auxiliary")
        update = gateway.build_update(
            {"accelerometer": np.array([9.0, 9.0, 9.0])}, twin.get_snapshot()
        )
        self.assertEqual(set(update), {"electronics"})

    def test_weighted_fusion_is_explicit(self):
        twin = ElectronicsDigitalTwin(0, BoardTwinTests._config())
        for step in range(1, 4):
            twin.step(
                VehiclePlantSample(step * 10_000_000, 4, 5, 0.2, 0, 0, 0), 0.01
            )
        gateway = ElectronicsDataGateway("weighted_fusion", electronics_weight=0.25)
        update = gateway.build_update(
            {
                "accelerometer": np.array([4.0, 4.0, 4.0]),
                "gyro_z": 4.0,
                "gps_position": np.array([0.0, 0.0, 0.0]),
            },
            twin.get_snapshot(),
        )
        np.testing.assert_allclose(update["gps_position"], [1.0, 1.25, 0.05])
        self.assertTrue(update["electronics_fusion_applied"])


class ElectronicsV2VBridgeTests(unittest.TestCase):
    class Clock:
        def __init__(self):
            self.now_ns = 0

        def __call__(self):
            return self.now_ns

    @staticmethod
    def _plant(timestamp_ns: int):
        return VehiclePlantSample(timestamp_ns, 0, 0, 0, 0, 0, 0)

    def test_firmware_mode_routes_datagram_through_two_com_mcus(self):
        config_a = BoardTwinTests._config()
        config_b = BoardTwinTests._config()
        config_a["vehicle_interface"] = "can_fd"
        config_b["vehicle_interface"] = "can_fd"
        config_a["firmware"] = {"publish_rate_hz": 0.0}
        config_b["firmware"] = {"publish_rate_hz": 0.0}
        twin_a = ElectronicsDigitalTwin(0, config_a)
        twin_b = ElectronicsDigitalTwin(1, config_b)
        clock = self.Clock()
        bridge_a = ElectronicsV2VBridge(
            twin_a, {"mode": "firmware", "faults": {}}, clock_ns=clock
        )
        bridge_b = ElectronicsV2VBridge(
            twin_b, {"mode": "firmware", "faults": {}}, clock_ns=clock
        )
        received = []

        def host_b_receive(payload, address):
            received.append((payload, address))

        def radio_send_to_b(payload, address):
            bridge_b.inbound(payload, ("vehicle-a", 8000), host_b_receive)
            return True

        twin_a.step(self._plant(10_000_000), 0.01)
        twin_b.step(self._plant(10_000_000), 0.01)
        raw_payload = b"raw-msgpack" * 30
        self.assertTrue(
            bridge_a.outbound(
                raw_payload, 1, ("vehicle-b", 8001), radio_send_to_b
            )
        )

        for step in range(2, 12):
            timestamp_ns = step * 10_000_000
            clock.now_ns = timestamp_ns
            twin_a.step(self._plant(timestamp_ns), 0.01)
            twin_b.step(self._plant(timestamp_ns), 0.01)
            bridge_a.pump()
            bridge_b.pump()

        self.assertEqual(received, [(raw_payload, ("vehicle-a", 8000))])
        status_a = bridge_a.get_status()
        status_b = bridge_b.get_status()
        self.assertEqual(status_a["firmware_tx_emitted"], 1)
        self.assertEqual(status_a["radio_tx_delivered"], 1)
        self.assertEqual(status_b["host_rx_delivered"], 1)

    def test_radio_drop_is_below_v2v_message_schema(self):
        config = BoardTwinTests._config()
        twin = ElectronicsDigitalTwin(0, config)
        clock = self.Clock()
        bridge = ElectronicsV2VBridge(
            twin,
            {
                "mode": "mirror",
                "faults": {"drop_probability": 1.0},
            },
            clock_ns=clock,
        )
        sent = []
        bridge.outbound(
            b"opaque-msgpack", 1, ("peer", 8001),
            lambda payload, address: sent.append(payload) or True,
        )
        bridge.pump(0)
        bridge.pump(1_000_000_000)
        self.assertEqual(sent, [])
        self.assertEqual(bridge.get_status()["radio"]["dropped"], 1)


if __name__ == "__main__":
    unittest.main()
