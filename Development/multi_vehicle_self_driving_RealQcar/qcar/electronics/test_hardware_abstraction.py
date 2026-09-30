"""Compliance tests for vendor-neutral MCU/SoC and HIL contracts."""

import ctypes
import json
import unittest
from pathlib import Path

from electronics import (
    ElectronicsDigitalTwin,
    HILFirmwareCore,
    HILFrame,
    HILMessageType,
    HardwareTargetManifest,
    LoopbackHILTransport,
    ReferenceHILDevice,
    VehiclePlantSample,
    negotiate_capabilities,
)


def _plant(timestamp_ns: int) -> VehiclePlantSample:
    return VehiclePlantSample(timestamp_ns, 1.0, 0.2, 0.1, 0.4, 0.0, 0.02)


NATIVE_LIBRARY = Path(__file__).resolve().parent / "native" / "build" / (
    "cran_electronics_core.dll"
)


def _custom_target() -> HardwareTargetManifest:
    return HardwareTargetManifest.from_config(
        {
            "profile": "nxp_zephyr_reference",
            "target_id": "nxp-s32k-board-01",
            "topology": "dual_node",
            "sensor_node": {
                "node_id": "acquisition",
                "platform": "nxp_s32k144",
                "architecture": "arm_cortex_m4f",
                "runtime": "zephyr",
                "execution": "hil_external",
                "float_width_bits": 32,
                "transports": ["spi", "can_fd"],
                "features": ["sensor_acquisition", "timestamping"],
            },
            "compute_node": {
                "node_id": "estimator",
                "platform": "nxp_s32k344",
                "architecture": "arm_cortex_m7",
                "runtime": "zephyr",
                "execution": "hil_external",
                "float_width_bits": 64,
                "max_payload_bytes": 4096,
                "transports": ["can_fd", "ethernet"],
                "features": ["firmware_core", "trust", "observer", "v2v"],
            },
        }
    )


class HardwareManifestTests(unittest.TestCase):
    def test_vendor_specific_target_is_capability_based(self) -> None:
        target = _custom_target()
        result = negotiate_capabilities(
            target,
            required_features=("firmware_core", "trust", "observer", "v2v"),
            min_float_width_bits=64,
            min_payload_bytes=1500,
        )
        self.assertTrue(result["accepted"], result["blockers"])
        self.assertEqual(result["remote_platform"], "nxp_s32k344")
        self.assertEqual(result["remote_profile"], "nxp_zephyr_reference")
        rejected = negotiate_capabilities(
            target,
            required_features=("firmware_core", "controller_authority"),
        )
        self.assertFalse(rejected["accepted"])
        self.assertIn("missing feature: controller_authority", rejected["blockers"])

    def test_single_soc_uses_one_power_state_machine_and_generic_links(self) -> None:
        twin = ElectronicsDigitalTwin(
            4,
            {
                "hardware_target": {"profile": "linux_soc"},
                "sensor_compute_interface": "shared_memory",
                "vehicle_interface": "ethernet",
                "mcu": {"nav_boot_time_s": 0.0, "com_boot_time_s": 0.0},
                "sensors": {
                    "imu_rate_hz": 100.0,
                    "gnss_rate_hz": 100.0,
                    "magnetometer_rate_hz": 100.0,
                },
                "firmware": {
                    "backend": "python_reference",
                    "publish_rate_hz": 100.0,
                },
            },
        )
        self.assertIs(twin.sensor_node, twin.compute_node)
        for step in range(1, 8):
            twin.step(_plant(step * 10_000_000), 0.01)
        status = twin.get_status_summary()
        self.assertTrue(status["healthy"])
        self.assertEqual(status["topology"], "single_node")
        self.assertEqual(status["target_profile"], "linux_soc")
        self.assertEqual(status["sensor_compute_interface"], "shared_memory")
        self.assertEqual(status["vehicle_interface"], "ethernet")
        self.assertEqual(status["sensor_node_state"], "running")
        self.assertEqual(status["compute_node_state"], "running")
        # Legacy aliases remain available to the existing VehicleLogic/UI.
        self.assertEqual(status["nav_mcu_state"], "running")
        self.assertEqual(status["com_mcu_state"], "running")


class HILProtocolTests(unittest.TestCase):
    def test_binary_frame_detects_corruption(self) -> None:
        original = HILFrame(
            HILMessageType.VEHICLE_INPUT,
            sequence=7,
            timestamp_ns=123456,
            payload=b"opaque-msgpack",
        )
        encoded = original.encode()
        self.assertEqual(HILFrame.decode(encoded), original)
        damaged = bytearray(encoded)
        damaged[-5] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC"):
            HILFrame.decode(bytes(damaged))

    @unittest.skipUnless(NATIVE_LIBRARY.is_file(), "Native core is not built")
    def test_python_and_portable_c_hil_framing_are_identical(self) -> None:
        library = ctypes.CDLL(str(NATIVE_LIBRARY))
        library.cran_hil_encode.argtypes = [
            ctypes.c_uint8,
            ctypes.c_uint16,
            ctypes.c_uint32,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_size_t,
        ]
        library.cran_hil_encode.restype = ctypes.c_int
        payload = b"portable-wire-contract"
        source = (ctypes.c_uint8 * len(payload)).from_buffer_copy(payload)
        output = (ctypes.c_uint8 * 256)()
        length = library.cran_hil_encode(
            int(HILMessageType.V2V_INPUT),
            3,
            91,
            987654321,
            source,
            len(payload),
            output,
            len(output),
        )
        self.assertGreater(length, 0)
        c_encoded = bytes(output[:length])
        python_encoded = HILFrame(
            HILMessageType.V2V_INPUT,
            sequence=91,
            timestamp_ns=987654321,
            payload=payload,
            flags=3,
        ).encode()
        self.assertEqual(c_encoded, python_encoded)
        self.assertEqual(HILFrame.decode(c_encoded).payload, payload)

    @unittest.skipUnless(NATIVE_LIBRARY.is_file(), "Native core is not built")
    def test_portable_platform_callbacks_drive_the_same_core(self) -> None:
        library = ctypes.CDLL(str(NATIVE_LIBRARY))
        clock_type = ctypes.CFUNCTYPE(ctypes.c_uint64, ctypes.c_void_p)
        write_type = ctypes.CFUNCTYPE(
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_uint8,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_size_t,
        )
        log_type = ctypes.CFUNCTYPE(
            None, ctypes.c_void_p, ctypes.c_uint8, ctypes.c_char_p
        )

        class PlatformOps(ctypes.Structure):
            _fields_ = [
                ("abi_version", ctypes.c_uint32),
                ("context", ctypes.c_void_p),
                ("monotonic_time_ns", clock_type),
                ("write_channel", write_type),
                ("log", log_type),
            ]

        class Capabilities(ctypes.Structure):
            _fields_ = [
                ("schema_version", ctypes.c_uint32),
                ("word_size_bits", ctypes.c_uint16),
                ("float_width_bits", ctypes.c_uint16),
                ("little_endian", ctypes.c_uint8),
                ("reserved", ctypes.c_uint8 * 3),
                ("max_payload_bytes", ctypes.c_uint32),
                ("memory_bytes", ctypes.c_uint32),
                ("feature_mask", ctypes.c_uint32),
                ("transport_mask", ctypes.c_uint32),
                ("dynamic_allocation", ctypes.c_uint8),
                ("capability_reserved", ctypes.c_uint8 * 3),
            ]

        outputs = []

        @clock_type
        def clock(_context):
            return 10_000_000

        @write_type
        def write(_context, channel, data, length):
            outputs.append((int(channel), bytes(data[:length])))
            return int(length)

        @log_type
        def log(_context, _level, _message):
            return None

        ops = PlatformOps(1, None, clock, write, log)
        capabilities = Capabilities(
            1,
            64,
            64,
            1,
            (ctypes.c_uint8 * 3)(0, 0, 0),
            65_536,
            524_288,
            0x0F,
            0x30,
            1,
            (ctypes.c_uint8 * 3)(0, 0, 0),
        )
        library.cran_target_create.argtypes = [
            ctypes.c_uint32,
            ctypes.c_double,
            ctypes.POINTER(PlatformOps),
            ctypes.POINTER(Capabilities),
        ]
        library.cran_target_create.restype = ctypes.c_void_p
        library.cran_target_destroy.argtypes = [ctypes.c_void_p]
        double_ptr = ctypes.POINTER(ctypes.c_double)
        library.cran_target_on_sensor.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint32,
            ctypes.c_uint64,
            double_ptr,
            double_ptr,
            double_ptr,
            double_ptr,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_uint8,
        ]
        library.cran_target_poll_at.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint64,
        ]
        library.cran_target_poll_at.restype = ctypes.c_int
        handle = library.cran_target_create(
            5, 100.0, ctypes.byref(ops), ctypes.byref(capabilities)
        )
        self.assertTrue(handle)
        try:
            accel = (ctypes.c_double * 3)(0.1, 0.0, 9.81)
            gyro = (ctypes.c_double * 3)(0.0, 0.0, 0.02)
            mag = (ctypes.c_double * 3)(20.0, 1.0, 40.0)
            position = (ctypes.c_double * 2)(1.2, -0.4)
            library.cran_target_on_sensor(
                handle,
                9,
                10_000_000,
                accel,
                gyro,
                mag,
                position,
                0.12,
                30.0,
                0x0F,
            )
            emitted = library.cran_target_poll_at(handle, 10_000_000)
            self.assertGreaterEqual(emitted, 1)
            self.assertTrue(outputs)
            self.assertEqual(outputs[0][0], 0)
            message = json.loads(outputs[0][1].decode("utf-8"))
            self.assertEqual(message["vehicle_id"], 5)
            self.assertEqual(message["sensor"]["sequence"], 9)
        finally:
            library.cran_target_destroy(handle)

    def test_external_target_handshake_and_sensor_pipeline(self) -> None:
        remote = _custom_target()
        device = ReferenceHILDevice(
            manifest=remote,
            vehicle_id=0,
            publish_rate_hz=100.0,
        )
        transport = LoopbackHILTransport(device.handle)
        host = HardwareTargetManifest.from_config(
            {"profile": "desktop_sil"}, vehicle_id=0
        )
        hil_core = HILFirmwareCore(
            vehicle_id=0,
            local_manifest=host,
            transport=transport,
            strict_handshake=True,
            required_features=("firmware_core", "trust", "observer", "v2v"),
            min_float_width_bits=64,
            min_payload_bytes=1500,
        )
        twin = ElectronicsDigitalTwin(
            0,
            {
                "hardware_target": remote.to_dict(),
                "sensor_compute_interface": "spi",
                "vehicle_interface": "can_fd",
                "mcu": {"nav_boot_time_s": 0.0, "com_boot_time_s": 0.0},
                "sensors": {
                    "imu_rate_hz": 100.0,
                    "gnss_rate_hz": 100.0,
                    "magnetometer_rate_hz": 100.0,
                },
            },
            firmware_core=hil_core,
        )
        try:
            for step in range(1, 12):
                twin.step(_plant(step * 10_000_000), 0.01)
            status = twin.get_status_summary()
            deliveries = twin.pop_vehicle_deliveries()
            self.assertEqual(status["firmware_backend"], "hil_external")
            self.assertTrue(status["healthy"], status)
            self.assertTrue(status["hil"]["handshake_ready"], status["hil"])
            self.assertTrue(
                status["hil"]["capability_negotiation"]["accepted"]
            )
            self.assertEqual(
                status["hil"]["remote_target"]["compute_node"]["platform"],
                "nxp_s32k344",
            )
            self.assertTrue(deliveries)
            message = json.loads(deliveries[-1].payload.decode("utf-8"))
            self.assertEqual(message["type"], "electronics_sensor")
        finally:
            twin.close()

    def test_lost_initial_hello_is_retried(self) -> None:
        remote = _custom_target()
        device = ReferenceHILDevice(manifest=remote, vehicle_id=0)
        link = {"enabled": False}

        def intermittent_endpoint(data: bytes):
            return device.handle(data) if link["enabled"] else []

        transport = LoopbackHILTransport(intermittent_endpoint)
        core = HILFirmwareCore(
            vehicle_id=0,
            local_manifest=HardwareTargetManifest.from_config(
                {"profile": "desktop_sil"}
            ),
            transport=transport,
            hello_retry_s=0.1,
            link_timeout_s=1.0,
        )
        try:
            first = core.step(10_000_000)
            self.assertFalse(first.status["handshake_ready"])
            link["enabled"] = True
            recovered = core.step(200_000_000)
            self.assertTrue(recovered.status["handshake_ready"], recovered.status)
            self.assertTrue(recovered.status["link_alive"])
        finally:
            core.close()


if __name__ == "__main__":
    unittest.main()
