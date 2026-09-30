"""ctypes adapter for a platform-neutral C/C++ board firmware core."""

from __future__ import annotations

import ctypes
from pathlib import Path
from typing import Any, List

from .firmware import FirmwareStepResult
from .types import BusDelivery, ElectronicsSensorFrame


class NativeFirmwareCore:
    """Run firmware algorithm code from a shared library in software-in-the-loop."""

    _OUTPUT_CAPACITY = 65_536

    def __init__(
        self,
        *,
        library_path: str,
        vehicle_id: int,
        publish_rate_hz: float = 20.0,
    ) -> None:
        supplied = Path(library_path).expanduser()
        candidates = (
            [supplied]
            if supplied.is_absolute()
            else [
                Path.cwd() / supplied,
                Path(__file__).resolve().parent.parent / supplied,
                Path(__file__).resolve().parent / supplied,
            ]
        )
        path = next(
            (candidate.resolve() for candidate in candidates if candidate.is_file()),
            candidates[0].resolve(),
        )
        if not path.is_file():
            raise FileNotFoundError(f"Native firmware library not found: {path}")
        self.library_path = path
        self.vehicle_id = int(vehicle_id)
        self._library = ctypes.CDLL(str(path))
        self._bind_functions()
        self._handle = self._library.cran_core_create(
            ctypes.c_uint32(self.vehicle_id), ctypes.c_double(publish_rate_hz)
        )
        if not self._handle:
            raise RuntimeError("cran_core_create returned a null handle")
        self._nav_rx_count = 0
        self._vehicle_rx_count = 0
        self._v2v_rx_count = 0

    def _bind_functions(self) -> None:
        lib = self._library
        lib.cran_core_create.argtypes = [ctypes.c_uint32, ctypes.c_double]
        lib.cran_core_create.restype = ctypes.c_void_p
        lib.cran_core_destroy.argtypes = [ctypes.c_void_p]
        lib.cran_core_reset.argtypes = [ctypes.c_void_p]
        double_ptr = ctypes.POINTER(ctypes.c_double)
        lib.cran_core_on_nav.argtypes = [
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
        lib.cran_core_on_input.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint8,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_size_t,
            ctypes.c_uint64,
        ]
        lib.cran_core_step.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
        lib.cran_core_pop_output.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint8,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_size_t,
        ]
        lib.cran_core_pop_output.restype = ctypes.c_int

    def close(self) -> None:
        handle = getattr(self, "_handle", None)
        if handle:
            self._library.cran_core_destroy(handle)
            self._handle = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def reset(self) -> None:
        self._require_handle()
        self._library.cran_core_reset(self._handle)
        self._nav_rx_count = 0
        self._vehicle_rx_count = 0
        self._v2v_rx_count = 0

    def on_nav_frame(
        self, frame: ElectronicsSensorFrame, delivery: BusDelivery
    ) -> None:
        self._require_handle()
        accel = (ctypes.c_double * 3)(*frame.acceleration_mps2)
        gyro = (ctypes.c_double * 3)(*frame.angular_rate_rps)
        mag = (ctypes.c_double * 3)(*frame.magnetic_field_ut)
        position = (ctypes.c_double * 2)(*frame.position_m)
        flags = (
            int(frame.imu_valid)
            | (int(frame.magnetometer_valid) << 1)
            | (int(frame.gnss_valid) << 2)
            | (int(frame.gnss_fresh) << 3)
        )
        self._library.cran_core_on_nav(
            self._handle,
            ctypes.c_uint32(frame.sequence),
            ctypes.c_uint64(frame.timestamp_ns),
            accel,
            gyro,
            mag,
            position,
            ctypes.c_double(frame.heading_rad),
            ctypes.c_double(frame.temperature_c),
            ctypes.c_uint8(flags),
        )
        self._nav_rx_count += 1

    def on_vehicle_payload(self, payload: bytes, timestamp_ns: int) -> None:
        self._send_input(0, payload, timestamp_ns)
        self._vehicle_rx_count += 1

    def on_v2v_payload(self, payload: bytes, timestamp_ns: int) -> None:
        self._send_input(1, payload, timestamp_ns)
        self._v2v_rx_count += 1

    def step(self, timestamp_ns: int) -> FirmwareStepResult:
        self._require_handle()
        self._library.cran_core_step(self._handle, ctypes.c_uint64(timestamp_ns))
        return FirmwareStepResult(
            vehicle_payloads=self._drain_outputs(0),
            v2v_payloads=self._drain_outputs(1),
            status={
                "backend": "native_cpp",
                "library": self.library_path.name,
                "nav_frames_received": self._nav_rx_count,
                "vehicle_frames_received": self._vehicle_rx_count,
                "v2v_frames_received": self._v2v_rx_count,
            },
        )

    def _send_input(self, channel: int, payload: bytes, timestamp_ns: int) -> None:
        self._require_handle()
        data = bytes(payload)
        buffer = (ctypes.c_uint8 * len(data)).from_buffer_copy(data) if data else None
        self._library.cran_core_on_input(
            self._handle,
            ctypes.c_uint8(channel),
            buffer,
            ctypes.c_size_t(len(data)),
            ctypes.c_uint64(timestamp_ns),
        )

    def _drain_outputs(self, channel: int) -> List[bytes]:
        result: List[bytes] = []
        buffer = (ctypes.c_uint8 * self._OUTPUT_CAPACITY)()
        while True:
            length = int(
                self._library.cran_core_pop_output(
                    self._handle,
                    ctypes.c_uint8(channel),
                    buffer,
                    ctypes.c_size_t(len(buffer)),
                )
            )
            if length == 0:
                return result
            if length < 0:
                raise BufferError(
                    f"Native output needs {-length} bytes; capacity is {len(buffer)}"
                )
            result.append(bytes(buffer[:length]))

    def _require_handle(self) -> None:
        if not getattr(self, "_handle", None):
            raise RuntimeError("Native firmware core is closed")
