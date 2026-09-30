"""NAV-to-COM packet codecs shared by the reference firmware model."""

from __future__ import annotations

import math
import struct
import zlib
from typing import Tuple

from .types import ElectronicsSensorFrame


class PacketDecodeError(ValueError):
    pass


def _int16(value: float) -> int:
    return max(-32768, min(32767, int(round(value))))


class NavImuPacketCodecV1:
    """Exact 18-byte IMU packet shape currently used by NAV/COM firmware."""

    name = "legacy_v1"
    packet_size = 18
    _prefix = struct.Struct(">BB7h")
    _accel_lsb_per_g = 2048.0
    _gyro_lsb_per_dps = 16.4

    @classmethod
    def encode(cls, frame: ElectronicsSensorFrame) -> bytes:
        accel_raw = [
            _int16(value / 9.80665 * cls._accel_lsb_per_g)
            for value in frame.acceleration_mps2
        ]
        gyro_raw = [
            _int16(math.degrees(value) * cls._gyro_lsb_per_dps)
            for value in frame.angular_rate_rps
        ]
        temperature_raw = _int16(frame.temperature_c * 100.0)
        status = 0x01 if frame.imu_valid else 0x00
        prefix = cls._prefix.pack(
            0xAA,
            status,
            *accel_raw,
            *gyro_raw,
            temperature_raw,
        )
        checksum = 0
        for value in prefix:
            checksum ^= value
        return prefix + bytes((checksum, 0x55))

    @classmethod
    def decode(cls, payload: bytes, *, sequence: int = 0, timestamp_ns: int = 0) -> ElectronicsSensorFrame:
        if len(payload) != cls.packet_size:
            raise PacketDecodeError(f"legacy_v1 packet must be {cls.packet_size} bytes")
        if payload[0] != 0xAA or payload[-1] != 0x55:
            raise PacketDecodeError("legacy_v1 header/footer mismatch")
        checksum = 0
        for value in payload[:16]:
            checksum ^= value
        if checksum != payload[16]:
            raise PacketDecodeError("legacy_v1 checksum mismatch")
        values = cls._prefix.unpack(payload[:16])
        status = values[1]
        accel = tuple(value / cls._accel_lsb_per_g * 9.80665 for value in values[2:5])
        gyro = tuple(math.radians(value / cls._gyro_lsb_per_dps) for value in values[5:8])
        return ElectronicsSensorFrame(
            sequence=int(sequence),
            timestamp_ns=int(timestamp_ns),
            acceleration_mps2=accel,
            angular_rate_rps=gyro,
            magnetic_field_ut=(0.0, 0.0, 0.0),
            position_m=(0.0, 0.0),
            heading_rad=0.0,
            temperature_c=values[8] / 100.0,
            imu_valid=bool(status & 0x01),
            magnetometer_valid=False,
            gnss_valid=False,
            gnss_fresh=False,
        )


class NavSensorPacketCodecV2:
    """Versioned full sensor frame proposed for NAV-to-COM firmware migration."""

    name = "sensor_frame_v2"
    _body = struct.Struct("<2sBBIQ10f3d")
    packet_size = _body.size + 4

    @classmethod
    def encode(cls, frame: ElectronicsSensorFrame) -> bytes:
        flags = (
            int(frame.imu_valid)
            | (int(frame.magnetometer_valid) << 1)
            | (int(frame.gnss_valid) << 2)
            | (int(frame.gnss_fresh) << 3)
        )
        body = cls._body.pack(
            b"ES",
            2,
            flags,
            int(frame.sequence) & 0xFFFFFFFF,
            int(frame.timestamp_ns) & 0xFFFFFFFFFFFFFFFF,
            *frame.acceleration_mps2,
            *frame.angular_rate_rps,
            *frame.magnetic_field_ut,
            float(frame.temperature_c),
            float(frame.position_m[0]),
            float(frame.position_m[1]),
            float(frame.heading_rad),
        )
        return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)

    @classmethod
    def decode(cls, payload: bytes, **_: object) -> ElectronicsSensorFrame:
        if len(payload) != cls.packet_size:
            raise PacketDecodeError(f"sensor_frame_v2 packet must be {cls.packet_size} bytes")
        body, crc_bytes = payload[:-4], payload[-4:]
        expected_crc = struct.unpack("<I", crc_bytes)[0]
        if zlib.crc32(body) & 0xFFFFFFFF != expected_crc:
            raise PacketDecodeError("sensor_frame_v2 CRC mismatch")
        values = cls._body.unpack(body)
        if values[0] != b"ES" or values[1] != 2:
            raise PacketDecodeError("sensor_frame_v2 header/version mismatch")
        flags = values[2]
        floats = values[5:15]
        return ElectronicsSensorFrame(
            sequence=values[3],
            timestamp_ns=values[4],
            acceleration_mps2=tuple(floats[0:3]),
            angular_rate_rps=tuple(floats[3:6]),
            magnetic_field_ut=tuple(floats[6:9]),
            temperature_c=floats[9],
            position_m=(values[15], values[16]),
            heading_rad=values[17],
            imu_valid=bool(flags & 0x01),
            magnetometer_valid=bool(flags & 0x02),
            gnss_valid=bool(flags & 0x04),
            gnss_fresh=bool(flags & 0x08),
        )


def codec_for_name(name: str):
    normalized = str(name).strip().lower()
    if normalized in {"legacy", "legacy_v1", "v1"}:
        return NavImuPacketCodecV1
    if normalized in {"sensor_frame_v2", "full_v2", "v2"}:
        return NavSensorPacketCodecV2
    raise ValueError(f"Unsupported NAV protocol: {name}")

