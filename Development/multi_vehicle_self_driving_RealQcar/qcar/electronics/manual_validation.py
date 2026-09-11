"""Repeatable end-to-end validation for the CRAN/SEGULA electronics twin.

Run from the qcar directory with::

    python -m electronics.manual_validation --scenario all --output simulation/results/electronics_manual_validation.json
"""

from __future__ import annotations

import argparse
import json
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Iterable

import numpy as np
import yaml

from Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator import (
    TrustBasedFleetEstimator,
)

from .board import ElectronicsDigitalTwin
from .hardware import HardwareTargetManifest, available_hardware_profiles
from .hil import HILFirmwareCore, LoopbackHILTransport, ReferenceHILDevice
from .native_backend import NativeFirmwareCore
from .types import MCUState, VehiclePlantSample
from .v2v_bridge import ElectronicsV2VBridge


PACKAGE_DIR = Path(__file__).resolve().parent
QCAR_DIR = PACKAGE_DIR.parent
NATIVE_LIBRARY = PACKAGE_DIR / "native" / "build" / "cran_electronics_core.dll"
TRUST_CONFIG = (
    QCAR_DIR
    / "Observer"
    / "TrustbasedDistributedObserver"
    / "config_trust_estimator.yaml"
)


class _Clock:
    def __init__(self) -> None:
        self.now_ns = 0

    def __call__(self) -> int:
        return self.now_ns


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def _plant(timestamp_ns: int, x: float = 0.0) -> VehiclePlantSample:
    return VehiclePlantSample(
        timestamp_ns=timestamp_ns,
        x_m=x,
        y_m=0.2,
        heading_rad=0.04,
        velocity_x_mps=0.45,
        velocity_y_mps=0.0,
        yaw_rate_rps=0.02,
        acceleration_x_mps2=0.05,
        acceleration_y_mps2=0.0,
    )


def _board_config(*, sensor_rate_hz: float = 100.0) -> Dict[str, Any]:
    _require(
        NATIVE_LIBRARY.is_file(),
        f"Native library is missing: {NATIVE_LIBRARY}. Build electronics/native first.",
    )
    return {
        "seed": 7301,
        "nav_protocol": "sensor_frame_v2",
        "nav_com_interface": "spi",
        "vehicle_interface": "can_fd",
        "mcu": {"nav_boot_time_s": 0.0, "com_boot_time_s": 0.0},
        "sensors": {
            "imu_rate_hz": sensor_rate_hz,
            "gnss_rate_hz": sensor_rate_hz,
            "magnetometer_rate_hz": sensor_rate_hz,
            "accel_noise_std_mps2": 0.0,
            "gyro_noise_std_rps": 0.0,
            "magnetometer_noise_std_ut": 0.0,
            "gnss_position_noise_std_m": 0.0,
            "gnss_heading_noise_std_rad": 0.0,
            "accel_bias_walk_std_mps2_sqrt_s": 0.0,
            "gyro_bias_walk_std_rps_sqrt_s": 0.0,
        },
        "firmware": {
            "backend": "native_cpp",
            "library_path": str(NATIVE_LIBRARY),
            "publish_rate_hz": sensor_rate_hz,
        },
    }


def _step_twin(twin: ElectronicsDigitalTwin, start: int, stop: int) -> None:
    for step in range(start, stop):
        timestamp_ns = step * 10_000_000
        twin.step(_plant(timestamp_ns, x=0.45 * timestamp_ns / 1e9), 0.01)


def validate_sensor_pipeline() -> Dict[str, Any]:
    """Validate plant -> PCB sensors -> NAV -> SPI -> COM C++ -> CAN-FD."""
    twin = ElectronicsDigitalTwin(0, _board_config())
    _step_twin(twin, 1, 20)
    snapshot = twin.get_snapshot()
    deliveries = twin.pop_vehicle_deliveries()
    decoded = [json.loads(item.payload.decode("utf-8")) for item in deliveries]

    _require(snapshot.sensor_node_state == MCUState.RUNNING, "Sensor node did not boot")
    _require(snapshot.compute_node_state == MCUState.RUNNING, "Compute node did not boot")
    _require(
        snapshot.compute_node_sensor_frame is not None,
        "Compute node received no PCB sensor frame",
    )
    _require(snapshot.compute_node_sensor_frame.imu_valid, "PCB IMU sample is invalid")
    _require(snapshot.compute_node_sensor_frame.gnss_valid, "PCB GNSS sample is invalid")
    _require(bool(deliveries), "Native compute firmware emitted no vehicle frame")
    _require(
        decoded[-1].get("type") == "electronics_sensor",
        "Vehicle frame schema is not electronics_sensor",
    )
    status = twin.get_status_summary()
    _require(status["firmware_backend"] == "native_cpp", "C++ firmware not active")
    return {
        "firmware_backend": status["firmware_backend"],
        "target_profile": status["target_profile"],
        "sensor_node_state": snapshot.sensor_node_state.value,
        "compute_node_state": snapshot.compute_node_state.value,
        "nav_frames_delivered": status["nav_frames_delivered"],
        "vehicle_frames_delivered": status["vehicle_frames_delivered"],
        "last_sensor_sequence": snapshot.compute_node_sensor_frame.sequence,
    }


def validate_fault_injection() -> Dict[str, Any]:
    """Validate GNSS dropout, power brownout, radio delay and radio loss."""
    twin = ElectronicsDigitalTwin(0, _board_config())
    _step_twin(twin, 1, 8)
    twin.set_sensor_fault("gnss", "dropout")
    _step_twin(twin, 8, 14)
    dropped_snapshot = twin.get_snapshot()
    _require(
        dropped_snapshot.compute_node_sensor_frame is not None
        and not dropped_snapshot.compute_node_sensor_frame.gnss_valid,
        "GNSS dropout did not reach COM",
    )

    twin.set_input_voltage(2.0)
    brownout = twin.step(_plant(140_000_000), 0.01)
    _require(brownout.power.brownout, "2 V input did not cause brownout")
    _require(
        brownout.sensor_node_state == MCUState.BROWNOUT
        and brownout.compute_node_state == MCUState.BROWNOUT,
        "Brownout did not reset both MCUs",
    )

    clock = _Clock()
    radio_twin = ElectronicsDigitalTwin(2, _board_config(sensor_rate_hz=0.0))
    radio_twin.step(_plant(1), 0.001)
    bridge = ElectronicsV2VBridge(
        radio_twin,
        {
            "mode": "mirror",
            "seed": 812,
            "faults": {"fixed_delay_s": 0.08, "jitter_s": 0.0},
        },
        clock_ns=clock,
    )
    sent = []
    bridge.outbound(
        b"delay-probe",
        3,
        ("peer-three", 8003),
        lambda payload, address: sent.append((payload, address)) or True,
    )
    bridge.pump(79_000_000)
    _require(not sent, "Delayed V2V frame arrived too early")
    bridge.pump(100_000_000)
    _require(len(sent) == 1, "Delayed V2V frame was not delivered")

    bridge.configure_faults(
        enabled=True,
        fixed_delay_s=0.0,
        jitter_s=0.0,
        drop_probability=1.0,
        bit_error_rate=0.0,
    )
    clock.now_ns = 200_000_000
    bridge.outbound(
        b"loss-probe",
        3,
        ("peer-three", 8003),
        lambda payload, address: sent.append((payload, address)) or True,
    )
    bridge.pump(300_000_000)
    radio_status = bridge.get_status()["radio"]
    _require(len(sent) == 1, "100% loss profile delivered a frame")
    _require(radio_status["dropped"] >= 1, "Radio drop counter did not increment")
    return {
        "gnss_dropout_reached_compute": True,
        "brownout": brownout.power.brownout,
        "sensor_node_state_at_2v": brownout.sensor_node_state.value,
        "compute_node_state_at_2v": brownout.compute_node_state.value,
        "delay_probe_latency_ns": 100_000_000,
        "radio_dropped": radio_status["dropped"],
    }


def validate_v2v_transport() -> Dict[str, Any]:
    """Validate a datagram through two C++ compute targets."""
    clock = _Clock()
    twin_a = ElectronicsDigitalTwin(0, _board_config(sensor_rate_hz=0.0))
    twin_b = ElectronicsDigitalTwin(1, _board_config(sensor_rate_hz=0.0))
    twin_a.step(_plant(10_000_000), 0.01)
    twin_b.step(_plant(10_000_000), 0.01)
    bridge_a = ElectronicsV2VBridge(twin_a, {"mode": "firmware"}, clock_ns=clock)
    bridge_b = ElectronicsV2VBridge(twin_b, {"mode": "firmware"}, clock_ns=clock)
    received = []

    def radio_to_b(payload: bytes, address: tuple[str, int]) -> bool:
        bridge_b.inbound(payload, ("vehicle-a", 8000), lambda data, peer: received.append((data, peer)))
        return True

    payload = b"msgpack-schema-remains-opaque" * 20
    accepted = bridge_a.outbound(payload, 1, ("vehicle-b", 8001), radio_to_b)
    _require(accepted, "Source COM rejected host V2V payload")
    for step in range(2, 30):
        timestamp_ns = step * 10_000_000
        clock.now_ns = timestamp_ns
        twin_a.step(_plant(timestamp_ns), 0.01)
        twin_b.step(_plant(timestamp_ns), 0.01)
        bridge_a.pump()
        bridge_b.pump()

    _require(received and received[0][0] == payload, "V2V payload changed or was lost")
    status_a = bridge_a.get_status()
    status_b = bridge_b.get_status()
    _require(status_a["firmware_tx_emitted"] == 1, "Source COM did not emit once")
    _require(status_b["host_rx_delivered"] == 1, "Destination host did not receive once")
    return {
        "payload_bytes": len(payload),
        "source_firmware_tx": status_a["firmware_tx_emitted"],
        "radio_tx_delivered": status_a["radio_tx_delivered"],
        "destination_host_rx": status_b["host_rx_delivered"],
        "source_firmware_backend": twin_a.get_status_summary()["firmware_backend"],
        "destination_firmware_backend": twin_b.get_status_summary()["firmware_backend"],
    }


def validate_hardware_abstraction() -> Dict[str, Any]:
    """Validate a vendor-neutral Linux SoC target over the HIL contract."""
    remote = HardwareTargetManifest.from_config(
        {
            "profile": "raspberry_pi_hil",
            "target_id": "vehicle-0-rpi5",
            "topology": "single_node",
            "sensor_node": {
                "node_id": "vehicle_compute",
                "platform": "raspberry_pi_5",
                "architecture": "aarch64",
                "runtime": "linux",
                "execution": "hil_external",
                "word_size_bits": 64,
                "float_width_bits": 64,
                "max_payload_bytes": 65536,
                "transports": ["shared_memory", "can_fd", "ethernet", "udp"],
                "features": ["sensor_acquisition", "timestamping"],
            },
            "compute_node": {
                "node_id": "vehicle_compute",
                "platform": "raspberry_pi_5",
                "architecture": "aarch64",
                "runtime": "linux",
                "execution": "hil_external",
                "word_size_bits": 64,
                "float_width_bits": 64,
                "max_payload_bytes": 65536,
                "transports": ["shared_memory", "can_fd", "ethernet", "udp"],
                "features": ["firmware_core", "trust", "observer", "v2v"],
            },
        },
        vehicle_id=0,
    )
    device_core = NativeFirmwareCore(
        library_path=str(NATIVE_LIBRARY),
        vehicle_id=0,
        publish_rate_hz=100.0,
    )
    device = ReferenceHILDevice(
        manifest=remote,
        vehicle_id=0,
        firmware_core=device_core,
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
            "sensor_compute_interface": "shared_memory",
            "vehicle_interface": "ethernet",
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
        _step_twin(twin, 1, 15)
        status = twin.get_status_summary()
        deliveries = twin.pop_vehicle_deliveries()
        _require(status["topology"] == "single_node", "SoC topology changed")
        _require(status["hil"]["handshake_ready"], "HIL handshake was rejected")
        _require(
            status["hil"]["capability_negotiation"].get("accepted", False),
            str(status["hil"]["capability_negotiation"].get("blockers", [])),
        )
        _require(bool(deliveries), "External SoC produced no vehicle payload")
        static_candidates = list(
            (PACKAGE_DIR / "native" / "build").glob(
                "*cran_electronics_core_static*"
            )
        )
        _require(bool(static_candidates), "Portable static core was not built")
        return {
            "target_id": status["target_id"],
            "profile": status["target_profile"],
            "topology": status["topology"],
            "remote_platform": status["hil"]["capability_negotiation"][
                "remote_platform"
            ],
            "remote_execution": status["hil"]["capability_negotiation"][
                "remote_execution"
            ],
            "hil_handshake_ready": status["hil"]["handshake_ready"],
            "remote_firmware_backend": status["hil"]["remote_status"].get(
                "backend"
            ),
            "available_profiles": list(available_hardware_profiles()),
            "portable_static_library": static_candidates[0].name,
        }
    finally:
        twin.close()
        device_core.close()


def _feed_estimator_step(
    estimator: TrustBasedFleetEstimator,
    step: int,
    host: np.ndarray,
    dt: float,
) -> None:
    timestamp_ns = int((1.0 + step * dt) * 1e9)
    x_target = 1.0 + 0.45 * step * dt
    target = {
        "x": x_target,
        "y": 0.05,
        "theta": 0.02,
        "velocity": 0.45,
        "acceleration": 0.0,
        "control_input": {"steering": 0.01, "throttle": 0.1},
    }
    estimator.add_received_local_state(1, target, timestamp_ns)
    estimator.add_received_clean_local_state(1, target, timestamp_ns)
    estimator.add_received_fleet_state(
        1,
        {
            0: {
                "x": float(host[0]),
                "y": float(host[1]),
                "theta": float(host[2]),
                "velocity": float(host[3]),
                "acceleration": float(host[4]),
            },
            1: target,
        },
        timestamp_ns,
    )
    estimator.add_received_fleet_state(
        2,
        {
            1: {
                "x": x_target + 0.01,
                "y": 0.045,
                "theta": 0.021,
                "velocity": 0.44,
                "acceleration": 0.0,
            }
        },
        timestamp_ns,
    )
    host[0] += host[3] * dt
    estimator.update(host.copy(), dt, timestamp_ns, np.array([0.0, 0.1]))


def validate_trust_authority(comparisons: int) -> Dict[str, Any]:
    """Open the four-stage parity gate and use C++ outputs as authority."""
    _require(comparisons >= 1, "Authority comparison count must be positive")
    with TRUST_CONFIG.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    config["embedded_core"]["library_path"] = str(NATIVE_LIBRARY)
    config["embedded_core"]["authority_min_comparisons_per_stage"] = comparisons
    config["embedded_core"]["mode"] = "shadow"

    estimator = TrustBasedFleetEstimator(
        vehicle_id=0,
        fleet_size=3,
        state_dim=5,
        config=config,
        logger=None,
    )
    try:
        dt = 0.04
        host = np.array([0.0, 0.0, 0.0, 0.4, 0.0])
        for step in range(comparisons + 15):
            _feed_estimator_step(estimator, step, host, dt)

        shadow_status = estimator.get_embedded_core_status()
        _require(shadow_status["available"], shadow_status["reason"])
        _require(shadow_status["authority_ready"], "; ".join(shadow_status["authority_blockers"]))
        _require(shadow_status["failures"] == 0, "Parity failure before handover")
        authority = estimator.set_embedded_core_mode("native_authority")
        _require(authority.get("mode_request_accepted", False), "Authority request rejected")

        for step in range(comparisons + 15, comparisons + 20):
            _feed_estimator_step(estimator, step, host, dt)
        active = estimator.get_embedded_core_status()
        _require(active["authority_active"], "Native authority did not remain active")
        _require(active["failback_count"] == 0, "Native authority failed back")
        for stage in ("trust", "weight", "correction", "prediction"):
            _require(active[f"native_{stage}_uses"] > 0, f"Native {stage} output was not used")
        return {
            "gate_per_stage": comparisons,
            "mode": active["mode"],
            "comparisons": active["comparisons"],
            "failures": active["failures"],
            "max_errors": {
                "trust": active["max_trust_error"],
                "weight": active["max_weight_error"],
                "correction": active["max_state_error"],
                "prediction": active["max_prediction_error"],
            },
            "native_uses": {
                "trust": active["native_trust_uses"],
                "weight": active["native_weight_uses"],
                "correction": active["native_correction_uses"],
                "prediction": active["native_prediction_uses"],
            },
            "failback_count": active["failback_count"],
        }
    finally:
        estimator.__del__()


def _execute(name: str, callback: Callable[[], Dict[str, Any]]) -> Dict[str, Any]:
    started = time.perf_counter()
    try:
        details = callback()
        return {
            "status": "PASS",
            "duration_s": round(time.perf_counter() - started, 3),
            "details": details,
        }
    except Exception as exc:
        return {
            "status": "FAIL",
            "duration_s": round(time.perf_counter() - started, 3),
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        }


def _scenario_names(requested: str) -> Iterable[str]:
    if requested == "all":
        return (
            "sensor_pipeline",
            "fault_injection",
            "v2v_transport",
            "hardware_abstraction",
            "trust_authority",
        )
    return (requested,)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        choices=(
            "all",
            "sensor_pipeline",
            "fault_injection",
            "v2v_transport",
            "hardware_abstraction",
            "trust_authority",
        ),
        default="all",
    )
    parser.add_argument(
        "--authority-comparisons",
        type=int,
        default=1000,
        help="Required zero-failure comparisons for each native Trust/Observer stage.",
    )
    parser.add_argument("--output", type=Path, help="Optional JSON report path.")
    args = parser.parse_args()

    callbacks: Dict[str, Callable[[], Dict[str, Any]]] = {
        "sensor_pipeline": validate_sensor_pipeline,
        "fault_injection": validate_fault_injection,
        "v2v_transport": validate_v2v_transport,
        "hardware_abstraction": validate_hardware_abstraction,
        "trust_authority": lambda: validate_trust_authority(args.authority_comparisons),
    }
    results = {
        name: _execute(name, callbacks[name])
        for name in _scenario_names(args.scenario)
    }
    passed = all(item["status"] == "PASS" for item in results.values())
    report = {
        "suite": "CRAN/SEGULA Electronics Digital Twin Manual Validation",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "native_library": str(NATIVE_LIBRARY),
        "overall_status": "PASS" if passed else "FAIL",
        "results": results,
    }
    rendered = json.dumps(report, indent=2, ensure_ascii=False)
    print(rendered)
    if args.output:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")
        print(f"Report written to: {output}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
