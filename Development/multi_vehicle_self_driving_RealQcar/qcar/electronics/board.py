"""CRAN/SEGULA electronics digital twin orchestration."""

from __future__ import annotations

from dataclasses import fields, replace
from typing import Any, Dict, List, Mapping, Optional, Type, TypeVar

from .buses import BusFaultProfile, CANBus, SPIBus, TimedSerialBus, UARTBus
from .codecs import PacketDecodeError, codec_for_name
from .firmware import BoardFirmwareCore, PythonReferenceFirmwareCore
from .hardware import HardwareTargetManifest
from .models import (
    ElectronicsSensorSuite,
    MCUModel,
    PowerConfig,
    PowerDomainModel,
    SensorSuiteConfig,
)
from .types import (
    BusDelivery,
    BusFrame,
    ElectronicsSensorFrame,
    ElectronicsSnapshot,
    FrameStatus,
    MCUState,
    VehiclePlantSample,
)
from .v2v_protocol import V2V_RX_MAGIC, decode_radio_v2v_rx


T = TypeVar("T")


def _dataclass_config(cls: Type[T], values: Optional[Mapping[str, Any]]) -> T:
    allowed = {item.name for item in fields(cls)}
    filtered = {key: value for key, value in dict(values or {}).items() if key in allowed}
    return cls(**filtered)


def _fault_profile(values: Optional[Mapping[str, Any]]) -> BusFaultProfile:
    return _dataclass_config(BusFaultProfile, values)


class ElectronicsDigitalTwin:
    """Transaction-level model of a configurable electronics target.

    Vehicle plant state is used only as physical truth from which the PCB's own
    sensors generate measurements.  It never reuses the vehicle sensor cache.
    """

    def __init__(
        self,
        vehicle_id: int,
        config: Optional[Mapping[str, Any]] = None,
        *,
        firmware_core: Optional[BoardFirmwareCore] = None,
    ) -> None:
        self.vehicle_id = int(vehicle_id)
        self.config: Dict[str, Any] = dict(config or {})
        self.seed = int(self.config.get("seed", 1000 + self.vehicle_id))
        self.hardware_manifest = HardwareTargetManifest.from_config(
            self.config.get("hardware_target"), vehicle_id=self.vehicle_id
        )
        self.sensor_compute_protocol = str(
            self.config.get(
                "sensor_compute_protocol",
                self.config.get("nav_protocol", "sensor_frame_v2"),
            )
        )
        self.nav_protocol = self.sensor_compute_protocol  # legacy alias
        self.codec = codec_for_name(self.sensor_compute_protocol)
        self.sensor_compute_interface = str(
            self.config.get(
                "sensor_compute_interface",
                self.config.get("nav_com_interface", "spi"),
            )
        ).lower()
        self.nav_com_interface = self.sensor_compute_interface  # legacy alias
        self.vehicle_interface = str(
            self.config.get("vehicle_interface", "uart")
        ).lower()
        self._time_ns = 0
        self._last_transmitted_sensor_sequence = -1
        self._latest_nav_sensor: Optional[ElectronicsSensorFrame] = None
        self._latest_com_sensor: Optional[ElectronicsSensorFrame] = None
        self._vehicle_deliveries: List[BusDelivery] = []
        self._v2v_transmissions: List[bytes] = []
        self._v2v_host_receptions: List[bytes] = []
        self._decode_errors = 0
        self._last_firmware_status: Mapping[str, Any] = {}
        self._vehicle_fragment_sequence = 0
        self._vehicle_fragment_buffers: Dict[str, Dict[str, Any]] = {}

        self.power = PowerDomainModel(
            _dataclass_config(PowerConfig, self.config.get("power"))
        )
        mcu_cfg = dict(self.config.get("mcu", {}))
        node_cfg = dict(self.config.get("nodes", {}))
        sensor_cfg = dict(node_cfg.get("sensor_node", {}))
        compute_cfg = dict(node_cfg.get("compute_node", {}))
        legacy_brownout = float(mcu_cfg.get("brownout_threshold_v", 2.9))
        self.sensor_node = MCUModel(
            self.hardware_manifest.sensor_node.node_id,
            boot_time_s=float(
                sensor_cfg.get(
                    "boot_time_s", mcu_cfg.get("nav_boot_time_s", 0.05)
                )
            ),
            brownout_threshold_v=float(
                sensor_cfg.get("brownout_threshold_v", legacy_brownout)
            ),
        )
        if self.hardware_manifest.topology == "single_node":
            self.compute_node = self.sensor_node
        else:
            self.compute_node = MCUModel(
                self.hardware_manifest.compute_node.node_id,
                boot_time_s=float(
                    compute_cfg.get(
                        "boot_time_s", mcu_cfg.get("com_boot_time_s", 0.08)
                    )
                ),
                brownout_threshold_v=float(
                    compute_cfg.get("brownout_threshold_v", legacy_brownout)
                ),
            )
        # Backward-compatible aliases for the existing CRAN/SEGULA board code.
        self.nav_mcu = self.sensor_node
        self.com_mcu = self.compute_node
        self.sensors = ElectronicsSensorSuite(
            _dataclass_config(SensorSuiteConfig, self.config.get("sensors")),
            seed=self.seed + 11,
        )
        buses_cfg = dict(self.config.get("buses", {}))
        self.sensor_compute_bus = self._create_bus(
            "sensor_compute",
            self.sensor_compute_interface,
            buses_cfg.get("sensor_compute", buses_cfg.get("nav_com", {})),
            seed=self.seed + 21,
        )
        self.nav_com_bus = self.sensor_compute_bus  # legacy alias
        self.vehicle_bus = self._create_bus(
            "vehicle_link",
            self.vehicle_interface,
            buses_cfg.get("vehicle", {}),
            seed=self.seed + 31,
        )
        firmware_cfg = dict(self.config.get("firmware", {}))
        self.firmware_core = firmware_core or self._create_firmware_core(firmware_cfg)
        self._snapshot = self._build_snapshot()

    @property
    def time_ns(self) -> int:
        return self._time_ns

    def reset(self, timestamp_ns: int = 0) -> None:
        self._time_ns = int(timestamp_ns)
        self._last_transmitted_sensor_sequence = -1
        self._latest_nav_sensor = None
        self._latest_com_sensor = None
        self._vehicle_deliveries.clear()
        self._v2v_transmissions.clear()
        self._v2v_host_receptions.clear()
        self._decode_errors = 0
        self._last_firmware_status = {}
        self._vehicle_fragment_sequence = 0
        self._vehicle_fragment_buffers.clear()
        self.sensor_node.reset()
        if self.compute_node is not self.sensor_node:
            self.compute_node.reset()
        self.sensors.reset()
        self.sensor_compute_bus.reset()
        self.vehicle_bus.reset()
        self.firmware_core.reset()
        self._snapshot = self._build_snapshot()

    def step(
        self, plant: VehiclePlantSample, dt_s: float
    ) -> ElectronicsSnapshot:
        if dt_s < 0.0:
            raise ValueError("dt_s must be non-negative")
        self._time_ns = max(self._time_ns, int(plant.timestamp_ns))
        load_current_a = self._estimate_load_current()
        power = self.power.step(load_current_a)
        sensor_state = self.sensor_node.update(
            self._time_ns, power.rail_3v3_v
        )
        compute_state = (
            sensor_state
            if self.compute_node is self.sensor_node
            else self.compute_node.update(self._time_ns, power.rail_3v3_v)
        )

        if sensor_state == MCUState.RUNNING:
            sensor_frame = self.sensors.sample(plant, dt_s)
            if sensor_frame is not None:
                self._latest_nav_sensor = sensor_frame
                if sensor_frame.sequence != self._last_transmitted_sensor_sequence:
                    self._last_transmitted_sensor_sequence = sensor_frame.sequence
                    self.sensor_compute_bus.transmit(
                        self.codec.encode(sensor_frame),
                        source=self.hardware_manifest.sensor_node.node_id,
                        destination=self.hardware_manifest.compute_node.node_id,
                        requested_at_ns=self._time_ns,
                        metadata={
                            "sequence": sensor_frame.sequence,
                            "sensor_timestamp_ns": sensor_frame.timestamp_ns,
                            "protocol": self.codec.name,
                        },
                    )

        for delivery in self.sensor_compute_bus.advance(self._time_ns):
            if (
                delivery.status == FrameStatus.DROPPED
                or compute_state != MCUState.RUNNING
            ):
                continue
            try:
                decoded = self.codec.decode(
                    delivery.payload,
                    sequence=int(delivery.frame.metadata.get("sequence", 0)),
                    timestamp_ns=int(
                        delivery.frame.metadata.get("sensor_timestamp_ns", self._time_ns)
                    ),
                )
                self._latest_com_sensor = decoded
                self.firmware_core.on_nav_frame(decoded, delivery)
            except PacketDecodeError:
                self._decode_errors += 1

        if compute_state == MCUState.RUNNING:
            firmware_result = self.firmware_core.step(self._time_ns)
            self._last_firmware_status = dict(firmware_result.status)
            for payload in firmware_result.vehicle_payloads:
                self._transmit_vehicle_payload(
                    payload, source=self.hardware_manifest.compute_node.node_id
                )
            self._v2v_transmissions.extend(bytes(item) for item in firmware_result.v2v_payloads)

        for delivery in self.vehicle_bus.advance(self._time_ns):
            if delivery.status == FrameStatus.DROPPED:
                message_id = delivery.frame.metadata.get("message_id")
                if message_id is not None:
                    self._vehicle_fragment_buffers.pop(str(message_id), None)
                continue
            delivery = self._reassemble_vehicle_delivery(delivery)
            if delivery is None:
                continue
            if delivery.frame.destination == "vehicle":
                if delivery.payload.startswith(V2V_RX_MAGIC):
                    self._v2v_host_receptions.append(
                        decode_radio_v2v_rx(delivery.payload)
                    )
                else:
                    self._vehicle_deliveries.append(delivery)
            elif (
                delivery.frame.destination
                == self.hardware_manifest.compute_node.node_id
                and compute_state == MCUState.RUNNING
            ):
                self.firmware_core.on_vehicle_payload(delivery.payload, self._time_ns)

        self._snapshot = self._build_snapshot()
        return self._snapshot

    def get_snapshot(self) -> ElectronicsSnapshot:
        return self._snapshot

    def get_status_summary(self) -> Dict[str, Any]:
        snapshot = self._snapshot
        sensor_compute_stats = self.sensor_compute_bus.statistics()
        vehicle_stats = self.vehicle_bus.statistics()
        firmware_status = dict(snapshot.firmware_status)
        firmware_backend = firmware_status.get("backend", "starting")
        hil_runtime_ready = bool(
            firmware_status.get("handshake_ready", False)
            and firmware_status.get("link_alive", False)
        )
        overall_healthy = bool(
            snapshot.healthy
            and (
                firmware_backend != "hil_external"
                or hil_runtime_ready
            )
        )
        return {
            "enabled": True,
            "healthy": overall_healthy,
            "hardware_target": self.hardware_manifest.to_dict(),
            "target_id": self.hardware_manifest.target_id,
            "target_profile": self.hardware_manifest.profile,
            "topology": self.hardware_manifest.topology,
            "sensor_node_state": snapshot.nav_mcu_state.value,
            "compute_node_state": snapshot.com_mcu_state.value,
            "node_states": {
                self.hardware_manifest.sensor_node.node_id: snapshot.nav_mcu_state.value,
                self.hardware_manifest.compute_node.node_id: snapshot.com_mcu_state.value,
            },
            "nav_mcu_state": snapshot.nav_mcu_state.value,
            "com_mcu_state": snapshot.com_mcu_state.value,
            "nav_protocol": self.codec.name,
            "nav_com_interface": self.nav_com_interface,
            "sensor_compute_protocol": self.codec.name,
            "sensor_compute_interface": self.sensor_compute_interface,
            "vehicle_interface": self.vehicle_interface,
            "execution_mode": self.hardware_manifest.compute_node.execution,
            "firmware_backend": firmware_backend,
            "hil": {
                "handshake_ready": firmware_status.get("handshake_ready", False),
                "link_alive": firmware_status.get("link_alive", False),
                "last_receive_age_ns": firmware_status.get(
                    "last_receive_age_ns"
                ),
                "capability_negotiation": firmware_status.get(
                    "capability_negotiation", {}
                ),
                "remote_target": firmware_status.get("remote_target"),
                "remote_status": firmware_status.get("remote_status", {}),
                "transport": firmware_status.get("transport"),
            },
            "rail_3v3_v": snapshot.power.rail_3v3_v,
            "input_voltage_v": snapshot.power.input_voltage_v,
            "nav_frames_delivered": sensor_compute_stats["delivered"],
            "nav_frames_dropped": sensor_compute_stats["dropped"],
            "sensor_compute_frames_delivered": sensor_compute_stats["delivered"],
            "sensor_compute_frames_dropped": sensor_compute_stats["dropped"],
            "nav_decode_errors": self._decode_errors,
            "vehicle_frames_delivered": vehicle_stats["delivered"],
        }

    def pop_vehicle_deliveries(self) -> List[BusDelivery]:
        result = list(self._vehicle_deliveries)
        self._vehicle_deliveries.clear()
        return result

    def pop_v2v_transmissions(self) -> List[bytes]:
        result = list(self._v2v_transmissions)
        self._v2v_transmissions.clear()
        return result

    def pop_v2v_host_receptions(self) -> List[bytes]:
        result = list(self._v2v_host_receptions)
        self._v2v_host_receptions.clear()
        return result

    def inject_vehicle_payload(self, payload: bytes) -> int:
        return self._transmit_vehicle_link(
            bytes(payload),
            source="vehicle",
            destination=self.hardware_manifest.compute_node.node_id,
            direction="vehicle_to_board",
        )

    def inject_v2v_payload(self, payload: bytes) -> bool:
        if self.compute_node.state == MCUState.RUNNING:
            self.firmware_core.on_v2v_payload(bytes(payload), self._time_ns)
            return True
        return False

    def set_bus_faults(self, bus: str, **values: Any) -> None:
        normalized = str(bus).strip().lower()
        if normalized in {
            "sensor",
            "sensor_compute",
            "nav",
            "nav_com",
            "spi",
            "uart_nav",
        }:
            self.sensor_compute_bus.configure_faults(**values)
            return
        if normalized in {"vehicle", "vehicle_link", "can", "uart_vehicle"}:
            self.vehicle_bus.configure_faults(**values)
            return
        raise KeyError(f"Unknown electronics bus: {bus}")

    def set_sensor_fault(self, sensor: str, mode: str, value: float = 0.0) -> None:
        self.sensors.set_fault(sensor, mode, value)

    def set_input_voltage(self, voltage_v: Optional[float]) -> None:
        self.power.set_input_voltage(voltage_v)

    def set_mcu_reset(self, mcu: str, asserted: bool) -> None:
        normalized = str(mcu).strip().lower()
        sensor_aliases = {
            "sensor",
            "sensor_node",
            "nav",
            "f405",
            "stm32f405",
            self.hardware_manifest.sensor_node.node_id.lower(),
            self.hardware_manifest.sensor_node.platform.lower(),
        }
        compute_aliases = {
            "compute",
            "compute_node",
            "com",
            "h753",
            "stm32h753",
            self.hardware_manifest.compute_node.node_id.lower(),
            self.hardware_manifest.compute_node.platform.lower(),
        }
        if normalized in sensor_aliases:
            self.sensor_node.set_reset(asserted)
        elif normalized in compute_aliases:
            self.compute_node.set_reset(asserted)
        else:
            raise KeyError(f"Unknown hardware node: {mcu}")

    def _transmit_vehicle_payload(self, payload: bytes, *, source: str) -> None:
        self._transmit_vehicle_link(
            bytes(payload),
            source=source,
            destination="vehicle",
            direction="board_to_vehicle",
        )

    def _transmit_vehicle_link(
        self,
        payload: bytes,
        *,
        source: str,
        destination: str,
        direction: str,
    ) -> int:
        max_payload = self.vehicle_bus.max_payload_bytes
        if max_payload is None or len(payload) <= max_payload:
            return self.vehicle_bus.transmit(
                payload,
                source=source,
                destination=destination,
                requested_at_ns=self._time_ns,
                metadata={"direction": direction},
            )

        # CAN/CAN-FD fragmentation remains below the firmware/application API.
        chunks = [payload[index : index + max_payload] for index in range(0, len(payload), max_payload)]
        message_id = (
            f"{self.vehicle_id}:{source}:{destination}:"
            f"{self._time_ns}:{self._vehicle_fragment_sequence}"
        )
        self._vehicle_fragment_sequence += 1
        first_frame_id = -1
        for index, chunk in enumerate(chunks):
            frame_id = self.vehicle_bus.transmit(
                chunk,
                source=source,
                destination=destination,
                requested_at_ns=self._time_ns,
                metadata={
                    "direction": direction,
                    "message_id": message_id,
                    "fragment_index": index,
                    "fragment_count": len(chunks),
                    "arbitration_id": 0x300 + self.vehicle_id,
                },
            )
            if first_frame_id < 0:
                first_frame_id = frame_id
        return first_frame_id

    def _reassemble_vehicle_delivery(
        self, delivery: BusDelivery
    ) -> Optional[BusDelivery]:
        metadata = dict(delivery.frame.metadata)
        fragment_count = int(metadata.get("fragment_count", 1))
        if fragment_count <= 1:
            return delivery
        message_id = str(metadata.get("message_id", ""))
        fragment_index = int(metadata.get("fragment_index", -1))
        if not message_id or fragment_index < 0 or fragment_index >= fragment_count:
            return None
        buffer = self._vehicle_fragment_buffers.setdefault(
            message_id,
            {
                "count": fragment_count,
                "chunks": {},
                "delivery": delivery,
            },
        )
        buffer["chunks"][fragment_index] = bytes(delivery.payload)
        if len(buffer["chunks"]) < fragment_count:
            return None
        try:
            payload = b"".join(
                buffer["chunks"][index] for index in range(fragment_count)
            )
        except KeyError:
            return None
        self._vehicle_fragment_buffers.pop(message_id, None)
        metadata["reassembled"] = True
        frame: BusFrame = replace(
            delivery.frame, payload=payload, metadata=metadata
        )
        return replace(delivery, frame=frame, payload=payload)

    def _estimate_load_current(self) -> float:
        # Functional load budget is declared by the target manifest. A
        # consolidated MCU/SoC is counted once even though it owns both roles.
        role_nodes = (
            (self.hardware_manifest.sensor_node, self.sensor_node),
            (self.hardware_manifest.compute_node, self.compute_node),
        )
        node_currents: Dict[str, float] = {}
        for manifest, model in role_nodes:
            current = (
                manifest.active_current_a
                if model.state != MCUState.OFF
                else manifest.idle_current_a
            )
            node_currents[manifest.node_id] = max(
                node_currents.get(manifest.node_id, 0.0), current
            )
        sensors_a = 0.025
        radio_a = 0.060 if self.compute_node.state == MCUState.RUNNING else 0.005
        return sum(node_currents.values()) + sensors_a + radio_a

    def _build_snapshot(self) -> ElectronicsSnapshot:
        firmware_status = dict(self._last_firmware_status)
        firmware_status["nav_decode_errors"] = self._decode_errors
        return ElectronicsSnapshot(
            vehicle_id=self.vehicle_id,
            timestamp_ns=self._time_ns,
            nav_mcu_state=self.nav_mcu.state,
            com_mcu_state=self.com_mcu.state,
            power=self.power.snapshot,
            nav_sensor_frame=self._latest_nav_sensor,
            com_sensor_frame=self._latest_com_sensor,
            nav_protocol=self.codec.name,
            nav_com_interface=self.nav_com_interface,
            vehicle_interface=self.vehicle_interface,
            bus_statistics={
                "sensor_compute": self.sensor_compute_bus.statistics(),
                "nav_com": self.sensor_compute_bus.statistics(),
                "vehicle": self.vehicle_bus.statistics(),
            },
            firmware_status=firmware_status,
        )

    @staticmethod
    def _create_bus(
        name: str,
        interface: str,
        config: Mapping[str, Any],
        *,
        seed: int,
    ) -> TimedSerialBus:
        cfg = dict(config or {})
        faults = _fault_profile(cfg.get("faults"))
        normalized = interface.strip().lower()
        if normalized == "spi":
            return SPIBus(
                name,
                clock_hz=float(cfg.get("clock_hz", 6_000_000.0)),
                chip_select_setup_s=float(cfg.get("chip_select_setup_s", 1e-6)),
                seed=seed,
                faults=faults,
            )
        if normalized == "uart":
            return UARTBus(
                name,
                baud_rate=float(cfg.get("baud_rate", 921_600.0)),
                data_bits=int(cfg.get("data_bits", 8)),
                stop_bits=int(cfg.get("stop_bits", 1)),
                parity_enabled=bool(cfg.get("parity_enabled", False)),
                seed=seed,
                faults=faults,
            )
        if normalized in {"can", "can_fd"}:
            return CANBus(
                name,
                bitrate=float(cfg.get("bitrate", 1_000_000.0)),
                can_fd=bool(cfg.get("can_fd", normalized == "can_fd")),
                seed=seed,
                faults=faults,
            )
        if normalized in {"ethernet", "udp", "ethernet_udp"}:
            return TimedSerialBus(
                name,
                bitrate=float(cfg.get("bitrate", 100_000_000.0)),
                seed=seed,
                faults=faults,
                crc_rejects_corruption=True,
                max_payload_bytes=int(cfg.get("max_payload_bytes", 1500)),
            )
        if normalized in {"shared_memory", "internal", "pcie"}:
            return TimedSerialBus(
                name,
                bitrate=float(cfg.get("bitrate", 10_000_000_000.0)),
                seed=seed,
                faults=faults,
                crc_rejects_corruption=False,
                max_payload_bytes=None,
            )
        raise ValueError(f"Unsupported electronics interface: {interface}")

    def _create_firmware_core(self, config: Mapping[str, Any]) -> BoardFirmwareCore:
        backend = str(config.get("backend", "python_reference")).strip().lower()
        publish_rate_hz = float(config.get("publish_rate_hz", 20.0))
        if backend in {"python", "python_reference"}:
            return PythonReferenceFirmwareCore(
                vehicle_id=self.vehicle_id,
                publish_rate_hz=publish_rate_hz,
            )
        if backend in {"native", "native_cpp", "cpp"}:
            from .native_backend import NativeFirmwareCore

            library_path = config.get("library_path")
            if not library_path:
                raise ValueError(
                    "electronics.firmware.library_path is required for native_cpp"
                )
            return NativeFirmwareCore(
                library_path=str(library_path),
                vehicle_id=self.vehicle_id,
                publish_rate_hz=publish_rate_hz,
            )
        if backend in {"hil", "hil_external", "external_hardware"}:
            from .hil import HILFirmwareCore, create_hil_transport

            hil_config = dict(config.get("hil", {}))
            transport = create_hil_transport(hil_config)
            return HILFirmwareCore(
                vehicle_id=self.vehicle_id,
                local_manifest=self.hardware_manifest,
                transport=transport,
                strict_handshake=bool(hil_config.get("strict_handshake", True)),
                required_features=hil_config.get(
                    "required_features", ["firmware_core"]
                ),
                min_float_width_bits=int(
                    hil_config.get("min_float_width_bits", 32)
                ),
                min_payload_bytes=int(hil_config.get("min_payload_bytes", 256)),
                min_memory_bytes=int(hil_config.get("min_memory_bytes", 65_536)),
                require_dynamic_allocation=bool(
                    hil_config.get("require_dynamic_allocation", True)
                ),
                hello_retry_s=float(hil_config.get("hello_retry_s", 1.0)),
                link_timeout_s=float(hil_config.get("link_timeout_s", 2.0)),
            )
        raise ValueError(f"Unsupported electronics firmware backend: {backend}")

    def close(self) -> None:
        close = getattr(self.firmware_core, "close", None)
        if callable(close):
            close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
