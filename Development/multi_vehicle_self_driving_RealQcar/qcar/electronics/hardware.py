"""Hardware-neutral target manifests and capability negotiation."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple


MANIFEST_SCHEMA_VERSION = 1


_PROFILES: Dict[str, Dict[str, Any]] = {
    "cran_segula_stm32": {
        "topology": "dual_node",
        "sensor_node": {
            "node_id": "sensor_node",
            "platform": "stm32f405",
            "architecture": "arm_cortex_m4f",
            "runtime": "bare_metal",
            "execution": "simulated",
            "clock_hz": 168_000_000,
            "float_width_bits": 32,
            "transports": ["spi", "uart"],
            "features": ["sensor_acquisition", "timestamping"],
            "active_current_a": 0.045,
            "idle_current_a": 0.003,
        },
        "compute_node": {
            "node_id": "compute_node",
            "platform": "stm32h753",
            "architecture": "arm_cortex_m7",
            "runtime": "bare_metal",
            "execution": "sil_native",
            "clock_hz": 480_000_000,
            "float_width_bits": 64,
            "transports": ["spi", "uart", "usb", "ethernet"],
            "features": ["firmware_core", "trust", "observer", "v2v"],
            "active_current_a": 0.080,
            "idle_current_a": 0.004,
        },
    },
    "generic_dual_mcu": {
        "topology": "dual_node",
        "sensor_node": {
            "node_id": "sensor_node",
            "platform": "generic_mcu",
            "architecture": "unspecified",
            "runtime": "bare_metal",
            "execution": "hil_external",
            "float_width_bits": 32,
            "transports": ["spi", "uart", "can_fd"],
            "features": ["sensor_acquisition", "timestamping"],
        },
        "compute_node": {
            "node_id": "compute_node",
            "platform": "generic_mcu",
            "architecture": "unspecified",
            "runtime": "bare_metal_or_rtos",
            "execution": "hil_external",
            "float_width_bits": 64,
            "transports": ["uart", "can_fd", "ethernet"],
            "features": ["firmware_core", "trust", "observer", "v2v"],
        },
    },
    "generic_single_mcu": {
        "topology": "single_node",
        "sensor_node": {
            "node_id": "vehicle_compute",
            "platform": "generic_mcu_or_soc",
            "architecture": "unspecified",
            "runtime": "bare_metal_or_rtos",
            "execution": "hil_external",
            "float_width_bits": 64,
            "transports": ["shared_memory", "can_fd", "ethernet"],
            "features": ["sensor_acquisition", "timestamping"],
        },
        "compute_node": {
            "node_id": "vehicle_compute",
            "platform": "generic_mcu_or_soc",
            "architecture": "unspecified",
            "runtime": "bare_metal_or_rtos",
            "execution": "hil_external",
            "float_width_bits": 64,
            "transports": ["shared_memory", "can_fd", "ethernet"],
            "features": [
                "sensor_acquisition",
                "timestamping",
                "firmware_core",
                "trust",
                "observer",
                "v2v",
            ],
        },
    },
    "linux_soc": {
        "topology": "single_node",
        "sensor_node": {
            "node_id": "linux_compute",
            "platform": "generic_linux_soc",
            "architecture": "aarch64_or_x86_64",
            "runtime": "linux",
            "execution": "hil_external",
            "word_size_bits": 64,
            "float_width_bits": 64,
            "transports": ["shared_memory", "udp", "ethernet", "can_fd"],
            "features": ["sensor_acquisition", "timestamping"],
        },
        "compute_node": {
            "node_id": "linux_compute",
            "platform": "generic_linux_soc",
            "architecture": "aarch64_or_x86_64",
            "runtime": "linux",
            "execution": "hil_external",
            "word_size_bits": 64,
            "float_width_bits": 64,
            "transports": ["shared_memory", "udp", "ethernet", "can_fd"],
            "features": ["firmware_core", "trust", "observer", "v2v"],
        },
    },
    "desktop_sil": {
        "topology": "single_node",
        "sensor_node": {
            "node_id": "desktop_native",
            "platform": "desktop",
            "architecture": "native",
            "runtime": "host_os",
            "execution": "sil_native",
            "word_size_bits": 64,
            "float_width_bits": 64,
            "transports": ["shared_memory", "udp"],
            "features": ["sensor_acquisition", "timestamping"],
        },
        "compute_node": {
            "node_id": "desktop_native",
            "platform": "desktop",
            "architecture": "native",
            "runtime": "host_os",
            "execution": "sil_native",
            "word_size_bits": 64,
            "float_width_bits": 64,
            "transports": ["shared_memory", "udp"],
            "features": ["firmware_core", "trust", "observer", "v2v"],
        },
    },
}


def _merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> Dict[str, Any]:
    result = deepcopy(dict(base))
    for key, value in dict(override).items():
        if isinstance(value, Mapping) and isinstance(result.get(key), Mapping):
            result[key] = _merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def available_hardware_profiles() -> Tuple[str, ...]:
    return tuple(sorted(_PROFILES))


@dataclass(frozen=True)
class HardwareNodeManifest:
    """Capabilities of one logical hardware role, independent of vendor HAL."""

    node_id: str
    role: str
    platform: str = "generic"
    architecture: str = "unspecified"
    runtime: str = "unspecified"
    execution: str = "simulated"
    clock_hz: int = 0
    word_size_bits: int = 32
    endianness: str = "little"
    float_width_bits: int = 64
    max_payload_bytes: int = 65_536
    memory_bytes: int = 524_288
    dynamic_allocation: bool = True
    transports: Tuple[str, ...] = field(default_factory=tuple)
    features: Tuple[str, ...] = field(default_factory=tuple)
    active_current_a: float = 0.050
    idle_current_a: float = 0.004

    @classmethod
    def from_dict(
        cls, role: str, values: Optional[Mapping[str, Any]] = None
    ) -> "HardwareNodeManifest":
        data = dict(values or {})
        node_id = str(data.get("node_id", role)).strip() or role
        endianness = str(data.get("endianness", "little")).strip().lower()
        if endianness not in {"little", "big"}:
            raise ValueError(f"Unsupported endianness for {node_id}: {endianness}")
        float_width = int(data.get("float_width_bits", 64))
        if float_width not in {32, 64}:
            raise ValueError("float_width_bits must be 32 or 64")
        execution = str(data.get("execution", "simulated")).strip().lower()
        if execution not in {"simulated", "sil_native", "hil_external", "physical"}:
            raise ValueError(f"Unsupported execution mode: {execution}")
        return cls(
            node_id=node_id,
            role=str(role),
            platform=str(data.get("platform", "generic")),
            architecture=str(data.get("architecture", "unspecified")),
            runtime=str(data.get("runtime", "unspecified")),
            execution=execution,
            clock_hz=max(0, int(data.get("clock_hz", 0))),
            word_size_bits=max(8, int(data.get("word_size_bits", 32))),
            endianness=endianness,
            float_width_bits=float_width,
            max_payload_bytes=max(1, int(data.get("max_payload_bytes", 65_536))),
            memory_bytes=max(0, int(data.get("memory_bytes", 524_288))),
            dynamic_allocation=bool(data.get("dynamic_allocation", True)),
            transports=tuple(
                str(item).strip().lower()
                for item in data.get("transports", ())
                if str(item).strip()
            ),
            features=tuple(
                str(item).strip().lower()
                for item in data.get("features", ())
                if str(item).strip()
            ),
            active_current_a=max(0.0, float(data.get("active_current_a", 0.050))),
            idle_current_a=max(0.0, float(data.get("idle_current_a", 0.004))),
        )

    def to_dict(self) -> Dict[str, Any]:
        result = asdict(self)
        result["transports"] = list(self.transports)
        result["features"] = list(self.features)
        return result


@dataclass(frozen=True)
class HardwareTargetManifest:
    """Topology and capability declaration exchanged by SIL/HIL peers."""

    target_id: str
    profile: str
    topology: str
    sensor_node: HardwareNodeManifest
    compute_node: HardwareNodeManifest
    schema_version: int = MANIFEST_SCHEMA_VERSION
    metadata: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def from_config(
        cls,
        values: Optional[Mapping[str, Any]] = None,
        *,
        vehicle_id: int = 0,
    ) -> "HardwareTargetManifest":
        raw = dict(values or {})
        profile = str(raw.get("profile", "cran_segula_stm32")).strip().lower()
        if profile in _PROFILES:
            base = _PROFILES[profile]
        elif "sensor_node" in raw and "compute_node" in raw:
            # A vendor/project-specific target is valid when it is described
            # completely; profiles are conveniences, not an allow-list.
            base = {}
        else:
            choices = ", ".join(available_hardware_profiles())
            raise ValueError(
                f"Unknown incomplete hardware profile '{profile}'; "
                f"provide both nodes or choose {choices}"
            )
        merged = _merge(base, raw)
        topology = str(merged.get("topology", "dual_node")).strip().lower()
        if topology not in {"single_node", "dual_node"}:
            raise ValueError("hardware topology must be single_node or dual_node")
        sensor = HardwareNodeManifest.from_dict(
            "sensor_node", merged.get("sensor_node")
        )
        compute = HardwareNodeManifest.from_dict(
            "compute_node", merged.get("compute_node")
        )
        if topology == "single_node" and sensor.node_id != compute.node_id:
            raise ValueError(
                "single_node topology requires identical sensor/compute node_id"
            )
        return cls(
            target_id=str(merged.get("target_id", f"vehicle_{vehicle_id}_electronics")),
            profile=profile,
            topology=topology,
            sensor_node=sensor,
            compute_node=compute,
            schema_version=int(merged.get("schema_version", MANIFEST_SCHEMA_VERSION)),
            metadata=dict(merged.get("metadata", {})),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "target_id": self.target_id,
            "profile": self.profile,
            "topology": self.topology,
            "sensor_node": self.sensor_node.to_dict(),
            "compute_node": self.compute_node.to_dict(),
            "metadata": dict(self.metadata),
        }


def negotiate_capabilities(
    remote: HardwareTargetManifest,
    *,
    required_features: Sequence[str] = ("firmware_core",),
    min_float_width_bits: int = 32,
    min_payload_bytes: int = 256,
    min_memory_bytes: int = 65_536,
    require_dynamic_allocation: bool = True,
) -> Dict[str, Any]:
    """Check only algorithm requirements; vendor and operating system are free."""
    blockers = []
    compute = remote.compute_node
    remote_features = set(compute.features)
    for feature in required_features:
        if str(feature).lower() not in remote_features:
            blockers.append(f"missing feature: {feature}")
    if compute.float_width_bits < int(min_float_width_bits):
        blockers.append(
            f"float width {compute.float_width_bits} < {min_float_width_bits}"
        )
    if compute.max_payload_bytes < int(min_payload_bytes):
        blockers.append(
            f"payload {compute.max_payload_bytes} < {min_payload_bytes} bytes"
        )
    if compute.memory_bytes < int(min_memory_bytes):
        blockers.append(
            f"memory {compute.memory_bytes} < {min_memory_bytes} bytes"
        )
    if require_dynamic_allocation and not compute.dynamic_allocation:
        blockers.append("dynamic allocation is required by the current core")
    if remote.schema_version != MANIFEST_SCHEMA_VERSION:
        blockers.append(
            f"manifest schema {remote.schema_version} != {MANIFEST_SCHEMA_VERSION}"
        )
    return {
        "accepted": not blockers,
        "blockers": blockers,
        "remote_target_id": remote.target_id,
        "remote_profile": remote.profile,
        "remote_platform": compute.platform,
        "remote_execution": compute.execution,
    }
