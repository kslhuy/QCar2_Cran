"""CRAN/SEGULA electronics digital twin package."""

from .board import ElectronicsDigitalTwin
from .buses import BusFaultProfile, CANBus, SPIBus, UARTBus
from .codecs import NavImuPacketCodecV1, NavSensorPacketCodecV2, PacketDecodeError
from .firmware import BoardFirmwareCore, FirmwareStepResult, PythonReferenceFirmwareCore
from .hardware import (
    HardwareNodeManifest,
    HardwareTargetManifest,
    available_hardware_profiles,
    negotiate_capabilities,
)
from .hil import (
    HILFirmwareCore,
    HILFrame,
    HILMessageType,
    HILTransport,
    LoopbackHILTransport,
    ReferenceHILDevice,
    UDPHILTransport,
)
from .integration import ElectronicsDataGateway, MockQCarElectronicsAdapter
from .native_backend import NativeFirmwareCore
from .trust_observer_native import (
    NativeTrustObserverCore,
    TrustObserverShadow,
    discover_native_library,
)
from .v2v_bridge import ElectronicsV2VBridge
from .types import (
    ElectronicsSensorFrame,
    ElectronicsSnapshot,
    MCUState,
    VehiclePlantSample,
)

__all__ = [
    "BoardFirmwareCore",
    "BusFaultProfile",
    "CANBus",
    "ElectronicsDataGateway",
    "ElectronicsDigitalTwin",
    "ElectronicsV2VBridge",
    "ElectronicsSensorFrame",
    "ElectronicsSnapshot",
    "FirmwareStepResult",
    "HardwareNodeManifest",
    "HardwareTargetManifest",
    "HILFirmwareCore",
    "HILFrame",
    "HILMessageType",
    "HILTransport",
    "LoopbackHILTransport",
    "MCUState",
    "MockQCarElectronicsAdapter",
    "NativeFirmwareCore",
    "NativeTrustObserverCore",
    "NavImuPacketCodecV1",
    "NavSensorPacketCodecV2",
    "PacketDecodeError",
    "PythonReferenceFirmwareCore",
    "ReferenceHILDevice",
    "SPIBus",
    "TrustObserverShadow",
    "UARTBus",
    "UDPHILTransport",
    "VehiclePlantSample",
    "discover_native_library",
    "available_hardware_profiles",
    "negotiate_capabilities",
]
