"""Physical-QCar process runner that supplies a device to IOQCar2.

The platform-neutral runtime intentionally does not create hardware.  This
small bootstrap owns the ``pal.products.qcar.QCar`` instance, safely releases
it after the runtime stops, and passes it to ``IOQCar2`` through the existing
``VehicleProcessSpec.resources`` interface.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import logging
import signal
import threading
import time
from typing import Any, Callable, Iterator, Mapping

from core.vehicle_config import ConfigError, load_config
from core.vehicle_process import VehicleProcessSpec, build_vehicle_process_runtime
from extra.platform.base import PlatformProcessContext, ScenarioPlatformProcessManager, StartPolicy


QCarFactory = Callable[..., Any]
QCarLidarFactory = Callable[..., Any]
RuntimeBuilder = Callable[[VehicleProcessSpec], Any]


@dataclass(frozen=True)
class QCarVehicleSetup:
    """One physical vehicle resolved from a selected vehicle configuration."""

    vehicle_id: int
    spec: VehicleProcessSpec


@dataclass(frozen=True)
class QCarSetup:
    """One-vehicle setup that lets QCar use the common scenario-manager form."""

    vehicles: tuple[QCarVehicleSetup, ...]


def _terminate_qcar(qcar: Any) -> None:
    """Release the PAL session, which sends its own final neutral command."""

    try:
        terminate = getattr(qcar, "terminate", None)
        if callable(terminate):
            terminate()
    except Exception:
        logging.getLogger(__name__).exception("Unable to terminate QCar session")


def _default_qcar_factory(**kwargs: Any) -> Any:
    try:
        from pal.products.qcar import QCar
    except ImportError as error:
        raise ConfigError(
            "QCar hardware support is unavailable. Install the Quanser PAL Python "
            "environment on the vehicle before using extra.platform.qcar.process_runner."
        ) from error
    return QCar(**kwargs)


def _default_qcar_lidar_factory(**kwargs: Any) -> Any:
    try:
        from pal.products.qcar import QCarLidar
    except ImportError as error:
        raise ConfigError(
            "QCar LiDAR support is unavailable. Install the Quanser PAL Python "
            "environment on the vehicle before enabling qcar_hardware LiDAR."
        ) from error
    return QCarLidar(**kwargs)


def _terminate_lidar(lidar: Any) -> None:
    """Release a PAL LiDAR resource without touching QCar actuation."""

    try:
        terminate = getattr(lidar, "terminate", None)
        if callable(terminate):
            terminate()
    except Exception:
        logging.getLogger(__name__).exception("Unable to terminate QCar LiDAR session")


class QCarLidarResourceManager:
    """Own an optional PAL LiDAR session on demand for one QCar process.

    The physical adapter may ask this platform-owned manager to acquire the
    device only after the operator enables LiDAR diagnostics.  Consequently,
    constructing the normal vehicle runtime does not start the QCar LiDAR
    motor.  The manager intentionally contains all PAL construction and
    termination code; ``IOQCar2`` only receives this small lifecycle contract.
    """

    def __init__(self, factory: QCarLidarFactory, options: Mapping[str, Any]) -> None:
        self._factory = factory
        self._options = dict(options)
        self._lock = threading.RLock()
        self._lidar: Any | None = None

    @property
    def available(self) -> bool:
        """A configured PAL resource can be acquired by the local adapter."""

        return True

    def acquire(self) -> Any:
        """Return the one PAL session, creating it only when first requested."""

        with self._lock:
            if self._lidar is None:
                self._lidar = self._factory(**self._options)
            return self._lidar

    def release(self) -> None:
        """Terminate the active PAL session, if any."""

        with self._lock:
            lidar, self._lidar = self._lidar, None
        if lidar is not None:
            _terminate_lidar(lidar)

    def close(self) -> None:
        """Alias used by the owning process context during final cleanup."""

        self.release()


@contextmanager
def qcar_lidar_resource_context(
    *,
    lidar_factory: QCarLidarFactory | None = None,
    num_measurements: int = 384,
    enable_filtering: bool = True,
) -> Iterator[Any]:
    """Own one direct PAL LiDAR session for a diagnostic or runtime context."""

    factory = lidar_factory or _default_qcar_lidar_factory
    lidar = factory(numMeasurements=num_measurements, enableFiltering=enable_filtering)
    try:
        yield lidar
    finally:
        _terminate_lidar(lidar)


@contextmanager
def qcar_resource_context(
    *,
    qcar_factory: QCarFactory | None = None,
    lidar_factory: QCarLidarFactory | None = None,
    lidar_config: Mapping[str, Any] | None = None,
    read_mode: int = 1,
):
    """Own selected PAL QCar resources and expose them to the IO adapter."""

    factory = qcar_factory or _default_qcar_factory
    qcar = factory(readMode=read_mode)
    lidar = None
    lidar_manager = None
    try:
        options = _qcar_lidar_options(lidar_config)
        if options is not None:
            lidar_factory = lidar_factory or _default_qcar_lidar_factory
            # Explicit capture configurations retain the direct resource for
            # their short, self-contained diagnostic lifetime.  The standard
            # runtime starts with LiDAR disabled and receives a lazy manager.
            if bool(lidar_config.get("enabled", False)):
                lidar = lidar_factory(**options)
            else:
                lidar_manager = QCarLidarResourceManager(lidar_factory, options)
        resources = {"qcar": qcar}
        if lidar is not None:
            resources["lidar"] = lidar
        if lidar_manager is not None:
            resources["lidar_manager"] = lidar_manager
        yield resources
    finally:
        if lidar_manager is not None:
            lidar_manager.close()
        if lidar is not None:
            _terminate_lidar(lidar)
        _terminate_qcar(qcar)


def _qcar_lidar_options(lidar_config: Mapping[str, Any] | None) -> dict[str, Any] | None:
    if lidar_config is None:
        return None
    if not isinstance(lidar_config, Mapping):
        raise ConfigError("QCar sensors.lidar must be a mapping")
    source = lidar_config.get("source", "qcar_hardware")
    if source != "qcar_hardware":
        raise ConfigError("Physical QCar runner supports only sensors.lidar.source: qcar_hardware")
    num_measurements = lidar_config.get("num_measurements", 384)
    if not isinstance(num_measurements, int) or isinstance(num_measurements, bool) or num_measurements <= 0:
        raise ConfigError("QCar sensors.lidar.num_measurements must be a positive integer")
    filtering = lidar_config.get("filtering", True)
    if not isinstance(filtering, bool):
        raise ConfigError("QCar sensors.lidar.filtering must be a boolean")
    return {"numMeasurements": num_measurements, "enableFiltering": filtering}


def _load_qcar_setup(spec: VehicleProcessSpec) -> QCarSetup:
    """Resolve one QCar configuration into the shared one-vehicle setup shape."""

    overrides = dict(spec.value_overrides or {})
    if spec.vehicle_id is not None:
        overrides["vehicle_id"] = spec.vehicle_id
    config = load_config(
        vehicle_config_file=spec.vehicle_config_file,
        selection_overrides=spec.selection_overrides,
        value_overrides=overrides,
    )
    if config.module("io").get("implementation") != "qcar":
        raise ConfigError("extra.platform.qcar.process_runner requires a vehicle configuration with modules.io: qcar")
    return QCarSetup(vehicles=(QCarVehicleSetup(vehicle_id=config.vehicle_id, spec=spec),))


class QCarProcessManager(ScenarioPlatformProcessManager):
    """Physical-QCar adapter for the shared platform process-manager contract."""

    platform_name = "QCar"
    start_policy = StartPolicy.OPERATOR_COMMAND
    pace = True

    def __init__(
        self,
        spec: VehicleProcessSpec,
        *,
        read_mode: int = 1,
        qcar_factory: QCarFactory | None = None,
        lidar_factory: QCarLidarFactory | None = None,
    ) -> None:
        super().__init__(spec.vehicle_config_file, spec.vehicle_id)
        self._spec = spec
        self._read_mode = read_mode
        self._qcar_factory = qcar_factory
        self._lidar_factory = lidar_factory

    def load_setup(self, setup_file: str) -> QCarSetup:
        """Load the selected vehicle configuration as a one-vehicle setup."""

        if setup_file != self._spec.vehicle_config_file:
            raise ValueError("QCar setup file does not match its vehicle configuration")
        return _load_qcar_setup(self._spec)

    def build_process_spec(self, setup: QCarSetup, vehicle: QCarVehicleSetup) -> VehicleProcessSpec:
        """Return the original selected configuration for the resolved vehicle."""

        del setup
        return vehicle.spec

    def resource_context(self, context: PlatformProcessContext):
        """Open the PAL device for exactly the shared process lifecycle."""

        del context
        io_config = _load_qcar_io_config(self._spec)
        sensors = io_config.get("sensors", {})
        lidar_config = sensors.get("lidar") if isinstance(sensors, Mapping) else None
        return qcar_resource_context(
            qcar_factory=self._qcar_factory,
            lidar_factory=self._lidar_factory,
            lidar_config=lidar_config,
            read_mode=self._read_mode,
        )


def run_qcar_process(
    spec: VehicleProcessSpec,
    *,
    cycles: int | None = None,
    read_mode: int = 1,
    qcar_factory: QCarFactory | None = None,
    lidar_factory: QCarLidarFactory | None = None,
    runtime_builder: RuntimeBuilder = build_vehicle_process_runtime,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
) -> list[object]:
    """Run the configured actor with a physical QCar supplied to ``IOQCar2``.

    This function never injects a START command.  The runtime remains safety
    gated in READY and continuously writes zero until its configured
    ground-station interface supplies a valid START command.
    """

    if cycles is not None and cycles <= 0:
        raise ValueError("cycles must be positive")
    if not isinstance(read_mode, int) or isinstance(read_mode, bool) or read_mode < 0:
        raise ValueError("read_mode must be a non-negative integer")

    manager = QCarProcessManager(
        spec,
        read_mode=read_mode,
        qcar_factory=qcar_factory,
        lidar_factory=lidar_factory,
    )
    _, telemetry = manager.run(
        cycles,
        runtime_builder=runtime_builder,
        sleep=sleep,
        monotonic=monotonic,
    )
    return telemetry


def _load_qcar_io_config(spec: VehicleProcessSpec) -> dict[str, Any]:
    overrides = dict(spec.value_overrides or {})
    if spec.vehicle_id is not None:
        overrides["vehicle_id"] = spec.vehicle_id
    config = load_config(
        vehicle_config_file=spec.vehicle_config_file,
        selection_overrides=spec.selection_overrides,
        value_overrides=overrides,
    )
    io_config = config.module("io")
    if io_config.get("implementation") != "qcar":
        raise ConfigError("extra.platform.qcar.process_runner requires modules.io: qcar")
    return io_config


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Run one physical QCar through the refactored IOQCar2 runtime."
    )
    parser.add_argument("--vehicle-id", type=int, help="override the configured vehicle ID")
    parser.add_argument("--cycles", type=int, default=None, help="stop after this many control-loop cycles")
    parser.add_argument("--vehicle-config", default="config_vehicle.yaml", help="vehicle profile file in config/")
    parser.add_argument("--read-mode", type=int, default=1, help="PAL QCar readMode (default: 1)")
    parser.add_argument("--ground-station", choices=("null", "tcp_client"), help="ground-station bridge profile")
    parser.add_argument("--ground-station-host", help="override TCP ground-station host")
    parser.add_argument("--ground-station-port", type=int, help="override TCP ground-station listener port")
    arguments = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    value_overrides: dict[str, Any] = {}
    selection_overrides: dict[str, str] = {}
    if arguments.ground_station is not None:
        selection_overrides["ground_station"] = arguments.ground_station
    if arguments.ground_station_host is not None or arguments.ground_station_port is not None:
        ground_station_values: dict[str, Any] = {}
        if arguments.ground_station_host is not None:
            ground_station_values["server_host"] = arguments.ground_station_host
        if arguments.ground_station_port is not None:
            ground_station_values["server_port"] = arguments.ground_station_port
        value_overrides["modules"] = {"ground_station": ground_station_values}

    def _handle_termination(signum, frame) -> None:
        del signum, frame
        raise KeyboardInterrupt

    previous_sigterm = signal.signal(signal.SIGTERM, _handle_termination)
    try:
        try:
            run_qcar_process(
                VehicleProcessSpec(
                    vehicle_id=arguments.vehicle_id,
                    vehicle_config_file=arguments.vehicle_config,
                    selection_overrides=selection_overrides or None,
                    value_overrides=value_overrides or None,
                ),
                cycles=arguments.cycles,
                read_mode=arguments.read_mode,
            )
        except KeyboardInterrupt:
            return 0
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
