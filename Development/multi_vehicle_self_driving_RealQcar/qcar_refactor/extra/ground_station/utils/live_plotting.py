"""Qt/pyqtgraph viewer for an explicitly requested sensor diagnostic stream.

This is a ground-station presentation tool, not a vehicle runtime or a second
localisation consumer.  It receives only bounded UDP frames sent by an
explicit diagnostic capture.  The camera pane is intentionally a placeholder
until a dependency-free ``CameraFrame`` contract and sender exist.
"""

from __future__ import annotations

import argparse
import math
import socket
import sys
from typing import Sequence

from core.vehicle_types import LaserScanSample, PoseMeasurement
from utils.ground_station.sensor_diagnostic import (
    LidarDiagnosticFrame,
    MAX_DIAGNOSTIC_PACKET_BYTES,
    SensorDiagnosticError,
    decode_lidar_diagnostic,
)


class LivePlottingError(RuntimeError):
    """Raised when an optional live diagnostic plot cannot start."""


class UdpLidarDiagnosticListener:
    """Non-blocking bounded diagnostic receiver owned by one ground station.

    It is intentionally separate from the Qt presentation class.  The
    standalone viewer and the terminal CLI can therefore share exactly the
    same packet validation and route frames by vehicle ID without each binding
    the same UDP port.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 5001) -> None:
        if not isinstance(host, str) or not host:
            raise LivePlottingError("UDP listener host must be a non-empty string")
        if not isinstance(port, int) or isinstance(port, bool) or not 1 <= port <= 65535:
            raise LivePlottingError("UDP listener port must be in [1, 65535]")
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._listener.bind((host, port))
            self._listener.setblocking(False)
        except OSError as error:
            self._listener.close()
            raise LivePlottingError(f"Unable to listen for diagnostic UDP on {host}:{port}: {error}") from error

    def drain(self) -> tuple[dict[int, LidarDiagnosticFrame], int]:
        """Return the newest valid frame per vehicle plus malformed-frame count."""

        newest: dict[int, LidarDiagnosticFrame] = {}
        invalid = 0
        while True:
            try:
                packet, _address = self._listener.recvfrom(MAX_DIAGNOSTIC_PACKET_BYTES + 1)
            except BlockingIOError:
                break
            except OSError:
                break
            try:
                frame = decode_lidar_diagnostic(packet)
            except SensorDiagnosticError:
                invalid += 1
                continue
            newest[frame.vehicle_id] = frame
        return newest, invalid

    def close(self) -> None:
        self._listener.close()


class LiveLidarViewer:
    """One Qt LiDAR plot that accepts already-normalized local scans.

    A local CARLA diagnostic calls :meth:`show_scan` immediately after
    ``IOCarla`` produces a common scan. The UDP command-line mode below uses
    the same presentation class after decoding an explicit QCar packet.
    Neither mode can access a normal runtime's local scan queue by itself.
    """

    def __init__(self, *, title: str = "QCar Sensor Diagnostic Viewer") -> None:
        try:
            import numpy as np
            import pyqtgraph as pg
            from PyQt5 import QtCore, QtWidgets
        except ImportError as error:
            raise LivePlottingError(
                "Live plotting requires PyQt5, pyqtgraph, and numpy in the ground-station "
                f"Python environment. Interpreter: {sys.executable}. Import error: {error}"
            ) from error
        self._np = np
        self._pg = pg
        self._app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self._window = QtWidgets.QWidget()
        self._window.setWindowTitle(title)
        layout = QtWidgets.QVBoxLayout(self._window)
        self._status = QtWidgets.QLabel("Waiting for a normalized LiDAR scan…")
        layout.addWidget(self._status)
        lidar_plot = pg.PlotWidget(title="Normalized LiDAR (local sensor frame)")
        lidar_plot.setLabel("bottom", "x", units="m")
        lidar_plot.setLabel("left", "y", units="m")
        lidar_plot.showGrid(x=True, y=True, alpha=0.25)
        lidar_plot.setAspectLocked(True)
        lidar_plot.addLegend()
        self._scan_points = pg.ScatterPlotItem(
            size=5,
            pen=None,
            brush=pg.mkBrush(70, 160, 255, 180),
            name="LiDAR",
        )
        sensor_origin = pg.ScatterPlotItem(
            size=12,
            symbol="+",
            pen=pg.mkPen("y", width=2),
            name="sensor origin",
        )
        sensor_origin.setData([0.0], [0.0])
        lidar_plot.addItem(self._scan_points)
        lidar_plot.addItem(sensor_origin)
        layout.addWidget(lidar_plot, stretch=3)
        camera_placeholder = QtWidgets.QLabel(
            "Camera diagnostic preview: reserved for the future CameraFrame sender.\n"
            "It is intentionally not coupled to the vehicle runtime."
        )
        camera_placeholder.setMinimumHeight(64)
        camera_placeholder.setStyleSheet("border: 1px solid #777; padding: 8px;")
        layout.addWidget(camera_placeholder)
        self._received = 0
        self._window.resize(900, 760)
        self._window.show()

    def show_scan(self, scan: LaserScanSample, *, vehicle_id: int = 0) -> None:
        """Render one scan supplied explicitly by a local diagnostic caller."""

        if not isinstance(scan, LaserScanSample):
            raise TypeError("LiveLidarViewer.show_scan requires a LaserScanSample")
        ranges = self._np.asarray(scan.ranges_m, dtype=float)
        angles = self._np.asarray(
            [
                scan.angle_min_rad + index * scan.angle_increment_rad
                for index in range(len(scan.ranges_m))
            ],
            dtype=float,
        )
        valid = self._np.isfinite(ranges) & (ranges >= scan.range_min_m) & (ranges <= scan.range_max_m)
        self._scan_points.setData(
            ranges[valid] * self._np.cos(angles[valid]),
            ranges[valid] * self._np.sin(angles[valid]),
        )
        self._received += 1
        self._status.setText(
            f"vehicle {vehicle_id} | frame {scan.frame_id} | scan {self._received} | "
            f"valid points {int(valid.sum())}/{len(ranges)}"
        )

    def process_events(self) -> bool:
        """Process pending GUI events without taking control of a runtime loop."""

        self._app.processEvents()
        return self._window.isVisible()

    def close(self) -> None:
        """Close this viewer window without stopping its caller's runtime."""

        self._window.close()


class LiveLidarLocalizationViewer:
    """Two-panel live view for an explicit runtime integration diagnostic.

    The caller owns the runtime, localiser, and scan queue. This presentation
    object only receives already-derived values: map-frame positions on the
    left and local-frame current/reference scans on the right.
    """

    def __init__(self, route_xy, *, title: str = "CARLA LiDAR runtime integration") -> None:
        try:
            import numpy as np
            import pyqtgraph as pg
            from PyQt5 import QtCore, QtWidgets
        except ImportError as error:
            raise LivePlottingError(
                "Live plotting requires PyQt5, pyqtgraph, and numpy in the ground-station "
                f"Python environment. Interpreter: {sys.executable}. Import error: {error}"
            ) from error
        route = np.asarray(route_xy, dtype=float)
        if route.ndim != 2 or route.shape[1] != 2 or len(route) < 2:
            raise ValueError("route_xy must contain at least two [x, y] positions")
        self._np = np
        self._pg = pg
        self._app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self._window = QtWidgets.QWidget()
        self._window.setWindowTitle(title)
        layout = QtWidgets.QVBoxLayout(self._window)
        self._status = QtWidgets.QLabel("Waiting for VehicleRuntime LiDAR scans…")
        layout.addWidget(self._status)
        plots = pg.GraphicsLayoutWidget()
        layout.addWidget(plots, stretch=1)

        self._map_plot = plots.addPlot(row=0, col=0, title="Localisation and planned route (map frame)")
        self._map_plot.setLabel("bottom", "map x", units="m")
        self._map_plot.setLabel("left", "map y", units="m")
        self._map_plot.showGrid(x=True, y=True, alpha=0.25)
        self._map_plot.setAspectLocked(True)
        self._map_plot.addLegend()
        self._map_plot.plot(route[:, 0], route[:, 1], pen=pg.mkPen("#666666", width=1, style=QtCore.Qt.DashLine), name="route")
        self._carla_trace = self._map_plot.plot(pen=pg.mkPen("#2e7d32", width=2), name="CARLA pose")
        self._ekf_trace = self._map_plot.plot(pen=pg.mkPen("#1565c0", width=2), name="EKF")
        self._lidar_trace = self._map_plot.plot(pen=pg.mkPen("#ef6c00", width=2), name="LiDAR localization")
        self._reference_pose = pg.ScatterPlotItem(
            size=12, symbol="star", pen=pg.mkPen("#aa00ff", width=2), name="reference pose"
        )
        self._map_plot.addItem(self._reference_pose)

        self._scan_plot = plots.addPlot(row=0, col=1, title="LiDAR scans (local sensor frame)")
        self._scan_plot.setLabel("bottom", "forward x", units="m")
        self._scan_plot.setLabel("left", "left y", units="m")
        self._scan_plot.showGrid(x=True, y=True, alpha=0.25)
        self._scan_plot.setAspectLocked(True)
        self._scan_plot.addLegend()
        self._reference_points = pg.ScatterPlotItem(
            size=4, pen=None, brush=pg.mkBrush(140, 140, 140, 110), name="reference scan"
        )
        self._current_points = pg.ScatterPlotItem(
            size=5, pen=None, brush=pg.mkBrush(70, 160, 255, 190), name="current scan"
        )
        sensor_origin = pg.ScatterPlotItem(
            size=12, symbol="+", pen=pg.mkPen("y", width=2), name="sensor origin"
        )
        sensor_origin.setData([0.0], [0.0])
        self._scan_plot.addItem(self._reference_points)
        self._scan_plot.addItem(self._current_points)
        self._scan_plot.addItem(sensor_origin)
        self._carla_positions: list[tuple[float, float]] = []
        self._ekf_positions: list[tuple[float, float]] = []
        self._lidar_positions: list[tuple[float, float]] = []
        self._updates = 0
        self._window.resize(1300, 760)
        self._window.show()

    def show_state(
        self,
        *,
        current_scan: LaserScanSample | None,
        reference_scan: LaserScanSample,
        carla_position: tuple[float, float],
        ekf_position: tuple[float, float],
        lidar_position: PoseMeasurement,
        reference_position: PoseMeasurement,
    ) -> None:
        """Render explicitly supplied runtime/localiser outputs once."""

        if not isinstance(reference_scan, LaserScanSample):
            raise TypeError("reference_scan must be a LaserScanSample")
        if current_scan is not None and not isinstance(current_scan, LaserScanSample):
            raise TypeError("current_scan must be a LaserScanSample or None")
        if not isinstance(lidar_position, PoseMeasurement) or not isinstance(reference_position, PoseMeasurement):
            raise TypeError("LiDAR positions must be PoseMeasurement values")
        self._set_scan_points(self._reference_points, reference_scan)
        if current_scan is not None:
            finite_count = self._set_scan_points(self._current_points, current_scan)
        else:
            finite_count = 0
        self._carla_positions.append((float(carla_position[0]), float(carla_position[1])))
        self._ekf_positions.append((float(ekf_position[0]), float(ekf_position[1])))
        if lidar_position.valid:
            self._lidar_positions.append((lidar_position.x_m, lidar_position.y_m))
        self._set_trace(self._carla_trace, self._carla_positions)
        self._set_trace(self._ekf_trace, self._ekf_positions)
        self._set_trace(self._lidar_trace, self._lidar_positions)
        self._reference_pose.setData([reference_position.x_m], [reference_position.y_m])
        self._updates += 1
        self._status.setText(
            f"runtime integration | update {self._updates} | current finite bins {finite_count} | "
            f"LiDAR localisation {'valid' if lidar_position.valid else 'unavailable'}"
        )

    def process_events(self) -> bool:
        """Process GUI events without owning the VehicleRuntime loop."""

        self._app.processEvents()
        return self._window.isVisible()

    def close(self) -> None:
        self._window.close()

    def _set_scan_points(self, item, scan: LaserScanSample) -> int:
        ranges = self._np.asarray(scan.ranges_m, dtype=float)
        angles = scan.angle_min_rad + self._np.arange(len(ranges), dtype=float) * scan.angle_increment_rad
        valid = self._np.isfinite(ranges) & (ranges >= scan.range_min_m) & (ranges <= scan.range_max_m)
        item.setData(ranges[valid] * self._np.cos(angles[valid]), ranges[valid] * self._np.sin(angles[valid]))
        self._scan_plot.setXRange(-scan.range_max_m, scan.range_max_m, padding=0.02)
        self._scan_plot.setYRange(-scan.range_max_m, scan.range_max_m, padding=0.02)
        return int(valid.sum())

    def _set_trace(self, line, positions: list[tuple[float, float]]) -> None:
        if not positions:
            line.setData([], [])
            return
        values = self._np.asarray(positions, dtype=float)
        line.setData(values[:, 0], values[:, 1])


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Show explicit QCar LiDAR diagnostics in a Qt/pyqtgraph window"
    )
    parser.add_argument("--host", default="0.0.0.0", help="UDP listener address")
    parser.add_argument("--port", type=int, default=5001, help="UDP listener port")
    parser.add_argument("--duration-s", type=float, default=None, help="close after this duration")
    arguments = parser.parse_args(argv)
    if not 1 <= arguments.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if arguments.duration_s is not None and arguments.duration_s <= 0.0:
        parser.error("--duration-s must be positive")
    try:
        return _run_viewer(arguments.host, arguments.port, arguments.duration_s)
    except LivePlottingError as error:
        parser.exit(2, f"error: {error}\n")


def _run_viewer(host: str, port: int, duration_s: float | None) -> int:
    try:
        from PyQt5 import QtCore, QtWidgets
    except ImportError as error:
        raise LivePlottingError(
            "Live plotting requires PyQt5, pyqtgraph, and numpy in the ground-station "
            f"Python environment. Interpreter: {sys.executable}. Import error: {error}"
        ) from error

    listener = UdpLidarDiagnosticListener(host, port)

    viewer = LiveLidarViewer(title=f"QCar Sensor Diagnostic Viewer — UDP {host}:{port}")
    app = QtWidgets.QApplication.instance()
    if app is None:  # Defensive; LiveLidarViewer always creates one.
        raise LivePlottingError("Unable to initialize a Qt application")

    received = 0
    invalid = 0

    def poll_udp() -> None:
        nonlocal received, invalid
        newest, rejected = listener.drain()
        invalid += rejected
        if not newest:
            return
        # The standalone viewer follows the most recently decoded vehicle if
        # more than one diagnostic target happens to use this listener.
        frame = next(reversed(newest.values()))
        viewer.show_scan(frame.scan, vehicle_id=frame.vehicle_id)
        received += 1

    timer = QtCore.QTimer()
    timer.timeout.connect(poll_udp)
    timer.start(20)
    if duration_s is not None:
        QtCore.QTimer.singleShot(int(duration_s * 1000.0), app.quit)
    try:
        return int(app.exec_())
    finally:
        timer.stop()
        listener.close()
        viewer.close()


if __name__ == "__main__":
    raise SystemExit(main())
