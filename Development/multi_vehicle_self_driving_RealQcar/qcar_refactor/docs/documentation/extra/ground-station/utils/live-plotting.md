# `extra/ground_station/utils/live_plotting.py`

## Purpose

Provides optional Qt/pyqtgraph views for already-normalized diagnostic data.
`LiveLidarViewer` renders bounded `LaserScanSample` values supplied by the
ground-station server; `LiveLidarLocalizationViewer` is an explicit CARLA
integration diagnostic. Neither opens vehicle IO or changes localisation.

The module also retains the standalone UDP receiver only for isolated capture
troubleshooting. Normal operation uses the registered vehicle TCP session.
