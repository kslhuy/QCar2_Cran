# `extra/ground_station/configuration.py`

## Purpose

Loads and validates only
`extra/ground_station/config/ground_station.yaml`. The operator cannot select
another ground-station application YAML at runtime; templates and local copies
are intentionally not used.

## Boundary

This is not `config/config_ground_station.yaml`, which configures each
vehicle-side outbound bridge. The bridge must target the listener address and
port defined by the fixed-path application YAML.
