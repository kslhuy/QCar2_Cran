# `extra/ground_station/utils/logging.py`

## Purpose

Provides namespaced, handler-free loggers for ground-station components.
The operator application decides handlers and output level; this helper never
changes process-wide logging configuration.

## Verification

`test/unit_test_ground_station.py` checks name construction and invalid
component rejection.
