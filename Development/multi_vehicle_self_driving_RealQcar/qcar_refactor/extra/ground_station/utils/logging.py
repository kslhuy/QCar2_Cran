"""Namespaced logging helpers for operator-side ground-station components."""

from __future__ import annotations

import logging


def get_ground_station_logger(component: str) -> logging.Logger:
    """Return the standard ground-station logger for one named component.

    Handler selection remains an operator application concern, so this helper
    never installs handlers or changes process-wide logging configuration.
    """

    if not isinstance(component, str) or not component.strip():
        raise ValueError("Ground-station logger component must be a non-empty string")
    return logging.getLogger(f"extra.ground_station.{component.strip()}")
