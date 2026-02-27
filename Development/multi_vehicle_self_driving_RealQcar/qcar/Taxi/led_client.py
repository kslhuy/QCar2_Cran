"""
led_client.py  —  Lightweight UDP client for controlling QCar2 LEDs
                   from pure Python code (no rclpy dependency).

This module sends a single UDP datagram to the ``taxi_led_bridge`` ROS 2
node, which in turn forwards the colour to the ``qcar2_hardware`` node.

LED Colour Map (matching qcar2_hardware.cpp):
    0 = Red        — stop / hub arrival
    1 = Green      — driving
    2 = Blue       — passenger pick-up
    3 = Yellow     — passenger drop-off
    4 = Cyan       — intermediate stop
    5 = Magenta    — idle at hub

Usage
-----
    from Taxi.led_client import LedClient

    led = LedClient()           # default 127.0.0.1:5050
    led.set_color(1)            # green
    led.set_color("driving")    # same thing, by name

Or use the module-level convenience function:

    from Taxi.led_client import set_led
    set_led("pickup")           # blue
"""

import json
import socket
from typing import Dict, Optional, Union

# ── Taxi-mode stop-type → LED colour mapping ──────────────────────────
# Keys match TaxiManager.get_current_stop_type() return values.
STOP_TYPE_TO_LED: Dict[str, int] = {
    "hub":          5,  # magenta  — idle at hub
    "pickup":       2,  # blue     — passenger pick-up
    "dropoff":      3,  # yellow   — passenger drop-off
    "intermediate": 4,  # cyan     — intermediate stop
    "driving":      1,  # green    — driving
}


class LedClient:
    """Fire-and-forget UDP client to the taxi_led_bridge ROS 2 node."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5050):
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._current_id: int = -1

    def set_color(self, color: Union[int, str]) -> None:
        """
        Set the LED strip colour.

        Parameters
        ----------
        color : int or str
            Either a numeric LED id (0-5) or a taxi stop-type string
            ("hub", "pickup", "dropoff", "intermediate", "driving").
        """
        if isinstance(color, str):
            led_id = STOP_TYPE_TO_LED.get(color.lower(), -1)
            if led_id < 0:
                raise ValueError(
                    f"Unknown stop type '{color}'. "
                    f"Valid: {list(STOP_TYPE_TO_LED)}"
                )
        else:
            led_id = int(color)

        if led_id == self._current_id:
            return  # skip duplicate

        if not (0 <= led_id <= 5):
            raise ValueError(f"LED id must be 0-5, got {led_id}")

        self._current_id = led_id
        payload = json.dumps({"led_id": led_id}).encode("utf-8")
        try:
            self._sock.sendto(payload, self._addr)
        except OSError:
            # Non-critical: if bridge isn't up yet, silently drop
            pass

    def close(self):
        """Close the underlying socket (optional, cleaned up on GC)."""
        try:
            self._sock.close()
        except OSError:
            pass


# ── Module-level convenience ──────────────────────────────────────────
_default_client: Optional[LedClient] = None


def set_led(color: Union[int, str], host: str = "127.0.0.1", port: int = 5050) -> None:
    """
    One-liner LED control.  Lazily creates a shared ``LedClient``.

    Examples
    --------
    >>> set_led("driving")   # green
    >>> set_led(5)           # magenta
    """
    global _default_client
    if _default_client is None:
        _default_client = LedClient(host=host, port=port)
    _default_client.set_color(color)
