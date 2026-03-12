import tkinter as tk
from typing import Optional, Callable
from dataclasses import dataclass, field

@dataclass
class CarState:
    """Data class holding current state of a car."""

    car_id: int
    connected: bool = False
    state: str = "Unknown"
    position: tuple = (0.0, 0.0)
    velocity: float = 0.0
    heading: float = 0.0
    throttle: float = 0.0
    steering: float = 0.0
    v2v_active: bool = False
    v2v_peers: int = 0
    platoon_enabled: bool = False
    platoon_is_leader: bool = False
    platoon_position: Optional[int] = None
    manual_mode: bool = False
    perception_active: bool = False
    scopes_active: bool = False
    local_observer_type: str = "unknown"
    fleet_observer_type: str = "unknown"
    path_long_ctrl: str = "unknown"
    path_lat_ctrl: str = "unknown"
    leader_long_ctrl: str = "unknown"
    leader_lat_ctrl: str = "unknown"
    gear: str = "DRIVE_1"
    online_sysid_status: dict = field(default_factory=dict)
    online_calibration_status: dict = field(default_factory=dict)

@dataclass
class CarPanelCallbacks:
    """Callbacks for car panel actions."""

    on_start: Callable[[int], None] = None
    on_stop: Callable[[int], None] = None
    on_calibrate: Callable[[int], None] = None
    on_set_velocity: Callable[[int, float], None] = None
    on_set_path: Callable[[int, list], None] = None
    on_set_initial_position: Callable[[int, float, float, float, bool], None] = None
    on_toggle_manual: Callable[[int], None] = None
    on_update_control_type: Callable[[int, str], None] = None
    on_platoon_position_change: Callable[[int, int], None] = None
    on_toggle_perception: Callable[[int], None] = None
    on_toggle_probing: Callable[[int], None] = None
    on_toggle_scopes: Callable[[int], None] = None
    on_toggle_remote_plot_local: Callable[[int], None] = None
    on_toggle_remote_plot_fleet: Callable[[int], None] = None
    on_set_local_observer: Callable[[int, str], None] = None
    on_set_fleet_observer: Callable[[int, str], None] = None
    on_set_controller: Callable[[int, str, str, str], None] = None
    on_set_controller_params: Callable[[int, str, dict, str], None] = None
    on_set_online_sysid: Callable[[int, str, dict], None] = None
    on_start_online_calibration: Callable[[int], None] = None
    on_stop_online_calibration: Callable[[int], None] = None
    on_trigger_online_analysis: Callable[[int, str, dict], None] = None
    on_clear_online_calibration: Callable[[int], None] = None
    on_trigger_active_calibration: Callable[[int, str, dict], None] = None
    on_set_gear: Callable[[int, str], None] = None
    on_enable_taxi_mode: Callable[[int], None] = None
    on_disable_taxi_mode: Callable[[int], None] = None
    on_set_taxi_trip: Callable[[int, list], None] = None
