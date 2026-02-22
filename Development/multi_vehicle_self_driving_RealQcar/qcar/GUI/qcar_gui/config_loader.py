"""
Config Loader for QCar Fleet Controller.

Loads observer and controller configurations from YAML files so the GUI
and web app can dynamically discover available types and their parameters
instead of relying on hard-coded lists.
"""

import os
import yaml
from typing import Dict, Any, Optional


# Resolve paths relative to this file.
# Layout:  GUI/qcar_gui/config_loader.py
#          Observer/config_local_estimators.yaml
#          Observer/config_fleet_estimators.yaml
#          Controller/config_controller_sim.yaml
#          Controller/config_controller_real.yaml
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_QCAR_DIR = os.path.dirname(os.path.dirname(_THIS_DIR))  # …/qcar

_LOCAL_OBS_PATH = os.path.join(_QCAR_DIR, "Observer", "config_local_estimators.yaml")
_FLEET_OBS_PATH = os.path.join(_QCAR_DIR, "Observer", "config_fleet_estimators.yaml")
_CTRL_SIM_PATH = os.path.join(_QCAR_DIR, "Controller", "config_controller_sim.yaml")
_CTRL_REAL_PATH = os.path.join(_QCAR_DIR, "Controller", "config_controller_real.yaml")


def _load_yaml(path: str) -> Dict[str, Any]:
    """Safely load a YAML file, returning empty dict on failure."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        print(f"[ConfigLoader] ⚠️  Config file not found: {path}")
        return {}
    except Exception as e:
        print(f"[ConfigLoader] ⚠️  Error loading {path}: {e}")
        return {}


def _flatten_params(d: Any) -> Dict[str, Any]:
    """Flatten a nested dict into a single-level dict of scalar params.

    Only keeps numeric, boolean, and string values (skips nested dicts).
    This is used to extract tunable parameters from YAML controller sections.
    """
    if not isinstance(d, dict):
        return {}
    result = {}
    for k, v in d.items():
        if isinstance(v, (int, float, bool, str)):
            result[k] = v
    return result


# ── Observer Config ─────────────────────────────────────────────────


def load_local_observer_config() -> Dict[str, Any]:
    """Load local observer config and extract available types + params."""
    cfg = _load_yaml(_LOCAL_OBS_PATH)
    local_section = cfg.get("local", {})

    # Available types are the keys under `local:`
    available = list(local_section.keys()) if local_section else []

    # Current default
    default_type = cfg.get("local_estimator_type", available[0] if available else "ekf")

    # Parameters per type
    params = {}
    for obs_type, obs_cfg in local_section.items():
        params[obs_type] = _flatten_params(obs_cfg) if isinstance(obs_cfg, dict) else {}

    return {
        "available": available,
        "default": default_type,
        "params": params,
    }


def load_fleet_observer_config() -> Dict[str, Any]:
    """Load fleet observer config and extract available types + params."""
    cfg = _load_yaml(_FLEET_OBS_PATH)
    fleet_section = cfg.get("fleet", {})

    # Filter out global settings (non-dict or common keys)
    _global_keys = {
        "state_dim",
        "max_state_age_ns",
        "received_local_states_limit",
        "received_fleet_states_limit",
        "fleet_observer_rate",
    }

    available = [
        k
        for k, v in fleet_section.items()
        if isinstance(v, dict) and k not in _global_keys
    ]

    default_type = cfg.get(
        "fleet_estimator_type", available[0] if available else "consensus"
    )

    params = {}
    for obs_type in available:
        params[obs_type] = _flatten_params(fleet_section.get(obs_type, {}))

    return {
        "available": available,
        "default": default_type,
        "params": params,
    }


# ── Controller Config ───────────────────────────────────────────────

# Known longitudinal and lateral controller names to classify them.
# These are used to split the flat YAML into longitudinal vs lateral lists.
_KNOWN_LONGITUDINAL = {"pid", "cacc", "sa_acc", "fix"}
_KNOWN_LATERAL = {
    "pure_pursuit",
    "pp_map",
    "stanley",
    "lookahead",
    "hybrid",
    "fusion",
    "hybrid_lateral",
    "fusion_lateral",
    "path",
}


def load_controller_config(mode: str = "sim") -> Dict[str, Any]:
    """Load controller config for the given mode ('sim' or 'real').

    Returns a structured dict with available types, defaults, and params.
    """
    path = _CTRL_SIM_PATH if mode == "sim" else _CTRL_REAL_PATH
    cfg = _load_yaml(path)

    # ── Defaults ────────────────────────────────────────────────
    defaults = {
        "path_longitudinal_controller_type": cfg.get(
            "path_longitudinal_controller_type", "pid"
        ),
        "path_lateral_controller_type": cfg.get(
            "path_lateral_controller_type", "pp_map"
        ),
        "leader_longitudinal_controller_type": cfg.get(
            "leader_longitudinal_controller_type", "cacc"
        ),
        "leader_lateral_controller_type": cfg.get(
            "leader_lateral_controller_type", "pure_pursuit"
        ),
    }

    # ── Discover available controllers from top-level dict keys ──
    # Any top-level key that is a dict and matches a known controller name
    # is considered an available controller with its params.
    controller_params: Dict[str, Dict[str, Any]] = {}
    longitudinal_set: set = set()
    lateral_set: set = set()

    for key, value in cfg.items():
        if not isinstance(value, dict):
            continue
        if key in _KNOWN_LONGITUDINAL:
            longitudinal_set.add(key)
            controller_params[key] = _flatten_params(value)
        elif key in _KNOWN_LATERAL:
            lateral_set.add(key)
            controller_params[key] = _flatten_params(value)

    # Also include any types referenced in defaults but not yet found
    for default_key in [
        "path_longitudinal_controller_type",
        "leader_longitudinal_controller_type",
    ]:
        t = defaults.get(default_key, "")
        if t:
            longitudinal_set.add(t)
    for default_key in [
        "path_lateral_controller_type",
        "leader_lateral_controller_type",
    ]:
        t = defaults.get(default_key, "")
        if t:
            lateral_set.add(t)

    # Build ordered lists (default first, then alphabetical)
    def _ordered(items: set, default: str) -> list:
        result = []
        if default in items:
            result.append(default)
        for i in sorted(items):
            if i not in result:
                result.append(i)
        return result

    path_long = _ordered(
        longitudinal_set, defaults["path_longitudinal_controller_type"]
    )
    path_lat = _ordered(lateral_set, defaults["path_lateral_controller_type"])
    leader_long = _ordered(
        longitudinal_set, defaults["leader_longitudinal_controller_type"]
    )
    leader_lat = _ordered(lateral_set, defaults["leader_lateral_controller_type"])

    return {
        "defaults": defaults,
        "path_longitudinal_controllers": path_long,
        "path_lateral_controllers": path_lat,
        "leader_longitudinal_controllers": leader_long,
        "leader_lateral_controllers": leader_lat,
        "controller_params": controller_params,
    }


# ── Public API ──────────────────────────────────────────────────────


def get_available_config(controller_mode: str = "sim") -> Dict[str, Any]:
    """Load all configs and return a single structured dict.

    Args:
        controller_mode: 'sim' or 'real' to select controller config variant.

    Returns:
        Dict with all observer/controller types, defaults, and parameters.
    """
    local_obs = load_local_observer_config()
    fleet_obs = load_fleet_observer_config()
    ctrl = load_controller_config(mode=controller_mode)

    return {
        # Observer lists
        "local_observers": local_obs["available"],
        "fleet_observers": fleet_obs["available"],
        # Controller lists (per state context)
        "path_longitudinal_controllers": ctrl["path_longitudinal_controllers"],
        "path_lateral_controllers": ctrl["path_lateral_controllers"],
        "leader_longitudinal_controllers": ctrl["leader_longitudinal_controllers"],
        "leader_lateral_controllers": ctrl["leader_lateral_controllers"],
        # Defaults
        "defaults": {
            "local_estimator_type": local_obs["default"],
            "fleet_estimator_type": fleet_obs["default"],
            **ctrl["defaults"],
        },
        # Parameters
        "controller_params": ctrl["controller_params"],
        "observer_params": {
            **local_obs["params"],
            **fleet_obs["params"],
        },
    }
