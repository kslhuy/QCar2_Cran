"""Shared runtime attack controls for EKF and Robust KalmanNet.

Imports the attack simulator only when enabled, so ordinary EKF use does not
require the neural estimator or load its checkpoint.
"""
import copy
from typing import Any, Dict, List, Optional


class RuntimeSensorAttackMixin:
    DEFAULT_BRANCH_ATTACK_TYPES = ("bias", "scale", "freeze", "noise", "ramp", "zero_out")
    DEFAULT_GPS_ATTACK_TYPES = ("noise", "freeze", "jump", "dropout", "reacquisition")

    def _initialize_sensor_attacks(self, config):
        self.sensor_failure_simulator = None
        self.last_sensor_failure_metadata = None
        fault_config = config.get("sensor_failure_simulation", config.get("fault_simulation", {}))
        self.sensor_failure_simulation_cfg = self._normalize_runtime_attack_config(fault_config)
        if self.sensor_failure_simulation_cfg.get("enabled", False):
            self.start_sensor_attack(self.sensor_failure_simulation_cfg)

    def _log_sensor_attack(self, message, error=None):
        logger = getattr(self, "logger", None)
        if error is not None and hasattr(logger, "log_error"):
            logger.log_error(message, error)
        elif hasattr(logger, "logger"):
            logger.logger.info(message)

    @classmethod
    def _normalize_runtime_attack_config(
        cls, config: Optional[Dict[str, Any]]
    ) -> Dict[str, Any]:
        normalized = copy.deepcopy(config) if isinstance(config, dict) else {}
        target_sensor = str(normalized.pop("target_sensor", "") or "").strip().lower()

        attack_types = normalized.pop("attack_types", None)
        if attack_types is not None and "enabled_attacks" not in normalized:
            normalized["enabled_attacks"] = attack_types

        enabled_attacks = normalized.get("enabled_attacks")
        if isinstance(enabled_attacks, str):
            normalized["enabled_attacks"] = [enabled_attacks]
        elif enabled_attacks is None:
            normalized["enabled_attacks"] = list(cls.DEFAULT_BRANCH_ATTACK_TYPES)
        else:
            normalized["enabled_attacks"] = list(enabled_attacks)

        gps_attack_types = normalized.get("gps_attack_types")
        if isinstance(gps_attack_types, str):
            normalized["gps_attack_types"] = [gps_attack_types]
        elif gps_attack_types is None:
            normalized["gps_attack_types"] = list(cls.DEFAULT_GPS_ATTACK_TYPES)
        else:
            normalized["gps_attack_types"] = list(gps_attack_types)

        gps_enabled = normalized.pop("gps_enabled", None)
        if gps_enabled is False:
            normalized["gps_attack_prob"] = 0.0

        if "forced_branches" in normalized:
            forced_branches = normalized.get("forced_branches")
            if isinstance(forced_branches, str):
                normalized["forced_branches"] = [forced_branches]
            else:
                normalized["forced_branches"] = list(forced_branches or [])

        branch_alias_map = {
            "imu": "imu",
            "steering": "steer",
            "steer": "steer",
            "velocity": "wheel",
            "wheel": "wheel",
        }
        if target_sensor == "gps":
            normalized["forced_branches"] = []
            normalized["force_gps_attack"] = True
            normalized["attack_prob"] = 0.0
            normalized["gps_attack_prob"] = 1.0
            normalized["immediate_attack"] = True
        elif target_sensor in branch_alias_map:
            normalized["forced_branches"] = [branch_alias_map[target_sensor]]
            normalized["force_gps_attack"] = False
            normalized["attack_prob"] = 1.0
            normalized["gps_attack_prob"] = 0.0
            normalized["immediate_attack"] = True
            normalized["max_branches_attacked"] = 1
        elif target_sensor in {"", "random"}:
            normalized.setdefault("force_gps_attack", False)
            normalized.setdefault("immediate_attack", True)
        else:
            raise ValueError(
                "Unsupported target_sensor "
                f"'{target_sensor}'. Expected one of: "
                "['random', 'imu', 'steering', 'velocity', 'gps']"
            )

        return normalized

    def start_sensor_attack(self, config: Optional[Dict[str, Any]] = None) -> bool:
        try:
            try:
                from .sensor_attack_augmentation import RuntimeAttackConfig, RuntimeSensorAttackSimulator
            except ImportError:
                from sensor_attack_augmentation import RuntimeAttackConfig, RuntimeSensorAttackSimulator

            normalized_config = self._normalize_runtime_attack_config(config)
            normalized_config["enabled"] = True
            runtime_attack_cfg = RuntimeAttackConfig.from_dict(normalized_config)
            self.sensor_failure_simulator = RuntimeSensorAttackSimulator(
                runtime_attack_cfg
            )
            self.sensor_failure_simulation_cfg = copy.deepcopy(normalized_config)
            self.last_sensor_failure_metadata = None
            self._log_sensor_attack(
                "Local sensor attack enabled "
                f"(branch_prob={runtime_attack_cfg.attack_prob}, "
                f"gps_prob={runtime_attack_cfg.gps_attack_prob}, "
                f"duration={runtime_attack_cfg.min_attack_steps}-"
                f"{runtime_attack_cfg.max_attack_steps} steps)"
            )
            return True
        except Exception as exc:
            self._log_sensor_attack("Failed to start local sensor attack", exc)
            return False

    def stop_sensor_attack(self) -> bool:
        self.sensor_failure_simulator = None
        self.last_sensor_failure_metadata = None
        if isinstance(self.sensor_failure_simulation_cfg, dict):
            self.sensor_failure_simulation_cfg["enabled"] = False
        self._log_sensor_attack("Local sensor attack disabled")
        return True

    def get_sensor_attack_status(self) -> Dict[str, Any]:
        metadata = (
            copy.deepcopy(self.last_sensor_failure_metadata)
            if isinstance(self.last_sensor_failure_metadata, dict)
            else {}
        )
        branch_attacks = metadata.get("branch_attacks", []) or []
        gps_attack = metadata.get("gps_attack")
        remaining_steps: List[int] = []
        branch_types: List[str] = []

        for item in branch_attacks:
            branch_name = str(item.get("branch", "")).strip()
            attack_type = str(item.get("attack_type", "")).strip()
            if branch_name and attack_type:
                branch_types.append(f"{branch_name}:{attack_type}")
            try:
                remaining_steps.append(int(item.get("remaining_steps", 0)))
            except (TypeError, ValueError):
                pass

        gps_type = ""
        if isinstance(gps_attack, dict):
            gps_type = str(gps_attack.get("attack_type", "")).strip()
            try:
                remaining_steps.append(int(gps_attack.get("remaining_steps", 0)))
            except (TypeError, ValueError):
                pass

        current_intensity = metadata.get("current_intensity", {}) or {}
        return {
            "local_sensor_attack_supported": True,
            "local_sensor_attack_enabled": bool(
                self.sensor_failure_simulator is not None
            ),
            "local_sensor_attack_active": bool(metadata.get("active", False)),
            "local_sensor_attack_branch_types": "|".join(branch_types),
            "local_sensor_attack_gps_type": gps_type,
            "local_sensor_attack_remaining_steps": max(remaining_steps)
            if remaining_steps
            else 0,
            "local_sensor_attack_intensity": float(
                current_intensity.get("overall", 0.0)
            ),
        }

