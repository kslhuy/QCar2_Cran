"""
Controller Configuration Loader

Loads controller parameters from YAML configuration files.
Provides a centralized way to manage all controller settings.
"""
import yaml
import os
import numpy as np
from typing import Dict, Any, Optional
from pal.products.qcar import  IS_PHYSICAL_QCAR


class ControllerConfig:
    """Loads and manages controller configuration from YAML file"""
    
    def __init__(self, config_path: Optional[str] = None, vehicle_id: Optional[int] = None):
        """
        Initialize configuration loader
        
        Args:
            config_path: Path to YAML config file. If None, uses default location.
            vehicle_id: ID of the vehicle for per-vehicle config override
        """
        self.vehicle_id = vehicle_id
        
        if config_path is None:
            # Default to controller_config.yaml in the same directory
            config_dir = os.path.dirname(os.path.abspath(__file__))
            if IS_PHYSICAL_QCAR:
                config_path = os.path.join(config_dir, 'config_controller_real.yaml')
            else:
                config_path = os.path.join(config_dir, 'config_controller_sim.yaml')
        
        self.config_path = config_path
        self.config = self._load_config()
        
        # Apply per-vehicle overrides if vehicle_id is provided
        if vehicle_id is not None:
            self._apply_per_vehicle_config(vehicle_id)
    
    def _load_config(self) -> Dict[str, Any]:
        """Load YAML configuration file"""
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(
                f"Controller config file not found: {self.config_path}\n"
                f"Please create a config_controller.yaml file or use config_controller_sim.yaml as template"
            )
        
        with open(self.config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        return config
    
    def _apply_per_vehicle_config(self, vehicle_id: int):
        """
        Apply per-vehicle configuration overrides
        
        Args:
            vehicle_id: ID of the vehicle to apply config for
        """
        per_vehicle = self.config.get('per_vehicle_controllers', {})
        vehicle_config = per_vehicle.get(vehicle_id, {})
        
        if not vehicle_config:
            # No per-vehicle config for this vehicle
            return
        
        # Override controller types if specified
        if 'longitudinal_controller_type' in vehicle_config:
            self.config['longitudinal_controller_type'] = vehicle_config['longitudinal_controller_type']
        
        if 'lateral_controller_type' in vehicle_config:
            self.config['lateral_controller_type'] = vehicle_config['lateral_controller_type']
        
        # Merge controller-specific parameters
        for controller_type in ['cacc', 'pid', 'hybrid_longitudinal', 'fix', 'throttle_sequence', 'velocity_sequence',
                               'state_feedback', 'state_feedback_no_observer', 'classical_distributed',
                               'pure_pursuit', 'stanley', 'stanley_y_hold', 'lookahead', 'hybrid_lateral', 
                               'fusion_lateral', 'fix_lateral']:
            if controller_type in vehicle_config:
                if controller_type not in self.config:
                    # For throttle_sequence (list type), use direct assignment
                    # For others (dict type), initialize as empty dict
                    if isinstance(vehicle_config[controller_type], list):
                        self.config[controller_type] = vehicle_config[controller_type]
                    else:
                        self.config[controller_type] = {}
                
                # Merge or assign
                if isinstance(self.config[controller_type], dict) and isinstance(vehicle_config[controller_type], dict):
                    # Dict-to-dict: use update for merging
                    self.config[controller_type].update(vehicle_config[controller_type])
                else:
                    # Other types (like lists): direct assignment
                    self.config[controller_type] = vehicle_config[controller_type]
    
    def get_longitudinal_controller_type(self) -> str:
        """Get the selected longitudinal controller type"""
        return self.config.get('longitudinal_controller_type', 'cacc')
    
    def get_lateral_controller_type(self) -> str:
        """Get the selected lateral controller type"""
        return self.config.get('lateral_controller_type', 'pure_pursuit')
    
    def get_available_longitudinal_types(self) -> list:
        """Get list of available longitudinal controller types based on config"""
        types = []
        if 'cacc' in self.config:
            types.append('cacc')
        if 'pid' in self.config:
            types.append('pid')
        if 'hybrid_longitudinal' in self.config:
            types.append('hybrid')
        if 'fix' in self.config:
            types.append('fix')
        if 'state_feedback' in self.config:
            types.append('state_feedback')
        if 'state_feedback_no_observer' in self.config:
            types.append('state_feedback_no_observer')
        if 'throttle_sequence' in self.config:
            types.append('throttle_sequence')
        if 'classical_distributed' in self.config:
            types.append('classical_distributed')
        return types

    def get_available_lateral_types(self) -> list:
        """Get list of available lateral controller types based on config"""
        types = ['path'] # 'path' is always available as a mode
        if 'pure_pursuit' in self.config:
            types.append('pure_pursuit')
        if 'stanley' in self.config:
            types.append('stanley')
        if 'stanley_y_hold' in self.config:
            types.append('stanley_y_hold')
        if 'lookahead' in self.config:
            types.append('lookahead')
        if 'hybrid_lateral' in self.config:
            types.append('hybrid')
        if 'fusion_lateral' in self.config:
            types.append('fusion')
        return types

    def get_longitudinal_params(self, controller_type: Optional[str] = None) -> Dict[str, Any]:
        """
        Get parameters for longitudinal controller
        
        Args:
            controller_type: Type of controller. If None, uses configured type.
            
        Returns:
            Dictionary of parameters ready to pass to controller factory
        """
        if controller_type is None:
            controller_type = self.get_longitudinal_controller_type()

        if controller_type == 'cacc':
            return self._get_cacc_params()
        elif controller_type == 'pid':
            return self._get_pid_params()
        elif controller_type == 'hybrid':
            return self._get_hybrid_longitudinal_params()
        elif controller_type == 'fix':
            return self._get_fix_params()
        elif controller_type == 'state_feedback':
            return self._get_state_feedback_params()
        elif controller_type == 'state_feedback_no_observer':
            return self._get_state_feedback_no_observer_params()
        elif controller_type == 'throttle_sequence':
            return self._get_throttle_sequence_params()
        elif controller_type == 'classical_distributed':
            return self._get_classical_distributed_params()
        else:
            raise ValueError(f"Unknown longitudinal controller type: {controller_type}")

    def _get_classical_distributed_params(self) -> Dict[str, Any]:
        """Get classical distributed controller parameters"""
        cd_config = self.config.get('classical_distributed', {})
        # K_all_vehicles should be a dict of dicts, or fallback to None
        return {
            'max_throttle': cd_config.get('max_throttle', 0.3),
            'throttle_smoothing': cd_config.get('throttle_smoothing', 0.7),
            'leader_fix_throttle': cd_config.get('leader_fix_throttle', 0.1),
            'K_all_vehicles': cd_config.get('K_all_vehicles', None),
        }
    
    def get_lateral_params(self, controller_type: Optional[str] = None) -> Dict[str, Any]:
        """
        Get parameters for lateral controller
        
        Args:
            controller_type: Type of controller. If None, uses configured type.
            
        Returns:
            Dictionary of parameters ready to pass to controller factory
        """
        if controller_type is None:
            controller_type = self.get_lateral_controller_type()
        
        if controller_type == 'pure_pursuit':
            return self._get_pure_pursuit_params()
        elif controller_type == 'stanley':
            return self._get_stanley_params()
        elif controller_type == 'stanley_y_hold':
            return self._get_stanley_y_hold_params()
        elif controller_type == 'lookahead':
            return self._get_lookahead_params()
        elif controller_type == 'hybrid':
            return self._get_hybrid_lateral_params()
        elif controller_type == 'fix_lateral':
            return self._get_fix_lateral_params()
        else:
            raise ValueError(f"Unknown lateral controller type: {controller_type}")
    
    # ========================================================================
    # Longitudinal Controller Parameter Getters
    # ========================================================================
    
    def _get_cacc_params(self) -> Dict[str, Any]:
        """Get CACC controller parameters"""
        cacc_config = self.config.get('cacc', {})
        
        # Build K matrix from individual gains
        K_spacing = cacc_config.get('K_spacing', 0.2)
        K_velocity = cacc_config.get('K_velocity', 0.05)
        
        return {
            's0': cacc_config.get('s0', 1.5),
            'h': cacc_config.get('h', 0.5),
            'K': np.array([[K_spacing, K_velocity]]),
            'acc_to_throttle_gain': cacc_config.get('acc_to_throttle_gain', 0.5),
            'max_throttle': cacc_config.get('max_throttle', 0.3),
            'alpha_filter': cacc_config.get('alpha_filter', 0.3),
            'ki_velocity': cacc_config.get('ki_velocity', 0.1),
        }
    
    def _get_pid_params(self) -> Dict[str, Any]:
        """Get PID controller parameters"""
        pid_config = self.config.get('pid', {})
        
        return {
            'kp': pid_config.get('kp', 0.1),
            'ki': pid_config.get('ki', 1.0),
            'kd': pid_config.get('kd', 0.01),
            'max_throttle': pid_config.get('max_throttle', 0.3),
            'ei_max': pid_config.get('ei_max', 1.0),
            'v_ref': pid_config.get('v_ref', 0.75),
        }
    
    def _get_hybrid_longitudinal_params(self) -> Dict[str, Any]:
        """Get hybrid longitudinal controller parameters"""
        hybrid_config = self.config.get('hybrid_longitudinal', {})
        
        # Get sub-controller params
        cacc_params = self._get_cacc_params() if hybrid_config.get('use_cacc_params', True) else {}
        pid_params = self._get_pid_params() if hybrid_config.get('use_pid_params', True) else {}
        
        return {
            'cacc_params': cacc_params,
            'pid_params': pid_params,
        }
    
    def _get_fix_params(self) -> Dict[str, Any]:
        """Get fix constant throttle controller parameters"""
        fix_config = self.config.get('fix', {})
        
        return {
            'throttle': fix_config.get('throttle', 0.1),
        }
    
    def _get_throttle_sequence_params(self) -> Dict[str, Any]:
        """Get throttle sequence controller parameters
        
        Handles two config formats:
        1. List format (new): Each item has throttle and duration
           - {throttle: 0.05, duration: 5}, {throttle: 0.06, duration: 5}
        2. Dict format (legacy): Single values and duration dict
           - {'values': [0.05, 0.06], 'duration': 5}
        """
        ts_config = self.config.get('throttle_sequence', {})
        
        # Handle list format from config file
        if isinstance(ts_config, list) and len(ts_config) > 0:
            # Extract throttle values and durations from list
            if isinstance(ts_config[0], dict) and 'throttle' in ts_config[0]:
                throttle_values = [item.get('throttle', 0.1) for item in ts_config]
                # Use duration from first item (can vary per item in future)
                duration = ts_config[0].get('duration', 5.0)
            else:
                # Fallback if list format is unexpected
                throttle_values = [0.5]
                duration = 5.0
        else:
            # Handle dict format
            throttle_values = ts_config.get('values', [0.5])
            duration = ts_config.get('duration', 5.0)
        
        return {
            'throttle_sequence': {
                'values': throttle_values,
                'duration': duration,
            }
        }
    
    def _get_state_feedback_params(self) -> Dict[str, Any]:
        """Get state feedback controller parameters"""
        sf_config = self.config.get('state_feedback', {})
        
        return {
            'max_throttle': sf_config.get('max_throttle', 0.3),
            'throttle_smoothing': sf_config.get('throttle_smoothing', 0.7),
            'leader_fix_throttle': sf_config.get('leader_fix_throttle', 0.1),
            'observer': None,  # Will be set by controller manager
        }
    
    def _get_state_feedback_no_observer_params(self) -> Dict[str, Any]:
        """
        Get state feedback controller (no observer) parameters
        
        This controller uses true vehicle states from V2V communication
        instead of observer estimates to compute control commands.
        """
        sf_config = self.config.get('state_feedback_no_observer', {})
        
        # If no specific config for no_observer variant, fall back to state_feedback config
        if not sf_config:
            sf_config = self.config.get('state_feedback', {})
        
        return {
            'max_throttle': sf_config.get('max_throttle', 0.3),
            'throttle_smoothing': sf_config.get('throttle_smoothing', 0.7),
            'leader_fix_throttle': sf_config.get('leader_fix_throttle', 0.1),
            'observer': None,  # Will be set by controller manager (used for V2V communication access)
        }
    
    # ========================================================================
    # Lateral Controller Parameter Getters
    # ========================================================================
    
    def _get_pure_pursuit_params(self) -> Dict[str, Any]:
        """Get Pure Pursuit controller parameters"""
        pp_config = self.config.get('pure_pursuit', {})
        
        return {
            'lookahead_distance': pp_config.get('lookahead_distance', 1.0),
            'k_steering': pp_config.get('k_steering', 1.0),
            'max_steering': pp_config.get('max_steering', 0.55),
            'adaptive_lookahead': pp_config.get('adaptive_lookahead', True),
        }
    
    def _get_stanley_params(self) -> Dict[str, Any]:
        """Get Stanley controller parameters"""
        stanley_config = self.config.get('stanley', {})
        
        return {
            'k_e': stanley_config.get('k_e', 0.5),
            'k_soft': stanley_config.get('k_soft', 1.0),
            'max_steering': stanley_config.get('max_steering', 0.55),
        }

    def _get_stanley_y_hold_params(self) -> Dict[str, Any]:
        """Get Stanley Y-hold controller parameters"""
        syh_config = self.config.get('stanley_y_hold', {})

        return {
            'y_ref': syh_config.get('y_ref', -0.62),
            'heading_ref': syh_config.get('heading_ref', 0.0),
            'k_e': syh_config.get('k_e', 0.6),
            'k_soft': syh_config.get('k_soft', 0.5),
            'k_heading': syh_config.get('k_heading', 2.2),
            'use_leader_heading': syh_config.get('use_leader_heading', False),
            'max_steering': syh_config.get('max_steering', 0.55),
        }
    
    def _get_lookahead_params(self) -> Dict[str, Any]:
        """Get Lookahead controller parameters"""
        lookahead_config = self.config.get('lookahead', {})
        
        return {
            'ri': lookahead_config.get('ri', 1.0),
            'hi': lookahead_config.get('hi', 0.3),
            'l_r': lookahead_config.get('l_r', 0.141),
            'l_f': lookahead_config.get('l_f', 0.115),
            'k1': lookahead_config.get('k1', 1.0),
            'k2': lookahead_config.get('k2', 1.0),
            'max_steering': lookahead_config.get('max_steering', 0.55),
        }
    
    def _get_hybrid_lateral_params(self) -> Dict[str, Any]:
        """Get hybrid lateral controller parameters - returns configured controllers"""
        hybrid_config = self.config.get('hybrid_lateral', {})
        
        # Import here to avoid circular dependency
        from Controller.lateral_controllers import LateralControllerFactory
        
        # Get primary and secondary controller types
        primary_type = hybrid_config.get('primary_controller', 'pure_pursuit')
        secondary_type = hybrid_config.get('secondary_controller', 'stanley')
        
        # Get their parameters
        primary_params = self.get_lateral_params(primary_type)
        secondary_params = self.get_lateral_params(secondary_type)
        
        # Create the controllers (factory will be called with these as sub-controllers)
        return {
            'primary_controller': LateralControllerFactory.create(
                primary_type,
                primary_params
            ),
            'secondary_controller': LateralControllerFactory.create(
                secondary_type,
                secondary_params
            ),
            'switch_distance': hybrid_config.get('switch_distance', 1.5),
        }
    
    def _get_fix_lateral_params(self) -> Dict[str, Any]:
        """Get fix constant steering controller parameters"""
        fix_config = self.config.get('fix_lateral', {})
        
        return {
            'steering': fix_config.get('steering', 0.0),
        }
    
    def get_vehicle_params(self) -> Dict[str, Any]:
        """Get vehicle physical parameters"""
        vehicle_config = self.config.get('vehicle', {})
        
        return {
            'wheelbase': vehicle_config.get('wheelbase', 0.256),
            'l_r': vehicle_config.get('l_r', 0.141),
            'l_f': vehicle_config.get('l_f', 0.115),
        }
    
    def get_enable_steering_control(self) -> bool:
        """Get enable_steering_control flag"""
        return self.config.get('enable_steering_control', True)

    def get_velocity_sequence(self) -> list:
        """Get velocity sequence config for FOLLOWING_PATH speed scheduling.

        Supported formats:
        1. List format (recommended):
           - {velocity: 0.45, duration: 40}
           - {velocity: 1.0, duration: 999}
        2. Dict legacy format:
           {'values': [0.45, 1.0], 'duration': 40}
        """
        vs_config = self.config.get('velocity_sequence', [])
        if isinstance(vs_config, list):
            return vs_config

        if isinstance(vs_config, dict):
            values = vs_config.get('values', [])
            duration = float(vs_config.get('duration', 0.0))
            result = []
            for v in values:
                result.append({'velocity': float(v), 'duration': duration})
            return result

        return []
    
    @property
    def enable_steering_control(self) -> bool:
        """Property accessor for enable_steering_control"""
        return self.get_enable_steering_control()
    
    def reload(self):
        """Reload configuration from file"""
        self.config = self._load_config()
    
    def __repr__(self):
        return (
            f"ControllerConfig(\n"
            f"  longitudinal: {self.get_longitudinal_controller_type()}\n"
            f"  lateral: {self.get_lateral_controller_type()}\n"
            f"  config_path: {self.config_path}\n"
            f")"
        )


# Singleton instance for easy access (deprecated - use per-vehicle config instead)
_default_config = None

def get_controller_config(config_path: Optional[str] = None, vehicle_id: Optional[int] = None) -> ControllerConfig:
    """
    Get controller configuration
    
    Args:
        config_path: Path to config file. If None, uses default.
        vehicle_id: Vehicle ID for per-vehicle config override
        
    Returns:
        ControllerConfig instance
        
    Note:
        If vehicle_id is provided, returns a new instance with vehicle-specific config.
        Otherwise uses singleton pattern for backward compatibility.
    """
    global _default_config
    
    # If vehicle_id specified, always create new instance with per-vehicle config
    if vehicle_id is not None:
        return ControllerConfig(config_path, vehicle_id)
    
    # Otherwise use singleton for backward compatibility
    if _default_config is None or config_path is not None:
        _default_config = ControllerConfig(config_path)
    
    return _default_config
    

