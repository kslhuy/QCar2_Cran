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
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize configuration loader
        
        Args:
            config_path: Path to YAML config file. If None, uses default location.
        """
        if config_path is None:
            # Default to controller_config.yaml in the same directory
            config_dir = os.path.dirname(os.path.abspath(__file__))
            if IS_PHYSICAL_QCAR:
                config_path = os.path.join(config_dir, 'config_controller_real.yaml')
            else:
                config_path = os.path.join(config_dir, 'config_controller_sim.yaml')
        
        self.config_path = config_path
        self.config = self._load_config()
    
    def _load_config(self) -> Dict[str, Any]:
        """Load YAML configuration file"""
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(
                f"Controller config file not found: {self.config_path}\n"
                f"Please create a config_controller.yaml file or use config_controller_sim.yaml as template"
            )
        
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        return config
    
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
        if 'sa_acc' in self.config:
            types.append('sa_acc')
        return types

    def get_available_lateral_types(self) -> list:
        """Get list of available lateral controller types based on config"""
        types = ['path'] # 'path' is always available as a mode
        if 'pp_map' in self.config:
            types.append('pp_map')
        if 'pure_pursuit' in self.config:
            types.append('pure_pursuit')
        if 'stanley' in self.config:
            types.append('stanley')
        if 'lookahead' in self.config:
            types.append('lookahead')
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

        elif controller_type == 'sa_acc':
            return self._get_sa_acc_params()
        else:
            raise ValueError(f"Unknown longitudinal controller type: {controller_type}")
    
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
        elif controller_type == 'pp_map':
            return self._get_pp_map_params()
        elif controller_type == 'stanley':
            return self._get_stanley_params()
        elif controller_type == 'lookahead':
            return self._get_lookahead_params()
        elif controller_type == 'hybrid':
            return self._get_hybrid_lateral_params()
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

    def _get_sa_acc_params(self) -> Dict[str, Any]:
        """Get SA-ACC controller parameters"""
        sa_acc_config = self.config.get('sa_acc', {})
        
        return {
            'tau': sa_acc_config.get('tau', 0.4),
            'h': sa_acc_config.get('h', 0.5),
            'k1': sa_acc_config.get('k1', -0.8),
            'k2': sa_acc_config.get('k2', 2.5),
            'li': sa_acc_config.get('li', 5.0),
            'Li': sa_acc_config.get('Li', 8.0),
            'acc_to_throttle_gain': sa_acc_config.get('acc_to_throttle_gain', 0.65),
            'max_throttle': sa_acc_config.get('max_throttle', 0.3),
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
            'lookahead_offset': stanley_config.get('lookahead_offset', 0.2),
            'k_e': stanley_config.get('k_e', 0.5),
            'k_soft': stanley_config.get('k_soft', 1.0),
            'max_steering': stanley_config.get('max_steering', 0.5),
        }

    def _get_pp_map_params(self) -> Dict[str, Any]:
        """Get map-based PP_Controller parameters."""
        pp_map_config = self.config.get('pp_map', {})

        return {
            'sample_ds': pp_map_config.get('sample_ds', 0.02),
            'desired_speed': pp_map_config.get('desired_speed', 0.7),
            'min_speed': pp_map_config.get('min_speed', 0.15),
            'kappa_speed_gain': pp_map_config.get('kappa_speed_gain', 2.0),
            'max_speed': pp_map_config.get('max_speed', 0.8),
            'hard_turn_kappa': pp_map_config.get('hard_turn_kappa', 0.85),
            'hard_turn_speed': pp_map_config.get('hard_turn_speed', 0.32),
            't_clip_min': pp_map_config.get('t_clip_min', 0.4),
            't_clip_max': pp_map_config.get('t_clip_max', 1.8),
            'm_l1': pp_map_config.get('m_l1', 0.35),
            'q_l1': pp_map_config.get('q_l1', 0.15),
            'speed_lookahead': pp_map_config.get('speed_lookahead', 0.15),
            'lat_err_coeff': pp_map_config.get('lat_err_coeff', 0.8),
            'acc_scaler_for_steer': pp_map_config.get('acc_scaler_for_steer', 1.0),
            'dec_scaler_for_steer': pp_map_config.get('dec_scaler_for_steer', 1.0),
            'start_scale_speed': pp_map_config.get('start_scale_speed', 0.2),
            'end_scale_speed': pp_map_config.get('end_scale_speed', 1.0),
            'downscale_factor': pp_map_config.get('downscale_factor', 0.35),
            'speed_lookahead_for_steer': pp_map_config.get('speed_lookahead_for_steer', 0.1),
            'prioritize_dyn': pp_map_config.get('prioritize_dyn', False),
            'trailing_gap': pp_map_config.get('trailing_gap', 0.8),
            'trailing_p_gain': pp_map_config.get('trailing_p_gain', 0.6),
            'trailing_i_gain': pp_map_config.get('trailing_i_gain', 0.0),
            'trailing_d_gain': pp_map_config.get('trailing_d_gain', 0.1),
            'blind_trailing_speed': pp_map_config.get('blind_trailing_speed', 0.2),
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


# Singleton instance for easy access
_default_config = None

def get_controller_config(config_path: Optional[str] = None) -> ControllerConfig:
    """
    Get controller configuration (singleton pattern)
    
    Args:
        config_path: Path to config file. If None, uses default.
        
    Returns:
        ControllerConfig instance
    """
    global _default_config
    
    if _default_config is None or config_path is not None:
        _default_config = ControllerConfig(config_path)
    
    return _default_config
