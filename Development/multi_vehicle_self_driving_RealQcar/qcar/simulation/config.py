import yaml
from pathlib import Path
from typing import Dict, Any, Optional
from omegaconf import OmegaConf

class SimulationConfig:
    """Configuration loader for vehicle simulation."""
    
    @staticmethod
    def load_config(config_path: str = "parameters.yaml") -> Dict[str, Any]:
        """
        Load simulation parameters from a YAML file.
        
        Args:
            config_path: Path to the YAML config file. 
                         If relative, searches in current directory or package dir.
        
        Returns:
            Dictionary (or OmegaConf object) with configuration.
        """
        path = Path(config_path)
        
        # If not absolute, check relative to this file
        if not path.is_absolute():
            local_path = Path(__file__).parent / config_path
            if local_path.exists():
                path = local_path
        
        if not path.exists():
            raise FileNotFoundError(f"Configuration file not found: {path}")
            
        try:
            # Use OmegaConf for robust loading
            conf = OmegaConf.load(str(path))
            return OmegaConf.to_object(conf) # Return as standard dict
        except Exception as e:
            print(f"Error loading config with OmegaConf: {e}, falling back to yaml")
            with open(path, 'r') as f:
                return yaml.safe_load(f)

    @staticmethod
    def get_default_config() -> Dict[str, Any]:
        """Return a default configuration structure."""
        return {
            'vehicle': {
                'id': 0,
                'model_type': 'kinematic',
                'longitudinal_model': 'default',
                'steering_model': 'default',
                'params_file': 'qcar',
                'dt': 0.05
            },
            'disturbances': {
                'vx': {'type': 'none', 'value': 0.0},
                'vy': {'type': 'none', 'value': 0.0},
                'r': {'type': 'none', 'value': 0.0}
            },
            'sensors': {
                'gps': {'update_rate': 10.0, 'noise_pos': 0.0, 'noise_theta': 0.0}
            },
            'electronics': {
                'enabled': False,
                'seed': 1000,
                'hardware_target': {
                    'profile': 'cran_segula_stm32'
                },
                'sensor_compute_protocol': 'sensor_frame_v2',
                'sensor_compute_interface': 'spi',
                'vehicle_interface': 'uart',
                'fusion': {
                    'mode': 'auxiliary',
                    'electronics_weight': 0.5
                },
                'v2v': {
                    'enabled': True,
                    'mode': 'firmware',
                    'bitrate_bps': 6000000.0,
                    'max_packet_size': 1500,
                    'faults': {
                        'enabled': True,
                        'fixed_delay_s': 0.0,
                        'jitter_s': 0.0,
                        'drop_probability': 0.0,
                        'bit_error_rate': 0.0
                    }
                }
            }
        }
