from pathlib import Path
from typing import Dict, Any
from omegaconf import OmegaConf

class SimulationConfig:
    """Configuration loader for vehicle simulation."""
    
    @staticmethod
    def load_config(config_path: str | Path | None = None) -> Dict[str, Any]:
        """Load packaged defaults, or an explicit path relative to the caller.

        Omitting the argument always selects this package's parameters.yaml.
        An explicit filename is never silently replaced by a packaged file.
        """
        path = (Path(__file__).resolve().parent / "parameters.yaml"
                if config_path is None else Path(config_path).expanduser())
        path = path.resolve(strict=True)
        with path.open(encoding="utf-8-sig") as handle:
            conf = OmegaConf.load(handle)
        config = OmegaConf.to_object(conf)
        if not isinstance(config, dict):
            raise ValueError(f"Simulation configuration must be a mapping: {path}")
        return config

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
