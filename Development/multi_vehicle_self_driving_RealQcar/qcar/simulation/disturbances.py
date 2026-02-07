import numpy as np
import time
from typing import Dict, List, Optional

class DisturbanceGenerator:
    """Generates time-varying disturbances for simulation testing."""
    def __init__(self, config: Optional[Dict] = None):
        self.active_disturbances = {
            'vx': {'type': 'none', 'value': 0.0},
            'vy': {'type': 'none', 'value': 0.0},
            'r': {'type': 'none', 'value': 0.0}
        }
        if config:
            self._configure_from_dict(config)
    
    def _configure_from_dict(self, config: Dict):
        """Configure disturbances from a dictionary."""
        for channel in ['vx', 'vy', 'r']:
            if channel in config:
                cfg = config[channel]
                self.set_disturbance(channel, cfg.get('type', 'none'), 
                                     cfg.get('value', 0.0), 
                                     cfg.get('freq', 1.0))

    def set_disturbance(self, channel: str, type: str, value: float = 0.0, freq: float = 1.0):
        """
        Set disturbance for a channel ('vx', 'vy', 'r').
        Types: 'none', 'constant', 'sine', 'step', 'random'
        """
        if channel in self.active_disturbances:
            self.active_disturbances[channel] = {
                'type': type,
                'value': value,
                'freq': freq,
                'start_time': time.time()
            }

    def get_disturbance(self, t: float) -> List[float]:
        """Get [d_vx, d_vy, d_r] vector for current time."""
        d = []
        for channel in ['vx', 'vy', 'r']:
            cfg = self.active_disturbances[channel]
            val = 0.0
            if cfg['type'] == 'constant':
                val = cfg['value']
            elif cfg['type'] == 'sine':
                # Sine wave: value * sin(2*pi*freq*t)
                val = cfg['value'] * np.sin(2 * np.pi * cfg.get('freq', 1.0) * t)
            elif cfg['type'] == 'step':
                # Step after 1 second of activation
                if t - cfg.get('start_time', 0) > 1.0:
                    val = cfg['value']
            elif cfg['type'] == 'random':
                # Random noise uniform [-value, value]
                val = np.random.uniform(-cfg['value'], cfg['value'])
                
            d.append(val)
        return d
