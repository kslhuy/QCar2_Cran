"""
Configuration Loader for Neural Observer

Provides utilities to load and merge YAML configuration files
with smart defaults for the NeuralLuenbergerEstimator.
"""

import os
import yaml
from pathlib import Path
from typing import Dict, Any, Optional


def _deep_merge(base: Dict, override: Dict) -> Dict:
    """
    Deep merge two dictionaries recursively.
    
    Args:
        base: Base dictionary with defaults
        override: Override dictionary (values take precedence)
    
    Returns:
        Merged dictionary
    """
    result = base.copy()
    
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    
    return result


def load_neural_obs_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load neural observer configuration from YAML file.
    
    Args:
        config_path: Path to YAML config file. If None, uses default
                     'neural_obs_params.yaml' in the same directory.
    
    Returns:
        Configuration dictionary with all parameters
    """
    # Default config path
    if config_path is None:
        config_path = Path(__file__).parent / "neural_obs_params.yaml"
    
    config = {}
    
    # Load from file if it exists
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f) or {}
    
    return config


def flatten_config(config: Dict[str, Any]) -> Dict[str, Any]:
    """
    Flatten nested YAML config to flat dictionary for NeuralLuenbergerEstimator.
    
    Converts nested structure like:
        neural_network:
          input_dim: 6
    
    To flat structure:
        input_dim: 6
    
    Args:
        config: Nested configuration from YAML
    
    Returns:
        Flat configuration dictionary matching expected format
    """
    flat = {}
    
    # Neural network parameters
    nn = config.get('neural_network', {})
    flat['input_dim'] = nn.get('input_dim', 6)
    flat['hidden_dim'] = nn.get('hidden_dim', 32)
    flat['output_dim'] = nn.get('output_dim', 2)
    flat['network_type'] = nn.get('network_type', 'gru')
    flat['output_scale'] = nn.get('output_scale', 50.0)
    flat['use_acceleration'] = nn.get('use_acceleration', False)
    flat['disturbance_mode'] = nn.get('disturbance_mode', 'tire')
    
    # Learning parameters
    learn = config.get('learning', {})
    flat['learning_rate'] = learn.get('learning_rate', 0.005)
    flat['batch_size'] = learn.get('batch_size', 3)
    flat['weight_decay'] = learn.get('weight_decay', 0.0)
    flat['lambda_regularization'] = learn.get('lambda_regularization', 0.0)
    flat['learning_mode'] = learn.get('learning_mode', 'learningby_dict')
    flat['dict_size'] = learn.get('dict_size', 20)
    flat['gradient_method'] = learn.get('gradient_method', 'autodiff')
    flat['grad_clip_norm'] = learn.get('grad_clip_norm', 10.0)
    
    # Selective data curation parameters (for learningby_dict mode)
    flat['novelty_threshold'] = learn.get('novelty_threshold', 0.05)
    flat['min_excitation'] = learn.get('min_excitation', 0.02)
    flat['excitation_bonus_weight'] = learn.get('excitation_bonus_weight', 2.0)
    flat['use_diversity_replacement'] = learn.get('use_diversity_replacement', True)
    
    # Observer parameters
    obs = config.get('observer', {})
    flat['observer_gain'] = obs.get('observer_gain', 0.5)
    flat['sample_time'] = obs.get('sample_time', 0.02)
    flat['min_vx'] = obs.get('min_vx', 0.5)
    
    # Loss configuration
    loss = config.get('loss', {})
    flat['loss_type'] = loss.get('type', 'measurement_full')
    
    # Physics-informed tire loss parameters
    physics_tire = config.get('weights_physics_tire', {})
    flat['weight_physics_target'] = physics_tire.get('w_physics_target', 30.0)
    flat['weight_ay_constraint'] = physics_tire.get('w_ay', 0.0)
    flat['weight_smooth'] = physics_tire.get('w_smooth', 2.0)
    flat['lambda_bound'] = physics_tire.get('lambda_bound', 0.1)
    flat['f_max'] = physics_tire.get('f_max', 50.0)
    flat['lambda_warmstart'] = physics_tire.get('lambda_warmstart', 5.0)
    flat['warmstart_decay'] = physics_tire.get('warmstart_decay', 500)
    flat['rdot_ema_alpha'] = physics_tire.get('rdot_ema_alpha', 0.3)
    
    # Prediction error loss weights
    pred_err = config.get('weights_prediction_error', {})
    flat['pred_w_vx'] = pred_err.get('w_vx', 5.0)
    flat['pred_w_r'] = pred_err.get('w_r', 15.0)
    flat['pred_w_psi'] = pred_err.get('w_psi', 1.0)
    flat['pred_w_X'] = pred_err.get('w_X', 0.5)
    flat['pred_w_Y'] = pred_err.get('w_Y', 0.5)
    flat['pred_w_ay'] = pred_err.get('w_ay', 30.0)
    flat['pred_w_ax'] = pred_err.get('w_ax', 5.0)
    flat['pred_lambda_l2'] = pred_err.get('lambda_l2', 0.01)
    flat['pred_w_smooth'] = pred_err.get('w_smooth', 0.1)
    flat['pred_lambda_bound'] = pred_err.get('lambda_bound', 0.1)
    flat['pred_f_max'] = pred_err.get('f_max', 25.0)
    flat['pred_lambda_warmstart'] = pred_err.get('lambda_warmstart', 0.5)
    flat['pred_warmstart_decay'] = pred_err.get('warmstart_decay', 100)
    flat['pred_w_uio'] = pred_err.get('w_uio', 0.02)
    flat['pred_w_corr'] = pred_err.get('w_corr', 1.0)
    
    # Measurement weights
    w_meas = config.get('weights_measurement', {})
    flat['weight_vx'] = w_meas.get('v_x', 10.0)
    flat['weight_vy'] = w_meas.get('v_y', 20000.0)
    flat['weight_psi'] = w_meas.get('psi', 10.0)
    flat['weight_psi_dot'] = w_meas.get('psi_dot', 20000.0)
    flat['weight_X'] = w_meas.get('X', 1.0)
    flat['weight_Y'] = w_meas.get('Y', 1.0)
    
    # Reference tracking weights (for composite UIO loss)
    w_ref = config.get('weights_ref', {})
    flat['weight_ref_vx'] = w_ref.get('v_x', 5.0)
    flat['weight_ref_vy'] = w_ref.get('v_y', 5.0)
    flat['weight_ref_psi'] = w_ref.get('psi', 10.0)
    flat['weight_ref_r'] = w_ref.get('psi_dot', 5.0)
    flat['weight_ref_X'] = w_ref.get('X', 10.0)
    flat['weight_ref_Y'] = w_ref.get('Y', 10.0)
    
    # UIO agreement weights (for composite UIO loss)
    w_uio = config.get('weights_uio', {})
    flat['weight_uio_vx'] = w_uio.get('v_x', 5.0)
    flat['weight_uio_vy'] = w_uio.get('v_y', 10.0)
    flat['weight_uio_psi'] = w_uio.get('psi', 5.0)
    flat['weight_uio_r'] = w_uio.get('psi_dot', 10.0)
    flat['weight_uio_X'] = w_uio.get('X', 1.0)
    flat['weight_uio_Y'] = w_uio.get('Y', 1.0)
    
    # First-layer observer
    fl = config.get('first_layer', {})
    flat['use_first_layer'] = fl.get('enabled', True)
    flat['first_layer_type'] = fl.get('type', 'qlpv')
    flat['output_first_layer_only'] = fl.get('output_first_layer_only', False)
    flat['use_8d_system'] = fl.get('use_8d_system', False)
    flat['override_threshold'] = fl.get('override_threshold', 0.1)
    
    # Model persistence
    model = config.get('model', {})
    flat['model_path'] = model.get('path', 'trained_data/neural_observer_model.pt')
    flat['load_pretrained'] = model.get('load_pretrained', False)
    
    # Observer gain design configuration
    gain_design = config.get('observer_gain_design', {})
    flat['gain_design_method'] = gain_design.get('method', 'default')
    flat['hinf_gamma'] = gain_design.get('hinf_gamma', 1.0)
    flat['l2_gamma'] = gain_design.get('l2_gamma', 2.0)
    flat['lmi_decay_rate'] = gain_design.get('lmi_decay_rate', 0.5)
    flat['nominal_vx'] = gain_design.get('nominal_vx', 1.5)
    flat['use_gain_scheduling'] = gain_design.get('use_gain_scheduling', False)
    flat['vx_range'] = gain_design.get('vx_range', [0.5, 3.0])
    flat['delta_max'] = gain_design.get('delta_max', 0.4)
    flat['n_vx_vertices'] = gain_design.get('n_vx_vertices', 3)
    flat['n_delta_vertices'] = gain_design.get('n_delta_vertices', 3)
    flat['use_common_lyapunov'] = gain_design.get('use_common_lyapunov', True)
    flat['contraction_rate'] = gain_design.get('contraction_rate', 0.95)
    
    # Recording configuration
    recording = config.get('recording', {})
    flat['enable_recording'] = recording.get('enable_recording', False)
    flat['recording_output_dir'] = recording.get('output_dir', 'neural_obs_recordings')
    flat['recording_filename'] = recording.get('filename', None)
    flat['recording_append_mode'] = recording.get('append_mode', False)
    
    return flat


def get_default_config() -> Dict[str, Any]:
    """
    Get default configuration without loading from file.
    
    Returns:
        Default configuration dictionary
    """
    return flatten_config({})


def merge_with_overrides(yaml_config: Dict[str, Any], overrides: Dict[str, Any]) -> Dict[str, Any]:
    """
    Merge flattened YAML config with runtime overrides.
    
    Args:
        yaml_config: Flattened config from YAML file
        overrides: Runtime override dictionary
    
    Returns:
        Merged configuration
    """
    result = yaml_config.copy()
    result.update(overrides)
    return result
