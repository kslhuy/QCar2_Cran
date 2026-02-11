"""
Neural Network Module for Local Observer

Implements a 3-layer feedforward neural network with spectral normalization
for learning unknown tire forces and disturbances in vehicle dynamics.

Architecture:
    Input: 6 dimensions [v_x, v_y, ψ, ψ̇, δ, a]
    Hidden: 24 neurons × 2 layers (SiLU activation)
    Output: 2 dimensions [f_nn_1, f_nn_2] (tire force compensation)
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.nn.utils import spectral_norm
import numpy as np
from typing import Optional, List, Tuple
import os


class SpectralNormLinear(nn.Module):
    """Linear layer with spectral normalization for stability"""
    
    def __init__(self, in_features: int, out_features: int):
        super(SpectralNormLinear, self).__init__()
        self.linear = spectral_norm(nn.Linear(in_features, out_features))
    
    def forward(self, x):
        return self.linear(x)


class NeuralObserverNet(nn.Module):
    """
    Neural network for learning unknown dynamics in vehicle observer
    
    Uses spectral normalization for training stability and SiLU activation
    for smooth gradients.
    """
    
    def __init__(self, input_dim: int = 6, hidden_dim: int = 24, output_dim: int = 2):
        """
        Initialize neural network
        
        Args:
            input_dim: Input dimension (default: 6 for [v_x, v_y, ψ, ψ̇, δ, a])
            hidden_dim: Hidden layer dimension (default: 24)
            output_dim: Output dimension (default: 2 for [f_f, f_r])
        """
        super(NeuralObserverNet, self).__init__()
        
        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.output_dim = output_dim
        
        # Three-layer network with spectral normalization
        self.fc1 = SpectralNormLinear(input_dim, hidden_dim)
        self.fc2 = SpectralNormLinear(hidden_dim, hidden_dim)
        self.fc3 = SpectralNormLinear(hidden_dim, output_dim)
        
        # SiLU (Swish) activation for smooth gradients
        self.activation = nn.SiLU()
    
    def forward(self, x):
        """
        Forward pass through the network
        
        Args:
            x: Input tensor of shape (batch_size, input_dim) or (input_dim, 1)
        
        Returns:
            Output tensor of shape (batch_size, output_dim) or (output_dim, 1)
        """
        # Handle both batch and single sample inputs
        if x.dim() == 2 and x.shape[1] == 1:
            x = x.squeeze(1)  # Convert (input_dim, 1) to (input_dim,)
            single_sample = True
        else:
            single_sample = False
        
        # Forward pass
        x = self.activation(self.fc1(x))
        x = self.activation(self.fc2(x))
        x = self.fc3(x)
        
        # Restore shape for single sample
        if single_sample:
            x = x.unsqueeze(1)  # Convert back to (output_dim, 1)
        
        return x
    
    def myloss(self, output, gradient):
        """
        Custom loss function for gradient-based learning
        
        This loss function uses the chain rule gradient from the observer
        dynamics to train the network.
        
        Args:
            output: Network output (f_nn)
            gradient: Gradient dL/df from chain rule
        
        Returns:
            Loss value (scalar)
        """
        # Convert gradient to tensor if it's numpy
        if isinstance(gradient, np.ndarray):
            gradient = torch.from_numpy(gradient).float()
        
        # Ensure gradient has correct shape
        if gradient.dim() == 2:
            gradient = gradient.squeeze(0)  # (1, output_dim) -> (output_dim,)
        
        # Compute loss: sum of element-wise product
        loss = torch.sum(output.squeeze() * gradient)
        
        return loss


class LearningBatch:
    """
    Experience replay buffer for dictionary-based learning
    
    Stores features, targets, and gradients for batch training.
    """
    
    def __init__(self, max_size: int = 20):
        """
        Initialize learning batch
        
        Args:
            max_size: Maximum number of samples to store
        """
        self.max_size = max_size
        self.feature_dict = []
    
    def add_feature(self, 
                   nn_input: np.ndarray,
                   f_nn: np.ndarray,
                   state_hat: np.ndarray,
                   target: np.ndarray,
                   f_uk: np.ndarray,
                   dx_df: np.ndarray,
                   time_step: int):
        """
        Add a new feature to the dictionary
        
        Args:
            nn_input: Neural network input
            f_nn: Neural network output
            state_hat: Estimated state
            target: Target measurement/reference
            f_uk: True unknown force (if available)
            dx_df: Sensitivity matrix
            time_step: Current time step
        """
        # Create feature tuple
        feature = (nn_input.copy(), f_nn.copy(), state_hat.copy(), 
                  target.copy(), f_uk.copy(), dx_df.copy(), time_step)
        
        # Add to dictionary
        self.feature_dict.append(feature)
        
        # Remove oldest if exceeds max size
        if len(self.feature_dict) > self.max_size:
            self.feature_dict.pop(0)
    
    def clear(self):
        """Clear all stored features"""
        self.feature_dict = []
    
    def __len__(self):
        return len(self.feature_dict)


class SelectiveLearningBatch:
    """
    Informative Experience Replay Buffer with Rich-Data Curation
    
    Only stores data that is sufficiently novel and informative for learning
    tire nonlinearities. Prioritizes turning maneuvers and diverse operating
    conditions over redundant straight-line driving data.
    
    Key strategies:
        1. Novelty gate: Reject data too similar to existing dictionary entries
        2. Excitation score: Prioritize high-information data (turning, lateral dynamics)
        3. Diversity replacement: Replace most similar entry instead of oldest (FIFO)
        4. Minimum excitation: Don't accept data below an excitation threshold
    """
    
    # Indices into nn_input: [v_x, v_y, ψ, r, δ, a] (6D) or 8D with accel
    _IDX_VX = 0
    _IDX_VY = 1
    _IDX_PSI = 2
    _IDX_R = 3
    _IDX_DELTA = 4
    _IDX_THROTTLE = 5
    
    def __init__(self, max_size: int = 20,
                 novelty_threshold: float = 0.05,
                 min_excitation: float = 0.02,
                 excitation_bonus_weight: float = 2.0,
                 use_diversity_replacement: bool = True):
        """
        Initialize selective learning batch
        
        Args:
            max_size: Maximum number of samples to store
            novelty_threshold: Minimum normalized distance to existing entries
                               for a new sample to be accepted (0.0 = accept all)
            min_excitation: Minimum excitation score to accept a sample.
                            Samples with score below this are rejected.
                            Based on |δ|, |r|, |v_y| signals.
            excitation_bonus_weight: Weight for excitation score in replacement
                                     priority (higher = prefer replacing low-excitation data)
            use_diversity_replacement: If True, replace most similar entry when full;
                                       if False, replace entry with lowest excitation score.
        """
        self.max_size = max_size
        self.feature_dict = []        # Same format as LearningBatch
        self._excitation_scores = []  # Parallel list of excitation scores
        self._nn_input_cache = []     # Cached nn_inputs for fast distance computation
        
        # Tunable parameters
        self.novelty_threshold = novelty_threshold
        self.min_excitation = min_excitation
        self.excitation_bonus_weight = excitation_bonus_weight
        self.use_diversity_replacement = use_diversity_replacement
        
        # Normalization scales for distance computation
        # [v_x, v_y, ψ, r, δ, a] — approximate operating ranges
        self._input_scales = np.array([2.0, 0.5, np.pi, 2.0, 0.5, 1.0])
        
        # Statistics
        self.total_offered = 0
        self.total_accepted = 0
        self.total_rejected_novelty = 0
        self.total_rejected_excitation = 0
    
    def set_input_scales(self, scales: np.ndarray):
        """Set normalization scales for distance computation (match input_dim)."""
        self._input_scales = np.array(scales).flatten()
    
    def _compute_excitation_score(self, nn_input: np.ndarray) -> float:
        """
        Compute excitation score measuring how informative this data point is
        for learning tire nonlinearities.
        
        High excitation = turning maneuver, lateral dynamics active.
        Low excitation = straight-line driving, coasting.
        
        Score components:
            - |δ| (steering angle): main indicator of cornering
            - |r| (yaw rate): turning dynamics
            - |v_y| (lateral velocity): side-slip indicates tire saturation
            - |δ̇| approximation via |δ| level (larger δ → more interesting)
        
        Args:
            nn_input: Neural network input vector (input_dim × 1) or (input_dim,)
        
        Returns:
            Excitation score ∈ [0, ∞), higher = more informative
        """
        inp = nn_input.flatten()
        
        # Extract relevant signals
        vy = abs(inp[self._IDX_VY]) if len(inp) > self._IDX_VY else 0.0
        r = abs(inp[self._IDX_R]) if len(inp) > self._IDX_R else 0.0
        delta = abs(inp[self._IDX_DELTA]) if len(inp) > self._IDX_DELTA else 0.0
        
        # Weighted excitation score
        # δ and r are strongest indicators of tire nonlinearity excitation
        score = (3.0 * delta       # Steering is the primary excitation
               + 2.0 * r           # Yaw rate confirms active turning
               + 1.5 * vy)         # Lateral velocity indicates side-slip
        
        return score
    
    def _compute_novelty(self, nn_input: np.ndarray) -> Tuple[float, int]:
        """
        Compute minimum normalized distance to all existing dictionary entries.
        
        Args:
            nn_input: New candidate input (input_dim × 1) or (input_dim,)
        
        Returns:
            Tuple of (min_distance, index_of_most_similar_entry)
        """
        if len(self._nn_input_cache) == 0:
            return float('inf'), -1
        
        inp = nn_input.flatten()
        # Use only the first len(scales) dimensions for distance
        n = min(len(inp), len(self._input_scales))
        inp_norm = inp[:n] / (self._input_scales[:n] + 1e-8)
        
        min_dist = float('inf')
        min_idx = 0
        
        for i, cached in enumerate(self._nn_input_cache):
            cached_flat = cached.flatten()
            cached_norm = cached_flat[:n] / (self._input_scales[:n] + 1e-8)
            dist = np.linalg.norm(inp_norm - cached_norm)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        
        return min_dist, min_idx
    
    def add_feature(self, 
                   nn_input: np.ndarray,
                   f_nn: np.ndarray,
                   state_hat: np.ndarray,
                   target: np.ndarray,
                   f_uk: np.ndarray,
                   dx_df: np.ndarray,
                   time_step: int) -> bool:
        """
        Selectively add a new feature to the dictionary.
        
        Only adds if the data passes novelty and excitation gates.
        When full, replaces the most similar (least diverse) entry or
        the least excited entry.
        
        Args:
            nn_input: Neural network input
            f_nn: Neural network output
            state_hat: Estimated state
            target: Target measurement/reference
            f_uk: True unknown force (if available)
            dx_df: Sensitivity matrix
            time_step: Current time step
            
        Returns:
            True if the sample was accepted, False if rejected
        """
        self.total_offered += 1
        
        # --- Gate 1: Minimum excitation ---
        excitation = self._compute_excitation_score(nn_input)
        if excitation < self.min_excitation:
            self.total_rejected_excitation += 1
            return False
        
        # --- Gate 2: Novelty check ---
        min_dist, most_similar_idx = self._compute_novelty(nn_input)
        if min_dist < self.novelty_threshold:
            # Data is too similar to an existing entry
            # Exception: if new data has MUCH higher excitation, replace the similar one
            if most_similar_idx >= 0 and excitation > self._excitation_scores[most_similar_idx] * 2.0:
                # Replace the similar but less excited entry
                self._replace_entry(most_similar_idx, nn_input, f_nn, state_hat,
                                   target, f_uk, dx_df, time_step, excitation)
                self.total_accepted += 1
                return True
            
            self.total_rejected_novelty += 1
            return False
        
        # --- Data accepted: Add or Replace ---
        feature = (nn_input.copy(), f_nn.copy(), state_hat.copy(),
                  target.copy(), f_uk.copy(), dx_df.copy(), time_step)
        
        if len(self.feature_dict) < self.max_size:
            # Dictionary not full — just append
            self.feature_dict.append(feature)
            self._excitation_scores.append(excitation)
            self._nn_input_cache.append(nn_input.copy())
        else:
            # Dictionary full — replace strategically
            if self.use_diversity_replacement:
                # Replace the entry most similar to new data (keep diversity)
                replace_idx = most_similar_idx if most_similar_idx >= 0 else 0
            else:
                # Replace the entry with lowest excitation score
                replace_idx = int(np.argmin(self._excitation_scores))
            
            # Only replace if new data is more informative
            if excitation > self._excitation_scores[replace_idx] * 0.8:
                self._replace_entry(replace_idx, nn_input, f_nn, state_hat,
                                   target, f_uk, dx_df, time_step, excitation)
            else:
                self.total_rejected_novelty += 1
                return False
        
        self.total_accepted += 1
        return True
    
    def _replace_entry(self, idx: int,
                      nn_input: np.ndarray, f_nn: np.ndarray,
                      state_hat: np.ndarray, target: np.ndarray,
                      f_uk: np.ndarray, dx_df: np.ndarray,
                      time_step: int, excitation: float):
        """Replace entry at given index with new data."""
        feature = (nn_input.copy(), f_nn.copy(), state_hat.copy(),
                  target.copy(), f_uk.copy(), dx_df.copy(), time_step)
        self.feature_dict[idx] = feature
        self._excitation_scores[idx] = excitation
        self._nn_input_cache[idx] = nn_input.copy()
    
    def clear(self):
        """Clear all stored features"""
        self.feature_dict = []
        self._excitation_scores = []
        self._nn_input_cache = []
    
    def get_acceptance_rate(self) -> float:
        """Get the fraction of offered samples that were accepted."""
        if self.total_offered == 0:
            return 0.0
        return self.total_accepted / self.total_offered
    
    def get_stats(self) -> dict:
        """Get dictionary curation statistics."""
        return {
            'total_offered': self.total_offered,
            'total_accepted': self.total_accepted,
            'total_rejected_novelty': self.total_rejected_novelty,
            'total_rejected_excitation': self.total_rejected_excitation,
            'acceptance_rate': self.get_acceptance_rate(),
            'current_size': len(self.feature_dict),
            'avg_excitation': float(np.mean(self._excitation_scores)) if self._excitation_scores else 0.0,
        }
    
    def __len__(self):
        return len(self.feature_dict)


class ModelQueue:
    """
    Queue of neural network models for continuous learning
    
    Maintains multiple models and uses weighted predictions.
    """
    
    def __init__(self, input_dim: int = 6, output_dim: int = 2, 
                 queue_size: int = 3, hidden_dim: int = 24):
        """
        Initialize model queue
        
        Args:
            input_dim: Input dimension
            output_dim: Output dimension
            queue_size: Maximum number of models in queue
            hidden_dim: Hidden layer dimension
        """
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.queue_size = queue_size
        self.hidden_dim = hidden_dim
        self.models = []
        self.weights = []
    
    def add_queue(self, duplicate: bool, input_dim: int, 
                  hidden_dim: int, output_dim: int):
        """
        Add a new model to the queue
        
        Args:
            duplicate: If True, duplicate the latest model; otherwise create new
            input_dim: Input dimension
            hidden_dim: Hidden layer dimension
            output_dim: Output dimension
        """
        if duplicate and len(self.models) > 0:
            # Duplicate the latest model
            new_model = NeuralObserverNet(input_dim, hidden_dim, output_dim)
            new_model.load_state_dict(self.models[-1].state_dict())
        else:
            # Create a new model
            new_model = NeuralObserverNet(input_dim, hidden_dim, output_dim)
        
        self.models.append(new_model)
        
        # Remove oldest if exceeds queue size
        if len(self.models) > self.queue_size:
            self.models.pop(0)
        
        # Update weights (exponential decay)
        self._update_weights()
    
    def push_model(self, source_model: NeuralObserverNet):
        """
        Push a snapshot of the source model into the queue.
        
        Args:
           source_model: The model to snapshot
        """
        # Create a new instance with same architecture
        new_model = NeuralObserverNet(self.input_dim, self.hidden_dim, self.output_dim)
        
        # Copy weights from source model
        new_model.load_state_dict(source_model.state_dict())
        
        # Add to queue
        self.models.append(new_model)
        
        # Remove oldest if exceeds queue size
        if len(self.models) > self.queue_size:
             self.models.pop(0)
             
        # Update ensemble weights
        self._update_weights()
    
    def _update_weights(self):
        """Update weights for model predictions (exponential decay)"""
        n = len(self.models)
        if n == 0:
            self.weights = []
            return
        
        # Exponential weights: newer models have higher weight
        weights = np.array([np.exp(i) for i in range(n)])
        weights = weights / np.sum(weights)
        self.weights = weights
    
    def predict(self, x):
        """
        Predict using weighted sum of all models
        
        Args:
            x: Input tensor
        
        Returns:
            Weighted prediction
        """
        if len(self.models) == 0:
            raise ValueError("No models in queue")
        
        # Convert input to tensor if needed
        if isinstance(x, np.ndarray):
            x = torch.from_numpy(x).float()
        
        # Get predictions from all models
        predictions = []
        for model in self.models:
            with torch.no_grad():
                pred = model(x)
            predictions.append(pred)
        
        # Weighted sum
        weighted_pred = sum(w * p for w, p in zip(self.weights, predictions))
        
        return weighted_pred
    
    def get_latest_model(self):
        """Get the most recent model in the queue"""
        if len(self.models) == 0:
            return None
        return self.models[-1]


def save_model(model: NeuralObserverNet, filepath: str):
    """
    Save model state dict to file
    
    Args:
        model: Neural network model
        filepath: Path to save file
    """
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    torch.save(model.state_dict(), filepath)


def load_model(filepath: str, input_dim: int = 6, 
               hidden_dim: int = 24, output_dim: int = 2) -> NeuralObserverNet:
    """
    Load model from file
    
    Args:
        filepath: Path to model file
        input_dim: Input dimension
        hidden_dim: Hidden layer dimension
        output_dim: Output dimension
    
    Returns:
        Loaded neural network model
    """
    model = NeuralObserverNet(input_dim, hidden_dim, output_dim)
    model.load_state_dict(torch.load(filepath))
    return model


def create_optimizer(model: NeuralObserverNet, learning_rate: float = 0.005,
                    weight_decay: float = 0.0) -> torch.optim.Optimizer:
    """
    Create Adam optimizer for the model
    
    Args:
        model: Neural network model
        learning_rate: Learning rate
        weight_decay: Weight decay (L2 regularization)
    
    Returns:
        Adam optimizer
    """
    return torch.optim.Adam(model.parameters(), lr=learning_rate, 
                           weight_decay=weight_decay)
