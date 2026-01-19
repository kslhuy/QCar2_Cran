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
