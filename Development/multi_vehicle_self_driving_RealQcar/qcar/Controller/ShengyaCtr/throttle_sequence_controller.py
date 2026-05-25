




import numpy as np
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any, TYPE_CHECKING
from ..longitudinal_controllers import LongitudinalControllerBase



class ThrottleSequenceController(LongitudinalControllerBase):

    def __init__(self, 
                 throttle_sequence: Optional[Dict[str, Any]] = None,
                 config=None,
                 logger=None):
        """
        Initialize ThrottleSequenceController.
        
        Args:
            throttle_sequence: Dictionary with throttle values and timing info
                              e.g., {'values': [0.3, 0.5, 0.2], 'durations': [5.0, 10.0, 5.0]}
            config: Configuration dictionary
            logger: Logger instance
        """
        self.throttle_sequence = throttle_sequence or {}
        self.config = config
        self.logger = logger
        
        # Extract sequence values and durations
        self.throttle_values = self.throttle_sequence.get('values', [0.5])
        self.sequence_duration = self.throttle_sequence.get('duration', 5.0)  # Legacy single duration
        self.sequence_durations = self.throttle_sequence.get('durations')

        if self.sequence_durations is None:
            self.sequence_durations = [self.sequence_duration] * len(self.throttle_values)
        elif len(self.sequence_durations) != len(self.throttle_values):
            if self.logger:
                self.logger.warning(
                    "ThrottleSequenceController durations length does not match throttle values; "
                    "falling back to legacy duration for all steps"
                )
            self.sequence_durations = [self.sequence_duration] * len(self.throttle_values)
        
        # Initialize sequence state tracking
        self.current_sequence_index = 0
        self.time_in_current_sequence = 0.0
        
        if self.logger:
            self.logger.info(f"ThrottleSequenceController initialized with {len(self.throttle_values)} throttle values, "
                           f"durations={self.sequence_durations}s")

    def _current_duration(self) -> float:
        """Get duration for the current throttle sequence step."""
        return self.sequence_durations[self.current_sequence_index]

    def _advance_sequence(self, dt: float):
        """Advance sequence timing and switch steps when the current duration expires."""
        self.time_in_current_sequence += dt
        current_duration = self._current_duration()
        
        if self.time_in_current_sequence >= current_duration:
            self.time_in_current_sequence = 0.0
            self.current_sequence_index = (self.current_sequence_index + 1) % len(self.throttle_values)
            
            if self.logger:
                self.logger.debug(
                    f"Switching to throttle sequence index: {self.current_sequence_index}, "
                    f"duration={self._current_duration()}s"
                )

    def compute_throttle(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """
        Compute throttle command based on a predefined sequence of throttle values.
        
        Args:
            follower_state: Current state of the follower vehicle (position, velocity, etc.)
            leader_state: Current state of the leader vehicle (if needed for more complex sequences)
            dt: Time step duration
            
        Returns:
            Throttle command to apply
        """
        self._advance_sequence(dt)
        
        # Get current throttle value
        current_throttle = self.throttle_values[self.current_sequence_index]
        
        return current_throttle
    
    def reset(self):
        """Reset controller state to initial sequence"""
        self.current_sequence_index = 0
        self.time_in_current_sequence = 0.0
        
        if self.logger:
            self.logger.debug("ThrottleSequenceController reset to initial state")
    
    def update(self, v: float, v_ref: float, dt: float) -> float:
        """
        Update speed controller for path following (FOLLOWING_PATH state).
        
        Required method for compatibility with FOLLOWING_PATH state.
        Ignores velocity feedback and reference velocity, only updates the sequence
        timing and returns the current throttle value from the sequence.
        
        Args:
            v: Current velocity (ignored - sequence is open-loop)
            v_ref: Reference velocity (ignored - sequence is open-loop)
            dt: Time step duration
            
        Returns:
            Throttle command from the current sequence position
        """
        self._advance_sequence(dt)
        
        # Get current throttle value
        current_throttle = self.throttle_values[self.current_sequence_index]
        
        return current_throttle 
