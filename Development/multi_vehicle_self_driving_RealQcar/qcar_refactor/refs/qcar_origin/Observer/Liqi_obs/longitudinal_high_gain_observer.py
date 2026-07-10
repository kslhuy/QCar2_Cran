"""
Fleet State Estimator using a Longitudinal High-Gain Observer,
the measurement input is the gap between the current vehicle and the front vehicle, which can be calculated 
"""
import copy
import os
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np
import scipy as sp
import yaml

from ..local_state_estimators import LocalStateEstimatorBase
from ..fleet_state_estimators import FleetStateEstimatorBase


class LongitudinalHighGainFleetStateEstimator(FleetStateEstimatorBase):
    def __init__(self,
                 vehicle_id: int,
                 fleet_size: int,
                 state_dim: int = 3,
                 config: Dict[str, Any] = None,
                 logger=None):
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)
        self.gamma = config.get('consensus_gain', 1.0)
        self.theta = config.get(f'direct_gain{self.vehicle_id}', 1.0)
        self.L = np.asarray(config.get(f'L{self.vehicle_id}', [[1], [2], [1]]), dtype=float)
        self.T = np.diag([self.theta, self.theta**2, self.theta**3])
        self.K = self.T @ self.L
        self.laplacian_matrix = config.get('laplacian_matrix', np.zeros((fleet_size, fleet_size)))
        self.neighbor_indices = self.laplacian_matrix[self.vehicle_id, :].nonzero()[0]
        self.alpha = np.asarray(config.get(f'alpha{self.vehicle_id}', 0.1), dtype=float)
        self.beta = np.asarray(config.get(f'beta{self.vehicle_id}', 0.2), dtype=float)
        self.A = np.array([[0, 1, 0],
                           [0, 0, 1],
                           [0, 0, 0]], dtype=float)
        self.B = np.array([[0], [0], [1]], dtype=float)
        self.C = np.array([[1, 0, 0]], dtype=float)
    
    def update(self, local_state: np.ndarray, dt: float, 
               current_time_ns: int, control: np.ndarray) -> np.ndarray:
        """
        Update the state estimates for all vehicles in the fleet using the high-gain observer.
        """
        try:
            # Get the gap measurement to the front vehicle
            position_i = local_state[0]
            gap_meas = self.gap_to_front_vehicle(position_i, current_time_ns)

            # Local observer correction
            local_correction = self.local_observer_correction(local_state, gap_meas)

            # Consensus correction
            consensus_correction = self.consensus_correction(current_time_ns)

            # Combine corrections and update state estimates using RK4 method
            total_correction = local_correction + consensus_correction
            updated_state = self._rk4_method(self.fleet_states, self._predict_longitudinal_model, dt, control) + total_correction.reshape(-1) * dt

            return updated_state
            
        except Exception as e:
            if self.logger:
                self.logger.logger.error(f"Error in Distributed High-Gain Observer update: {e}")
            return None
        
    def gap_to_front_vehicle(self,
                            position_i: float,
                            current_time_ns: int) -> np.ndarray:
        # Here we assume the car is driving along x axis.
        try:
            position_i_1  = self._get_latest_received_state(self.vehicle_id - 1, current_time_ns)
            gap = position_i_1[0] - position_i
        except Exception as e:
            if self.logger:
                self.logger.logger.error(f"Error in gap measurement, maybe the front vehicle is not available or overflow: {e}")
        return np.array([gap])

    def local_observer_correction(self,
                                  local_state: np.ndarray,
                                  measurement: np.ndarray) -> np.ndarray:
        """ Corr_loc = K * (y - C * x_hat) """
        top_zeros = np.zeros([3, self.vehicle_id - 1])
        bottom_zeros = np.zeros([3, self.fleet_size - self.vehicle_id])
        local_correction = self.K @ (measurement - self.C @ local_state)
        local_correction = np.hstack((top_zeros, local_correction, bottom_zeros))
        return local_correction
    
    def consensus_correction(self,
                             current_time_ns: int) -> np.ndarray:
        consensus_correction = np.zeros([3, self.fleet_size])
        for i in self.neighbor_indices:
            neighbor_state = self._get_latest_fleet_data(i, current_time_ns)
            consensus_correction += self.laplacian_matrix[self.vehicle_id, i] * (neighbor_state - self.fleet_states)
        consensus_correction *= self.gamma
        return consensus_correction

    @staticmethod
    def _rk4_method(state_variable: np.ndarray,
                        dynamics_func: callable,
                        dt: float,
                        params=None) -> np.ndarray:
        k1 = dynamics_func(state_variable, params)
        k2 = dynamics_func(state_variable + 0.5 * dt * k1, params)
        k3 = dynamics_func(state_variable + 0.5 * dt * k2, params)
        k4 = dynamics_func(state_variable + dt * k3, params)
        return state_variable + (k1 + 2*k2 + 2*k3 + k4)/6 * dt

    @staticmethod
    def _euler_method(state_variable: np.ndarray,
                        dynamics_func: callable,
                        dt: float,
                        params=None) -> np.ndarray:
        return state_variable + dynamics_func(state_variable, params) * dt

    def _predict_longitudinal_model(self, control_input: np.ndarray) -> np.ndarray:
        fleet_states_dot = np.zeros_like(self.fleet_states)
        for i in range(self.fleet_size):
            if i == 0:
                pi, vi, acci = self.fleet_states[:, i].copy()
                si_dot = vi
                vsi_dot = acci
                acci_dot = 0
            else:
                si, vsi, accsi = self.fleet_states[:, self.vehicle_id].copy()
                si_dot = vsi
                vsi_dot = accsi
                # Calculate the speed
                pi = pi + si
                vi = vi + vsi
                accsi_dot = 0
            fleet_states_dot[:,]

        return np.array([si_dot, vsi_dot, accsi_dot]).reshape(-1, 1)