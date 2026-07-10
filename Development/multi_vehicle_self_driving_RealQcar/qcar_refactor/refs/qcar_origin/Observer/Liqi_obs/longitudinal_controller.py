from transformers import Any

from ...Controller.longitudinal_controllers import LongitudinalControllerBase
from ...Controller.lateral_controllers import LateralControllerBase
import numpy as np
"""A Controller backup for experiments with distributed observer with longitudinal control only.
The controller itself is deployed on the Controller package and the two controllers: LeaderCruiseController and DummyLateralController are implemented in the longitudinal_controller module and lateral_controller module respectively.

The longitudinal controller is a simple cruise controller that tries to maintain a desired speed for leader and maintain a desired gap for followers,

while the lateral controller is a dummy controller that does not perform any steering control.

The main purpose of this setup is to test the performance of the longitudinal high-gain observer without the influence of lateral control.

The measurement input for the observer is the gap between the current vehicle and the front vehicle, which can be calculated using the position information from the local state estimator."""

class LeaderCruiseController(LongitudinalControllerBase):
    """
    Longitudinal controller implementing the paper's LCC platoon strategy.
    """
    def __init__(self,
                 vehicle_index: int = 0,
                 is_leader: bool = False,
                 config: Optional[Dict[str, Any]] = None,
                 logger: Any = None):
        self.logger = logger
        self.is_leader = bool(is_leader)
        self.vehicle_index = int(vehicle_index)

        self.v_h = getattr(config, f"v_h{self.vehicle_index}", 0.5)    # leader desired speed
        self.received_local_states = defaultdict(list)
        self.parameters = {
            "alpha": getattr(config, f"alpha{self.vehicle_index}", 0.5),  # spacing gain
            "beta":  getattr(config, f"beta{self.vehicle_index}", 0.5),   # speed
            "s_st":  getattr(config, f"s_st{self.vehicle_index}", 0.2),
            "s_go":  getattr(config, f"s_go{self.vehicle_index}", 0.5),
            "v_max": getattr(config, f"v_max{self.vehicle_index}", 0.5),
            "torque_to_throttle": getattr(config, f"torque_to_throttle{self.vehicle_index}", 1.0),
            "max_throttle": getattr(config, f"max_throttle{self.vehicle_index}", 1.0),
        }

    def compute_throttle(self,
                         follower_state: Dict[str, float],
                         leader_state: Dict[str, float],
                         dt: float) -> float:
        if self.is_leader:
            desired_speed = self._leader_velocity_logic(follower_state, dt)
        else:
            desired_speed = self._follower_velocity_logic(follower_state, leader_state, dt)
        speed_error = desired_speed - follower_state["speed"]
        throttle_command = self._speed_to_throttle(speed_error)
        return throttle_command
    
    def _leader_velocity_logic(self, follower_state: Dict[str, float], dt: float) -> float:
        desired_speed = self.v_h
        return desired_speed
    
    def _follower_velocity_logic(self, follower_state: Dict[str, float], leader_state: Dict[str, float], dt: float) -> float:
        gap = leader_state["x"] - follower_state["x"]
        if gap <= self.parameters["s_st"]:
            return 0.0
        if gap >= self.parameters["s_go"]:
            return self.parameters["v_max"]
        return 0.5 * self.parameters["v_max"] * (1.0 - math.cos(math.pi * (gap - self.parameters["s_st"]) / (self.parameters["s_go"] - self.parameters["s_st"])))
    
    def _speed_to_throttle(self, speed_error: float) -> float:
        throttle_command = self.parameters["alpha"] * speed_error
        throttle_command *= self.parameters["torque_to_throttle"]
        throttle_command = np.clip(throttle_command, -self.parameters["max_throttle"], self.parameters["max_throttle"])
        return throttle_command
    
    def _get_latest_received_state(self, vehicle_id: int, current_time_ns: int) -> Optional[np.ndarray]:
        """Return the most recent received state (as ndarray) within the age limit.

        Returns None if no recent state is available or conversion fails.
        """
        if vehicle_id not in self.received_local_states:
            if self.logger:
                #TODO: Better fall out the distributed estimator when its happen too much. 
                self.logger.logger.warning(f"vehicle_id {vehicle_id} not in self.received_local_states")
            return None
    
    def update_params(self, params: Dict[str, Any]):
        pass

    def reset(self):
        pass

class DummyLateralController(LateralControllerBase):
    """Returns zero steering — placeholder for pure longitudinal studies."""

    def __init__(self, config=None, logger=None):
        self.logger = logger

    def compute_steering(self,
                         follower_state: Dict[str, float],
                         leader_state: Optional[Dict[str, float]],
                         dt: float) -> float:
        return 0.0

    def update_params(self, params: Dict[str, Any]):
        pass

    def reset(self):
        pass
