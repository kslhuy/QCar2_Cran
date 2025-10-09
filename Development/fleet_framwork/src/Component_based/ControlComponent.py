import time
import numpy as np
import logging
from typing import Optional, Dict, Any, Tuple


class ControlComponent:
    """
    Control component for vehicle control logic.
    Handles both leader and follower control algorithms.
    """

    def __init__(self, vehicle_id: int, is_leader: bool, config: Dict,
                 leader_controller=None, follower_controller=None,
                 gps_component=None, state_queue=None, gps_sync=None,
                 qcar=None, physical_qcar=None,
                 logger: logging.Logger = None):
        """
        Initialize control component.

        Args:
            vehicle_id: Unique vehicle identifier
            is_leader: Whether this is a leader vehicle
            config: Configuration dictionary
            leader_controller: Leader controller instance
            follower_controller: Follower controller instance
            gps_component: GPS component for state data
            state_queue: State queue for received states
            gps_sync: GPS sync for time synchronization
            logger: Logger instance
        """
        self.vehicle_id = vehicle_id
        self.is_leader = is_leader
        self.config = config
        self.leader_controller = leader_controller
        self.follower_controller = follower_controller
        self.gps_component = gps_component
        self.state_queue = state_queue
        self.gps_sync = gps_sync
        self.logger = logger or logging.getLogger(f'control_vehicle_{vehicle_id}')

        # Control parameters
        self.max_steering = config.get('max_steering', 0.6)
        self.lookahead_distance = config.get('lookahead_distance', 7.0)
        self.k_steering = 2.0
        self.update_rate = config.get('general_update_rate', 100)
        self.controller_rate = config.get('controller_rate', 100)

        # Chain-following configuration
        self.following_target = config.get('following_target', None)
        
        # Debug logging
        self.logger.info(f"ControlComponent initialized for vehicle {vehicle_id}, is_leader={is_leader}, following_target={self.following_target}")

        # Control state
        self.current_control_input = np.array([0.0, 0.0])
        self._last_control_time = None
        self._last_steering_angle = 0.0

        # Physical QCar settings
        self.use_physical_qcar = config.get('use_physical_qcar', False)
        self.use_control_observer_mode = config.get('use_control_observer_mode', False)
        self.physical_qcar = physical_qcar  # Set from parameter

        # Virtual QCar settings
        self.qcar = qcar  # Set from parameter

    def update_control(self) -> Tuple[float, float]:
        """
        Update vehicle control based on role (leader/follower).

        Returns:
            Tuple of (forward_speed, steering_angle)
        """
        try:
            if self.is_leader:
                return self._leader_control_logic()
            else:
                return self._follower_control_logic()
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Control update error: {e}")
            return 0.0, 0.0

    def _leader_control_logic(self) -> Tuple[float, float]:
        """Leader control logic that delegates to VehicleLeaderController."""
        if self.leader_controller is None:
            return 0.0, 0.0

        # Get state for control
        control_state = self._get_state_for_control()

        # Calculate actual time elapsed since last control update
        current_time = time.time()
        if self._last_control_time is None:
            dt = 1.0 / self.update_rate  # Use nominal dt for first iteration
        else:
            dt = current_time - self._last_control_time
            # Clamp dt to reasonable bounds to avoid instability
            dt = max(0.001, min(dt, 0.1))  # Between 1ms and 100ms

        self._last_control_time = current_time

        # Compute control commands using the dedicated leader controller
        forward_speed, steering_angle = self.leader_controller.compute_control_auto(
            current_pos=control_state['position'],
            current_rot=control_state['rotation'],
            velocity=control_state['velocity'],
            dt=dt
        )

        # Update control input for observer
        self.current_control_input = np.array([steering_angle, forward_speed])

        # Store steering angle for physical QCar EKF
        self._last_steering_angle = steering_angle

        # Log control timing for debugging
        if self.logger.isEnabledFor(logging.DEBUG):
            target_dt = 1.0 / self.update_rate
            timing_error = abs(dt - target_dt) / target_dt * 100
            self.logger.debug(f"Leader control timing - "
                            f"dt={dt:.4f}s (target={target_dt:.4f}s, error={timing_error:.1f}%)")

        return forward_speed, steering_angle

    def _follower_control_logic(self) -> Tuple[float, float]:
        """Follower control logic that delegates to VehicleFollowerController."""
        if self.follower_controller is None:
            self.logger.debug(f"Vehicle {self.vehicle_id}: Skipping follower control - no controller")
            return 0.0, 0.0

        try:
            # Use GPS-synchronized time for better prediction
            current_gps_time = self.gps_sync.get_synced_time()

            # Get the state data from the target vehicle (chain-following)
            target_data = self._get_target_vehicle_state(current_gps_time)

            if target_data is None:
                # print(f"Vehicle {self.vehicle_id}: No target data available, maintaining current control")
                # self.logger.warning(f"Vehicle {self.vehicle_id}: No target data available, maintaining current control")
                # Return current control input to maintain vehicle state instead of stopping
                # return self.current_control_input[1], self.current_control_input[0]  # [steering, speed] -> [speed, steering]
                return 0.0, 0.0  # [steering, speed] -> [speed, steering]
            # Get current vehicle state for control
            current_state = self._get_state_for_control()

            # Compute control using the follower controller
            forward_speed, steering_angle = self.follower_controller.compute_control(
                current_pos=current_state['position'],
                current_rot=current_state['rotation'],
                current_velocity=current_state['velocity'],
                leader_pos=target_data['position'],
                leader_rot=target_data['rotation'],
                leader_velocity=target_data['velocity'],
                leader_timestamp=target_data['timestamp'],
                dt=1.0 / self.update_rate
            )

            # Update control input for observer
            self.current_control_input = np.array([steering_angle, forward_speed])

            # Debug logging
            if self.logger.isEnabledFor(logging.DEBUG):
                self.logger.debug(f"Follower control: forward={forward_speed:.3f}, steering={steering_angle:.3f}, "
                                f"target_vehicle={self.following_target}")

            return forward_speed, steering_angle

        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Follower control error: {e}")
            self.current_control_input = np.array([0.0, 0.0])
            return 0.0, 0.0

    def _get_target_vehicle_state(self, target_time: float) -> Optional[Dict[str, Any]]:
        """
        Get interpolated state from the target vehicle this one should follow.
        """
        if self.following_target is None:
            return None

        target_vehicle_id = self.following_target

        # Get interpolated state from the target vehicle
        result = self.state_queue.get_interpolated_state(
            target_time,
            sender_id=target_vehicle_id,
            current_gps_time=self.gps_sync.get_synced_time()
        )

        if result:
            self.logger.debug(f"Vehicle {self.vehicle_id}: Got target state from vehicle {target_vehicle_id}: pos={result.get('pos', [0,0,0])}, v={result.get('v', 0.0):.3f}")
            return {
                'position': result.get('pos', [0, 0, 0]),
                'rotation': result.get('rot', [0, 0, 0]),
                'velocity': result.get('v', 0.0),
                'timestamp': result.get('timestamp', target_time)
            }
        else:
            # Get queue stats for debugging
            queue_stats = self.state_queue.get_queue_stats()
            self.logger.warning(f"Vehicle {self.vehicle_id}: No target state available from vehicle {target_vehicle_id}. Queue stats: size={queue_stats['current_queue_size']}, total_received={queue_stats['total_received']}, valid={queue_stats['valid_states']}")
            return None

    def _get_state_for_control(self) -> Dict[str, Any]:
        """Get current vehicle state for control algorithms."""
        if self.gps_component:
            return self.gps_component.get_current_state()
        else:
            # Fallback if no GPS component available
            return {
                'position': [0, 0, 0],
                'rotation': [0, 0, 0],
                'velocity': 0.0
            }

    def apply_control_commands(self, forward_speed: float, steering_angle: float):
        """
        Apply control commands to the vehicle.

        Args:
            forward_speed: Forward speed command
            steering_angle: Steering angle command
        """
        try:
            # For leader vehicles in observer mode, commands are already applied in VehicleLeaderController
            if self.is_leader and self.use_control_observer_mode:
                # Commands already applied by VehicleLeaderController.compute_control_w_observer()
                return

            if self.use_physical_qcar and self.physical_qcar is not None:
                # Physical QCar mode (non-observer)
                self.physical_qcar.write(forward_speed, steering_angle)
            else:
                # Virtual QCar mode
                self.qcar.set_velocity_and_request_state(
                    forward=forward_speed,
                    turn=steering_angle,
                    headlights=False,
                    leftTurnSignal=False,
                    rightTurnSignal=False,
                    brakeSignal=False,
                    reverseSignal=False
                )
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Error applying control commands: {e}")

    def emergency_stop(self):
        """Emergency stop the vehicle."""
        try:
            # For leader vehicles in observer mode, emergency stop is handled by VehicleLeaderController
            if self.is_leader and self.use_control_observer_mode:
                # Emergency stop already handled by VehicleLeaderController.stop_control()
                return

            if self.use_physical_qcar and self.physical_qcar is not None:
                self.physical_qcar.write(0, 0)
            elif self.qcar:
                self.qcar.set_velocity_and_request_state(
                    forward=0.0, turn=0.0,
                    headlights=False, leftTurnSignal=False,
                    rightTurnSignal=False, brakeSignal=False, reverseSignal=False
                )
            self.current_control_input = np.array([0.0, 0.0])
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Error in emergency stop: {e}")

    def get_current_control_input(self) -> np.ndarray:
        """Get current control input for observer."""
        return self.current_control_input.copy()

    def cleanup(self):
        """Clean up control component resources."""
        try:
            # Stop controllers
            if self.leader_controller and hasattr(self.leader_controller, 'stop_control'):
                self.leader_controller.stop_control()
            if self.follower_controller and hasattr(self.follower_controller, 'stop_control'):
                self.follower_controller.stop_control()
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Control cleanup error: {e}")