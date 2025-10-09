import time
import logging
from typing import Dict, Any, Optional
from collections import deque


class CommunicationComponent:
    """
    Communication component for vehicle-to-vehicle communication.
    Handles state broadcasting, fleet estimates, and received message processing.
    """

    def __init__(self, vehicle_id: int, config: Dict, comm_handler=None,
                 observer=None, state_queue=None, gps_sync=None,
                 trust_evaluator=None, cam_lidar_fusion=None, logger: logging.Logger = None):
        """
        Initialize communication component.

        Args:
            vehicle_id: Unique vehicle identifier
            config: Configuration dictionary
            comm_handler: Communication handler instance
            observer: Vehicle observer instance
            state_queue: State queue for received states
            gps_sync: GPS sync for time synchronization
            trust_evaluator: Trust evaluator instance
            cam_lidar_fusion: CamLidarFusion component for relative distance measurement
            logger: Logger instance
        """
        self.vehicle_id = vehicle_id
        self.config = config
        self.comm = comm_handler
        self.observer = observer
        self.state_queue = state_queue
        self.gps_sync = gps_sync
        self.trust_evaluator = trust_evaluator
        self.cam_lidar_fusion = cam_lidar_fusion
        self.logger = logger or logging.getLogger(f'communication_vehicle_{vehicle_id}')

        # Cross-reference to control component (set by VehicleProcess)
        self.control_component = None

        # Communication settings
        self.target_ip = config['target_ip']
        self.send_port = config['send_port']
        self.recv_port = config['recv_port']
        self.ack_port = config['ack_port']
        self.peer_ports = config.get('peer_ports', {})
        self.communication_mode = config.get('communication_mode', 'bidirectional')

        # Fleet state management
        self.fleet_state_estimates = {}
        self.external_fleet_estimates = {}

        # Leader state fallback
        self.leader_state = None
        self.following_target = config.get('following_target', None)

        # Logging control
        self._last_no_data_log = 0.0
        self._last_comm_error_log = 0.0

    def initialize(self):
        """Initialize communication sockets."""
        if self.comm:
            try:
                self.comm.initialize_sockets()
                self.logger.info(f"Vehicle {self.vehicle_id}: Communication sockets initialized")
                return True
            except Exception as e:
                self.logger.error(f"Vehicle {self.vehicle_id}: Communication initialization failed: {e}")
                return False
        return False

    def handle_communication(self):
        """
        Handle non-blocking communication with direct processing.
        """
        try:
            # Check for incoming messages
            if self.comm is None:
                # Log communication handler not available (rate limited)
                if time.time() - self._last_comm_error_log > 10.0:
                    self.logger.error(f"Vehicle {self.vehicle_id}: Communication handler not available")
                    self._last_comm_error_log = time.time()
                return

            received_data = self.comm.receive_state_non_blocking()
            if received_data:
                self._process_communication_data(received_data)
            else:
                # Only log this occasionally to avoid spam
                if time.time() - self._last_no_data_log > 1.0:
                    self.logger.debug(f"Vehicle {self.vehicle_id}: No data received from peers")
                    self._last_no_data_log = time.time()

        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Communication error: {e}")

    def _process_communication_data(self, received_data: dict):
        """Process received communication data."""
        try:
            # Handle different message types
            msg_type = received_data.get('type', received_data.get('msg_type', 'vehicle_state'))

            if msg_type == 'fleet_estimates':
                self._handle_fleet_estimates(received_data)
            else:
                self._handle_vehicle_state(received_data)

        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Error processing communication data: {e}")

    def _handle_fleet_estimates(self, fleet_message: dict):
        """Handle fleet estimates from distributed observer."""
        try:
            sender_id = fleet_message.get('sender_id', fleet_message.get('vehicle_id', -1))
            estimates = fleet_message.get('estimates', {})
            message_timestamp = fleet_message.get('timestamp', time.time())
            seq = fleet_message.get('seq', -1)

            # Log fleet estimates
            est_data = {}
            for veh_id, est in estimates.items():
                pos = est.get('pos', [0, 0])
                vel = est.get('vel', 0.0)
                est_data[f"V{veh_id}"] = f"Pos=({pos[0]:.4f},{pos[1]:.4f}) Vel={vel:.4f}"

            self.logger.info(f"RECV_FLEET From=V{sender_id} Seq={seq} T={message_timestamp:.3f} {est_data}")

            # Update external fleet estimates
            self.external_fleet_estimates[sender_id] = {
                'timestamp': message_timestamp,
                'estimates': estimates
            }

            # Add fleet estimates to observer if available
            if self.observer is not None:
                success = self.observer.add_received_state_fleet(
                    sender_id=sender_id,
                    fleet_estimates=estimates,
                    timestamp=message_timestamp
                )
                if success:
                    self.logger.debug(f"Vehicle {self.vehicle_id}: Added fleet estimates to observer from sender {sender_id}")

        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Error processing fleet estimates: {e}")

    def _handle_vehicle_state(self, received_state: dict):
        """Handle individual vehicle state."""
        try:
            # Extract sender information
            sender_id = received_state.get('vehicle_id', received_state.get('id', received_state.get('sender_id', -1)))
            seq = received_state.get('seq', -1)
            timestamp = received_state.get('timestamp', time.time())

            # Extract state data
            pos = received_state.get('pos', received_state.get('position', [0, 0, 0]))
            rot = received_state.get('rot', received_state.get('rotation', [0, 0, 0]))
            vel = received_state.get('v', received_state.get('vel', received_state.get('velocity', 0.0)))
            control = received_state.get('ctrl_u', received_state.get('control_input', [0.0, 0.0]))

            # Log received state
            self.logger.info(f"STATE_RECV From=V{sender_id} Seq={seq} T={timestamp:.3f} "
                            f"Pos=({pos[0]:.4f},{pos[1]:.4f}) Rot={rot[2]:.4f} Vel={vel:.4f} "
                            f"Control=({control[0]:.3f},{control[1]:.3f})")

            # Add to state queue
            success = self.state_queue.add_state(received_state, self.gps_sync)

            if success:
                # Update leader_state for fallback (preserve original structure)
                if sender_id == self.following_target and 'pos' in received_state:
                    self.leader_state = received_state.copy()

                # Feed received state to distributed observer
                if self.observer is not None and sender_id >= 0:
                    try:
                        if len(pos) >= 2:
                            state_array = [
                                float(pos[0]),  # x
                                float(pos[1]),  # y
                                float(rot[2]) if len(rot) > 2 else 0.0,  # theta (yaw)
                                float(vel)      # velocity
                            ]

                            control_array = [float(control[0]), float(control[1])] if len(control) >= 2 else [0.0, 0.0]

                            self.observer.add_received_state(
                                sender_id=sender_id,
                                state=state_array,
                                control=control_array,
                                timestamp=timestamp
                            )
                    except Exception as obs_error:
                        self.logger.warning(f"Vehicle {self.vehicle_id}: Error feeding state to observer: {obs_error}")

                # Evaluate trust for this vehicle if enabled
                if self.trust_evaluator and sender_id != self.vehicle_id:
                    self._evaluate_trust_for_received_state(sender_id, received_state)

        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Error handling vehicle state: {e}")

    def _evaluate_trust_for_received_state(self, sender_id: int, received_state: dict):
        """Evaluate trust for a received state."""
        try:
            if not hasattr(self, 'gps_component') or not self.gps_component:
                return

            # Get our current state for comparison
            our_state = self.gps_component.get_current_state()

            # Get relative distance to the sender vehicle for trust evaluation
            relative_distance = self._get_relative_distance_to_vehicle(sender_id)

            # Evaluate trust with relative distance information
            trust_score = self.trust_evaluator.evaluate_trust_for_received_state(
                sender_id, received_state, our_state, self.logger, relative_distance=relative_distance
            )

            # Apply trust-based control if this is our following target
            if trust_score is not None and self.following_target == sender_id:
                base_distance = self.config.get('distance_between_cars', 8.0)
                adjusted_distance = self.trust_evaluator.apply_trust_based_control(
                    sender_id, base_distance, self.logger
                )

                # Apply adjustment to controller if supported
                if (hasattr(self, 'control_component') and self.control_component and
                    self.control_component.follower_controller and
                    hasattr(self.control_component.follower_controller, 'set_following_distance')):
                    self.control_component.follower_controller.set_following_distance(adjusted_distance)

        except Exception as e:
            self.logger.warning(f"Vehicle {self.vehicle_id}: Error in trust evaluation: {e}")

    def broadcast_own_state(self, current_state: Dict[str, Any]):
        """Broadcast this vehicle's current state to all other vehicles."""
        if self.comm is None:
            return

        try:
            # Add necessary fields for network transmission
            broadcast_state = {
                'vehicle_id': self.vehicle_id,
                'position': [float(x) for x in current_state['position']],
                'rotation': [float(r) for r in current_state['rotation']],
                'velocity': float(current_state['velocity']),
                'timestamp': time.time(),
                'control_input': [float(x) for x in self._get_current_control_input()]
            }

            # Broadcast state
            self.comm.send_state_broadcast(broadcast_state)

            # Debug logging
            self.logger.debug(
                f"Vehicle {self.vehicle_id}: Broadcasted state - pos={broadcast_state['position']}, "
                f"vel={broadcast_state['velocity']:.3f}"
            )

        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Failed to broadcast own state: {e}")

    def broadcast_fleet_estimates(self):
        """Broadcast fleet state estimates from distributed observer."""
        if self.comm is None or not hasattr(self, 'fleet_state_estimates') or not self.fleet_state_estimates:
            return

        try:
            current_time = time.time()

            # Build message
            fleet_message = self._build_fleet_estimates_message(current_time)
            if not fleet_message:
                return

            # Broadcast fleet estimates
            success = self.comm.send_fleet_estimates_broadcast(fleet_message)

            if success:
                self.logger.debug(f"Vehicle {self.vehicle_id}: Broadcasted fleet estimates for {len(self.fleet_state_estimates)} vehicles")

        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Fleet broadcast error: {e}")

    def _build_fleet_estimates_message(self, timestamp: float) -> Optional[Dict[str, Any]]:
        """Create the fleet estimates message."""
        if not self.fleet_state_estimates:
            return None

        msg = {
            'msg_type': 'fleet_estimates',
            'sender_id': self.vehicle_id,
            'timestamp': timestamp,
            'fleet_size': len(self.fleet_state_estimates),
            'estimates': {}
        }

        for vid, state in self.fleet_state_estimates.items():
            msg['estimates'][vid] = {
                'pos': state['position'][:2],
                'rot': state['rotation'],
                'vel': state['velocity'],
                'timestamp': state['timestamp']
            }

        return msg

    def _get_relative_distance_to_vehicle(self, target_vehicle_id: int) -> Optional[float]:
        """
        Get relative distance to a specific vehicle using CamLidarFusion.

        Args:
            target_vehicle_id: ID of the target vehicle

        Returns:
            Relative distance in meters, or None if not available
        """
        if self.cam_lidar_fusion is None:
            return None

        try:
            # Get vehicle-specific distances from CamLidarFusion
            vehicle_distances = self.cam_lidar_fusion.get_vehicle_distances()

            if not vehicle_distances:
                return None

            # Look for the target vehicle in the detected vehicles
            # Note: CamLidarFusion may not know vehicle IDs, so we take the closest detection
            # In a more sophisticated implementation, you might need to correlate detections with known vehicle positions

            # For now, return the distance to the closest detected vehicle
            # This assumes the closest vehicle is the one we're interested in
            if vehicle_distances:
                closest_vehicle = min(vehicle_distances, key=lambda v: v.get('distance', float('inf')))
                distance = closest_vehicle.get('distance')
                if distance is not None and distance > 0:
                    self.logger.debug(f"Vehicle {self.vehicle_id}: Relative distance to target {target_vehicle_id}: {distance:.2f}m")
                    return distance

            return None

        except Exception as e:
            self.logger.warning(f"Vehicle {self.vehicle_id}: Error getting relative distance to vehicle {target_vehicle_id}: {e}")
            return None

    def set_fleet_vehicle_estimate(self, vehicle_idx: int, vehicle_state, timestamp: float):
        """Store per-vehicle distributed observer estimate."""
        if not hasattr(self, 'fleet_state_estimates'):
            self.fleet_state_estimates = {}

        try:
            self.fleet_state_estimates[vehicle_idx] = {
                'position': [float(vehicle_state[0]), float(vehicle_state[1]), 0.0],
                'rotation': [0.0, 0.0, float(vehicle_state[2])],
                'velocity': float(vehicle_state[3]),
                'timestamp': timestamp
            }
        except Exception:
            pass  # Silent fail to avoid impacting real-time loop

    def cleanup(self):
        """Clean up communication component resources."""
        try:
            if self.comm:
                self.comm.cleanup()
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Communication cleanup error: {e}")