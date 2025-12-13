"""
V2V Manager - Complete Vehicle-to-Vehicle Communication Management System
Handles all V2V functionality including communication initialization,
broadcasting logic, message routing, and high-level control operations
"""
import time
import threading
from typing import Dict, List, Optional, Callable
from queue import Queue, Empty
from collections import defaultdict, deque
from dataclasses import dataclass
from enum import Enum
import numpy as np


from V2V.v2v_communication import V2VCommunication, V2VMessage, MessageType


class V2VDataType(Enum):
    """Types of V2V data"""
    LOCAL_STATE = "local_state"
    FLEET_STATE = "fleet_state"
    INTENT = "intent"
    WARNING = "warning"
    HEARTBEAT = "heartbeat"


@dataclass
class V2VBroadcastConfig:
    """Configuration for V2V broadcasting"""
    local_state_frequency: float = 20.0  # Hz - High frequency for local states
    fleet_state_frequency: float = 5.0   # Hz - Lower frequency for fleet states
    heartbeat_frequency: float = 1.0     # Hz - Very low frequency for heartbeats
    max_queue_size: int = 100
    state_timeout: float = 2.0           # Seconds before state is considered stale


class V2VManager:
    """
    Complete V2V Manager that handles all V2V functionality including:
    - V2V communication initialization and management
    - Broadcasting logic and message processing
    - High-level V2V control operations (activate, disable)
    - Status monitoring and reporting
    """
    
    def __init__(self, vehicle_id: int, vehicle_logger, config: Optional[V2VBroadcastConfig] = None, 
                 vehicle_observer=None, base_port: int = 8000, status_callback: Optional[Callable] = None,
                 vehicle_logic=None):
        self.vehicle_id = vehicle_id
        self.vehicle_logger = vehicle_logger
        self.logger = vehicle_logger.logger
        self.config = config or V2VBroadcastConfig()
        self.vehicle_observer = vehicle_observer
        self.status_callback = status_callback
        self.vehicle_logic = vehicle_logic  # Reference to main vehicle logic for Ground Station reporting
        
        # Prepare send intervals from config for V2VCommunication (convert to nanoseconds)
        send_intervals = {
            'local_state': int((1.0 / self.config.local_state_frequency) * 1e9),  # Convert Hz to nanoseconds
            'fleet_state': int((1.0 / self.config.fleet_state_frequency) * 1e9),
            'heartbeat': int((1.0 / self.config.heartbeat_frequency) * 1e9),
        }
        
        # Initialize V2V Communication system internally with configured intervals
        self.v2v_communication = V2VCommunication(
            vehicle_id=vehicle_id,
            logger=self.logger,
            base_port=base_port,
            status_callback=self._handle_v2v_status_change,
            send_intervals=send_intervals
        )
        
        # Separate queues for different data types
        self.local_state_queue = Queue(maxsize=self.config.max_queue_size)
        self.fleet_state_queue = Queue(maxsize=self.config.max_queue_size)
        self.intent_queue = Queue(maxsize=self.config.max_queue_size)
        self.warning_queue = Queue(maxsize=self.config.max_queue_size)
        
        # Received data storage with timestamps
        self.received_local_states = defaultdict(lambda: deque(maxlen=50))  # vehicle_id -> deque of (timestamp, data)
        self.received_fleet_states = defaultdict(lambda: deque(maxlen=20))  # vehicle_id -> deque of (timestamp, data)
        self.received_intents = defaultdict(lambda: deque(maxlen=10))
        self.received_warnings = defaultdict(lambda: deque(maxlen=10))
        
        # Platoon formation mapping (position -> vehicle_id)
        # This will be updated when platoon formation commands are received
        self.position_to_vehicle_id_map = {}  # {position: vehicle_id}
        
        # Thread safety
        self._lock = threading.RLock()
        
        # Logging counters for periodic data structure logging
        self._local_state_log_counter = 0
        self._fleet_state_log_counter = 0
        self._log_data_structure_interval = 500  # Log every 500 messages
        
        # Statistics
        self.stats = {
            'local_broadcasts': 0,
            'fleet_broadcasts': 0,
            'messages_received': 0,
            'messages_processed': 0
        }
        
        # Setup message handlers
        self._setup_message_handlers()
        
        if self.logger:
            self.logger.info(f"V2VManager initialized for vehicle {vehicle_id}")
    
    def _setup_message_handlers(self):
        """Setup handlers for different message types"""
        self.v2v_communication.register_message_handler(
            MessageType.TELEMETRY.value, self._handle_telemetry_message
        )
        self.v2v_communication.register_message_handler(
            "local_state", self._handle_local_state_message
        )
        self.v2v_communication.register_message_handler(
            "fleet_state", self._handle_fleet_state_message
        )
        self.v2v_communication.register_message_handler(
            MessageType.INTENT.value, self._handle_intent_message
        )
        self.v2v_communication.register_message_handler(
            MessageType.WARNING.value, self._handle_warning_message
        )
    
    def update_broadcast(self) -> bool:
        """
        Main update method - attempts broadcasting.
        V2VCommunication handles rate-limiting internally based on configured intervals.
        Returns True if any broadcast was sent.
        """
        broadcast_sent = False
        
        try:
            # Attempt to broadcast local state (V2VCommunication will rate-limit)
            if self._broadcast_local_state():
                broadcast_sent = True
            
            # Attempt to broadcast fleet state (V2VCommunication will rate-limit)
            if self._broadcast_fleet_state():
                broadcast_sent = True
            
            # Attempt to broadcast heartbeat (V2VCommunication will rate-limit)
            if self._broadcast_heartbeat():
                broadcast_sent = True
            
            # Process received messages
            self._process_received_messages()
            
            return broadcast_sent
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"V2VManager broadcast update error: {e}")
            return False
    
    def _broadcast_local_state(self) -> bool:
        """Broadcast local state - rate-limited by V2VCommunication"""
        try:
            if not self.vehicle_observer:
                if self.logger:
                    self.logger.debug(f"Vehicle {self.vehicle_id}: No vehicle observer for local state broadcast")
                return False
            
            local_state = self.vehicle_observer.get_local_state_for_broadcast()
            
            # # Periodically log what we're broadcasting
            # if self.stats['local_broadcasts'] % 100 == 0 and self.logger:  # Every 100 broadcasts (every 5 seconds at 20Hz)
            #     self.logger.info(f"V2VManager: Broadcasting local state #{self.stats['local_broadcasts']} from vehicle {self.vehicle_id}:")
            #     self.logger.info(f"  Data: {local_state}")
            
            success = self.v2v_communication.send_message(
                message_type="local_state",
                data=local_state
            )
            
            if success:
                with self._lock:
                    self.stats['local_broadcasts'] += 1
                if self.logger:
                    self.logger.debug(f"Vehicle {self.vehicle_id}: Local state broadcast #{self.stats['local_broadcasts']} sent")
            else:
                if self.logger:
                    self.logger.debug(f"Vehicle {self.vehicle_id}: Local state broadcast failed (rate limited or error)")
            
            return success
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Local state broadcast error: {e}")
            return False
    
    def _broadcast_fleet_state(self) -> bool:
        """Broadcast fleet state - rate-limited by V2VCommunication"""
        try:
            if not self.vehicle_observer:
                if self.logger:
                    self.logger.debug(f"Vehicle {self.vehicle_id}: No vehicle observer for fleet state broadcast")
                return False
            
            fleet_state = self.vehicle_observer.get_fleet_state_for_broadcast()
            
            # # Periodically log what we're broadcasting
            # self._fleet_state_log_counter += 1
            # if self._fleet_state_log_counter % 25 == 0 and self.logger:  # Every 25 broadcasts (every 5 seconds at 5Hz)
            #     self.logger.info(f"V2VManager: Broadcasting fleet state #{self._fleet_state_log_counter} from vehicle {self.vehicle_id}:")
            #     if 'fleet_states' in fleet_state:
            #         for vid, vstate in fleet_state['fleet_states'].items():
            #             self.logger.info(
            #                 f"  vehicle_{vid}: x={vstate['x']:.3f}, y={vstate['y']:.3f}, "
            #                 f"theta={vstate['theta']:.3f}, v={vstate['velocity']:.3f}, conf={vstate['confidence']:.2f}"
            #             )
            
            success = self.v2v_communication.send_message(
                message_type="fleet_state",
                data=fleet_state
            )
            
            if success:
                with self._lock:
                    self.stats['fleet_broadcasts'] += 1
                if self.logger:
                    self.logger.debug(f"Vehicle {self.vehicle_id}: Fleet state broadcast #{self.stats['fleet_broadcasts']} sent")
            else:
                if self.logger:
                    self.logger.debug(f"Vehicle {self.vehicle_id}: Fleet state broadcast failed (rate limited or error)")
            
            return success
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Fleet state broadcast error: {e}")
            return False
    
    def _broadcast_heartbeat(self) -> bool:
        """Broadcast heartbeat message"""
        try:
            heartbeat_data = {
                'vehicle_id': self.vehicle_id,
                'timestamp': time.time(),
                'status': 'active'
            }
            
            return self.v2v_communication.send_message(
                message_type=MessageType.HEARTBEAT.value,
                data=heartbeat_data
            )
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Heartbeat broadcast error: {e}")
            return False
    
    def _process_received_messages(self):
        """Process messages from the low-level communication layer"""
        try:
            messages = self.v2v_communication.get_messages(max_count=20)
            
            for message in messages:
                with self._lock:
                    self.stats['messages_received'] += 1
                
                # Messages are automatically routed to handlers via registered callbacks
                # No manual processing needed here
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"Message processing error: {e}")
    
    # Message handlers for different data types
    def _handle_telemetry_message(self, message: V2VMessage):
        """Handle legacy telemetry messages (for backward compatibility)"""
        try:
            self._add_to_queue(self.local_state_queue, message.data, message.sender_id)
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Telemetry message handling error: {e}")
    
    def _handle_local_state_message(self, message: V2VMessage):
        """Handle local state messages with known data structure from get_local_state_for_broadcast()
        
        Expected data structure:
        {
            'vehicle_id': int,
            'x': float, 'y': float, 'theta': float, 'velocity': float,
            'acceleration': float,  # Optional for future use
            'control_input': {'steering': float, 'throttle': float},  # Optional for future use
            'source': 'local_sensors'
        }
        """
        try:
            sender_id = message.sender_id
            data = message.data
            send_time_ns = message.send_time_ns
            
            # Validate required fields for local state (acceleration and control_input are optional)
            required_fields = ['vehicle_id', 'x', 'y', 'theta', 'velocity']
            missing_fields = [field for field in required_fields if field not in data]
            
            if missing_fields:
                if self.logger:
                    self.logger.warning(f"V2VManager: Local state message from vehicle {sender_id} missing fields: {missing_fields}")
                return
            
            # # Validate data types and ranges
            # if not self._validate_local_state_data(data, sender_id):
            #     return
            
            if self.logger:
                self.logger.debug(f"Vehicle {self.vehicle_id}: Received local state from vehicle {sender_id}")
                
                # Periodically log the data structure for debugging
                self._local_state_log_counter += 1
                if self._local_state_log_counter % self._log_data_structure_interval == 0:
                    self.logger.info(f"V2VManager: Local state data structure from vehicle {sender_id}:")
                    self.logger.info(f"  vehicle_id: {data.get('vehicle_id')} (int)")
                    self.logger.info(f"  position: ({data.get('x'):.3f}, {data.get('y'):.3f}) (float)")
                    self.logger.info(f"  theta: {data.get('theta'):.3f} rad (float)")
                    self.logger.info(f"  velocity: {data.get('velocity'):.3f} m/s (float)")
                    self.logger.info(f"  acceleration: {data.get('acceleration', 0.0):.3f} m/s² (float)")
                    control_input = data.get('control_input', {})
                    if isinstance(control_input, dict):
                        self.logger.info(f"  control_input: steering={control_input.get('steering', 0.0):.3f}, throttle={control_input.get('throttle', 0.0):.3f}")
                    self.logger.info(f"  source: {data.get('source', 'unknown')} (str)")
                    self.logger.info(f"  send_time_ns: {send_time_ns} (nanoseconds)")
            
            with self._lock:
                # Add to received local states with send time in nanoseconds
                self.received_local_states[sender_id].append((send_time_ns, data))
                
                # Log received local estimation to dedicated CSV file
                if hasattr(self.vehicle_logger, 'log_local_estimation'):
                    try:
                        state_dict = {
                            'x': data.get('x', 0.0),
                            'y': data.get('y', 0.0),
                            'theta': data.get('theta', 0.0),
                            'v': data.get('velocity', 0.0),
                            'confidence': data.get('confidence', 1.0),  # Default confidence if not present
                            'acceleration': data.get('acceleration', 0.0),  # Include acceleration
                            'control_input': data.get('control_input', {})  # Include control inputs
                        }
                        self.vehicle_logger.log_local_estimation(
                            sender_id=sender_id,
                            state=state_dict,
                            source=data.get('source', 'local_sensors'),
                            seq_id=message.seq_id,
                            send_time_ns=send_time_ns
                        )
                    except Exception as e:
                        if self.logger:
                            self.logger.logger.warning(f"Failed to log local estimation: {e}")
                
                # Add to VehicleObserver if available
                if (self.vehicle_observer and 
                    all(key in data for key in ['x', 'y', 'theta', 'velocity'])):
                    
                    try:
                        # Create 5D state vector [x, y, theta, v, a]
                        state_vector = np.array([
                            data['x'], 
                            data['y'], 
                            data['theta'], 
                            data['velocity'],
                            data.get('acceleration', 0.0)  # Include acceleration, default to 0 if not present
                        ])
                        self.vehicle_observer.add_received_state(sender_id, state_vector, send_time_ns)
                    except Exception as e:
                        if self.logger:
                            self.logger.warning(f"Failed to add state to VehicleObserver: {e}")
            
            # Add to queue for other consumers
            self._add_to_queue(self.local_state_queue, data, sender_id)
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Local state message handling error: {e}")
    
    def _handle_fleet_state_message(self, message: V2VMessage):
        """Handle fleet state messages with known data structure from get_fleet_state_for_broadcast()
        
        Expected data structure:
        {
            'sender_id': int,
            'fleet_states': {
                vehicle_id: {
                    'x': float, 'y': float, 'theta': float, 'velocity': float,
                    'confidence': float
                },
                ...
            },
            'source': 'fleet_consensus'
        }
        Note: Timestamp comes from message.send_time_ns (not in data payload)
        """
        try:
            sender_id = message.sender_id
            data = message.data
            send_time_ns = message.send_time_ns
            
            # Validate required fields for fleet state 
            required_fields = ['sender_id', 'fleet_states']
            missing_fields = [field for field in required_fields if field not in data]
            
            if missing_fields:
                if self.logger:
                    self.logger.warning(f"V2VManager: Fleet state message from vehicle {sender_id} missing fields: {missing_fields}")
                return
            
            # Validate fleet states structure
            fleet_states = data.get('fleet_states', {})
            if not isinstance(fleet_states, dict):
                if self.logger:
                    self.logger.warning(f"V2VManager: Invalid fleet_states format from vehicle {sender_id}")
                return
            
            # # Validate individual vehicle states in fleet data
            # valid_fleet_count = 0
            # for vehicle_id, vehicle_state in fleet_states.items():
            #     if self._validate_fleet_vehicle_state(vehicle_state, vehicle_id, sender_id):
            #         valid_fleet_count += 1
            
            # if valid_fleet_count == 0:
            #     if self.logger:
            #         self.logger.warning(f"V2VManager: No valid vehicle states in fleet message from vehicle {sender_id}")
            #     return
            
            with self._lock:
                # Add to received fleet states with send time in nanoseconds
                self.received_fleet_states[sender_id].append((send_time_ns, data))
                
                # Log received fleet estimation to dedicated CSV file
                if hasattr(self.vehicle_logger, 'log_fleet_estimation'):
                    try:
                        self.vehicle_logger.log_fleet_estimation(
                            sender_id=sender_id,
                            fleet_states=fleet_states,
                            source=data.get('source', 'unknown'),
                            seq_id=message.seq_id,
                            send_time_ns=send_time_ns
                        )
                    except Exception as e:
                        if self.logger:
                            self.logger.logger.warning(f"Failed to log fleet estimation: {e}")
                
                # Periodically log the fleet state data structure for debugging
                self._fleet_state_log_counter += 1
                if self._fleet_state_log_counter % self._log_data_structure_interval == 0:
                    if self.logger:
                        self.logger.info(f"V2VManager: Fleet state data structure from vehicle {sender_id}:")
                        self.logger.info(f"  sender_id: {data.get('sender_id')} (int)")
                        self.logger.info(f"  fleet_states: dict with {len(fleet_states)} vehicles")
                        for vid, vstate in list(fleet_states.items())[:3]:  # Show first 3 vehicles
                            self.logger.info(f"    vehicle_{vid}: x={vstate.get('x', 0):.3f}, y={vstate.get('y', 0):.3f}, theta={vstate.get('theta', 0):.3f}, v={vstate.get('velocity', 0):.3f}, conf={vstate.get('confidence', 0):.2f}")
                        if len(fleet_states) > 3:
                            self.logger.info(f"    ... and {len(fleet_states) - 3} more vehicles")
                        self.logger.info(f"  source: {data.get('source', 'unknown')} (str)")
                        self.logger.info(f"  timestamp: {data.get('timestamp'):.3f} (float)")
            
            # Pass entire fleet estimates to vehicle observer for processing
            if self.vehicle_observer is not None:
                try:
                    success = self.vehicle_observer.add_received_fleet_state(
                        sender_id=sender_id,
                        fleet_estimates=fleet_states,
                        timestamp_ns=send_time_ns
                    )
                    
                    if success and self.logger:
                        self.logger.debug(
                            f"V2VManager: Processed fleet estimates from vehicle {sender_id} "
                            f"({len(fleet_states)} vehicles) to fleet estimator"
                        )
                    elif not success and self.logger:
                        self.logger.debug(
                            f"V2VManager: No valid fleet states added from vehicle {sender_id}"
                        )
                
                except Exception as e:
                    if self.logger:
                        self.logger.warning(f"V2VManager: Failed to process fleet states to estimator: {e}")
            
            # Add to queue for other consumers
            self._add_to_queue(self.fleet_state_queue, data, sender_id)
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Fleet state message handling error: {e}")
    
    def _validate_local_state_data(self, data: dict, sender_id: int) -> bool:
        """Validate local state data integrity and ranges"""
        try:
            # Check vehicle_id matches sender
            if data.get('vehicle_id') != sender_id:
                if self.logger:
                    self.logger.warning(f"V2VManager: Vehicle ID mismatch in local state: expected {sender_id}, got {data.get('vehicle_id')}")
                return False
            
            # Validate position values are reasonable (not NaN or infinite)
            position_fields = ['x', 'y', 'theta', 'velocity']
            for field in position_fields:
                value = data.get(field)
                if not isinstance(value, (int, float)) or not (-1000 <= value <= 1000):  # Reasonable range check
                    if self.logger:
                        self.logger.warning(f"V2VManager: Invalid {field} value in local state from vehicle {sender_id}: {value}")
                    return False
                
                # Check for NaN or infinite values
                if isinstance(value, float) and (value != value or abs(value) == float('inf')):
                    if self.logger:
                        self.logger.warning(f"V2VManager: NaN/Inf {field} value in local state from vehicle {sender_id}: {value}")
                    return False
            
            # Validate timestamp is recent (within last 10 seconds)
            msg_timestamp = data.get('timestamp', 0)
            current_time = time.time()
            if abs(current_time - msg_timestamp) > 10.0:
                if self.logger:
                    self.logger.debug(f"V2VManager: Old timestamp in local state from vehicle {sender_id}: {current_time - msg_timestamp:.2f}s ago")
                # Don't reject old data, just log it
            
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.warning(f"V2VManager: Local state validation error for vehicle {sender_id}: {e}")
            return False
    
    def _validate_fleet_vehicle_state(self, vehicle_state: dict, vehicle_id: int, sender_id: int) -> bool:
        """Validate individual vehicle state within fleet data"""
        try:
            if not isinstance(vehicle_state, dict):
                return False
            
            required_fields = ['x', 'y', 'theta', 'velocity']
            for field in required_fields:
                value = vehicle_state.get(field)
                if not isinstance(value, (int, float)):
                    return False
                
                # Check for reasonable ranges and NaN/Inf
                if not (-1000 <= value <= 1000) or (isinstance(value, float) and (value != value or abs(value) == float('inf'))):
                    if self.logger:
                        self.logger.debug(f"V2VManager: Invalid {field} for vehicle {vehicle_id} in fleet state from {sender_id}: {value}")
                    return False
            
            # Validate confidence if present
            confidence = vehicle_state.get('confidence')
            if confidence is not None and (not isinstance(confidence, (int, float)) or not (0.0 <= confidence <= 1.0)):
                if self.logger:
                    self.logger.debug(f"V2VManager: Invalid confidence for vehicle {vehicle_id} in fleet state from {sender_id}: {confidence}")
                return False
            
            return True
            
        except Exception:
            return False
    
    def _handle_intent_message(self, message: V2VMessage):
        """Handle intent messages"""
        try:
            self._add_to_queue(self.intent_queue, message.data, message.sender_id)
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Intent message handling error: {e}")
    
    def _handle_warning_message(self, message: V2VMessage):
        """Handle warning messages"""
        try:
            self._add_to_queue(self.warning_queue, message.data, message.sender_id)
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Warning message handling error: {e}")
    
    def _add_to_queue(self, queue: Queue, data: dict, sender_id: int):
        """Add data to queue with sender information"""
        try:
            queue_data = {
                'sender_id': sender_id,
                'timestamp': time.time(),
                'data': data
            }
            
            try:
                queue.put_nowait(queue_data)
            except:
                # Queue full, remove oldest and add new
                try:
                    queue.get_nowait()
                    queue.put_nowait(queue_data)
                except:
                    pass  # Queue operations failed
                    
        except Exception as e:
            if self.logger:
                self.logger.error(f"Queue add error: {e}")
    
    # Public interface methods
    def get_local_states(self, max_count: int = 10) -> List[dict]:
        """Get received local states"""
        return self._get_from_queue(self.local_state_queue, max_count)
    
    def get_fleet_states(self, max_count: int = 5) -> List[dict]:
        """Get received fleet states"""
        return self._get_from_queue(self.fleet_state_queue, max_count)
    
    def get_intents(self, max_count: int = 10) -> List[dict]:
        """Get received intents"""
        return self._get_from_queue(self.intent_queue, max_count)
    
    def get_warnings(self, max_count: int = 10) -> List[dict]:
        """Get received warnings"""
        return self._get_from_queue(self.warning_queue, max_count)
    
    def _get_from_queue(self, queue: Queue, max_count: int) -> List[dict]:
        """Get messages from queue"""
        messages = []
        for _ in range(max_count):
            try:
                messages.append(queue.get_nowait())
            except Empty:
                break
        return messages
    
    def get_latest_local_state(self, vehicle_id: int) -> Optional[dict]:
        """Get latest local state from a specific vehicle"""
        with self._lock:
            if vehicle_id in self.received_local_states:
                states = self.received_local_states[vehicle_id]
                if states:
                    return states[-1][1]  # Return data part of (timestamp, data)
        return None
    
    def get_latest_fleet_state(self, vehicle_id: int) -> Optional[dict]:
        """Get latest fleet state from a specific vehicle"""
        with self._lock:
            if vehicle_id in self.received_fleet_states:
                states = self.received_fleet_states[vehicle_id]
                if states:
                    return states[-1][1]  # Return data part of (timestamp, data)
        return None
    
    def get_direct_leader_data(self, current_vehicle_position: int) -> Optional[dict]:
        """Get direct leader's local state data for the given vehicle position in platoon
        
        Args:
            current_vehicle_position: The position of the current vehicle in platoon (1=leader, 2=first follower, etc.)
            
        Returns:
            Dictionary containing direct leader's state data or None if no leader or no data
        """
        try:
            # Vehicle at position 1 is the leader (no leader above)
            if current_vehicle_position <= 1:
                if self.logger:
                    self.logger.debug(f"Vehicle at position {current_vehicle_position} is lead vehicle (no leader)")
                return None
            
            # Calculate direct leader position (position - 1)
            leader_position = current_vehicle_position - 1
            
            
            # Find the vehicle_id that has the leader_position
            # Use the position-to-vehicle_id mapping if available
            if leader_position in self.position_to_vehicle_id_map:
                leader_vehicle_id = self.position_to_vehicle_id_map[leader_position]
            else:
                # Fallback: assume position maps directly to vehicle_id
                leader_vehicle_id = leader_position
                if self.logger:
                    self.logger.debug(f"V2VManager: No position mapping found, using position {leader_position} as vehicle_id")
            
            # Get leader data using the leader's vehicle_id
            leader_state = self.get_latest_local_state(leader_vehicle_id)
            
            if leader_state:
                if self.logger:
                    self.logger.debug(
                        f"V2VManager: Got direct leader (pos {leader_position}, id {leader_vehicle_id}) "
                        f"state for vehicle position {current_vehicle_position}: "
                        f"x={leader_state.get('x', 'N/A'):.2f}, y={leader_state.get('y', 'N/A'):.2f}"
                    )
                return leader_state
            else:
                if self.logger:
                    self.logger.debug(
                        f"V2VManager: No V2V data for direct leader (pos {leader_position}, id {leader_vehicle_id})"
                    )
                return None
                
        except Exception as e:
            if self.logger:
                self.logger.warning(f"V2VManager: Error getting direct leader data: {e}")
            return None
    
    def get_my_direct_leader_data(self) -> Optional[dict]:
        """Get direct leader's local state data for this vehicle's position
        
        Returns:
            Dictionary containing direct leader's state data or None if no leader or no data
        """
        # Note: This requires vehicle_logic reference to get vehicle_position
        # For now, use vehicle_id as position (can be improved when vehicle_logic is properly referenced)
        return self.get_direct_leader_data(self.vehicle_id)
    
    def cleanup_old_data(self):
        """Clean up old received data"""
        current_time = time.time()
        timeout = self.config.state_timeout
        
        with self._lock:
            # Clean up old local states
            for vehicle_id in list(self.received_local_states.keys()):
                states = self.received_local_states[vehicle_id]
                # Remove states older than timeout
                while states and current_time - states[0][0] > timeout:
                    states.popleft()
                
                # Remove empty entries
                if not states:
                    del self.received_local_states[vehicle_id]
            
            # Clean up old fleet states
            for vehicle_id in list(self.received_fleet_states.keys()):
                states = self.received_fleet_states[vehicle_id]
                while states and current_time - states[0][0] > timeout:
                    states.popleft()
                
                if not states:
                    del self.received_fleet_states[vehicle_id]
    
    def send_intent(self, intention: str, parameters: dict) -> bool:
        """Send driving intent to other vehicles"""
        try:
            intent_data = {
                'vehicle_id': self.vehicle_id,
                'intention': intention,
                'parameters': parameters,
                'timestamp': time.time()
            }
            
            return self.v2v_communication.send_message(
                message_type=MessageType.INTENT.value,
                data=intent_data
            )
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Intent sending error: {e}")
            return False
    
    def send_warning(self, warning_type: str, urgency: str, data: dict) -> bool:
        """Send warning message to other vehicles"""
        try:
            warning_data = {
                'vehicle_id': self.vehicle_id,
                'warning_type': warning_type,
                'urgency': urgency,
                'data': data,
                'timestamp': time.time()
            }
            
            return self.v2v_communication.send_message(
                message_type=MessageType.WARNING.value,
                data=warning_data
            )
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Warning sending error: {e}")
            return False
    
    def get_statistics(self) -> dict:
        """Get V2VManager statistics"""
        with self._lock:
            stats = self.stats.copy()
            
            # Add queue sizes
            stats.update({
                'local_state_queue_size': self.local_state_queue.qsize(),
                'fleet_state_queue_size': self.fleet_state_queue.qsize(),
                'intent_queue_size': self.intent_queue.qsize(),
                'warning_queue_size': self.warning_queue.qsize(),
                'active_local_peers': len(self.received_local_states),
                'active_fleet_peers': len(self.received_fleet_states),
                'local_broadcast_rate': self.config.local_state_frequency,
                'fleet_broadcast_rate': self.config.fleet_state_frequency,
                'local_state_messages_received': self._local_state_log_counter,
                'fleet_state_messages_received': self._fleet_state_log_counter
            })
            
            return stats
    
    def is_active(self) -> bool:
        """Check if V2V manager is active"""
        return self.v2v_communication.is_active if self.v2v_communication else False
    
    def activate(self, peer_vehicles: List[int], peer_ips: List[str]) -> bool:
        """Activate V2V communication"""
        try:
            if not self.v2v_communication:
                if self.logger:
                    self.logger.error("V2VManager: No V2V communication instance available")
                return False
            
            success = self.v2v_communication.activate(peer_vehicles, peer_ips)
            
            if success and self.logger:
                fleet_size = len(peer_vehicles) + 1  # peers + self
                self.logger.info(f"V2VManager activated for vehicle {self.vehicle_id} "
                               f"with {len(peer_vehicles)} peers (fleet size: {fleet_size})")
                self.logger.info(f"Broadcast rates: Local={self.config.local_state_frequency}Hz, "
                               f"Fleet={self.config.fleet_state_frequency}Hz")
            
            return success
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"V2VManager activation error: {e}")
            return False
    
    def deactivate(self):
        """Deactivate V2V communication"""
        if self.v2v_communication:
            self.v2v_communication.deactivate()
    
    # ===== High-level V2V Control Methods =====
    
    def activate_v2v(self, peer_vehicles: List[int], peer_ips: List[str]) -> bool:
        """
        Activate V2V communication with specified peers
        This is the main entry point for V2V activation from external systems
        """
        try:
            if not peer_vehicles or not peer_ips:
                if self.logger:
                    self.logger.warning("V2VManager: Cannot activate V2V - no peers specified")
                return False
            
            if len(peer_vehicles) != len(peer_ips):
                if self.logger:
                    self.logger.error(f"V2VManager: Peer count mismatch - {len(peer_vehicles)} vehicles, {len(peer_ips)} IPs")
                return False
            
            if self.logger:
                self.logger.info(f"V2VManager: Activating V2V for vehicle {self.vehicle_id}")
                self.logger.info(f"V2VManager: Connecting to peers: {peer_vehicles}")
                self.logger.info(f"V2VManager: Peer IPs: {peer_ips}")
            
            # Reinitialize fleet estimation via vehicle_logic
            if self.vehicle_logic and hasattr(self.vehicle_logic, 'reinitialize_fleet_estimation'):
                self.vehicle_logic.reinitialize_fleet_estimation(peer_vehicles)
            
            # Activate the underlying V2V communication
            success = self.activate(peer_vehicles, peer_ips)
            
            if success:
                fleet_size = len(peer_vehicles) + 1
                if self.logger:
                    self.logger.info(f"V2VManager: V2V communication activated successfully")
                    self.logger.info(f"V2VManager: Broadcasting rates - Local: {self.config.local_state_frequency}Hz, Fleet: {self.config.fleet_state_frequency}Hz")
                
                # Report activation to Ground Station via vehicle_logic
                if self.vehicle_logic and hasattr(self.vehicle_logic, 'report_v2v_status_to_gs'):
                    self.vehicle_logic.report_v2v_status_to_gs({
                        'status': 'activated',
                        'peer_count': len(peer_vehicles),
                        'peer_vehicles': peer_vehicles,
                        'expected_peers': len([v for v in peer_vehicles if v != self.vehicle_id]),
                        'vehicle_id': self.vehicle_id,
                        'timestamp': time.time(),
                        'fleet_size': fleet_size,
                        'protocol': 'UDP-Manager'
                    })
                
                # Report activation to external callback
                if self.status_callback:
                    self.status_callback('v2v_activated', {
                        'peer_vehicles': peer_vehicles,
                        'peer_ips': peer_ips,
                        'fleet_size': fleet_size
                    })
            else:
                if self.logger:
                    self.logger.error(f"V2VManager: V2V communication activation failed")
            
            return success
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"V2VManager: V2V activation error - {e}")
            return False
    
    def disable_v2v(self) -> bool:
        """
        Disable V2V communication
        This is the main entry point for V2V deactivation from external systems
        """
        try:
            if not self.is_active():
                if self.logger:
                    self.logger.info("V2VManager: V2V already inactive")
                return True
            
            if self.logger:
                self.logger.info(f"V2VManager: Disabling V2V for vehicle {self.vehicle_id}")
            
            # Deactivate the underlying V2V communication
            self.deactivate()
            
            if self.logger:
                self.logger.info(f"V2VManager: V2V communication disabled successfully")
            
            # Report deactivation to Ground Station via vehicle_logic
            if self.vehicle_logic and hasattr(self.vehicle_logic, 'report_v2v_status_to_gs'):
                self.vehicle_logic.report_v2v_status_to_gs({
                    'status': 'disconnected',
                    'vehicle_id': self.vehicle_id,
                    'timestamp': time.time()
                })
            
            # Report deactivation to external callback
            if self.status_callback:
                self.status_callback('v2v_deactivated', {})
            
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"V2VManager: V2V disable error - {e}")
            return False
    
    def _handle_v2v_status_change(self, event_type: str, peer_id: int):
        """
        Handle immediate V2V status changes from the communication module
        Internal callback from V2VCommunication
        """
        try:
            if self.logger:
                if event_type == 'peer_connected':
                    self.logger.info(f"V2VManager: Vehicle {peer_id} connected to V2V network")
                elif event_type == 'peer_disconnected':
                    self.logger.warning(f"V2VManager: Vehicle {peer_id} disconnected from V2V network")
                elif event_type == 'peer_timeout':
                    self.logger.warning(f"V2VManager: Vehicle {peer_id} timed out")
                else:
                    self.logger.debug(f"V2VManager: V2V status change - {event_type} for vehicle {peer_id}")
            
            # Forward status change to external callback if available
            if self.status_callback:
                self.status_callback(event_type, peer_id)
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"V2VManager: Status change handling error - {e}")
    
    def get_connection_status(self) -> dict:
        """
        Get comprehensive V2V connection status
        """
        try:
            status = {
                'is_active': self.is_active(),
                'vehicle_id': self.vehicle_id,
                'connected_peers': [],
                'fleet_size': 0,
                'communication_stats': {},
                'broadcast_config': {
                    'local_state_frequency': self.config.local_state_frequency,
                    'fleet_state_frequency': self.config.fleet_state_frequency,
                    'heartbeat_frequency': self.config.heartbeat_frequency
                }
            }
            
            if self.is_active():
                connected_peers = self.v2v_communication.get_connected_peers()
                status['connected_peers'] = connected_peers
                status['fleet_size'] = len(connected_peers) + 1  # peers + self
                status['communication_stats'] = self.v2v_communication.get_statistics()
            
            return status
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"V2VManager: Status retrieval error - {e}")
            return {
                'is_active': False,
                'vehicle_id': self.vehicle_id,
                'error': str(e)
            }
    
    def get_status_summary(self) -> str:
        """
        Get human-readable V2V status summary
        """
        try:
            if not self.is_active():
                return f"V2V: Inactive (Vehicle {self.vehicle_id})"
            
            status = self.get_connection_status()
            fleet_size = status.get('fleet_size', 0)
            peer_count = fleet_size - 1
            
            stats = status.get('communication_stats', {})
            send_rate = stats.get('actual_rate_hz', 0.0)
            
            if peer_count > 0:
                return f"V2V: Active (Fleet: {fleet_size}, Rate: {send_rate:.1f}Hz)"
            else:
                return f"V2V: Active (No peers, Rate: {send_rate:.1f}Hz)"
                
        except Exception as e:
            return f"V2V: Error - {e}"
    
    def update_vehicle_observer(self, vehicle_observer):
        """
        Update the vehicle observer reference
        """
        self.vehicle_observer = vehicle_observer
    
    def update_vehicle_logic(self, vehicle_logic):
        """
        Update the vehicle logic reference for Ground Station reporting
        """
        self.vehicle_logic = vehicle_logic
    
    def update_platoon_formation(self, formation_map: dict):
        """
        Update the platoon formation mapping
        
        Args:
            formation_map: Dictionary mapping vehicle_id to position {vehicle_id: position}
        """
        try:
            # Convert vehicle_id -> position map to position -> vehicle_id map
            self.position_to_vehicle_id_map = {position: vehicle_id for vehicle_id, position in formation_map.items()}
            
            if self.logger:
                self.logger.info(f"V2VManager: Updated platoon formation mapping: {self.position_to_vehicle_id_map}")
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"V2VManager: Error updating platoon formation: {e}")
    
    def log_received_data_summary(self):
        """Log a summary of received data for debugging purposes"""
        try:
            with self._lock:
                if self.logger:
                    self.logger.info("=== V2V Received Data Summary ===")
                    
                    # Local states summary
                    local_count = sum(len(states) for states in self.received_local_states.values())
                    self.logger.info(f"Local states: {local_count} total from {len(self.received_local_states)} vehicles")
                    for vehicle_id, states in self.received_local_states.items():
                        if states:
                            latest_data = states[-1][1]
                            self.logger.info(f"  Vehicle {vehicle_id}: {len(states)} states, latest keys: {list(latest_data.keys())}")
                    
                    # Fleet states summary
                    fleet_count = sum(len(states) for states in self.received_fleet_states.values())
                    self.logger.info(f"Fleet states: {fleet_count} total from {len(self.received_fleet_states)} vehicles")
                    for vehicle_id, states in self.received_fleet_states.items():
                        if states:
                            latest_data = states[-1][1]
                            self.logger.info(f"  Vehicle {vehicle_id}: {len(states)} states, latest keys: {list(latest_data.keys())}")
                    
                    # Position mapping
                    if self.position_to_vehicle_id_map:
                        self.logger.info(f"Position mapping: {self.position_to_vehicle_id_map}")
                    else:
                        self.logger.info("Position mapping: Not configured")
                        
                    self.logger.info("=== End V2V Data Summary ===")
                    
        except Exception as e:
            if self.logger:
                self.logger.error(f"V2VManager: Error logging data summary: {e}")
