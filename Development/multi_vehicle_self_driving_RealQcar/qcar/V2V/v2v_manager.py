"""
V2V Manager - High-level Vehicle-to-Vehicle Communication Management
Handles broadcasting logic, message routing, and data processing
"""
import time
import threading
from typing import Dict, List, Optional, Callable
from queue import Queue, Empty
from collections import defaultdict, deque
from dataclasses import dataclass
from enum import Enum

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
    High-level V2V Manager that handles broadcasting logic and message processing.
    Separates high-level logic from low-level communication (V2VCommunication).
    """
    
    def __init__(self, vehicle_id: int, v2v_communication: V2VCommunication, 
                 vehicle_observer, logger, config: Optional[V2VBroadcastConfig] = None):
        self.vehicle_id = vehicle_id
        self.v2v_communication = v2v_communication
        self.vehicle_observer = vehicle_observer
        self.logger = logger
        self.config = config or V2VBroadcastConfig()
        
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
        
        # Broadcast timing control
        self._last_local_broadcast = 0.0
        self._last_fleet_broadcast = 0.0
        self._last_heartbeat = 0.0
        
        # Thread safety
        self._lock = threading.RLock()
        
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
        Main update method - handles broadcasting at different frequencies.
        Returns True if any broadcast was sent.
        """
        current_time = time.time()
        broadcast_sent = False
        
        try:
            # Broadcast local state at high frequency
            if self._should_broadcast_local_state(current_time):
                if self._broadcast_local_state():
                    self._last_local_broadcast = current_time
                    broadcast_sent = True
            
            # Broadcast fleet state at lower frequency
            if self._should_broadcast_fleet_state(current_time):
                if self._broadcast_fleet_state():
                    self._last_fleet_broadcast = current_time
                    broadcast_sent = True
            
            # Broadcast heartbeat at very low frequency
            if self._should_broadcast_heartbeat(current_time):
                if self._broadcast_heartbeat():
                    self._last_heartbeat = current_time
                    broadcast_sent = True
            
            # Process received messages
            self._process_received_messages()
            
            return broadcast_sent
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"V2VManager broadcast update error: {e}")
            return False
    
    def _should_broadcast_local_state(self, current_time: float) -> bool:
        """Check if local state should be broadcast"""
        return current_time - self._last_local_broadcast >= 1.0 / self.config.local_state_frequency
    
    def _should_broadcast_fleet_state(self, current_time: float) -> bool:
        """Check if fleet state should be broadcast"""
        return current_time - self._last_fleet_broadcast >= 1.0 / self.config.fleet_state_frequency
    
    def _should_broadcast_heartbeat(self, current_time: float) -> bool:
        """Check if heartbeat should be broadcast"""
        return current_time - self._last_heartbeat >= 1.0 / self.config.heartbeat_frequency
    
    def _broadcast_local_state(self) -> bool:
        """Broadcast local state at high frequency"""
        try:
            if not self.vehicle_observer:
                if self.logger:
                    self.logger.debug(f"Vehicle {self.vehicle_id}: No vehicle observer for local state broadcast")
                return False
            
            local_state = self.vehicle_observer.get_local_state_for_broadcast()
            
            success = self.v2v_communication.send_message(
                message_type="local_state",
                data=local_state
            )
            
            if success:
                with self._lock:
                    self.stats['local_broadcasts'] += 1
                    self._last_local_broadcast = time.time()
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
        """Broadcast fleet state at lower frequency"""
        try:
            if not self.vehicle_observer:
                if self.logger:
                    self.logger.debug(f"Vehicle {self.vehicle_id}: No vehicle observer for fleet state broadcast")
                return False
            
            fleet_state = self.vehicle_observer.get_fleet_state_for_broadcast()
            
            success = self.v2v_communication.send_message(
                message_type="fleet_state",
                data=fleet_state
            )
            
            if success:
                with self._lock:
                    self.stats['fleet_broadcasts'] += 1
                    self._last_fleet_broadcast = time.time()
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
        """Handle local state messages"""
        try:
            sender_id = message.sender_id
            data = message.data
            timestamp = message.timestamp
            
            if self.logger:
                self.logger.debug(f"Vehicle {self.vehicle_id}: Received local state from vehicle {sender_id}")
            
            with self._lock:
                # Add to received local states with timestamp
                self.received_local_states[sender_id].append((timestamp, data))
                
                # Add to VehicleObserver if available
                if self.vehicle_observer and 'x' in data:
                    import numpy as np
                    state_vector = np.array([data['x'], data['y'], data['theta'], data['velocity']])
                    self.vehicle_observer.add_received_state(sender_id, state_vector, timestamp)
            
            self._add_to_queue(self.local_state_queue, data, sender_id)
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Local state message handling error: {e}")
                self.logger.error(f"Local state message handling error: {e}")
    
    def _handle_fleet_state_message(self, message: V2VMessage):
        """Handle fleet state messages"""
        try:
            sender_id = message.sender_id
            data = message.data
            timestamp = message.timestamp
            
            with self._lock:
                # Add to received fleet states with timestamp
                self.received_fleet_states[sender_id].append((timestamp, data))
            
            self._add_to_queue(self.fleet_state_queue, data, sender_id)
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Fleet state message handling error: {e}")
    
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
                'fleet_broadcast_rate': self.config.fleet_state_frequency
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
