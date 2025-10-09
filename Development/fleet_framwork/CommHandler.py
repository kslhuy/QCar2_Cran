import socket
import json as ujson
import time
import logging
from typing import Dict, Any, Tuple, Optional
import threading

# Import logging configuration
from md_logging_config import get_fleet_observer_logger

# Import performance monitoring
try:
    from performance_monitor import perf_monitor
    PERFORMANCE_MONITORING = True
except ImportError:
    PERFORMANCE_MONITORING = False
    perf_monitor = None

class CommHandler:
    """
    Handles UDP communication for vehicle fleet coordination.
    
    This class manages:
    - Sending vehicle state updates with reliable delivery (ACK mechanism)
    - Receiving messages from other vehicles (state updates, heartbeats)
    - ACK processing for reliable message delivery
    - Thread-safe operations for concurrent communication
    - Non-blocking operations for process-based vehicles
    
    Communication Protocol:
    - State messages: Vehicle position, rotation, velocity data
    - ACK messages: Acknowledgment of received state messages
    - Heartbeat messages: Keep-alive signals between vehicles
    
    Modes:
    - Threaded mode: Uses background threads for send/receive operations
    - Non-blocking mode: Uses polling-based operations for process-based vehicles
    """
    
    # Communication timing constants
    STATE_SEND_PERIOD = 0.05        # Send state every 50ms (20Hz) - reduced from 100Hz for performance
    RECEIVE_PERIOD = 0.05            # Receive loop period 50ms (20Hz) - reduced from 100Hz
    ACK_TIMEOUT = 0.2               # Wait 200ms for ACK response - reduced for faster retries
    RECV_TIMEOUT = 0.01             # Socket receive timeout 10ms
    MAX_RETRIES = 2                 # Maximum retry attempts - reduced from 3
    HEARTBEAT_TIMEOUT = 2.0         # Heartbeat timeout threshold
    
    def __init__(self, vehicle_id: int, target_ip: str, send_port: int, recv_port: int, 
                 ack_port: int, logger, running_flag=None, vehicle=None, mode='threaded',
                 peer_ports=None, communication_mode='unidirectional' , ack_enabled = True,
                 topology='fully_connected', fleet_size=3):
        """
        Initialize communication handler for a vehicle.
        
        Args:
            vehicle_id: Unique identifier for this vehicle
            target_ip: IP address to send messages to (broadcast or specific vehicle)
            send_port: Port for sending state messages
            recv_port: Port for receiving messages from other vehicles
            ack_port: Port for receiving ACK confirmations
            logger: Logger instance for debugging and monitoring
            running_flag: Shared threading event to control operation (optional for non-blocking mode)
            vehicle: Reference to the Vehicle instance for state access (optional for non-blocking mode)
            mode: Communication mode - 'threaded' for background threads, 'non_blocking' for polling
            peer_ports: Dictionary mapping peer vehicle IDs to their port configurations for bidirectional communication
            communication_mode: 'unidirectional' (leader->followers) or 'bidirectional' (peer-to-peer)
            topology: Communication topology - 'fully_connected', 'chain', 'star'
            fleet_size: Total number of vehicles in the fleet
        """
        # Vehicle identification and network configuration
        self.vehicle_id = vehicle_id
        self.target_ip = target_ip
        self.send_port = send_port
        self.recv_port = recv_port
        self.ack_port = ack_port
        self.logger = logger
        
        # Add fleet observer logger for separate fleet estimation logging
        self.fleet_comm_logger = get_fleet_observer_logger(vehicle_id)
        
        self.mode = mode
        
        # NEW: Bidirectional communication support
        self.peer_ports = peer_ports or {}
        self.communication_mode = communication_mode
        self.is_bidirectional = (communication_mode == 'bidirectional')
        
        # Initialize communication state
        self.initialized = False
        self.last_send_time = 0
        self.send_interval = 0.05  # 20Hz for non-blocking mode
        
        # NEW: Communication topology configuration (from vehicle config)
        self.topology = topology
        self.fleet_size = fleet_size
        
        # Pre-compute communication targets for efficiency
        self.communication_targets = self._get_communication_targets()
        
        self.ack_enabled = ack_enabled  # NEW: Enable/disable ACK mechanism
        
        # NEW: Initialize batching attributes for non-blocking mode
        self.message_batch = []  # Batch of messages to send
        self.batch_size = 1      # Send immediately (can be increased for batching)
        
        # NEW: Initialize ACK counter for periodic ACK sending
        self.ack_counter = 0
        self.ack_interval = 5    # Send ACK every 5th message to reduce overhead
        
        # Initialize UDP sockets for different communication purposes
        if mode == 'threaded':
            self._setup_sockets_threaded()
            # Thread synchronization
            self.lock = threading.Lock()
            self.running = running_flag  # Reference to Vehicle's running flag
            self.vehicle = vehicle       # Reference to Vehicle instance
        else:  # non_blocking mode
            self._setup_sockets_non_blocking()
        
        # Message sequencing and synchronization
        self.sequence_number = 0
        self.last_heartbeat_time = time.monotonic()  # Use monotonic for timing measurements
        self.heartbeat_timeout = self.HEARTBEAT_TIMEOUT
        self.heartbeat = True
        
        self.logger.info(f"CommHandler initialized for Vehicle {self.vehicle_id} in {mode} mode")
        
    def _setup_sockets_threaded(self):
        """
        Initialize and configure all UDP sockets for threaded communication.
        
        Creates four sockets:
        - send_sock: For sending state messages to other vehicles
        - recv_sock: For receiving messages from other vehicles
        - send_ack_sock: For sending ACK responses
        - ack_sock: For receiving ACK confirmations
        """
        try:
            # Socket for sending state messages
            self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 262144)  # 256KB buffer
            self.send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)  # 256KB buffer
            
            # Socket for receiving messages from other vehicles
            self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 262144)
            self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
            self.recv_sock.bind(('0.0.0.0', self.recv_port))
            self.recv_sock.settimeout(self.RECV_TIMEOUT)
            
            # Socket for sending ACK responses
            self.send_ack_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.send_ack_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 262144)
            self.send_ack_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
            
            # Socket for receiving ACK confirmations
            self.ack_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.ack_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 262144)
            self.ack_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
            self.ack_sock.settimeout(self.ACK_TIMEOUT)
            self.ack_sock.bind(('0.0.0.0', self.ack_port))
            
            self.logger.info(f"Threaded sockets configured - recv_port: {self.recv_port}, ack_port: {self.ack_port}")
            
        except Exception as e:
            self.logger.error(f"Failed to setup threaded sockets: {e}")
            raise
    
    def _setup_sockets_non_blocking(self):
        """
        Initialize and configure UDP sockets for non-blocking communication.
        """
        try:
            # Create send socket
            self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 262144)  # 256KB buffer
            self.send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
            
            # Create receive socket
            self.recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 262144)
            self.recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
            self.recv_socket.bind(('', self.recv_port))
            self.recv_socket.settimeout(0.001)  # 1ms timeout for non-blocking
            
            # Create ACK socket
            self.ack_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.ack_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.ack_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 262144)
            self.ack_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
            self.ack_socket.bind(('', self.ack_port))
            self.ack_socket.settimeout(0.001)  # 1ms timeout for non-blocking
            
            self.initialized = True
            self.logger.info(f"Non-blocking sockets configured - recv_port: {self.recv_port}, ack_port: {self.ack_port}")
            
        except Exception as e:
            self.logger.error(f"Socket initialization error: {e}")
            self.initialized = False
            raise
        
    def _setup_sockets(self):
        """
        Legacy method for backward compatibility - calls threaded setup.
        """
        return self._setup_sockets_threaded()

    def _create_state_message(self) -> Dict[str, Any]:
        """
        Create a state message with current vehicle information.
        
        Returns:
            Dictionary containing vehicle state data for transmission
        """
        with self.lock:
            # Gather current vehicle state data
            current_pos = self.vehicle.current_pos
            current_rot = self.vehicle.current_rot
            velocity = self.vehicle.velocity
            
            # Use GPS-synchronized time if available, fallback to system time
            try:
                synced_time = self.vehicle.gps_sync.get_synced_time()
            except AttributeError:
                synced_time = time.time()  # Fallback if GPS sync not available
            
            # Create structured message
            state_data = {
                'type': 'state',
                'seq': self.sequence_number,
                'id': self.vehicle_id,
                'pos': current_pos,
                'rot': current_rot,
                'v': velocity,
                'timestamp': synced_time,
                'ack_port': self.ack_port,
                'control': self.vehicle.current_control_input.tolist() if hasattr(self.vehicle, 'current_control_input') else [0.0, 0.0]
            }
            
        return state_data
    
    def _send_state_message(self, data: Dict[str, Any]) -> bool:
        """
        Send a state message via UDP socket.
        
        Args:
            data: State message data to send
            
        Returns:
            True if message sent successfully, False otherwise
        """
        try:
            message_json = ujson.dumps(data).encode()
            self.send_sock.sendto(message_json, (self.target_ip, self.send_port))
            
            # Log detailed send data for vehicle 0 (save all data sent by vehicle 0)
            if self.vehicle_id == 0:
                current_time = time.time()
                self.logger.info(f"SEND_DATA: {{\"timestamp\": {current_time:.6f}, \"seq\": {data['seq']}, \"vehicle_id\": {self.vehicle_id}, \"target_ip\": \"{self.target_ip}\", \"send_port\": {self.send_port}, \"position\": {data['pos']}, \"rotation\": {data['rot']}, \"velocity\": {data['v']:.6f}, \"message_size\": {len(message_json)}}}")
            
            self.logger.info(f"SENT: Seq: {self.sequence_number}, "
                           f"Pos: {data['pos']}, V: {data['v']:.3f}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send state message: {e}")
            return False
    
    def _wait_for_ack(self) -> bool:
        """
        Wait for ACK response after sending a state message.
        
        Returns:
            True if valid ACK received, False on timeout or invalid ACK
        """
        try:
            self.ack_sock.settimeout(self.ACK_TIMEOUT)
            ack_data, addr = self.ack_sock.recvfrom(1024)
            ack = ujson.loads(ack_data.decode())
            
            self.logger.debug(f"ACK received on port {self.ack_port} from {addr}")
            self.logger.debug(f"ACK content: ack_id={ack.get('ack_id')}, "
                            f"ack_seq={ack.get('ack_seq')}, expected_seq={self.sequence_number}")
            
            # Validate ACK message
            if (ack.get('type') == 'ack' and 
                ack.get('ack_seq') == self.sequence_number and 
                ack.get('ack_id') != self.vehicle_id):  # Don't ACK our own messages
                
                # Increment sequence number for next message
                with self.lock:
                    self.sequence_number += 1
                    
                self.logger.info(f"Valid ACK received for seq: {self.sequence_number - 1} from {addr}")
                return True
            else:
                self.logger.debug(f"ACK discarded - invalid or unexpected: {ack}")
                return False
                
        except socket.timeout:
            self.logger.warning(f"ACK timeout for seq: {self.sequence_number}")
            return False
        except Exception as e:
            self.logger.error(f"ACK processing error: {e}")
            return False

    def send_state(self):
        """
        Main state transmission loop - runs in dedicated thread.
        
        Continuously sends vehicle state updates with reliable delivery:
        1. Create state message with current vehicle data
        2. Send message via UDP
        3. Wait for ACK confirmation
        4. Retry up to MAX_RETRIES times if no ACK received
        5. Maintain consistent transmission rate
        """
        self.logger.info(f"Starting state transmission thread for Vehicle {self.vehicle_id}")
        
        try:
            while self.running.is_set():
                self.logger.debug(f"Vehicle {self.vehicle_id} send_state active, seq: {self.sequence_number}")
                
                start_time = time.monotonic()  # Use monotonic for elapsed time measurement
                retries = 0
                ack_received = False
                
                # Retry loop for reliable delivery
                while retries < self.MAX_RETRIES and not ack_received and self.running.is_set():
                    try:
                        # Step 1: Create state message
                        state_data = self._create_state_message()
                        
                        # Step 2: Send state message
                        if self._send_state_message(state_data):
                            # Step 3: Wait for ACK confirmation
                            ack_received = self._wait_for_ack()
                        
                        if not ack_received:
                            retries += 1
                            if retries < self.MAX_RETRIES:
                                self.logger.warning(f"Retry {retries}/{self.MAX_RETRIES} for seq: {self.sequence_number}")
                                time.sleep(0.1)  # Brief delay before retry
                            
                    except Exception as e:
                        self.logger.error(f"State transmission error: {e}")
                        retries += 1
                        time.sleep(0.1)
                
                # Log final result for this transmission cycle
                if not ack_received:
                    self.logger.error(f"Failed to receive ACK for seq: {self.sequence_number} after {self.MAX_RETRIES} retries")
                
                # Maintain consistent transmission rate
                elapsed = time.monotonic() - start_time
                sleep_time = max(0, self.STATE_SEND_PERIOD - elapsed)
                time.sleep(sleep_time)
                
        except Exception as e:
            self.logger.error(f"State transmission thread crashed: {e}")
        finally:
            self.logger.info(f"Vehicle {self.vehicle_id} state transmission thread exited")

    def _send_ack_response(self, seq: int, sender_addr: Tuple[str, int], sender_ack_port: int):
        """
        Send ACK response to acknowledge received message.
        
        Args:
            seq: Sequence number to acknowledge
            sender_addr: Address tuple (IP, port) of message sender
            sender_ack_port: Port where sender expects ACK response
        """
        try:
            ack_message = {
                'type': 'ack',
                'ack_seq': seq,
                'ack_id': self.vehicle_id
            }
            
            ack_json = ujson.dumps(ack_message).encode()
            self.send_ack_sock.sendto(ack_json, (sender_addr[0], sender_ack_port))
            
            self.logger.info(f"SENT: ACK for seq: {seq} to {sender_addr[0]}:{sender_ack_port}")
            
        except Exception as e:
            self.logger.error(f"Failed to send ACK response: {e}, sender: {sender_addr}")
    
    def _handle_state_message(self, message: Dict[str, Any], sender_addr: Tuple[str, int]):
        """
        Process received state message from another vehicle.
        
        Args:
            message: Decoded state message data
            sender_addr: Address tuple (IP, port) of message sender
        """
        sender_id = message.get('id')
        seq = message.get('seq', -1)
        
        # Ignore messages from ourselves
        if sender_id == self.vehicle_id:
            return
        
        # Log detailed receive data for vehicle 1 (save all data received by vehicle 1)
        if self.vehicle_id == 1:
            receive_time = time.time()
            pos = message.get('pos', [0, 0, 0])
            rot = message.get('rot', [0, 0, 0])
            vel = message.get('v', 0.0)
            msg_timestamp = message.get('timestamp', 0.0)
            delay = receive_time - msg_timestamp if msg_timestamp > 0 else 0.0
            
            self.logger.info(f"STATE_RECV_DATA: {{\"receive_timestamp\": {receive_time:.6f}, \"message_timestamp\": {msg_timestamp:.6f}, \"delay\": {delay:.6f}, \"seq\": {seq}, \"sender_id\": {sender_id}, \"sender_ip\": \"{sender_addr[0]}\", \"sender_port\": {sender_addr[1]}, \"position\": {pos}, \"rotation\": {rot}, \"velocity\": {vel:.6f}}}")
            
        self.logger.info(f"STATE_RECV: Seq={seq}, Sender={sender_id}, "
                        f"Pos=({message.get('pos', [0,0,0])[0]:.3f},{message.get('pos', [0,0,0])[1]:.3f}), "
                        f"Vel={message.get('v', 0.0):.3f}")
        
        # Send ACK response
        sender_ack_port = message.get('ack_port')
        if sender_ack_port:
            self._send_ack_response(seq, sender_addr, sender_ack_port)
        
        # Process state through Vehicle's enhanced state management
        try:
            # Call the Vehicle's process_received_state method which handles StateQueue validation
            self.vehicle.process_received_state(message)
        except Exception as e:
            self.logger.error(f"Error processing received state through Vehicle: {e}")
            
            # Fallback: Update follower's leader state directly (old method)
            if not self.vehicle.is_leader:
                with self.lock:
                    self.vehicle.leader_state = message
                    self.logger.info(f"Fallback: Updated leader_state from Vehicle {sender_id}")
    
    def _handle_heartbeat_message(self, message: Dict[str, Any]):
        """
        Process received heartbeat message.
        
        Args:
            message: Decoded heartbeat message data
        """
        sender_id = message.get('id')
        self.last_heartbeat_time = time.monotonic()  # Use monotonic for timing
        self.heartbeat = True
        self.logger.info(f"RECEIVED HEARTBEAT from Vehicle {sender_id}")
    
    def _handle_ack_message(self, message: Dict[str, Any]):
        """
        Process received ACK message.
        
        Args:
            message: Decoded ACK message data
            
        Note: ACK messages are primarily handled in _wait_for_ack() method.
        This method is for any additional ACK processing if needed.
        """
        # ACK messages are handled in the send_state method's _wait_for_ack call
        self.logger.debug(f"ACK message received: {message}")
    
    def _handle_fleet_estimates_message(self, message: Dict[str, Any], sender_addr: Tuple[str, int]):
        """
        Process received fleet estimates message from distributed observer.
        
        Args:
            message: Decoded fleet estimates message data
            sender_addr: Address tuple (IP, port) of message sender
        """
        sender_id = message.get('sender_id') or message.get('vehicle_id')
        seq = message.get('seq', -1)
        timestamp = message.get('timestamp', time.time())
        
        # Ignore messages from ourselves
        if sender_id == self.vehicle_id:
            return
        
        # Log fleet estimates reception with complete format
        estimates = message.get('estimates', {})
        sender_id = message.get('sender_id', message.get('vehicle_id', 'unknown'))
        seq = message.get('seq', -1)
        timestamp = message.get('timestamp', 0.0)
        
        # Extract complete estimate data (pos, rot, vel)
        est_data = {}
        for vid, est in estimates.items():
            pos = est.get('pos', [0, 0])
            rot = est.get('rot', [0, 0, 0])
            vel = est.get('vel', 0.0)
            est_data[f'V{vid}'] = f'Pos=({pos[0]:.2f},{pos[1]:.2f}) Rot={rot[2]:.2f} Vel={vel:.2f}'
        
        self.fleet_comm_logger.info(f"RECV From=V{sender_id} Seq={seq} T={timestamp:.3f} {est_data}")
        
        # Send ACK response if required
        sender_ack_port = message.get('ack_port')
        if sender_ack_port:
            self._send_ack_response(seq, sender_addr, sender_ack_port)
        
        # Process fleet estimates through Vehicle's observer system
        try:
            if hasattr(self.vehicle, 'process_received_fleet_estimates'):
                self.vehicle.process_received_fleet_estimates(message)
            else:
                # Fallback: Log warning for missing processor
                self.fleet_comm_logger.warning(f"No fleet estimates processor available for Vehicle {self.vehicle_id}")
                    
        except Exception as e:
            self.logger.error(f"Error processing received fleet estimates: {e}")
    
    def _process_received_message(self, data: bytes, sender_addr: Tuple[str, int]):
        """
        Decode and route received message to appropriate handler.
        
        Args:
            data: Raw message bytes received from socket
            sender_addr: Address tuple (IP, port) of message sender
        """
        try:
            # Start JSON decode timing
            if PERFORMANCE_MONITORING:
                json_start = perf_monitor.start_timing()
            
            # Fast JSON decode
            message = ujson.loads(data.decode())
            
            # End JSON decode timing
            if PERFORMANCE_MONITORING:
                perf_monitor.end_timing(json_start, "json_decode")
            
            msg_type = message.get('type', '')
            sender_id = message.get('id')
            
            # Quick type check to avoid unnecessary processing
            if sender_id == self.vehicle_id:
                return  # Ignore messages from ourselves
            
            # Route to appropriate message handler (optimized order by frequency)
            if msg_type == 'state':
                self._handle_state_message_optimized(message, sender_addr)
            elif msg_type == 'fleet_estimates':
                self._handle_fleet_estimates_message(message, sender_addr)
            elif msg_type == 'ack':
                self._handle_ack_message(message)
            elif msg_type == 'heartbeat':
                self._handle_heartbeat_message(message)
            else:
                self.logger.warning(f"Unknown message type received: {msg_type} from {sender_addr}")
                
        except Exception as e:
            self.logger.error(f"Failed to process received message: {e}, sender: {sender_addr}")
    
    def _handle_state_message_optimized(self, message: Dict[str, Any], sender_addr: Tuple[str, int]):
        """
        Optimized version of state message handling.
        
        Args:
            message: Decoded state message data
            sender_addr: Address tuple (IP, port) of message sender
        """
        # Start performance timing
        if PERFORMANCE_MONITORING:
            process_start = perf_monitor.start_timing()
        
        # Extract essential data quickly
        sender_id = message.get('id')
        seq = message.get('seq', -1)
        
        # Log detailed receive data for vehicle 1 (save all data received by vehicle 1)
        if self.vehicle_id == 1:
            receive_time = time.time()
            pos = message.get('pos', [0, 0, 0])
            rot = message.get('rot', [0, 0, 0])
            vel = message.get('v', 0.0)
            msg_timestamp = message.get('timestamp', 0.0)
            delay = receive_time - msg_timestamp if msg_timestamp > 0 else 0.0
            
            self.logger.info(f"RECEIVE_DATA: {{\"receive_timestamp\": {receive_time:.6f}, \"message_timestamp\": {msg_timestamp:.6f}, \"delay\": {delay:.6f}, \"seq\": {seq}, \"sender_id\": {sender_id}, \"sender_ip\": \"{sender_addr[0]}\", \"sender_port\": {sender_addr[1]}, \"position\": {pos}, \"rotation\": {rot}, \"velocity\": {vel:.6f}}}")
        
        # Log with minimal formatting for performance
        self.logger.info(f"RECEIVED STATE: Seq: {seq}, Sender ID: {sender_id}, "
                        f"Pos: {message.get('pos')}, V: {message.get('v', 0.0):.3f}")
        
        # Send ACK response (non-blocking)
        sender_ack_port = message.get('ack_port')
        if sender_ack_port:
            self._send_ack_response(seq, sender_addr, sender_ack_port)
        
        # Process state through Vehicle's enhanced state management
        # This is the potentially slow part - do it last
        try:
            if PERFORMANCE_MONITORING:
                validation_start = perf_monitor.start_timing()
            
            self.vehicle.process_received_state(message)
            
            if PERFORMANCE_MONITORING:
                perf_monitor.end_timing(validation_start, "validation")
        except Exception as e:
            self.logger.error(f"Error processing received state through Vehicle: {e}")
            
            # Fallback: Update follower's leader state directly (old method)
            if not self.vehicle.is_leader:
                with self.lock:
                    self.vehicle.leader_state = message
                    self.logger.info(f"Fallback: Updated leader_state from Vehicle {sender_id}")
        
        # End performance timing
        if PERFORMANCE_MONITORING:
            perf_monitor.end_timing(process_start, "processing")

    def receive_messages(self):
        """
        Main message reception loop - runs in dedicated thread.
        
        Continuously listens for incoming messages:
        1. Receive UDP packets on the designated port
        2. Decode and validate message format
        3. Route messages to appropriate handlers based on type
        4. Send ACK responses for state messages
        5. Update vehicle state for follower vehicles
        """
        self.logger.info(f"Starting message reception thread for Vehicle {self.vehicle_id}")
        
        while self.running.is_set():
            start_time = time.monotonic()  # Use monotonic for loop timing
            
            try:
                # Inner loop for more responsive shutdown checking
                while time.monotonic() - start_time < 0.09 and self.running.is_set():
                    try:
                        # Attempt to receive message with timeout
                        data, sender_addr = self.recv_sock.recvfrom(1024)
                        
                        # self.logger.debug(f"Vehicle {self.vehicle_id} received data from {sender_addr}")
                        
                        # Process the received message
                        self._process_received_message(data, sender_addr)
                        
                    except socket.timeout:
                        # Timeout is expected - allows for responsive shutdown
                        break
                    except Exception as e:
                        self.logger.error(f"Message reception error: {e}")
                        break
                        
            except Exception as e:
                self.logger.error(f"Reception loop error: {e}")
            
            # Maintain consistent loop timing
            elapsed = time.monotonic() - start_time
            sleep_time = max(0, self.RECEIVE_PERIOD - elapsed)
            time.sleep(sleep_time)
            
        self.logger.info(f"Message reception thread terminated for Vehicle {self.vehicle_id}")

    def is_heartbeat_alive(self) -> bool:
        """
        Check if heartbeat signal is still active.
        
        Returns:
            True if recent heartbeat received, False if timeout exceeded
        """
        time_since_heartbeat = time.monotonic() - self.last_heartbeat_time
        is_alive = time_since_heartbeat < self.heartbeat_timeout
        
        if not is_alive:
            self.logger.warning(f"Heartbeat timeout: {time_since_heartbeat:.2f}s since last heartbeat")
            
        return is_alive
    
    def get_communication_stats(self) -> Dict[str, Any]:
        """
        Get current communication statistics for monitoring.
        
        Returns:
            Dictionary with communication status and statistics
        """
        return {
            'vehicle_id': self.vehicle_id,
            'sequence_number': self.sequence_number,
            'heartbeat_active': self.heartbeat,
            'last_heartbeat_time': self.last_heartbeat_time,
            'time_since_heartbeat': time.monotonic() - self.last_heartbeat_time,
            'is_running': self.running.is_set(),
            'target_ip': self.target_ip,
            'ports': {
                'send': self.send_port,
                'receive': self.recv_port,
                'ack': self.ack_port
            }
        }

    # Non-blocking communication methods for process-based vehicles
    def send_state_broadcast(self, state_data: dict):
        """Send state data to other vehicles using non-blocking method with optimizations.
        
        NEW: Supports batching and optional ACKs for improved performance.
        """
        if self.mode != 'non_blocking':
            self.logger.warning("send_state_broadcast called in threaded mode - use send_state instead")
            return
            
        current_time = time.time()
        if not self.initialized or current_time - self.last_send_time < self.send_interval:
            return
            
        try:
            # Start performance timing if available
            if PERFORMANCE_MONITORING:
                send_start = perf_monitor.start_timing()
            
            # Create optimized message structure
            message = {
                'type': 'state',
                'vehicle_id': self.vehicle_id,
                'timestamp': current_time,
                'id': self.vehicle_id,  # For compatibility with existing message handlers
                'pos': state_data.get('pos',  [0, 0, 0]),
                'rot': state_data.get('rot',  [0, 0, 0]),
                'v': state_data.get('vel',  0.0),
                'ctrl_u': state_data.get('ctrl_u', [0.0, 0.0]),
                'seq': self.sequence_number,
                'ack_port': self.ack_port if self.ack_enabled else None,  # Optional ACK
                'covariance_trace': state_data.get('covariance_trace', 1.0)  # NEW: Include covariance for quality tracking
            }
            
            # Add to batch
            self.message_batch.append(message)
            
            # Send batch when full or on timeout
            if len(self.message_batch) >= self.batch_size:
                self._send_message_batch()
            else:
                # Send immediately if batching disabled
                self._send_message_batch()
            
            # Update state
            self.last_send_time = current_time
            self.sequence_number += 1
            
            # End performance timing
            if PERFORMANCE_MONITORING:
                perf_monitor.end_timing(send_start, "non_blocking_send")
            
            # Debug logging (only if enabled to avoid overhead)
            if self.logger.isEnabledFor(logging.DEBUG):
                self.logger.debug(f"Vehicle {self.vehicle_id}: Broadcasted seq={message['seq']} - "
                                f"pos={message['pos']}, v={message['v']:.3f}, mode={self.communication_mode}")
            
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Send error: {e}")
            # Reset sequence number on error to avoid getting stuck
            if "Address already in use" in str(e):
                self.logger.warning(f"Vehicle {self.vehicle_id}: Address in use, reinitializing sockets")
                try:
                    self._setup_sockets_non_blocking()
                except Exception as reinit_error:
                    self.logger.error(f"Vehicle {self.vehicle_id}: Socket reinit failed: {reinit_error}")
            return False
        
        return True
    
    def _get_communication_targets(self):
        """Get list of vehicle IDs to communicate with based on topology."""
        if self.topology == 'fully_connected':
            # Communicate with all other vehicles
            return [i for i in range(self.fleet_size) if i != self.vehicle_id]
        elif self.topology == 'chain':
            # Communicate only with immediate neighbors
            targets = []
            if self.vehicle_id > 0:  # Has predecessor
                targets.append(self.vehicle_id - 1)
            if self.vehicle_id < self.fleet_size - 1:  # Has successor
                targets.append(self.vehicle_id + 1)
            return targets
        elif self.topology == 'star':
            # Star topology: all communicate with leader (vehicle 0)
            return [0] if self.vehicle_id != 0 else [i for i in range(1, self.fleet_size)]
        else:
            # Default to fully connected
            return [i for i in range(self.fleet_size) if i != self.vehicle_id]
    
    def _send_message_batch(self):
        """Send accumulated message batch efficiently."""
        if not self.message_batch:
            return
            
        try:
            # Use pre-computed communication targets for efficiency
            targets = self.communication_targets
            
            # For now, send messages individually (can be optimized to send as array)
            for message in self.message_batch:
                message_bytes = ujson.dumps(message).encode('utf-8')
                
                if self.topology == 'star' and self.vehicle_id == 0:
                    # Leader broadcasts to all followers
                    self.send_socket.sendto(message_bytes, (self.target_ip, self.send_port))
                    self.logger.debug(f"Vehicle {self.vehicle_id}: Star topology batch broadcast")
                elif targets:
                    # Send to specific targets based on topology
                    for target_id in targets:
                        if self.is_bidirectional and self.peer_ports and target_id in self.peer_ports:
                            peer_config = self.peer_ports[target_id]
                            peer_recv_port = peer_config.get('send_to_peer', self.send_port)
                            self.send_socket.sendto(message_bytes, (self.target_ip, peer_recv_port))
                            self.logger.debug(f"Vehicle {self.vehicle_id}: Topology send to vehicle {target_id}")
                        else:
                            # Fallback to broadcast if no specific peer config
                            self.send_socket.sendto(message_bytes, (self.target_ip, self.send_port))
                else:
                    self.logger.debug(f"Vehicle {self.vehicle_id}: No targets for {self.topology} topology")
            
            # Clear batch after sending
            self.message_batch.clear()
            
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Batch send error: {e}")
    
    def receive_state_non_blocking(self) -> Optional[dict]:
        """
        Receive state data from other vehicles (non-blocking) with improved performance.
        
        Returns:
            Dictionary containing received state data or None if no data available
        """
        if self.mode != 'non_blocking':
            self.logger.warning("receive_state_non_blocking called in threaded mode")
            return None
            
        if not self.initialized:
            return None
            
        try:
            # Start performance timing if available
            if PERFORMANCE_MONITORING:
                receive_start = perf_monitor.start_timing()
            
            # Try to receive data with minimal timeout
            data, addr = self.recv_socket.recvfrom(1024)
            
            # Fast JSON decode
            message = ujson.loads(data.decode('utf-8'))
            
            # Quick validity checks
            sender_id = message.get('vehicle_id') or message.get('id')
            if sender_id == self.vehicle_id:
                return None  # Don't process our own messages
            
            # # Log detailed receive data in vehicle 1 (we see all messages send to vehicle 1)
            # if self.vehicle_id == 1:
            #     receive_time = time.time()
            #     pos = message.get('pos', message.get('position', [0, 0, 0]))
            #     rot = message.get('rot', message.get('rotation', [0, 0, 0]))
            #     vel = message.get('v', message.get('velocity', 0.0))
            #     control = message.get('ctrl_u', [0.0, 0.0])
            #     msg_timestamp = message.get('timestamp', 0.0)
            #     seq = message.get('seq', -1)
            #     delay = receive_time - msg_timestamp if msg_timestamp > 0 else 0.0
                
                # self.logger.info(f"RECEIVE_DATA: {{\"receive_timestamp\": {receive_time:.6f}, \"message_timestamp\": {msg_timestamp:.6f}, \"delay\": {delay:.6f}, \"seq\": {seq}, \"sender_id\": {sender_id}, \"position\": {pos}, \"rotation\": {rot}, \"velocity\": {vel:.2f}, \"control_input\": {control}, \"message_size\": {len(data)}}}")
                
            # Send ACK if this is a state message (non-blocking)
            msg_type = message.get('type', 'state')  # Default to state for backward compatibility
            if msg_type == 'state':
                seq = message.get('seq', -1)
                sender_ack_port = message.get('ack_port')
                if sender_ack_port and seq >= 0 and self.ack_enabled:
                    # Send ACK periodically to reduce overhead
                    self.ack_counter += 1
                    if self.ack_counter >= self.ack_interval:
                        self.ack_counter = 0
                        # Send ACK asynchronously without blocking
                        try:
                            self._send_ack_non_blocking(seq, addr[0], sender_ack_port)
                        except Exception as ack_error:
                            self.logger.debug(f"ACK send failed (non-critical): {ack_error}")
            

            
            # End performance timing
            if PERFORMANCE_MONITORING:
                perf_monitor.end_timing(receive_start, "non_blocking_receive")
                
            return message
            
        except socket.timeout:
            return None  # No data available
        except (ValueError, KeyError) as parse_error:
            self.logger.warning(f"Vehicle {self.vehicle_id}: Message parse error: {parse_error}")
            return None
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Receive error: {e}")
            return None
    
    def send_fleet_estimates_broadcast(self, fleet_message: dict) -> bool:
        """
        Send fleet estimation data to other vehicles using non-blocking method.
        
        This method is specifically designed for broadcasting distributed observer
        fleet estimates, which have a different message structure than regular state messages.
        
        Args:
            fleet_message: Dictionary containing fleet estimates with structure:
                          {'msg_type': 'fleet_estimates', 'estimates': {...}, ...}
        
        Returns:
            True if broadcast successful, False otherwise
        """
        if self.mode == 'threaded':
            # For threaded mode, use the existing socket infrastructure
            return self._send_fleet_estimates_threaded(fleet_message)
        elif self.mode == 'non_blocking':
            return self._send_fleet_estimates_non_blocking(fleet_message)
        else:
            self.logger.warning(f"Unknown mode '{self.mode}' for fleet estimates broadcast")
            return False
    
    def _send_fleet_estimates_threaded(self, fleet_message: dict) -> bool:
        """Send fleet estimates in threaded mode."""
        try:
            # Prepare fleet estimates message with minimal fields
            message = {
                'type': 'fleet_estimates',
                'vehicle_id': self.vehicle_id,
                'timestamp': time.time(),
                'seq': self.sequence_number,
                'fleet_size': fleet_message.get('fleet_size', 0),
                'estimates': fleet_message.get('estimates', {})
            }
            
            # JSON encode and send
            message_json = ujson.dumps(message).encode()
            self.send_sock.sendto(message_json, (self.target_ip, self.send_port))
            
            # Update sequence number
            with self.lock:
                self.sequence_number += 1
            
            # Log to fleet logger with complete format
            estimates = message.get('estimates', {})
            seq = message['seq']
            timestamp = message.get('timestamp', 0.0)
            
            # Extract complete estimate data (pos, rot, vel)
            est_data = {}
            for vid, est in estimates.items():
                pos = est.get('pos', [0, 0])
                rot = est.get('rot', [0, 0, 0])
                vel = est.get('vel', 0.0)
                est_data[f'V{vid}'] = f'Pos=({pos[0]:.2f},{pos[1]:.2f}) Rot={rot[2]:.2f} Vel={vel:.2f}'
            
            # self.fleet_comm_logger.info(f"SEND From=V{self.vehicle_id} Seq={seq} T={timestamp:.3f} {est_data}")
            return True
            
        except Exception as e:
            self.fleet_comm_logger.error(f"Vehicle {self.vehicle_id}: Threaded fleet estimates broadcast error: {e}")
            return False
    
    def _send_fleet_estimates_non_blocking(self, fleet_message: dict) -> bool:
        """Send fleet estimates in non-blocking mode."""        
        if not self.initialized:
            self.logger.warning(f"Vehicle {self.vehicle_id}: Sockets not initialized for fleet broadcast")
            return False
            
        try:
            # Start performance timing if available
            if PERFORMANCE_MONITORING:
                send_start = perf_monitor.start_timing()
            
            # Prepare fleet estimates message with minimal fields
            message = {
                'type': 'fleet_estimates',  # Different type for fleet estimates
                'vehicle_id': self.vehicle_id,
                'timestamp': time.time(),
                'seq': self.sequence_number,
                'fleet_size': fleet_message.get('fleet_size', 0),
                'estimates': fleet_message.get('estimates', {})
            }
            
            # Fast JSON encode
            message_bytes = ujson.dumps(message).encode('utf-8')
            
            # Choose sending strategy based on communication mode
            if self.is_bidirectional and self.peer_ports:
                # BIDIRECTIONAL: Send to all peer vehicles individually
                sent_count = 0
                for peer_id, peer_config in self.peer_ports.items():
                    try:
                        peer_recv_port = peer_config.get('send_to_peer', self.send_port)
                        self.send_socket.sendto(message_bytes, (self.target_ip, peer_recv_port))
                        sent_count += 1
                    except Exception as peer_error:
                        self.fleet_comm_logger.warning(f"Failed to send to peer V{peer_id}: {peer_error}")
                
                # Log with complete format
                estimates = fleet_message.get('estimates', {})
                seq = self.sequence_number
                timestamp = message.get('timestamp', 0.0)
                
                # Extract complete estimate data (pos, rot, vel)
                est_data = {}
                for vid, est in estimates.items():
                    pos = est.get('pos', [0, 0])
                    rot = est.get('rot', [0, 0, 0])
                    vel = est.get('vel', 0.0)
                    est_data[f'V{vid}'] = f'Pos=({pos[0]:.2f},{pos[1]:.2f}) Rot={rot[2]:.2f} Vel={vel:.2f}'
                
                # self.fleet_comm_logger.info(f"SEND From=V{self.vehicle_id} Seq={seq} T={timestamp:.3f} {est_data}")
            else:
                # UNIDIRECTIONAL: Broadcast to all vehicles on standard port
                self.send_socket.sendto(message_bytes, (self.target_ip, self.send_port))
                
                # Log with complete format
                estimates = fleet_message.get('estimates', {})
                seq = self.sequence_number
                timestamp = message.get('timestamp', 0.0)
                
                # Extract complete estimate data (pos, rot, vel)
                est_data = {}
                for vid, est in estimates.items():
                    pos = est.get('pos', [0, 0])
                    rot = est.get('rot', [0, 0, 0])
                    vel = est.get('vel', 0.0)
                    est_data[f'V{vid}'] = f'Pos=({pos[0]:.2f},{pos[1]:.2f}) Rot={rot[2]:.2f} Vel={vel:.2f}'
                
                # self.fleet_comm_logger.info(f"SEND From=V{self.vehicle_id} Seq={seq} T={timestamp:.3f} {est_data}")
            
            # Update state
            self.sequence_number += 1
            
            # End performance timing
            if PERFORMANCE_MONITORING:
                perf_monitor.end_timing(send_start, "fleet_estimates_send")
            
            return True
            
        except Exception as e:
            self.fleet_comm_logger.error(f"Vehicle {self.vehicle_id}: Fleet estimates broadcast error: {e}")
            return False
    
    
    
    def _send_ack_non_blocking(self, seq: int, sender_ip: str, sender_ack_port: int):
        """Send ACK response in non-blocking mode."""
        try:
            ack_message = {
                'type': 'ack',
                'ack_seq': seq,
                'ack_id': self.vehicle_id
            }
            
            ack_json = ujson.dumps(ack_message).encode('utf-8')
            
            # Use a temporary socket for ACK sending to avoid port conflicts
            ack_send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            ack_send_socket.sendto(ack_json, (sender_ip, sender_ack_port))
            ack_send_socket.close()
            
            self.logger.debug(f"Vehicle {self.vehicle_id}: Sent ACK for seq={seq} to {sender_ip}:{sender_ack_port}")
            
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Failed to send ACK: {e}")
    
    def initialize_sockets(self):
        """Initialize sockets for non-blocking mode (compatibility method)."""
        if self.mode == 'non_blocking':
            self._setup_sockets_non_blocking()
        else:
            self._setup_sockets_threaded()

    def cleanup(self):
        """
        Clean shutdown of communication handler.
        
        Safely closes all UDP sockets and logs cleanup status.
        Should be called when shutting down the vehicle or communication system.
        """
        self.logger.info(f"Starting cleanup for Vehicle {self.vehicle_id} CommHandler")
        
        if self.mode == 'threaded':
            # Cleanup threaded mode sockets
            sockets = [
                ('send_sock', getattr(self, 'send_sock', None)),
                ('recv_sock', getattr(self, 'recv_sock', None)), 
                ('send_ack_sock', getattr(self, 'send_ack_sock', None)),
                ('ack_sock', getattr(self, 'ack_sock', None))
            ]
        else:
            # Cleanup non-blocking mode sockets
            sockets = [
                ('send_socket', getattr(self, 'send_socket', None)),
                ('recv_socket', getattr(self, 'recv_socket', None)),
                ('ack_socket', getattr(self, 'ack_socket', None))
            ]
        
        for socket_name, sock in sockets:
            try:
                if sock:
                    sock.close()
                    self.logger.debug(f"Closed {socket_name}")
            except Exception as e:
                self.logger.error(f"Error closing {socket_name}: {e}")
        
        if self.mode == 'non_blocking':
            self.initialized = False
        
        self.logger.info(f"CommHandler cleanup completed for Vehicle {self.vehicle_id}")
