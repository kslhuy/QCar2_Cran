import socket
import json as ujson
import time
import random
import logging
from md_logging_config import get_gps_logger

class GPSSync:
    """
    GPS-like time synchronization client for vehicle fleet coordination.
    
    This class acts as a GPS time synchronization client that connects to a central
    GPS time server to maintain synchronized time across multiple vehicles in a fleet.
    It provides functionality to request time from a GPS server, calculate time offsets,
    and provide synchronized timestamps for coordination purposes.
    
    Key Features:
    - UDP-based communication with GPS time server
    - Automatic time offset calculation and management
    - Fault tolerance with fallback to last known offset
    - Simulated GPS jitter for realistic behavior
    - Periodic synchronization capability
    
    Attributes:
        gps_time_offset (float): Current time offset between GPS and local time
        last_sync_time (float): Timestamp of last successful synchronization
        gps_server_ip (str): IP address of the GPS time server
        gps_server_port (int): Port number of the GPS time server
        last_valid_offset (float): Last known valid time offset for fallback
    """
    
    def __init__(self, gps_server_ip='127.0.0.1', gps_server_port=8001, vehicle_id=None):
        """
        Initialize GPS synchronization client.
        
        Sets up the GPS sync client with connection parameters and initializes
        time tracking variables. The client is configured to connect to a
        GPS time server for time synchronization.
        
        Args:
            gps_server_ip (str): IP address of the GPS time server (default: localhost)
            gps_server_port (int): Port number of the GPS time server (default: 8001)
            vehicle_id (int): Vehicle ID for logging purposes (default: None)
        """
        # Time synchronization state
        self.gps_time_offset = 0              # Current calculated time offset (GPS - local)
        self.last_sync_time = time.time()     # Timestamp of last successful sync
        self.last_valid_offset = 0            # Backup offset for when server is unreachable
        
        # GPS server connection parameters
        self.gps_server_ip = gps_server_ip     # Target GPS server IP address
        self.gps_server_port = gps_server_port # Target GPS server port
        
        # Vehicle identification for logging
        self.vehicle_id = vehicle_id or 0
        
        # GPS-specific logging setup
        self.logger = get_gps_logger(self.vehicle_id)

    def request_gps_time(self):
        """
        Request current GPS time from the central time server.
        
        Establishes a UDP connection to the GPS time server and requests the current
        synchronized time. Includes error handling for network failures and simulates
        realistic GPS behavior by adding random jitter to the received time.
        
        The method implements a timeout mechanism to avoid blocking and provides
        fallback behavior using the last known valid offset when the server is
        unreachable.
        
        Returns:
            float: GPS timestamp with simulated jitter, or estimated time using
                   last valid offset if server communication fails
                   
        Raises:
            Exception: Network or communication errors are caught and logged,
                      fallback time is returned instead of propagating exceptions
        """
        # Create UDP socket for communication with GPS server
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.5)  # 500ms timeout to prevent blocking
        
        try:
            # Send time request to GPS server
            sock.sendto(b"time_request", (self.gps_server_ip, self.gps_server_port))
            
            # Receive GPS time response
            data, _ = sock.recvfrom(1024)  # 1KB buffer should be sufficient
            gps_time = ujson.loads(data.decode())
            
            # Simulate realistic GPS jitter (±10ms)
            # Real GPS systems have inherent timing uncertainties due to:
            # - Atmospheric delays, multipath effects, clock drift
            gps_time += random.uniform(-0.01, 0.01)
            
            return gps_time
            
        except Exception as e:
            # Log communication error and use fallback mechanism
            self.logger.warning(f"GPS server connection failed: {e}, using fallback time")
            # Return estimated time using last known valid offset
            return time.time() + self.last_valid_offset
            
        finally:
            # Always close socket to free resources
            sock.close()

    def sync_with_gps(self):
        """
        Synchronize local time with GPS server and update time offset.
        
        Performs a complete synchronization cycle by requesting GPS time,
        calculating the time offset between GPS and local system time,
        and updating internal synchronization state.
        
        This method should be called periodically to maintain accurate
        time synchronization across the vehicle fleet. The calculated
        offset is used by get_synced_time() to provide synchronized
        timestamps for fleet coordination.
        
        Side Effects:
            - Updates gps_time_offset with new calculated offset
            - Updates last_valid_offset for fallback scenarios
            - Updates last_sync_time timestamp
            - Logs synchronization results
        """
        # Request current GPS time from server
        gps_time = self.request_gps_time()
        local_time = time.time()
        
        # Calculate time offset (GPS time - local system time)
        self.gps_time_offset = gps_time - local_time
        
        # Store this offset as backup for when server becomes unreachable
        self.last_valid_offset = self.gps_time_offset
        
        # Record when this synchronization occurred
        self.last_sync_time = local_time
        
        # Log synchronization results for monitoring and debugging
        self.logger.info(f"GPS Time: {gps_time:.3f}, Local Time: {local_time:.3f}, Offset: {self.gps_time_offset:.3f} sec")

    def get_synced_time(self):
        """
        Get current synchronized time based on GPS offset.
        
        Returns the current time adjusted by the GPS time offset calculated
        during the last synchronization. This provides a synchronized timestamp
        that should be consistent across all vehicles in the fleet that are
        using the same GPS time server.
        
        The synchronized time is calculated as:
        synchronized_time = local_system_time + gps_time_offset
        
        Returns:
            float: Current synchronized timestamp that matches GPS time reference
            
        Note:
            The accuracy of this time depends on:
            - How recently sync_with_gps() was called
            - Stability of local system clock since last sync
            - Network latency variations since last sync
        """
        return time.time() + self.gps_time_offset