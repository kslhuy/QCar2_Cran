#!/usr/bin/env python3
"""
Simple GPS Time Server for Vehicle Fleet Coordination

This server provides a centralized time reference for vehicle fleet synchronization.
It simulates a GPS time source and handles time requests from vehicles.

Usage:
    python gps_time_server.py [--port PORT] [--host HOST]

Default: Listens on 127.0.0.1:8001
"""

import socket
import json as ujson
import time
import threading
import logging
import argparse
from typing import Dict, Any
import random


class GPSTimeServer:
    """
    GPS Time Server that provides synchronized time reference for vehicle fleet.
    
    Features:
    - UDP-based time service
    - Configurable time drift simulation
    - Request/response logging
    - Thread-safe operations
    """
    
    def __init__(self, host: str = "127.0.0.1", port: int = 8001):
        """
        Initialize GPS Time Server.
        
        Args:
            host: IP address to bind to
            port: Port to listen on
        """
        self.host = host
        self.port = port
        self.running = threading.Event()
        
        # GPS simulation parameters
        self.base_time_offset = 0.0  # Base offset from system time
        self.drift_rate = 1e-6  # Simulated clock drift (seconds per second)
        self.noise_amplitude = 0.001  # Random noise amplitude (±1ms)
        self.start_time = time.time()
        
        # Statistics
        self.stats = {
            'requests_served': 0,
            'start_time': time.time(),
            'last_request_time': 0
        }
        
        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger("GPSTimeServer")
        
        # Create UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
    def get_gps_time(self) -> float:
        """
        Generate simulated GPS time with realistic characteristics.
        
        Returns:
            GPS time as float timestamp
        """
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # Apply simulated drift
        drift_offset = self.drift_rate * elapsed
        
        # Add random noise (simulates GPS jitter/multipath)
        noise = random.uniform(-self.noise_amplitude, self.noise_amplitude)
        
        # Calculate GPS time
        gps_time = current_time + self.base_time_offset + drift_offset + noise
        
        return gps_time
    
    def handle_time_request(self, data: bytes, client_addr: tuple) -> None:
        """
        Handle incoming time request from vehicle.
        
        Args:
            data: Request data from client
            client_addr: Client address tuple (IP, port)
        """
        try:
            # Generate GPS time
            gps_time = self.get_gps_time()
            
            # Create response
            response = ujson.dumps(gps_time).encode()
            
            # Send response
            self.socket.sendto(response, client_addr)
            
            # Update statistics
            self.stats['requests_served'] += 1
            self.stats['last_request_time'] = time.time()
            
            self.logger.debug(f"Served time request from {client_addr}: {gps_time:.6f}")
            
        except Exception as e:
            self.logger.error(f"Error handling time request from {client_addr}: {e}")
    
    def handle_status_request(self, client_addr: tuple) -> None:
        """
        Handle status request from monitoring tools.
        
        Args:
            client_addr: Client address tuple (IP, port)
        """
        try:
            current_time = time.time()
            uptime = current_time - self.stats['start_time']
            
            status = {
                'server_type': 'GPS_Time_Server',
                'version': '1.0',
                'uptime_seconds': uptime,
                'requests_served': self.stats['requests_served'],
                'current_gps_time': self.get_gps_time(),
                'current_system_time': current_time,
                'drift_rate': self.drift_rate,
                'noise_amplitude': self.noise_amplitude,
                'last_request_time': self.stats['last_request_time']
            }
            
            response = ujson.dumps(status).encode()
            self.socket.sendto(response, client_addr)
            
            self.logger.info(f"Status request served to {client_addr}")
            
        except Exception as e:
            self.logger.error(f"Error handling status request from {client_addr}: {e}")
    
    def start_server(self) -> None:
        """Start the GPS time server."""
        try:
            # Bind socket
            self.socket.bind((self.host, self.port))
            self.socket.settimeout(1.0)  # 1 second timeout for shutdown responsiveness
            
            self.logger.info(f"GPS Time Server started on {self.host}:{self.port}")
            self.logger.info(f"Drift rate: {self.drift_rate} s/s, Noise: ±{self.noise_amplitude*1000:.1f}ms")
            
            self.running.set()
            
            # Main server loop
            while self.running.is_set():
                try:
                    # Receive request
                    data, client_addr = self.socket.recvfrom(1024)
                    
                    # Decode request
                    try:
                        request = data.decode().strip()
                    except UnicodeDecodeError:
                        request = "time_request"  # Default to time request
                    
                    # Route request
                    if request == "time_request":
                        self.handle_time_request(data, client_addr)
                    elif request == "status_request":
                        self.handle_status_request(client_addr)
                    else:
                        # Unknown request, treat as time request for compatibility
                        self.handle_time_request(data, client_addr)
                        
                except socket.timeout:
                    # Timeout is expected for shutdown responsiveness
                    continue
                except Exception as e:
                    self.logger.error(f"Server loop error: {e}")
                    
        except Exception as e:
            self.logger.error(f"Failed to start server: {e}")
            raise
        finally:
            self.cleanup()
    
    def stop_server(self) -> None:
        """Stop the GPS time server."""
        self.logger.info("Stopping GPS Time Server...")
        self.running.clear()
    
    def cleanup(self) -> None:
        """Clean up server resources."""
        try:
            self.socket.close()
            self.logger.info("GPS Time Server stopped and cleaned up")
        except Exception as e:
            self.logger.error(f"Cleanup error: {e}")
    
    def print_stats(self) -> None:
        """Print server statistics."""
        current_time = time.time()
        uptime = current_time - self.stats['start_time']
        
        print(f"\n=== GPS Time Server Statistics ===")
        print(f"Uptime: {uptime:.1f} seconds")
        print(f"Requests served: {self.stats['requests_served']}")
        print(f"Average requests/sec: {self.stats['requests_served']/max(uptime, 1):.2f}")
        print(f"Current GPS time: {self.get_gps_time():.6f}")
        print(f"Current system time: {current_time:.6f}")
        print(f"GPS offset: {self.get_gps_time() - current_time:.6f} seconds")
        print(f"Drift rate: {self.drift_rate} s/s")
        print(f"Noise amplitude: ±{self.noise_amplitude*1000:.1f}ms")


def main():
    """Main function for GPS Time Server."""
    parser = argparse.ArgumentParser(description="GPS Time Server for Vehicle Fleet")
    parser.add_argument("--host", default="127.0.0.1", help="Host IP to bind to")
    parser.add_argument("--port", type=int, default=8001, help="Port to listen on")
    parser.add_argument("--drift", type=float, default=1e-6, help="Simulated clock drift rate (s/s)")
    parser.add_argument("--noise", type=float, default=0.001, help="GPS noise amplitude (seconds)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Enable verbose logging")
    
    args = parser.parse_args()
    
    # Set logging level
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Create and configure server
    server = GPSTimeServer(host=args.host, port=args.port)
    server.drift_rate = args.drift
    server.noise_amplitude = args.noise
    
    try:
        print(f"Starting GPS Time Server on {args.host}:{args.port}")
        print(f"Drift rate: {args.drift} s/s")
        print(f"Noise amplitude: ±{args.noise*1000:.1f}ms")
        print("Press Ctrl+C to stop\n")
        
        # Start server in a thread so we can handle keyboard interrupt
        server_thread = threading.Thread(target=server.start_server, daemon=True)
        server_thread.start()
        
        # Wait for keyboard interrupt
        try:
            while server_thread.is_alive():
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutdown requested...")
            server.stop_server()
            server_thread.join(timeout=2)
            
        # Print final statistics
        server.print_stats()
        
    except Exception as e:
        print(f"Server error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
