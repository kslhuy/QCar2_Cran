"""
WebSocket Bridge for QCar Ground Station
Bridges WebSocket connections from browser to TCP connections to QCars

Architecture:
    Browser (React) <--WebSocket--> Bridge <--TCP--> QCar(s)

Usage:
    python websocket_bridge.py [--port 8080] [--config fleet_config.yaml]
"""

import asyncio
import json
import argparse
import os
import sys
from typing import Dict, Optional, Set
from dataclasses import dataclass, field

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
except ImportError:
    print("ERROR: websockets library not installed. Run: pip install websockets")
    sys.exit(1)

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML not installed. Run: pip install pyyaml")
    sys.exit(1)


@dataclass
class VehicleConnection:
    """Represents a TCP connection to a single QCar"""
    car_id: int
    ip: str
    port: int
    reader: Optional[asyncio.StreamReader] = None
    writer: Optional[asyncio.StreamWriter] = None
    connected: bool = False
    recv_buffer: str = ""
    
    
@dataclass
class BridgeConfig:
    """Bridge configuration"""
    websocket_host: str = "0.0.0.0"
    websocket_port: int = 8080
    vehicles: Dict[str, VehicleConnection] = field(default_factory=dict)
    

class WebSocketBridge:
    """
    WebSocket to TCP Bridge for QCar Ground Station
    
    Handles:
    - Multiple browser clients via WebSocket
    - Multiple QCar connections via TCP
    - Bidirectional message routing
    """
    
    def __init__(self, config: BridgeConfig):
        self.config = config
        self.vehicles: Dict[str, VehicleConnection] = config.vehicles
        self.websocket_clients: Set[WebSocketServerProtocol] = set()
        self._running = False
        
    async def start(self):
        """Start the bridge server"""
        self._running = True
        
        print("=" * 60)
        print(" QCar WebSocket Bridge")
        print("=" * 60)
        print(f"WebSocket server: ws://{self.config.websocket_host}:{self.config.websocket_port}")
        print(f"Configured vehicles: {len(self.vehicles)}")
        for vid, v in self.vehicles.items():
            print(f"  • {vid}: {v.ip}:{v.port}")
        print("=" * 60)
        
        # Start WebSocket server
        server = await websockets.serve(
            self._handle_websocket_client,
            self.config.websocket_host,
            self.config.websocket_port
        )
        
        print(f"\n[READY] WebSocket Bridge started on ws://localhost:{self.config.websocket_port}")
        print("Waiting for browser connections...")
        
        # Connect to vehicles in background
        asyncio.create_task(self._connect_to_vehicles())
        
        # Keep running
        await server.wait_closed()
    
    async def _connect_to_vehicles(self):
        """Connect to all configured QCar vehicles via TCP"""
        for vehicle_id, vehicle in self.vehicles.items():
            asyncio.create_task(self._connect_vehicle(vehicle_id, vehicle))
    
    async def _connect_vehicle(self, vehicle_id: str, vehicle: VehicleConnection):
        """Connect to a single vehicle and maintain connection"""
        while self._running:
            if not vehicle.connected:
                try:
                    print(f"[TCP] Connecting to {vehicle_id} at {vehicle.ip}:{vehicle.port}...")
                    reader, writer = await asyncio.wait_for(
                        asyncio.open_connection(vehicle.ip, vehicle.port),
                        timeout=5.0
                    )
                    vehicle.reader = reader
                    vehicle.writer = writer
                    vehicle.connected = True
                    print(f"[TCP] ✓ Connected to {vehicle_id}")
                    
                    # Notify browser clients
                    await self._broadcast_to_browsers({
                        "type": "vehicle_status",
                        "vehicle_id": vehicle_id,
                        "status": "connected"
                    })
                    
                    # Start receiving telemetry from this vehicle
                    asyncio.create_task(self._receive_from_vehicle(vehicle_id, vehicle))
                    
                except asyncio.TimeoutError:
                    print(f"[TCP] ✗ Connection to {vehicle_id} timed out")
                except ConnectionRefusedError:
                    print(f"[TCP] ✗ Connection to {vehicle_id} refused")
                except Exception as e:
                    print(f"[TCP] ✗ Error connecting to {vehicle_id}: {e}")
            
            await asyncio.sleep(5.0)  # Retry every 5 seconds
    
    async def _receive_from_vehicle(self, vehicle_id: str, vehicle: VehicleConnection):
        """Receive telemetry data from a vehicle and forward to browsers"""
        try:
            while vehicle.connected and self._running:
                try:
                    data = await asyncio.wait_for(
                        vehicle.reader.read(4096),
                        timeout=1.0
                    )
                    
                    if not data:
                        # Connection closed
                        break
                    
                    # Parse newline-delimited JSON
                    vehicle.recv_buffer += data.decode('utf-8')
                    while '\n' in vehicle.recv_buffer:
                        line, vehicle.recv_buffer = vehicle.recv_buffer.split('\n', 1)
                        if line.strip():
                            try:
                                telemetry = json.loads(line)
                                # Add vehicle ID and forward to browsers
                                telemetry['vehicle_id'] = vehicle_id
                                telemetry['type'] = 'telemetry'
                                await self._broadcast_to_browsers(telemetry)
                            except json.JSONDecodeError:
                                print(f"[TCP] Invalid JSON from {vehicle_id}: {line[:50]}")
                                
                except asyncio.TimeoutError:
                    continue  # Normal timeout, just keep waiting
                    
        except Exception as e:
            print(f"[TCP] Error receiving from {vehicle_id}: {e}")
        finally:
            vehicle.connected = False
            vehicle.recv_buffer = ""
            if vehicle.writer:
                vehicle.writer.close()
                try:
                    await vehicle.writer.wait_closed()
                except:
                    pass
            print(f"[TCP] Disconnected from {vehicle_id}")
            
            # Notify browsers
            await self._broadcast_to_browsers({
                "type": "vehicle_status",
                "vehicle_id": vehicle_id,
                "status": "disconnected"
            })
    
    async def _handle_websocket_client(self, websocket: WebSocketServerProtocol, path: str = ""):
        """Handle a WebSocket connection from browser"""
        self.websocket_clients.add(websocket)
        client_addr = websocket.remote_address
        print(f"[WS] Browser connected from {client_addr}")
        
        # Send initial vehicle status
        for vid, v in self.vehicles.items():
            await websocket.send(json.dumps({
                "type": "vehicle_status",
                "vehicle_id": vid,
                "status": "connected" if v.connected else "disconnected",
                "ip": v.ip,
                "port": v.port
            }))
        
        try:
            async for message in websocket:
                await self._handle_browser_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            print(f"[WS] Browser disconnected from {client_addr}")
        except Exception as e:
            print(f"[WS] Error handling browser client: {e}")
        finally:
            self.websocket_clients.discard(websocket)
    
    async def _handle_browser_message(self, websocket: WebSocketServerProtocol, message: str):
        """Handle a message from the browser"""
        try:
            data = json.loads(message)
            msg_type = data.get('type', 'command')
            target = data.get('target', 'all')  # vehicle ID or 'all'
            
            print(f"[WS] Received: {data.get('action', msg_type)} -> {target}")
            
            if msg_type == 'ping':
                # Heartbeat
                await websocket.send(json.dumps({"type": "pong"}))
                return
            
            if msg_type == 'get_status':
                # Return current vehicle status
                for vid, v in self.vehicles.items():
                    await websocket.send(json.dumps({
                        "type": "vehicle_status",
                        "vehicle_id": vid,
                        "status": "connected" if v.connected else "disconnected"
                    }))
                return
            
            # Forward command to vehicle(s)
            if target == 'all':
                for vid in self.vehicles:
                    await self._send_to_vehicle(vid, data)
            else:
                await self._send_to_vehicle(target, data)
                
            # Acknowledge command
            await websocket.send(json.dumps({
                "type": "command_ack",
                "action": data.get('action'),
                "target": target,
                "success": True
            }))
            
        except json.JSONDecodeError:
            print(f"[WS] Invalid JSON from browser: {message[:50]}")
            await websocket.send(json.dumps({
                "type": "error",
                "message": "Invalid JSON"
            }))
        except Exception as e:
            print(f"[WS] Error handling message: {e}")
            await websocket.send(json.dumps({
                "type": "error", 
                "message": str(e)
            }))
    
    async def _send_to_vehicle(self, vehicle_id: str, data: dict):
        """Send a command to a specific vehicle via TCP"""
        if vehicle_id not in self.vehicles:
            print(f"[TCP] Unknown vehicle: {vehicle_id}")
            return False
        
        vehicle = self.vehicles[vehicle_id]
        if not vehicle.connected or not vehicle.writer:
            print(f"[TCP] Vehicle {vehicle_id} not connected")
            return False
        
        try:
            # Format as newline-delimited JSON (matching QCar protocol)
            message = json.dumps(data) + '\n'
            vehicle.writer.write(message.encode('utf-8'))
            await vehicle.writer.drain()
            print(f"[TCP] Sent to {vehicle_id}: {data.get('action', 'data')}")
            return True
        except Exception as e:
            print(f"[TCP] Error sending to {vehicle_id}: {e}")
            vehicle.connected = False
            return False
    
    async def _broadcast_to_browsers(self, data: dict):
        """Send data to all connected browser clients"""
        if not self.websocket_clients:
            return
        
        message = json.dumps(data)
        disconnected = set()
        
        for client in self.websocket_clients:
            try:
                await client.send(message)
            except:
                disconnected.add(client)
        
        # Clean up disconnected clients
        self.websocket_clients -= disconnected
    
    def stop(self):
        """Stop the bridge"""
        self._running = False
        print("\n[BRIDGE] Shutting down...")


def load_config(config_path: str) -> BridgeConfig:
    """Load configuration from fleet_config.yaml"""
    config = BridgeConfig()
    
    if not os.path.exists(config_path):
        print(f"[WARN] Config file not found: {config_path}")
        print("Using default configuration with example vehicles")
        # Add example vehicles for testing
        config.vehicles = {
            "qcar-01": VehicleConnection(car_id=0, ip="192.168.137.102", port=5000),
            "qcar-02": VehicleConnection(car_id=1, ip="192.168.137.208", port=5001),
        }
        return config
    
    try:
        with open(config_path, 'r') as f:
            yaml_config = yaml.safe_load(f)
        
        # Parse ground station config
        gs = yaml_config.get('ground_station', {})
        base_port = gs.get('base_port', 5000)
        
        # Parse vehicles
        vehicles = yaml_config.get('vehicles', [])
        for v in vehicles:
            if not v.get('enabled', True):
                continue
            car_id = v['car_id']
            vehicle_id = f"qcar-{car_id:02d}"
            config.vehicles[vehicle_id] = VehicleConnection(
                car_id=car_id,
                ip=v['ip'],
                port=base_port + car_id
            )
        
        print(f"[CONFIG] Loaded {len(config.vehicles)} vehicles from {config_path}")
        
    except Exception as e:
        print(f"[ERROR] Failed to load config: {e}")
        sys.exit(1)
    
    return config


def main():
    parser = argparse.ArgumentParser(
        description="WebSocket Bridge for QCar Ground Station",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python websocket_bridge.py
    python websocket_bridge.py --port 8080
    python websocket_bridge.py --config ../fleet_config.yaml
        """
    )
    
    parser.add_argument(
        '-p', '--port',
        type=int,
        default=8080,
        help='WebSocket server port (default: 8080)'
    )
    
    parser.add_argument(
        '-c', '--config',
        type=str,
        default='../fleet_config.yaml',
        help='Path to fleet_config.yaml'
    )
    
    parser.add_argument(
        '--host',
        type=str,
        default='0.0.0.0',
        help='WebSocket server host (default: 0.0.0.0)'
    )
    
    args = parser.parse_args()
    
    # Resolve config path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = args.config
    if not os.path.isabs(config_path):
        config_path = os.path.normpath(os.path.join(script_dir, config_path))
    
    # Load configuration
    config = load_config(config_path)
    config.websocket_host = args.host
    config.websocket_port = args.port
    
    # Create and run bridge
    bridge = WebSocketBridge(config)
    
    try:
        asyncio.run(bridge.start())
    except KeyboardInterrupt:
        bridge.stop()
        print("\n[BRIDGE] Stopped by user")


if __name__ == "__main__":
    main()
