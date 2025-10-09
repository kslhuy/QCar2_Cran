""" 
Remote Controlled Physical QCar 2 - Car 0

This version connects to a remote host PC for centralized control.
Run this script on the physical QCar hardware.

Configuration:
- Use --host to set the host PC's IP address (default: 127.0.0.1)
- Use --port to set the base port number (default: 5000)
- Use --car-id to set the car ID (default: 0)

# # Use defaults (localhost, port 5000, car 0)
# python vehicle_control_remote.py

# # Custom host IP
# python vehicle_control_remote.py --host 192.168.1.100

# # Custom port and car ID
# python vehicle_control_remote.py --port 6000 --car-id 2

# # All custom
# python vehicle_control_remote.py --host 192.168.1.100 --port 6000 --car-id 1

# # Show help
# python vehicle_control_remote.py --help

 """
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports

import os
import signal
import numpy as np
from threading import Thread, Event
import time
import socket
import json
import argparse

from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.utilities.math import wrap_to_pi
from hal.content.qcar_functions import QCarEKF
from hal.products.mats import SDCSRoadMap




#================ Command Line Arguments ================
parser = argparse.ArgumentParser(description='Remote Controlled QCar 2')
parser.add_argument('--host', type=str, default='127.0.0.1', 
                    help='Host PC IP address (default: 127.0.0.1)')
parser.add_argument('--port', type=int, default=5000, 
                    help='Base port number (default: 5000)')
parser.add_argument('--car-id', type=int, default=0, 
                    help='Car ID (0, 1, 2, ...) (default: 0)')
args = parser.parse_args()

#================ Remote Connection Configuration ================
HOST_PC_IP = args.host
BASE_PORT = args.port
CAR_ID = args.car_id

#================ Experiment Configuration ================
# ===== Timing Parameters
tf = 6000
startDelay = 1
controllerUpdateRate = 100

# ===== Speed Controller Parameters (can be overridden by remote commands)
v_ref = 0.0  # Start stopped, wait for remote command
K_p = 0.1
K_i = 1

# ===== Steering Controller Parameters
enableSteeringControl = True
K_stanley = 1
nodeSequence = [10, 4, 20, 13, 10]  # D13efault path

#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Initial setup

# Simulation/Physical QCar setup
if enableSteeringControl:
    roadmap = SDCSRoadMap(leftHandTraffic=False)
    waypointSequence = roadmap.generate_path(nodeSequence)
    initialPose = roadmap.get_node_pose(nodeSequence[0]).squeeze()
    print(initialPose)
else:
    initialPose = [0, 0, 0]

if not IS_PHYSICAL_QCAR:
    # Simulation mode: get robot ports from multi-agent config
    from qvl.multi_agent import readRobots
    robotsDir = readRobots()
    # Use car ID to select correct robot
    car_key = f"QC2_{CAR_ID}"
    if car_key not in robotsDir:
        print(f"ERROR: Simulated car {car_key} not found in robotsDir!")
        exit(1)
    CarSim = robotsDir[car_key]
    calibrate = False
else:
    calibrate = 'y' in input('Do you want to recalibrate?(y/n): ')

# Calibration pose (can be changed as needed)
calibrationPose = [0, 2, -np.pi/2]

# Remote control state
remote_control_enabled = False
remote_commands_queue = []

# Used to enable safe keyboard triggered shutdown
kill_event = Event()
ctrl_c_count = 0

def sig_handler(signum, frame):
    global ctrl_c_count
    ctrl_c_count += 1
    print(f"\n\n[Car {CAR_ID}] Received SIGINT (Ctrl+C #{ctrl_c_count}). Shutting down...")
    kill_event.set()
    
    # Force exit after 2 Ctrl+C presses
    if ctrl_c_count >= 2:
        print(f"[Car {CAR_ID}] Force exit due to multiple Ctrl+C!")
        import os
        os._exit(1)

signal.signal(signal.SIGINT, sig_handler)
#endregion

class NetworkClient:
    """Handles communication with the host PC"""
    
    def __init__(self, host_ip: str, port: int, car_id: int):
        self.host_ip = host_ip
        self.port = port
        self.car_id = car_id
        self.socket = None
        self.connected = False
        self.running = False
        
    def connect(self, timeout: int = 5):
        """Connect to the host PC"""
        print(f"[Car {self.car_id}] Attempting connection to Host PC at {self.host_ip}:{self.port}...")
        
        try:
            if self.socket:
                self.socket.close()
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(timeout)
            self.socket.connect((self.host_ip, self.port))
            self.socket.settimeout(None)  # Remove timeout for normal operation
            self.connected = True
            print(f"[Car {self.car_id}] Connected to Host PC!")
            return True
        except Exception as e:
            print(f"[Car {self.car_id}] Connection failed: {e}")
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
                self.socket = None
            self.connected = False
            return False
    
    def send_telemetry(self, data: dict):
        """Send telemetry data to host PC"""
        if not self.connected:
            return
        
        try:
            data['car_id'] = self.car_id
            data['timestamp'] = time.time()
            msg = json.dumps(data) + '\n'
            self.socket.sendall(msg.encode('utf-8'))
        except Exception as e:
            print(f"[Car {self.car_id}] Error sending telemetry: {e}")
            self.connected = False
    
    def receive_commands(self):
        """Receive commands from host PC (non-blocking)"""
        if not self.connected:
            return None
        
        try:
            self.socket.setblocking(False)
            data = self.socket.recv(4096).decode('utf-8')
            if data:
                # Handle multiple commands separated by newlines
                commands = data.strip().split('\n')
                result = []
                for cmd_str in commands:
                    if cmd_str:
                        try:
                            cmd = json.loads(cmd_str)
                            result.append(cmd)
                        except json.JSONDecodeError:
                            pass
                return result if result else None
        except BlockingIOError:
            return None
        except Exception as e:
            print(f"[Car {self.car_id}] Error receiving commands: {e}")
            self.connected = False
            return None
        finally:
            self.socket.setblocking(True)
    
    def close(self):
        """Close connection"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.connected = False


class SpeedController:
    def __init__(self, kp=0, ki=0):
        self.maxThrottle = 0.3
        self.kp = kp
        self.ki = ki
        self.ei = 0

    def update(self, v, v_ref, dt):
        e = v_ref - v
        self.ei += dt*e
        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )

class SteeringController:
    def __init__(self, waypoints, k=1, cyclic=True):
        self.maxSteeringAngle = np.pi/6
        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0
        self.k = k
        self.cyclic = cyclic
        self.p_ref = (0, 0)
        self.th_ref = 0

    def update(self, p, th, speed):
        wp_1 = self.wp[:, np.mod(self.wpi, self.N-1)]
        wp_2 = self.wp[:, np.mod(self.wpi+1, self.N-1)]
        
        v = wp_2 - wp_1
        v_mag = np.linalg.norm(v)
        try:
            v_uv = v / v_mag
        except ZeroDivisionError:
            return 0

        tangent = np.arctan2(v_uv[1], v_uv[0])
        s = np.dot(p-wp_1, v_uv)

        if s >= v_mag:
            if self.cyclic or self.wpi < self.N-2:
                self.wpi += 1

        ep = wp_1 + v_uv*s
        ct = ep - p
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)
        ect = np.linalg.norm(ct) * np.sign(dir)
        psi = wrap_to_pi(tangent-th)

        self.p_ref = ep
        self.th_ref = tangent

        return np.clip(
            wrap_to_pi(psi + np.arctan2(self.k*ect, speed)),
            -self.maxSteeringAngle,
            self.maxSteeringAngle)


def controlLoop():
    global v_ref, enableSteeringControl, nodeSequence, waypointSequence
    
    u = 0
    delta = 0
    telemetry_send_counter = 0
    
    # Initialize network client
    network = NetworkClient(HOST_PC_IP, BASE_PORT + CAR_ID, CAR_ID)
    
    # Wait for connection to host PC
    print(f"[Car {CAR_ID}] Waiting for connection to host PC...")
    while not network.connect() and not kill_event.is_set():
        print(f"[Car {CAR_ID}] Connection failed, retrying in 2 seconds...")
        time.sleep(2)
    
    if kill_event.is_set():
        print(f"[Car {CAR_ID}] Shutdown requested before connection established")
        return
    
    print(f"[Car {CAR_ID}] Connection established! Initializing car systems...")
    
    # Wait for initialization commands from host PC
    print(f"[Car {CAR_ID}] Waiting for initialization commands...")
    initialization_complete = False
    
    while not initialization_complete and not kill_event.is_set():
        commands = network.receive_commands()
        if commands:
            for cmd in commands:
                cmd_type = cmd.get('type', '')
                if cmd_type == 'initialize' or cmd_type == 'start':
                    initialization_complete = True
                    print(f"[Car {CAR_ID}] Initialization complete!")
                    break
                elif cmd_type == 'set_params':
                    # Allow parameter setting during initialization
                    if 'v_ref' in cmd:
                        v_ref = cmd['v_ref']
                        print(f"[Car {CAR_ID}] Initial velocity set to {v_ref}")
                    if 'node_sequence' in cmd:
                        nodeSequence = cmd['node_sequence']
                        waypointSequence = roadmap.generate_path(nodeSequence)
                        print(f"[Car {CAR_ID}] Initial path set: {nodeSequence}")
        time.sleep(0.1)  # Small delay to avoid busy waiting
    
    if kill_event.is_set():
        print(f"[Car {CAR_ID}] Shutdown requested during initialization")
        network.close()
        return
    
    # Controller initialization
    speedController = SpeedController(kp=K_p, ki=K_i)
    if enableSteeringControl:
        steeringController = SteeringController(
            waypoints=waypointSequence,
            k=K_stanley
        )
    
    # QCar interface setup
    if not IS_PHYSICAL_QCAR:
        # Use simulation ports from CarSim
        qcar = QCar(readMode=1, frequency=controllerUpdateRate, hilPort=CarSim["hilPort"])
        if enableSteeringControl or calibrate:
            ekf = QCarEKF(x_0=initialPose)
            gps = QCarGPS(initialPose=calibrationPose, calibrate=calibrate,
                          gpsPort=CarSim["gpsPort"], lidarIdealPort=CarSim["lidarIdealPort"])
        else:
            gps = memoryview(b'')
    else:
        qcar = QCar(readMode=1, frequency=controllerUpdateRate)
        if enableSteeringControl or calibrate:
            ekf = QCarEKF(x_0=initialPose)
            gps = QCarGPS(initialPose=calibrationPose, calibrate=calibrate)
        else:
            gps = memoryview(b'')
    
    with qcar, gps:
        t0 = time.time()
        t = 0
        running = True
        
        while running and not kill_event.is_set():
            # Check for kill signal first for faster response
            if kill_event.is_set():
                print(f"[Car {CAR_ID}] Shutdown signal received")
                break
                
            tp = t
            t = time.time() - t0
            dt = t-tp
            
            # Check if we've exceeded maximum time
            if t >= tf + startDelay:
                print(f"[Car {CAR_ID}] Maximum time reached")
                break
            
            # Check for remote commands
            if network.connected:
                commands = network.receive_commands()
                if commands:
                    for cmd in commands:
                        cmd_type = cmd.get('type', '')
                        
                        if cmd_type == 'stop':
                            v_ref = 0.0
                            print(f"[Car {CAR_ID}] STOP command received")
                        
                        elif cmd_type == 'start':
                            v_ref = 0.9
                            print(f"[Car {CAR_ID}] START command received")
                        
                        elif cmd_type == 'set_params':
                            if 'v_ref' in cmd:
                                v_ref = cmd['v_ref']
                                print(f"[Car {CAR_ID}] Velocity set to {v_ref}")
                            
                            if 'node_sequence' in cmd:
                                nodeSequence = cmd['node_sequence']
                                waypointSequence = roadmap.generate_path(nodeSequence)
                                steeringController = SteeringController(
                                    waypoints=waypointSequence,
                                    k=K_stanley
                                )
                                print(f"[Car {CAR_ID}] Path updated: {nodeSequence}")
                        
                        elif cmd_type == 'emergency':
                            v_ref = 0.0
                            running = False
                            print(f"[Car {CAR_ID}] EMERGENCY STOP!")
            
            # Check kill signal before sensor reading
            if kill_event.is_set():
                print(f"[Car {CAR_ID}] Shutdown signal received during sensor read")
                break
                
            # Read from sensors and update state estimates
            qcar.read()
            if enableSteeringControl:
                if gps.readGPS():
                    y_gps = np.array([
                        gps.position[0],
                        gps.position[1],
                        gps.orientation[2]
                    ])
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        y_gps,
                        qcar.gyroscope[2],
                    )
                else:
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        None,
                        qcar.gyroscope[2],
                    )

                x = ekf.x_hat[0,0]
                y = ekf.x_hat[1,0]
                th = ekf.x_hat[2,0]
                p = (np.array([x, y]) + np.array([np.cos(th), np.sin(th)]) * 0.2)
            else:
                x, y, th = 0, 0, 0
            
            v = qcar.motorTach
            
            # Update controllers
            if t < startDelay:
                u = 0
                delta = 0
            else:
                u = speedController.update(v, v_ref, dt)
                if enableSteeringControl:
                    delta = steeringController.update(p, th, v)
                else:
                    delta = 0
            
            qcar.write(u, delta)
            
            # Send telemetry to host PC (at 10 Hz)
            telemetry_send_counter += 1
            if network.connected and telemetry_send_counter >= controllerUpdateRate / 10:
                telemetry = {
                    'x': float(x) if enableSteeringControl else 0.0,
                    'y': float(y) if enableSteeringControl else 0.0,
                    'th': float(th) if enableSteeringControl else 0.0,
                    'v': float(v),
                    'v_ref': float(v_ref),
                    'u': float(u),
                    'delta': float(delta),
                    't': float(t)
                }
                network.send_telemetry(telemetry)
                telemetry_send_counter = 0
        
        # Stop the car
        print(f"[Car {CAR_ID}] Stopping the car...")
        qcar.read_write_std(throttle=0, steering=0)
    
    network.close()
    print(f"[Car {CAR_ID}] Control loop ended")


# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

if __name__ == '__main__':
    print(f"\n{'='*60}")
    print(f"Remote Controlled QCar - Car {CAR_ID}")
    print(f"{'='*60}")
    print(f"Will connect to: {HOST_PC_IP}:{BASE_PORT + CAR_ID}")
    print(f"{'='*60}\n")
    
    controlThread = Thread(target=controlLoop)
    controlThread.start()

    try:
        # Use a loop with timeout so we can check for kill_event
        while controlThread.is_alive() and not kill_event.is_set():
            controlThread.join(timeout=0.5)
        
        if kill_event.is_set():
            print(f"\n[Car {CAR_ID}] Shutdown signal received, waiting for thread to exit...")
            controlThread.join(timeout=5)
            if controlThread.is_alive():
                print(f"[Car {CAR_ID}] Control thread did not exit gracefully")
    except KeyboardInterrupt:
        print(f"\n\n[Car {CAR_ID}] Main thread interrupted. Shutting down...")
        kill_event.set()
        # Give the control thread time to exit gracefully
        controlThread.join(timeout=5)
        if controlThread.is_alive():
            print(f"[Car {CAR_ID}] Control thread did not exit gracefully")

    # Terminate simulation models if running in simulation
    if not IS_PHYSICAL_QCAR:
        from qvl.real_time import QLabsRealTime
        cmd = QLabsRealTime().terminate_all_real_time_models()
        time.sleep(1)
        cmd = QLabsRealTime().terminate_all_real_time_models()

    input('Experiment complete. Press any key to exit...')
