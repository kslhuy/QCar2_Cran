# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports

"""
vehicle_control.py

Skills acivity code for vehicle control lab guide.
Students will implement a vehicle speed and steering controller.
Please review Lab Guide - vehicle control PDF
"""
import signal
import numpy as np
from threading import Thread, Event
import time
import random
import socket
import json
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR

from pal.utilities.math import wrap_to_pi
from hal.content.qcar_functions import QCarEKF
from hal.products.mats import SDCSRoadMap
import argparse
from utils import YOLOReceiver, YOLODriveLogic

parser = argparse.ArgumentParser(prog='Vehicle control')
parser.add_argument('-c','--calibrate', default=False)
parser.add_argument('-n','--node_configuration', default=0)
parser.add_argument('--host', type=str, default=None, 
                    help='Host PC IP address for remote control (optional)')
parser.add_argument('--port', type=int, default=5000, 
                    help='Base port number (default: 5000)')
parser.add_argument('--car-id', type=int, default=0, 
                    help='Car ID (0, 1, 2, ...) (default: 0)')

args = parser.parse_args()
node_options = args.node_configuration

# Remote control configuration
REMOTE_CONTROL_ENABLED = args.host is not None
HOST_PC_IP = args.host
BASE_PORT = args.port
CAR_ID = args.car_id

# choosing between two sets of nodes which avoids two way traffic
if node_options==0:
    valid_nodes=[10, 2, 4, 6, 8, 1]
else:
    valid_nodes=[10, 2, 4, 6, 8, 1]

# choosing random node sequence
# random.shuffle(valid_nodes)
# num_nodes=random.randint(4,5)
# nodeSequence=valid_nodes[:num_nodes]+[valid_nodes[0]]
nodeSequence = valid_nodes

print(f"Selected nodes: {nodeSequence}")
#================ Experiment Configuration ================
# ===== Timing Parameters
# - tf: experiment duration in seconds.
# - startDelay: delay to give filters time to settle in seconds.
# - controllerUpdateRate: control update rate in Hz. Shouldn't exceed 500
tf = 6000
startDelay = 1
controllerUpdateRate = 200
startTime = time.time()
def elapsed_time():
    return time.time() - startTime

# ===== Speed Controller Parameters
# - v_ref: desired velocity in m/s
# - K_p: proportional gain for speed controller
# - K_i: integral gain for speed controller
v_ref = 0.75
K_p = 0.1
K_i = 1

# ===== Steering Controller Parameters
# - enableSteeringControl: whether or not to enable steering control
# - K_stanley: K gain for stanley controller
# - nodeSequence: list of nodes from roadmap. Used for trajectory generation.
enableSteeringControl = True
K_stanley = 0.7

# Define the calibration pose
# Calibration pose is either [0,0,-pi/2] or [0,2,-pi/2]
# Comment out the one that is not used 
#calibrationPose = [0,0,-np.pi/2]
calibrationPose = [-1.28205, -0.45991, -0.7330382858376184]
# calibrate = 'y' in input('do you want to recalibrate?(y/n)')
calibrate = args.calibrate 

#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Initial setup
if enableSteeringControl:
    roadmap = SDCSRoadMap(leftHandTraffic=False , useSmallMap= True)
    waypointSequence = roadmap.generate_path(nodeSequence)
    initialPose = roadmap.get_node_pose(nodeSequence[0]).squeeze()
else:
    initialPose = [0, 0, 0]


# Used to enable safe keyboard triggered shutdown
kill_event = Event()
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
    kill_event.set()
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
        
    def connect(self, timeout: int = 5):
        """Connect to the host PC"""
        print(f"[Car {self.car_id}] Attempting connection to Host PC at {self.host_ip}:{self.port}...")
        
        try:
            if self.socket:
                self.socket.close()
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(timeout)
            self.socket.connect((self.host_ip, self.port))
            self.socket.settimeout(None)
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


    # ==============  SECTION A -  Speed Control  ====================
    def update(self, v, v_ref, dt):
        
        e = v_ref - v
        self.ei += dt*e

        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )
    
        return 0

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

    # ==============  SECTION B -  Steering Control  ====================
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
            if  self.cyclic or self.wpi < self.N-2:
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
        
        # return 0

def controlLoop():
    #region controlLoop setup
    global KILL_THREAD, v_ref, nodeSequence, waypointSequence
    u = 0
    delta = 0
    x = 0.2
    y = 0.6
    th = 0.7
    telemetry_send_counter = 0

    #endregion

    #region Network setup (if remote control enabled)
    network = None
    if REMOTE_CONTROL_ENABLED:
        network = NetworkClient(HOST_PC_IP, BASE_PORT + CAR_ID, CAR_ID)
        print(f"[Car {CAR_ID}] Remote control enabled. Connecting to {HOST_PC_IP}:{BASE_PORT + CAR_ID}...")
        
        while not network.connect() and not kill_event.is_set():
            print(f"[Car {CAR_ID}] Connection failed, retrying in 2 seconds...")
            time.sleep(2)
        
        if kill_event.is_set():
            print(f"[Car {CAR_ID}] Shutdown requested before connection established")
            return
        
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
                        if 'v_ref' in cmd:
                            v_ref = cmd['v_ref']
                            print(f"[Car {CAR_ID}] Initial velocity set to {v_ref}")
                        if 'node_sequence' in cmd:
                            nodeSequence = cmd['node_sequence']
                            waypointSequence = roadmap.generate_path(nodeSequence)
                            print(f"[Car {CAR_ID}] Initial path set: {nodeSequence}")
            time.sleep(0.1)
        
        if kill_event.is_set():
            print(f"[Car {CAR_ID}] Shutdown requested during initialization")
            if network:
                network.close()
            return
    #endregion

    #region Controller initialization
    speedController = SpeedController(
        kp=K_p,
        ki=K_i
    )
    if enableSteeringControl:
        steeringController = SteeringController(
            waypoints=waypointSequence,
            k=K_stanley
        )
    #endregion

    #region QCar interface setup
    qcar = QCar(readMode=1, frequency=controllerUpdateRate)
    yolo = YOLOReceiver()
    yoloDrive = YOLODriveLogic(pulseLength=controllerUpdateRate*3)

    if enableSteeringControl:
        ekf = QCarEKF(x_0=initialPose)
        gps = QCarGPS(initialPose=calibrationPose,calibrate=calibrate)
    else:
        gps = memoryview(b'')
    #endregion

    # check the initial pose of the car 
    new = False
    while not new:
        new = gps.readGPS()
    initPose = np.array([gps.position[0],
                         gps.position[1],
                         gps.orientation[2]
                         ])
    startNodeReached, initWaypointSequence = roadmap.initial_check(initPose, nodeSequence,waypointSequence)
    if not startNodeReached:
        initSteeringController = SteeringController(waypoints=initWaypointSequence,
                                                    k=K_stanley,
                                                    cyclic=False)
    with qcar, gps, yolo:
        if calibrate:
            return
        t0 = time.time()
        t=0
        startTime = time.time()
        running = True
        
        while (t < tf+startDelay) and (not KILL_THREAD) and running:
            #region : Check for remote commands
            if network and network.connected:
                commands = network.receive_commands()
                if commands:
                    for cmd in commands:
                        cmd_type = cmd.get('type', '')
                        
                        if cmd_type == 'stop':
                            v_ref = 0.0
                            print(f"[Car {CAR_ID}] STOP command received")
                        
                        elif cmd_type == 'start':
                            v_ref = 0.75
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
            #endregion
            
            #region : Loop timing update
            # start = elapsed_time()
            tp = t
            t = time.time() - t0
            dt = t-tp
            #endregion

            #region : Read from sensors and update state estimates
            qcar.read()

            # point_A = elapsed_time()
            yolo.read()
            vGain = yoloDrive.check_yolo(yolo.stopSign,
                                         yolo.trafficlight,
                                         yolo.cars, 
                                         yolo.yieldSign,
                                         yolo.person)
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
                p = ( np.array([x, y])
                    + np.array([np.cos(th), np.sin(th)]) * 0.2)
                
            v = qcar.motorTach
            #endregion

            #region : Update controllers and write to car
            if t < startDelay:
                u = 0
                delta = 0
            else:
                #region : Speed controller update
                u = speedController.update(v, v_ref*vGain, dt)
                #endregion

                #region : Steering controller update
                if enableSteeringControl:
                    if not startNodeReached:
                        # check the criterion inside the if condition to ensure it will
                        # only be executed once
                        distToStart = np.linalg.norm(waypointSequence[:,0]-p)
                        startNodeReached = distToStart < 0.2
                        # if the start node is not reached, use the initial steering controller
                        delta = initSteeringController.update(p, th, v)
                    else:
                        delta = steeringController.update(p, th, v)
                else:
                    delta = 0
                #endregion
            qcar.write(u, delta)
            
            #region : Send telemetry to host PC (if connected)
            if network and network.connected:
                telemetry_send_counter += 1
                if telemetry_send_counter >= controllerUpdateRate / 10:  # Send at 10 Hz
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
            #endregion
            
            # end = elapsed_time()
            continue
        
        qcar.read_write_std(throttle= 0, steering= 0)
        
        if network:
            network.close()
            print(f"[Car {CAR_ID}] Network connection closed")

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Setup and run experiment
if __name__ == '__main__':
    
    if REMOTE_CONTROL_ENABLED:
        print(f"\n{'='*60}")
        print(f"Remote Controlled QCar - Car {CAR_ID}")
        print(f"{'='*60}")
        print(f"Will connect to: {HOST_PC_IP}:{BASE_PORT + CAR_ID}")
        print(f"{'='*60}\n")
    
    #sleep for 10 seconds to wait for YOLO server to start
    time.sleep(10)
    
    #region : Setup control thread, then run experiment
    controlThread = Thread(target=controlLoop)
    controlThread.start()

    try:
        while controlThread.is_alive() and (not KILL_THREAD):
            # MultiScope.refreshAll()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print(f"\n[Car {CAR_ID if REMOTE_CONTROL_ENABLED else 'Local'}] Keyboard interrupt. Shutting down...")
        KILL_THREAD = True
        kill_event.set()
    finally:
        KILL_THREAD = True
        kill_event.set()
        
        # Wait for control thread to finish
        controlThread.join(timeout=5)
        if controlThread.is_alive():
            print(f"[Car {CAR_ID if REMOTE_CONTROL_ENABLED else 'Local'}] Control thread did not exit gracefully")
        
        print("Experiment complete.")
