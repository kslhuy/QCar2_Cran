# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports

"""
vehicle_control.py

Skills acivity code for vehicle control lab guide.
Students will implement a vehicle speed and steering controller.
Please review Lab Guide - vehicle control PDF
"""
import os
import signal
import numpy as np
from threading import Thread
import time
import cv2
import pyqtgraph as pg
import matplotlib.pyplot as plt
import logging


from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from hal.content.qcar import QCarRealSense  # RealSense RGB-D camera
from pal.utilities.scope import MultiScope
from pal.utilities.math import wrap_to_pi
from hal.content.qcar_functions import QCarEKF
from hal.products.mats import SDCSRoadMap
import pal.resources.images as images

# Import for leader controller testing
from src.OpenRoad import OpenRoad
from src.Controller.ControllerLeader import SpeedController as OriginalSpeedController, SteeringController as OriginalSteeringController

#===================== QLabs Setup ========================
# This modified version of vehicle control is intended to be used in
# the Development Container with the Virtual QCar. 
# 
# It is intended to be run after you have spawned the QCar in the 
# diagonal portion of the SDCS roadmap or the diagonal portion of
# the Cityscape.


#================ Experiment Configuration ================
# ===== Timing Parameters
# - tf: experiment duration in seconds.
# - startDelay: delay to give filters time to settle in seconds.
# - controllerUpdateRate: control update rate in Hz. Shouldn't exceed 500
tf = 3000
startDelay = 1
controllerUpdateRate = 100

# ===== Speed Controller Parameters
# - v_ref: desired velocity in m/s
# - K_p: proportional gain for speed controller
# - K_i: integral gain for speed controller
v_ref = 0.3
K_p = 0.1
K_i = 1

# ===== Steering Controller Parameters
# - enableSteeringControl: whether or not to enable steering control
# - K_stanley: K gain for stanley controller
# - nodeSequence: list of nodes from roadmap. Used for trajectory generation.
enableSteeringControl = True
K_stanley = 1
nodeSequence = [10, 4, 20, 13,10]

#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Initial setup
if enableSteeringControl:
    roadmap = SDCSRoadMap(leftHandTraffic=False)
    # # print(dir(roadmap))

    
    # # Plot all nodes
    # for i in range(len(roadmap.nodes)):  # Or len(roadmap.nodes)
    #     pose = roadmap.get_node_pose(i).squeeze()
    #     plt.plot(pose[0], pose[1], 'bo')  # blue dot
    #     plt.text(pose[0], pose[1], str(i), fontsize=9)

    # # Optionally: draw connections (if you know them or can access .edges or .adjacency)
    # # Example: if roadmap.edges = [(0,1), (1,2)]
    # # for a, b in roadmap.edges:
    # #     pose_a = roadmap.get_node_pose(a).squeeze()
    # #     pose_b = roadmap.get_node_pose(b).squeeze()
    # #     plt.plot([pose_a[0], pose_b[0]], [pose_a[1], pose_b[1]], 'k--')

    # plt.axis('equal')
    # plt.title('SDCS Road Map Nodes')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    # plt.grid(True)
    # plt.show()

    waypointSequence = roadmap.generate_path(nodeSequence)
    initialPose = roadmap.get_node_pose(nodeSequence[0]).squeeze()
    print("Initial Pose:", initialPose)

else:
    initialPose = [0, 0, 0]

calibrate=False

# Define the calibration pose
# Calibration pose is either [0,0,-pi/2] or [0,2,-pi/2]
# Comment out the one that is not used 
#calibrationPose = [0,0,-np.pi/2]
calibrationPose = [0,2,-np.pi/2]

# Used to enable safe keyboard triggered shutdown
global KILL_THREAD
KILL_THREAD = False

follower_path = []


def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)
#endregion

class VehicleLeaderController:
    """
    Dedicated controller for leader vehicles that encapsulates all the complex
    leader control logic from ControlLeader class.
    """
    
    def __init__(self, vehicle_id: int, config=None, logger=None):
        """
        Initialize the leader controller.
        
        Args:
            vehicle_id: ID of the vehicle this controller manages
            config: Configuration object containing parameters
            logger: Logger instance for this controller
        """
        self.vehicle_id = vehicle_id
        self.config = config
        self.logger = logger or logging.getLogger(f"LeaderController_{vehicle_id}")
        
        # Control parameters (from ControlLeader)
        self.controllerUpdateRate = 100
        self.K_p = 0.1
        self.K_i = 0.8
        self.enableSteeringControl = True
        self.K_stanley = 0.8
        self.startDelay = 1
        
        # Control state
        self.start_time = None
        self.delta = 0  # Steering command
        self.u = 0      # Throttle command
        self.max_steering = config.get('max_steering', 0.6) if config else 0.6
        
        # Path and controllers
        self.waypointSequence = None
        self.InitialPose = None
        self.speedController = None
        self.steeringController = None
        
        # Initialize components
        self._init_path_and_controllers()
        
        print(f"Leader controller initialized for vehicle {vehicle_id}")
    
    def _init_path_and_controllers(self):
        """Initialize path generation and controllers."""
        # Get configuration parameters
        road_type = self.config.get('road_type', 'Studio') if self.config else "Studio"
        node_sequence = self.config.get('node_sequence', [10, 4, 20, 13, 10]) if self.config else [10, 4, 20, 13, 10]
        
        # Generate path
        self.waypointSequence, self.InitialPose = self._generate_path(road_type, node_sequence)
        
        # Initialize speed controller
        self.speedController = OriginalSpeedController(
            kp=self.K_p,
            ki=self.K_i
        )
        
        # Initialize steering controller
        if self.enableSteeringControl:
            self.steeringController = OriginalSteeringController(
                waypoints=self.waypointSequence,
                k=self.K_stanley
            )
        
        print(f"Generated path with {len(self.waypointSequence)} waypoints")
    
    def _generate_path(self, road_type: str, node_sequence: list):
        """Generate path for leader vehicle (from ControlLeader.PathGeneration)."""
        if road_type == "OpenRoad":
            roadmap = OpenRoad()
        elif road_type == "Studio":
            roadmap = SDCSRoadMap()
        else:
            raise ValueError(f"Unknown road type: {road_type}")
        
        waypointSequence = roadmap.generate_path(node_sequence)
        print(f"Generated waypoint sequence: {waypointSequence}")
        InitialPose = roadmap.get_node_pose(node_sequence[0]).squeeze()
        
        return waypointSequence, InitialPose
    
    def _get_vref(self, t):
        """Get reference velocity based on time (from ControlLeader.vref)."""
        road_type = self.config.get('road_type', 'Studio') if self.config else "Studio"
        
        if road_type == "OpenRoad":
            if t < 5:
                v_ref = 2
            elif t < 10:
                v_ref = 0
            elif t < 15:
                v_ref = -0.5
            elif t < 20:
                v_ref = 1
            else:
                v_ref = 2
            return v_ref
        elif road_type == "Studio":
            return 0.3
        else:
            return 0.3
    
    def start_control(self, dt=0.1):
        """Start the control session (call this when vehicle starts)."""
        if self.start_time is None:
            self.start_time = time.time()
            print("Leader control session started")
    
    def compute_control(self, current_pos, current_rot, velocity, dt=0.1):
        """
        Compute control commands based on current vehicle state.
        Uses the provided state directly since local observer handles state estimation.
        
        Args:
            current_pos: Current position [x, y, z] from Vehicle's observer
            current_rot: Current rotation [roll, pitch, yaw] from Vehicle's observer  
            velocity: Current velocity from Vehicle's observer
            dt: Time step for control update
            
        Returns:
            tuple: (forward_speed, steering_angle) control commands
        """
        if self.start_time is None:
            self.start_control(dt)
        
        # Calculate elapsed time
        t = time.time() - self.start_time
        
        # Get reference velocity
        vref = self._get_vref(t)
        
        # Extract position and orientation directly from provided state
        x, y = current_pos[0], current_pos[1]
        th = current_rot[2]  # Heading in radians
        
        # Position for steering controller (with look-ahead point)
        p = np.array([x, y]) + np.array([np.cos(th), np.sin(th)]) * 0.2
        
        # Use provided velocity directly
        v = velocity
        
        # Control logic (from ControlLeader.run())
        if t < self.startDelay:
            self.u = 0
            self.delta = 0
        else:
            # Speed control
            self.u = self.speedController.update(v, vref, dt)
            
            # Steering control
            if self.enableSteeringControl:
                self.delta = self.steeringController.update(p, th, v)
            else:
                self.delta = 0
        
        # Convert and clamp commands
        forward_speed = self.u
        steering_angle = self.delta
        
        # Clamp to safe ranges
        forward_speed = max(-2.0, min(2.0, forward_speed))
        steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))
        
        return forward_speed, steering_angle
    
    def get_control_state(self):
        """Get current control state information."""
        if self.start_time is None:
            return {
                'initialized': False,
                'elapsed_time': 0,
                'throttle_command': 0,
                'steering_command': 0,
                'reference_velocity': 0,
                'waypoint_count': len(self.waypointSequence) if self.waypointSequence is not None else 0
            }
        
        elapsed_time = time.time() - self.start_time
        return {
            'initialized': True,
            'elapsed_time': elapsed_time,
            'throttle_command': self.u,
            'steering_command': self.delta,
            'reference_velocity': self._get_vref(elapsed_time),
            'waypoint_count': len(self.waypointSequence) if self.waypointSequence is not None else 0
        }
    
    def stop_control(self):
        """Stop the control session and cleanup."""
        self.start_time = None
        self.u = 0
        self.delta = 0
        print("Leader control session stopped")
    
    def reset_control(self):
        """Reset the control session (restart timing)."""
        self.stop_control()
        self.start_control()
        print("Leader control session reset")

class SpeedController:

    def __init__(self, kp=0, ki=0):
        self.maxThrottle = 0.3

        self.kp = kp
        self.ki = ki

        self.ei = 0


    # ==============  SECTION A -  Speed Control  ====================
    def update(self, v, v_ref, dt):
        
        e = v_ref - v
        # self.ei += dt*e

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
        

def controlLoop():
    #region controlLoop setup
    global KILL_THREAD
    u = 0
    delta = 0
    # used to limit data sampling to 10hz
    countMax = controllerUpdateRate / 10
    count = 0
    #endregion

    #region Controller initialization
    # Create a simple config for VehicleLeaderController
    controller_config = {
        'road_type': 'Studio',
        'node_sequence': nodeSequence,
        'max_steering': 0.6
    }
    
    # Initialize your VehicleLeaderController
    leaderController = VehicleLeaderController(
        vehicle_id=0,
        config=controller_config,
        logger=None
    )
    
    # Backup original controllers for comparison (optional)
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
    if enableSteeringControl or calibrate:
        ekf = QCarEKF(x_0=initialPose)
        gps = QCarGPS(initialPose=calibrationPose,calibrate=calibrate)
    else:
        gps = memoryview(b'')
    #endregion

    with qcar, gps:
        t0 = time.time()
        t=0
        while (t < tf+startDelay) and (not KILL_THREAD):
            #region : Loop timing update
            tp = t
            t = time.time() - t0
            dt = t-tp
            #endregion

        #region : Read from sensors and update state estimates
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
            p = ( np.array([x, y])
                + np.array([np.cos(th), np.sin(th)]) * 0.2)
            
            # ---- Follower Path Logging ----
            follower_path.append([p[0], p[1]])
            if len(follower_path) > 1000:  # Prevent overflow
                follower_path.pop(0)


        v = qcar.motorTach
        # print('v',v)
        #endregion

        #region : Update controllers and write to car
        if t < startDelay:
            u = 0
            delta = 0
        else:
            # Use VehicleLeaderController for both speed and steering control
            if enableSteeringControl:
                # Get current state from EKF
                current_pos = [ekf.x_hat[0,0], ekf.x_hat[1,0], 0]  # x, y, z
                current_rot = [0, 0, ekf.x_hat[2,0]]  # roll, pitch, yaw
                current_velocity = v
                
                # Compute control using your VehicleLeaderController
                u, delta = leaderController.compute_control(
                    current_pos=current_pos,
                    current_rot=current_rot,
                    velocity=current_velocity,
                    dt=dt
                )
            else:
                # Fallback to original speed controller only
                u = speedController.update(v, v_ref, dt)
                delta = 0

            qcar.write(u, delta)
            #endregion

            #region : Update Scopes
            count += 1
            if count >= countMax and t > startDelay:
                
                t_plot = t - startDelay

                if len(follower_path) > 1:
                    xs, ys = zip(*follower_path)
                    followerPath.setData(xs, ys)




                # Speed control scope
                # Get reference velocity from VehicleLeaderController
                control_state = leaderController.get_control_state()
                v_ref_leader = control_state['reference_velocity']
                
                speedScope.axes[0].sample(t_plot, [v, v_ref_leader])
                speedScope.axes[1].sample(t_plot, [v_ref_leader-v])
                speedScope.axes[2].sample(t_plot, [u])
                # (Optional) Could log headway to an unused axis or stdout already handled

                # Steering control scope
                if enableSteeringControl:
                    steeringScope.axes[4].sample(t_plot, [[p[0],p[1]]])

                    p[0] = ekf.x_hat[0,0]
                    p[1] = ekf.x_hat[1,0]

                    # Use GPS data for reference plotting
                    x_ref = gps.position[0]
                    y_ref = gps.position[1]
                    th_ref = gps.orientation[2]

                    steeringScope.axes[0].sample(t_plot, [p[0], x_ref])
                    steeringScope.axes[1].sample(t_plot, [p[1], y_ref])
                    steeringScope.axes[2].sample(t_plot, [th, th_ref])
                    steeringScope.axes[3].sample(t_plot, [delta])


                    arrow.setPos(p[0], p[1])
                    arrow.setStyle(angle=180-th*180/np.pi)

                count = 0
            #endregion
        
        # out of while loop : Stop the car
        qcar.read_write_std(throttle=0, steering=0)# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Setup and run experiment
if __name__ == '__main__':

    #region : Setup scopes
    if IS_PHYSICAL_QCAR:
        fps = 10
    else:
        fps = 30
    # Scope for monitoring speed controller
    speedScope = MultiScope(
        rows=3,
        cols=1,
        title='Vehicle Speed Control',
        fps=fps
    )
    speedScope.addAxis(
        row=0,
        col=0,
        timeWindow=10,#tf
        yLabel='Vehicle Speed [m/s]',
        yLim=(0, 1)
    )
    speedScope.axes[0].attachSignal(name='v_meas', width=2)
    speedScope.axes[0].attachSignal(name='v_ref')

    speedScope.addAxis(
        row=1,
        col=0,
        timeWindow=10,
        yLabel='Speed Error [m/s]',
        yLim=(-0.5, 0.5)
    )
    speedScope.axes[1].attachSignal()

    speedScope.addAxis(
        row=2,
        col=0,
        timeWindow=10,
        xLabel='Time [s]',
        yLabel='Throttle Command [%]',
        yLim=(-0.3, 0.3)
    )
    speedScope.axes[2].attachSignal()

    # Scope for monitoring steering controller
    if enableSteeringControl:
        steeringScope = MultiScope(
            rows=4,
            cols=2,
            title='Vehicle Steering Control',
            fps=fps
        )

        steeringScope.addAxis(
            row=0,
            col=0,
            timeWindow=10,
            yLabel='x Position [m]',
            yLim=(-2.5, 2.5)
        )
        steeringScope.axes[0].attachSignal(name='x_meas')
        steeringScope.axes[0].attachSignal(name='x_ref')

        steeringScope.addAxis(
            row=1,
            col=0,
            timeWindow=10,
            yLabel='y Position [m]',
            yLim=(-1, 5)
        )
        steeringScope.axes[1].attachSignal(name='y_meas')
        steeringScope.axes[1].attachSignal(name='y_ref')
        # Heading angle
        steeringScope.addAxis(
            row=2,
            col=0,
            timeWindow=10,
            yLabel='Heading Angle [rad]',
            yLim=(-3.5, 3.5)
        )
        steeringScope.axes[2].attachSignal(name='th_meas')
        steeringScope.axes[2].attachSignal(name='th_ref')

        steeringScope.addAxis(
            row=3,
            col=0,
            timeWindow=10,
            yLabel='Steering Angle [rad]',
            yLim=(-0.6, 0.6)
        )
        steeringScope.axes[3].attachSignal()
        steeringScope.axes[3].xLabel = 'Time [s]'

        steeringScope.addXYAxis(
            row=0,
            col=1,
            rowSpan=4,
            xLabel='x Position [m]',
            yLabel='y Position [m]',
            xLim=(-2.5, 2.5),
            yLim=(-1, 5)
        )

        im = cv2.imread(
            images.SDCS_CITYSCAPE,
            cv2.IMREAD_GRAYSCALE
        )

        steeringScope.axes[4].attachImage(
            scale=(-0.002035, 0.002035),
            offset=(1125,2365),
            rotation=180,
            levels=(0, 255)
        )
        steeringScope.axes[4].images[0].setImage(image=im)

        referencePath = pg.PlotDataItem(
            pen={'color': (85,168,104), 'width': 2},
            name='Reference'
        )
        steeringScope.axes[4].plot.addItem(referencePath)
        referencePath.setData(waypointSequence[0, :],waypointSequence[1, :])

        followerPath = pg.PlotDataItem(
            pen={'color': (255, 255, 0), 'width': 2},  # Yellow path
            name='Follower'
        )
        steeringScope.axes[4].plot.addItem(followerPath)


        steeringScope.axes[4].attachSignal(name='Estimated', width=2)

        steeringScope.axes[4].attachSignal(name='Follower', width=2, color='y')


        arrow = pg.ArrowItem(
            angle=180,
            tipAngle=60,
            headLen=10,
            tailLen=10,
            tailWidth=5,
            pen={'color': 'w', 'fillColor': [196,78,82], 'width': 1},
            brush=[196,78,82]
        )
        arrow.setPos(initialPose[0], initialPose[1])
        steeringScope.axes[4].plot.addItem(arrow)
    #endregion

    #region : Setup control thread, then run experiment
    controlThread = Thread(target=controlLoop)
    controlThread.start()

    try:
        while controlThread.is_alive() and (not KILL_THREAD):
            MultiScope.refreshAll()
            time.sleep(0.01)
    finally:
        KILL_THREAD = True

    input('Experiment complete. Press any key to exit...')
    #endregion
#endregion