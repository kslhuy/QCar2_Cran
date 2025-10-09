from collections import deque
import time
from hal.products.mats import SDCSRoadMap
import rclpy
from rclpy.node import Node
from qcar2_interfaces.msg import MotorCommands
import numpy as np
from geometry_msgs.msg import PoseStamped
from hal.utilities.control import StanleyController
from ros2test.controller import SpeedController
from ros2test.util import quaternion_to_yaw
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
from sensor_msgs.msg import LaserScan


class VehicleControl(Node):
    def __init__(self):
        super().__init__('vehicle_control')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('v_ref', 0.9),
                ('K_p', 0.1),
                ('K_i', 1),
                ('K_stanley', 1),
                ('NODE_SEQUENCE', [10, 4, 20, 10]),
                ('ENCODER_COUNTS_PER_REV', 720.0),
                ('WHEEL_RADIUS', 0.033),
                ('PIN_TO_SPUR_RATIO', 0.09536679536679536),
                ('qcar_id', 1),
            ]
        )        
        
        # Get parameters
        self.v_ref = self.get_parameter("v_ref").value
        self.K_p = self.get_parameter("K_p").value
        self.K_i = self.get_parameter("K_i").value
        self.K_stanley = self.get_parameter("K_stanley").value
        self.nodeSequence = self.get_parameter("NODE_SEQUENCE").value
        self.ENCODER_COUNTS_PER_REV = self.get_parameter("ENCODER_COUNTS_PER_REV").value
        self.WHEEL_RADIUS = self.get_parameter("WHEEL_RADIUS").value
        self.PIN_TO_SPUR_RATIO = self.get_parameter("PIN_TO_SPUR_RATIO").value
        self.qcar_id = self.get_parameter("qcar_id").value
        
        self.CPS_TO_MPS = (1/(self.ENCODER_COUNTS_PER_REV*4) # motor-speed unit conversion
            * self.PIN_TO_SPUR_RATIO * 2*np.pi * self.WHEEL_RADIUS)
        
        roadmap = SDCSRoadMap(leftHandTraffic=False)
        self.waypointSequence = roadmap.generate_path(self.nodeSequence)

        waypoint = pg.plot(title=f"qcar_{self.qcar_id}")

        waypoint.setXRange(-4, 4)

        waypoint.setYRange(-2, 6)
        
        self.plotwaypoints = waypoint.plot([], [], pen=None, symbol='o', symbolBrush='r', symbolPen=None, symbolSize=2)
        self.plotpos = waypoint.plot([], [], pen=None, symbol='o', symbolBrush='g', symbolPen=None, symbolSize=2)
        self.plotlidar = waypoint.plot([], [], pen=None, symbol='o', symbolBrush='b', symbolPen=None, symbolSize=2)
        
        self.poslistx = deque(maxlen=300)
        self.poslisty = deque(maxlen=300)
        
        self.sub = self.create_subscription(PoseStamped, f"/qcar_{self.qcar_id}/ekf_pose", self.ekf_callback, 10)
        self.sublidar = self.create_subscription(LaserScan, f"/qcar_{self.qcar_id}/lidar", self.lidar_callback, 10)
        # self.sub2 = self.create_subscription(JointState, '/qcar2_joint', self.joint_callback, 10)

        self.pub = self.create_publisher(MotorCommands, f"/qcar_{self.qcar_id}/cmd_vel", 10)
        
        self.speedController = SpeedController(
            kp=self.K_p,
            ki=self.K_i
        )
        self.stanleyController = StanleyController(
            waypoints=self.waypointSequence,
            k=self.K_stanley
        )
        
        self.t0 = time.time()
        self.t = 0
        self.motorTach = 0
        self.delta = 0 
        
        self.x = None
        self.y = None
        self.p = None
        self.th = None
        self.timer = self.create_timer(0.004, self.timer_callback)


    def lidar_callback(self, msg: LaserScan):
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        x_local = msg.ranges * np.sin(angles)
        y_local = msg.ranges * np.cos(angles)

        if self.x is not None and self.y is not None and self.th is not None:
            heading_adjusted = self.th + np.pi / 2
            x_global = self.x + x_local * np.cos(heading_adjusted) - y_local * np.sin(heading_adjusted)
            y_global = self.y + x_local * np.sin(heading_adjusted) + y_local * np.cos(heading_adjusted)

            self.plotlidar.setData(x_global, y_global)
            QtWidgets.QApplication.instance().processEvents()



    def timer_callback(self):
        if self.p is not None and self.th is not None:
            tp = self.t
            self.t = time.time() - self.t0
            dt = self.t-tp
            
            v = self.motorTach
            u = self.speedController.update(v, self.v_ref, dt)
            self.delta = self.stanleyController.update(self.p, self.th, v)
            self.send_control(u, self.delta)
            
            self.motorTach = u

        
    # def joint_callback(self, msg: JointState): 
    #     # self.get_logger().info("joint " + str(msg.velocity[0] * self.CPS_TO_MPS))
    #     self.motorTach = msg.velocity[0] * self.CPS_TO_MPS
        
    def ekf_callback(self, msg: PoseStamped):
        waypoints_x = self.waypointSequence[0]
        waypoints_y = self.waypointSequence[1] 
        self.plotwaypoints.setData(waypoints_x, waypoints_y)
        
        QtWidgets.QApplication.instance().processEvents()
        
        pose = msg.pose

        self.x = pose.position.x
        self.y = pose.position.y
        self.poslistx.append(self.x)
        self.poslisty.append(self.y)
        self.plotpos.setData(self.poslistx, self.poslisty)
        
        ox = pose.orientation.x
        oy = pose.orientation.y
        oz = pose.orientation.z
        ow = pose.orientation.w
        
        self.th = quaternion_to_yaw(ox, oy, oz, ow)
        # self.get_logger().info("th " + str(th))
        
        self.p = ( np.array([self.x, self.y])
            + np.array([np.cos(self.th), np.sin(self.th)]) * 0.2)


    def send_control(self, motor, steer):
        msg = MotorCommands()
        msg.motor_names.append("steering_angle")
        msg.motor_names.append("motor_throttle")
        msg.values.append(steer)
        msg.values.append(motor)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
