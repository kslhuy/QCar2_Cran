import time
from ros2test.util import yaw_to_quaternion
from hal.content.qcar_functions import QCarEKF
from hal.products.mats import SDCSRoadMap
from pal.products.qcar import QCarGPS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import PoseStamped

class EKF(Node):
    def __init__(self):
        super().__init__('ekf')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('NODE_SEQUENCE', [10, 4, 20, 10]),
                ('ENCODER_COUNTS_PER_REV', 720.0),
                ('WHEEL_RADIUS', 0.033),
                ('PIN_TO_SPUR_RATIO', 0.09536679536679536),
                ('VIRTUAL', True),
            ]
        )        
        
        # Get parameters
        self.nodeSequence = self.get_parameter("NODE_SEQUENCE").value
        self.ENCODER_COUNTS_PER_REV = self.get_parameter("ENCODER_COUNTS_PER_REV").value
        self.WHEEL_RADIUS = self.get_parameter("WHEEL_RADIUS").value
        self.PIN_TO_SPUR_RATIO = self.get_parameter("PIN_TO_SPUR_RATIO").value
        self.VIRTUAL = self.get_parameter("VIRTUAL").value

        self.CPS_TO_MPS = (1/(self.ENCODER_COUNTS_PER_REV*4) # motor-speed unit conversion
            * self.PIN_TO_SPUR_RATIO * 2*np.pi * self.WHEEL_RADIUS)
        
        roadmap = SDCSRoadMap(leftHandTraffic=False)
        initialPose = roadmap.get_node_pose(self.nodeSequence[0]).squeeze()
        calibrate = True
        if self.VIRTUAL : 
            calibrate = False
        self.gps = QCarGPS(initialPose=initialPose,calibrate=calibrate)
        self.ekf = QCarEKF(x_0=initialPose)
        
        self.sub_imu = self.create_subscription(Imu, '/qcar2_imu', self.imu_callback, 10)
        self.sub_joint = self.create_subscription(JointState, '/qcar2_joint', self.joint_callback, 10)

        self.pub_pose = self.create_publisher(PoseStamped, "/ekf_pose", 50)

        self.t0 = time.time()
        self.t = 0
        self.motorTach = 0
        self.delta = 0 
        
        
    def joint_callback(self, msg: JointState): 
        # self.get_logger().info("joint " + str(msg.velocity[0] * self.CPS_TO_MPS))
        self.motorTach = msg.velocity[0] * self.CPS_TO_MPS
        
    def imu_callback(self, msg: Imu): 
        self.imu_data = msg.angular_velocity.z
        tp = self.t
        self.t = time.time() - self.t0
        dt = self.t-tp

        if self.imu_data:
            if self.gps.readGPS():
                y_gps = np.array([
                    self.gps.position[0],
                    self.gps.position[1],
                    self.gps.orientation[2]
                ])
                self.ekf.update(
                    [self.motorTach, self.delta],
                    dt,
                    y_gps,
                    self.imu_data,
                )
                
            else:
                self.ekf.update(
                    [self.motorTach, self.delta],
                    dt,
                    None,
                    self.imu_data,
                )

        self.publish_pose()

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.ekf.x_hat[0,0]
        msg.pose.position.y = self.ekf.x_hat[1,0]
        msg.pose.position.z = 0.0

        # Convert yaw to quaternion
        q = yaw_to_quaternion(self.ekf.x_hat[2,0])
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pub_pose.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = EKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
