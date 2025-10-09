# qcar_ros2_tcp_client.py
import numpy as np
from ros2test.util import yaw_to_quaternion
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import json
from qcar2_interfaces.msg import MotorCommands
from geometry_msgs.msg import PoseStamped

HOST = '0.0.0.0'
PORT = 5005

class QCarROSBridge(Node):
    def __init__(self):
        super().__init__('qcar_ros_bridge')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('qcar_id', 0),
            ]
        )   
        self.qcar_id = self.get_parameter("qcar_id").value
        
        self.port = PORT + self.qcar_id
        
        self.get_logger().info(f"port: {self.port}")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, self.port))
        self.buffer = b""

        self.desired_steering = 0.0
        self.desired_speed = 0.0
        
        # ROS 2 publishers
        self.lidar_pub = self.create_publisher(LaserScan, f"qcar_{str(self.qcar_id)}/lidar", 10)
        self.pub_pose = self.create_publisher(PoseStamped, f"qcar_{str(self.qcar_id)}/ekf_pose", 10)

        # ROS 2 subscriber for velocity command
        self.create_subscription(MotorCommands, f"qcar_{str(self.qcar_id)}/cmd_vel" , self.motor_command_callback, 10)

        # Timer to receive data
        self.timer = self.create_timer(0.004, self.timer_callback)


    def motor_command_callback(self, motor_commands: MotorCommands):
        motor_channel_map = {
            'steering_angle': 1000,
            'motor_throttle': 11000
        }

        motor_commands_size = len(motor_commands.motor_names)

        if motor_commands_size > len(motor_channel_map):
            self.get_logger().warning(
                f"In motor_command_callback - size of MotorCommands message must be between 0 and {len(motor_channel_map)}, but received size is: {motor_commands_size}...ignoring"
            )
            return

        for i in range(motor_commands_size):
            motor_name = motor_commands.motor_names[i]
            if motor_name not in motor_channel_map:
                self.get_logger().warning(
                    f"In motor_command_callback - MotorCommands message command_name {motor_name} is invalid...ignoring"
                )
                break

            if i == 0:
                self.desired_steering = -motor_commands.values[0]

            if i == 1:
                self.desired_speed = motor_commands.values[1]


    def timer_callback(self):
        cmd = json.dumps({"motor": self.desired_speed, "steering": self.desired_steering}) + "\n"
        self.sock.sendall(cmd.encode())
        try:
            data = self.sock.recv(4096)
            if not data:
                self.get_logger().error(f"Receive empty")
                return
            self.buffer += data
            while b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)
                packet = json.loads(line.decode())
                self.publish_ros_topics(packet)
        except Exception as e:
            self.get_logger().error(f"Receive error: {e}")
            return

    def publish_ros_topics(self, data):
        # lidar
        if len(data["lidar"]["angles"]) > 0:
            scan_msg = LaserScan()
            scan_msg.angle_min = -np.pi
            scan_msg.angle_max = np.pi
            scan_msg.angle_increment = 2 * np.pi / len(data["lidar"]["angles"])
            scan_msg.ranges = data["lidar"]["distances"]
            scan_msg.header.frame_id = "laser"
            self.lidar_pub.publish(scan_msg)
        
        # position
        if "position" in data and "rotation" in data:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = data["position"][0]
            msg.pose.position.y = data["position"][1]
            msg.pose.position.z = data["position"][2]

            # Convert yaw to quaternion
            q = yaw_to_quaternion(data["rotation"][2])
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]

            self.pub_pose.publish(msg)
        
        # images

def main(args=None):
    rclpy.init(args=args)
    node = QCarROSBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
