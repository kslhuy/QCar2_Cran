#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from qcar2_interfaces.msg import MotorCommands
from sensor_msgs.msg import Imu

class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.get_logger().info("log in test node")
        self.pub = self.create_publisher(MotorCommands, "/qcar2_motor_speed_cmd", 10)
        # self.sub = self.create_subscription(Imu, "/qcar2_imu", self.receive, 10)
        self.timer = self.create_timer(2, self.send)

    def send(self):
        msg = MotorCommands()
        msg.motor_names.append("steering_angle")
        msg.motor_names.append("motor_throttle")
        msg.values.append(-1)
        msg.values.append(3)
        self.pub.publish(msg)
        self.get_logger().info(str(msg))
    
    def receive(self, msg: Imu):
        self.get_logger().info(str(msg))

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()