import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from qcar2_interfaces.msg import MotorCommands
from cv_bridge import CvBridge
import cv2
import numpy as np

from hal.utilities.image_processing import ImageProcessing

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        self.bridge = CvBridge()
        self.image_processing = ImageProcessing()
        self.sub = self.create_subscription(Image, '/camera/color_image', self.image_callback, 10)
        self.pub = self.create_publisher(MotorCommands, "/qcar2_motor_speed_cmd", 10)


    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame is None or frame.size == 0:
            self.get_logger().warn("Received empty or invalid image!")
            return        
        self.image_processing.height, self.image_processing.width, _ = frame.shape
        
        # do canny
        edges = self.image_processing.do_canny(frame)

        left_lines, right_lines = self.image_processing.calculate_lines(edges)

        if left_lines and right_lines:
            # Find average midline
            parameters, output = self.image_processing.average_lines(frame, left_lines, right_lines)

            # Steering logic
            motor_speed, steering = self.image_processing.driving_parameters(parameters)
            self.send_control(motor_speed, steering)

            cv2.imshow("Lane Detection", output)

        cv2.waitKey(1)

    def send_control(self, motor, steer):
        msg = MotorCommands()
        msg.motor_names.append("steering_angle")
        msg.motor_names.append("motor_throttle")
        msg.values.append(steer)
        msg.values.append(motor)
        self.pub.publish(msg)
        self.get_logger().info(str(msg))


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
