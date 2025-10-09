from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node

class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(2.0, self.send_goal)

    def send_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = 0.9
        goal.pose.position.y = -1
        goal.pose.orientation.w = 1.0
        self.publisher.publish(goal)
        self.get_logger().info("Goal sent!")

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalSender()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()