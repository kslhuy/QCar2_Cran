import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class OdomPoseGetter(Node):
    def __init__(self):
        super().__init__('get_odom_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.004, self.get_pose)

    def get_pose(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                now)
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            self.get_logger().info(f'Position in odom: x={x:.2f}, y={y:.2f}, quaternion=({q.x:.2f}, {q.y:.2f}, {q.z:.2f}, {q.w:.2f})')
        except Exception as e:
            self.get_logger().warn(f'Could not get odom->base_link transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomPoseGetter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
