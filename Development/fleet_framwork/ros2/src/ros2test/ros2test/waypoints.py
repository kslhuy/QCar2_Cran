import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from hal.products.mats import SDCSRoadMap
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import math

class Waypoints(Node):
    def __init__(self):
        super().__init__('waypoints')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('nodeSequence', [10, 4, 20, 10]),
            ]
        )
        
        self.nodeSequence = self.get_parameter("nodeSequence").value
        self.roadmap = SDCSRoadMap(leftHandTraffic=False)
        self.waypointSequence = self.roadmap.generate_path(self.nodeSequence)
        self.waypoints_x = self.waypointSequence[0]
        self.waypoints_y = self.waypointSequence[1]
        self.vis_path_pub = self.create_publisher(MarkerArray, "/waypoints", 10)        
        self.timer = self.create_timer(1.0, self.pub)

    def pub(self):
        marker_array = MarkerArray()

        for i, (x, y) in enumerate(zip(self.waypoints_x, self.waypoints_y)):
            dx, dy = (0.45, 1.35)
            angle_radians = math.radians(34)
            
            # Rotation around Z axis
            x_rot = x * math.cos(angle_radians) - y * math.sin(angle_radians)
            y_rot = x * math.sin(angle_radians) + y * math.cos(angle_radians)
            
            # Translation
            x_final = x_rot + dx
            y_final = y_rot + dy
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x_final
            marker.pose.position.y = y_final
            marker.pose.position.z = 0.0
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.a = 1.0
            marker.color.g = 1.0  # Green markers

            # lifetime = forever
            marker.lifetime = Duration(sec=0, nanosec=0)

            marker_array.markers.append(marker)

        self.vis_path_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = Waypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
