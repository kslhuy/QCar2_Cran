import math
import csv
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from qcar2_interfaces.msg import MotorCommands
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, LookupException
import numpy as np
from rclpy.duration import Duration
from hal.products.mats import SDCSRoadMap


class Waypoints:
    def __init__(self):
        self.X = []
        self.Y = []
        self.V = []
        self.index = 0
        self.velocity_index = 0
        self.lookahead_point_world = np.zeros(3)
        self.current_point_world = np.zeros(3)
        self.lookahead_point_car = np.zeros(3)


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('nodeSequence', [10, 4, 20, 10]),
                ('min_lookahead', 0.5),
                ('max_lookahead', 1.0),
                ('lookahead_ratio', 8.0),
                ('K_p', 0.5),
                ('steering_limit', 25.0),
                ('velocity_percentage', 0.1)
            ]
        )
        self.waypoints = Waypoints()
        self.rotation_m = np.zeros((3, 3))
        self.curr_velocity = 0.0
        self.x_car_world = 0.0
        self.y_car_world = 0.0

        # Get parameters
        self.min_lookahead = self.get_parameter("min_lookahead").value
        self.max_lookahead = self.get_parameter("max_lookahead").value
        self.lookahead_ratio = self.get_parameter("lookahead_ratio").value
        self.K_p = self.get_parameter("K_p").value
        self.steering_limit = self.get_parameter("steering_limit").value
        self.velocity_percentage = self.get_parameter("velocity_percentage").value

        # ROS2 comms
        self.publisher_drive = self.create_publisher(
            MotorCommands, "/qcar2_motor_speed_cmd", 10)
        self.vis_current_point_pub = self.create_publisher(Marker, '/current_waypoint', 10)
        self.vis_lookahead_point_pub = self.create_publisher(Marker, '/lookahead_waypoint', 10)
        self.vis_lookahead_point_car = self.create_publisher(Marker, '/lookahead_point_car', 10)

        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer_ = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info("Pure Pursuit node has been launched.")

        self.load_waypoints()
        
        self.tf_buffer2 = Buffer()
        self.tf_listener2 = TransformListener(self.tf_buffer2, self)
        self.timer2 = self.create_timer(0.004, self.get_pose)

    def get_pose(self):
        print(self.velocity_percentage)
        try:
            now = rclpy.time.Time()
            transform: TransformStamped = self.tf_buffer2.lookup_transform(
                'map',
                'base_link',
                now)
            self.x_car_world = transform.transform.translation.x
            self.y_car_world = transform.transform.translation.y
            self.get_waypoint()
            
            #####################################
            # self.waypoints.lookahead_point_world = np.array([self.waypoints.X[self.waypoints.index],
            #                                              self.waypoints.Y[self.waypoints.index], 0.0])
            # self.waypoints.current_point_world = np.array([self.waypoints.X[self.waypoints.velocity_index],
            #                                             self.waypoints.Y[self.waypoints.velocity_index], 0.0])

            # self.visualize_point(self.waypoints.lookahead_point_world, self.vis_lookahead_point_pub, 'r')
            # self.visualize_point(self.waypoints.current_point_world, self.vis_current_point_pub, 'b')

            # trans = transform.transform.translation
            # rot = transform.transform.rotation
            # translation_v = np.array([trans.x, trans.y, trans.z])
            # self.quat_to_rot(rot.w, rot.x, rot.y, rot.z)
            # self.waypoints.lookahead_point_car = self.rotation_m @ self.waypoints.lookahead_point_world + translation_v


            self.transformandinterp_waypoint()            
            self.visualize_point(self.waypoints.lookahead_point_car, self.vis_lookahead_point_car, 'r')

            angle = self.p_controller()
            self.publish_message(angle)
            
        except Exception as e:
            self.get_logger().warn(f'Could not get odom->base_link transform: {e}')

    def to_radians(self, degrees):
        return degrees * math.pi / 180.0

    def to_degrees(self, radians):
        return radians * 180.0 / math.pi

    def p2pdist(self, x1, x2, y1, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def load_waypoints(self):
        self.nodeSequence = [10, 4, 20, 10]
        self.roadmap = SDCSRoadMap(leftHandTraffic=False)
        self.waypointSequence = self.roadmap.generate_path(self.nodeSequence)
        self.waypoints_x = self.waypointSequence[0]
        self.waypoints_y = self.waypointSequence[1]
        
        for i, (x, y) in enumerate(zip(self.waypoints_x, self.waypoints_y)):
            dx, dy = (0.45, 1.35)
            angle_radians = math.radians(34)
            
            # Rotation around Z axis
            x_rot = x * math.cos(angle_radians) - y * math.sin(angle_radians)
            y_rot = x * math.sin(angle_radians) + y * math.cos(angle_radians)
            
            # Translation
            self.waypoints_x[i] = x_rot + dx
            self.waypoints_y[i] = y_rot + dy
        
        self.waypoints.X = self.waypoints_x
        self.waypoints.Y = self.waypoints_y 
        self.num_waypoints = len(self.waypoints.X)

    def visualize_point(self, point, topic, color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.25
        marker.color.a = 1.0
        setattr(marker.color, color, 1.0)
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.id = 1
        topic.publish(marker)

    def get_waypoint(self):
        longest_distance = 0
        final_i = -1
        start = self.waypoints.index
        end = (start + 500) % self.num_waypoints
        lookahead = min(max(self.min_lookahead, self.max_lookahead * self.curr_velocity / self.lookahead_ratio), self.max_lookahead)

        def within_range(i):
            d = self.p2pdist(self.waypoints.X[i], self.x_car_world, self.waypoints.Y[i], self.y_car_world)
            return lookahead >= d >= longest_distance

        index_range = list(range(start, self.num_waypoints)) + list(range(0, end)) if end < start else list(range(start, end))
        for i in index_range:
            if within_range(i):
                longest_distance = self.p2pdist(self.waypoints.X[i], self.x_car_world, self.waypoints.Y[i], self.y_car_world)
                final_i = i

        if final_i == -1:
            for i in range(self.num_waypoints):
                if within_range(i):
                    final_i = i

        shortest_distance = self.p2pdist(self.waypoints.X[0], self.x_car_world, self.waypoints.Y[0], self.y_car_world)
        velocity_i = 0
        for i in range(self.num_waypoints):
            d = self.p2pdist(self.waypoints.X[i], self.x_car_world, self.waypoints.Y[i], self.y_car_world)
            if d <= shortest_distance:
                shortest_distance = d
                velocity_i = i

        self.waypoints.index = final_i
        self.waypoints.velocity_index = velocity_i

    def quat_to_rot(self, q0, q1, q2, q3):
        self.rotation_m = np.array([
            [2 * (q0 * q0 + q1 * q1) - 1, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
            [2 * (q1 * q2 + q0 * q3), 2 * (q0 * q0 + q2 * q2) - 1, 2 * (q2 * q3 - q0 * q1)],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 2 * (q0 * q0 + q3 * q3) - 1]
        ])

    def transformandinterp_waypoint(self):
        self.waypoints.lookahead_point_world = np.array([self.waypoints.X[self.waypoints.index],
                                                         self.waypoints.Y[self.waypoints.index], 0.0])
        self.waypoints.current_point_world = np.array([self.waypoints.X[self.waypoints.velocity_index],
                                                       self.waypoints.Y[self.waypoints.velocity_index], 0.0])

        self.visualize_point(self.waypoints.lookahead_point_world, self.vis_lookahead_point_pub, 'r')
        self.visualize_point(self.waypoints.current_point_world, self.vis_current_point_pub, 'b')

        try:
            transform = self.tf_buffer2.lookup_transform("base_link", "map", rclpy.time.Time())
        except LookupException as ex:
            self.get_logger().info(f"Transform error: {ex}")
            return

        trans = transform.transform.translation
        rot = transform.transform.rotation
        translation_v = np.array([trans.x, trans.y, trans.z])
        self.quat_to_rot(rot.w, rot.x, rot.y, rot.z)
        self.waypoints.lookahead_point_car = self.rotation_m @ self.waypoints.lookahead_point_world + translation_v

    def p_controller(self):
        r = np.linalg.norm(self.waypoints.lookahead_point_car)
        y = self.waypoints.lookahead_point_car[1]
        return self.K_p * 2 * y / (r**2)

    def get_velocity(self, steering_angle):
        if self.waypoints.V and self.waypoints.V[self.waypoints.velocity_index]:
            return self.waypoints.V[self.waypoints.velocity_index] * self.velocity_percentage
        else:
            angle = abs(steering_angle)
            if angle < self.to_radians(10):
                return 6.0 * self.velocity_percentage
            elif angle <= self.to_radians(20):
                return 2.5 * self.velocity_percentage
            else:
                return 2.0 * self.velocity_percentage

    def publish_message(self, steering_angle):
        
        msg = MotorCommands()
        msg.motor_names.append("steering_angle")
        msg.motor_names.append("motor_throttle")
        steering = max(min(steering_angle, self.to_radians(self.steering_limit)), -self.to_radians(self.steering_limit))
        msg.values.append(steering)
        self.curr_velocity = self.get_velocity(steering)
        msg.values.append(self.curr_velocity)
        self.publisher_drive.publish(msg)

    def timer_callback(self):
        self.K_p = self.get_parameter("K_p").value
        self.velocity_percentage = self.get_parameter("velocity_percentage").value
        self.min_lookahead = self.get_parameter("min_lookahead").value
        self.max_lookahead = self.get_parameter("max_lookahead").value
        self.lookahead_ratio = self.get_parameter("lookahead_ratio").value
        self.steering_limit = self.get_parameter("steering_limit").value
        
        self.get_logger().info(str(self.K_p))
        self.get_logger().info(str(self.velocity_percentage))
        self.get_logger().info(str(self.min_lookahead))
        self.get_logger().info(str(self.max_lookahead))
        self.get_logger().info(str(self.lookahead_ratio))
        self.get_logger().info(str(self.steering_limit))


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
