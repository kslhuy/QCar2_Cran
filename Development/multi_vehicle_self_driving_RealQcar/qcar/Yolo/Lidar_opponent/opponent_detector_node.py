#!/usr/bin/env python3
"""
Opponent Detection Node (ROS 2) — Adapted from ForzaETH abd_tracker detect.py

This node detects obstacles on the QCar2 track by:
1. Converting LaserScan data to a 2D point cloud in the 'map' frame
2. Segmenting the point cloud into clusters using Adaptive Breakpoint Detection (ABD)
3. Fitting a bounding rectangle to each cluster to extract center position and size
4. Filtering detections by size, minimum point count, and proximity to the track waypoints

Subscribes:
    /scan               (sensor_msgs/LaserScan)   – raw LiDAR data
    /qcar/waypoints_xy  (std_msgs/Float32MultiArray) – track waypoints for on-track filtering

Publishes:
    /perception/raw_obstacles       (std_msgs/Float32MultiArray) – [cx, cy, size, theta] per obstacle
    /perception/detection_markers   (visualization_msgs/MarkerArray) – RViz2 visualization
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R


class OpponentDetectorNode(Node):
    """ROS 2 node for LiDAR-based obstacle detection."""

    def __init__(self):
        super().__init__('opponent_detector_node')

        # ===== Declare Parameters =====
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('min_obs_size', 5)        # min laser points per cluster
        self.declare_parameter('max_obs_size', 0.5)      # max obstacle width in meters
        self.declare_parameter('max_viewing_distance', 5.0)
        self.declare_parameter('track_margin', 0.5)      # max distance from waypoint to be "on track"
        self.declare_parameter('lambda_angle', 10.0)     # adaptive breakpoint threshold (degrees)
        self.declare_parameter('sigma', 0.01)            # LiDAR noise std dev
        self.declare_parameter('min_2_points_dist', 0.01)
        self.declare_parameter('lidar_frame', 'base_scan')

        # ===== Read Parameters =====
        self.rate = self.get_parameter('rate').value
        self.min_obs_size = self.get_parameter('min_obs_size').value
        self.max_obs_size = self.get_parameter('max_obs_size').value
        self.max_viewing_distance = self.get_parameter('max_viewing_distance').value
        self.track_margin = self.get_parameter('track_margin').value
        self.lambda_angle = math.radians(self.get_parameter('lambda_angle').value)
        self.sigma = self.get_parameter('sigma').value
        self.min_2_points_dist = self.get_parameter('min_2_points_dist').value
        self.lidar_frame = self.get_parameter('lidar_frame').value

        # ===== TF2 =====
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ===== State =====
        self.scans = None
        self.waypoints_xy = None  # Nx2 numpy array of [x, y]

        # ===== Subscribers =====
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10
        )
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray, '/qcar/waypoints_xy', self._waypoint_callback, 10
        )

        # ===== Publishers =====
        self.obstacles_pub = self.create_publisher(
            Float32MultiArray, '/perception/raw_obstacles', 10
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, '/perception/detection_markers', 10
        )

        # ===== Timer =====
        self.timer = self.create_timer(1.0 / self.rate, self._detect_callback)

        # ===== Dynamic parameter callback =====
        self.add_on_set_parameters_callback(self._param_callback)

        self.get_logger().info('[Opponent Detection]: Ready')

    # ===== Parameter Callback =====

    def _param_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'min_obs_size':
                self.min_obs_size = param.value
            elif param.name == 'max_obs_size':
                self.max_obs_size = param.value
            elif param.name == 'max_viewing_distance':
                self.max_viewing_distance = param.value
            elif param.name == 'track_margin':
                self.track_margin = param.value
            elif param.name == 'lambda_angle':
                self.lambda_angle = math.radians(param.value)
            elif param.name == 'sigma':
                self.sigma = param.value
        self.get_logger().info(
            f'[Opponent Detection]: Params updated — min_obs={self.min_obs_size}, '
            f'max_obs={self.max_obs_size:.2f}m, max_view={self.max_viewing_distance:.1f}m, '
            f'track_margin={self.track_margin:.2f}m'
        )
        return SetParametersResult(successful=True)

    # ===== ROS Callbacks =====

    def _scan_callback(self, msg: LaserScan):
        self.scans = msg

    def _waypoint_callback(self, msg: Float32MultiArray):
        """Parse waypoints from Float32MultiArray: first half = x values, second half = y values."""
        data = np.array(msg.data, dtype=np.float64)
        if len(data) < 4:
            return
        n = len(data) // 2
        self.waypoints_xy = np.column_stack([data[:n], data[n:]])

    # ===== Main Detection Loop =====

    def _detect_callback(self):
        if self.scans is None:
            return

        # Step 1: Convert scan to point cloud in map frame
        cloud_points = self._scans_to_map_cloud()
        if cloud_points is None or len(cloud_points) < 2:
            return

        # Step 2: Adaptive Breakpoint Segmentation
        clusters = self._segment_clusters(cloud_points)

        # Step 3: Filter small clusters
        clusters = [c for c in clusters if len(c) >= self.min_obs_size]

        # Step 4: Track proximity filtering (if waypoints available)
        if self.waypoints_xy is not None and len(self.waypoints_xy) > 0:
            clusters = self._filter_by_track_proximity(clusters)

        # Step 5: Rectangle fitting → extract obstacle center, size, angle
        obstacles = self._fit_rectangles(clusters)

        # Step 6: Filter by max obstacle size
        obstacles = [obs for obs in obstacles if obs['size'] <= self.max_obs_size]

        # Step 7: Publish
        self._publish_obstacles(obstacles)
        self._publish_markers(obstacles)

    # ===== Point Cloud Conversion =====

    def _scans_to_map_cloud(self):
        """Transform LaserScan ranges to 2D points in the 'map' frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', self.lidar_frame, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warning(
                f'[Opponent Detection]: TF lookup map←{self.lidar_frame} failed: {e}',
                throttle_duration_sec=5.0
            )
            return None

        # Extract translation and rotation from TF
        t = transform.transform.translation
        q = transform.transform.rotation
        T = np.array([t.x, t.y, t.z])
        rot_matrix = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        scan = self.scans
        n_points = len(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, n_points)
        ranges = np.array(scan.ranges, dtype=np.float64)

        # Filter out invalid ranges
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)

        # Filter by max viewing distance
        valid &= (ranges <= self.max_viewing_distance)

        if np.sum(valid) < 2:
            return None

        # Convert polar to Cartesian in LiDAR frame
        x_lf = ranges[valid] * np.cos(angles[valid])
        y_lf = ranges[valid] * np.sin(angles[valid])
        z_lf = np.zeros_like(x_lf)

        # 3D points in LiDAR frame (Nx3)
        pts_lidar = np.column_stack([x_lf, y_lf, z_lf])

        # Transform to map frame: p_map = R @ p_lidar + T
        pts_map = (rot_matrix @ pts_lidar.T).T + T

        # Return Nx2 (x, y only)
        return pts_map[:, :2]

    # ===== Adaptive Breakpoint Segmentation =====

    def _segment_clusters(self, cloud_points):
        """
        Segment 2D point cloud into clusters using Adaptive Breakpoint Detection.
        Points that are far apart (relative to their distance from sensor) start a new cluster.
        """
        if len(cloud_points) < 2:
            return []

        d_phi = self.scans.angle_increment if self.scans else 0.004  # ~0.2 degrees
        lam = self.lambda_angle
        sigma = self.sigma

        clusters = [[cloud_points[0].tolist()]]

        for idx in range(1, len(cloud_points)):
            pt = cloud_points[idx]
            pt_prev = cloud_points[idx - 1]

            # Distance from sensor origin (approx)
            dist = np.linalg.norm(pt)

            # Adaptive threshold
            d_max = (dist * math.sin(d_phi) / math.sin(lam - d_phi) + 3 * sigma) / 2.0

            # Distance between consecutive points
            gap = np.linalg.norm(pt - pt_prev)

            if gap > d_max:
                # Start a new cluster
                clusters.append([pt.tolist()])
            else:
                clusters[-1].append(pt.tolist())

        return clusters

    # ===== Track Proximity Filtering =====

    def _filter_by_track_proximity(self, clusters):
        """
        Filter clusters by proximity to the waypoint path.
        Only keep clusters whose center is within track_margin of the nearest waypoint.
        """
        if self.waypoints_xy is None or len(self.waypoints_xy) == 0:
            return clusters

        filtered = []
        for cluster in clusters:
            pts = np.array(cluster)
            center = pts.mean(axis=0)  # center of the cluster

            # Distance from center to all waypoints → take minimum
            dists = np.linalg.norm(self.waypoints_xy - center, axis=1)
            min_dist = np.min(dists)

            if min_dist <= self.track_margin:
                filtered.append(cluster)

        return filtered

    # ===== Rectangle Fitting =====

    def _fit_rectangles(self, clusters):
        """
        Fit a bounding rectangle to each cluster to extract center, size, and angle.
        Adapted from abd_tracker detect.py obsPointClouds2obsArray.
        """
        obstacles = []
        min_dist = self.min_2_points_dist

        for cluster in clusters:
            pts = np.array(cluster, dtype=np.float64)
            if len(pts) < 2:
                continue

            # --- Rectangle fitting via rotation sweep ---
            theta_range = np.linspace(0, np.pi / 2 - np.pi / 180, 90)
            cos_theta = np.cos(theta_range)
            sin_theta = np.sin(theta_range)

            # Project points onto rotated axes
            dist1 = pts @ np.vstack([cos_theta, sin_theta])  # Nx90
            dist2 = pts @ np.vstack([-sin_theta, cos_theta])  # Nx90

            D10 = -dist1 + np.amax(dist1, axis=0)
            D11 = dist1 - np.amin(dist1, axis=0)
            D20 = -dist2 + np.amax(dist2, axis=0)
            D21 = dist2 - np.amin(dist2, axis=0)

            min_array = np.argmin([np.linalg.norm(D10, axis=0),
                                   np.linalg.norm(D11, axis=0)], axis=0)
            D10_t = D10.T.copy()
            D11_t = D11.T.copy()
            D10_t[min_array == 1] = D11_t[min_array == 1]
            D10 = D10_t.T

            min_array = np.argmin([np.linalg.norm(D20, axis=0),
                                   np.linalg.norm(D21, axis=0)], axis=0)
            D20_t = D20.T.copy()
            D21_t = D21.T.copy()
            D20_t[min_array == 1] = D21_t[min_array == 1]
            D20 = D20_t.T

            D = np.minimum(D10, D20)
            D[D < min_dist] = min_dist

            # Optimal rotation angle
            theta_opt_idx = np.argmax(np.sum(np.reciprocal(D), axis=0))
            theta_opt = theta_opt_idx * np.pi / 180

            # Project with optimal angle
            cos_opt = np.cos(theta_opt)
            sin_opt = np.sin(theta_opt)
            d1 = pts @ np.array([cos_opt, sin_opt])
            d2 = pts @ np.array([-sin_opt, cos_opt])

            max_d1, min_d1 = np.max(d1), np.min(d1)
            max_d2, min_d2 = np.max(d2), np.min(d2)

            # Determine corners based on point distribution
            if np.var(d2) > np.var(d1):
                if np.linalg.norm(-d1 + max_d1) < np.linalg.norm(d1 - min_d1):
                    corner1 = np.array([cos_opt * max_d1 - sin_opt * min_d2,
                                        sin_opt * max_d1 + cos_opt * min_d2])
                    corner2 = np.array([cos_opt * max_d1 - sin_opt * max_d2,
                                        sin_opt * max_d1 + cos_opt * max_d2])
                else:
                    corner1 = np.array([cos_opt * min_d1 - sin_opt * max_d2,
                                        sin_opt * min_d1 + cos_opt * max_d2])
                    corner2 = np.array([cos_opt * min_d1 - sin_opt * min_d2,
                                        sin_opt * min_d1 + cos_opt * min_d2])
            else:
                if np.linalg.norm(-d2 + max_d2) < np.linalg.norm(d2 - min_d2):
                    corner1 = np.array([cos_opt * max_d1 - sin_opt * max_d2,
                                        sin_opt * max_d1 + cos_opt * max_d2])
                    corner2 = np.array([cos_opt * min_d1 - sin_opt * max_d2,
                                        sin_opt * min_d1 + cos_opt * max_d2])
                else:
                    corner1 = np.array([cos_opt * min_d1 - sin_opt * min_d2,
                                        sin_opt * min_d1 + cos_opt * min_d2])
                    corner2 = np.array([cos_opt * max_d1 - sin_opt * min_d2,
                                        sin_opt * max_d1 + cos_opt * min_d2])

            col_vec = corner2 - corner1
            orth_vec = np.array([-col_vec[1], col_vec[0]])
            center = corner1 + 0.5 * col_vec + 0.5 * orth_vec
            size = np.linalg.norm(col_vec)

            obstacles.append({
                'center_x': float(center[0]),
                'center_y': float(center[1]),
                'size': float(size),
                'theta': float(theta_opt),
            })

        return obstacles

    # ===== Publishing =====

    def _publish_obstacles(self, obstacles):
        """Publish raw obstacles as Float32MultiArray: [cx, cy, size, theta] per obstacle."""
        msg = Float32MultiArray()
        data = []
        for obs in obstacles:
            data.extend([obs['center_x'], obs['center_y'], obs['size'], obs['theta']])
        msg.data = [float(v) for v in data]
        self.obstacles_pub.publish(msg)

    def _publish_markers(self, obstacles):
        """Publish RViz markers for detected obstacles."""
        marker_array = MarkerArray()

        # Clear old markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for idx, obs in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = obs['center_x']
            marker.pose.position.y = obs['center_y']
            marker.pose.position.z = 0.1

            # Rotation from theta
            q = R.from_euler('z', obs['theta']).as_quat()
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]

            marker.scale.x = max(obs['size'], 0.05)
            marker.scale.y = max(obs['size'], 0.05)
            marker.scale.z = 0.15

            # Cyan color for raw detections
            marker.color.a = 0.6
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.lifetime = Duration(sec=0, nanosec=int(0.2 * 1e9))

            marker_array.markers.append(marker)

        self.markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
