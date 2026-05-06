#!/usr/bin/env python3
"""
Opponent Tracking Node (ROS 2) — Adapted from ForzaETH abd_tracker tracking.py

This node tracks detected obstacles over time by:
1. Associating new detections with existing tracked obstacles (nearest-neighbor)
2. Classifying obstacles as static or dynamic based on position variance
3. Predicting dynamic obstacle motion using an Extended Kalman Filter (Cartesian)
4. Managing obstacle lifecycle with time-to-live (TTL)

Subscribes:
    /perception/raw_obstacles  (std_msgs/Float32MultiArray)  – from detector node
    /odom                      (nav_msgs/Odometry)           – ego car state
    /scan                      (sensor_msgs/LaserScan)       – for field-of-view checks

Publishes:
    /perception/tracked_opponents   (std_msgs/Float32MultiArray) – tracked opponent data
    /perception/tracking_markers    (visualization_msgs/MarkerArray) – RViz2 visualization
"""

import math
import time as time_module
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation as R

from filterpy.kalman import ExtendedKalmanFilter as EKF
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag


# ===== Tracked Obstacle (static/dynamic classification) =====

class TrackedObstacle:
    """Stores measurement history and classification for one tracked obstacle."""

    def __init__(self, obs_id, x, y, size):
        self.id = obs_id
        self.measurements_x = [x]
        self.measurements_y = [y]
        self.mean_x = x
        self.mean_y = y
        self.size = size
        self.nb_meas = 0
        self.static_count = 0
        self.total_count = 0
        self.ttl = 10  # will be set from parameters
        self.is_visible = True
        self.static_flag = None  # None=unclassified, True=static, False=dynamic

    def update_mean(self):
        n = self.nb_meas
        if n == 0:
            self.mean_x = self.measurements_x[-1]
            self.mean_y = self.measurements_y[-1]
        else:
            self.mean_x = (self.mean_x * n + self.measurements_x[-1]) / (n + 1)
            self.mean_y = (self.mean_y * n + self.measurements_y[-1]) / (n + 1)

    def std_x(self):
        return float(np.std(self.measurements_x))

    def std_y(self):
        return float(np.std(self.measurements_y))

    def classify(self, min_nb_meas, min_std, max_std):
        """Classify obstacle as static or dynamic based on position variance."""
        if self.nb_meas > min_nb_meas:
            sx = self.std_x()
            sy = self.std_y()

            if sx < min_std and sy < min_std:
                self.static_count += 1
            elif sx > max_std or sy > max_std:
                self.static_count = 0

            self.total_count += 1
            self.static_flag = (self.static_count / max(self.total_count, 1)) >= 0.5
        else:
            self.static_flag = None

    def add_measurement(self, x, y, size):
        self.measurements_x.append(x)
        self.measurements_y.append(y)
        # Keep measurement history bounded
        if len(self.measurements_x) > 30:
            self.measurements_x = self.measurements_x[-20:]
            self.measurements_y = self.measurements_y[-20:]
        self.update_mean()
        self.nb_meas += 1
        self.is_visible = True
        self.size = size


# ===== Dynamic Opponent State (EKF-based tracker) =====

class DynamicOpponentEKF:
    """Extended Kalman Filter for a dynamic opponent in Cartesian coordinates.

    State: [x, vx, y, vy]
    Measurement: [x, vx, y, vy]  (position + finite-difference velocity)
    """

    def __init__(self, rate, measurement_var_x, measurement_var_y,
                 process_var_vx, process_var_vy):
        self.rate = rate
        dt = 1.0 / rate

        self.kf = EKF(dim_x=4, dim_z=4)
        # State transition: constant velocity model
        self.kf.F = np.array([
            [1., dt, 0., 0.],
            [0., 1., 0., 0.],
            [0., 0., 1., dt],
            [0., 0., 0., 1.],
        ])

        # Process noise
        q1 = Q_discrete_white_noise(dim=2, dt=dt, var=process_var_vx)
        q2 = Q_discrete_white_noise(dim=2, dt=dt, var=process_var_vy)
        self.kf.Q = block_diag(q1, q2)

        # Measurement function (identity)
        self.kf.H = np.eye(4)

        # Measurement noise
        self.kf.R = np.diag([
            measurement_var_x, process_var_vx,
            measurement_var_y, process_var_vy
        ])

        # Initial covariance
        self.kf.P = np.diag([
            measurement_var_x, process_var_vx,
            measurement_var_y, process_var_vy
        ])

        self.kf.B = np.eye(4)

        self.is_initialized = False
        self.id = -1
        self.size = 0.0
        self.ttl = 40
        self.vs_list = []
        self.avg_speed = 0.0
        self.vx_filt = np.zeros(5)
        self.vy_filt = np.zeros(5)

    def initialize(self, tracked_obs: TrackedObstacle):
        """Initialize EKF from measurement history of a tracked obstacle."""
        if len(tracked_obs.measurements_x) < 2:
            return

        x = tracked_obs.measurements_x[-1]
        y = tracked_obs.measurements_y[-1]
        vx = (tracked_obs.measurements_x[-1] - tracked_obs.measurements_x[-2]) * self.rate
        vy = (tracked_obs.measurements_y[-1] - tracked_obs.measurements_y[-2]) * self.rate

        self.kf.x = np.array([x, vx, y, vy])
        self.is_initialized = True
        self.id = tracked_obs.id
        self.size = tracked_obs.size
        self.avg_speed = 0.0
        self.vs_list = []

    def predict(self):
        if not self.is_initialized:
            return
        # Simple constant velocity prediction (no control input)
        self.kf.predict(u=np.zeros(4))

    def update(self, tracked_obs: TrackedObstacle):
        if not self.is_initialized:
            return
        if len(tracked_obs.measurements_x) < 3:
            return

        # Finite-difference velocity
        vx = (
            (2.0 / 3.0) * (tracked_obs.measurements_x[-1] - tracked_obs.measurements_x[-2]) * self.rate +
            (1.0 / 3.0) * (tracked_obs.measurements_x[-2] - tracked_obs.measurements_x[-3]) * self.rate
        )
        vy = (
            (2.0 / 3.0) * (tracked_obs.measurements_y[-1] - tracked_obs.measurements_y[-2]) * self.rate +
            (1.0 / 3.0) * (tracked_obs.measurements_y[-2] - tracked_obs.measurements_y[-3]) * self.rate
        )

        # Sanity check on velocity
        speed = math.sqrt(vx ** 2 + vy ** 2)
        if speed > 8.0:
            self.is_initialized = False
            return

        z = np.array([
            tracked_obs.measurements_x[-1], vx,
            tracked_obs.measurements_y[-1], vy,
        ])

        # Standard EKF update with identity H
        self.kf.update(
            z,
            lambda x: np.eye(4),  # Hjac
            lambda x: x,          # hx (identity)
        )

        # Update velocity tracking
        self.vs_list.append(speed)
        if len(self.vs_list) > 20:
            self.vs_list = self.vs_list[-10:]
        self.avg_speed = np.mean(self.vs_list) if self.vs_list else 0.0

        self.vx_filt = np.roll(self.vx_filt, 1)
        self.vx_filt[0] = self.kf.x[1]
        self.vy_filt = np.roll(self.vy_filt, 1)
        self.vy_filt[0] = self.kf.x[3]

    @property
    def x(self):
        return float(self.kf.x[0])

    @property
    def vx(self):
        return float(np.mean(self.vx_filt))

    @property
    def y(self):
        return float(self.kf.x[2])

    @property
    def vy(self):
        return float(np.mean(self.vy_filt))

    @property
    def position_variance(self):
        return float(self.kf.P[0, 0])


# ===== Main Tracking Node =====

class OpponentTrackerNode(Node):
    """ROS 2 node for obstacle tracking with static/dynamic classification + EKF."""

    def __init__(self):
        super().__init__('opponent_tracker_node')

        # ===== Declare Parameters =====
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('max_association_dist', 0.5)
        self.declare_parameter('ttl_static', 10)
        self.declare_parameter('ttl_dynamic', 40)
        self.declare_parameter('min_nb_meas', 5)
        self.declare_parameter('min_std', 0.05)
        self.declare_parameter('max_std', 0.10)
        self.declare_parameter('vs_reset', 0.05)
        self.declare_parameter('publish_static', True)
        self.declare_parameter('measurement_var_x', 0.01)
        self.declare_parameter('measurement_var_y', 0.01)
        self.declare_parameter('process_var_vx', 0.1)
        self.declare_parameter('process_var_vy', 0.1)
        self.declare_parameter('var_pub_threshold', 1.0)
        self.declare_parameter('aggro_multiplier', 2.0)

        # ===== Read Parameters =====
        self.rate = self.get_parameter('rate').value
        self.max_association_dist = self.get_parameter('max_association_dist').value
        self.ttl_static = self.get_parameter('ttl_static').value
        self.ttl_dynamic = self.get_parameter('ttl_dynamic').value
        self.min_nb_meas = self.get_parameter('min_nb_meas').value
        self.min_std = self.get_parameter('min_std').value
        self.max_std = self.get_parameter('max_std').value
        self.vs_reset = self.get_parameter('vs_reset').value
        self.publish_static = self.get_parameter('publish_static').value
        self.measurement_var_x = self.get_parameter('measurement_var_x').value
        self.measurement_var_y = self.get_parameter('measurement_var_y').value
        self.process_var_vx = self.get_parameter('process_var_vx').value
        self.process_var_vy = self.get_parameter('process_var_vy').value
        self.var_pub_threshold = self.get_parameter('var_pub_threshold').value
        self.aggro_multiplier = self.get_parameter('aggro_multiplier').value

        # ===== State =====
        self.tracked_obstacles = []  # List[TrackedObstacle]
        self.opponent_ekf = DynamicOpponentEKF(
            rate=self.rate,
            measurement_var_x=self.measurement_var_x,
            measurement_var_y=self.measurement_var_y,
            process_var_vx=self.process_var_vx,
            process_var_vy=self.process_var_vy,
        )
        self.current_id = 1
        self.raw_obstacles = []  # Latest raw detections
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_theta = 0.0
        self.scans = None

        # ===== Subscribers =====
        self.raw_obs_sub = self.create_subscription(
            Float32MultiArray, '/perception/raw_obstacles', self._raw_obs_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10
        )

        # ===== Publishers =====
        self.tracked_pub = self.create_publisher(
            Float32MultiArray, '/perception/tracked_opponents', 10
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, '/perception/tracking_markers', 10
        )

        # ===== Timer =====
        self.timer = self.create_timer(1.0 / self.rate, self._tracking_callback)

        # ===== Dynamic parameter callback =====
        self.add_on_set_parameters_callback(self._param_callback)

        self.get_logger().info('[Opponent Tracking]: Ready')

    # ===== Parameter Callback =====

    def _param_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'max_association_dist':
                self.max_association_dist = param.value
            elif param.name == 'ttl_static':
                self.ttl_static = param.value
            elif param.name == 'ttl_dynamic':
                self.ttl_dynamic = param.value
            elif param.name == 'min_nb_meas':
                self.min_nb_meas = param.value
            elif param.name == 'min_std':
                self.min_std = param.value
            elif param.name == 'max_std':
                self.max_std = param.value
            elif param.name == 'publish_static':
                self.publish_static = param.value
        return SetParametersResult(successful=True)

    # ===== ROS Callbacks =====

    def _raw_obs_callback(self, msg: Float32MultiArray):
        """Parse raw obstacles: each 4 floats = [cx, cy, size, theta]."""
        data = msg.data
        obstacles = []
        FIELDS = 4
        n = len(data) // FIELDS
        for i in range(n):
            offset = i * FIELDS
            obstacles.append({
                'center_x': data[offset],
                'center_y': data[offset + 1],
                'size': data[offset + 2],
                'theta': data[offset + 3],
            })
        self.raw_obstacles = obstacles

    def _odom_callback(self, msg: Odometry):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        try:
            euler = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
            self.car_theta = euler[2]
        except Exception:
            pass

    def _scan_callback(self, msg: LaserScan):
        self.scans = msg

    # ===== Main Tracking Loop =====

    def _tracking_callback(self):
        # Predict dynamic opponent
        if self.opponent_ekf.is_initialized:
            self.opponent_ekf.predict()

        # Update tracked obstacles with new detections
        self._update_tracking()

        # Publish results
        self._publish_tracked_opponents()
        self._publish_markers()

    def _update_tracking(self):
        """Associate detections with tracked obstacles, classify, and manage lifecycle."""
        meas_copy = list(self.raw_obstacles)
        removal_list = []

        for tracked in self.tracked_obstacles:
            # Try to associate with a measured obstacle
            matched, best_meas = self._find_association(tracked, meas_copy)

            if matched:
                tracked.add_measurement(best_meas['center_x'], best_meas['center_y'], best_meas['size'])
                tracked.classify(self.min_nb_meas, self.min_std, self.max_std)
                tracked.ttl = self.ttl_static if tracked.static_flag else self.ttl_dynamic

                # Handle dynamic classification
                if tracked.static_flag is False:  # dynamic
                    if self.opponent_ekf.is_initialized and self.opponent_ekf.id == tracked.id:
                        # Reset to static if speed too low
                        if (self.opponent_ekf.avg_speed < self.vs_reset and
                                len(self.opponent_ekf.vs_list) > 10):
                            self.opponent_ekf.is_initialized = False
                            tracked.static_flag = True
                            tracked.static_count = 0
                            tracked.total_count = 0
                            tracked.nb_meas = 0
                        else:
                            self.opponent_ekf.update(tracked)
                            self.opponent_ekf.id = tracked.id
                            self.opponent_ekf.ttl = self.ttl_dynamic
                            self.opponent_ekf.size = tracked.size
                    else:
                        self.opponent_ekf.initialize(tracked)

                meas_copy.remove(best_meas)

            else:
                # Not matched — decrease TTL
                tracked.ttl -= 1
                tracked.is_visible = False
                if tracked.ttl <= 0:
                    if tracked.static_flag is False:
                        pass  # Dynamic obstacle lost
                    removal_list.append(tracked)

        # Update dynamic obstacle TTL
        if self.opponent_ekf.is_initialized:
            self.opponent_ekf.ttl -= 1
            if self.opponent_ekf.ttl <= 0:
                self.opponent_ekf.is_initialized = False

        # Remove dead obstacles
        for dead in removal_list:
            self.tracked_obstacles.remove(dead)

        # Add unmatched detections as new tracked obstacles
        for meas in meas_copy:
            new_obs = TrackedObstacle(
                obs_id=self.current_id,
                x=meas['center_x'],
                y=meas['center_y'],
                size=meas['size'],
            )
            new_obs.ttl = self.ttl_static
            self.tracked_obstacles.append(new_obs)
            self.current_id += 1

    def _find_association(self, tracked, meas_list):
        """Find the nearest measurement to a tracked obstacle within max_association_dist."""
        max_dist = self.max_association_dist

        # For dynamic obstacles, use EKF predicted position
        if (tracked.static_flag is False and
                self.opponent_ekf.is_initialized and
                self.opponent_ekf.id == tracked.id):
            ref_x = self.opponent_ekf.x
            ref_y = self.opponent_ekf.y
            max_dist *= self.aggro_multiplier
        else:
            ref_x = tracked.mean_x
            ref_y = tracked.mean_y

        best_meas = None
        best_dist = float('inf')

        for meas in meas_list:
            d = math.sqrt((ref_x - meas['center_x']) ** 2 +
                          (ref_y - meas['center_y']) ** 2)
            if d < max_dist and d < best_dist:
                best_dist = d
                best_meas = meas

        # Fallback: for dynamic obstacles, try mean position if EKF prediction failed
        if best_meas is None and tracked.static_flag is False:
            ref_x = tracked.mean_x
            ref_y = tracked.mean_y
            for meas in meas_list:
                d = math.sqrt((ref_x - meas['center_x']) ** 2 +
                              (ref_y - meas['center_y']) ** 2)
                if d < max_dist and d < best_dist:
                    best_dist = d
                    best_meas = meas

        return best_meas is not None, best_meas

    # ===== Publishing =====

    def _publish_tracked_opponents(self):
        """Publish tracked opponents as Float32MultiArray.

        Per opponent (10 floats):
            [id, x, y, size, vx, vy, is_static, is_visible, distance_to_ego, confidence]
        """
        msg = Float32MultiArray()
        data = []

        for tracked in self.tracked_obstacles:
            # Static or unclassified obstacles
            if tracked.static_flag is not False:
                if not self.publish_static and tracked.static_flag is True:
                    continue

                x = tracked.mean_x
                y = tracked.mean_y
                vx, vy = 0.0, 0.0
                is_static = 1.0 if tracked.static_flag is True else 0.5  # 0.5 = unclassified
                distance = math.sqrt((x - self.car_x) ** 2 + (y - self.car_y) ** 2)
                confidence = min(tracked.nb_meas / max(self.min_nb_meas, 1), 1.0)

                data.extend([
                    float(tracked.id), x, y, tracked.size,
                    vx, vy, is_static,
                    1.0 if tracked.is_visible else 0.0,
                    distance, confidence,
                ])

        # Add dynamic opponent from EKF
        if self.opponent_ekf.is_initialized:
            if self.opponent_ekf.position_variance < self.var_pub_threshold:
                x = self.opponent_ekf.x
                y = self.opponent_ekf.y
                vx = self.opponent_ekf.vx
                vy = self.opponent_ekf.vy
                distance = math.sqrt((x - self.car_x) ** 2 + (y - self.car_y) ** 2)
                confidence = 1.0 - min(self.opponent_ekf.position_variance, 1.0)

                data.extend([
                    float(self.opponent_ekf.id), x, y, self.opponent_ekf.size,
                    vx, vy, 0.0,  # is_static = False
                    1.0,  # is_visible
                    distance, confidence,
                ])

        msg.data = [float(v) for v in data]
        self.tracked_pub.publish(msg)

    def _publish_markers(self):
        """Publish RViz markers for tracked obstacles (color-coded by classification)."""
        marker_array = MarkerArray()

        # Clear old markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        now = self.get_clock().now().to_msg()

        for tracked in self.tracked_obstacles:
            if tracked.static_flag is False:
                continue  # Dynamic shown via EKF below

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.id = tracked.id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = tracked.mean_x
            marker.pose.position.y = tracked.mean_y
            marker.pose.position.z = 0.15

            scale = 0.3 if tracked.is_visible else 0.15
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale

            marker.color.a = 0.7

            if tracked.static_flag is None:
                # Unclassified → pink
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            elif tracked.static_flag:
                # Static → green
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            marker.lifetime = Duration(sec=0, nanosec=int(0.3 * 1e9))
            if self.publish_static or tracked.static_flag is None:
                marker_array.markers.append(marker)

        # Dynamic opponent from EKF
        if self.opponent_ekf.is_initialized:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.id = self.opponent_ekf.id + 10000  # avoid ID collision
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.opponent_ekf.x
            marker.pose.position.y = self.opponent_ekf.y
            marker.pose.position.z = 0.2

            scale = 0.4 if self.opponent_ekf.position_variance < self.var_pub_threshold else 0.2
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale

            # Dynamic → red
            marker.color.a = 0.8
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker.lifetime = Duration(sec=0, nanosec=int(0.3 * 1e9))
            marker_array.markers.append(marker)

        self.markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
