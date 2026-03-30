"""
ROS 2 Version of vehicle_main.py (Full Vehicle Control System) - QCar Coordinate Style

Architecture:
  TF tree: SDCQcar -> map -> odom -> base_link
  - SDCQcar: The QCar/SDCSRoadMap world frame (trajectories live here)
  - map: ROS SLAM/AMCL frame (localization and navigation work here normally)
  - Static TF SDCQcar->map is published in the launch file (calibration)
  - VehicleLogic gets robot pose in SDCQcar via TF lookup
  - AMCL, Nav2, RViz all work in map frame as usual
  - NO manual coordinate transforms in code
"""

import sys
import os
import time
import numpy as np
from threading import Event
import math
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, PoseStamped , PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from limo_msgs.msg import LimoStatus
from std_msgs.msg import String, Float32MultiArray
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R

# ===== ADD PATH TO QCAR FOLDER =====
current_dir = os.path.dirname(os.path.abspath(__file__))
qcar_path = os.path.dirname(current_dir)
qcar_module_path = current_dir

if os.path.exists(qcar_path):
    if qcar_path not in sys.path:
        sys.path.insert(0, qcar_path)
        print(f"[PATH] Added to sys.path: {qcar_path}")
else:
    print(f"[PATH ERROR] QCar path not found: {qcar_path}")
    raise FileNotFoundError(f"Required directory not found: {qcar_path}")

if os.path.exists(qcar_module_path):
    if qcar_module_path not in sys.path:
        sys.path.insert(0, qcar_module_path)
        print(f"[PATH] Added to sys.path: {qcar_module_path}")
else:
    print(f"[PATH ERROR] QCar module path not found: {qcar_module_path}")
    raise FileNotFoundError(f"Required directory not found: {qcar_module_path}")

from qcar.config_main import VehicleMainConfig
from qcar.vehicle_logic import VehicleLogic
from qcar.command_types import CommandType
from limo.limo import ROSQCarAdapter, ROSGPSAdapterQCar


# ===== MAIN ROS NODE (QCar Style) =====
class VehicleControlFullSystemQCar(Node):
    """ROS 2 wrapper for complete VehicleLogic system - QCar coordinate style"""
    
    def __init__(self):
        super().__init__('vehicle_control_full_system_qcar')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("="*70)
        self.get_logger().info("Initializing Full Vehicle Control System (QCar Coordinate Style)")
        self.get_logger().info("="*70)
        
        # ===== ROS PARAMETERS =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('car_id', 3),
                ('vehicle_type', 'Limo'),
                ('programme_type', 'Ros'),
                ('v_ref', 0.6),
                ('controller_rate', 100),
                ('calibrate', False),
                ('path_number', 0),
                ('no_steering', False),
                ('config_file', ''),
                ('log_dir', ''),
                ('data_log_dir', ''),
                ('initial_x', 0.0),
                ('initial_y', 0.0),
                ('initial_yaw', 0.0),
                ('send_initial_pose_on_startup', False),
                ('enable_sdc_initialpose_listener', True),
                ('sdc_initialpose_topic', '/initialpose_sdc'),
                ('sdc_initialpose_input_frame', 'SDCQcar'),
                ('require_path_for_ready', False),
                ('host', ''),
                ('port', 0),
            ]
        )
        
        car_id = self.get_parameter('car_id').value
        vehicle_type = self.get_parameter('vehicle_type').value
        programme_type = self.get_parameter('programme_type').value
        v_ref = self.get_parameter('v_ref').value
        controller_rate = self.get_parameter('controller_rate').value
        calibrate = self.get_parameter('calibrate').value
        path_number = self.get_parameter('path_number').value
        no_steering = self.get_parameter('no_steering').value
        config_file = self.get_parameter('config_file').value
        log_dir = self.get_parameter('log_dir').value
        data_log_dir = self.get_parameter('data_log_dir').value
        initial_x = self.get_parameter('initial_x').value
        initial_y = self.get_parameter('initial_y').value
        initial_yaw = self.get_parameter('initial_yaw').value
        send_initial_pose_on_startup = self.get_parameter(
            'send_initial_pose_on_startup').value
        enable_sdc_initialpose_listener = self.get_parameter(
            'enable_sdc_initialpose_listener').value
        sdc_initialpose_topic = self.get_parameter(
            'sdc_initialpose_topic').value
        sdc_initialpose_input_frame = self.get_parameter(
            'sdc_initialpose_input_frame').value
        require_path_for_ready = self.get_parameter(
            'require_path_for_ready').value
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        
        self.get_logger().info(f"QCar Style - Car ID: {car_id}, v_ref: {v_ref}, rate: {controller_rate} Hz")
        
        # ===== INITIALIZE DATA STORAGE =====
        self.latest_odom = None
        self.latest_imu = None
        self.latest_limo_status = None
        
        # ===== ROS PUBLISHERS =====
        self.motor_pub = self.create_publisher(Twist, "/cmd_vel", 30)
        
        # ===== CREATE ROS ADAPTERS (QCar Style) =====
        self.qcar_adapter = ROSQCarAdapter(self)
        self.gps_adapter = ROSGPSAdapterQCar(self, self.tf_buffer)
        self.get_logger().info("✓ ROS hardware adapters created (QCar style)")

        self.sdc_initialpose_topic = str(sdc_initialpose_topic)
        self.sdc_initialpose_input_frame = str(sdc_initialpose_input_frame).strip()
        if not self.sdc_initialpose_input_frame:
            self.sdc_initialpose_input_frame = 'SDCQcar'

        self.pending_initial_pose_xyz_deg = None
        self.pending_initial_pose_source = ""
        self.pending_initial_pose_wait_logged = False
        self.initial_pose_timer = self.create_timer(
            0.2, self._publish_pending_initial_pose_when_ready)
        self.require_path_for_ready = bool(require_path_for_ready)

        if send_initial_pose_on_startup:
            self.pending_initial_pose_xyz_deg = (
                float(initial_x), float(initial_y), float(initial_yaw))
            self.pending_initial_pose_source = "startup parameters (map frame)"
            self.get_logger().info(
                "Startup initial pose is enabled "
                f"(x={initial_x:.3f}, y={initial_y:.3f}, yaw_deg={initial_yaw:.3f})")
        else:
            self.get_logger().info(
                "Startup initial pose is disabled; use RViz 2D Pose Estimate.")
        
        # ===== ROS SUBSCRIPTIONS =====
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 15
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_callback, 15
        )
        self.limo_status_sub = self.create_subscription(
            LimoStatus, '/limo_status', self._limo_status_callback, 10
        )
        
        # Subscribe to QCar-style path topic
        self.path_sub = self.create_subscription(
            Path, '/plan_qcar', self._path_callback, 10
        )

        if enable_sdc_initialpose_listener:
            self.sdc_initialpose_sub = self.create_subscription(
                PoseStamped,
                self.sdc_initialpose_topic,
                self._sdc_initial_pose_callback,
                10,
            )
            self.get_logger().info(
                "SDC initial pose listener enabled "
                f"(topic={self.sdc_initialpose_topic}, default_frame={self.sdc_initialpose_input_frame})")
        else:
            self.sdc_initialpose_sub = None
            self.get_logger().info("SDC initial pose listener disabled.")
        
        # ===== LOAD CONFIGURATION =====
        self.get_logger().info("Loading VehicleMainConfig...")
        
        if config_file and config_file.strip():
            config_path = config_file.strip()
        else:
            fleet_config_path = os.path.join(qcar_path, 'qcar', 'fleet_config.yaml')
            local_config_path = os.path.join(qcar_path, 'qcar', 'config_vehicle_main.yaml')
            if os.path.exists(fleet_config_path):
                config_path = fleet_config_path
                self.get_logger().info("No config_file provided; using fleet_config.yaml")
            else:
                config_path = local_config_path
                self.get_logger().info(
                    "No config_file provided; fleet_config.yaml not found, using config_vehicle_main.yaml")
        
        if os.path.exists(config_path):
            if config_path.endswith('.json'):
                config = VehicleMainConfig.from_json(config_path)
            elif config_path.endswith('.yaml') or config_path.endswith('.yml'):
                with open(config_path, 'r') as f:
                    raw_config = yaml.safe_load(f) or {}

                if isinstance(raw_config, dict) and 'vehicles' in raw_config:
                    config = VehicleMainConfig.from_fleet_yaml(config_path, int(car_id))
                    self.get_logger().info(
                        f"Detected fleet config format; loading vehicle settings for car_id={car_id}")
                else:
                    config = VehicleMainConfig.from_yaml(config_path)
            else:
                raise ValueError(f"Invalid config file format: {config_path}")
            self.get_logger().info(f"Loaded config from: {config_path}")
        else:
            self.get_logger().warning(
                f"Config file not found: {config_path}. Using default VehicleMainConfig()")
            config = VehicleMainConfig()
            
        config.network.car_id = car_id
        if host and str(host).strip():
            config.network.host_ip = str(host).strip()
        if port:
            config.network.base_port = int(port)
            
        vehicle_type_normalized = str(vehicle_type).strip().lower()
        if vehicle_type_normalized == 'limo':
            config.vehicle.vehicle_type = 'Limo'
            config.vehicle.programme_type = 'Ros'
        elif vehicle_type_normalized == 'qcar':
            config.vehicle.vehicle_type = 'Qcar'
            config.vehicle.programme_type = 'Ros'
        else:
            self.get_logger().warning(
                f"Unknown vehicle_type='{vehicle_type}', keeping config value '{config.vehicle.vehicle_type}'")
        config.timing.controller_update_rate = controller_rate
        config.path.path_number = path_number
        
        if log_dir and log_dir.strip():
            config.logging.log_dir = log_dir
        else:
            config.logging.log_dir = os.path.join(qcar_path, 'qcar', 'logs')
        
        if data_log_dir and data_log_dir.strip():
            config.logging.data_log_dir = data_log_dir
        else:
            config.logging.data_log_dir = os.path.join(qcar_path, 'qcar', 'data_logs')
        
        self.calibrate = calibrate
        
        # ===== CREATE VEHICLE LOGIC =====
        self.kill_event = Event()
        self.get_logger().info("Creating VehicleLogic instance...")
        
        self.vehicle_logic = VehicleLogic(config, self.kill_event)
        
        # Replace hardware interfaces with ROS adapters
        self.vehicle_logic.qcar = self.qcar_adapter
        self.vehicle_logic.gps = self.gps_adapter
        self.vehicle_logic.v_ref = v_ref
        
        self._setup_ros_initialization_override()
        self.get_logger().info("VehicleLogic created successfully")
        
        # ===== ROS TIMERS FOR CONTROL LOOPS =====
        self.observer_rate = 120
        self.observer_timer = self.create_timer(1.0/self.observer_rate, self._observer_callback)
        
        control_period = 1.0 / controller_rate
        self.control_timer = self.create_timer(control_period, self._control_callback)
        
        self.last_control_time = time.time()
        self.last_observer_time = time.time()
        self.control_dt = control_period
        
        # ROS topic readiness flags
        self.topics_ready = False
        self.pose_received = False
        self.joint_received = False
        self.path_received = False
        self._optional_path_notice_logged = False
        
        self.init_check_timer = self.create_timer(0.1, self._check_initialization)
        self.tf_update_timer = self.create_timer(0.2, self._update_gps_from_tf)
        
        self.get_logger().info("="*70)
        self.get_logger().info("Full Vehicle Control System Ready! (QCar Coordinate Style)")
        self.get_logger().info("TF tree: SDCQcar -> map -> odom -> base_link")
        self.get_logger().info("="*70)
        
    # ===== ROS CALLBACKS =====
    
    def _odom_callback(self, msg: Odometry):
        """Update motor tach from odometry data"""
        linear_vel = msg.twist.twist.linear.x
        self.qcar_adapter.update_motor_tach(linear_vel)
        
        if not self.pose_received:
            self.pose_received = True
            self.get_logger().info("✓ Odometry topic connected")
    
    def _limo_status_callback(self, msg: LimoStatus):
        """Handle Limo status updates"""
        self.qcar_adapter.update_Limo_status(msg)
        
        if not self.joint_received:
            self.joint_received = True
            self.get_logger().info("✓ Limo status topic connected")
            
            
    def _imu_callback(self, msg: Imu):
        """Update gyroscope data"""
        gyro_z = msg.angular_velocity.z
        accel_x = msg.linear_acceleration.x
        self.qcar_adapter.update_gyro(gyro_z)
        self.qcar_adapter.update_accel(accel_x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        
    def _path_callback(self, msg: Path):
        """Receive path from waypoints_qcar node (in SDCQcar frame)"""
        if len(msg.poses) < 2:
            self.get_logger().warning("Received path with less than 2 waypoints, ignoring")
            return
        
        waypoints_x = []
        waypoints_y = []
        
        for pose_stamped in msg.poses:
            waypoints_x.append(pose_stamped.pose.position.x)
            waypoints_y.append(pose_stamped.pose.position.y)
        
        waypoint_sequence = np.array([waypoints_x, waypoints_y])
        self.vehicle_logic.waypoint_sequence = waypoint_sequence
        
        if not self.path_received:
            self.path_received = True
            self.get_logger().info(f"✓ QCar-style path received: {waypoint_sequence.shape[1]} waypoints")
            self.get_logger().info(f"  Start: ({waypoints_x[0]:.2f}, {waypoints_y[0]:.2f})")
            self.get_logger().info(f"  End: ({waypoints_x[-1]:.2f}, {waypoints_y[-1]:.2f})")
    
    # ===== INITIALIZATION CHECK =====

    def _publish_pending_initial_pose_when_ready(self):
        """Publish queued /initialpose once AMCL subscriber is available."""
        if self.pending_initial_pose_xyz_deg is None:
            return

        if len(self.get_subscriptions_info_by_topic('/initialpose')) == 0:
            if not self.pending_initial_pose_wait_logged:
                self.get_logger().info(
                    "Waiting for AMCL subscriber on /initialpose before publishing initial pose.")
                self.pending_initial_pose_wait_logged = True
            return

        x, y, yaw_deg = self.pending_initial_pose_xyz_deg
        self.gps_adapter.send_initial_pose(x, y, yaw_deg)
        self.get_logger().info(
            f"Published initial pose to AMCL ({self.pending_initial_pose_source}): "
            f"x={x:.3f}, y={y:.3f}, yaw_deg={yaw_deg:.3f}")
        self.pending_initial_pose_xyz_deg = None
        self.pending_initial_pose_source = ""
        self.pending_initial_pose_wait_logged = False

    def _sdc_initial_pose_callback(self, msg: PoseStamped):
        """Convert incoming pose to map frame and publish as AMCL initial pose."""
        source_frame = msg.header.frame_id.strip() if msg.header.frame_id else ""
        if not source_frame:
            source_frame = self.sdc_initialpose_input_frame

        try:
            map_x, map_y, map_yaw_deg = self._convert_pose_to_map(msg, source_frame)
        except Exception as exc:
            self.get_logger().warning(
                f"Failed to convert initial pose from '{source_frame}' to map: {exc}")
            return

        self.pending_initial_pose_xyz_deg = (map_x, map_y, map_yaw_deg)
        self.pending_initial_pose_source = (
            f"converted from {source_frame} via {self.sdc_initialpose_topic}")
        self.pending_initial_pose_wait_logged = False
        self.get_logger().info(
            f"Received initial pose in {source_frame}: "
            f"x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f} "
            f"-> map x={map_x:.3f}, y={map_y:.3f}, yaw_deg={map_yaw_deg:.3f}")
        self._publish_pending_initial_pose_when_ready()

    def _normalize_quaternion(self, qx, qy, qz, qw):
        quat = np.array([qx, qy, qz, qw], dtype=float)
        norm = np.linalg.norm(quat)
        if norm < 1e-9:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        return quat / norm

    def _convert_pose_to_map(self, msg: PoseStamped, source_frame: str):
        source_quat = self._normalize_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        source_rot = R.from_quat(source_quat)
        source_xyz = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)

        if source_frame == 'map':
            map_xyz = source_xyz
            map_rot = source_rot
        else:
            t = self.tf_buffer.lookup_transform('map', source_frame, rclpy.time.Time())
            tf_xyz = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ], dtype=float)
            tf_quat = self._normalize_quaternion(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            )
            tf_rot = R.from_quat(tf_quat)

            map_xyz = tf_rot.apply(source_xyz) + tf_xyz
            map_rot = tf_rot * source_rot

        map_yaw_deg = math.degrees(map_rot.as_euler('xyz')[2])
        return float(map_xyz[0]), float(map_xyz[1]), float(map_yaw_deg)
    
    def _check_initialization(self):
        """Check if ROS topics are connected"""
        if self.topics_ready:
            return

        required_topics_ready = self.pose_received and self.joint_received
        path_gate_ready = self.path_received or (not self.require_path_for_ready)

        if required_topics_ready and not self.path_received and not self.require_path_for_ready:
            if not self._optional_path_notice_logged:
                self.get_logger().info(
                    "Starting without /plan_qcar; waiting for path updates in parallel.")
                self._optional_path_notice_logged = True

        if required_topics_ready and path_gate_ready:
            if not self.topics_ready:
                self.get_logger().info("="*70)
                self.get_logger().info("✓ ALL ROS TOPICS READY - State Machine Can Initialize")
                self.get_logger().info("="*70)
                self.topics_ready = True
                
                self.init_check_timer.cancel()
                
                if hasattr(self.vehicle_logic, '_ros_topics_ready'):
                    self.vehicle_logic._ros_topics_ready = True
    
    def _update_gps_from_tf(self):
        """Update GPS adapter from TF transform (SDCQcar -> base_link)"""
        self.gps_adapter.update_from_tf()
    
    # ===== OBSERVER UPDATE LOOP =====
    
    def _observer_callback(self):
        """High-rate observer update"""
        if not self.topics_ready:
            return
            
        current_time = time.time()
        dt = current_time - self.last_observer_time
        self.last_observer_time = current_time
        
        try:
            self.vehicle_logic._update_sensor_data(dt)
            self.vehicle_logic._observer_update(dt)
        except Exception as e:
            self.get_logger().error(f"Observer update error: {e}")
    
    # ===== CONTROL LOOP =====
    
    def _control_callback(self):
        """Main control loop"""
        current_time = time.time()
        dt = current_time - self.last_control_time
        self.last_control_time = current_time
        
        try:
            # Keep communication alive even while waiting for full ROS readiness.
            self.vehicle_logic._send_telemetry_to_ground_station()
            self.vehicle_logic._broadcast_periodic_status()
            self.vehicle_logic._process_queued_commands()
            self.vehicle_logic._broadcast_v2v_state()

            if self.topics_ready:
                success = self.vehicle_logic._control_logic_update(dt)
                if not success:
                    self.get_logger().warning("Control logic update failed")

            self.vehicle_logic.loop_counter += 1

        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}")
            import traceback
            traceback.print_exc()
        
    def _setup_ros_initialization_override(self):
        """Setup ROS-specific initialization"""
        self.vehicle_logic._ros_mode = True
        self.vehicle_logic._ros_topics_ready = False
        self.get_logger().info("✓ ROS-specific initialization mode enabled")
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down VehicleControlFullSystemQCar...")
        self.kill_event.set()
        
        if hasattr(self, 'vehicle_logic'):
            self.vehicle_logic._shutdown()
            
        super().destroy_node()


# ===== MAIN ENTRY POINT =====
def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VehicleControlFullSystemQCar()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutdown requested (Ctrl+C)")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
