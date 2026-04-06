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
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as NavPath
from limo_msgs.msg import LimoStatus
from std_msgs.msg import String, Float32MultiArray
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
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
                ('enable_sdc_map_tf_broadcaster', True),
                ('sdc_map_update_topic', '/sdc_map_tf_update'),
                ('sdc_map_tf_publish_rate', 20.0),
                ('sdc_map_x', -0.1),
                ('sdc_map_y', 0.0),
                ('sdc_map_z', 0.0),
                ('sdc_map_yaw', 1.3701),
                ('sdc_map_pitch', 0.0),
                ('sdc_map_roll', 0.0),
                ('enable_external_path_subscriber', False),
                ('external_path_topic', '/plan_qcar'),
                ('publish_internal_path_for_rviz', True),
                ('internal_path_topic', '/plan_qcar'),
                ('internal_path_publish_rate', 2.0),
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
        enable_sdc_map_tf_broadcaster = self.get_parameter(
            'enable_sdc_map_tf_broadcaster').value
        sdc_map_update_topic = self.get_parameter(
            'sdc_map_update_topic').value
        sdc_map_tf_publish_rate = self.get_parameter(
            'sdc_map_tf_publish_rate').value
        sdc_map_x = self.get_parameter('sdc_map_x').value
        sdc_map_y = self.get_parameter('sdc_map_y').value
        sdc_map_z = self.get_parameter('sdc_map_z').value
        sdc_map_yaw = self.get_parameter('sdc_map_yaw').value
        sdc_map_pitch = self.get_parameter('sdc_map_pitch').value
        sdc_map_roll = self.get_parameter('sdc_map_roll').value
        enable_external_path_subscriber = self.get_parameter(
            'enable_external_path_subscriber').value
        external_path_topic = self.get_parameter('external_path_topic').value
        publish_internal_path_for_rviz = self.get_parameter(
            'publish_internal_path_for_rviz').value
        internal_path_topic = self.get_parameter('internal_path_topic').value
        internal_path_publish_rate = self.get_parameter(
            'internal_path_publish_rate').value
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

        self.enable_sdc_map_tf_broadcaster = bool(enable_sdc_map_tf_broadcaster)
        self.sdc_map_update_topic = str(sdc_map_update_topic).strip() or '/sdc_map_tf_update'
        self.sdc_map_tf_publish_rate = max(float(sdc_map_tf_publish_rate), 1.0)
        self.sdc_map_tf_parent_frame = 'SDCQcar'
        self.sdc_map_tf_child_frame = 'map'
        self.sdc_map_tf_translation = [float(sdc_map_x), float(sdc_map_y), float(sdc_map_z)]
        self.sdc_map_tf_rpy = [float(sdc_map_roll), float(sdc_map_pitch), float(sdc_map_yaw)]
        self.sdc_map_tf_broadcaster = None
        self.sdc_map_tf_timer = None

        if self.enable_sdc_map_tf_broadcaster:
            self.sdc_map_tf_broadcaster = TransformBroadcaster(self)
            self.sdc_map_tf_timer = self.create_timer(
                1.0 / self.sdc_map_tf_publish_rate, self._publish_sdc_map_tf
            )
            self.get_logger().info(
                "Runtime SDCQcar->map TF broadcaster enabled "
                f"at {self.sdc_map_tf_publish_rate:.1f} Hz"
            )
        else:
            self.get_logger().info(
                "Runtime SDCQcar->map TF broadcaster disabled "
                "(expected when launch static TF fallback is used)"
            )

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
        
        # Subscribe to new ROS 2 YOLO Detections instead of ZMQ
        self.yolo_sub = self.create_subscription(
            Float32MultiArray, '/limo/yolo_detections', self._yolo_callback, 10
        )
        self.enable_external_path_subscriber = bool(enable_external_path_subscriber)
        self.external_path_topic = str(external_path_topic).strip() or '/plan_qcar'
        self.publish_internal_path_for_rviz = bool(publish_internal_path_for_rviz)
        self.internal_path_topic = str(internal_path_topic).strip() or '/plan_qcar'
        self.internal_path_publish_rate = max(float(internal_path_publish_rate), 0.5)
        self.internal_path_pub = self.create_publisher(
            NavPath, self.internal_path_topic, 10
        )
        self.internal_vis_path_pub = self.create_publisher(
            MarkerArray, "/waypoints_viz_qcar", 10
        )
        self.internal_path_timer = None
        self._last_internal_path_point_count = -1

        if (
            self.enable_external_path_subscriber
            and self.publish_internal_path_for_rviz
            and self.internal_path_topic == self.external_path_topic
        ):
            self.get_logger().warning(
                "Disabled internal path publishing because internal_path_topic "
                "matches external_path_topic while external subscriber is enabled"
            )
            self.publish_internal_path_for_rviz = False

        if self.publish_internal_path_for_rviz:
            self.internal_path_timer = self.create_timer(
                1.0 / self.internal_path_publish_rate,
                self._publish_internal_path_for_rviz,
            )
            self.get_logger().info(
                "Internal path publishing enabled for RViz on "
                f"{self.internal_path_topic} at {self.internal_path_publish_rate:.1f} Hz"
            )
        else:
            self.get_logger().info("Internal path publishing for RViz is disabled")

        if self.enable_external_path_subscriber:
            self.path_sub = self.create_subscription(
                NavPath, self.external_path_topic, self._path_callback, 10
            )
            self.get_logger().info(
                f"External path subscriber enabled on {self.external_path_topic}"
            )
        else:
            self.path_sub = None
            self.path_received = True
            self.get_logger().info(
                "External path subscriber disabled. Using internal path generation "
                "(InitializingState + SET_PATH commands)."
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

        self.sdc_map_tf_update_sub = self.create_subscription(
            PoseStamped,
            self.sdc_map_update_topic,
            self._sdc_map_tf_update_callback,
            10,
        )
        self.get_logger().info(
            "Runtime SDCQcar->map TF update topic enabled "
            f"on {self.sdc_map_update_topic}"
        )
        
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
        self.vehicle_logic.sdc_map_tf_update_callback = self._handle_sdc_map_tf_command
        
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
        self.path_received = not self.enable_external_path_subscriber
        self._optional_path_notice_logged = False
        
        self.init_check_timer = self.create_timer(0.1, self._check_initialization)
        self.tf_update_timer = self.create_timer(0.2, self._update_gps_from_tf)
        
        self.get_logger().info("="*70)
        self.get_logger().info("Full Vehicle Control System Ready! (QCar Coordinate Style)")
        self.get_logger().info("TF tree: SDCQcar -> map -> odom -> base_link")
        self.get_logger().info("="*70)

    def _publish_sdc_map_tf(self):
        """Publish runtime SDCQcar->map transform from the current calibration state."""
        if not self.enable_sdc_map_tf_broadcaster or self.sdc_map_tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(self.sdc_map_tf_parent_frame)
        transform.child_frame_id = str(self.sdc_map_tf_child_frame)
        transform.transform.translation.x = float(self.sdc_map_tf_translation[0] or 0.0)
        transform.transform.translation.y = float(self.sdc_map_tf_translation[1] or 0.0)
        transform.transform.translation.z = float(self.sdc_map_tf_translation[2] or 0.0)

        roll, pitch, yaw = [float(val) for val in self.sdc_map_tf_rpy]
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])

        if self.sdc_map_tf_broadcaster:
            self.sdc_map_tf_broadcaster.sendTransform(transform)

    def _is_sdc_map_tf_update_safe(self, force: bool = False) -> bool:
        """Only allow live frame recalibration in low-risk operating conditions."""
        if force:
            return True

        current_state = None
        if hasattr(self, 'vehicle_logic') and hasattr(self.vehicle_logic, 'state_machine'):
            current_state = getattr(self.vehicle_logic.state_machine, 'current_state_id', None)

        state_name = getattr(current_state, 'name', str(current_state)) if current_state is not None else ''
        if state_name in {'STOPPED', 'WAITING_FOR_START', 'INITIALIZING'}:
            return True

        speed_mps = abs(float(getattr(self.qcar_adapter, 'motorTach', 0.0)))
        if speed_mps <= 0.03:
            return True

        self.get_logger().warning(
            "Rejected runtime SDCQcar->map TF update while moving. "
            f"state={state_name}, speed={speed_mps:.3f} m/s"
        )
        return False

    def _apply_sdc_map_tf_update(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        pitch: float = 0.0,
        roll: float = 0.0,
        source: str = 'runtime update',
        force: bool = False,
    ) -> bool:
        """Apply a new SDCQcar->map transform used by the runtime broadcaster."""
        if not self.enable_sdc_map_tf_broadcaster:
            self.get_logger().warning(
                "Runtime TF update ignored because enable_sdc_map_tf_broadcaster is false"
            )
            return False

        if not self._is_sdc_map_tf_update_safe(force=force):
            return False

        self.sdc_map_tf_translation = [float(x), float(y), float(z)]
        self.sdc_map_tf_rpy = [float(roll), float(pitch), float(yaw)]
        self._publish_sdc_map_tf()

        self.get_logger().info(
            "Updated SDCQcar->map TF from "
            f"{source}: x={x:.4f}, y={y:.4f}, z={z:.4f}, "
            f"yaw={yaw:.4f}, pitch={pitch:.4f}, roll={roll:.4f}"
        )
        return True

    def _sdc_map_tf_update_callback(self, msg: PoseStamped):
        """Receive runtime SDCQcar->map transform updates from Ground Station."""
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]

        try:
            roll, pitch, yaw = R.from_quat(q).as_euler('xyz')
        except Exception as exc:
            self.get_logger().warning(f"Invalid quaternion on {self.sdc_map_update_topic}: {exc}")
            return

        self._apply_sdc_map_tf_update(
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            z=float(msg.pose.position.z),
            yaw=float(yaw),
            pitch=float(pitch),
            roll=float(roll),
            source=f"topic {self.sdc_map_update_topic}",
            force=False,
        )

    def _handle_sdc_map_tf_command(self, params: dict) -> bool:
        """Apply command-based runtime SDCQcar->map transform updates (SET_PARAMS)."""
        if not isinstance(params, dict):
            self.get_logger().warning("Invalid sdc_map_tf params payload (expected dict)")
            return False

        x = float(params.get('x', self.sdc_map_tf_translation[0]))
        y = float(params.get('y', self.sdc_map_tf_translation[1]))
        z = float(params.get('z', self.sdc_map_tf_translation[2]))

        if 'yaw' in params:
            yaw = float(params.get('yaw'))
        elif 'yaw_deg' in params:
            yaw = math.radians(float(params.get('yaw_deg')))
        else:
            yaw = float(self.sdc_map_tf_rpy[2])

        if 'pitch' in params:
            pitch = float(params.get('pitch'))
        elif 'pitch_deg' in params:
            pitch = math.radians(float(params.get('pitch_deg')))
        else:
            pitch = float(self.sdc_map_tf_rpy[1])

        if 'roll' in params:
            roll = float(params.get('roll'))
        elif 'roll_deg' in params:
            roll = math.radians(float(params.get('roll_deg')))
        else:
            roll = float(self.sdc_map_tf_rpy[0])

        force = bool(params.get('force', False))

        return self._apply_sdc_map_tf_update(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            pitch=pitch,
            roll=roll,
            source='SET_PARAMS category=sdc_map_tf',
            force=force,
        )
        
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

    def _yolo_callback(self, msg: Float32MultiArray):
        """Handle native ROS 2 YOLO detection array updates"""
        try:
            # Reconstruct the 6x7 numpy array from the 42-element Float32MultiArray
            packet = np.array(msg.data, dtype=np.float64).reshape((6, 7))

            # Inject it straight into the YOLOManager's cached data, bypassing ZMQ!
            if hasattr(self, 'vehicle_logic') and self.vehicle_logic.yolo is not None:
                # Get the cached data struct
                data = self.vehicle_logic.yolo._cached_data

                # Unpack Arrays
                data.stop_sign[:] = packet[0, :]
                data.traffic_light[:] = packet[1, :]
                data.cars[:] = packet[2, :]
                data.yield_sign[:] = packet[3, :]
                data.person[:] = packet[4, :]
                data.timestamp = time.time()
                data.is_valid = True

                # Check for velocity gain from YOLODriveLogic automatically
                if self.vehicle_logic.yolo.yolo_drive is not None:
                    data.yolo_gain = self.vehicle_logic.yolo.yolo_drive.check_yolo(
                        data.stop_sign, data.traffic_light, data.cars,
                        data.yield_sign, data.person
                    )
        except Exception as e:
            self.get_logger().error(f"Failed to parse YOLO detection: {e}")

    def _path_callback(self, msg: NavPath):
        """Receive external path updates (expected in SDCQcar frame)."""
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

    def _publish_internal_path_for_rviz(self):
        """Publish internally generated waypoint_sequence for RViz visualization."""
        if not self.publish_internal_path_for_rviz:
            return

        if not hasattr(self, 'vehicle_logic'):
            return

        waypoint_sequence = getattr(self.vehicle_logic, 'waypoint_sequence', None)
        if waypoint_sequence is None:
            return

        if not isinstance(waypoint_sequence, np.ndarray):
            return

        if waypoint_sequence.ndim != 2 or waypoint_sequence.shape[0] < 2:
            return

        path_msg = NavPath()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'SDCQcar'

        marker_array = MarkerArray()

        x_values = waypoint_sequence[0]
        y_values = waypoint_sequence[1]
        point_count = min(len(x_values), len(y_values))
        if point_count < 2:
            return

        for i in range(point_count):
            x_final = float(x_values[i])
            y_final = float(y_values[i])

            # --- Robot Path ---
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x_final
            pose.pose.position.y = y_final
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

            # --- Visual Markers ---
            marker = Marker()
            marker.header.frame_id = "SDCQcar"
            marker.type = Marker.SPHERE
            marker.id = i
            marker.action = Marker.ADD
            marker.pose.position.x = x_final
            marker.pose.position.y = y_final
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.lifetime = Duration(sec=1, nanosec=0)
            marker_array.markers.append(marker)

        self.internal_path_pub.publish(path_msg)
        self.internal_vis_path_pub.publish(marker_array)
        if point_count != self._last_internal_path_point_count:
            self._last_internal_path_point_count = point_count
            self.get_logger().info(
                f"Published internal path to RViz: {point_count} waypoints on {self.internal_path_topic}"
            )
    
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
