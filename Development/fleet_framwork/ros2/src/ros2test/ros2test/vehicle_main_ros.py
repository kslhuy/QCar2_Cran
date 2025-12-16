"""
ROS 2 Version of vehicle_main.py (Full Vehicle Control System)

This is the ROS 2 adaptation of the standalone vehicle_main.py, providing:
- VehicleLogic integration via thin wrapper pattern
- Hardware abstraction using ROS adapters (replaces QCar/GPS hardware)
- Original TCP/UDP communication preserved (Ground Station, V2V)
- ROS-specific initialization state (bypasses hardware init)
- All command-line arguments migrated to ROS parameters
"""

import sys
import os
import time
import numpy as np
from threading import Event

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from qcar2_interfaces.msg import MotorCommands
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# ===== ADD PATH TO QCAR FOLDER =====
# Navigate up from ros2test/ros2test/ to Development/multi_vehicle_self_driving_RealQcar
current_dir = os.path.dirname(os.path.abspath(__file__))
qcar_path = os.path.abspath(os.path.join(current_dir, "../../../../../multi_vehicle_self_driving_RealQcar"))

if qcar_path not in sys.path:
    sys.path.insert(0, qcar_path)
    print(f"[PATH] Added to sys.path: {qcar_path}")

# Now import from qcar folder
from qcar.config_main import VehicleMainConfig
from qcar.vehicle_logic import VehicleLogic
from qcar.command_types import CommandType

# ===== ROS ADAPTER FOR QCAR HARDWARE =====
class ROSQCarAdapter:
    """Adapter to replace QCar hardware interface with ROS topics"""
    
    def __init__(self, ros_node):
        self.ros_node = ros_node
        
        # Cached sensor data from ROS subscriptions
        self._motor_tach = 0.0
        self._gyro_z = 0.0
        self._last_update = time.time()
        
        # Motor command publisher
        self.motor_pub = ros_node.motor_pub
        
    def read(self):
        """Mimic QCar.read() interface - returns sensor array"""
        # QCar.read() returns: [motor_tach, encoder, gyro_x, gyro_y, gyro_z, ...]
        # We'll return a simplified version with what we have from ROS
        return [
            self._motor_tach,  # index 0: motor tachometer
            0.0,               # index 1: encoder (not used)
            0.0,               # index 2: gyro_x
            0.0,               # index 3: gyro_y
            self._gyro_z,      # index 4: gyro_z
        ]
    
    def write(self, throttle=0.0, steering=0.0):
        """Mimic QCar.write() interface - publish to ROS"""
        msg = MotorCommands()
        msg.motor_names = ["steering_angle", "motor_throttle"]
        msg.values = [float(steering), float(throttle)]
        self.motor_pub.publish(msg)
        
    def update_motor_tach(self, value):
        """Update motor tachometer from ROS joint callback"""
        self._motor_tach = value
        self._last_update = time.time()
        
    def update_gyro(self, gyro_z):
        """Update gyroscope from ROS IMU callback"""
        self._gyro_z = gyro_z


# ===== ROS GPS ADAPTER =====
class ROSGPSAdapter:
    """Adapter to replace QCarGPS with ROS pose data"""
    
    def __init__(self):
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_update = time.time()
        
    def read(self):
        """Mimic QCarGPS.read() interface"""
        # Returns: [x, y, z, ...]
        return [self._x, self._y, 0.0]
    
    def update_pose(self, x, y, yaw):
        """Update pose from ROS EKF callback"""
        self._x = x
        self._y = y
        self._yaw = yaw
        self._last_update = time.time()


# ===== MAIN ROS NODE =====
class VehicleControlFullSystem(Node):
    """ROS 2 wrapper for complete VehicleLogic system"""
    
    def __init__(self):
        super().__init__('vehicle_control_full_system')
        
        self.get_logger().info("="*70)
        self.get_logger().info("Initializing Full Vehicle Control System (ROS 2 Wrapper)")
        self.get_logger().info("="*70)
        
        # ===== ROS PARAMETERS =====
        # Migrated from vehicle_main.py command-line arguments
        self.declare_parameters(
            namespace='',
            parameters=[
                ('car_id', 0),
                ('v_ref', 0.75),
                ('controller_rate', 100),  # 100 Hz for simulation
                ('calibrate', False),  # Recalibrate vehicle before starting
                ('path_number', 0),  # Node configuration (0, 1, 2)
                ('no_steering', False),  # Disable steering control
                ('config_file', ''),  # Custom config file path (empty = use default)
                ('log_dir', ''),  # Custom log directory (empty = use config default)
                ('data_log_dir', ''),  # Custom data log directory (empty = use config default)
                ('ENCODER_COUNTS_PER_REV', 720.0),
        car_id = self.get_parameter('car_id').value
        v_ref = self.get_parameter('v_ref').value
        controller_rate = self.get_parameter('controller_rate').value
        calibrate = self.get_parameter('calibrate').value
        path_number = self.get_parameter('path_number').value
        no_steering = self.get_parameter('no_steering').value
        config_file = self.get_parameter('config_file').value
        log_dir = self.get_parameter('log_dir').value
        data_log_dir = self.get_parameter('data_log_dir').value
        
        self.ENCODER_COUNTS_PER_REV = self.get_parameter('ENCODER_COUNTS_PER_REV').value
        car_id = self.get_parameter('car_id').value
        v_ref = self.get_parameter('v_ref').value
        controller_rate = self.get_parameter('controller_rate').value
        
        self.ENCODER_COUNTS_PER_REV = self.get_parameter('ENCODER_COUNTS_PER_REV').value
        self.WHEEL_RADIUS = self.get_parameter('WHEEL_RADIUS').value
        self.PIN_TO_SPUR_RATIO = self.get_parameter('PIN_TO_SPUR_RATIO').value
        
        # Motor speed unit conversion
        self.CPS_TO_MPS = (1/(self.ENCODER_COUNTS_PER_REV*4) 
            * self.PIN_TO_SPUR_RATIO * 2*np.pi * self.WHEEL_RADIUS)
        
        self.get_logger().info(f"Car ID: {car_id}, v_ref: {v_ref}, rate: {controller_rate} Hz")
        
        # ===== ROS PUBLISHERS =====
        self.motor_pub = self.create_publisher(MotorCommands, "/qcar2_motor_speed_cmd", 10)
        # Note: Ground Station telemetry uses TCP (original), not ROS topics
        # Note: V2V uses UDP multicast (original), not ROS DDS
        
        # ===== ROS SUBSCRIPTIONS =====
        self.pose_sub = self.create_subscription(
            PoseStamped, '/ekf_pose', self._pose_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/qcar2_joint', self._joint_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_callback, 10
        )
        self.command_sub = self.create_subscription(
            String, f'/vehicle_{car_id}/commands', self._command_callback, 10
        )
        # Note: V2V uses original UDP multicast, not ROS subscriptions
        
        # ===== LOAD CONFIGURATION =====
        self.get_logger().info("Loading VehicleMainConfig...")
        
        # Use custom config file if specified, otherwise default
        if config_file and config_file.strip():
            config_path = config_file
        else:
            config_path = os.path.join(qcar_path, 'qcar', 'config_vehicle_main.yaml')
        
        if os.path.exists(config_path):
            if config_path.endswith('.json'):
                config = VehicleMainConfig.from_json(config_path)
            elif config_path.endswith('.yaml') or config_path.endswith('.yml'):
                config = VehicleMainConfig.from_yaml(config_path)
            else:
                self.get_logger().error("Config file must be .json or .yaml")
                raise ValueError(f"Invalid config file format: {config_path}")
            self.get_logger().info(f"Loaded config from: {config_path}")
        else:
            if config_file and config_file.strip():
                self.get_logger().error(f"Config file not found: {config_path}")
                raise FileNotFoundError(f"Config file not found: {config_path}")
            else:
                config = VehicleMainConfig()
        # Override config with ROS parameters (migrated from vehicle_main.py args)
        config.network.car_id = car_id
        config.timing.controller_update_rate = controller_rate
        config.path_planning.path_number = path_number
        config.control.no_steering = no_steering
        
        # Override log directories if specified (use absolute paths for ROS)
        if log_dir and log_dir.strip():
            config.logging.log_dir = log_dir
            self.get_logger().info(f"Using custom log directory: {log_dir}")
        else:
            # Use absolute path to qcar/logs by default
            config.logging.log_dir = os.path.join(qcar_path, 'qcar', 'logs')
            self.get_logger().info(f"Using default log directory: {config.logging.log_dir}")
        
        if data_log_dir and data_log_dir.strip():
            config.logging.data_log_dir = data_log_dir
            self.get_logger().info(f"Using custom data log directory: {data_log_dir}")
        else:
            # Use absolute path to qcar/data_logs by default
            config.logging.data_log_dir = os.path.join(qcar_path, 'qcar', 'data_logs')
            self.get_logger().info(f"Using default data log directory: {config.logging.data_log_dir}")
        
        # Store calibration flag for potential use
        self.calibrate = calibrate
        if calibrate:
            self.get_logger().warning("Calibration requested but not yet implemented in ROS mode")
        if calibrate:
            self.get_logger().warning("Calibration requested but not yet implemented in ROS mode")
        
        # ===== CREATE VEHICLE LOGIC =====
        self.kill_event = Event()
        self.get_logger().info("Creating VehicleLogic instance...")
        
        self.vehicle_logic = VehicleLogic(config, self.kill_event)
        
        # **CRITICAL**: Replace hardware interfaces with ROS adapters
        self.vehicle_logic.qcar = self.qcar_adapter
        self.vehicle_logic.gps = self.gps_adapter
        
        # Set reference velocity
        self.vehicle_logic.v_ref = v_ref
        
        # **ORIGINAL TCP/UDP COMMUNICATION PRESERVED**
        # VehicleLogic.__init__() already created these components:
        # 1. self.v2v_manager (line 69-77 in vehicle_logic.py) - UDP multicast
        # 2. self.client_Ground_Station (line 93 in vehicle_logic.py) - TCP client
        # These will start automatically during state machine initialization
        if hasattr(self.vehicle_logic, 'v2v_manager'):
            self.get_logger().info(f"✓ V2V Manager ready (UDP port {self.vehicle_logic.v2v_manager.base_port})")
        if hasattr(self.vehicle_logic, 'client_Ground_Station'):
            self.get_logger().info(f"✓ Ground Station client ready (TCP {config.network.host_ip}:{config.network.port})")
        
        # **OVERRIDE INITIALIZATION STATE** for ROS
        # Replace hardware initialization with ROS-aware version
        self._setup_ros_initialization_override()
        
        self.get_logger().info("VehicleLogic created successfully")
        
        # ===== ROS TIMERS FOR CONTROL LOOPS =====
        # Observer update timer (200 Hz for high-rate state estimation)
        self.observer_rate = 200  # Hz
        self.observer_timer = self.create_timer(1.0/self.observer_rate, self._observer_callback)
        
        # Main control timer (100 Hz)
        control_period = 1.0 / controller_rate
        self.control_timer = self.create_timer(control_period, self._control_callback)
        
        # Note: Telemetry and V2V handled by original TCP/UDP threads in VehicleLogic
        # No ROS timers needed for these
        
        # Timing tracking
        self.last_control_time = time.time()
        self.last_observer_time = time.time()
        self.control_dt = control_period
        
        # ROS topic readiness flags for state machine initialization
        self.topics_ready = False
        self.pose_received = False
        self.joint_received = False
        
        # Initialization check timer (runs until topics are ready)
        self.init_check_timer = self.create_timer(0.1, self._check_initialization)
        
        self.get_logger().info("="*70)
        self.get_logger().info("Full Vehicle Control System Ready!")
        self.get_logger().info("="*70)
        
    # ===== ROS CALLBACKS =====
    
    def _pose_callback(self, msg: PoseStamped):
        """Update GPS adapter with pose data"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Convert quaternion to yaw
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # Simple quaternion to yaw conversion
        yaw = np.arctan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        
        self.gps_adapter.update_pose(x, y, yaw)
        
        # Mark pose as received for initialization check
        if not self.pose_received:
            self.pose_received = True
            self.get_logger().info("✓ Pose topic connected")
        
    def _joint_callback(self, msg: JointState):
        """Update motor tachometer"""
        if len(msg.velocity) > 0:
            motor_tach_mps = msg.velocity[0] * self.CPS_TO_MPS
            self.qcar_adapter.update_motor_tach(motor_tach_mps)
            
            # Mark joint as received for initialization check
            if not self.joint_received:
                self.joint_received = True
                self.get_logger().info("✓ Joint topic connected")
            
    def _imu_callback(self, msg: Imu):
        """Update gyroscope data"""
        gyro_z = msg.angular_velocity.z
        self.qcar_adapter.update_gyro(gyro_z)
        
    def _command_callback(self, msg: String):
        """Handle commands from Ground Station or remote controller"""
        try:
            # Parse command (expecting format: "COMMAND_TYPE:data_json")
            parts = msg.data.split(':', 1)
            command_name = parts[0]
            data_json = parts[1] if len(parts) > 1 else "{}"
            
            # Convert to CommandType enum
            command_type = CommandType[command_name]
            
            # Queue command to VehicleLogic's CommandHandler
            import json
            data = json.loads(data_json)
            self.vehicle_logic.command_handler.queue_command(command_type, data)
            
            self.get_logger().info(f"Received command: {command_name}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
    
    # ===== INITIALIZATION CHECK =====
    
    def _check_initialization(self):
        """Check if ROS topics are connected before allowing state machine to proceed"""
        if self.topics_ready:
            return  # Already initialized
        
        # Check if we've received at least one message from essential topics
        if self.pose_received and self.joint_received:
            if not self.topics_ready:
                self.get_logger().info("="*70)
                self.get_logger().info("✓ ALL ROS TOPICS READY - State Machine Can Initialize")
                self.get_logger().info("="*70)
                self.topics_ready = True
                
                # Cancel this timer
                self.init_check_timer.cancel()
                
                # Signal to ROS InitializingState that topics are ready
                if hasattr(self.vehicle_logic, '_ros_topics_ready'):
                    self.vehicle_logic._ros_topics_ready = True
                
                # Allow VehicleLogic state machine to proceed from INITIALIZING state
        else:
            # Still waiting
            if not self.pose_received:
                pose_pub_count = len([p for p in self.get_publishers_info_by_topic('/ekf_pose')])
                self.get_logger().info(f"Waiting for /ekf_pose... (publishers: {pose_pub_count})")
            if not self.joint_received:
                joint_pub_count = len([p for p in self.get_publishers_info_by_topic('/qcar2_joint')])
                self.get_logger().info(f"Waiting for /qcar2_joint... (publishers: {joint_pub_count})")
    
    # ===== OBSERVER UPDATE LOOP (200 Hz) =====
    
    def _observer_callback(self):
        """High-rate observer update (200 Hz) - runs independently of control loop"""
        if not self.topics_ready:
            return  # Don't run observer until topics are ready
            
        current_time = time.time()
        dt = current_time - self.last_observer_time
        self.last_observer_time = current_time
        
        try:
            # Update sensor data from cached ROS data
            self.vehicle_logic._update_sensor_data(dt)
            
            # Run observer update at 200 Hz
            self.vehicle_logic._observer_update(dt)
            
        except Exception as e:
            self.get_logger().error(f"Observer update error: {e}")
    
    # ===== CONTROL LOOP =====
    
    def _control_callback(self):
        """Main control loop (100 Hz) - delegates to VehicleLogic"""
        if not self.topics_ready:
            return  # Don't run control until topics are ready
            
        current_time = time.time()
        dt = current_time - self.last_control_time
        self.last_control_time = current_time
        
        try:
            # Observer runs separately at 200 Hz, so we skip it here
            # Just run control logic (state machine update)
            success = self.vehicle_logic._control_logic_update(dt)
            
            if not success:
                self.get_logger().warning("Control logic update failed")
                    
        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}")
            import traceback
            traceback.print_exc()
    
    # Note: Telemetry and V2V handled by VehicleLogic's original TCP/UDP threads
    
    def _setup_ros_initialization_override(self):
        """Setup ROS-specific initialization that bypasses hardware init"""
        # Replace the InitializingState with ROS-specific version
        try:
            # Import ROS-specific initialization state
            import sys
            import os
            state_machine_path = os.path.join(qcar_path, 'qcar', 'StateMachine')
            if state_machine_path not in sys.path:
                sys.path.insert(0, state_machine_path)
            
            from ros_initializing_state import ROSInitializingState
            
            # Replace the state in the state machine
    def destroy_node(self):
        """Clean shutdown (migrated from vehicle_main.py)"""  
        self.get_logger().info("="*70)
        self.get_logger().info("Shutting down VehicleControlFullSystem...")
        self.get_logger().info("="*70)
        
        self.kill_event.set()
        
        # Shutdown VehicleLogic
        if hasattr(self, 'vehicle_logic'):
            try:
                self.vehicle_logic._shutdown()
                self.get_logger().info("✓ VehicleLogic shutdown complete")
            except Exception as e:
                self.get_logger().error(f"Error during VehicleLogic shutdown: {e}")
        
        # Stop QUARC models (for physical QCar) or QLabs (for simulation)
        try:
            from pal.products.qcar import IS_PHYSICAL_QCAR
            
            if IS_PHYSICAL_QCAR:
                self.get_logger().info("Physical QCar detected - QUARC cleanup skipped (handled by VehicleLogic)")
                # QUARC model stop would go here if needed
            else:
                self.get_logger().info("Simulation mode - terminating QLabs models...")
                from qvl.real_time import QLabsRealTime
                QLabsRealTime().terminate_all_real_time_models()
                time.sleep(0.5)
                self.get_logger().info("✓ QLabs real-time models terminated")
                
        except ImportError:
            self.get_logger().warning("QLabs/QUARC modules not available - skipping cleanup")
        except Exception as e:
            self.get_logger().warning(f"Error during hardware cleanup: {e}")
        
        self.get_logger().info("="*70)
        self.get_logger().info("Shutdown complete")
        self.get_logger().info("="*70)
        
        super().destroy_node()le_logic.vehicle_logger
                )
                
                self.get_logger().info("✓ ROS-specific initialization state installed")
            
            # Mark ROS mode
            self.vehicle_logic._ros_mode = True
            self.vehicle_logic._ros_topics_ready = False  # Will be set by _check_initialization
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup ROS initialization override: {e}")
            import traceback
            traceback.print_exc()
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down VehicleControlFullSystem...")
        self.kill_event.set()
        
        if hasattr(self, 'vehicle_logic'):
            self.vehicle_logic._shutdown()
            
        super().destroy_node()


# ===== MAIN ENTRY POINT =====
def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VehicleControlFullSystem()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutdown requested (Ctrl+C)")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
