#!/usr/bin/env python3
"""
YOLO Server ROS 2 - Native ROS 2 Node for Physical Limo QCar

This node:
1. Subscribes to /camera/color/image_raw and /camera/depth/image_raw (Astra Camera)
2. Runs YOLOv8 tracking and 3D depth extraction via YOLOv8Wrapper_Huy
3. Publishes a Float32MultiArray to /limo/yolo_detections for VehicleLogic
4. (Optional) Publishes a ZMQ video stream for Ground Station probing
"""

import os
import sys
import time
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import message_filters
from cv_bridge import CvBridge

current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from limo_yolo import LimoYOLO, DetectionBuffers, detect_line_and_draw
from YoLo import YOLOVideoPublisher, YOLOPublisher
import cv2

try:
    from LaneFollow import create_lane_detector
    from LaneFollow.lane_detection_interface import LaneDetectionResult
    LANE_MODULE_AVAILABLE = True
except ImportError as e:
    print(f"[SERVER] WARNING: LaneFollow module not available: {e}")
    LANE_MODULE_AVAILABLE = False


class YoloRos2Server(Node):
    def __init__(self):
        super().__init__('yolo_ros2_server')

        # ROS Parameters
        self.declare_parameter('car_id', 0)
        self.declare_parameter('probing', False)
        self.declare_parameter('enable_inference', True)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('video_port', -1) # Default: auto-compute 18760 + car_id
        self.declare_parameter('yolo_port', 18666)
        self.declare_parameter('camera_rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('yolo_publish_topic', '/limo/yolo_detections')


        # Load parameters
        self.car_id = self.get_parameter('car_id').value
        self.probing = self.get_parameter('probing').value
        self.enable_inference = self.get_parameter('enable_inference').value
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        configured_video_port = int(self.get_parameter('video_port').value)
        self.video_port = (
            18760 + int(self.car_id) if configured_video_port < 0 else configured_video_port
        )
        self.rgb_topic = self.get_parameter('camera_rgb_topic').value
        self.depth_topic = self.get_parameter('camera_depth_topic').value
        self.yolo_port = self.get_parameter('yolo_port').value

        self.bridge = CvBridge()
        self.buffers = DetectionBuffers()
        self._zmq_frame_count = 0

        # Match Astra camera QoS (BEST_EFFORT)
        self._sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Separate callback groups so ZMQ streaming doesn't block on YOLO inference
        self._stream_cb_group = MutuallyExclusiveCallbackGroup()
        self._yolo_cb_group = MutuallyExclusiveCallbackGroup()

        self.get_logger().info("="*70)
        self.get_logger().info(f"Starting YOLOv8 ROS 2 Server (Car ID: {self.car_id})")
        self.get_logger().info(
            f"Config: probing={self.probing}, video_port={self.video_port}, yolo_port={self.yolo_port}"
        )
        
        # Initialize ZMQ Video Publisher for Ground Station
        self.video_pub = None
        if self.probing:
            try:
                self.video_pub = YOLOVideoPublisher(ip="*", port=str(self.video_port))
                self.get_logger().info(f"ZMQ Video probing enabled on port {self.video_port}")
            except Exception as e:
                self.get_logger().error(f"Failed to start ZMQ video publisher: {e}")
        else:
            self.get_logger().info(
                "ZMQ Video probing disabled. multi_probing.py will wait for frames "
                f"on tcp://<LIMO_IP>:{self.video_port} until the server is started with probing:=True."
            )

        if self.enable_inference:
            # Initialize YOLO
            self.get_logger().info("Initializing YOLO model...")
            self.yolo = LimoYOLO(imageHeight=self.img_height, imageWidth=self.img_width)
            
            # Initialize ZMQ Publisher for detections
            self.get_logger().info(f"YOLO ZMQ Publisher binding to port {self.yolo_port}")
            self.yolo_pub = YOLOPublisher(port=str(self.yolo_port))
            
            # Synchronize RGB and Depth image topics
            self.get_logger().info(f"Subscribing to {self.rgb_topic} and {self.depth_topic} (Inference Enabled)")
            self.color_sub = message_filters.Subscriber(self, Image, self.rgb_topic,
                                                        qos_profile=self._sensor_qos)
            self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic,
                                                        qos_profile=self._sensor_qos)

            # Allow up to 0.1s difference between RGB and Depth frames
            self.ts = message_filters.ApproximateTimeSynchronizer(
                [self.color_sub, self.depth_sub], queue_size=5, slop=0.1)
            self.ts.registerCallback(self.image_callback)

            # Initialize Lane Detector
            self.lane_enabled = False
            self.lane_detector = None
            if LANE_MODULE_AVAILABLE:
                try:
                    self.lane_detector = create_lane_detector("ultrafast")
                    if self.lane_detector and self.lane_detector.is_initialized:
                        self.yolo.set_lane_detector(self.lane_detector)
                        self.lane_enabled = True
                        self.get_logger().info("UltraFast Lane detector initialized")
                    else:
                        self.lane_detector = None
                        self.get_logger().info("UltraFast Lane detection initialization failed or fallback activated.")
                except Exception as e:
                    self.get_logger().error(f"Failed to init lane detector: {e}")
        else:
            self.get_logger().info("Inference explicitly disabled. Bypassing YOLO and Depth Sync.")
            # Standalone color subscriber for ZMQ video streaming (does NOT require depth)
            if self.video_pub is not None:
                self.create_subscription(Image, self.rgb_topic, self.color_only_callback,
                                         self._sensor_qos, callback_group=self._stream_cb_group)
                self.get_logger().info(f"ZMQ color stream subscriber on {self.rgb_topic} (Raw Feed Only)")

        self.get_logger().info("="*70)
        self.get_logger().info("YOLOv8 ROS 2 Server Ready!")

    def color_only_callback(self, color_msg):
        """Called for every RGB frame — streams raw image via ZMQ for Ground Station viewing"""
        if self.video_pub is None:
            return
        try:
            cv_color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            annotated = cv_color.copy()
            if getattr(self, 'lane_enabled', False) and getattr(self, 'lane_detector', None):
                lane_result = self.lane_detector.detect(cv_color)
                annotated = self.lane_detector.render_lane_overlay(annotated, lane_result)
            else:
                steering_angle, linear_x = detect_line_and_draw(annotated)
            self.video_pub.send(annotated)
            self._zmq_frame_count += 1
            if self._zmq_frame_count == 1:
                self.get_logger().info(f"[ZMQ] First frame sent! Streaming on port {self.video_port}. Connect your PC to tcp://<LIMO_IP>:{self.video_port}")
            # elif self._zmq_frame_count % 100 == 0:
            #     self.get_logger().info(f"[ZMQ] Streaming OK — {self._zmq_frame_count} frames sent on port {self.video_port}")
        except Exception as e:
            self.get_logger().error(f"Error in color_only_callback: {e}")

    def image_callback(self, color_msg, depth_msg):
        """Called whenever a synchronized pair of RGB and Depth images arrive"""
        
        try:
            # Convert ROS images to OpenCV format
            # Use bgr8 for color so it matches what cv2 and the wrapper expect
            cv_color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            
            # Astra publishes depth in 16UC1 (millimeters). 
            # The YOLOv8Wrapper_Huy seems to be able to handle standard depth arrays.
            # Make sure to convert depth to the exact format QCar2DepthAligned expects 
            # (which is usually a raw numpy float 32/64 array in meters if normalized, or straight 16UC1).
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1').astype(np.float32)
            
            # Reset buffers for new frame
            self.buffers.reset()
            
            # Pre-process image for YOLO
            processed_img = self.yolo.pre_process(cv_color)
            
            # Run YOLO prediction
            self.yolo.predict(
                inputImg=processed_img,
                classes=[0, 2, 9, 11, 33],  # person, car, traffic light, stop sign, suitcase(yield?)
                confidence=0.4,
                half=True,
                verbose=False
            )
            
            # Post-process with depth matching
            results = self.yolo.post_processing(alignedDepth=cv_depth, clippingDistance=10)
            
            # Lane Detection
            lane_result = None
            if getattr(self, 'lane_enabled', False) and getattr(self, 'lane_detector', None):
                lane_result = self.lane_detector.detect(cv_color)
                self.yolo.set_lane_result(lane_result)

            bboxes = None
            if hasattr(self.yolo, "predictions") and self.yolo.predictions is not None and len(self.yolo.predictions) > 0:
                bboxes = self.yolo.predictions[0].boxes.xyxy.cpu().numpy()
            else:
                bboxes = getattr(self.yolo, "bounding", None)
            
            # Build and send detection packet first for obstacle state
            self.buffers.fill_from_results(results, bounding_boxes=bboxes)
            self.buffers.fill_lane(lane_result)
            
            raw_packet = self.buffers.to_packet() # This returns a (7, 7) np array
            self.yolo_pub.send(raw_packet)
            
            # Render annotations and publish video stream via ZMQ if probing
            if self.video_pub is not None:
                annotated = self.yolo.post_process_render(showFPS=True, show_lane_overlay=True)
                
                # Draw obstacle box
                margin_x = (1.0 - self.buffers.center_box_width_ratio) / 2
                x_min = int(self.img_width * margin_x)
                x_max = int(self.img_width * (1.0 - margin_x))
                y_min = int(self.img_height * (1.0 - self.buffers.center_box_height_ratio))
                y_max = int(self.img_height)

                if self.buffers.obstacle[0] > 0:
                    color = (0, 0, 255)  # Red - obstacle
                    text = "Obstacle Detected"
                else:
                    color = (0, 255, 0)  # Green - clear
                    text = "Clear Path"

                cv2.rectangle(annotated, (x_min, y_min), (x_max, y_max), color, 2)
                cv2.putText(annotated, text, (x_min + 5, y_min + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                self.video_pub.send(annotated)
                
        except Exception as e:
            self.get_logger().error(f"Error in YOLO callback: {e}")

    def block_shutdown(self):
        if self.video_pub:
            self.video_pub.terminate()
        self.get_logger().info("Shutting down YOLO server.")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YoloRos2Server()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Server crashed: {e}")
    finally:
        if 'node' in locals():
            node.block_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
