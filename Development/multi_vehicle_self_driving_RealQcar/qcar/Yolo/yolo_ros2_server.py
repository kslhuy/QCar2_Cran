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
from YoLo import YOLOVideoPublisher



class YoloRos2Server(Node):
    def __init__(self):
        super().__init__('yolo_ros2_server')

        # ROS Parameters
        self.declare_parameter('car_id', 0)
        self.declare_parameter('probing', False)
        self.declare_parameter('enable_inference', True)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('video_port', 18760) # Default base video port
        self.declare_parameter('camera_rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('yolo_publish_topic', '/limo/yolo_detections')

        # Load parameters
        self.car_id = self.get_parameter('car_id').value
        self.probing = self.get_parameter('probing').value
        self.enable_inference = self.get_parameter('enable_inference').value
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        self.video_port = self.get_parameter('video_port').value
        self.rgb_topic = self.get_parameter('camera_rgb_topic').value
        self.depth_topic = self.get_parameter('camera_depth_topic').value
        self.pub_topic = self.get_parameter('yolo_publish_topic').value

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
        
        # Initialize ZMQ Video Publisher for Ground Station
        self.video_pub = None
        if self.probing:
            try:
                self.video_pub = YOLOVideoPublisher(ip="*", port=str(self.video_port))
                self.get_logger().info(f"ZMQ Video probing enabled on port {self.video_port}")
            except Exception as e:
                self.get_logger().error(f"Failed to start ZMQ video publisher: {e}")

        if self.enable_inference:
            # Initialize YOLO
            self.get_logger().info("Initializing YOLO model...")
            self.yolo = LimoYOLO(imageHeight=self.img_height, imageWidth=self.img_width)
            
            # Initialize native ROS 2 Publisher for detections
            self.detection_pub = self.create_publisher(Float32MultiArray, self.pub_topic, 10)
            
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
            steering_angle, linear_x = detect_line_and_draw(cv_color)
            self.video_pub.send(cv_color)
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
            # Note: We pass cv_depth directly; the wrapper handles the metric extraction.
            # The original physical server passed camera.depth which is usually mapped 1-to-1.
            results = self.yolo.post_processing(alignedDepth=cv_depth, clippingDistance=10)
            
            # Fill detection buffers and send via ROS publisher
            self.buffers.fill_from_results(results)
            
            # Create ROS message: flattening the (6, 7) array into a 1D array of 42 floats
            raw_packet = self.buffers.to_packet() # This returns a (6, 7) np array
            msg = Float32MultiArray()
            msg.data = raw_packet.flatten().tolist()
            self.detection_pub.publish(msg)
            
            # Render annotations and publish video stream via ZMQ if probing
            # (color_only_callback also streams; here we override with annotated frame)
            if self.video_pub is not None:
                annotated = self.yolo.post_process_render(showFPS=True, show_lane_overlay=False)
                steering_angle, linear_x = detect_line_and_draw(annotated)
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
