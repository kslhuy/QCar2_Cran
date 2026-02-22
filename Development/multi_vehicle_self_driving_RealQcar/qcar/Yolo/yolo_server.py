"""
YOLO Server Physical - Refactored for Robustness
matches architecture of yolo_server_virtual.py but for physical QCar.
"""

import numpy as np
import time
import cv2
import os
import sys
import io
import argparse
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Optional, Any

# Retaining original camera import as requested
from pit.YOLO.utils import QCar2DepthAligned

# Using enhanced wrapper for consistency with virtual server (better rendering)
from YOLOv8Wrapper_Huy import YOLOv8Wrapper_Huy, DetectionBuffers
from Yolo.YoLo import YOLOPublisher, YOLOVideoPublisher


# =============================================================================
# Configuration
# =============================================================================
@dataclass
class ServerConfig:
    """Configuration for YOLO server."""

    ip_host: str = "localhost"
    width: int = 320
    height: int = 200
    car_id: int = 0
    probing: bool = False
    image_width: int = 640
    image_height: int = 480
    video_port: int = (
        18760  # Base port; actual port = 18760 + car_id (set via --video-port)
    )

    @classmethod
    def from_args(cls) -> "ServerConfig":
        """Create config from command line arguments."""
        parser = argparse.ArgumentParser(prog="YOLO Server Physical")
        parser.add_argument("-i", "--ip_host", default="localhost")
        parser.add_argument("-p", "--probing", default="False")
        parser.add_argument("-w", "--width", type=int, default=320)
        parser.add_argument("-ht", "--height", type=int, default=200)
        parser.add_argument("-idx", "--caridx", type=int, default=0)
        parser.add_argument("--video-port", type=int, default=18766)
        # Note: --car-id was used in original script, mapping both just in case
        parser.add_argument("--car-id", type=int, dest="caridx_alt", default=None)

        args = parser.parse_args()

        # Handle car_id / idx ambiguity
        cid = args.caridx
        if args.caridx_alt is not None:
            cid = args.caridx_alt

        return cls(
            ip_host=args.ip_host,
            width=args.width,
            height=args.height,
            car_id=cid,
            probing=(args.probing == "True"),
            video_port=args.video_port,
        )


# =============================================================================
# Probe Manager - Handles observer connection and streaming
# =============================================================================


# =============================================================================
# Detection Buffers - Pre-allocated for performance
# =============================================================================
# DetectionBuffers imported from YOLOv8Wrapper_Huy


# =============================================================================
# Main YOLO Server Class for Physical Car
# =============================================================================
class YOLOServerPhysical:
    """
    Main YOLO server for Physical QCar.
    - Uses hardcoded port 18666
    - Uses QCar2DepthAligned (pit.YOLO.utils)
    - No local display (Probe only)
    """

    def __init__(self, config: ServerConfig):
        self.config = config
        self.running = False

        # Initialize components
        self._init_camera()
        self._init_yolo()
        self._init_publisher()
        self._init_video_publisher()

        # Pre-allocated buffers
        self.buffers = DetectionBuffers()

        print(f"[SERVER] YOLOServerPhysical initialized for Car {config.car_id}")

    def _init_camera(self):
        """Initialize camera using the specified QCar2DepthAligned class."""
        # NOTE: Using hardcoded port 18777 as in original script for physical car
        # Just in case multiple cars run on same hardware (?) no,
        # usually 1 car = 1 OS instance.
        # Original script had: QCarImg = QCar2DepthAligned(port='18777')
        try:
            self.camera = QCar2DepthAligned(port="18777")
            print("[SERVER] Camera initialized (QCar2DepthAligned)")
        except Exception as e:
            print(f"[SERVER] Data capture init failed: {e}")
            raise e

    def _init_yolo(self):
        """Initialize YOLO detector."""
        # Using enhanced wrapper for better visualization in Probe
        self.yolo = YOLOv8Wrapper_Huy(
            imageHeight=self.config.image_height, imageWidth=self.config.image_width
        )
        print("[SERVER] YOLOv8 initialized")

    def _init_publisher(self):
        """Initialize YOLO data publisher."""
        # HARDCODED PORT 18666 as requested
        self.publisher = YOLOPublisher(port="18666")
        print("[SERVER] Publisher initialized on port 18666")

    def _init_video_publisher(self):
        """Initialize video publisher if probing is enabled."""
        self.video_publisher = None
        if self.config.probing:
            # Use configured video port and IP
            video_port = str(self.config.video_port)
            # Always bind to all interfaces ('*') so the Ground Station can connect
            # from a different machine.  ip_host is intentionally not used here.
            self.video_publisher = YOLOVideoPublisher(ip="*", port=video_port)
            print(
                f"[SERVER] Video streaming enabled on *:{video_port} (all interfaces)"
            )

    def run(self):
        """Main processing loop."""
        self.running = True
        print(f"[SERVER] Starting main loop for Car {self.config.car_id}")

        try:
            while self.running:
                self._process_frame()
        except KeyboardInterrupt:
            print("[SERVER] User interrupted")
        finally:
            self.terminate()

    def _process_frame(self):
        """Process a single frame."""
        # Reset buffers
        self.buffers.reset()

        # Get aligned RGB and depth
        self.camera.read()

        # Original logic: just read .rgb and .depth from camera object
        # QCar2DepthAligned puts them in .rgb and .depth attributes
        raw_rgb = self.camera.rgb
        raw_depth = self.camera.depth

        # YOLO detection
        # Note: YOLOv8Wrapper_Huy might expect 640x480.
        # If camera gives 640x480, we are good.
        processed = self.yolo.pre_process(raw_rgb)

        self.yolo.predict(
            inputImg=processed,
            classes=[0, 2, 9, 11, 33],
            confidence=0.4,
            half=True,
            verbose=False,
        )

        results = self.yolo.post_processing(alignedDepth=raw_depth, clippingDistance=10)

        # Render annotated image
        # Even without Lane Detection, this renders bounding boxes nicer
        annotated = self.yolo.post_process_render(showFPS=True, show_lane_overlay=True)

        # Send video if enabled
        if self.video_publisher:
            self.video_publisher.send(annotated)

        # Build and send detection packet
        self.buffers.fill_from_results(results)
        # No lane data for physical car yet, buffer stays 0

        self.publisher.send(self.buffers.to_packet())

    def terminate(self):
        """Clean shutdown of all components."""
        print("[SERVER] Terminating...")
        self.running = False

        if hasattr(self, "camera"):
            try:
                self.camera.terminate()
            except:
                pass

        if hasattr(self, "publisher"):
            try:
                self.publisher.terminate()
            except:
                pass

        if self.video_publisher:
            self.video_publisher.terminate()

        print("[SERVER] Shutdown complete")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.terminate()
        return False


# =============================================================================
# Entry Point
# =============================================================================
def main():
    config = ServerConfig.from_args()
    try:
        with YOLOServerPhysical(config) as server:
            server.run()
    except Exception as e:
        print(f"[FATAL] Server crashed: {e}")
        # Try to print traceback
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
