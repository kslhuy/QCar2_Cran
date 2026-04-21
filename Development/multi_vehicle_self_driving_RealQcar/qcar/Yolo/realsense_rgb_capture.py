"""
Simple RGB capture script for an Intel RealSense D435i.

Requirements:
    pip install pyrealsense2 opencv-python numpy

Run:
    python realsense_rgb_capture.py

Keys:
    s - save the current RGB frame as realsense_rgb.png
    q - quit
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: pyrealsense2. Install it with:\n"
        "    pip install pyrealsense2"
    ) from exc


def print_connected_devices():
    """Print RealSense devices visible to pyrealsense2."""
    devices = rs.context().query_devices()
    if len(devices) == 0:
        print("No RealSense device found.")
        return

    print("Connected RealSense device(s):")
    for index, device in enumerate(devices):
        name = device.get_info(rs.camera_info.name)
        serial = device.get_info(rs.camera_info.serial_number)
        print(f"  [{index}] {name}, serial={serial}")


def print_active_profile(profile):
    """Print the stream profile selected by the RealSense SDK."""
    device = profile.get_device()
    device_name = device.get_info(rs.camera_info.name)
    serial = device.get_info(rs.camera_info.serial_number)
    print(f"Using device: {device_name}, serial={serial}")

    for stream in profile.get_streams():
        video_profile = stream.as_video_stream_profile()
        if video_profile:
            intrinsics = video_profile.get_intrinsics()
            print(
                "Active stream: "
                f"{stream.stream_name()} "
                f"{intrinsics.width}x{intrinsics.height} "
                f"{stream.format()} "
                f"{stream.fps()} FPS"
            )


def get_rgb_frame(pipeline, timeout_ms):
    """Read one frame from the RealSense camera and return it as RGB."""
    try:
        frames = pipeline.wait_for_frames(timeout_ms)
    except RuntimeError:
        return None

    color_frame = frames.get_color_frame()

    if not color_frame:
        return None

    return np.asanyarray(color_frame.get_data())


def main():
    parser = argparse.ArgumentParser(
        description="Show and optionally save RGB frames from a RealSense D435i."
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=10000,
        help="How long to wait for a frame before reporting a missed frame.",
    )
    parser.add_argument(
        "--serial",
        default=None,
        help="Optional RealSense serial number to use when multiple cameras are connected.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("realsense_rgb.png"),
        help="Path used when pressing 's'.",
    )
    args = parser.parse_args()

    print_connected_devices()

    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)

    config.enable_stream(
        rs.stream.color,
        args.width,
        args.height,
        rs.format.rgb8,
        args.fps,
    )

    print("Starting RealSense RGB stream...")
    profile = pipeline.start(config)
    print_active_profile(profile)

    try:
        # Let auto exposure settle for a short moment.
        time.sleep(1.0)
        missed_frames = 0

        while True:
            rgb_image = get_rgb_frame(pipeline, args.timeout_ms)
            if rgb_image is None:
                missed_frames += 1
                print(
                    "No RGB frame received. "
                    "Check that no other app is using the camera, use a USB 3 port/cable, "
                    "or try lower settings such as --width 424 --height 240 --fps 15."
                )
                if missed_frames >= 3:
                    print("Stopping after 3 missed frames.")
                    break
                continue

            missed_frames = 0

            # OpenCV windows and imwrite expect BGR, but rgb_image stays RGB.
            bgr_for_display = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            cv2.imshow("RealSense D435i RGB", bgr_for_display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                cv2.imwrite(str(args.output), bgr_for_display)
                print(f"Saved RGB image to {args.output}")
            elif key == ord("q"):
                break

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
