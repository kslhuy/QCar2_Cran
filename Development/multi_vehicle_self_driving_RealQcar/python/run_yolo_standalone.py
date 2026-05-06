#!/usr/bin/env python3
"""
Run physical yolo_server.py directly from the car for debugging.

This launcher is standalone and path-robust:
- Finds qcar/Yolo/yolo_server.py by searching parent folders
- Computes default video port as 18760 + car_id
- Runs in foreground so stdout/stderr are visible immediately

Example:
    python run_yolo_standalone.py --car-id 2 --probing true
"""

import argparse
import os
import subprocess
import sys
from typing import Optional


def _find_yolo_server(start_dir: str, max_depth: int = 8) -> Optional[str]:
    """Find qcar/Yolo/yolo_server.py by walking up parent directories."""
    search_dir = os.path.abspath(start_dir)

    for _ in range(max_depth):
        candidate = os.path.join(search_dir, "qcar", "Yolo", "yolo_server.py")
        if os.path.exists(candidate):
            return candidate

        parent = os.path.dirname(search_dir)
        if parent == search_dir:
            break
        search_dir = parent

    return None


def _normalize_bool(value: str) -> str:
    """Map common truthy/falsy values to yolo_server.py expected strings."""
    true_values = {"1", "true", "yes", "y", "on"}
    false_values = {"0", "false", "no", "n", "off"}

    v = value.strip().lower()
    if v in true_values:
        return "True"
    if v in false_values:
        return "False"

    raise ValueError(f"Invalid boolean value '{value}'. Use true/false.")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run physical YOLO server standalone")
    parser.add_argument("--car-id", type=int, required=True, help="Vehicle ID")
    parser.add_argument(
        "--probing",
        type=str,
        default="true",
        help="Enable ZMQ video probing (true/false). Default: true",
    )
    parser.add_argument(
        "--video-port",
        type=int,
        default=None,
        help="Video port override. Default: 18760 + car_id",
    )
    parser.add_argument("--ip-host", type=str, default="localhost", help="Host IP")
    parser.add_argument("--width", type=int, default=320, help="Frame width")
    parser.add_argument("--height", type=int, default=200, help="Frame height")

    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    yolo_server = _find_yolo_server(script_dir)
    if not yolo_server:
        print("[ERROR] Could not find qcar/Yolo/yolo_server.py")
        return 2

    probing_flag = _normalize_bool(args.probing)
    video_port = args.video_port if args.video_port is not None else (18760 + args.car_id)

    cmd = [
        sys.executable,
        yolo_server,
        "-i",
        args.ip_host,
        "-p",
        probing_flag,
        "-w",
        str(args.width),
        "-ht",
        str(args.height),
        "-idx",
        str(args.car_id),
        "--video-port",
        str(video_port),
    ]

    print("=" * 72)
    print("YOLO Standalone Launcher")
    print("=" * 72)
    print(f"Script : {yolo_server}")
    print(f"Car ID : {args.car_id}")
    print(f"Probing: {probing_flag}")
    print(f"Port   : {video_port}")
    print("Command:")
    print(" ".join(cmd))
    print("=" * 72)

    # Run in foreground so logs/errors are visible in this terminal.
    process = subprocess.Popen(cmd)

    try:
        return process.wait()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user, terminating YOLO server...")
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
