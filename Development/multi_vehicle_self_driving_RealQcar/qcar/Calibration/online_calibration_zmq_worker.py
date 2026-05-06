#!/usr/bin/env python3
"""
Standalone ZMQ worker for passive online calibration.

Run this process separately from vehicle_logic.
It receives streamed calibration samples over ZMQ, buffers them, and
triggers analysis when commanded.

Usage:
    python online_calibration_zmq_worker.py \\
        --sample-port 18890 --control-port 18891 --status-port 18892

The worker instantiates OnlineCalibrationService which handles all
buffering and analysis internally.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Any, Dict

import numpy as np

try:
    import zmq
except Exception as exc:
    raise RuntimeError(
        "pyzmq is required for online_calibration_zmq_worker"
    ) from exc

# Ensure this directory is on sys.path for local imports
_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

from online_calibration_service import OnlineCalibrationService


SAMPLE_RCVHWM = 8192
CONTROL_RCVHWM = 256
STATUS_SNDHWM = 128


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ZMQ worker for passive online calibration"
    )
    parser.add_argument(
        "--vehicle-id", type=int, default=0,
        help="Vehicle ID to accept samples from",
    )
    parser.add_argument("--sample-host", default="127.0.0.1")
    parser.add_argument("--sample-port", type=int, default=18890)
    parser.add_argument("--control-host", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=18891)
    parser.add_argument("--status-port", type=int, default=18892)
    parser.add_argument(
        "--sample-dt", type=float, default=0.02,
        help="Expected sample period [s]",
    )
    parser.add_argument(
        "--buffer-size", type=int, default=20000,
        help="Maximum number of samples to buffer",
    )
    parser.add_argument(
        "--poly-degree", type=int, default=3,
        help="Polynomial degree for throttle-velocity / steering fits",
    )
    parser.add_argument(
        "--wheelbase", type=float, default=0.256,
        help="Nominal wheelbase [m] for Ackermann fit",
    )
    return parser.parse_args()


def publish_status(
    pub_socket,
    status: Dict[str, Any],
    msg_type: str = "online_calibration_status",
    **extra,
) -> None:
    payload = {
        "type": msg_type,
        "timestamp": time.time(),
        "status": status,
    }
    payload.update(extra)
    try:
        pub_socket.send_json(payload, flags=zmq.NOBLOCK)
    except Exception:
        pass


def main() -> int:
    args = parse_args()

    service = OnlineCalibrationService(
        logger=None,
        vehicle_id=args.vehicle_id,
        sample_dt=args.sample_dt,
        buffer_size=args.buffer_size,
        poly_degree=args.poly_degree,
        wheelbase_nom=args.wheelbase,
    )
    service.start(collect=False)

    ctx = zmq.Context.instance()

    # SUB: receive samples from vehicle client
    sample_sub = ctx.socket(zmq.SUB)
    sample_sub.setsockopt(zmq.SUBSCRIBE, b"")
    sample_sub.setsockopt(zmq.RCVHWM, SAMPLE_RCVHWM)
    sample_sub.setsockopt(zmq.RCVTIMEO, 0)
    sample_sub.setsockopt(zmq.LINGER, 0)
    sample_sub.connect(f"tcp://{args.sample_host}:{args.sample_port}")

    # SUB: receive control commands from vehicle client
    control_sub = ctx.socket(zmq.SUB)
    control_sub.setsockopt(zmq.SUBSCRIBE, b"")
    control_sub.setsockopt(zmq.RCVHWM, CONTROL_RCVHWM)
    control_sub.setsockopt(zmq.RCVTIMEO, 0)
    control_sub.setsockopt(zmq.LINGER, 0)
    control_sub.connect(f"tcp://{args.control_host}:{args.control_port}")

    # PUB: publish status/results back to vehicle client
    status_pub = ctx.socket(zmq.PUB)
    status_pub.setsockopt(zmq.SNDHWM, STATUS_SNDHWM)
    status_pub.setsockopt(zmq.LINGER, 0)
    status_pub.bind(f"tcp://*:{args.status_port}")

    print(
        "[OnlineCal-Worker] Started | "
        f"sample=tcp://{args.sample_host}:{args.sample_port}, "
        f"control=tcp://{args.control_host}:{args.control_port}, "
        f"status=tcp://*:{args.status_port}, vehicle_id={args.vehicle_id}"
    )

    time.sleep(0.2)  # allow subscribers to connect

    poller = zmq.Poller()
    poller.register(sample_sub, zmq.POLLIN)
    poller.register(control_sub, zmq.POLLIN)

    last_heartbeat = 0.0
    samples_rx_total = 0
    last_sample_log_total = 0
    last_sample_log_time = 0.0
    running = True

    try:
        while running:
            events = dict(poller.poll(timeout=50))

            # --- Process samples ---
            if sample_sub in events and events[sample_sub] == zmq.POLLIN:
                while True:
                    try:
                        msg = sample_sub.recv_json(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    except Exception:
                        break

                    if isinstance(msg, dict) and msg.get("type") == "sample":
                        vid = int(msg.get("vehicle_id", -1))
                        if vid != args.vehicle_id:
                            continue
                        sample = np.asarray(
                            msg.get("sample", []), dtype=np.float32
                        )
                        if sample.size == OnlineCalibrationService.SAMPLE_SIZE:
                            ts = float(msg.get("timestamp", time.time()))
                            if service.submit_sample(sample, timestamp=ts):
                                samples_rx_total += 1
                                now = time.time()
                                if (
                                    samples_rx_total == 1
                                    or samples_rx_total - last_sample_log_total >= 1000
                                    or now - last_sample_log_time >= 10.0
                                ):
                                    status = service.get_status()
                                    print(
                                        "[OnlineCal-Worker] Receiving samples | "
                                        f"accepted_total={samples_rx_total}, "
                                        f"buffered={status.get('buffered_samples', 0)}, "
                                        f"collecting={status.get('collecting', False)}",
                                        flush=True,
                                    )
                                    last_sample_log_total = samples_rx_total
                                    last_sample_log_time = now

            # --- Process commands ---
            if control_sub in events and events[control_sub] == zmq.POLLIN:
                while True:
                    try:
                        cmd = control_sub.recv_json(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    except Exception:
                        break

                    if not isinstance(cmd, dict) or cmd.get("type") != "cmd":
                        continue

                    cmd_vid = int(cmd.get("vehicle_id", -1))
                    if cmd_vid != args.vehicle_id:
                        continue

                    action = str(cmd.get("action", "")).strip().lower()
                    ok = True
                    message = "ok"

                    if action == "start":
                        service.start_collection()
                        status = service.get_status()
                        print(
                            "[OnlineCal-Worker] GUI command: Collect -> "
                            f"collecting=True, buffered={status.get('buffered_samples', 0)}",
                            flush=True,
                        )
                    elif action == "stop":
                        service.stop_collection()
                        status = service.get_status()
                        print(
                            "[OnlineCal-Worker] GUI command: Pause -> "
                            f"collecting=False, buffered={status.get('buffered_samples', 0)}",
                            flush=True,
                        )
                    elif action == "clear":
                        before = service.get_status().get("buffered_samples", 0)
                        service.clear_buffer()
                        print(
                            "[OnlineCal-Worker] GUI command: Clear -> "
                            f"buffered {before} -> 0",
                            flush=True,
                        )
                    elif action in ("analyse", "analyze", "trigger_analyse"):
                        cal_type = str(
                            cmd.get("calibration_type", "throttle_velocity")
                        )
                        options = cmd.get("options", {})
                        ok, message = service.trigger_analyse(
                            cal_type,
                            options if isinstance(options, dict) else {},
                        )
                        status = service.get_status()
                        print(
                            "[OnlineCal-Worker] GUI command: Analyse -> "
                            f"type={cal_type}, queued={ok}, "
                            f"buffered={status.get('buffered_samples', 0)}, "
                            f"message={message}",
                            flush=True,
                        )
                    elif action in ("status", "get_status"):
                        status = service.get_status()
                        print(
                            "[OnlineCal-Worker] GUI command: Status -> "
                            f"collecting={status.get('collecting', False)}, "
                            f"buffered={status.get('buffered_samples', 0)}",
                            flush=True,
                        )
                    elif action in ("shutdown", "exit"):
                        print(
                            "[OnlineCal-Worker] GUI command: Shutdown",
                            flush=True,
                        )
                        running = False
                    else:
                        ok = False
                        message = f"unknown action '{action}'"
                        print(
                            f"[OnlineCal-Worker] Unknown GUI command: {action}",
                            flush=True,
                        )

                    publish_status(
                        status_pub,
                        service.get_status(),
                        msg_type="online_calibration_ack",
                        action=action,
                        ok=ok,
                        message=message,
                    )

            # --- Periodic heartbeat ---
            now = time.time()
            if now - last_heartbeat > 1.0:
                publish_status(status_pub, service.get_status())
                last_heartbeat = now

    except KeyboardInterrupt:
        pass
    finally:
        service.shutdown()
        for sock in (sample_sub, control_sub, status_pub):
            try:
                sock.close(linger=0)
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
