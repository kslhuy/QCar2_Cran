#!/usr/bin/env python3
"""
Standalone ZMQ worker for ROS-free online SysID.

Run this process separately from vehicle_logic.
It receives streamed samples over ZMQ, buffers them, and triggers training
via On_Track_SysID helpers when commanded.

python Development/multi_vehicle_self_driving_RealQcar/qcar/Calibration/On_Track_SysID/src/online_sysid_zmq_worker.py --sample-port 18880 --control-port 18881 --status-port 18882

"""

from __future__ import annotations

import argparse
import time
from typing import Any, Dict

import numpy as np

try:
    import zmq
except Exception as exc:
    raise RuntimeError("pyzmq is required for online_sysid_zmq_worker") from exc

from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Calibration.On_Track_SysID.src.online_sysid_service import OnlineSysIDService


SAMPLE_RCVHWM = 8192
CONTROL_RCVHWM = 256
STATUS_SNDHWM = 128


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ZMQ worker for online SysID")
    parser.add_argument(
        "--vehicle-id",
        type=int,
        default=0,
        help="Fixed vehicle_id to accept from sample and command channels.",
    )
    parser.add_argument("--sample-host", default="127.0.0.1")
    parser.add_argument("--sample-port", type=int, default=18880)
    parser.add_argument("--control-host", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=18881)
    parser.add_argument("--status-port", type=int, default=18882)

    parser.add_argument("--racecar-version", default="SIM")
    parser.add_argument("--sample-dt", type=float, default=0.02)
    parser.add_argument("--min-speed-threshold", type=float, default=1.0)
    parser.add_argument("--min-samples", type=int, default=1500)
    parser.add_argument("--save-lut-name", default="online_sysid")
    parser.add_argument("--generate-lut", action="store_true")
    parser.add_argument("--plot-model", action="store_true")
    parser.add_argument("--package-path", default=None)
    return parser.parse_args()


def publish_status(pub_socket, status: Dict[str, Any], msg_type: str = "online_sysid_status", **extra) -> None:
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

    service = OnlineSysIDService(
        logger=None,
        vehicle_id=args.vehicle_id,
        racecar_version=args.racecar_version,
        sample_dt=args.sample_dt,
        min_speed_threshold=args.min_speed_threshold,
        min_samples=args.min_samples,
        save_lut_name=args.save_lut_name,
        generate_lut=args.generate_lut,
        plot_model=args.plot_model,
        package_path=args.package_path,
        zmq_pub_enabled=False,
    )
    service.start(collect=False)

    ctx = zmq.Context.instance()

    sample_sub = ctx.socket(zmq.SUB)
    sample_sub.setsockopt(zmq.SUBSCRIBE, b"")
    sample_sub.setsockopt(zmq.RCVHWM, SAMPLE_RCVHWM)
    sample_sub.setsockopt(zmq.RCVTIMEO, 0)
    sample_sub.setsockopt(zmq.LINGER, 0)
    sample_sub.connect(f"tcp://{args.sample_host}:{args.sample_port}")

    control_sub = ctx.socket(zmq.SUB)
    control_sub.setsockopt(zmq.SUBSCRIBE, b"")
    control_sub.setsockopt(zmq.RCVHWM, CONTROL_RCVHWM)
    control_sub.setsockopt(zmq.RCVTIMEO, 0)
    control_sub.setsockopt(zmq.LINGER, 0)
    control_sub.connect(f"tcp://{args.control_host}:{args.control_port}")

    status_pub = ctx.socket(zmq.PUB)
    status_pub.setsockopt(zmq.SNDHWM, STATUS_SNDHWM)
    status_pub.setsockopt(zmq.LINGER, 0)
    status_pub.bind(f"tcp://*:{args.status_port}")

    print(
        "[OnlineSysID-Worker] Started | "
        f"sample=tcp://{args.sample_host}:{args.sample_port}, "
        f"control=tcp://{args.control_host}:{args.control_port}, "
        f"status=tcp://*:{args.status_port}, vehicle_id={args.vehicle_id}"
    )

    # Allow subscribers to connect.
    time.sleep(0.2)

    poller = zmq.Poller()
    poller.register(sample_sub, zmq.POLLIN)
    poller.register(control_sub, zmq.POLLIN)

    last_heartbeat = 0.0
    running = True

    try:
        while running:
            events = dict(poller.poll(timeout=50))

            if sample_sub in events and events[sample_sub] == zmq.POLLIN:
                while True:
                    try:
                        msg = sample_sub.recv_json(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    except Exception:
                        break

                    if isinstance(msg, dict) and msg.get("type") == "sample":
                        vehicle_id = int(msg.get("vehicle_id", -1))
                        if vehicle_id != args.vehicle_id:
                            continue
                        sample = np.asarray(msg.get("sample", []), dtype=np.float32)
                        if sample.size == 4:
                            ts = float(msg.get("timestamp", time.time()))
                            service.submit_sample(sample, timestamp=ts)

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

                    cmd_vehicle_id = int(cmd.get("vehicle_id", -1))
                    if cmd_vehicle_id != args.vehicle_id:
                        continue

                    action = str(cmd.get("action", "")).strip().lower()
                    ok = True
                    message = "ok"

                    if action == "start":
                        service.start_collection()
                    elif action == "stop":
                        service.stop_collection()
                    elif action == "clear":
                        service.clear_buffer()
                    elif action in ("set_config", "config", "update_config"):
                        cfg = cmd.get("config", {})
                        if isinstance(cfg, dict):
                            service.update_runtime_config(cfg)
                        else:
                            ok = False
                            message = "config must be a dict"
                    elif action in ("train", "trigger_train"):
                        train_options = cmd.get("train_options", {})
                        ok, message = service.trigger_train(
                            train_options if isinstance(train_options, dict) else {}
                        )
                    elif action in ("status", "get_status"):
                        pass
                    elif action in ("shutdown", "exit"):
                        running = False
                    else:
                        ok = False
                        message = f"unknown action '{action}'"

                    publish_status(
                        status_pub,
                        service.get_status(),
                        msg_type="online_sysid_ack",
                        action=action,
                        ok=ok,
                        message=message,
                    )

            now = time.time()
            if now - last_heartbeat > 1.0:
                publish_status(status_pub, service.get_status())
                last_heartbeat = now

    except KeyboardInterrupt:
        pass
    finally:
        service.shutdown()
        try:
            sample_sub.close(linger=0)
            control_sub.close(linger=0)
            status_pub.close(linger=0)
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
