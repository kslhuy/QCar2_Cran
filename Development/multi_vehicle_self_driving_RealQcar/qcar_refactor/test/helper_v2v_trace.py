"""Test-only multi-process worker for generic V2V transport diagnostics."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any

from extra.simulator.carla.process_runner import CarlaProcessManager
from extra.simulator.virtual.process_runner import VirtualProcessManager


_TEST_MESSAGE_TYPE = "TEST_STATE_V1"


class TestV2VTraceRecorder:
    """Publish a synthetic payload and capture generic transport metadata."""

    def __init__(self) -> None:
        self._latest_messages: dict[int, Any] = {}
        self.trace: list[dict[str, Any]] = []

    def record_step(self, runtime: Any, telemetry: Any) -> None:
        for message in runtime.v2v.drain_received():
            if message.message_type == _TEST_MESSAGE_TYPE:
                self._latest_messages[message.sender_id] = message
        runtime.v2v.publish(_TEST_MESSAGE_TYPE, _test_state_payload(telemetry))
        self.trace.append(_serialize_trace(telemetry, runtime.v2v, self._latest_messages))


def main(argv: list[str] | None = None) -> int:
    """Run one platform manager with test-only V2V transport instrumentation."""
    parser = argparse.ArgumentParser(description="Run one simulator vehicle with a test-only V2V trace")
    parser.add_argument("--platform", choices=("carla", "virtual"), required=True)
    parser.add_argument("--setup-file", required=True)
    parser.add_argument("--vehicle-id", required=True, type=int)
    parser.add_argument("--cycles", required=True, type=int)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--start-file", type=Path, required=True)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--realtime", action="store_true")
    args = parser.parse_args(argv)

    manager = _build_manager(args.platform, args.setup_file, args.vehicle_id)
    recorder = TestV2VTraceRecorder()

    def on_ready(runtime, context) -> None:
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
        args.ready_file.touch()
        manager.wait_for_start_signal(args.start_file)
        if args.platform == "carla" and context.vehicle.tick_owner:
            time.sleep(1.0)

    def on_step(runtime, telemetry, context) -> None:
        recorder.record_step(runtime, telemetry)
        if args.platform == "carla" and context.vehicle.tick_owner:
            time.sleep(float(runtime.config.module("simulation")["fixed_delta_seconds"]))
        if args.realtime:
            time.sleep(args.dt)

    context, telemetry = manager.run(
        args.cycles,
        dt=lambda runtime, _context: _resolve_dt(args.platform, runtime, args.dt),
        on_ready=on_ready,
        on_step=on_step,
    )
    print(
        json.dumps(
            {
                "vehicle_id": context.vehicle.vehicle_id,
                "rows": [manager.serialize_telemetry(item) for item in telemetry],
                "v2v_trace": recorder.trace,
            }
        ),
        flush=True,
    )
    return 0


def _build_manager(platform: str, setup_file: str, vehicle_id: int):
    if platform == "carla":
        return CarlaProcessManager(setup_file, vehicle_id)
    return VirtualProcessManager(setup_file, vehicle_id)


def _resolve_dt(platform: str, runtime: Any, requested_dt: float) -> float:
    if platform == "carla":
        return float(runtime.config.module("simulation")["fixed_delta_seconds"])
    return requested_dt


def _test_state_payload(telemetry: Any) -> dict[str, float]:
    return {
        "time_s": float(telemetry.estimate.timestamp),
        "x_m": float(telemetry.estimate.x),
        "y_m": float(telemetry.estimate.y),
        "velocity_mps": float(telemetry.estimate.velocity),
    }


def _serialize_trace(telemetry: Any, v2v: Any, latest_messages: dict[int, Any]) -> dict[str, Any]:
    status = v2v.get_status()
    drained_at_perf_counter_ns = time.perf_counter_ns()
    peers = []
    for peer_id, message in latest_messages.items():
        payload = message.payload
        if not all(key in payload for key in ("time_s", "x_m", "y_m", "velocity_mps")):
            continue
        peers.append(
            {
                "peer_id": int(peer_id),
                "peer_time_s": float(payload["time_s"]),
                "peer_x_m": float(payload["x_m"]),
                "peer_y_m": float(payload["y_m"]),
                "peer_velocity_mps": float(payload["velocity_mps"]),
                "sequence": int(message.sequence),
                "transport_latency_us": float(
                    (message.received_at_perf_counter_ns - message.sent_at_perf_counter_ns) / 1000.0
                ),
                "runtime_queue_delay_us": float(
                    (drained_at_perf_counter_ns - message.received_at_perf_counter_ns) / 1000.0
                ),
            }
        )
    return {
        "ego_time_s": float(telemetry.sensor_data.sensor_timestamp),
        "messages_sent": int(status.get("messages_sent", 0)),
        "messages_received": int(status.get("messages_received", 0)),
        "estimated_packets_lost": int(status.get("estimated_packets_lost", 0)),
        "packets_dropped": int(status.get("packets_dropped", 0)),
        "peers": peers,
    }


if __name__ == "__main__":
    raise SystemExit(main())
