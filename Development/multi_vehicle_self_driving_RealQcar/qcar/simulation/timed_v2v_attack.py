#!/usr/bin/env python3
"""Trigger and stop a V2V attack through the Ground Station WebSocket."""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from typing import List

try:
    import websockets
except ImportError:  # pragma: no cover - runtime environment check
    websockets = None


def _parse_victim_ids(raw: str) -> List[int]:
    raw = str(raw).strip()
    if raw.lower() in {"all", "-1", "[-1]"}:
        return [-1]
    if raw.startswith("["):
        values = json.loads(raw)
        if not isinstance(values, list):
            raise argparse.ArgumentTypeError("victim id JSON must be a list")
        return [int(value) for value in values]
    return [int(part.strip()) for part in raw.split(",") if part.strip()]


def _build_trigger_payload(args: argparse.Namespace) -> dict:
    return {
        "type": "trigger_attack",
        "target": args.target,
        "attack_type": args.attack_type,
        "case_num": args.case_num,
        "data_type": args.data_type,
        "attacker_id": args.attacker_id,
        "victim_ids": args.victim_ids,
    }


def _build_disable_payload(args: argparse.Namespace) -> dict:
    return {
        "type": "disable_attack",
        "target": args.target,
        "attacker_id": args.attacker_id,
    }


async def _send_timed_attack(args: argparse.Namespace) -> None:
    trigger_payload = _build_trigger_payload(args)
    disable_payload = _build_disable_payload(args)

    if args.dry_run:
        print(json.dumps(trigger_payload, indent=2, sort_keys=True))
        if not args.no_disable:
            print(json.dumps(disable_payload, indent=2, sort_keys=True))
        return

    if websockets is None:
        raise RuntimeError(
            "The 'websockets' package is required. Install project requirements or "
            "run this helper in the same environment as the Ground Station GUI."
        )

    async with websockets.connect(
        args.ws_url,
        open_timeout=args.connect_timeout,
        close_timeout=args.connect_timeout,
    ) as websocket:
        if args.attack_delay > 0.0:
            print(f"Waiting {args.attack_delay:.2f}s before triggering attack...")
            await asyncio.sleep(args.attack_delay)

        await websocket.send(json.dumps(trigger_payload))
        print(
            "Triggered "
            f"{args.attack_type} case {args.case_num} "
            f"from attacker V{args.attacker_id} on target {args.target}."
        )

        if args.no_disable:
            return

        await asyncio.sleep(args.duration)
        await websocket.send(json.dumps(disable_payload))
        print(f"Disabled attack after {args.duration:.2f}s.")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Send trigger_attack and disable_attack commands through the "
            "Ground Station WebSocket. Defaults reproduce the paper-style "
            "Mix_test Case 2 fleet/global position fault for a 3-vehicle run."
        )
    )
    parser.add_argument("--ws-url", default="ws://127.0.0.1:8080")
    parser.add_argument("--target", default="all", help="all, qcar-N, or numeric car id")
    parser.add_argument("--attack-type", default="Mix_test")
    parser.add_argument("--case-num", type=int, default=2)
    parser.add_argument(
        "--data-type",
        choices=("local", "fleet", "both"),
        default="fleet",
        help="fleet corresponds to the global/distributed V2V channel.",
    )
    parser.add_argument("--attacker-id", type=int, default=0)
    parser.add_argument("--victim-ids", type=_parse_victim_ids, default=[-1])
    parser.add_argument("--attack-delay", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--no-disable", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    if args.duration < 0.0:
        parser.error("--duration must be non-negative")
    if args.attack_delay < 0.0:
        parser.error("--attack-delay must be non-negative")
    try:
        asyncio.run(_send_timed_attack(args))
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"timed_v2v_attack failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
