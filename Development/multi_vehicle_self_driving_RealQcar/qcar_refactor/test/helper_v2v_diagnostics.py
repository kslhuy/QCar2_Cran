"""Test-only V2V diagnostic result checks and artifact generation."""

from __future__ import annotations

import csv
from pathlib import Path


def assert_two_vehicle_v2v_results(test_case, results: dict[int, dict], cycles: int) -> None:
    """Check the common two-vehicle diagnostic contract."""
    test_case.assertEqual(set(results), {1, 2})
    for vehicle_id, result in results.items():
        test_case.assertEqual(len(result["rows"]), cycles)
        test_case.assertEqual(len(result["v2v_trace"]), cycles)
        test_case.assertTrue(any(trace["peers"] for trace in result["v2v_trace"]), vehicle_id)
        test_case.assertGreater(max(trace["messages_received"] for trace in result["v2v_trace"]), 0)


def write_v2v_artifacts(
    results: dict[int, dict], raw_directory: Path, figures_directory: Path, platform_name: str
) -> None:
    """Write platform-neutral peer traces and one common diagnostic plot."""
    raw_directory.mkdir(parents=True, exist_ok=True)
    figures_directory.mkdir(parents=True, exist_ok=True)
    flattened = {vehicle_id: _flatten_trace(result["v2v_trace"]) for vehicle_id, result in results.items()}
    for vehicle_id, rows in flattened.items():
        with (raw_directory / f"vehicle_{vehicle_id}_v2v.csv").open("w", newline="", encoding="ascii") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(4, 1, figsize=(10, 13), constrained_layout=True)
    colors = {1: "tab:blue", 2: "tab:orange"}
    for vehicle_id, result in results.items():
        color = colors[vehicle_id]
        peer_rows = [row for row in flattened[vehicle_id] if row["peer_id"] is not None]
        axes[0].plot(
            [row["estimate_x_m"] for row in result["rows"]],
            [row["estimate_y_m"] for row in result["rows"]],
            color=color,
            label=f"vehicle {vehicle_id} ego estimate",
        )
        axes[0].plot(
            [row["peer_x_m"] for row in peer_rows],
            [row["peer_y_m"] for row in peer_rows],
            linestyle="--",
            color=color,
            label=f"vehicle {vehicle_id} observed peer state",
        )
        axes[1].plot(
            [row["ego_time_s"] for row in peer_rows],
            [row["transport_latency_us"] for row in peer_rows],
            color=color,
            label=f"vehicle {vehicle_id} transport latency",
        )
        axes[2].plot(
            [row["ego_time_s"] for row in peer_rows],
            [row["runtime_queue_delay_us"] for row in peer_rows],
            color=color,
            label=f"vehicle {vehicle_id} runtime queue delay",
        )
        axes[3].step(
            [row["ego_time_s"] for row in result["v2v_trace"]],
            [row["estimated_packets_lost"] for row in result["v2v_trace"]],
            where="post",
            color=color,
            label=f"vehicle {vehicle_id} estimated loss",
        )

    axes[0].set_title(f"{platform_name} Ego and V2V-Observed Peer States")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].axis("equal")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].set_title("Local UDP Transport Latency")
    axes[1].set_xlabel("ego time [s]")
    axes[1].set_ylabel("delay [us]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    axes[2].set_title("Runtime Queue Delay Before Fleet/Test Consumption")
    axes[2].set_xlabel("ego time [s]")
    axes[2].set_ylabel("delay [us]")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()
    axes[3].set_title("Cumulative Estimated UDP Packet Loss From Sequence Gaps")
    axes[3].set_xlabel("ego time [s]")
    axes[3].set_ylabel("packets")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend()
    figure.savefig(figures_directory / "peer_state_latency_loss.png", dpi=150)
    plt.close(figure)


def _flatten_trace(trace: list[dict]) -> list[dict]:
    rows = []
    for sample in trace:
        for peer in sample["peers"] or [None]:
            rows.append(
                {
                    "ego_time_s": sample["ego_time_s"],
                    "messages_sent": sample["messages_sent"],
                    "messages_received": sample["messages_received"],
                    "estimated_packets_lost": sample["estimated_packets_lost"],
                    "packets_dropped": sample["packets_dropped"],
                    "peer_id": None if peer is None else peer["peer_id"],
                    "peer_time_s": None if peer is None else peer["peer_time_s"],
                    "peer_x_m": None if peer is None else peer["peer_x_m"],
                    "peer_y_m": None if peer is None else peer["peer_y_m"],
                    "peer_velocity_mps": None if peer is None else peer["peer_velocity_mps"],
                    "sequence": None if peer is None else peer["sequence"],
                    "transport_latency_us": None if peer is None else peer["transport_latency_us"],
                    "runtime_queue_delay_us": None if peer is None else peer["runtime_queue_delay_us"],
                }
            )
    return rows
