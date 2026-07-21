"""Reusable CSV and plotting artifacts for fleet integration tests."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Mapping, Sequence

try:
    from .helper_simulator_process import run_vehicle_workers
except ImportError:
    from helper_simulator_process import run_vehicle_workers


_COLORS = ("#2563eb", "#dc2626", "#059669", "#7c3aed", "#ea580c", "#0891b2")


def run_fleet_integration(
    test_case: Any,
    *,
    project_root: Path,
    runner_module: str,
    setup_file: Path,
    cycles: int,
    extra_args: Sequence[str],
    timeout_s: float,
    fleet_order: Sequence[int],
    platform_name: str,
    artifact_directory: str | Path,
    summary_filename: str,
    validation_position_columns: tuple[str, str] = ("gps_x_m", "gps_y_m"),
    plot_position_columns: tuple[str, str] = ("gps_x_m", "gps_y_m"),
    minimum_gap_m: float = 0.0,
    minimum_steering_rad: float | None = None,
    desired_gap_m: float | None = None,
    time_headway_s: float = 0.0,
    maximum_steady_state_spacing_error_m: float | None = None,
    display_y_sign: float = 1.0,
    exported_position_prefix: str | None = None,
    trajectory_aspect: str = "adaptive",
    require_final_active: bool = False,
) -> dict[int, Sequence[Mapping[str, object]]]:
    """Run, validate, and record a fleet scenario on any simulator platform."""
    results = run_vehicle_workers(
        test_case,
        project_root=project_root,
        runner_module=runner_module,
        setup_file=setup_file,
        cycles=cycles,
        extra_args=list(extra_args),
        require_v2v_trace=False,
        timeout_s=timeout_s,
        vehicle_ids=fleet_order,
    )
    rows_by_vehicle = assert_fleet_run(
        test_case,
        results,
        fleet_order=fleet_order,
        cycles=cycles,
        position_columns=validation_position_columns,
        minimum_gap_m=minimum_gap_m,
        minimum_steering_rad=minimum_steering_rad,
        desired_gap_m=desired_gap_m,
        time_headway_s=time_headway_s,
        maximum_steady_state_spacing_error_m=maximum_steady_state_spacing_error_m,
    )
    if require_final_active:
        for index, vehicle_id in enumerate(fleet_order):
            final_row = rows_by_vehicle[vehicle_id][-1]
            expected_source = "simple_path_controller" if index == 0 else "fleet_2d_controller"
            test_case.assertEqual(final_row["fleet_phase"], "active")
            test_case.assertEqual(final_row["command_source"], expected_source)

    summary_path = write_fleet_artifacts(
        rows_by_vehicle,
        artifact_directory,
        fleet_order=fleet_order,
        platform_name=platform_name,
        summary_filename=summary_filename,
        position_columns=plot_position_columns,
        display_y_sign=display_y_sign,
        exported_position_prefix=exported_position_prefix,
        trajectory_aspect=trajectory_aspect,
        desired_gap_m=desired_gap_m,
        time_headway_s=time_headway_s,
    )
    test_case.assertTrue(summary_path.is_file())
    test_case.assertTrue(_diagnostics_path(summary_path).is_file())
    return rows_by_vehicle


def assert_fleet_run(
    test_case: Any,
    results: Mapping[int, Mapping[str, Any]],
    *,
    fleet_order: Sequence[int],
    cycles: int,
    position_columns: tuple[str, str] = ("gps_x_m", "gps_y_m"),
    minimum_gap_m: float = 0.0,
    minimum_steering_rad: float | None = None,
    desired_gap_m: float | None = None,
    time_headway_s: float = 0.0,
    maximum_steady_state_spacing_error_m: float | None = None,
) -> dict[int, Sequence[Mapping[str, object]]]:
    """Validate common leader/follower behavior for an arbitrary fleet size."""
    vehicle_ids = tuple(fleet_order)
    test_case.assertGreaterEqual(len(vehicle_ids), 2)
    test_case.assertEqual(len(set(vehicle_ids)), len(vehicle_ids))
    test_case.assertEqual(set(results), set(vehicle_ids))
    rows_by_vehicle = {vehicle_id: results[vehicle_id]["rows"] for vehicle_id in vehicle_ids}
    for vehicle_id, rows in rows_by_vehicle.items():
        test_case.assertEqual(len(rows), cycles, f"vehicle {vehicle_id} returned an unexpected sample count")

    for vehicle_id in vehicle_ids[1:]:
        rows = rows_by_vehicle[vehicle_id]
        test_case.assertIn("active", [row["fleet_phase"] for row in rows])
        test_case.assertIn("fleet_2d_controller", [row["command_source"] for row in rows])
        test_case.assertGreater(max(abs(_number(row, "throttle")) for row in rows), 0.01)

    if minimum_steering_rad is not None:
        follower_steering = [
            abs(_number(row, "steering_rad"))
            for vehicle_id in vehicle_ids[1:]
            for row in rows_by_vehicle[vehicle_id]
        ]
        test_case.assertGreater(max(follower_steering), minimum_steering_rad)

    if minimum_gap_m > 0.0:
        x_key, y_key = position_columns
        for front_id, rear_id in zip(vehicle_ids, vehicle_ids[1:]):
            minimum_gap = min(
                _distance(front, rear, x_key, y_key)
                for front, rear in zip(rows_by_vehicle[front_id], rows_by_vehicle[rear_id])
            )
            test_case.assertGreaterEqual(minimum_gap, minimum_gap_m)
    if maximum_steady_state_spacing_error_m is not None:
        if desired_gap_m is None:
            raise ValueError("desired_gap_m is required when checking spacing error")
        for rear_id in vehicle_ids[1:]:
            active_rows = [row for row in rows_by_vehicle[rear_id] if row.get("fleet_phase") == "active"]
            steady_rows = active_rows[len(active_rows) // 2:]
            errors = [
                abs(_number(row, "predecessor_gap_m") - _desired_gap(row, desired_gap_m, time_headway_s))
                for row in steady_rows
                if row.get("predecessor_gap_m") is not None
            ]
            test_case.assertTrue(errors, f"vehicle {rear_id} has no active predecessor-gap samples")
            test_case.assertLessEqual(max(errors), maximum_steady_state_spacing_error_m)
    return rows_by_vehicle


def write_fleet_artifacts(
    rows_by_vehicle: Mapping[int, Sequence[Mapping[str, object]]],
    artifact_directory: str | Path,
    *,
    fleet_order: Sequence[int] | None = None,
    platform_name: str,
    summary_filename: str,
    position_columns: tuple[str, str] = ("gps_x_m", "gps_y_m"),
    display_y_sign: float = 1.0,
    exported_position_prefix: str | None = None,
    trajectory_aspect: str = "adaptive",
    desired_gap_m: float | None = None,
    time_headway_s: float = 0.0,
) -> Path:
    """Write fleet telemetry CSVs and a 2x2 summary for any fleet size.

    ``fleet_order`` defines the leader followed by each predecessor-successor
    pair. ``display_y_sign`` converts a project-frame Y coordinate to a
    platform-native display frame when needed, such as CARLA's Y axis.
    """
    ordered_vehicle_ids = tuple(fleet_order or sorted(rows_by_vehicle))
    if not ordered_vehicle_ids:
        raise ValueError("Fleet artifacts require at least one vehicle")
    if set(ordered_vehicle_ids) != set(rows_by_vehicle):
        raise ValueError("fleet_order must contain every telemetry vehicle exactly once")
    if len(set(ordered_vehicle_ids)) != len(ordered_vehicle_ids):
        raise ValueError("fleet_order cannot contain duplicate vehicle IDs")
    if any(not rows_by_vehicle[vehicle_id] for vehicle_id in ordered_vehicle_ids):
        raise ValueError("Fleet telemetry cannot contain an empty vehicle series")

    directory = Path(artifact_directory)
    directory.mkdir(parents=True, exist_ok=True)
    labels = _vehicle_labels(ordered_vehicle_ids)
    for index, vehicle_id in enumerate(ordered_vehicle_ids):
        rows = rows_by_vehicle[vehicle_id]
        artifact_rows = [
            _artifact_row(
                row,
                position_columns,
                display_y_sign,
                exported_position_prefix,
                desired_gap_m if index > 0 else None,
                time_headway_s,
            )
            for row in rows
        ]
        with (directory / f"vehicle_{vehicle_id}.csv").open("w", newline="", encoding="ascii") as file:
            writer = csv.DictWriter(file, fieldnames=list(artifact_rows[0]))
            writer.writeheader()
            writer.writerows(artifact_rows)

    if trajectory_aspect not in {"adaptive", "equal"}:
        raise ValueError("trajectory_aspect must be 'adaptive' or 'equal'")
    _write_summary_plot(
        rows_by_vehicle,
        ordered_vehicle_ids,
        labels,
        directory / summary_filename,
        platform_name,
        position_columns,
        display_y_sign,
        trajectory_aspect,
        desired_gap_m,
        time_headway_s,
    )
    _write_diagnostics_plot(
        rows_by_vehicle,
        ordered_vehicle_ids,
        labels,
        _diagnostics_path(directory / summary_filename),
        platform_name,
    )
    return directory / summary_filename


def _artifact_row(
    row: Mapping[str, object],
    position_columns: tuple[str, str],
    display_y_sign: float,
    exported_position_prefix: str | None,
    desired_gap_m: float | None,
    time_headway_s: float,
) -> dict[str, object]:
    artifact = dict(row)
    if exported_position_prefix is not None:
        x_key, y_key = position_columns
        artifact[f"{exported_position_prefix}_x_m"] = _number(row, x_key)
        artifact[f"{exported_position_prefix}_y_m"] = display_y_sign * _number(row, y_key)
    if desired_gap_m is not None:
        artifact["target_predecessor_gap_m"] = _desired_gap(row, desired_gap_m, time_headway_s)
    return artifact


def _write_summary_plot(
    rows_by_vehicle: Mapping[int, Sequence[Mapping[str, object]]],
    vehicle_ids: Sequence[int],
    labels: Mapping[int, str],
    output_path: Path,
    platform_name: str,
    position_columns: tuple[str, str],
    display_y_sign: float,
    trajectory_aspect: str,
    desired_gap_m: float | None,
    time_headway_s: float,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 2, figsize=(14, 11))
    figure.subplots_adjust(left=0.08, right=0.98, bottom=0.08, top=0.94, wspace=0.12, hspace=0.38)
    trajectory_axis, spacing_axis = axes[0]
    speed_axis, steering_axis = axes[1]
    x_key, y_key = position_columns
    trajectory_x = []
    trajectory_y = []
    time_origin = _time_origin(rows_by_vehicle, vehicle_ids)

    for index, vehicle_id in enumerate(vehicle_ids):
        rows = rows_by_vehicle[vehicle_id]
        position_rows = _rows_with_valid_gps(rows)
        color = _COLORS[index % len(_COLORS)]
        label = labels[vehicle_id]
        x_values = [_number(row, x_key) for row in position_rows]
        y_values = [display_y_sign * _number(row, y_key) for row in position_rows]
        trajectory_x.extend(x_values)
        trajectory_y.extend(y_values)
        trajectory_axis.plot(
            x_values,
            y_values,
            color=color, linewidth=2.2, label=label,
        )
        speed_axis.plot(
            _elapsed_times(rows, time_origin),
            [_number(row, "speed_mps") for row in rows],
            color=color, linewidth=1.8, label=label,
        )
        steering_axis.plot(
            _elapsed_times(rows, time_origin),
            [_number(row, "steering_rad") for row in rows],
            color=color, linewidth=1.8, label=label,
        )

    for front_id, rear_id in zip(vehicle_ids, vehicle_ids[1:]):
        front_rows = _rows_with_valid_gps(rows_by_vehicle[front_id])
        rear_rows = _rows_with_valid_gps(rows_by_vehicle[rear_id])
        snapshot_gaps = [row.get("predecessor_gap_m") for row in rear_rows[:len(front_rows)]]
        has_snapshot_gaps = bool(snapshot_gaps) and all(_has_number(value) for value in snapshot_gaps)
        spacing_axis.plot(
            _elapsed_times(rear_rows[:len(front_rows)], time_origin),
            [float(value) for value in snapshot_gaps]
            if has_snapshot_gaps
            else [_distance(front, rear, x_key, y_key) for front, rear in zip(front_rows, rear_rows)],
            linewidth=2.0,
            label=f"{labels[front_id]} - {labels[rear_id]}",
        )
        if desired_gap_m is not None:
            spacing_axis.plot(
                _elapsed_times(rear_rows[:len(front_rows)], time_origin),
                [_desired_gap(row, desired_gap_m, time_headway_s) for row in rear_rows[:len(front_rows)]],
                color="#475569", linestyle="--", linewidth=1.2,
                label=f"{labels[rear_id]} target",
            )

    _set_trajectory_view(trajectory_axis, trajectory_x, trajectory_y, trajectory_aspect)
    trajectory_axis.set_title(f"{platform_name} trajectories ({trajectory_aspect} view)")
    trajectory_axis.set_xlabel(f"{platform_name} x [m]")
    trajectory_axis.set_ylabel(f"{platform_name} y [m]")
    spacing_axis.set_title("Physical predecessor spacing")
    spacing_axis.set_xlabel("elapsed time [s]")
    spacing_axis.set_ylabel("distance [m]")
    speed_axis.set_title("Vehicle speeds")
    speed_axis.set_xlabel("elapsed time [s]")
    speed_axis.set_ylabel("speed [m/s]")
    steering_axis.set_title("Steering commands")
    steering_axis.set_xlabel("elapsed time [s]")
    steering_axis.set_ylabel("steering [rad]")
    legend_columns = 2 if len(vehicle_ids) > 3 else 1
    for axis in (trajectory_axis, spacing_axis, speed_axis, steering_axis):
        axis.grid(True, alpha=0.28)
        axis.legend(frameon=False, fontsize=8, ncol=legend_columns)
    figure.savefig(output_path, dpi=150)
    plt.close(figure)


def _write_diagnostics_plot(
    rows_by_vehicle: Mapping[int, Sequence[Mapping[str, object]]],
    vehicle_ids: Sequence[int],
    labels: Mapping[int, str],
    output_path: Path,
    platform_name: str,
) -> None:
    """Write fleet lifecycle and communication diagnostics in a second 2x2 figure."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 2, figsize=(15, 10))
    figure.subplots_adjust(left=0.08, right=0.98, bottom=0.08, top=0.93, wspace=0.16, hspace=0.52)
    phase_axis, age_axis = axes[0]
    gap_axis, estimate_axis = axes[1]
    phase_values = {"disabled": 0, "building": 1, "active": 2, "cancelling": 3, "fault": 4}
    time_origin = _time_origin(rows_by_vehicle, vehicle_ids)

    for index, vehicle_id in enumerate(vehicle_ids):
        rows = rows_by_vehicle[vehicle_id]
        color = _COLORS[index % len(_COLORS)]
        time_values = _elapsed_times(rows, time_origin)
        phase_axis.step(
            time_values,
            [phase_values.get(str(row.get("fleet_phase", "disabled")), -1) for row in rows],
            where="post", color=color, linewidth=1.8, label=labels[vehicle_id],
        )
        if vehicle_id != vehicle_ids[0]:
            age_values = [_optional_number(row.get("predecessor_age_s")) for row in rows]
            age_axis.plot(time_values, age_values, color=color, linewidth=1.6, label=labels[vehicle_id])
        gap_axis.plot(
            time_values,
            [_optional_number(row.get("v2v_sequence_gap"), default=0.0) for row in rows],
            color=color, linewidth=1.6, label=labels[vehicle_id],
        )
        estimate_axis.step(
            time_values,
            [_optional_number(row.get("distributed_estimate_count"), default=0.0) for row in rows],
            where="post", color=color, linewidth=1.6, label=labels[vehicle_id],
        )

    phase_axis.set_yticks(tuple(phase_values.values()), tuple(phase_values))
    phase_axis.set_title("Fleet phase")
    phase_axis.set_xlabel("elapsed time [s]")
    age_axis.set_title("Local predecessor snapshot age")
    age_axis.set_xlabel("elapsed time [s]")
    age_axis.set_ylabel("age [s]")
    gap_axis.set_title("Cumulative V2V\nsequence gaps")
    gap_axis.set_xlabel("elapsed time [s]")
    gap_axis.set_ylabel("dropped messages")
    estimate_axis.set_title("Advisory distributed-estimate\ncoverage")
    estimate_axis.set_xlabel("elapsed time [s]")
    estimate_axis.set_ylabel("vehicle estimates")
    for axis in (phase_axis, age_axis, gap_axis, estimate_axis):
        axis.grid(True, alpha=0.28)
        axis.legend(frameon=False, fontsize=8, ncol=2 if len(vehicle_ids) > 3 else 1)
    figure.savefig(output_path, dpi=150)
    plt.close(figure)


def _vehicle_labels(vehicle_ids: Sequence[int]) -> dict[int, str]:
    return {
        vehicle_id: "leader" if index == 0 else f"follower {index}"
        for index, vehicle_id in enumerate(vehicle_ids)
    }


def _distance(
    front: Mapping[str, object],
    rear: Mapping[str, object],
    x_key: str,
    y_key: str,
) -> float:
    dx = _number(front, x_key) - _number(rear, x_key)
    dy = _number(front, y_key) - _number(rear, y_key)
    return (dx * dx + dy * dy) ** 0.5


def _number(row: Mapping[str, object], key: str) -> float:
    try:
        return float(row[key])
    except KeyError as exc:
        raise ValueError(f"Fleet telemetry row has no '{key}' field") from exc


def _desired_gap(row: Mapping[str, object], desired_gap_m: float, time_headway_s: float) -> float:
    return float(desired_gap_m) + float(time_headway_s) * max(0.0, _number(row, "speed_mps"))


def _time_origin(rows_by_vehicle: Mapping[int, Sequence[Mapping[str, object]]], vehicle_ids: Sequence[int]) -> float:
    return min(_number(rows_by_vehicle[vehicle_id][0], "time_s") for vehicle_id in vehicle_ids)


def _elapsed_times(rows: Sequence[Mapping[str, object]], time_origin: float) -> list[float]:
    return [_number(row, "time_s") - time_origin for row in rows]


def _optional_number(value: object, default: float = float("nan")) -> float:
    if value is None or (isinstance(value, str) and not value.strip()):
        return default
    return float(value)


def _has_number(value: object) -> bool:
    return value is not None and not (isinstance(value, str) and not value.strip())


def _diagnostics_path(summary_path: Path) -> Path:
    return summary_path.with_name(f"{summary_path.stem}_diagnostics{summary_path.suffix}")


def _rows_with_valid_gps(rows: Sequence[Mapping[str, object]]) -> Sequence[Mapping[str, object]]:
    """Ignore uninitialised GPS cache values in deterministic virtual IO."""
    if not rows or "gps_valid" not in rows[0]:
        return rows
    valid_rows = [row for row in rows if _boolean(row["gps_valid"])]
    return valid_rows or rows


def _boolean(value: object) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes"}
    return bool(value)


def _set_trajectory_view(axis, x_values: Sequence[float], y_values: Sequence[float], aspect: str) -> None:
    """Frame long, low-curvature routes without wasting most of a 2x2 tile."""
    if not x_values or not y_values:
        return
    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    x_span = max(x_max - x_min, 1e-6)
    y_span = max(y_max - y_min, 1e-6)
    axis.set_xlim(x_min - max(0.25, 0.04 * x_span), x_max + max(0.25, 0.04 * x_span))
    axis.set_ylim(y_min - max(0.10, 0.18 * y_span), y_max + max(0.10, 0.18 * y_span))
    axis.set_aspect("equal" if aspect == "equal" else "auto", adjustable="box")
