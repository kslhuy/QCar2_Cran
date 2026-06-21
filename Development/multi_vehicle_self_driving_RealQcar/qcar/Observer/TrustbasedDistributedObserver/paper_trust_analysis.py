"""
Build compact AI-readable summaries and paper figures from trust_weight_log CSVs.

The realtime logger intentionally records a wide CSV for plotting/debugging.
This script creates smaller sidecar artifacts:
  - ai_trust_summary_V*.csv: one compact row per timestep
  - ai_trust_events_V*.jsonl: threshold, attack, and rollback events
  - ai_trust_metrics_V*.json: run-level metrics and paper-readiness notes
  - fig/qcar_limo_trust_overview_V*.png: quick paper/debug figure

It avoids matplotlib so it can run in environments where compiled numpy/matplotlib
packages are unavailable.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    from PIL import Image, ImageDraw, ImageFont
except Exception:  # pragma: no cover - optional dependency
    Image = None
    ImageDraw = None
    ImageFont = None


SUMMARY_COLUMNS = [
    "time",
    "active_vehicle_count",
    "mean_direct_trust",
    "mean_generalized_trust",
    "mean_gamma_host",
    "mean_gamma_local_peer",
    "mean_gamma_self",
    "mean_neighbor_weight",
    "total_neighbor_weight",
    "weighted_neighbor_trust",
    "trusted_neighbor_count",
    "prediction_mode_count",
    "rollback_enabled",
    "rollback_triggered",
    "rollback_total",
    "rollback_active_count",
    "rollback_active_vehicles",
    "rollback_newly_flagged_count",
    "rollback_newly_flagged",
    "v2v_attack_enabled",
    "v2v_attack_active",
    "v2v_attack_active_count",
    "v2v_attack_types",
    "v2v_attack_names",
    "v2v_attack_data_types",
    "flag_attack_count",
    "flag_local_count",
    "flag_global_count",
    "inject_attack_active_count",
    "min_trust",
    "min_gtrust",
    "max_neighbor_weight",
    "max_est_pos_err",
    "max_consensus_pos_err",
    "max_postpred_pos_err",
    "max_abs_est_vel_err",
    "worst_vehicle_by_est_pos_err",
    "worst_vehicle_by_consensus_pos_err",
]

BASE_COLORS = [
    (31, 119, 180),
    (214, 39, 40),
    (44, 160, 44),
    (255, 127, 14),
    (148, 103, 189),
    (140, 86, 75),
    (23, 190, 207),
    (127, 127, 127),
]


def safe_float(value, default: float = float("nan")) -> float:
    try:
        if value is None:
            return default
        if isinstance(value, str):
            value = value.strip()
            if value == "":
                return default
        return float(value)
    except (TypeError, ValueError):
        return default


def finite(value: float) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def fmt(value, precision: int = 6) -> str:
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.{precision}g}"
    if value is None:
        return ""
    return str(value)


def load_csv(path: Path) -> Tuple[List[str], List[Dict[str, str]]]:
    cleaned_lines: List[str] = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            line = line.replace("\x00", "")
            if len(line) < 131072:
                cleaned_lines.append(line)
    reader = csv.DictReader(cleaned_lines)
    return reader.fieldnames or [], list(reader)


def col_series(rows: Sequence[Dict[str, str]], column: str) -> List[float]:
    return [safe_float(row.get(column, "")) for row in rows]


def text_series(rows: Sequence[Dict[str, str]], column: str) -> List[str]:
    return [str(row.get(column, "") or "").strip() for row in rows]


def finite_values(values: Iterable[float]) -> List[float]:
    return [float(value) for value in values if finite(value)]


def stats(values: Iterable[float]) -> Dict[str, float]:
    clean = finite_values(values)
    if not clean:
        nan = float("nan")
        return {"count": 0, "mean": nan, "min": nan, "max": nan, "rmse": nan}
    return {
        "count": len(clean),
        "mean": sum(clean) / len(clean),
        "min": min(clean),
        "max": max(clean),
        "rmse": math.sqrt(sum(value * value for value in clean) / len(clean)),
    }


def median(values: Sequence[float]) -> float:
    clean = sorted(finite_values(values))
    if not clean:
        return float("nan")
    mid = len(clean) // 2
    if len(clean) % 2:
        return clean[mid]
    return 0.5 * (clean[mid - 1] + clean[mid])


def extract_vehicle_ids(columns: Sequence[str]) -> List[int]:
    ids = set()
    prefixes = {
        "vehicle_present",
        "trust",
        "gtrust",
        "w_neighbor",
        "est_x",
        "est_y",
        "est_pos_err",
        "consensus_pos_err",
        "postpred_pos_err",
        "flag_attack",
        "inject_attack_active",
    }
    for col in columns:
        match = re.search(r"_(\d+)$", col)
        if match and col[: match.start()] in prefixes:
            ids.add(int(match.group(1)))
    return sorted(ids)


def active_vehicle_ids(rows: Sequence[Dict[str, str]], columns: Sequence[str]) -> List[int]:
    active: List[int] = []
    for vid in extract_vehicle_ids(columns):
        present = False
        for prefix in ("vehicle_present", "trust", "gtrust", "w_neighbor", "est_x"):
            col = f"{prefix}_{vid}"
            if col not in columns:
                continue
            values = col_series(rows, col)
            if prefix == "vehicle_present":
                if any(finite(value) and value > 0 for value in values):
                    present = True
                    break
            elif any(finite(value) for value in values):
                present = True
                break
        if present:
            active.append(vid)
    return active


def host_id_from_filename(path: Path) -> Optional[int]:
    match = re.search(r"V(\d+)", path.stem)
    return int(match.group(1)) if match else None


def max_value_with_vehicle(
    row: Dict[str, str], active: Sequence[int], prefix: str, absolute: bool = False
) -> Tuple[float, str]:
    best_value = float("nan")
    best_vehicle = ""
    for vid in active:
        value = safe_float(row.get(f"{prefix}_{vid}", ""))
        if not finite(value):
            continue
        compare_value = abs(value) if absolute else value
        if not finite(best_value) or compare_value > best_value:
            best_value = compare_value
            best_vehicle = str(vid)
    return best_value, best_vehicle


def min_vehicle_value(row: Dict[str, str], active: Sequence[int], prefix: str) -> float:
    values = [safe_float(row.get(f"{prefix}_{vid}", "")) for vid in active]
    clean = finite_values(values)
    return min(clean) if clean else float("nan")


def max_vehicle_value(row: Dict[str, str], active: Sequence[int], prefix: str) -> float:
    values = [safe_float(row.get(f"{prefix}_{vid}", "")) for vid in active]
    clean = finite_values(values)
    return max(clean) if clean else float("nan")


def count_positive(row: Dict[str, str], active: Sequence[int], prefix: str) -> int:
    count = 0
    for vid in active:
        value = safe_float(row.get(f"{prefix}_{vid}", ""), 0.0)
        if finite(value) and value > 0:
            count += 1
    return count


def build_summary_rows(
    rows: Sequence[Dict[str, str]], columns: Sequence[str], active: Sequence[int]
) -> List[Dict[str, object]]:
    summary_rows: List[Dict[str, object]] = []
    for row in rows:
        max_est_pos_err, worst_est = max_value_with_vehicle(row, active, "est_pos_err")
        max_consensus_pos_err, worst_consensus = max_value_with_vehicle(
            row, active, "consensus_pos_err"
        )
        max_postpred_pos_err, _ = max_value_with_vehicle(row, active, "postpred_pos_err")
        max_abs_est_vel_err, _ = max_value_with_vehicle(
            row, active, "est_vel_err", absolute=True
        )
        out = {
            "time": safe_float(row.get("time", "")),
            "active_vehicle_count": safe_float(row.get("active_vehicle_count", "")),
            "mean_direct_trust": safe_float(row.get("mean_direct_trust", "")),
            "mean_generalized_trust": safe_float(row.get("mean_generalized_trust", "")),
            "mean_gamma_host": safe_float(row.get("mean_gamma_host", "")),
            "mean_gamma_local_peer": safe_float(row.get("mean_gamma_local_peer", "")),
            "mean_gamma_self": safe_float(row.get("mean_gamma_self", "")),
            "mean_neighbor_weight": safe_float(row.get("mean_neighbor_weight", "")),
            "total_neighbor_weight": safe_float(row.get("total_neighbor_weight", "")),
            "weighted_neighbor_trust": safe_float(row.get("weighted_neighbor_trust", "")),
            "trusted_neighbor_count": safe_float(row.get("trusted_neighbor_count", "")),
            "prediction_mode_count": safe_float(row.get("prediction_mode_count", "")),
            "rollback_enabled": int(safe_float(row.get("rollback_enabled", 0.0), 0.0)),
            "rollback_triggered": int(safe_float(row.get("rollback_triggered", 0.0), 0.0)),
            "rollback_total": int(safe_float(row.get("rollback_total", 0.0), 0.0)),
            "rollback_active_count": int(
                safe_float(row.get("rollback_active_count", 0.0), 0.0)
            ),
            "rollback_active_vehicles": row.get("rollback_active_vehicles", ""),
            "rollback_newly_flagged_count": int(
                safe_float(row.get("rollback_newly_flagged_count", 0.0), 0.0)
            ),
            "rollback_newly_flagged": row.get("rollback_newly_flagged", ""),
            "v2v_attack_enabled": int(
                safe_float(row.get("v2v_attack_enabled", 0.0), 0.0)
            ),
            "v2v_attack_active": int(
                safe_float(row.get("v2v_attack_active", 0.0), 0.0)
            ),
            "v2v_attack_active_count": int(
                safe_float(row.get("v2v_attack_active_count", 0.0), 0.0)
            ),
            "v2v_attack_types": row.get("v2v_attack_types", ""),
            "v2v_attack_names": row.get("v2v_attack_names", ""),
            "v2v_attack_data_types": row.get("v2v_attack_data_types", ""),
            "flag_attack_count": count_positive(row, active, "flag_attack"),
            "flag_local_count": count_positive(row, active, "flag_local"),
            "flag_global_count": count_positive(row, active, "flag_global"),
            "inject_attack_active_count": count_positive(
                row, active, "inject_attack_active"
            ),
            "min_trust": min_vehicle_value(row, active, "trust"),
            "min_gtrust": min_vehicle_value(row, active, "gtrust"),
            "max_neighbor_weight": max_vehicle_value(row, active, "w_neighbor"),
            "max_est_pos_err": max_est_pos_err,
            "max_consensus_pos_err": max_consensus_pos_err,
            "max_postpred_pos_err": max_postpred_pos_err,
            "max_abs_est_vel_err": max_abs_est_vel_err,
            "worst_vehicle_by_est_pos_err": worst_est,
            "worst_vehicle_by_consensus_pos_err": worst_consensus,
        }
        summary_rows.append(out)
    return summary_rows


def write_summary_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: fmt(row.get(key, "")) for key in SUMMARY_COLUMNS})


def write_json(path: Path, data: Dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(json_safe(data), handle, indent=2, sort_keys=True, allow_nan=False)
        handle.write("\n")


def json_safe(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    if isinstance(value, tuple):
        return [json_safe(item) for item in value]
    return value


def write_events(path: Path, events: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for event in events:
            handle.write(json.dumps(event, sort_keys=True) + "\n")


def build_events(
    rows: Sequence[Dict[str, str]], active: Sequence[int], threshold: float
) -> List[Dict[str, object]]:
    events: List[Dict[str, object]] = []
    if not rows:
        return events

    prev_attack_active = 0
    prev_rollback_total = 0
    prev_flag = {vid: 0 for vid in active}
    prev_low = {
        vid: safe_float(rows[0].get(f"trust_{vid}", "")) < threshold
        if finite(safe_float(rows[0].get(f"trust_{vid}", "")))
        else False
        for vid in active
    }

    for row in rows:
        t = safe_float(row.get("time", ""))
        if not finite(t):
            continue

        attack_active = int(safe_float(row.get("v2v_attack_active", 0.0), 0.0) > 0)
        if attack_active != prev_attack_active:
            events.append(
                {
                    "time": t,
                    "event": "attack_start" if attack_active else "attack_end",
                    "active_count": int(
                        safe_float(row.get("v2v_attack_active_count", 0.0), 0.0)
                    ),
                    "types": row.get("v2v_attack_types", ""),
                    "names": row.get("v2v_attack_names", ""),
                    "data_types": row.get("v2v_attack_data_types", ""),
                }
            )
            prev_attack_active = attack_active

        rollback_total = int(safe_float(row.get("rollback_total", 0.0), 0.0))
        rollback_triggered = int(safe_float(row.get("rollback_triggered", 0.0), 0.0))
        if rollback_triggered or rollback_total > prev_rollback_total:
            events.append(
                {
                    "time": t,
                    "event": "rollback",
                    "total_rollbacks": rollback_total,
                    "active_vehicles": row.get("rollback_active_vehicles", ""),
                    "newly_flagged": row.get("rollback_newly_flagged", ""),
                }
            )
            prev_rollback_total = max(prev_rollback_total, rollback_total)

        for vid in active:
            trust = safe_float(row.get(f"trust_{vid}", ""))
            if finite(trust):
                is_low = trust < threshold
                if is_low and not prev_low[vid]:
                    events.append(
                        {
                            "time": t,
                            "event": "trust_below_threshold",
                            "vehicle": vid,
                            "trust": trust,
                            "threshold": threshold,
                        }
                    )
                elif not is_low and prev_low[vid]:
                    events.append(
                        {
                            "time": t,
                            "event": "trust_recovered",
                            "vehicle": vid,
                            "trust": trust,
                            "threshold": threshold,
                        }
                    )
                prev_low[vid] = is_low

            flag = int(safe_float(row.get(f"flag_attack_{vid}", 0.0), 0.0) > 0)
            if flag != prev_flag[vid]:
                events.append(
                    {
                        "time": t,
                        "event": "attack_flag_on" if flag else "attack_flag_off",
                        "vehicle": vid,
                    }
                )
                prev_flag[vid] = flag

    return events


def time_step_seconds(times: Sequence[float]) -> List[float]:
    deltas: List[float] = []
    previous = None
    for value in times:
        if finite(value):
            if previous is not None and value >= previous:
                deltas.append(value - previous)
            previous = value
    return deltas


def active_duration(times: Sequence[float], mask: Sequence[float]) -> float:
    deltas = time_step_seconds(times)
    if not deltas:
        return 0.0
    duration = 0.0
    for idx, dt in enumerate(deltas):
        if idx < len(mask) and finite(mask[idx]) and mask[idx] > 0:
            duration += dt
    return duration


def first_time_where(
    rows: Sequence[Dict[str, str]], column: str, threshold: float, below: bool
) -> Optional[float]:
    for row in rows:
        value = safe_float(row.get(column, ""))
        t = safe_float(row.get("time", ""))
        if not finite(value) or not finite(t):
            continue
        if below and value < threshold:
            return t
        if not below and value >= threshold:
            return t
    return None


def run_metrics(
    csv_path: Path,
    rows: Sequence[Dict[str, str]],
    columns: Sequence[str],
    active: Sequence[int],
    events: Sequence[Dict[str, object]],
    threshold: float,
    warmup_s: float,
) -> Dict[str, object]:
    times = col_series(rows, "time")
    deltas = time_step_seconds(times)
    t_clean = finite_values(times)
    duration = (max(t_clean) - min(t_clean)) if t_clean else float("nan")
    warmup_cutoff = (min(t_clean) + warmup_s) if t_clean else float("nan")
    attack_mask = col_series(rows, "v2v_attack_active")
    rollback_total = stats(col_series(rows, "rollback_total"))["max"]
    attack_duration = active_duration(times, attack_mask)

    vehicle_metrics: Dict[str, object] = {}
    worst_est_vehicle = ""
    worst_est_peak = float("nan")
    min_trust_global = float("nan")
    min_trust_after_warmup = float("nan")
    for vid in active:
        trust_series = col_series(rows, f"trust_{vid}")
        trust = stats(trust_series)
        trust_after_warmup = [
            trust_value
            for t, trust_value in zip(times, trust_series)
            if finite(t) and finite(trust_value) and t >= warmup_cutoff
        ]
        trust_after_warmup_stats = stats(trust_after_warmup)
        gtrust = stats(col_series(rows, f"gtrust_{vid}"))
        w_neighbor = stats(col_series(rows, f"w_neighbor_{vid}"))
        est_err = stats(col_series(rows, f"est_pos_err_{vid}"))
        consensus_err = stats(col_series(rows, f"consensus_pos_err_{vid}"))
        postpred_err = stats(col_series(rows, f"postpred_pos_err_{vid}"))
        flag_count = sum(
            1
            for value in col_series(rows, f"flag_attack_{vid}")
            if finite(value) and value > 0
        )
        final_trust = float("nan")
        for value in reversed(col_series(rows, f"trust_{vid}")):
            if finite(value):
                final_trust = value
                break
        vehicle_metrics[str(vid)] = {
            "trust": trust,
            "trust_after_warmup": trust_after_warmup_stats,
            "generalized_trust": gtrust,
            "neighbor_weight": w_neighbor,
            "est_pos_err": est_err,
            "consensus_pos_err": consensus_err,
            "postpred_pos_err": postpred_err,
            "flag_attack_samples": flag_count,
            "first_trust_below_threshold_s": first_time_where(
                rows, f"trust_{vid}", threshold, below=True
            ),
            "final_trust": final_trust,
        }
        if finite(trust["min"]):
            if not finite(min_trust_global) or trust["min"] < min_trust_global:
                min_trust_global = trust["min"]
        if finite(trust_after_warmup_stats["min"]):
            if (
                not finite(min_trust_after_warmup)
                or trust_after_warmup_stats["min"] < min_trust_after_warmup
            ):
                min_trust_after_warmup = trust_after_warmup_stats["min"]
        if finite(est_err["max"]):
            if not finite(worst_est_peak) or est_err["max"] > worst_est_peak:
                worst_est_peak = est_err["max"]
                worst_est_vehicle = str(vid)

    has_reference_error = any(
        stats(col_series(rows, f"est_pos_err_{vid}"))["count"] > 0 for vid in active
    )
    has_attack = any(finite(value) and value > 0 for value in attack_mask) or any(
        any(finite(value) and value > 0 for value in col_series(rows, f"inject_attack_active_{vid}"))
        for vid in active
    )
    rollback_events = [event for event in events if event.get("event") == "rollback"]

    if not has_attack:
        readiness = (
            "baseline_only: no active attack was logged, so this run cannot support "
            "the rollback validation figure."
        )
    elif not rollback_events:
        readiness = (
            "attack_without_rollback: useful no-rollback baseline if matched with "
            "a rollback-enabled run."
        )
    else:
        readiness = "rollback_candidate: attack and rollback events are present."

    if not has_reference_error:
        readiness += " Reference-error columns are empty; use trust/weight/trajectory metrics or log clean references for RMSE."

    return {
        "source_csv": str(csv_path),
        "host_vehicle": host_id_from_filename(csv_path),
        "samples": len(rows),
        "time_start_s": min(t_clean) if t_clean else None,
        "time_end_s": max(t_clean) if t_clean else None,
        "duration_s": duration if finite(duration) else None,
        "median_dt_s": median(deltas) if deltas else None,
        "active_vehicles": list(active),
        "warmup_s": warmup_s,
        "has_attack": has_attack,
        "attack_active_duration_s": attack_duration,
        "rollback_events": len(rollback_events),
        "rollback_total_max": rollback_total if finite(rollback_total) else None,
        "min_trust": min_trust_global if finite(min_trust_global) else None,
        "min_trust_after_warmup": min_trust_after_warmup
        if finite(min_trust_after_warmup)
        else None,
        "worst_est_pos_err_vehicle": worst_est_vehicle,
        "worst_est_pos_err_peak_m": worst_est_peak if finite(worst_est_peak) else None,
        "has_reference_error": has_reference_error,
        "paper_readiness": readiness,
        "vehicle_metrics": vehicle_metrics,
    }


def get_font(size: int, bold: bool = False):
    if ImageFont is None:
        return None
    names = ["arialbd.ttf" if bold else "arial.ttf", "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"]
    for name in names:
        try:
            return ImageFont.truetype(name, size)
        except Exception:
            continue
    return ImageFont.load_default()


def nice_bounds(values: Sequence[float], default: Tuple[float, float]) -> Tuple[float, float]:
    clean = finite_values(values)
    if not clean:
        return default
    lo = min(clean)
    hi = max(clean)
    if not math.isfinite(lo) or not math.isfinite(hi) or lo == hi:
        pad = 1.0 if lo == 0 else abs(lo) * 0.1
        return lo - pad, hi + pad
    pad = 0.08 * (hi - lo)
    return lo - pad, hi + pad


def draw_text(draw, xy, text, fill=(35, 35, 35), font=None, anchor=None):
    kwargs = {"fill": fill}
    if font is not None:
        kwargs["font"] = font
    if anchor is not None:
        kwargs["anchor"] = anchor
    draw.text(xy, text, **kwargs)


def draw_panel_frame(draw, box, title: str, font_title, font_small):
    x0, y0, x1, y1 = box
    draw.rounded_rectangle(box, radius=8, outline=(185, 190, 198), width=2, fill=(255, 255, 255))
    draw_text(draw, (x0 + 14, y0 + 10), title, font=font_title, fill=(20, 35, 55))
    plot_box = (x0 + 70, y0 + 48, x1 - 22, y1 - 46)
    px0, py0, px1, py1 = plot_box
    draw.line((px0, py1, px1, py1), fill=(90, 100, 115), width=2)
    draw.line((px0, py0, px0, py1), fill=(90, 100, 115), width=2)
    return plot_box


def map_point(
    x: float,
    y: float,
    box: Tuple[int, int, int, int],
    xb: Tuple[float, float],
    yb: Tuple[float, float],
) -> Tuple[int, int]:
    x0, y0, x1, y1 = box
    xmin, xmax = xb
    ymin, ymax = yb
    if xmax <= xmin:
        xmax = xmin + 1.0
    if ymax <= ymin:
        ymax = ymin + 1.0
    px = x0 + (x - xmin) / (xmax - xmin) * (x1 - x0)
    py = y1 - (y - ymin) / (ymax - ymin) * (y1 - y0)
    return int(round(px)), int(round(py))


def draw_ticks(
    draw,
    box: Tuple[int, int, int, int],
    xb: Tuple[float, float],
    yb: Tuple[float, float],
    font_small,
    x_label: str = "time [s]",
) -> None:
    x0, y0, x1, y1 = box
    for idx in range(5):
        frac = idx / 4.0
        x = int(x0 + frac * (x1 - x0))
        y = int(y1 - frac * (y1 - y0))
        xv = xb[0] + frac * (xb[1] - xb[0])
        yv = yb[0] + frac * (yb[1] - yb[0])
        draw.line((x, y1, x, y1 + 5), fill=(105, 115, 125), width=1)
        draw.line((x0 - 5, y, x0, y), fill=(105, 115, 125), width=1)
        draw_text(draw, (x - 18, y1 + 9), f"{xv:.1f}", font=font_small, fill=(80, 86, 95))
        draw_text(draw, (x0 - 62, y - 7), f"{yv:.2g}", font=font_small, fill=(80, 86, 95))
    draw_text(draw, ((x0 + x1) // 2 - 28, y1 + 28), x_label, font=font_small, fill=(70, 76, 85))


def draw_line_series(
    draw,
    box: Tuple[int, int, int, int],
    xs: Sequence[float],
    ys: Sequence[float],
    xb: Tuple[float, float],
    yb: Tuple[float, float],
    color: Tuple[int, int, int],
    width: int = 3,
) -> None:
    segment: List[Tuple[int, int]] = []
    for x, y in zip(xs, ys):
        if finite(x) and finite(y):
            segment.append(map_point(x, y, box, xb, yb))
        else:
            if len(segment) >= 2:
                draw.line(segment, fill=color, width=width, joint="curve")
            segment = []
    if len(segment) >= 2:
        draw.line(segment, fill=color, width=width, joint="curve")


def draw_legend(draw, x: int, y: int, labels: Sequence[Tuple[str, Tuple[int, int, int]]], font_small) -> None:
    offset = 0
    for label, color in labels[:8]:
        draw.line((x + offset, y + 8, x + offset + 24, y + 8), fill=color, width=4)
        draw_text(draw, (x + offset + 30, y), label, font=font_small, fill=(60, 65, 72))
        offset += max(90, 30 + len(label) * 8)


def draw_no_data(draw, box: Tuple[int, int, int, int], text: str, font) -> None:
    x0, y0, x1, y1 = box
    draw_text(
        draw,
        ((x0 + x1) // 2, (y0 + y1) // 2),
        text,
        font=font,
        fill=(125, 130, 138),
        anchor="mm",
    )


def draw_time_panel(
    draw,
    box: Tuple[int, int, int, int],
    title: str,
    times: Sequence[float],
    series: Sequence[Tuple[str, Sequence[float], Tuple[int, int, int]]],
    y_bounds: Optional[Tuple[float, float]],
    font_title,
    font_small,
    empty_text: str = "No data",
) -> None:
    plot_box = draw_panel_frame(draw, box, title, font_title, font_small)
    all_y: List[float] = []
    for _, ys, _ in series:
        all_y.extend(ys)
    if not finite_values(times) or not finite_values(all_y):
        draw_no_data(draw, plot_box, empty_text, font_title)
        return
    xb = (min(finite_values(times)), max(finite_values(times)))
    yb = y_bounds if y_bounds else nice_bounds(all_y, (0.0, 1.0))
    draw_ticks(draw, plot_box, xb, yb, font_small)
    for label, ys, color in series:
        draw_line_series(draw, plot_box, times, ys, xb, yb, color)
    draw_legend(draw, plot_box[0] + 8, box[1] + 16, [(label, color) for label, _, color in series], font_small)


def draw_xy_panel(
    draw,
    box: Tuple[int, int, int, int],
    title: str,
    rows: Sequence[Dict[str, str]],
    active: Sequence[int],
    font_title,
    font_small,
) -> None:
    plot_box = draw_panel_frame(draw, box, title, font_title, font_small)
    trajectories: List[Tuple[str, List[float], List[float], Tuple[int, int, int]]] = []
    all_x: List[float] = []
    all_y: List[float] = []
    for idx, vid in enumerate(active):
        xs = col_series(rows, f"est_x_{vid}")
        ys = col_series(rows, f"est_y_{vid}")
        if finite_values(xs) and finite_values(ys):
            color = BASE_COLORS[idx % len(BASE_COLORS)]
            trajectories.append((f"est V{vid}", xs, ys, color))
            all_x.extend(xs)
            all_y.extend(ys)
    if not trajectories:
        draw_no_data(draw, plot_box, "No estimated XY trajectory in this log", font_title)
        return
    xb = nice_bounds(all_x, (0.0, 1.0))
    yb = nice_bounds(all_y, (0.0, 1.0))
    draw_ticks(draw, plot_box, xb, yb, font_small, x_label="x [m]")
    for label, xs, ys, color in trajectories:
        segment: List[Tuple[int, int]] = []
        for x, y in zip(xs, ys):
            if finite(x) and finite(y):
                segment.append(map_point(x, y, plot_box, xb, yb))
            else:
                if len(segment) >= 2:
                    draw.line(segment, fill=color, width=3, joint="curve")
                segment = []
        if len(segment) >= 2:
            draw.line(segment, fill=color, width=3, joint="curve")
    draw_legend(draw, plot_box[0] + 8, box[1] + 16, [(label, color) for label, _, _, color in trajectories], font_small)


def save_overview_figure(
    fig_path: Path,
    csv_path: Path,
    rows: Sequence[Dict[str, str]],
    active: Sequence[int],
    metrics: Dict[str, object],
) -> bool:
    if Image is None or ImageDraw is None:
        return False
    fig_path.parent.mkdir(parents=True, exist_ok=True)

    width, height = 1800, 1420
    image = Image.new("RGB", (width, height), (245, 247, 250))
    draw = ImageDraw.Draw(image)
    title_font = get_font(28, bold=True)
    panel_font = get_font(22, bold=True)
    small_font = get_font(16)
    note_font = get_font(18)

    host = metrics.get("host_vehicle")
    heading = f"Trust/rollback overview: {csv_path.name}"
    if host is not None:
        heading += f" (host V{host})"
    draw_text(draw, (40, 28), heading, font=title_font, fill=(18, 30, 45))
    readiness = str(metrics.get("paper_readiness", ""))
    draw_text(draw, (40, 66), readiness[:185], font=note_font, fill=(95, 75, 35))

    times = col_series(rows, "time")
    trust_series = [
        (f"V{vid}", col_series(rows, f"trust_{vid}"), BASE_COLORS[idx % len(BASE_COLORS)])
        for idx, vid in enumerate(active)
    ]
    draw_time_panel(
        draw,
        (40, 112, 1760, 420),
        "Trust assigned to each vehicle",
        times,
        trust_series,
        (0.0, 1.05),
        panel_font,
        small_font,
    )

    weight_series: List[Tuple[str, Sequence[float], Tuple[int, int, int]]] = [
        ("w0", col_series(rows, "w0"), (20, 40, 70)),
        ("w_self", col_series(rows, "w_self"), (80, 80, 80)),
        ("neighbor total", col_series(rows, "total_neighbor_weight"), (255, 127, 14)),
    ]
    for idx, vid in enumerate(active[:5]):
        weight_series.append(
            (
                f"w V{vid}",
                col_series(rows, f"w_neighbor_{vid}"),
                BASE_COLORS[(idx + 2) % len(BASE_COLORS)],
            )
        )
    draw_time_panel(
        draw,
        (40, 448, 1760, 756),
        "Observer weights",
        times,
        weight_series,
        None,
        panel_font,
        small_font,
    )

    event_series = [
        ("attack active", col_series(rows, "v2v_attack_active"), (214, 39, 40)),
        ("rollback", col_series(rows, "rollback_triggered"), (148, 103, 189)),
        ("attack flags", [float(count_positive(row, active, "flag_attack")) for row in rows], (255, 127, 14)),
    ]
    has_event_data = any(finite(value) and value > 0 for _, ys, _ in event_series for value in ys)
    draw_time_panel(
        draw,
        (40, 784, 1760, 1092),
        "Attack, rollback, and flag timeline",
        times,
        event_series,
        None,
        panel_font,
        small_font,
        empty_text="No attack or rollback event recorded in this log",
    )
    if not has_event_data:
        draw_text(
            draw,
            (120, 1038),
            "Current CSVs are useful as clean/baseline runs, but not as rollback evidence.",
            font=note_font,
            fill=(120, 75, 35),
        )

    draw_xy_panel(
        draw,
        (40, 1120, 1760, 1392),
        "Estimated XY trajectories",
        rows,
        active,
        panel_font,
        small_font,
    )

    image.save(fig_path)
    return True


def process_file(
    csv_path: Path,
    output_dir: Path,
    fig_dir: Path,
    threshold: float,
    warmup_s: float,
    make_figures: bool,
) -> Dict[str, object]:
    columns, rows = load_csv(csv_path)
    active = active_vehicle_ids(rows, columns)
    summary = build_summary_rows(rows, columns, active)
    events = build_events(rows, active, threshold)
    metrics = run_metrics(csv_path, rows, columns, active, events, threshold, warmup_s)

    suffix = csv_path.stem.replace("trust_weight_log_", "")
    summary_path = output_dir / f"ai_trust_summary_{suffix}.csv"
    events_path = output_dir / f"ai_trust_events_{suffix}.jsonl"
    metrics_path = output_dir / f"ai_trust_metrics_{suffix}.json"
    write_summary_csv(summary_path, summary)
    write_events(events_path, events)
    write_json(metrics_path, metrics)

    figure_path = None
    if make_figures:
        figure_path = fig_dir / f"qcar_limo_trust_overview_{suffix}.png"
        if save_overview_figure(figure_path, csv_path, rows, active, metrics):
            metrics["figure"] = str(figure_path)
        else:
            metrics["figure"] = None
            metrics["figure_note"] = "Pillow is unavailable; no PNG generated."

    metrics["summary_csv"] = str(summary_path)
    metrics["events_jsonl"] = str(events_path)
    metrics["metrics_json"] = str(metrics_path)
    return metrics


def write_run_metrics_csv(path: Path, metrics_list: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = [
        "source_csv",
        "host_vehicle",
        "samples",
        "duration_s",
        "median_dt_s",
        "active_vehicles",
        "has_attack",
        "attack_active_duration_s",
        "rollback_events",
        "rollback_total_max",
        "min_trust",
        "min_trust_after_warmup",
        "worst_est_pos_err_vehicle",
        "worst_est_pos_err_peak_m",
        "has_reference_error",
        "paper_readiness",
        "summary_csv",
        "events_jsonl",
        "figure",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for metrics in metrics_list:
            row = {}
            for col in columns:
                value = metrics.get(col, "")
                if isinstance(value, (list, tuple)):
                    value = "|".join(str(item) for item in value)
                row[col] = fmt(value)
            writer.writerow(row)


def write_latex_table(path: Path, metrics_list: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "% Auto-generated by paper_trust_analysis.py",
        "\\begin{table}[!t]",
        "\\centering",
        "\\caption{Compact trust-log readiness summary.}",
        "\\label{tab:trust_log_readiness}",
        "\\scriptsize",
        "\\begin{tabular}{lccccc}",
        "\\toprule",
        "\\textbf{Log} & \\textbf{Host} & \\textbf{Samples} & \\textbf{Attack} & \\textbf{Rollback} & \\textbf{Min trust} \\\\",
        "\\midrule",
    ]
    for metrics in metrics_list:
        name = Path(str(metrics.get("source_csv", "log"))).name.replace("_", "\\_")
        host = metrics.get("host_vehicle", "")
        samples = metrics.get("samples", "")
        attack = "Yes" if metrics.get("has_attack") else "No"
        rollback = str(metrics.get("rollback_events", 0))
        min_trust = metrics.get("min_trust_after_warmup", metrics.get("min_trust"))
        trust_text = "--" if min_trust is None else f"{float(min_trust):.3f}"
        lines.append(
            f"{name} & {host} & {samples} & {attack} & {rollback} & {trust_text} \\\\"
        )
    lines.extend(["\\bottomrule", "\\end{tabular}", "\\end{table}", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def default_qcar_root() -> Path:
    return Path(__file__).resolve().parents[2]


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    qcar_root = default_qcar_root()
    parser = argparse.ArgumentParser(
        description="Create AI-friendly trust summaries, metrics, and paper figures."
    )
    parser.add_argument(
        "--input-dir",
        default=str(script_dir),
        help="Directory containing trust_weight_log_V*.csv files.",
    )
    parser.add_argument(
        "--files",
        nargs="*",
        help="Specific CSV files. Defaults to all trust_weight_log_V*.csv in input-dir.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(script_dir / "ai_trust_outputs"),
        help="Directory for compact summary/event/metric files.",
    )
    parser.add_argument(
        "--fig-dir",
        default=str(qcar_root / "fig"),
        help="Directory for generated PNG figures.",
    )
    parser.add_argument(
        "--trust-threshold",
        type=float,
        default=0.5,
        help="Trust threshold used for event extraction.",
    )
    parser.add_argument(
        "--warmup-s",
        type=float,
        default=1.0,
        help="Warmup window excluded from post-warmup summary metrics.",
    )
    parser.add_argument(
        "--no-figures",
        action="store_true",
        help="Only write summary/event/metric files.",
    )
    parser.add_argument(
        "--latex-table",
        default=None,
        help="Optional path for a compact LaTeX table snippet.",
    )
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    fig_dir = Path(args.fig_dir)

    if args.files:
        csv_files = [Path(file) for file in args.files]
    else:
        csv_files = sorted(input_dir.glob("trust_weight_log_V*.csv"))
    if not csv_files:
        raise SystemExit(f"No trust_weight_log_V*.csv files found in {input_dir}")

    metrics_list = []
    for csv_path in csv_files:
        if not csv_path.is_absolute():
            csv_path = (Path.cwd() / csv_path).resolve()
        metrics = process_file(
            csv_path=csv_path,
            output_dir=output_dir,
            fig_dir=fig_dir,
            threshold=args.trust_threshold,
            warmup_s=args.warmup_s,
            make_figures=not args.no_figures,
        )
        metrics_list.append(metrics)
        print(
            f"{csv_path.name}: samples={metrics['samples']}, "
            f"attack={metrics['has_attack']}, rollback_events={metrics['rollback_events']}"
        )

    run_metrics_path = output_dir / "ai_trust_run_metrics.csv"
    write_run_metrics_csv(run_metrics_path, metrics_list)
    write_json(output_dir / "ai_trust_run_metrics.json", {"runs": metrics_list})
    if args.latex_table:
        write_latex_table(Path(args.latex_table), metrics_list)

    print(f"Wrote compact outputs to: {output_dir}")
    if not args.no_figures:
        print(f"Wrote figures to: {fig_dir}")
    print(f"Run metric index: {run_metrics_path}")


if __name__ == "__main__":
    main()
