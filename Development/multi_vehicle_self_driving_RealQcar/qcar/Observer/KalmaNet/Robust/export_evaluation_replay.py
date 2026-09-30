"""Export saved benchmark arrays for the Ground Station evaluation replay.

Only the display is downsampled. Metrics are copied from the original JSON.
The web page never runs an estimator or sends commands to a vehicle.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def export_replay(source: Path, output: Path, fps: float = 20.0):
    source, output = Path(source), Path(output)
    if fps <= 0:
        raise ValueError("Display fps must be positive")
    report = json.loads(source.read_text(encoding="utf-8"))
    closed = report["protocol"] == "headless_mockqcar_stanley_pid"
    if not closed and report["protocol"] != "continuous_causal_mockqcar_v1":
        raise ValueError(f"Unsupported protocol: {report['protocol']}")
    dt = float(report["dt"])
    if dt <= 0:
        raise ValueError("Benchmark dt must be positive")
    groups = {}
    for row in report["results"]:
        groups.setdefault((row["seed"], row["scenario"]), []).append(row)
    runs = []
    with np.load(source.with_suffix(".npz"), allow_pickle=False) as arrays:
        for (seed, scenario), rows in groups.items():
            prefix = f"{seed}_{scenario}"
            truth_key = f"{prefix}_{rows[0]['estimator']}_truth" if closed else f"{prefix}_truth"
            n = len(arrays[truth_key])
            if n < 2:
                raise ValueError(f"{prefix}: at least two samples are required")
            if f"{prefix}_attack" in arrays:
                attack = np.asarray(arrays[f"{prefix}_attack"], dtype=bool)
            elif closed:
                # Compatibility with the first published closed-loop benchmark.
                t = np.arange(n) * dt
                attack = ((t >= .25*report["duration"]) & (t < .60*report["duration"])
                          & (scenario != "clean"))
            else:
                raise ValueError(f"Missing attack mask for {prefix}")
            if attack.shape != (n,):
                raise ValueError(f"{prefix}: attack mask does not match the trace")
            transitions = np.flatnonzero(np.diff(attack.astype(int))) + 1
            stride = max(1, round(1 / (fps * dt)))
            indices = np.unique(np.r_[np.arange(0, n, stride), transitions, n-1])
            traces = []

            def trace(name, label, role, key):
                states = arrays[key]
                if states.shape != (n, 4) or not np.isfinite(states).all():
                    raise ValueError(f"Invalid state array: {key}")
                traces.append(dict(name=name, label=label, role=role,
                                   states=np.round(states[indices], 6).tolist()))

            if not closed:
                trace("truth", "Ground truth", "truth", truth_key)
            for row in rows:
                name = row["estimator"]
                label = {"robust": "RobustKLNet", "ekf": "EKF", "legacy": "Legacy GRU",
                         "analytic_ablation": "Analytical ablation"}.get(name, name)
                if closed:
                    trace(name+"_truth", label+" car (truth)", "truth", f"{prefix}_{name}_truth")
                trace(name, label+" estimate", "estimate",
                      f"{prefix}_{name}_estimate" if closed else f"{prefix}_{name}")
            runs.append(dict(seed=seed, scenario=scenario, sample_count=n,
                             time_s=np.round(indices*dt, 8).tolist(),
                             attack=attack[indices].tolist(), traces=traces, metrics=rows))
    payload = dict(format="qcar_evaluation_replay_v1", mode="closed_loop" if closed else "open_loop",
                   source_name=source.name, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                   metadata={k: v for k, v in report.items() if k != "results"},
                   display_fps=fps, runs=runs)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, separators=(",", ":"), allow_nan=False), encoding="utf-8")
    print(f"Replay: {output} ({len(runs)} runs; original metrics retained)", flush=True)
    return payload


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="Benchmark .json with adjacent .npz")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=20.)
    args = parser.parse_args()
    export_replay(args.source, args.output, args.fps)


if __name__ == "__main__":
    main()
