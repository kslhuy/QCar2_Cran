"""One entry point for simulation training, headless evaluation and web replay."""
from __future__ import annotations

import argparse
from datetime import datetime
import importlib.util
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
WEB = ROOT / "GroundStation-Qcar-App"
RUNS = HERE / "results/workflow"
DEFAULT_MODEL = HERE / "models/innovation_trust_sim.npz"
REPLAY = WEB / "public/evaluation/latest.json"
ACTIONS = ("menu", "check", "train", "quick", "full", "web", "closed-loop", "closed-loop-web", "replay")


def execute(script, *args):
    command = [sys.executable, "-u", str(HERE/script), *map(str, args)]
    print("\n> " + subprocess.list2cmdline(command), flush=True)
    subprocess.run(command, cwd=HERE, check=True)


def require_python():
    required = ("numpy", "torch", "scipy", "yaml", "omegaconf", "matplotlib")
    missing = [name for name in required if importlib.util.find_spec(name) is None]
    if missing:
        raise RuntimeError(f"Missing {missing} in {sys.executable}. See ROBUST_KALMANNET_WORKFLOW.md.")


def require_web():
    npm = shutil.which("npm.cmd" if os.name == "nt" else "npm")
    if not npm or not shutil.which("node"):
        raise RuntimeError("Node.js/npm missing. Install Node.js and reopen the terminal.")
    if not (WEB/"node_modules/vite/bin/vite.js").is_file():
        raise RuntimeError(f"Frontend dependencies missing. Run: cd \"{WEB}\" ; npm ci")
    return npm


def remember(name, path):
    RUNS.mkdir(parents=True, exist_ok=True)
    (RUNS/f"latest_{name}.json").write_text(json.dumps({"path": str(path.resolve())}, indent=2), encoding="utf-8")


def latest(name, fallback):
    pointer = RUNS/f"latest_{name}.json"
    return Path(json.loads(pointer.read_text(encoding="utf-8"))["path"]) if pointer.exists() else fallback


def model_path(value):
    path = latest("training", DEFAULT_MODEL) if value == "latest" else Path(value) if value else DEFAULT_MODEL
    if not path.is_file():
        raise FileNotFoundError(f"Checkpoint missing: {path}")
    return path.resolve()


def new_run(label):
    path = RUNS / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_" + label)
    path.mkdir(parents=True)
    return path


def print_scores(path):
    report = json.loads(path.read_text(encoding="utf-8"))
    print("\nPosition RMSE (m), pooled over equal-length trials:")
    print(f"{'Scenario':18} {'EKF':>10} {'RobustKLNet':>13}")
    for scenario in dict.fromkeys(r["scenario"] for r in report["results"]):
        scores = []
        for estimator in ("ekf", "robust"):
            values = [r["position_rmse_m"] for r in report["results"]
                      if r["scenario"] == scenario and r["estimator"] == estimator]
            scores.append((sum(v*v for v in values)/len(values))**.5)
        print(f"{scenario:18} {scores[0]:10.4f} {scores[1]:13.4f}")
    print(f"\nMetrics: {path}\nTrajectories: {path.with_suffix('.npz')}")


def serve(source, port, no_browser):
    require_web()
    from export_evaluation_replay import export_replay
    export_replay(source, REPLAY)
    # Do not silently switch ports or attach to an unrelated server.
    with socket.socket() as probe:
        if probe.connect_ex(("127.0.0.1", port)) == 0:
            raise RuntimeError(f"Port {port} is in use. Stop that server or use -Port {port+1}.")
    url = f"http://127.0.0.1:{port}/?evaluation=1"
    print(f"\nEvaluation replay: {url}\nCtrl+C stops this web server. No bridge is required.", flush=True)
    command = [shutil.which("node"), str(WEB/"node_modules/vite/bin/vite.js"),
               "--host", "127.0.0.1", "--port", str(port), "--strictPort"]
    if not no_browser:
        command += ["--open", "/?evaluation=1"]
    subprocess.run(command, cwd=WEB, check=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", nargs="?", default="menu", choices=ACTIONS)
    parser.add_argument("--checkpoint", default="", help=".npz path, or latest (last non-smoke training)")
    parser.add_argument("--results", type=Path, help="Existing benchmark JSON for replay")
    parser.add_argument("--port", type=int, default=3000)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--smoke", action="store_true", help="Small plumbing check; not a performance benchmark")
    args = parser.parse_args(argv)
    if args.action == "menu":
        print("\nRobustKLNet | simulation workflow\n")
        choices = {"1": "check", "2": "quick", "3": "web", "4": "full",
                   "5": "closed-loop", "6": "closed-loop-web", "7": "replay", "8": "train"}
        for key, label in [("1", "Check setup"), ("2", "Quick evaluation - no animation"),
                           ("3", "Quick evaluation + web animation"), ("4", "Full evaluation - no animation"),
                           ("5", "Closed-loop driving test - no animation"),
                           ("6", "Closed-loop driving test + web animation"),
                           ("7", "Replay latest result (no new evaluation)"),
                           ("8", "Train a new checkpoint (keep delivered model)")]:
            print(f"  {key}. {label}")
        print("  0. Exit")
        answer = input("\nChoose: ").strip()
        if answer == "0":
            return
        if answer not in choices:
            raise ValueError("Choose one of the menu numbers")
        args.action = choices[answer]
    require_python()
    print(f"Python: {sys.executable}", flush=True)
    if args.action == "check":
        print(f"Python dependencies: OK\nDelivered checkpoint: {model_path('')}")
        require_web()
        print(f"Frontend dependencies: OK\nGuide: {ROOT/'ROBUST_KALMANNET_WORKFLOW.md'}")
        return
    if args.action in ("web", "replay", "closed-loop-web"):
        require_web()  # Fail before spending time on an evaluation.
    if args.action == "train":
        run = new_run("training_smoke" if args.smoke else "training")
        output = run/"innovation_trust.npz"
        options = ["--epochs", 2 if args.smoke else 60, "--rounds", 1 if args.smoke else 2,
                   "--duration", 3 if args.smoke else 30, "--output", output,
                   "--cache", run/"training_cache.npz"]
        if args.smoke:
            options += ["--train-seeds", 1, "--val-seeds", 21]
        execute("train_robust_kalmannet.py", "--simulation-trust", *options)
        if not args.smoke:
            remember("training", output)
        print(f"\nNew checkpoint: {output}\nTraining history: {output.with_suffix('.history.json')}")
        print("Next: .\\robust_workflow.cmd quick -Checkpoint latest" if not args.smoke else
              "Smoke training only. It has not been selected as the latest trained model.")
        return
    if args.action == "replay":
        source = args.results or latest("evaluation", HERE/"results/heldout.json")
    else:
        checkpoint = model_path(args.checkpoint)
        run = new_run(args.action + ("_smoke" if args.smoke else ""))
        source = run/"metrics.json"
        print(f"Checkpoint: {checkpoint}\nRun folder: {run}", flush=True)
        if args.action in ("closed-loop", "closed-loop-web"):
            options = ["--checkpoint", checkpoint, "--output", source]
            if args.smoke:
                options += ["--seeds", 401, "--duration", 4, "--scenarios", "clean", "gps_jump"]
            execute("closed_loop_benchmark.py", *options)
        else:
            options = ["--checkpoint", checkpoint, "--output", source]
            if args.smoke:
                options += ["--duration", 2, "--seeds", 101, "--scenarios", "clean", "gps_jump"]
            elif args.action == "full":
                options += ["--duration", 60, "--seeds", 101, 102, 103, "--legacy"]
            else:
                options += ["--duration", 15, "--seeds", 101, "--scenarios", "clean", "gps_jump",
                            "gps_ramp", "wheel_freeze", "gps_wheel"]
            execute("validate_robust_kalmannet.py", "--simulation", *options)
        remember("evaluation", source)
        print_scores(source)
    if args.action in ("web", "replay", "closed-loop-web"):
        serve(source, args.port, args.no_browser)
    else:
        print("\nView this result: .\\robust_workflow.cmd replay")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
        sys.exit(130)
    except (RuntimeError, ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        sys.exit(1)
