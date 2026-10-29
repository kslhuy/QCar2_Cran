"""Check this interpreter's dependencies and packaged resources without starting a vehicle."""
from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path
import platform
import site
import sys

CORE = ("numpy", "scipy", "yaml", "omegaconf", "msgpack", "zmq")
PROFILES = {
    "core": CORE,
    "simulation": CORE + ("torch", "matplotlib"),
    "gui": CORE + ("tkinter", "websockets", "paramiko", "scp", "pygame", "PyQt6", "pyqtgraph"),
    "vision": CORE + ("cv2", "torch", "ultralytics"),
    "qcar": CORE + ("pal.products.qcar", "hal.products.mats"),
}


def diagnose(profile="simulation"):
    """Actually import dependencies, catching missing modules and binary-load errors."""
    if profile not in PROFILES:
        raise ValueError(f"Unknown profile: {profile}")
    checks = []
    modules = list(PROFILES[profile])
    if profile in {"core", "simulation"}:
        modules += ["qcar.simulation.mock_vehicle", "qcar.Observer.VehicleObserverSimple"]
    if profile == "simulation":
        modules += ["qcar.simulation.observer_pipeline"]
    for name in modules:
        try:
            module = importlib.import_module(name)
            checks.append({"name": name, "ok": True,
                           "location": getattr(module, "__file__", None),
                           "version": str(getattr(module, "__version__", ""))})
        except Exception as exc:
            checks.append({"name": name, "ok": False,
                           "error": f"{type(exc).__name__}: {exc}"})
    root = Path(__file__).resolve().parent
    resources = (
        "fleet_config.yaml", "simulation/parameters.yaml",
        "simulation/vehiclemodels/parameters/parameters_qcar.yaml",
        "Observer/config_local_estimators.yaml", "Observer/config_fleet_estimators.yaml",
        "Observer/config_relative_estimators.yaml", "Controller/config_controller_sim.yaml",
    )
    if profile == "simulation":
        resources += ("Observer/KalmaNet/Robust/models/innovation_trust_sim.npz",)
    for resource in resources:
        checks.append({"name": resource, "ok": (root / resource).is_file(),
                       "location": str(root / resource)})
    return {"ok": all(check["ok"] for check in checks), "profile": profile,
            "python": sys.executable, "python_version": platform.python_version(),
            "platform": platform.platform(), "machine": platform.machine(),
            "user_site_enabled": site.ENABLE_USER_SITE, "checks": checks}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=PROFILES, default="simulation")
    parser.add_argument("--json", action="store_true", help="Print a machine-readable report")
    args = parser.parse_args(argv)
    # Some third-party imports print banners. Keep JSON output parseable.
    from contextlib import redirect_stdout
    with redirect_stdout(sys.stderr):
        report = diagnose(args.profile)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"Python: {report['python']} ({report['python_version']})")
        print(f"Platform: {report['platform']} / {report['machine']}")
        print(f"User-site packages enabled: {report['user_site_enabled']}")
        for check in report["checks"]:
            detail = check.get("error", check.get("location", ""))
            print(f"{'OK' if check['ok'] else 'FAIL'} {check['name']}: {detail}")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
