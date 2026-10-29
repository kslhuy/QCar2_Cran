"""Regression checks for launch directories, clean machines and Linux path casing."""
import ast
import json
from pathlib import Path
import subprocess
import sys
import shutil

import pytest
import qcar

REPO = Path(__file__).resolve().parents[2]
SOURCE = REPO / "Development" / "multi_vehicle_self_driving_RealQcar"
PACKAGE = SOURCE / "qcar"


def run_isolated(code, cwd, *args):
    return subprocess.run([sys.executable, "-I", "-c", code, str(SOURCE), *args],
                          cwd=cwd, text=True, capture_output=True, timeout=120)


def test_package_import_does_not_initialize_runtime(tmp_path):
    code = """
import sys
sys.path.insert(0, sys.argv[1])
before = list(sys.path)
import qcar
assert sys.path == before
assert not any(n in sys.modules for n in ('torch', 'pal', 'hal', 'tkinter', 'numpy'))
"""
    result = run_isolated(code, tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr


def test_vehicle_help_after_upload_to_renamed_package(tmp_path):
    uploaded = tmp_path / "robot code é" / "machine_bundle"
    uploaded.mkdir(parents=True)
    for name in ("__init__.py", "vehicle_main.py", "config_main.py"):
        shutil.copy2(PACKAGE / name, uploaded / name)
    result = subprocess.run([sys.executable, "-I", str(uploaded / "vehicle_main.py"), "--help"],
                            cwd=tmp_path, text=True, capture_output=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    # ROS-style entry points can import the uploaded folder under their own namespace.
    code = """
import pathlib, runpy, sys
root = pathlib.Path(sys.argv[1])
sys.path.insert(0, str(root.parent))
sys.argv = ['robot', '--help']
try:
    runpy.run_module('machine_bundle.vehicle_main', run_name='__main__')
except SystemExit as exc:
    assert exc.code == 0
import qcar
assert pathlib.Path(qcar.__file__).parent == root
"""
    result = subprocess.run([sys.executable, "-I", "-c", code, str(uploaded)],
                            cwd=tmp_path, text=True, capture_output=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr


def test_simulation_without_sdk_gui_or_vision(tmp_path):
    code = """
import importlib.abc
import importlib.util
import sys
sys.path.insert(0, sys.argv[1])
blocked = {'pal', 'hal', 'qvl', 'pit', 'cv2', 'tkinter', 'PyQt6', 'pyqtgraph',
           'pygame', 'paramiko', 'scp', 'casadi', 'rclpy', 'tensorrt'}
class Block(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in blocked:
            raise ModuleNotFoundError('Unavailable on this test machine: ' + fullname, name=fullname)
sys.meta_path.insert(0, Block())
original_find_spec = importlib.util.find_spec
def find_spec(name, *args, **kwargs):
    if name.split('.')[0] in blocked:
        return None
    return original_find_spec(name, *args, **kwargs)
importlib.util.find_spec = find_spec
before = list(sys.path)
from qcar.portability import diagnose
report = diagnose('simulation')
assert report['ok'], report
from qcar.simulation.config import SimulationConfig
from qcar.simulation.mock_vehicle import MockQCar
from qcar.Observer.KalmaNet.Robust.simulation_benchmark import generate_drive
from qcar.vehicle_logic import VehicleLogic
from qcar.simulation.fake_vehicle_real_logic import FakeVehicleWithRealLogic
blocked.add('cvxpy')
from qcar.Observer.LocalNeuralObs import NeuralLuenbergerEstimator
from qcar.Observer.local_state_estimators import LocalStateEstimatorBase
assert issubclass(NeuralLuenbergerEstimator, LocalStateEstimatorBase)
neural = NeuralLuenbergerEstimator(config={'gain_design_method': 'default',
    'first_layer_type': 'qlpv_kalman', 'load_pretrained': False, 'learning_mode': 'none'})
assert neural.first_layer_observer is not None
assert neural.dynamics is neural.first_layer_observer.dynamics
for name in ('qlpv_observer', 'qlpv_observer_kalma', 'differentiator_uio_observer',
             'differentiator_uio_ekf', 'z_layer1_observer', 'huy_uio_observer'):
    importlib.import_module('qcar.Observer.LocalNeuralObs.1LayerObs.' + name)
car = MockQCar(SimulationConfig.load_config())
car.step(.01)
assert sys.path == before, sys.path
assert not any(n in sys.modules for n in ('Observer', 'simulation', 'electronics', 'StateMachine', 'LaneFollow'))
assert not any(n in sys.modules for n in ('neural_network', 'config_loader', 'firstLayerObserverBase', 'Design_LMI_neural'))
"""
    result = run_isolated(code, tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("command", [None, "vehicle", "fake", "pipeline", "batch", "gui", "workflow", "doctor"])
def test_cli_help_from_other_directory(command, tmp_path):
    args = [sys.executable, "-s", str(REPO / "run_qcar.py")]
    if command:
        args.append(command)
    result = subprocess.run([*args, "--help"], cwd=tmp_path,
                            text=True, capture_output=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "usage:" in result.stdout.lower()


def test_all_qualified_imports_exist_with_exact_casing():
    errors = []
    local_roots = {p.stem for p in PACKAGE.glob("*.py")} | {
        p.name for p in PACKAGE.iterdir() if p.is_dir() and p.name.isidentifier()
    }
    for file in PACKAGE.rglob("*.py"):
        if {"Origine", "Liqi_obs", "oldss", "__pycache__"}.intersection(file.parts):
            continue
        tree = ast.parse(file.read_text(encoding="utf-8-sig"), filename=str(file))
        for node in ast.walk(tree):
            if not isinstance(node, ast.ImportFrom) or node.level or not node.module:
                continue
            if not node.module.startswith("qcar."):
                if node.module.split(".")[0] in local_roots and node.module != "qcar":
                    errors.append(f"{file.relative_to(PACKAGE)}:{node.lineno}: unqualified {node.module}")
                continue
            path = PACKAGE
            for part in node.module.split(".")[1:]:
                entries = {p.name: p for p in path.iterdir()} if path.is_dir() else {}
                if part in entries:
                    path = entries[part]
                elif part + ".py" in entries:
                    path = entries[part + ".py"]
                else:
                    errors.append(f"{file.relative_to(PACKAGE)}:{node.lineno}: {node.module}")
                    break
    assert not errors, "Missing or incorrectly cased imports:\n" + "\n".join(errors)


def test_default_resources_are_in_package():
    from qcar.portability import diagnose
    report = diagnose("simulation")
    assert report["ok"], json.dumps(report, indent=2)


def test_explicit_simulation_config_is_not_shadowed_by_default(tmp_path, monkeypatch):
    from qcar.simulation.config import SimulationConfig
    monkeypatch.chdir(tmp_path)
    (tmp_path / "parameters.yaml").write_text("vehicle: {id: 873}\n", encoding="utf-8")
    assert SimulationConfig.load_config()["vehicle"]["id"] != 873
    assert SimulationConfig.load_config("parameters.yaml")["vehicle"]["id"] == 873


def test_doctor_reports_binary_import_failures(monkeypatch):
    from qcar import portability
    original = portability.importlib.import_module
    def broken(name):
        if name == "numpy":
            raise OSError("binary architecture mismatch")
        return original(name)
    monkeypatch.setattr(portability.importlib, "import_module", broken)
    report = portability.diagnose("core")
    assert not report["ok"]
    assert "OSError: binary architecture mismatch" in report["checks"][0]["error"]


@pytest.mark.parametrize("platform_name,suffix", [("win32", ".dll"), ("linux", ".so"), ("darwin", ".dylib")])
def test_native_discovery_uses_target_platform(monkeypatch, platform_name, suffix):
    from qcar.electronics import trust_observer_native
    monkeypatch.setattr(sys, "platform", platform_name)
    candidates = list(trust_observer_native._native_library_candidates(None))
    assert candidates and all(path.suffix == suffix for path in candidates)
