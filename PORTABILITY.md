# Running the project on Windows and Ubuntu

The supported starting point is a fresh **64-bit Python 3.11 environment**.
The package declares Python 3.10+; CI is configured for 3.10–3.13 on Windows
and Ubuntu x86-64. A configured CI matrix is not evidence that those remote
runs have already passed. Local verification for this change used Windows
and Python 3.11.11, with user-site packages disabled.

## Install on a new machine

Create a new environment on each machine. Do not copy `env`, `.venv`, Conda
environments, compiled libraries, TensorRT engines or `node_modules` between
machines.

From the repository root, on Windows PowerShell:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
python -m pip install -e ".[simulation,test]"
python -m qcar doctor --profile simulation
python -m qcar pipeline --duration 1
python -m pytest -q
```

On Ubuntu, using an installed Python 3.11 interpreter with its `venv` support:

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
python -m pip install -e ".[simulation,test]"
python -m qcar doctor --profile simulation
python -m qcar pipeline --duration 1
python -m pytest -q
```

CPU PyTorch supports the simulation without an NVIDIA GPU. For GPU workloads,
choose a build compatible with the target's driver and hardware using the
[official PyTorch selector](https://pytorch.org/get-started/locally/). Jetson
installations may require NVIDIA's matching PyTorch builds; the desktop CUDA
wheel is not a universal ARM/JetPack installation recipe.

Use `python -m pip` so installation and execution use the same interpreter.
`requirements.txt` now lists portable simulation/test dependencies. The nested
requirements file forwards to it. These files install dependencies; installing
the project with `pip install -e .` also makes `qcar` importable.

## Launch without changing directories

After installation, these commands work from any working directory:

```text
python -m qcar doctor --profile simulation
python -m qcar pipeline --duration 12 --closed-loop
python -m qcar workflow quick --smoke
python -m qcar gui --help
python -m qcar fake --help
python -m qcar vehicle --help
```

`qcar` is also installed as a console command. A source checkout can be launched
without installing the package itself, using `python /path/to/run_qcar.py ...`;
its dependencies still need installation. Main direct-file entry points remain
available, including vehicle scripts uploaded into a differently named robot
folder. Upload the root `__init__.py` together with the scripts and subfolders.

Use `from qcar.Observer...`, `from qcar.simulation...`, etc. in integrations.
Do not combine these with `from Observer...` or `from simulation...`. Different
module names can create separate class objects, enums and module-level state.
Library imports should not modify `sys.path` or change the working directory.
Archived research scripts are outside the portable regression suite and still
contain older standalone import patterns.

Packaged configuration defaults resolve from their module locations. Explicit
user paths resolve from the caller's directory; use absolute paths when handing
configurations between processes. `SimulationConfig.load_config()` uses the
packaged default, while `load_config("parameters.yaml")` uses that explicit
filename and fails if it is missing. Text fleet configurations use UTF-8 and
accept a UTF-8 BOM from Windows editors.

The workflow uses the source tree's results directory when launched from a
checkout. A wheel installation writes workflow runs under
`./qcar_runs/workflow`. Set `QCAR_RUNS_DIR` to choose a writable output folder.
Web replay requires a checkout with the frontend; set `QCAR_WORKSPACE` to its
root if the Python package is installed elsewhere. Install frontend dependencies
there with `npm ci`.

## Install only the features needed

| Feature | Install / additional requirement |
| --- | --- |
| Mathematical models and classical observers | `python -m pip install -e .` |
| Headless pipeline and learned observers | `python -m pip install -e ".[simulation]"` |
| Ground Station desktop GUI / bridge | `python -m pip install -e ".[gui]"`; working Tk/Qt display |
| YOLO and camera processing | `python -m pip install -e ".[vision]"`; matching model, camera and driver |
| MPC and online LMI gain design | `python -m pip install -e ".[optimization]"` |
| Plotting / data analysis | `python -m pip install -e ".[analysis]"` |
| Serial adapters | `python -m pip install -e ".[hardware]"` |
| QCar / QLabs and route-map simulation | Matching Quanser SDK and libraries installed separately |
| ROS vehicle entry points | Matching Ubuntu/ROS distribution, sourced ROS/workspace setup |
| Native electronics / Trust core | CMake and a C++17 compiler; build separately on each OS/architecture |

The complete live fake-vehicle stack uses Quanser route maps. The headless
`pipeline` is the SDK-free simulation path. Some existing Limo/LiDAR vision
wrappers explicitly use CUDA and cannot be described as CPU portable.
CARLA also needs a client compatible with its server. Optional dependency
profiles install Python libraries; they do not install these vendor runtimes.

On Ubuntu a GUI installation may also need the distribution's Tk, Qt/XCB,
OpenGL and display-server packages. Headless tests use `MPLBACKEND=Agg` and do
not require a desktop. Remote usernames, ROS workspace paths, IP addresses and
ports remain machine-specific configuration in `fleet_config.yaml`; verify
them for each deployment.

Build the native core from the checkout on either OS:

```text
cmake -S Development/multi_vehicle_self_driving_RealQcar/qcar/electronics/native -B Development/multi_vehicle_self_driving_RealQcar/qcar/electronics/native/build
cmake --build Development/multi_vehicle_self_driving_RealQcar/qcar/electronics/native/build --config Release
python -m qcar.electronics.manual_validation --scenario all
```

`library_path: auto` discovers the current platform's `.dll`, `.so` or `.dylib`,
including CMake `Release`/`Debug` subdirectories. An explicitly selected native
backend still requires a compatible library; missing native code does not
silently select Python authority. Native build products are not bundled into
the Python wheel.

## Diagnose and prevent regressions

`python -m qcar doctor --profile simulation --json` reports the interpreter,
OS, architecture, package versions/locations and required resources. It actually
imports libraries, so a missing DLL or incompatible binary is reported as a
failure. Other profiles are `core`, `gui`, `vision` and `qcar`. It does not open
hardware or validate cameras, SDK licensing, GUI displays or network reachability.

If a Conda environment accidentally uses packages from another installation,
compare `python -s -m qcar doctor --profile simulation` with the normal command.
On Windows the audit found user-site packages masking the environment's own
dependencies. A fresh virtual environment avoids that contamination by default.
The existing `Qcar` environment also failed `pip check`: `cryptography 46.0.2`
requires `cffi>=2.0.0`, while that environment has `cffi 1.17.1`. The repository
changes do not alter installed Conda packages. Use a fresh environment, or
resolve that conflict before relying on SSH/cryptography in the existing one.

The curated default pytest suite covers core observer/control behaviour,
communication/electronics and portability. Portability checks include:

- Foreign working directories and paths containing spaces.
- CLI help without initializing a vehicle.
- Blocked SDK, GUI, vision and hardware modules.
- Exact case of package imports, including on Windows.
- Single canonical module names and no runtime search-path mutation.
- Configuration/checkpoint resources and explicit/default path selection.
- Platform-specific native-library discovery.

[CI](.github/workflows/portability.yml) adds fresh Windows/Ubuntu environments,
CPU PyTorch, native builds and a wheel installed outside the source tree.
The complete hardware stack and older standalone research experiments need
their own integration runs.

Local audit evidence (2026-10-29): the curated suite passed **169 tests and
45 subtests** with user-site packages disabled. All **350** scanned Python
files parsed successfully (excluding the original KalmanNet distribution,
`oldss`, `Liqi_obs` and caches). Additional checks exercised the neural
observer's constructor and shared dynamics model, direct launches from a
renamed upload folder, and an installed wheel's headless workflow outside
the checkout. The neural layer imports also use package context now; they
no longer add their `config_loader` or layer directories to the global import
search path. These checks do not certify every estimator's numerical behaviour
or physical vehicle operation.

For repeatable experiments, retain the application commit, configuration,
checkpoint hash and `python -m pip freeze` from each validated target environment.
Use separate tested lock/constraint files for Windows, Ubuntu and Jetson when
needed. The compatible ranges in `pyproject.toml` are portable dependency
declarations, not a reproducibility lock. Optional profiles follow the
[standard Python packaging model](https://packaging.python.org/en/latest/guides/writing-pyproject-toml/#optional-dependencies).
