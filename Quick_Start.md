# Quick start

Use a fresh environment on each machine. Follow [PORTABILITY.md](PORTABILITY.md)
for Windows/Ubuntu setup and CPU/GPU installation. Do not copy an existing
`env` or `.venv` folder between machines.

From the repository root, with the intended environment active:

```text
python -m pip install -e ".[simulation,test]"
python -m qcar doctor --profile simulation
python -m qcar pipeline --duration 1
python -m pytest -q
```

After installation, `python -m qcar ...` works from any directory. A source
checkout can also use `python /path/to/run_qcar.py ...` with dependencies installed.

## SDK-free headless simulation

```text
python -m qcar pipeline --duration 12
python -m qcar pipeline --duration 12 --closed-loop
python -m qcar workflow quick --smoke
```

The workflow evaluates the delivered checkpoint; training again is optional.
See [the workflow guide](ROBUST_KALMANNET_WORKFLOW.md) for training and web replay.
Windows `.cmd`/`.ps1` workflow wrappers remain available; the Python command
works on Windows and Ubuntu.

## Live fleet GUI and vehicle logic

Install the optional GUI libraries, a working desktop display, and the Quanser
libraries needed for route maps. Real vehicles also need their matching SDK,
drivers and fleet/network configuration.

```text
python -m pip install -e ".[gui]"
python -m qcar doctor --profile gui
python -m qcar doctor --profile qcar
python -m qcar gui
```

In separate terminals, start fake vehicles with the real fleet logic:

```text
python -m qcar fake 0
python -m qcar fake 1
python -m qcar fake 2
```

For QCar/QLabs vehicle entry points:

```text
python -m qcar vehicle --car-id 0
python -m qcar vehicle --car-id 1
```

The live fake-vehicle electronics backend is native C++ by default. Build it
as described in [PORTABILITY.md](PORTABILITY.md) before using that mode.
QLabs scene initialization and ROS deployment have additional platform/vendor
requirements; see [the vehicle user guide](Development/multi_vehicle_self_driving_RealQcar/ForUser.md).
