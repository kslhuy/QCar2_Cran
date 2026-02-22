# QCar Calibration Suite

A set of scripts to **measure, identify, and configure** the QCar's longitudinal and
lateral control parameters.  All scripts run without physical hardware using `--sim`.

---

## Folder Structure

```
Calibration/
├── calibration_utils.py               ← shared helpers (CSV, plotting, fitting)
├── 01_throttle_velocity_calibration.py
├── 02_motor_model_identification.py
├── 03_steering_calibration.py
├── 04_pid_autotuner.py
└── results/                           ← output CSVs, YAMLs, PNG plots
```

---
## Quickstart (QLabs virtual car)
# Step 1: Spawn the virtual car in QLabs
```bash
cd QCar2_multi-vehicle_control
python initPlatoon.py          # num_cars = 1, actor = QC2_0
```
# Step 2: Run calibration against the virtual car
```bash
cd Development/multi_vehicle_self_driving_RealQcar/qcar/Calibration
python 01_throttle_velocity_calibration.py --qlabs --actor QC2_0
python 02_motor_model_identification.py    --qlabs --actor QC2_0 --throttle 0.10
# python 04_pid_autotuner.py                 # reads results/motor_model_id.yaml
```
## Quickstart (Simulation – no hardware needed)

```bash
cd qcar/Calibration

# 1. Map throttle command → steady-state velocity
python 01_throttle_velocity_calibration.py --sim

# 2. Identify first-order motor model (τ, K) from a step response
python 02_motor_model_identification.py --sim --throttle 0.10

# 3. Map steering command → curvature / turning radius
python 03_steering_calibration.py --sim

# 4. Compute PID gains from the identified model
python 04_pid_autotuner.py
```

---

## Script Details

### `01_throttle_velocity_calibration.py` – Throttle → Velocity Map

Steps through throttle levels, waits for steady state, measures velocity.

| Flag | Default | Description |
|------|---------|-------------|
| `--sim` | false | Use simulated motor |
| `--throttle_levels` | `0.02,…,0.20` | Comma-separated throttle steps |
| `--settle_time` | `4.0 s` | Wait per level |
| `--measure_time` | `1.0 s` | Averaging window at steady state |
| `--poly_deg` | `2` | Degree of `v = f(throttle)` polynomial |

**Outputs:** `results/throttle_velocity_map_<tag>.csv`, `.png`, `_poly.yaml`

Use the polynomial coefficients as a **feedforward term** in the PID controller
to reduce the initial velocity error and speed up settling.

---

### `02_motor_model_identification.py` – Motor Model ID

Applies a single throttle step and fits the first-order model:

```
τ · dv/dt + v = K · u     →  v(t) = K·u · (1 − exp(−t/τ))
```

| Flag | Default | Description |
|------|---------|-------------|
| `--sim` | false | Use simulated motor |
| `--throttle` | `0.10` | Step amplitude |
| `--duration` | `8.0 s` | Recording length |

**Outputs:** `results/motor_model_id.yaml`, `step_response_<tag>.csv/.png`

The canonical output `motor_model_id.yaml` is read by script `04`.

---

### `03_steering_calibration.py` – Steering → Curvature Map

Drives at constant speed with each steering level and measures yaw-rate.
Computes `κ = yaw_rate / velocity` and fits the Ackermann model for
effective wheelbase.

| Flag | Default | Description |
|------|---------|-------------|
| `--sim` | false | Use kinematic bicycle model |
| `--steer_levels` | `-0.4…0.4` | Steering command values to test |
| `--speed_throttle` | `0.06` | Constant throttle during test |
| `--speed_sim` | `0.30 m/s` | Simulation velocity |

**Outputs:** `results/steering_curvature_map_<tag>.csv`, `.png`, `steering_calibration_<tag>.yaml`

Use `poly_coefficients` to implement a **curvature → steering lookup** in the path
follower for more accurate lateral control.

---

### `04_pid_autotuner.py` – PID Gain Recommender

Reads `results/motor_model_id.yaml` and computes three sets of PID gains:

| Method | Character |
|--------|-----------|
| Ziegler-Nichols | Fast, may overshoot (~10-20%) |
| **IMC** (recommended) | Smooth, no overshoot, good for real car |
| ITAE | Best disturbance rejection |

Also runs a quick simulation to show predicted overshoot and settling time.

```bash
# Optionally write IMC gains directly to controller config:
python 04_pid_autotuner.py --apply --method IMC
```

**Outputs:** `results/pid_gains_recommendation.yaml`

---

## Calibration Workflow (Hardware)

1. SSH into QCar and upload the `Calibration/` folder (the `calibrate.py` launcher
   in `python/` does this automatically).
2. Run scripts 01 and 02 to measure the motor response.
3. Run script 03 to calibrate steering.
4. Run script 04 to generate PID gains and optionally apply them.
5. Copy the updated config back to the Ground Station.

---

## Dependencies

```
numpy
scipy   (for curve_fit in script 02)
matplotlib   (optional – plots are saved as PNG, not displayed)
pyyaml
```

Install on QCar:
```bash
pip install scipy matplotlib pyyaml
```
