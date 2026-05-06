# QCar Calibration



This folder contains two different calibration workflows:

1. Passive online calibration over ZMQ
2. Offline scripted calibration runs such as `05_throttle_acceleration_calibration.py`

The two workflows solve different problems. Use the online path when you want to
collect data while driving normally. Use the offline scripts when you want a
controlled experiment with repeatable step inputs.

Assume the commands below are run from:

```powershell
Development/multi_vehicle_self_driving_RealQcar/qcar
```

## Passive Online Calibration



### What it does

The online calibration path streams samples from the vehicle process to an
external ZMQ worker.

Sample format:

```text
[v, throttle, steering, yaw_rate, ax, ay, az, x, y, theta, a_ref, v_raw]
```

Notes:

- `v` is the observer / estimator velocity reference.
- `v_raw` is the raw motor-tach velocity saved for offline identification.
- `a_ref` is the observer acceleration reference saved for comparison or blended
  offline targets.

Supported passive analyses in `online_calibration_service.py`:

- `throttle_velocity`
- `steering_curvature`
- `throttle_acceleration`
- `coupled_kinematic`

Important: the online `throttle_acceleration` analysis is still the lightweight
passive step detector implemented in `online_calibration_service.py`. It is not
the same as the newer offline acceleration-lag fit in
`05_throttle_acceleration_calibration.py`.

### Architecture

```text
vehicle_logic
  -> OnlineCalibrationZMQClient
  -> publishes samples and commands
  -> online_calibration_zmq_worker.py
  -> OnlineCalibrationService
  -> saves YAML and raw CSV results
```

Default ports:

- sample stream: `18890`
- control stream: `18891`
- worker status PUB: `18892`

### Start the worker

Run this in a separate terminal on the machine that will receive the streamed
samples:

```powershell
python Calibration/online_calibration_zmq_worker.py
```

You can override ports if needed:

```powershell
python Calibration/online_calibration_zmq_worker.py --sample-port 18890 --control-port 18891 --status-port 18892
```

`pyzmq` must be installed for the worker and the vehicle-side client.

### Start collection from the GUI

The Ground Station GUI already sends the correct commands.

Workflow:

1. Start the ZMQ worker.
2. Start the vehicle and Ground Station as usual.
3. In the Calibration panel for the target car, click `Collect`.
4. Drive normally and cover the operating range you care about.
5. Click `Pause` when you have enough data.
6. Click `Analyse` for the desired calibration type.
7. Use `Clear` before a new run if you want a fresh buffer.

GUI actions map to these commands:

- start collection: `enable_online_calibration`
- stop collection: `disable_online_calibration`
- clear buffer: `set_params` with category `online_calibration`, action `clear`
- analyse: `set_params` with category `online_calibration`, action `analyse`


### Where the online results are saved

The worker writes results under:

```text
Calibration/results/online_<calibration_type>/
```

Examples:

- `Calibration/results/online_throttle_velocity/`
- `Calibration/results/online_steering_curvature/`
- `Calibration/results/online_throttle_acceleration/`
- `Calibration/results/online_coupled_kinematic/`

Each run typically saves:

- `<calibration_type>_<timestamp>.yaml`
- `<calibration_type>_latest.yaml`
- `raw_samples_<timestamp>.csv`

### When to use each online analysis

`throttle_velocity`

- Best when you have long dwell periods at roughly constant throttle.
- Produces a steady-state map `v_ss = f(throttle)`.

`steering_curvature`

- Best when you drive smooth constant-radius turns with nonzero speed.
- Produces a curvature map and an effective wheelbase estimate.

`throttle_acceleration`

- Best when your passive drive log contains clear throttle step changes.
- Produces a simple first-order step summary from buffered samples.
- Good for quick inspection, but not the preferred path if you need the
  controller-facing `acc_to_throttle_gain`.

`coupled_kinematic`

- Best when the passive drive log covers both straight and turning motion.
- Uses filtered online references for `v`, `a`, `yaw_rate`, `x`, `y`, and `theta`.
- Fits a coupled motion model where acceleration and yaw rate both depend on
  velocity, throttle, and steering.
- Exports an observer-model YAML that can patch the local EKF / Robust
  KalmanNet analytical predictor.

## Offline Acceleration-Lag Calibration: `05_throttle_acceleration_calibration.py`

### What this script estimates

`05_throttle_acceleration_calibration.py` now fits the acceleration-lag model:

```text
a_dot = -(1/tau) * a + (input_gain/tau) * u
v_dot = a
```

For each tested throttle transition it estimates:

- `tau_s`
- `a_pre_mps2`
- `a_ss_mps2`
- `delta_a_mps2`
- `input_gain_mps2_per_throttle`
- `acc_to_throttle_gain`
- `t63_s`, `t90_s`, `t95_s`
- `lead_time_s`

The YAML also includes a `recommended_parameters` block with averaged values.

Use this script when you want parameters for:

```text
throttle ~= desired_accel * acc_to_throttle_gain
```

### Recommended workflow

For this script, controlled step tests matter. Do not treat it like passive
background logging.

Recommended procedure:

1. Put the vehicle on a straight, safe, open section.
2. Start with small throttle levels.
3. Use IMU acceleration if available.
4. Run one full step-test sequence.
5. Inspect the generated YAML and raw CSV.
6. Repeat with different level ranges if needed.

### Example commands

Simulation smoke test:

```powershell
python Calibration/05_throttle_acceleration_calibration.py --sim --throttle_levels 0.0,0.1,0.2 --step_time 2.0 --pre_hold 1.0 --settle_time 1.0
```

QLabs run:

```powershell
python Calibration/05_throttle_acceleration_calibration.py --qlabs --actor QC2_0 --accel_source tach --throttle_levels 0.0,0.1,0.2,0.3
```

Physical QCar run using IMU acceleration:

```powershell
python Calibration/05_throttle_acceleration_calibration.py --accel_source imu --imu_axis 0 --imu_sign 1.0 --imu_remove_bias --throttle_levels 0.0,0.1,0.2,0.3
```

If the car is sensitive, reduce the tested levels:

```powershell
python Calibration/05_throttle_acceleration_calibration.py --accel_source imu --imu_axis 0 --imu_sign 1.0 --imu_remove_bias --throttle_levels 0.0,0.06,0.10,0.14,0.18 --max_throttle 0.18
```

Useful options:

- `--pre_hold`: hold the initial throttle before the step
- `--step_time`: duration of the stepped input
- `--settle_time`: time allowed to settle before each new transition
- `--accel_alpha`: low-pass smoothing on measured acceleration
- `--all_pairs`: test all throttle pairs instead of only adjacent steps
- `--no_down`: skip descending steps
- `--tag`: append a custom tag to output filenames
- `--no_plot`: skip the end-of-run plot

### Where the `05` results are saved

Outputs are written to:

```text
Calibration/results/05_throttle_acceleration_calibration/
```

Important files:

- `throttle_accel_step_raw_<tag>.csv`
- `throttle_accel_lookup_<tag>.csv`
- `throttle_accel_model_<tag>.yaml`
- `throttle_accel_model.yaml`

### How to use the fitted parameters

Open the generated YAML and look at:

- `recommended_parameters.avg_tau_s`
- `recommended_parameters.avg_input_gain_mps2_per_throttle`
- `recommended_parameters.avg_acc_to_throttle_gain`

For controller tuning:

- use `avg_acc_to_throttle_gain` if you want a single global
  `acc_to_throttle_gain`
- use the per-transition lookup table if the gain changes significantly across
  throttle ranges
- use `avg_tau_s` if you want a first-order actuator model in an observer or a
  future controller update

### Practical notes

- IMU acceleration is preferred over tach-derived acceleration when available.
- Very long step times can make drag dominate the late part of the signal.
- Very short step times can make `a_ss` noisy.
- If feedforward is enabled in the controller, calibrate with that effect in
  mind or disable it during dedicated experiments.
- For repeatability, keep the road flat and avoid steering during the run.

## Summary

Use passive online calibration when you want quick maps from normal driving.

Use `05_throttle_acceleration_calibration.py` when you need a controlled
acceleration-lag model with `tau` and `acc_to_throttle_gain` that can be used
directly in longitudinal controller or observer design.
