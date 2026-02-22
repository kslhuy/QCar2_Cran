# PP_map Tuning Guide for QCar Path Following
## Based on Calibration Scripts 01 / 02 / 03

---

## 1. What the Three Calibration Scripts Produce

| Script | Identifies | Key Output File |
|--------|-----------|----------------|
| `01_throttle_velocity_calibration.py` | Static map: **throttle → steady-state velocity** | `throttle_velocity_poly.yaml` |
| `02_motor_model_identification.py` | Dynamic model: **τ (time constant)** and **K (gain)** | `motor_model_id.yaml` |
| `03_steering_calibration.py` | **Steering command → curvature** + effective wheelbase `L_eff` | `steering_calibration.yaml` |

---

## 2. How Calibration Feeds into `pp_map` (Data Flow)

```
Script 03  →  steering_calibration.yaml
   └─► L_eff_m  →  config_controller_real.yaml: vehicle.wheelbase
                →  PP_Controller(wheelbase=...)  [following_path_state.py line 261]
                →  Steering formula: δ = arctan(2·L·sin(α) / L1)

Script 02  →  motor_model_id.yaml
   └─► IMC gains (Kp, Ki)  →  config_controller_real.yaml: pid.kp / pid.ki
                            →  speed_controller.update(velocity, speed_target, dt)

Script 01  →  throttle_velocity_poly.yaml
   └─► v = polyval(coeffs, throttle)  →  feedforward / inverse map
                                      →  speed controller setpoint tracking
```

---

## 3. Current QLabs Calibration Results (Issues Found)

### Motor model (`motor_model_id_qlabs.yaml`)

```yaml
tau_s: 7.98053            # ⚠️  Unrealistically large — car barely moved in first 5 s
K_mps_per_throttle: 6.22
IMC suggested: Kp=0.32, Ki=0.04, Kd=0.0
```

> **Problem:** `τ ≈ 8 s` is invalid. The step response data was collected when the car was nearly stationary (v ≈ 0 for ~5 s).  
> **Fix:** Re-run with a higher throttle step so the car moves immediately.

```bash
python 02_motor_model_identification.py --qlabs --actor QC2_0 --throttle 0.20 --duration 20
```

---

### Steering calibration (`steering_calibration_qlabs.yaml`)

```yaml
L_eff_m : 0.20983   # effective wheelbase = 21.0 cm
L_nom_m : 0.256     # nominal wheelbase  = 25.6 cm  → 18% error
```

Measured curvature at positive steering (non-monotonic, noisy):

| δ_cmd | κ [1/m] |
|-------|---------|
| +0.1  | +1.277  |
| +0.2  | +0.876  | ← drops! should increase
| +0.3  | +0.503  | ← still dropping
| +0.4  | +0.100  | ← nearly zero

> **Problem:** Curvature is **not monotonic** for positive steering — the data is noisy / run_time too short.  
> **Fix:** Re-run with longer settle time and multiple throttle levels.

```bash
python 03_steering_calibration.py --qlabs --actor QC2_0 \
  --throttle_levels 0.10,0.15,0.20 \
  --steer_levels -0.4,-0.3,-0.2,-0.1,0.1,0.2,0.3,0.4 \
  --run_time 8.0 --settle_time 2.0
```

---

## 4. Critical Bugs to Fix in Code

### Bug 1 — Wrong wheelbase in config (biggest lateral tracking error)

`config_controller_real.yaml`:
```yaml
# BEFORE (wrong):
vehicle:
  wheelbase: 0.256

# AFTER (use calibrated value):
vehicle:
  wheelbase: 0.210    # L_eff_m from steering_calibration_qlabs.yaml
```

The PP steering formula uses this directly. Using 25.6 cm instead of 21 cm → ~20% systematic over-steering on every curve.

---

### Bug 2 — Speed target passed directly as throttle

In `following_path_state.py` around line 397:

```python
# CURRENT (wrong) — PP outputs speed in m/s, but it's used as throttle directly!
u = speed_target

# FIXED — use the PID speed controller (already initialized):
dt_safe = max(float(dt), 1e-3)
u = self.speed_controller.update(velocity, speed_target, dt_safe)
```

Without this fix: a `speed_target = 0.6 m/s` becomes `throttle = 0.6`, which is ~3× too much.

---

### Bug 3 — PID gains not matching calibration

`config_controller_real.yaml`:
```yaml
# BEFORE:
pid:
  kp: 0.3
  ki: 0.2       # ← WAY too high → integral windup
  max_throttle: 0.1  # ← too restrictive for 0.6 m/s cruise

# AFTER (from IMC method in motor_model_id):
pid:
  kp: 0.32
  ki: 0.04      # ← IMC recommendation
  kd: 0.0
  max_throttle: 0.30
```

---

## 5. PP_map Parameter Tuning Table

All parameters are in `config_controller_real.yaml` under the `pp_map:` section.

| Parameter | What it Does | Current | Recommended | Notes |
|-----------|-------------|---------|-------------|-------|
| `vehicle.wheelbase` | PP steering formula geometry | 0.256 | **0.210** | Use `L_eff_m` from calibration |
| `m_l1` | Lookahead slope: `L1 = m_l1·v + q_l1` | 0.35 | **0.45–0.55** | Increase for smoother curves |
| `q_l1` | Minimum lookahead distance [m] | 0.15 | 0.15 | OK |
| `t_clip_min` | Min time-based L1 clip [s] | 0.4 | 0.4 | OK |
| `t_clip_max` | Max time-based L1 clip [s] | 1.8 | 1.8 | OK |
| `lat_err_coeff` | Speed reduction from lateral error | 0.8 | **0.5** | Less aggressive speed drop |
| `kappa_speed_gain` | Speed reduction from curvature | 2.0 | **1.5** | Less braking in gentle curves |
| `hard_turn_kappa` | Curvature threshold for hard turn | 0.85 | 0.85 | OK |
| `hard_turn_speed` | Speed floor in hard turns [m/s] | 0.28 | 0.28–0.32 | Tune up if sluggish |
| `desired_speed` | Target cruise speed [m/s] | 0.6 | 0.6 | OK |
| `start_scale_speed` | Speed scale at start of run | 0.15 | 0.15 | OK |
| `end_scale_speed` | Speed scale after ramp-up | 0.9 | 0.9 | OK |

---

## 6. Recommended Order of Operations

```
Step 1 — Re-run Script 02 at throttle = 0.20
         → get valid τ and IMC gains

Step 2 — Re-run Script 03 at throttle = 0.10, 0.15, 0.20 with 8 s run_time
         → get monotonic, reliable curvature vs steering data

Step 3 — Update config_controller_real.yaml
         vehicle.wheelbase  ← L_eff_m  from steering_calibration.yaml
         pid.ki             ← IMC Ki   from motor_model_id.yaml
         pid.max_throttle   ← 0.30

Step 4 — Fix following_path_state.py line 397
         u = self.speed_controller.update(velocity, speed_target, dt_safe)

Step 5 — Test PP_map and observe log lines:
         "[PP-PATH] v=..., v_cmd=..., throttle=..., steer=..., d=..."
         Watch 'd' (Frenet lateral error) — should converge to ~0
```

---

## 7. How to Monitor PP_map Performance at Runtime

The periodic log (every 200 control cycles) prints:

```
[PP-PATH] v=0.55, v_cmd=0.60, throttle=0.12, steer=-0.08, s=3.21, d=0.04, wp=47
```

| Field | Meaning | Good Sign |
|-------|---------|-----------|
| `v` | Current velocity [m/s] | Close to `v_cmd` |
| `v_cmd` | PP speed target [m/s] | Smooth, not oscillating |
| `throttle` | Motor command sent | Should track `pid.max_throttle` ceiling |
| `steer` | Steering command [−0.5, 0.5] | Smooth, no large spikes |
| `d` | Frenet lateral error [m] | Should converge to **< 0.05 m** |
| `s` | Distance along track [m] | Always increasing |
| `wp` | Nearest waypoint index | Incrementing steadily |
