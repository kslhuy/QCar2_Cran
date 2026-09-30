# Paper Experiment To Do List

## 0. Logging Setup

- [ ] In `fleet_config.yaml`, enable:
  - `logging.enable_trust_weight_logging: true`
  - `logging.enable_following_leader_logging: true`
  - `logging.enable_state_logging: true`
- [ ] Keep the same attack, duration, vehicles, path, and start timing for every comparison.
- [ ] Use one folder per run:
  - `paper_runs/rollback_off/`
  - `paper_runs/rollback_on/`
  - `paper_runs/controller_A_cacc_attacked/`
  - `paper_runs/controller_B_trust_cacc/`
  - `paper_runs/controller_C_trust_fusion/`
  - `paper_runs/controller_D_local_acc/`

## 1. Run System

Start GUI / ground station:

```powershell
cd Development\multi_vehicle_self_driving_RealQcar\qcar
python GUI\app_main.py --cars 3 --ws-port 8080
```

Trigger the hardest attack:

```powershell
cd Development\multi_vehicle_self_driving_RealQcar\qcar
python simulation\timed_v2v_attack.py --attack-type Mix_test --case-num 2 --data-type fleet --attacker-id 0 --victim-ids -1 --attack-delay 9.5 --duration 7
```

This is the paper-style global/fleet position fault: intermittent X-position noise, sigma = 10 m, probability = 0.5.

## 2. Rollback Validation

Main claim: rollback reduces delayed contamination.

Run the same attack twice.

### Run R0: No Rollback

- [ ] Set in `Observer/TrustbasedDistributedObserver/config_trust_estimator.yaml`:

```yaml
observer:
  rollback_enabled: false
```

- [ ] Run system.
- [ ] Trigger attack.
- [ ] Save trust logs and controller logs as `rollback_off`.

### Run R1: Rollback Enabled

- [ ] Set:

```yaml
observer:
  rollback_enabled: true
```

- [ ] Run exactly the same attack.
- [ ] Save logs as `rollback_on`.

### Rollback Metrics

- [ ] Report:
  - `detection_delay = t_trust_below_threshold - t_attack_start`
  - `rollback_latency = t_rollback - t_attack_start`
  - `peak_error_before_rollback`
  - `peak_error_after_rollback`
  - `error_reduction_% = 100 * (1 - rollback_error / no_rollback_error)`
  - `source_weight_drop_%`
  - `number_of_rollbacks`

### Rollback Figure

Use:

```powershell
cd Development\multi_vehicle_self_driving_RealQcar\qcar\Observer\TrustbasedDistributedObserver
python plot_trust_data.py --file ".\trust_weight_log_V0.csv" --focus 4 --motion-test-start 9.5 --motion-test-seconds 7 --motion-test-export
```

Best figure for paper:

- [ ] Position estimation error: no rollback vs rollback.
- [ ] Shade attack interval `[9.5, 16.5] s`.
- [ ] Add vertical line for detection.
- [ ] Add vertical line for rollback trigger.
- [ ] Include small XY trajectory inset near attack.

Paper comment:

> Rollback reduces the post-detection error peak by replaying recent observer history after removing contaminated fleet-state updates. The benefit is strongest around the most affected vehicle, where the no-rollback estimate remains biased after trust degradation.

## 3. Controller Validation

Main claim: trust-aware estimation helps platoon control preserve spacing under attack.

Run these four cases.

### A. Attacked V2V / Normal CACC

```yaml
leader_longitudinal_state_source:
  mode: direct_v2v_attacked

trust_longitudinal_fusion:
  enabled: false
```

### B. Trust Estimator + CACC Only

```yaml
leader_longitudinal_state_source:
  mode: fleet_estimator

trust_longitudinal_fusion:
  enabled: false
```

### C. Trust Estimator + ACC/IDM Fallback Fusion

```yaml
leader_longitudinal_state_source:
  mode: fleet_estimator

trust_longitudinal_fusion:
  enabled: true
```

### D. Local-Only ACC/IDM Baseline

```yaml
leader_sensor_acc:
  enabled: true

trust_longitudinal_fusion:
  enabled: false
```

Verify from logs that the leader source / policy is local sensor based.

## 4. Controller Plots and Metrics

List logs:

```powershell
cd Development\multi_vehicle_self_driving_RealQcar\qcar
python Controller\plot_following_leader_control.py --dir ".\logs" --list
```

Generate figure and metrics:

```powershell
python Controller\plot_following_leader_control.py --dir ".\logs" --pick 0 --save ".\paper_outputs\fig_spacing_control.png" --metrics-csv ".\paper_outputs\metrics_controller.csv" --no-show --trust-low 0.40 --trust-high 0.70 --desired-distance 0.25 --time-headway 0.45
```

Best metrics:

- [ ] `s_min`
- [ ] `spacing_RMSE`
- [ ] `max_spacing_error`
- [ ] `max_acceleration`
- [ ] `max_jerk`
- [ ] `minimum_trust_gamma`
- [ ] `time_in_low_trust`
- [ ] `collision_flag = s_min <= 0`

Best controller figure:

- [ ] Projected inter-vehicle spacing with safety line at `0`.
- [ ] Trust/fusion signal `gamma_i(t)` or `alpha(t)`.
- [ ] `u_loc`, `u_coop`, and final `u_i`.
- [ ] Estimation error during attack.

Paper comment:

> Under the same global position fault, direct CACC is sensitive to corrupted cooperative states, while trust-estimator CACC reduces the corrupted input. The trust-fusion controller gives the safest behavior because low trust shifts authority toward local ACC/IDM, preserving positive projected spacing.

## 5. Final Paper Output

Use only:

- [ ] Figure 1: rollback comparison.
- [ ] Figure 2: spacing/control comparison.
- [ ] Table 1: rollback and controller metrics.

Do not add many figures. The strongest story is:

```text
Worst attack case
No rollback vs rollback
Then same attack with CACC-only vs trust-fusion controller
```
