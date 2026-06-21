# Trust-Based Distributed Observer

This package implements a trust-aware fleet observer for the QCar multi-vehicle
stack. It combines the TriP trust model, trust-weighted consensus updates,
optional prediction-only isolation, and an optional contamination rollback pass.

The current implementation is centered on these files:

| File | Role |
| --- | --- |
| `trust_based_fleet_estimator.py` | Main `trust_consensus` estimator and factory entry point. |
| `trust_based_kalman_estimator.py` | `trust_kalman` variant with trust-scaled measurement noise. |
| `trust_model.py` | TriP trust scoring, Dirichlet trust evolution, attack flags, generalized trust vector. |
| `weight_trust_module.py` | Global diagnostic weights and per-target fusion weights. |
| `contamination_rollback.py` | Trust-triggered replay that removes newly malicious source contributions. |
| `config_trust_estimator.yaml` | Trust observer defaults merged into the fleet observer config. |

## Current Setup

`config_trust_estimator.yaml` contains the trust observer defaults. When the full
`VehicleObserver` is used, this child config is merged into
`Observer/config_fleet_estimators.yaml`:

- `trust` stays nested under the selected trust estimator config.
- `weight` stays nested under the selected trust estimator config.
- `vehicle` stays nested and is also merged with vehicle geometry overrides.
- `observer` keys are flattened into the selected estimator config.
- `observer.kalman` is flattened into the `trust_kalman` config.

Important: `Observer/config_fleet_estimators.yaml` is the source of truth for the
active fleet estimator type. The child file has:

```yaml
fleet_estimator_type: trust_consensus
```

but this only becomes active if the parent file does not already select another
fleet estimator. In the current repo, the parent file currently selects
`distributed_high_gain`, so the trust framework is not the active
`VehicleObserver` fleet estimator unless that value is changed. To run this
framework through `VehicleObserver`, set the parent top-level
`fleet_estimator_type` to one of:

```yaml
fleet_estimator_type: trust_consensus
# or
fleet_estimator_type: trust_kalman
```

The current trust observer defaults are:

| Area | Current default |
| --- | --- |
| Estimator | `trust_consensus` in the child config |
| Trust threshold | `trust.trust_threshold: 0.5` |
| Trust method | `dirichlet_method: matlab`, `dirichlet_type: Dual` |
| Generalized trust | `use_generalized_trust_vector: true` |
| Weight mode | `weight.weight_type: trust_based` |
| Direct measurement weight | `weight.w0_fixed: 0.3` |
| Summary self base | `weight.w_self_base: 0.2` |
| Per-neighbor cap | `weight.w_cap: 0.4` |
| Neighbor limit | `weight.kappa: 3` |
| Weight smoothing | `enable_smoothing: false` |
| Attack mitigation | `observer.attack_mitigation: false` |
| Prediction-only mode | `observer.use_predict_observer: false` |
| Rollback | `observer.rollback_enabled: false` |
| Motor model | `vehicle.motor_model.enabled: false` |

## Runtime Pipeline

Each `TrustBasedFleetEstimator.update(local_state, dt, current_time_ns, control)`
cycle performs the following steps.

1. Store the host local state in `fleet_states[:, vehicle_id]`.
2. Update trust scores for every known non-host vehicle.
3. Optionally apply attack mitigation by halving trust for `target_attack` flags.
4. Build generalized trust vector `O_i(j)` when enabled.
5. Compute a global `WeightResult` for logging and diagnostics.
6. For each target vehicle, compute target-specific fusion weights.
7. Apply direct and neighbor consensus corrections.
8. Propagate the corrected state through vehicle dynamics.
9. Optionally switch to prediction-only or blended prediction mode.
10. Log trust, weights, flags, confidence, and fleet estimates.
11. Optionally record and trigger contamination rollback.
12. Remove stale V2V data.

For `trust_consensus`, the state update is correction-then-prediction:

```text
x_corr_j = x_hat_j
         + w0 * (z_j - x_hat_j)
         + sum_l w_l * (x_hat_l_j - x_hat_j)

x_hat_j_next = constraints(f(x_corr_j, u_j, dt))
```

where:

- `x_hat_j` is the host estimate of target `j`.
- `z_j` is the target vehicle's direct V2V local-state broadcast.
- `x_hat_l_j` is neighbor `l`'s fleet estimate of target `j`.
- `u_j` is the target control input if received, otherwise the host control input.
- `f(...)` is bicycle kinematics plus the optional first-order motor model.

The `w_self` term is implicit. Because the update only adds weighted deltas from
external sources, any unused weight remains on the previous estimate:

```text
x_corr_j = w_self * x_hat_j + w0 * z_j + sum_l w_l * x_hat_l_j
w_self = 1 - w0 - sum_l w_l
```

## Trust Framework Logic

`TriPTrustModel.calculate_trust(...)` produces one `TrustScore` per target.

### Inputs

The trust model uses:

- host local state `[x, y, theta, velocity, acceleration]`;
- target direct V2V local state;
- neighbor fleet estimates of the target;
- neighbor fleet estimates of the host;
- the host's current fleet estimate;
- the target's own fleet estimate broadcast, when available;
- optional external relative measurement, such as YOLO or radar distance.

### Local Trust Sample

The local trust sample checks whether the target's directly received state is
locally plausible from the host point of view. The model computes component
scores in `[0, 1]`:

| Component | Meaning |
| --- | --- |
| `velocity_score` | Velocity consistency with adaptive tolerance for turns and acceleration. |
| `distance_score` | Relative distance consistency. |
| `acceleration_score` | Acceleration plausibility with speed, host, turn, and distance terms. |
| `heading_score` | Heading/path similarity with turn-aware tolerance. |
| `beacon_score` | Local beacon availability. |
| `quality_factor` | Message age and communication quality factor. |

The local sample is a weighted geometric mean:

```text
gamma_local =
  velocity_score^0.30 *
  distance_score^0.20 *
  acceleration_score^0.15 *
  heading_score^0.15 *
  beacon_score^0.10 *
  quality_factor^0.10
```

Each score is floored at `0.01` before the product so a failed component strongly
penalizes the result without causing numerical collapse.

Optional gates can multiply the component scores before this fusion:

- `use_physical_constraints_check`
- `use_temporal_consistency_check`

In the current YAML, both are set to `false`.

### Global Trust Sample

The global trust sample checks whether the target's distributed/fleet estimate is
consistent with other available estimates. The implementation uses paper-style
distributed trust:

```text
gamma_global = gamma_host * gamma_local_peer

if gamma_self < trust_threshold:
    gamma_global = gamma_global * gamma_self
```

The components are:

| Component | Meaning |
| --- | --- |
| `gamma_host` | Full-state Mahalanobis agreement between the host fleet estimate and the target's broadcast fleet estimate. |
| `gamma_local_peer` | Relative-measurement agreement between host-observed relative state and the target's implied host-target relative estimate. |
| `gamma_self` | Host self-consistency between host local relative measurement and host global estimate of the target. |

Mahalanobis distances are converted to trust by `distance_to_gamma_method`. The
current YAML uses:

```yaml
distance_to_gamma_method: exponential
```

so:

```text
gamma = exp(-distance)
```

When required data is unavailable, the trust model uses
`distributed_trust_fallback` from the config. In the current YAML this is `0.2`.

### Dirichlet Final Score

The current YAML uses MATLAB-style Dual Dirichlet scoring:

```yaml
dirichlet_method: matlab
dirichlet_type: Dual
dirichlet_C: 0.2
dirichlet_wt_local: 0.4
dirichlet_wt_global: 0.5
```

The local and global samples update separate rating vectors. Each trust sample is
mapped to one of `num_trust_levels` bins. The rating vector is aged and updated:

```text
lambda_y = sigma_y * wt
R_next = (1 - lambda_y) * R + one_hot(sample_level)
```

A regularized score is computed from each vector:

```text
S_y = (R + C / k) / (C + sum(R))
sigma = dot(level_weights, S_y)
```

For Dual mode:

```text
final_score = score_local * score_global
```

If score history exists, `final_score` is then EMA-smoothed with
`trust.ema_alpha`.

### Missing Observations

If no valid direct packet is available for a known target, the trust model calls
`update_missing_observation(...)`:

- unknown targets start near `distributed_trust_fallback`;
- local and global trust decay when beacons are missed;
- beacon and quality scores are set low;
- attack flags are recomputed after decay.

### Attack Flags

Attack flags are set from the local and global trust samples:

| Flag | Condition | Interpretation |
| --- | --- | --- |
| `flag_target_attack` | `gamma_local > threshold` and `gamma_global < threshold` | Direct state looks locally plausible, but distributed cross-check fails. |
| `flag_global_est_check` | `gamma_local < threshold` and `gamma_global > threshold` | Host local observation is suspicious, but global agreement is high. |
| `flag_local_est_check` | `gamma_local < threshold` | The target's direct/local measurement channel is unreliable. |

If `observer.attack_mitigation` is enabled, the estimator halves the final trust
score for vehicles with `flag_target_attack` before computing weights.

### Generalized Trust Vector

When `trust.use_generalized_trust_vector` is true, the estimator builds `O_i(j)`.

Rules:

1. `O_i(i) = 1`.
2. Direct neighbors use the host's direct trust score.
3. Non-direct targets use a weighted median of credible direct-neighbor reports
   when available.
4. If no credible reports exist, the model falls back to an index-distance
   weighted average over direct neighbors.
5. If no direct neighbors exist, it uses `distributed_trust_fallback`.

`weight_type: paper` uses this generalized vector as its opinion source. Other
weight modes use the direct trust scores for weight calculation.

## Weight and Gain Calculation

`WeightTrustModule` has two different roles:

- `calculate_weights(...)` produces a global `WeightResult` used mainly for
  logging, diagnostics, and high-level summaries.
- `calculate_weights_for_target(...)` or `calculate_paper_weights_for_target(...)`
  produces the weights actually used in each target's state update.

This distinction matters because the per-target update only includes neighbors
that currently provide an estimate for that specific target.

### Trusted Neighbor Selection

For both global and per-target calculations, a vehicle is trusted if:

```text
trust_score >= weight.trust_threshold
```

The selected neighbors are sorted by trust descending and limited to:

```text
weight.kappa
```

In the current YAML, `kappa: 3`.

### Current Default: `weight_type: trust_based`

The global diagnostic weight calculation uses:

```text
w0 = w0_fixed
w_self = w_self_base
neighbor_budget = 1 - w0 - w_self
raw_w_l = trust_l / sum(trust_trusted) * neighbor_budget
w_l = min(raw_w_l, w_cap)
normalize all weights
```

With the current YAML:

```text
w0_fixed = 0.3
w_self_base = 0.2
neighbor_budget = 0.5
w_cap = 0.4
```

The per-target update uses a slightly different live formula:

```text
w0 = w0_fixed
remaining = 1 - w0
raw_w_l = trust_l / sum(trust_available) * remaining
w_l = min(raw_w_l, w_cap)
w_self = remaining - sum_l w_l
```

With the current YAML, the direct target broadcast starts at `0.3`, and up to
`0.7` is assigned to neighbors. If `w_cap` clips neighbor influence, the leftover
stays in `w_self`.

If no trusted neighbor has an estimate for the target:

```text
w_self = 1 - w0
```

If no direct measurement is available:

```text
w0 = 0
```

and the old `w0` budget is redistributed proportionally to existing neighbor
weights. If there are no neighbors, it is added to `w_self`.

### Paper Mode

For `weight_type: paper`, the per-target live formula is:

```text
LN_i_j = {l | neighbor l has an estimate of target j and O_i(l) >= theta_min}
anchor = direct measurement included if target_local_trust >= theta_min
n = max(kappa, |LN_i_j| + anchor + 1)
base_w = 1 / n

w0 = base_w if anchor else 0
w_l = base_w for every l in LN_i_j
w_self = 1 - w0 - sum_l w_l
```

The `+1` in `n` reserves the self residual channel.

### Equal and Graph Modes

`calculate_weights(...)` supports:

- `equal`: equal global diagnostic weights over the virtual node and trusted neighbors.
- `graph_based`: topology-based global diagnostic weights using a virtual graph.

The current `trust_consensus` state update still uses
`calculate_weights_for_target(...)` for non-paper modes, so the actual per-target
fusion remains the direct/neighbor/self formula described above.

### Weight Smoothing

When `weight.enable_smoothing` is true, global diagnostic weights are EMA-smoothed:

```text
w_smooth = eta * w_new + (1 - eta) * w_previous
```

The smoothed vector is normalized afterward. In the current YAML this is disabled:

```yaml
enable_smoothing: false
eta: 0.15
```

Per-target live weights are not EMA-smoothed in the current implementation.

### Flag-Driven `w0` Adaptation

After per-target weights are calculated, active trust flags can adjust `w0`:

| Flag | Current action |
| --- | --- |
| `flag_target_attack` | Keep or raise `w0`: `w0 = max(w0, w0 * flag_w0_target_attack_factor)`. |
| `flag_global_est_check` | Reduce direct measurement: `w0 *= flag_w0_global_est_check_factor`. |
| `flag_local_est_check` | Reduce direct measurement: `w0 *= flag_w0_local_est_check_factor`. |

Current factors:

```yaml
flag_w0_target_attack_factor: 1.0
flag_w0_global_est_check_factor: 0.3
flag_w0_local_est_check_factor: 0.5
```

After this adaptation:

```text
w_self = max(0, 1 - w0 - sum_l w_l)
```

Freed direct-measurement weight goes to `w_self`, not to neighbors.

## Contamination Rollback Framework

Rollback is implemented by `ContaminationRollback` and is disabled by default:

```yaml
observer:
  rollback_enabled: false
  rollback_window_size: 15
```

When enabled, each update records a replay buffer entry after normal estimation.
The buffer stores:

- the pre-update fleet state snapshot;
- the direct measurement delta for each target;
- each neighbor contribution delta for each target;
- the dynamics delta applied after consensus correction.

The trigger check runs immediately after recording. A vehicle is newly malicious
when its current trust score falls below `trust_threshold` and it was not already
in the rollback module's malicious set.

On trigger:

1. Start from the oldest buffered `pre_update_states`.
2. Replay each buffered step target by target.
3. Skip direct deltas whose source is newly malicious.
4. Skip neighbor deltas whose neighbor ID is newly malicious.
5. Always keep the dynamics delta.
6. Apply state constraints after each replayed target update.
7. Restore the host vehicle's own current state.
8. Clear the replay buffer.
9. Update rollback stats.

The replay constraints are self-contained:

```text
theta = wrap_to_pi(theta)
velocity = clip(velocity, -2.0, 2.0)
acceleration = clip(acceleration, -5.0, 5.0)
```

Rollback statistics are exposed through:

```python
estimator.get_statistics()["rollbacks"]
```

The returned fields are:

| Field | Meaning |
| --- | --- |
| `total_rollbacks` | Number of rollback triggers. |
| `vehicles_flagged` | Vehicle IDs that triggered rollback. |
| `rollback_times_ns` | Trigger timestamps. |

## Prediction-Only Isolation

Prediction-only mode is separate from rollback. It is disabled by default:

```yaml
observer:
  use_predict_observer: false
```

When enabled, the estimator compares the normal trust-weighted estimate with a
pure dynamics prediction from the previous state. The normalized difference uses:

```yaml
similarity_tolerances: [3.5, 2.0, 0.14, 2.0, 1.0]
```

If the normal estimate is inconsistent for fewer than `n_good` consecutive good
cycles, the target enters prediction mode. Depending on the normalized error, the
estimator either blends toward prediction or uses prediction only. The mode is
released after enough good cycles or after `max_predict_only_time`.

The current prediction confidence is included in logs and contributes to:

```python
estimator.get_self_belief()
```

## Trust Kalman Variant

`trust_kalman` inherits the trust framework but uses a Kalman-style target update:

```text
prediction: x_pred = f(x_old, u, dt)
measurement noise: R = measurement_noise / max(target_trust, 0.1)
Kalman update: x = x_pred + K * (z - x_pred)
consensus correction: average trust_scaled neighbor corrections
```

Kalman parameters are loaded from `observer.kalman` in the child config and
flattened into the `trust_kalman` runtime config:

```yaml
observer:
  kalman:
    process_noise: 0.01
    measurement_noise: 0.1
    initial_covariance: 1.0
```

The rollback component breakdown is most precise for `trust_consensus`, where
direct, neighbor, and dynamics deltas are recorded separately.

## Public API

Typical factory usage:

```python
from Observer.TrustbasedDistributedObserver import create_trust_based_estimator

estimator = create_trust_based_estimator(
    estimator_type="trust_consensus",
    vehicle_id=0,
    fleet_size=3,
    state_dim=5,
    config=config,
    logger=logger,
)
```

Useful methods:

| Method | Purpose |
| --- | --- |
| `add_received_local_state(sender_id, state, timestamp_ns)` | Store a target's direct V2V local state. |
| `add_received_fleet_state(sender_id, fleet_estimates, timestamp_ns)` | Store a neighbor's fleet estimate broadcast. |
| `set_external_relative_measurement(target_id, distance, relative_velocity, ...)` | Add YOLO/radar-like relative measurement for trust checks. |
| `update(local_state, dt, current_time_ns, control)` | Run one trust and fleet-estimation cycle. |
| `get_trust_score(vehicle_id)` | Return detailed `TrustScore`. |
| `get_all_trust_scores()` | Return `{vehicle_id: final_score}`. |
| `get_generalized_trust_vector()` | Return latest `O_i(j)` opinion vector. |
| `get_current_weights()` | Return latest global diagnostic weight vector. |
| `get_attack_flags()` | Return all trust flags. |
| `get_self_belief()` | Return mean prediction confidence. |
| `get_statistics()` | Return update counts and rollback stats. |
| `reset()` | Reset estimator, trust, weights, prediction mode, rollback, and logs. |

## Logging

`TrustWeightLogger` starts automatically in the estimator constructor and writes
per-vehicle CSV logs in this package directory. Logs include:

- global diagnostic weights: `w0`, `w_self`, `total_neighbor_weight`;
- direct and generalized trust;
- local/global trust samples and gamma components;
- component scores and attack flags;
- external relative-measurement diagnostics;
- fleet estimates;
- per-target confidence and prediction-mode status;
- `self_belief`.

Use `plot_trust_data.py` for post-run visualization.

## Tuning Guide

| Goal | Main parameters |
| --- | --- |
| Trust fewer vehicles | Increase `trust.trust_threshold`. |
| Make direct target broadcasts stronger | Increase `weight.w0_fixed`. |
| Limit one neighbor's influence | Decrease `weight.w_cap`. |
| Use fewer neighbors | Decrease `weight.kappa`. |
| Smooth diagnostic weights | Set `weight.enable_smoothing: true` and tune `eta`. |
| Penalize missing/global data more | Lower `distributed_trust_fallback` or enable message age quality. |
| Enable attack isolation without replay | Set `observer.use_predict_observer: true`. |
| Enable historical correction | Set `observer.rollback_enabled: true`. |
| Use paper-style equal legitimate-neighbor weights | Set `weight.weight_type: paper`. |

## Notes and Caveats

- The global `WeightResult` is not always the same as per-target live weights.
  The per-target methods are authoritative for state updates.
- In non-paper modes, per-target live weights are trust-proportional even when
  the global diagnostic mode is `equal` or `graph_based`.
- `w_self` is not applied as an explicit correction term. It is the residual
  weight left on the previous estimate.
- `observer_gain` is currently not used as a direct correction term in the base
  `trust_consensus` path. Dynamics propagation is applied after consensus
  correction.
- `consensus_gain` is currently used by `trust_kalman`, not by the base
  `trust_consensus` correction path.
- Rollback removes contributions from newly flagged malicious sources and then
  clears the buffer.
