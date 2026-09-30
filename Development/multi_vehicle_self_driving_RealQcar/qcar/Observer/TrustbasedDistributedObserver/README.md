# Trust-Based Distributed Observer

This observer estimates every vehicle state with trust-aware distributed
consensus. The important path is:

1. receive target `local_state` packets and neighbor `fleet_state` packets;
2. compute trust for each target;
3. compute per-target fusion weights;
4. correct the current estimate with direct and neighbor information;
5. predict the corrected state with the vehicle dynamics model;
6. optionally rollback recent contaminated updates when trust later fails.

The main files are:

| File | Purpose |
| --- | --- |
| `trust_based_fleet_estimator.py` | Main observer update loop. |
| `trust_model.py` | Local/global trust, Dirichlet score, attack flags. |
| `weight_trust_module.py` | Direct/self/neighbor weight calculation. |
| `contamination_rollback.py` | Replay rollback after trust-triggered contamination. |
| `config_trust_estimator.yaml` | Current trust observer parameters. |

## Current Important Config

The active defaults in `config_trust_estimator.yaml` are the values to check
first:

```yaml
fleet_estimator_type: trust_consensus

trust:
  trust_threshold: 0.5
  dirichlet_method: matlab
  dirichlet_type: Dual
  use_generalized_trust_vector: true

weight:
  weight_type: trust_based
  w0_fixed: 0.4
  w_self_base: 0.2
  w_cap: 0.4
  kappa: 3
  include_target_self_fleet_estimate: false

observer:
  dynamics_prediction_mode: model
  rollback_enabled: true
  rollback_window_size: 50
  rollback_trusted_state_guard_steps: 5
```

`include_target_self_fleet_estimate: false` is important: `w0` uses only the
target vehicle's direct `local_state`. The target's own
`fleet_state[target]` is not reused as a neighbor source. Set it to `true` only
for ablation/debug tests where that extra self-fleet source is desired.

## Trust Logic

Each target gets a `TrustScore` with two main samples:

- `local_trust_sample`: checks whether the target's direct V2V local state is
  physically plausible from the host view. It uses velocity, distance,
  acceleration, heading, beacon, and quality terms.
- `global_trust_sample`: checks whether distributed/fleet estimates agree. It
  combines host-target state consistency, relative-measurement consistency, and
  `gamma_self` self-consistency.

With the current config, the two samples are accumulated by MATLAB-style Dual
Dirichlet trust, then EMA-smoothed into `final_score`.

Trust flags describe which channel is suspicious:

| Flag | Meaning |
| --- | --- |
| `flag_target_attack` | Local and global trust are both below threshold. |
| `flag_global_est_check` | Direct/local evidence is good, but fleet cross-check is bad. |
| `flag_local_est_check` | Direct/local evidence is bad. |

These flags affect both weight selection and rollback triggers.

## Weight Calculation

The current mode is:

```yaml
weight_type: trust_based
```

For each target `j`, the observer builds live weights over:

- `w0`: the target's direct local-state packet `z_j`;
- `w_l`: neighbor `l`'s fleet estimate of target `j`;
- `w_self`: the residual weight left on the host's previous estimate.

Only trusted neighbors are used:

```text
trust_l >= trust_threshold
```

They are sorted by trust, limited by `kappa: 3`, and capped by `w_cap: 0.4`.
The raw gains are based on:

```text
w0 gain       = w0_fixed * local_trust
self gain     = w_self_base, optionally scaled by gamma_self
neighbor gain = 1 - w0_fixed - w_self_base
```

Then all active raw gains are normalized. If a neighbor exceeds `w_cap`, the
overflow goes to `w_self`. If the direct/local channel is bad and `w0` becomes
zero, total neighbor pull is capped by
`local_bad_zero_w0_neighbor_total_cap`, so prediction/self carries most of the
state.

## Observer Update And Prediction

For target `j`, the consensus correction is:

```text
x_corr_j = x_hat_j
         + w0 * residual(z_j, x_hat_j)
         + sum_l w_l * residual(x_hat_l_j, x_hat_j)
```

`w_self` is not applied as a separate correction term. It is the remaining
weight on the previous host estimate because the update is written as deltas.

After correction, the observer predicts:

```text
x_next_j = f(x_corr_j, u_j, dt)
```

With the current config:

```yaml
dynamics_prediction_mode: model
```

`model` means the standard vehicle dynamics path is used after consensus. Other
modes exist for experiments (`clean_data`, `mixed_clean_data`,
`relative_host_anchor_mixed`, `dead_reckoning`, `none`), but the normal
algorithm should be read as correction first, model prediction second.

## Rollback

Rollback is enabled in the current config:

```yaml
rollback_enabled: true
rollback_window_size: 50
rollback_trusted_state_guard_steps: 5
```

Each update stores replay data: the pre-update fleet state, direct contribution,
neighbor contributions, and prediction metadata. Rollback can trigger from:

- final trust below threshold when the local channel is untrusted;
- `flag_local_est_check`;
- `flag_global_est_check`.

When a vehicle is newly flagged, rollback:

1. seeds the affected target from the last trusted snapshot, stepping back
   `rollback_trusted_state_guard_steps` samples for safety;
2. replays the buffered updates;
3. removes direct and neighbor contributions from malicious sources;
4. keeps the dynamics prediction step;
5. restores the host vehicle's own current state;
6. clears the rollback buffer.

This corrects delayed trust decisions: if bad data was used for a few cycles
before trust dropped, replay removes that contamination from the stored window.

## Practical Tuning

| Goal | Main parameter |
| --- | --- |
| Trust fewer vehicles | Increase `trust.trust_threshold`. |
| Make direct target packets stronger | Increase `weight.w0_fixed`. |
| Keep more prediction/self memory | Increase `weight.w_self_base`. |
| Limit any one neighbor | Decrease `weight.w_cap`. |
| Use fewer neighbors | Decrease `weight.kappa`. |
| Change prediction behavior | Change `observer.dynamics_prediction_mode`. |
| Make rollback more conservative | Increase `rollback_trusted_state_guard_steps`. |

