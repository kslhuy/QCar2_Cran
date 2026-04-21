# Local Platoon-Ready Gate for Trust Weight Activation

## Summary
Replace the hard `startup_fixed_duration_s: 5.0` behavior with a pure-local readiness gate. Each vehicle independently decides when its own estimator has enough fresh communication, stable formation geometry, and observer confidence to start using trust-adaptive weights. Vehicles will usually switch close together, but not exactly at the same timestamp, because each host has different packet timing and sensor observations.

## Key Changes
- Add flat `weight:` config fields in `config_trust_estimator.yaml` and `WeightConfig`:
  - `startup_gate_enabled: true`
  - `startup_fixed_duration_s: 0.0` kept only as a backwards-compatible fallback.
  - `startup_min_confirm_cycles: 15`
  - `startup_max_bad_cycles: 10`
  - `startup_require_fleet_state: true`
  - `startup_spacing_tolerance_m: 0.20`
  - `startup_relative_velocity_tolerance_mps: 0.08`
  - `startup_estimator_confidence_min: 0.75`
  - `startup_trust_weight_ramp_s: 1.5`
- Add estimator state in `TrustBasedFleetEstimator`:
  - `self.platoon_ready`
  - `self.platoon_ready_since_ns`
  - `self._startup_good_cycles`
  - `self._startup_bad_cycles`
  - `self._latest_platoon_gate_status`
- Replace `_use_startup_fixed_weights()` with a readiness-based method:
  - If `startup_gate_enabled` is false, trust weights are active immediately.
  - If fixed duration is still configured above zero, use it only as an optional minimum warmup.
  - Otherwise, keep neutral startup weights until local readiness passes for `startup_min_confirm_cycles`.
- Readiness checks are local only:
  - At least one valid recent local-state packet exists for each known target.
  - If configured, at least one valid recent fleet-estimate packet exists from available neighbors.
  - For targets with external relative measurements, distance error and relative velocity must be within tolerance.
  - Estimator confidence from `_apply_prediction_mode_switch()` must be above threshold once available.
  - A failed check increments bad cycles; readiness drops only after `startup_max_bad_cycles`.
- Change startup transition from hard switch to ramp:
  - Before ready: use neutral trust inputs of `1.0`.
  - After ready: blend from neutral to real trust over `startup_trust_weight_ramp_s`.
  - Use the same blended trust source for both global weight summary and per-target weights.

## Logging
- Add trust log fields:
  - `platoon_ready`
  - `startup_gate_active`
  - `startup_good_cycles`
  - `startup_bad_cycles`
  - `trust_weight_alpha`
  - `startup_gate_reason`
- Keep existing `startup_fixed_weights` for compatibility, but make it reflect whether startup neutral weighting is still active.

## Test Plan
- Add focused unit-style tests or a small dry-run script for `TrustBasedFleetEstimator`:
  - No received vehicles: gate stays inactive and weights remain neutral.
  - Fresh local and fleet packets for enough cycles: `platoon_ready` becomes true.
  - Relative spacing or relative velocity outside tolerance: gate stays inactive.
  - Temporary packet drop after ready: readiness remains true until bad-cycle threshold.
  - Sustained bad packets after ready: readiness turns false.
  - Ramp behavior: `trust_weight_alpha` moves from `0.0` to `1.0` after ready.
- Run Python compile check for changed files:
  - `python -m py_compile trust_based_fleet_estimator.py weight_trust_module.py trust_logger.py`

## Assumptions
- Use the user-selected **Pure Local** design: no leader decision and no shared `platoon_ready` broadcast in this implementation.
- “Almost same time” is expected in normal operation, but exact synchronization is not guaranteed or required.
- Formation geometry uses available external relative measurements when present; if no external relative measurement exists for a target, communication freshness and estimator confidence are enough for v1.
- Existing trust computation remains unchanged; only the activation of trust-based weights changes.
