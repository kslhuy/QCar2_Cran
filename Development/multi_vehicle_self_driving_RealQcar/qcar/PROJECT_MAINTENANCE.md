# Observer and simulation maintenance — 2026-10-29

The follow-up [deep flow audit](DEEP_FLOW_AUDIT.md) documents additional
covariance, timing, replay, packet-validation, native-backend and controller-loop
fixes, including quantitative results and the remaining work in priority order.

This pass audited 322 Python files under `qcar`, excluding vendored vehicle
models, the original KalmanNet distribution, caches and `oldss`. The syntax and
duplicate-definition scan passed after cleanup. Functional validation focused
on the active observer, simulation, V2V and following-controller paths.

## Clean V2V remains available for testing

Clean communication data is a valid controlled-test input. The previous change
blocked more than intended; selectable clean inputs have been restored.

- Trust evidence: `observer.relative_measurement_source: clean_v2v`.
- Prediction: `clean_data`, `mixed_clean_data` and `relative_host_anchor_mixed`.
- Recovery reset: `observer.recovery_state_source: clean_v2v`.
- Controller gap: `leader_sensor_acc.gap_source: clean_v2v`.
- Complete leader state: existing `leader_source: direct_v2v_clean`.

Independent sensing and model prediction remain the defaults. A missing
selected input does not change the source or algorithm. Reference-assisted
runs report their selected inputs and `test_reference_active` status.

The [observer workflow](simulation/README_observer_pipeline.md) lists the
configuration files, command examples and availability behaviour.

## Code changes

| Finding | Applied change |
| --- | --- |
| Duplicate `MockQCar.write` and `StateBase._init_controllers` methods | Removed the shadowed definitions, retaining the effective implementations. |
| Repeated parsing of observer/controller YAML | Shared safe parser and a bounded cache keyed by path, modification time and size; each caller receives its own copy. |
| Replaced fleet estimators retained recording/native resources | Added an idempotent `close()` lifecycle and closed old estimators on replacement and shutdown. |
| Disabled recording still built detailed log dictionaries | Skip diagnostic formatting when CSV recording is inactive; added `logging.enabled`. |
| Clean prediction functions repeated reference lookup and alignment | Reused one required-reference helper and removed an unreferenced private helper. |
| Controller gap source was implicit | Added an explicit source selector and bounded reference freshness. |
| V2V reads exposed mutable cached dictionaries | Return a snapshot; optional age limits reject expired/future packets. |
| Invalid reference-source names could be unclear | Centralized source settings and reject unsupported values. |
| Stale documentation described clean test modes as forbidden | Updated workflow and experiment documentation. |
| Tool state and generated verification artifacts cluttered Git status | Added narrow ignore rules without deleting experiment data. |

Unused imports were also removed from the touched runtime modules. Existing
pending PCB, frontend, paper and dataset work was preserved.

## Validation and performance

- 42 existing observer/integration tests and 15 additional reference/configuration
  tests passed. Coverage includes independent and clean-assisted three-vehicle
  runs, active V2V attacks, controller gap selection, missing references,
  recovery selection, parser cache invalidation and estimator replacement.
- The clean-reference mean-attack batch smoke test completed.
- All 322 scanned Python files parsed; no duplicate method definitions remained
  in the scanned source set.
- A local microbenchmark reading five configurations five times measured
  0.20681 s with the previous safe loader and 0.01145 s with the shared cached
  loader, approximately 18× faster for that workload. This is not an end-to-end
  simulation speedup measurement.

Profiling also showed that cold Robust KalmanNet initialization spends substantial
time loading PyTorch. The inference backend and trained checkpoints were kept
unchanged. Large observer modules and legacy research scripts remain candidates
for future structural work; they were not mechanically split or reformatted.

These checks validate simulation integration. They do not establish real-vehicle
or full closed-loop driving performance.
