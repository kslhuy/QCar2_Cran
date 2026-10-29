# Local, relative and distributed observer simulation

The live fake-car launcher now runs this path:

```text
GPS / wheel / IMU -> selected local observer ---------------------+
                                                                |
independent relative sensor -> per-target SA_ACC_UIO -> trust ----+-> fleet estimates
neighbours' ordinary V2V local / fleet packets -------------------+
```

The default local observer is Robust KalmanNet with the `innovation_trust`
backend and `innovation_trust_sim.npz`. Each host maintains a separate UIO for
each detected target. Own-vehicle control uses the local estimate; leader
following configured with `leader_source: fleet_estimator` uses the distributed
estimate of the leader.

## Independent sensing

`parameters.yaml -> sensors.relative` configures a surround-radar abstraction:
20 Hz detections, 30 ms latency, range/range-rate/bearing noise, 1% independent
drop probability, and an 8 m range. Field of view and range are configurable.
This models detection errors and visibility limits; it does not model radar
waveforms, multipath, occlusion or target-ID association errors.

Local GPS is sampled at 10 Hz and exposes its noisy sampled position, rather
than the plant's current pose. Wheel and IMU noise settings are applied to
measurements without changing the plant dynamics. Their random sequence is
seeded per vehicle. The standalone Robust KalmanNet benchmark retains its own
noise protocol and disables plant-sensor noise to avoid counting it twice.

Separately launched fake cars exchange plant poses over **loopback-only UDP**
ports `49000 + vehicle_id`. This is the simulated world's internal transport,
not V2V. Only the sensor module reads those poses. Observers receive noisy,
delayed relative detections, never those plant poses. Fake cars must run on the
same computer and appear in `sensors.relative.vehicle_ids`. A port conflict is
an error; the launcher does not choose another sensor or data source.

The UIO uses its fixed 0.01 s model grid and the actual sensor sample interval.
Range is converted to the descriptor's gap convention by subtracting `li` on
input and adding it on output. Its fourth state is a **lumped unknown input**
(including uncommunicated target acceleration and model error); it is not a
separately identified cyberattack magnitude.

Fleet updates predict to the current time before consensus correction. Local
broadcasts carry `estimate_timestamp_ns` separately from packet send time.
Fleet broadcasts and controller reads use the selected predictor to expose
current-time snapshots without advancing the internal estimator cache. A
missing selected input still makes the target unavailable. Snapshot alignment
uses constant-velocity extrapolation within its configured age limit.

## Explicit failure behaviour

- Missing checkpoint, unsupported algorithm, invalid gains or update failure:
  raise an error. No switch to EKF, consensus, zero gains or last-known output.
- Missing or expired relative detections: that target is unavailable after
  0.2 s. It is omitted from fleet broadcasts and returns `None` to controllers.
  Fresh independent detections allow the same UIO algorithm to restart.
- Model prediction also requires fresh target control input. Missing control
  is not replaced by host control or constant-velocity prediction.
- Clean V2V is available for explicitly selected test inputs, including trust,
  prediction, recovery and controller gap. A missing selected reference does
  not automatically select another source.
- `relative_observer_status` and `fleet_observer_status` in observer updates
  report availability and reasons.

## Select clean V2V for controlled tests

These settings are independent; choose only the reference inputs needed by the
experiment. Normal defaults remain independent sensing and model prediction.

| Purpose | Configuration | Choices |
| --- | --- | --- |
| Relative evidence for trust | Trust YAML `observer.relative_measurement_source` | `external` (raw/UIO sensor), `clean_v2v` |
| Fleet prediction | Trust YAML `observer.dynamics_prediction_mode` | `model`, `dead_reckoning`, `none`, `clean_data`, `mixed_clean_data`, `relative_host_anchor_mixed` |
| State after manual recovery reset | Trust YAML `observer.recovery_state_source` | `estimate`, `direct_v2v`, `clean_v2v` |
| Controller's gap input | Controller YAML `leader_sensor_acc.gap_source` | `relative_sensor`, `clean_v2v` |
| Controller's complete leader state | Controller YAML `leader_source` | `fleet_estimator`, `direct_v2v_attacked`, `direct_v2v_clean` |

For example, in `Observer/TrustbasedDistributedObserver/config_trust_estimator.yaml`:

```yaml
observer:
  relative_measurement_source: clean_v2v
  dynamics_prediction_mode: mixed_clean_data
  force_clean_pose_anchor: true
  recovery_state_source: clean_v2v
  require_relative_measurement: true
```

In `Controller/config_controller_sim.yaml`, select `leader_sensor_acc.gap_source:
clean_v2v` to use clean V2V for the gap helper. This selector applies whenever
that helper is enabled or used by trust fusion; it does not enable the helper
by itself. Clean means the communication payload before a V2V attack, which
can still contain local-estimator error. It is not plant ground truth.

Observer status, fleet broadcasts and headless results include
`reference_inputs`/`test_reference_active`, making assisted tests identifiable.
Specialized trust CSV recording can be disabled with trust YAML
`logging.enabled: false` without changing estimation behaviour.

`config_relative_estimators.yaml -> enable_relative: false` explicitly selects
raw independent sensing for trust instead of UIO processing. It does not
select V2V-derived measurements. `config_fleet_estimators.yaml` selects the
fleet algorithm; `trust_consensus` is the implemented trust variant.

V2V must still be activated in Ground Station before distributed updates run.
The simulation controller configuration specifies `stop` when trust fusion is
enabled and required trust is missing or low; it does not select another
controller. The existing trust-fusion enable switch remains available.

## Verify the complete chain without Ground Station

Activate the `Qcar` environment and run from the `qcar` directory:

```powershell
python -m simulation.observer_pipeline --duration 12 --scenario clean
python -m simulation.observer_pipeline --duration 12 --scenario v2v_pose_bias
python -m simulation.observer_pipeline --duration 12 --scenario relative_dropout
python -m simulation.observer_pipeline --duration 12 --scenario gps_bias
python -m simulation.observer_pipeline --duration 12 --scenario turning
python -m simulation.observer_pipeline --duration 12 --closed-loop
python -m simulation.observer_pipeline --duration 12 --closed-loop --scenario relative_dropout
python -m simulation.observer_pipeline --duration 12 --relative-source clean_v2v --prediction-mode clean_data
python -m simulation.observer_pipeline --duration 12 --relative-source clean_v2v --prediction-mode mixed_clean_data --force-clean-pose-anchor
python -m unittest simulation.test_observer_pipeline
python -m unittest simulation.test_reference_modes
python -m unittest simulation.test_flow_math
```

This uses three actual MockQCar plants, the runtime local-estimator factory,
independent relative sensors, per-target UIOs and distributed trust. It reports
RMSE, availability and evidence provenance. It is an estimator integration
check with commanded plant inputs by default. `--closed-loop` drives the two
followers using the configured runtime CACC and their distributed leader
estimates; an unavailable leader requests zero throttle and resets the CACC
integrators. It reports command bounds, stop requests, spacing error and minimum
center-to-center gap. This is a straight longitudinal experiment; it does not
exercise route/lateral control, Ground Station, or the entire state machine.

The child trust YAML's `embedded_core` settings now reach the runtime factory.
Shadow mode monitors Python/C++ parity while Python is selected. Explicitly
granted native authority stops on a mismatch instead of changing backend.

See [the detailed math and architecture audit](../DEEP_FLOW_AUDIT.md) for the
equations, validation results, and remaining model limitations.

For live interaction, continue using `fake_vehicle_real_logic.py` for each car
and activate V2V through Ground Station. `robust_workflow.cmd` remains the
separate local-estimator benchmark/replay workflow.
