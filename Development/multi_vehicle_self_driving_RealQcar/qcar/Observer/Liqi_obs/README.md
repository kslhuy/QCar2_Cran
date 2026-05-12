# Distributed High-Gain Fleet Observer

This folder contains the QCar implementation of a distributed high-gain observer for fleet state estimation.

The main files are:

- `distributed_high_gain_observer.py`: Python estimator used at runtime.
- `config_distributed_high_gain.yaml`: default gains, topology, dynamics, and safety limits.
- `root_review.tex`: theory notes for the Liqi observer structure.

## Current Design

The observer now keeps the existing public fleet state format:

```text
[x, y, theta, velocity, acceleration]
```

but runs the high-gain correction internally in transformed companion-like coordinates:

```text
z = [x, x_dot, x_ddot, y, y_dot, y_ddot]
```

This means the rest of the QCar stack still sees the same 5D fleet estimate, while the observer itself uses a more faithful high-gain state for measurement injection and consensus.

## What The Method Does

Each vehicle runs its own observer. The observer estimates the state of every vehicle in the fleet, including itself and remote vehicles.

The estimated fleet state is stored as:

```text
fleet_states =
[
  x,
  y,
  theta,
  velocity,
  acceleration
] x number_of_vehicles
```

So column `j` is the current estimate of vehicle `j`.

The method combines three ideas:

1. **Model prediction**
   Use the QCar bicycle kinematic model to predict where each vehicle should move next.

2. **High-gain correction**
   If a direct state message is received from another vehicle, compare that measurement with the prediction and apply a strong correction.

3. **Distributed consensus**
   If neighbor vehicles broadcast their own fleet estimates, compare their estimate of a target vehicle with this vehicle's estimate, then correct toward the neighbor consensus.

In simple words:

```text
new estimate =
    predicted motion
  + correction from the target vehicle's own state message
  + correction from neighbor vehicles' fleet estimates
```

## Comparison With The High-Gain Papers

There are two related high-gain observer ideas in this folder:

1. **Alai et al. 2024**
   `Vehicle Trajectory Estimation Using a High-Gain Multi-Output Nonlinear Observer`

2. **Liqi distributed high-gain observer notes**
   `root_review.tex`

They both use high-gain observer ideas, but they solve different problems.

### Alai 2024: Local Multi-Output High-Gain Observer

The Alai paper is a **local observer**. It estimates the trajectory of one target vehicle using local sensor measurements, for example LIDAR position measurements.

The measured output is:

```text
y = [X, Y]
```

Here:

```text
X = x-position of the target vehicle
Y = y-position of the target vehicle
```

Yes, `X` is position. `Y` is also position, but in the other planar direction.

The paper transforms the bicycle model into a companion-form state:

```text
z = [X, X_dot, X_ddot, Y, Y_dot, Y_ddot]
```

This means:

```text
X_dot  = x-direction velocity
X_ddot = x-direction acceleration
Y_dot  = y-direction velocity
Y_ddot = y-direction acceleration
```

The phrase **two output chains** means the measured outputs `X` and `Y` each generate their own derivative chain:

```text
X -> X_dot -> X_ddot
Y -> Y_dot -> Y_ddot
```

More explicitly:

```text
d/dt(X)      = X_dot
d/dt(X_dot)  = X_ddot
d/dt(X_ddot) = nonlinear function f1(z)

d/dt(Y)      = Y_dot
d/dt(Y_dot)  = Y_ddot
d/dt(Y_ddot) = nonlinear function f2(z)
```

That is useful because high-gain observers are easiest to design for integrator-chain or companion-form systems.

The Alai observer has this structure:

```text
z_hat_dot = F z_hat + G f(z_hat) + L(y - H z_hat)
```

It has:

- model prediction,
- high-gain correction from local sensor measurement,
- no V2V communication,
- no neighbor graph,
- no consensus term.

So Alai is high-gain, but it is not distributed.

### Liqi Notes: Distributed High-Gain Observer

The Liqi notes in `root_review.tex` are about a **distributed observer**. Each agent estimates the collective system state and also receives estimates from neighbors.

The theoretical model is already written in triangular companion-like form:

```text
x_i_dot = A_i x_i + B_i phi_i(x_1, ..., x_i, u_i)
y_i     = C_i x_i
```

So Liqi's starting point is not exactly the Alai 6D state:

```text
[X, X_dot, X_ddot, Y, Y_dot, Y_ddot]
```

Instead, Liqi assumes or constructs a chain form per subsystem. In the platoon example, the paper uses longitudinal chain states such as:

```text
leader:   [p, v, a]
follower: [s, s_dot, s_ddot]
```

where:

- `p` is position,
- `v` is velocity,
- `a` is acceleration,
- `s` is inter-vehicle spacing or gap.

The distributed observer structure is:

```text
prediction
+ local high-gain measurement correction
+ neighbor consensus correction
```

The important extra term compared with Alai is the consensus part:

```text
sum_j a_ij (x_hat_j - x_hat_i)
```

This term makes different vehicles' estimates agree with each other.

### Current QCar Implementation

The current implementation in `distributed_high_gain_observer.py` is still a fleet-compatible engineering implementation, but it now includes the missing coordinate transformation.

The public runtime fleet state is still fixed by the existing QCar observer system:

```text
[x, y, theta, velocity, acceleration]
```

Internally, the observer maps that 5D state to:

```text
z = [x, x_dot, x_ddot, y, y_dot, y_ddot]
```

using:

```text
x_dot  = velocity * cos(theta)
y_dot  = velocity * sin(theta)
x_ddot = projection of acceleration and turning dynamics on x
y_ddot = projection of acceleration and turning dynamics on y
```

The observer then applies the high-gain correction in this transformed `z` state and converts back to the standard 5D fleet state for publication.

The prediction model uses bicycle kinematics:

```text
x_next     = x + velocity * cos(theta) * dt
y_next     = y + velocity * sin(theta) * dt
theta_next = theta + velocity * tan(steering) / wheelbase * dt
velocity_next = velocity + acceleration * dt
acceleration_next = acceleration
```

Then the local correction uses companion-chain innovation on planar position and velocity:

```text
innovation = [x_error, y_error, x_dot_error, y_dot_error]
```

This is now closer to the practical local observer in `Compare_method_to_kalman_practical.m`:

- position error corrects position, velocity, and acceleration,
- velocity error corrects velocity and acceleration,
- heading is corrected separately from predicted yaw toward the velocity direction.

Then the distributed consensus correction uses the transformed-state neighbor disagreement:

```text
neighbor_estimate_of_target - my_estimate_of_target
```

### Summary Table

| Item | Alai 2024 local HGO | Liqi distributed HGO | Current QCar implementation |
| --- | --- | --- | --- |
| Main purpose | Track one target vehicle | Distributed estimation over agents | Estimate all vehicles in QCar fleet |
| Local or distributed | Local | Distributed | Distributed |
| Uses V2V | No | Yes, theoretically | Yes |
| Uses consensus | No | Yes | Yes |
| State form | `z = [X, X_dot, X_ddot, Y, Y_dot, Y_ddot]` | Chain states per subsystem | internal `z = [x, x_dot, x_ddot, y, y_dot, y_ddot]`, public output remains 5D |
| Measurement | `y = [X, Y]` | local output `y_i = C_i x_i` | direct V2V state report, injected as position-chain innovation |
| Model | transformed bicycle companion form | triangular/interconnected companion form | bicycle prediction plus transformed high-gain correction |
| Theory match | Strong local HGO theory | Strong distributed HGO theory | closer practical match than the old raw-5D correction |

### Why The Public Interface Is Still 5D

Using Alai's exact state would require each vehicle estimate to be:

```text
[X, X_dot, X_ddot, Y, Y_dot, Y_ddot]
```

But the QCar fleet estimator, V2V messages, plotting, controllers, and recorders already expect:

```text
[x, y, theta, velocity, acceleration]
```

To use Alai exactly inside this fleet system, we would need conversion logic:

```text
x = X
y = Y
theta = atan2(Y_dot, X_dot)
velocity = sqrt(X_dot^2 + Y_dot^2)
acceleration = projection of [X_ddot, Y_ddot] along heading
```

That would introduce extra problems:

- low-speed singularity when `X_dot` and `Y_dot` are near zero,
- noisy derivatives because high-gain already amplifies noise,
- more difficult V2V message compatibility,
- harder consensus because neighbors would need to agree on 6D transformed states,
- extra conversion back to 5D for controllers and plotting.

So the current implementation keeps the QCar fleet state's native 5D public interface, but the observer no longer performs the correction directly in that mixed coordinate system.

In short:

```text
Alai = local high-gain observer after transforming bicycle model to 6D companion form.
Liqi = distributed high-gain observer for interconnected companion-form systems.
This code = QCar-compatible distributed high-gain observer with an internal 6D transformed state and a 5D published fleet state.
```

## Why It Is Called High-Gain

The observer uses a gain parameter called `theta`.

In the code, `theta` creates a diagonal matrix:

```text
D(theta) = diag(theta, theta^2, theta^3, theta^4, theta^5)
```

This matrix amplifies the measurement error before it is added back to the estimate.

Larger `theta` means:

- faster correction,
- stronger reaction to measurement errors,
- more noise sensitivity,
- higher risk of unstable or jumpy estimates.

For that reason, the QCar default is mild:

```yaml
high_gain_theta: 1.2
```

The paper-style values mentioned in the YAML are much larger, but this implementation runs a discrete-time 5D QCar bicycle model, so the defaults are intentionally safer.

## Runtime Flow

The observer is selected in:

```text
Observer/config_fleet_estimators.yaml
```

with:

```yaml
fleet_estimator_type: distributed_high_gain
```

When V2V is active, `VehicleObserverSimple.py` creates this estimator through `FleetEstimatorFactory`.

The update loop is:

1. The local observer estimates this vehicle's own state.
2. V2V messages arrive from other vehicles.
3. Local state messages are stored with `add_received_local_state()`.
4. Fleet estimate messages are stored with `add_received_fleet_state()`.
5. At the fleet observer rate, `update()` is called.
6. The observer predicts and corrects every vehicle state.
7. Old V2V data is removed.
8. The updated `fleet_states` matrix is returned.

## Main Code Flow In `update()`

`update(local_state, dt, current_time_ns, control)` is the main runtime method.

It does the following:

1. **Normalize inputs**
   Converts the local state to a clean 5D vector:

   ```text
   [x, y, theta, velocity, acceleration]
   ```

2. **Clamp time step**
   Keeps `dt` between `min_dt` and `max_dt` so one bad timing value does not create a huge jump.

3. **Seed known states**
   If a vehicle estimate is still zero and a recent direct V2V state exists, the observer uses that direct state as the initial estimate.

4. **Set own vehicle state**
   If `set_own_state_from_local: true`, the host vehicle column is always copied from the local observer. The distributed observer mainly estimates the other vehicles.

5. **Loop over each target vehicle**
   For each target vehicle, the observer performs prediction, measurement correction, and consensus correction.

6. **Apply constraints**
   Position, heading, velocity, acceleration, steering, and throttle limits are applied to avoid invalid estimates.

7. **Save and return**
   The updated matrix is saved in `self.fleet_states` and returned.

## Prediction Step

The prediction is handled by `_predict_bicycle()`.

For each vehicle state:

```text
x_next     = x + velocity * cos(theta) * dt
y_next     = y + velocity * sin(theta) * dt
theta_next = theta + velocity * tan(steering) / wheelbase * dt
```

If `motor_model_enabled: false`, velocity is predicted with:

```text
velocity_next = velocity + acceleration * dt
```

If `motor_model_enabled: true`, the code uses a simple first-order motor acceleration model with throttle, braking, drag, and friction.

## Measurement Correction

The measurement correction is handled by `_measurement_term()`.

Input:

- `z_state`: predicted internal estimate.
- `y_measured`: direct state received from the target vehicle.

The code computes:

```text
innovation = y_measured - z_state
```

The heading error is wrapped so angle differences stay in `[-pi, pi]`.

Then it applies a chain-form high-gain injection:

```text
measurement_correction = T(theta) * L * [x_error, y_error]
```

This is the high-gain part.

The correction is clipped with `max_innovation_norm` and `max_measurement_term_norm` to avoid a single bad message causing a large jump.

## Consensus Correction

The consensus correction is handled by `_consensus_term()`.

The host vehicle checks its topology row:

```text
adjacency_matrix[host_vehicle_id]
```

For every connected neighbor, it asks:

```text
What does this neighbor estimate for the target vehicle?
```

Then it accumulates:

```text
neighbor_estimate_of_target - my_estimate_of_target
```

If `normalize_consensus_by_weight: true`, the average is normalized by total neighbor weight.

The final consensus correction is:

```text
consensus_gain * r_i * T(theta) * P_inverse * T(theta)^-1 * average_neighbor_error
```

where:

- `consensus_gain` is `gamma` in the YAML.
- `r_i` is the host vehicle's row-vector weight.
- `P_inverse` comes from `p_matrix`.
- `T(theta)` is built from the high-gain dilation matrix.

In simple words, this term makes the fleet estimates agree across vehicles.

## Important Functions

| Function | Simple meaning |
| --- | --- |
| `__init__()` | Loads YAML config, gains, topology, dynamics, constraints, and debug settings. |
| `_load_effective_config()` | Loads `config_distributed_high_gain.yaml`, then merges runtime overrides. |
| `add_received_local_state()` | Stores another vehicle's own state message and its control input if provided. |
| `add_received_fleet_state()` | Stores another vehicle's complete fleet estimate broadcast. |
| `update()` | Main estimator step. Predicts and corrects all vehicle states. |
| `_predict_bicycle()` | Applies QCar bicycle kinematics. |
| `_measurement_term()` | Computes direct high-gain correction from the target vehicle's own state report. |
| `_consensus_term()` | Computes distributed correction using neighbor fleet estimates. |
| `_apply_state_constraints()` | Clips unsafe values and wraps heading angle. |
| `get_debug_data()` | Returns debug information when `debug.enabled: true`. |

## Configuration Guide

Main observer parameters are in `config_distributed_high_gain.yaml`.

### `observer`

- `coordinate_mode`: `chain_6d` uses the transformed state `z = [x, x_dot, x_ddot, y, y_dot, y_ddot]`.
- `z_dim`: internal observer state size. In transformed mode this implementation expects `6`.
- `high_gain_theta`: high-gain strength. Higher means faster but noisier correction.
- `measurement_output`: in transformed mode the default is `position_velocity`, so the observer uses both position and derived planar velocity innovation.
- `use_practical_chain_update`: enables the MATLAB-style discrete chain correction.
- `position_gain`, `position_to_velocity_gain`, `position_to_acceleration_gain`: gains for position innovation.
- `velocity_gain`, `acceleration_from_velocity_gain`: gains for velocity innovation.
- `yaw_correction_gain`, `min_speed_for_heading`: separate heading correction parameters.
- `observer_gain`: fallback chain injection gain used by the non-practical transformed correction path.
- `consensus_gain`: how strongly to follow neighbor fleet estimates.
- `p_matrix`: matrix used in the distributed consensus transform. Default is identity.
- `set_own_state_from_local`: keep the host vehicle column equal to its local state.
- `direct_state_blend`: optional direct blend toward a received state after the observer update.
- `max_*_norm`: safety clips for measurement and consensus corrections.
- `min_dt`, `max_dt`: allowed time-step bounds.

### `topology`

- `fully_connected`: every vehicle can use every other vehicle as neighbor.
- `chain`: each vehicle uses previous and next vehicle.
- `ring`: chain with first and last connected.
- `custom`: use `adjacency_matrix`.

For a custom adjacency matrix:

```text
adjacency_matrix[i][k] > 0
```

means vehicle `i` uses messages from vehicle `k`.

### `dynamics`

These values control the QCar prediction model.

- `wheelbase`: QCar wheelbase.
- `motor_model_enabled`: use throttle-based motor acceleration model if true.
- `motor_tau`, `k_th`, `k_br`, `c_v`, `c_fric`: motor and drag parameters.

### `constraints`

These limits prevent unrealistic estimates.

- `max_abs_position`
- `max_abs_velocity`
- `max_abs_acceleration`
- `max_abs_steering`
- `max_abs_throttle`

### `debug`

Set:

```yaml
debug:
  enabled: true
```

to record per-target innovation, measurement correction norm, consensus correction norm, and neighbor count inside `debug_data`.

## Data Needed For Best Results

The observer works best when each vehicle broadcasts:

1. Its own local state:

   ```text
   x, y, theta, velocity, acceleration
   ```

2. Its current control input:

   ```text
   steering, throttle
   ```

3. Its current fleet estimate dictionary.

The estimator can still run with missing data, but:

- without direct local-state messages, high-gain correction is zero,
- without neighbor fleet messages, consensus correction is zero,
- with no fresh data, the estimate mainly follows model prediction.

## Tuning Advice

Start conservative.

Recommended order:

1. Keep `high_gain_theta` near `1.1` to `1.5`.
2. Tune `observer_gain` first until direct V2V corrections are smooth.
3. Increase `consensus_gain` only after direct correction is stable.
4. Keep clipping limits active during real-vehicle tests.
5. Enable debug data and check whether measurement or consensus corrections are saturating.

Symptoms:

- Estimate reacts too slowly: increase `observer_gain` or `high_gain_theta` slightly.
- Estimate is noisy or jumps: decrease `high_gain_theta`, decrease `observer_gain`, or lower norm limits.
- Vehicles disagree with each other: increase `consensus_gain` slightly.
- Consensus causes wrong pulls: check topology, stale V2V timestamps, and neighbor fleet messages.

## Short Mental Model

Think of the observer as three layers:

```text
motion model       -> where the vehicle should be
direct correction  -> what the target vehicle says about itself
consensus          -> what neighbors believe about the fleet
```

The final estimate is the balance of those three layers.
