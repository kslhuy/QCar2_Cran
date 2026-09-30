# QCar Controller: ACC, CACC, Fusion

Short note for the leader-following controller. Main code:
`longitudinal_controllers.py`, `lateral_controllers.py`,
`controller_manager.py`, and `StateMachine/following_leader_state.py`.

In `FOLLOWING_LEADER`, the vehicle computes a cooperative CACC command, a local
ACC command, then fuses them:
```text
u_cacc  = command from V2V / fleet estimator
u_acc   = command from measured leader gap
u_final = alpha * u_cacc + (1 - alpha) * u_acc
```
`alpha = 1` means CACC only. `alpha = 0` means ACC only.

## ACC
ACC is local sensor following. It uses `sensor_leader_distance_filtered`,
normally from YOLO `car_dist`, and does not need trusted V2V.
```text
d_des = desired_distance + time_headway * v_i
e_d   = d_meas - d_des

if d_meas <= stop_distance:
    v_ref = 0
else:
    v_ref = clip(target_velocity + distance_gain * e_d,
                 min_target_velocity, max_target_velocity)

e_v   = v_ref - v_i
u_acc = u_ff(v_ref) + kp * e_v + ki * integral(e_v) + kd * d(e_v)/dt
```
Concept: small gap lowers `v_ref`; safe extra gap allows speed recovery.
Config: `leader_sensor_acc`, `pid`.

## CACC
CACC is cooperative platoon following. It uses leader state from `leader_source`
such as `fleet_estimator`, `direct_v2v_clean`, or `direct_v2v_attacked`.
```text
s_projected = (x_j - x_i) * cos(psi_ref) + (y_j - y_i) * sin(psi_ref)
s           = path_gap if available, else s_projected
s_des       = s0 + h * v_i
e_s         = s - s_des
e_v         = v_j_long - v_i_long

# Optional acceleration matching
e_v = e_v + leader_acceleration_weight * (a_j - a_i)

a_cmd  = K_spacing * e_s + K_velocity * e_v
         + ki_spacing * integral(e_s)
u_cacc = clip(acc_to_throttle_gain * a_cmd + u_ff, 0, max_throttle)
```
Concept: positive `e_s` means too far back; negative `e_s` means too close.
Positive `e_v` means the leader is faster; negative `e_v` means closing in.
Config: `cacc`.

## Trust Fusion
Fusion protects CACC when cooperative data has low trust.
```text
direct_opinion:
    alpha = clip(opinion, 0, 1)

threshold_map:
    alpha = clip((opinion - trust_low) / (trust_high - trust_low), 0, 1)

u_final = alpha * u_cacc + (1 - alpha) * u_acc
```
The opinion comes from trust fields such as `generalized_trust`,
`leader_trust`, or `direct_trust`. With `opinion_scope: used_predecessors`, the
minimum opinion among the leader and trusted upstream predecessors is used.

High trust gives mostly CACC. Low trust gives mostly ACC. Missing trust or
missing ACC follows `unavailable_policy` and `low_trust_policy`.
Config: `trust_longitudinal_fusion`.

## Lateral Fusion
Steering can also fuse path tracking and leader tracking:
```text
delta_raw = path_weight * delta_path + leader_weight * delta_leader
delta     = smoothing_factor * delta_raw
            + (1 - smoothing_factor) * delta_previous
```
`delta_path` is map/path steering. `delta_leader` is pure pursuit toward the
leader. Config: `fusion_lateral`.

## Tune First
- `cacc.s0`, `cacc.h`: minimum gap and time headway.
- `cacc.K_spacing`, `cacc.K_velocity`: distance and speed gains.
- `leader_sensor_acc.desired_distance`, `stop_distance`: ACC safety gap.
- `trust_longitudinal_fusion.trust_low`, `trust_high`: fusion limits.
- `fusion_lateral.path_weight`, `leader_weight`: path vs leader steering.
