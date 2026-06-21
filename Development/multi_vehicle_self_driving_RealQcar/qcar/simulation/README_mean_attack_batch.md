# Python Mean Attack Batch Simulation

This folder contains a MATLAB-style batch runner for the current Python trust
system:

```text
simulation/mean_attack_batch.py
```

It runs five synthetic vehicles in one Python process, feeds the current
`TrustBasedFleetEstimator`, applies the six MATLAB `Mix_test` attack cases, and
writes attack-impact metrics.

## Default Experiment

Defaults are aligned with `simulation/matlab/main_mean_attack_plot.m` where it
fits the Python stack:

| Setting | Default |
| --- | --- |
| Vehicles | `5` (`V0` to `V4`, Python zero-based IDs) |
| Attacker | `V0` |
| Attack family | `Mix_test` |
| Attack cases | `1-6` |
| Attack window | `10 s` to `15 s` |
| Simulation time | `25 s` |
| Step time | `0.05 s` |
| Data channel | `fleet` |

MATLAB's `data_type_attack = "global"` maps to Python's `fleet` broadcast,
because Python uses `local_state` and `fleet_state` V2V messages.

For fleet attacks, the default is:

```text
--fleet-target all
```

This means the attacker sends a corrupted fleet/global estimate for every
vehicle entry, matching the MATLAB batch idea of attacking shared global data.
Use `--fleet-target attacker` to corrupt only the attacker's own fleet entry.

## Attack Cases

| Case | Label | Behavior |
| --- | --- | --- |
| 1 | `P Bias -5m` | Add `-5 m` to `x` |
| 2 | `P Faulty 10m` | Gaussian `x` noise with sigma `10 m`, probability `0.3` |
| 3 | `V Bias -2m/s` | Add `-2 m/s` to velocity |
| 4 | `V Faulty 2.5m/s` | Gaussian velocity noise with sigma `2.5 m/s`, probability `0.3` |
| 5 | `A Faulty 1.0m/s^2` | Gaussian acceleration noise with sigma `1.0 m/s^2`, probability `0.3` |
| 6 | `DoS Attack` | Drop the attacked message channel |

## Run

From the repository root:

```powershell
python Development/multi_vehicle_self_driving_RealQcar/qcar/simulation/mean_attack_batch.py
```

Run a quick smoke test:

```powershell
python Development/multi_vehicle_self_driving_RealQcar/qcar/simulation/mean_attack_batch.py --cases 1 --simulation-time 2 --t-start 0.5 --t-end 1.5 --no-plots
```

Use one-based-looking MATLAB case ranges, but vehicle IDs remain Python
zero-based:

```powershell
python Development/multi_vehicle_self_driving_RealQcar/qcar/simulation/mean_attack_batch.py --attacker-id 1 --cases 1-6
```

Test local-state attacks instead of fleet/global attacks:

```powershell
python Development/multi_vehicle_self_driving_RealQcar/qcar/simulation/mean_attack_batch.py --data-type local
```

## Outputs

By default results are written to:

```text
simulation/results/mean_attack_batch/
```

Main summary:

```text
Results_Mix_test_Attacker_V0.csv
```

Columns:

| Column | Meaning |
| --- | --- |
| `AttackType` | `Mix_test` |
| `AttackerVehicle` | Attacker label |
| `Case` | Attack case number |
| `AttackLabel` | Human-readable case label |
| `Vehicle` | Non-attacker vehicle being evaluated |
| `Mean_Distance` | Average per-observer position RMSE for that target |
| `Mean_Orientation` | Average per-observer heading RMSE |
| `Mean_Velocity` | Average per-observer velocity RMSE |
| `Mean_Acceleration` | Average per-observer acceleration RMSE |
| `Mean_Trust_Score` | Mean trust of that vehicle toward the attacker |
| `Trust_Degradation` | Pre-attack trust minus during-attack trust |
| `Attack_Detection_Time` | First attack-window time when trust drops below `0.7` |

Plots are generated when `matplotlib` is available:

```text
mean_error_heatmaps.png
combined_impact_heatmap.png
mean_trust_heatmap.png
```

Per-case trust CSVs from the trust estimator are copied into:

```text
case_01_trust_logs/
case_02_trust_logs/
...
```

## Notes

- This runner does not modify or launch the real `VehicleLogic` runtime.
- It uses the current Python `TrustBasedFleetEstimator` directly.
- The batch override uses `dynamics_prediction_mode=model` by default so clean
  V2V data is not used as the prediction source. Override with:

```powershell
python Development/multi_vehicle_self_driving_RealQcar/qcar/simulation/mean_attack_batch.py --prediction-mode clean_data
```

