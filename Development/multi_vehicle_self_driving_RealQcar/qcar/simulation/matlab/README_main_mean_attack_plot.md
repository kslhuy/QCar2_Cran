# Mean Attack Analysis for `main_mean_attack_plot.m`

This file documents the MATLAB batch experiment implemented in:

```text
test/main_mean_attack_plot.m
```

The script evaluates how cyberattacks affect distributed vehicle-state estimation and trust in a cooperative vehicle platoon. It runs several attack cases, computes mean estimation errors and trust metrics for each non-attacker vehicle, appends results to Excel, and creates paper-oriented comparison plots.

## Purpose

`main_mean_attack_plot.m` evaluates a five-vehicle platoon under the `Mix_test` attack set. In the default configuration:

- Vehicle `V1` is the attacker.
- Vehicles `V2` to `V5` are evaluated.
- The attack is applied to global shared data.
- The attack runs from `t = 10 s` to `t = 15 s`.
- The full simulation runs for `25 s` with `dt = 0.01 s`, inherited from `test/Config.m`.
- The lead scenario is fixed as `constant`.
- Each attack case is simulated independently with a fresh attack module and communication module.

The script is designed for attack-impact comparison across cases and vehicles, not for single-scenario animation.

## Default Experiment Settings

The main settings are defined near the top of `test/main_mean_attack_plot.m`:

```matlab
t_star = 10;
t_end = 15;
attacker_vehicle_id = 1;
victim_id = -1;
data_type_attack = "global";
attack_type = "Mix_test";
start_attack_senarios_index = 1;
last_attack_senarios_index = 6;
num_vehicles = 5;
```

`victim_id = -1` means the attack targets all receiver vehicles.

## Attack Cases

The six default `Mix_test` cases are defined in `test/core/communication/Atk_Scenarios.m`.

| Case | Label | Attack behavior |
| --- | --- | --- |
| 1 | P Bias -5m | Position `X` bias of `-5 m` |
| 2 | P Faulty 10m | Random faulty position with intensity `10 m` and probability `0.3` |
| 3 | V Bias -2m/s | Velocity bias of `-2 m/s` |
| 4 | V Faulty 2.5m/s | Random faulty velocity with intensity `2.5 m/s` and probability `0.3` |
| 5 | A Faulty 1.0m/s^2 | Random faulty acceleration with intensity `1.0 m/s^2` and probability `0.3` |
| 6 | DoS Attack | Drop attack on global communication data |

## Vehicle Setup

The script overrides the four-vehicle graph from `Config.m` and creates a fully connected five-vehicle graph:

```matlab
graph = ones(num_vehicles) - eye(num_vehicles);
```

Initial conditions:

| Vehicle | Initial x position | Initial speed | Primary controller | Cooperative controller |
| --- | ---: | ---: | --- | --- |
| V1 | 80 m | 23 m/s | None | None |
| V2 | 60 m | 26 m/s | IDM | CACC |
| V3 | 40 m | 26 m/s | IDM | CACC |
| V4 | 20 m | 26 m/s | IDM | CACC |
| V5 | 0 m | 26 m/s | IDM | CACC |

All vehicles start in lane `1`, use the same lane centerline, and keep the current lane.

## Metrics

For each attack case, the script computes observer estimation errors for each non-attacker vehicle:

- Mean distance error
- Mean orientation error
- Mean velocity error
- Mean acceleration error

It also extracts trust metrics from each evaluator vehicle's `trust_log` toward the attacker:

- Mean trust score
- Trust degradation, computed as pre-attack mean trust minus during-attack mean trust
- Attack detection time, defined as the first time during the attack window when trust falls below `0.7`

## Excel Output

Results are appended to:

```text
test/Results_Mix_test_Attacker_V1.xlsx
```

The sheet name is:

```text
Summary
```

With the default settings, the script writes:

```text
6 attack cases x 4 evaluated vehicles = 24 rows
```

Columns written by the script:

| Column | Meaning |
| --- | --- |
| `AttackType` | Attack family, default `Mix_test` |
| `AttackerVehicle` | Attacker ID, default `V1` |
| `Case` | Attack case number |
| `Vehicle` | Evaluated vehicle |
| `Mean_Distance` | Mean distance estimation error |
| `Mean_Orientation` | Mean orientation estimation error |
| `Mean_Velocity` | Mean velocity estimation error |
| `Mean_Acceleration` | Mean acceleration estimation error |
| `Mean_Trust_Score` | Average trust score toward the attacker |
| `Trust_Degradation` | Trust drop from pre-attack to during-attack period |
| `Attack_Detection_Time` | First detection time based on trust threshold `0.7`; `NaN` if not detected |

The script appends to the existing Excel file. Delete or rename the workbook before running if you want a clean result table.

## Figures Generated

The script creates these MATLAB figures:

1. Bar plots for distance, velocity, and acceleration estimation errors.
2. Separate heatmaps for distance, velocity, and acceleration impact.
3. Normalized combined impact heatmap using equal weights for distance, velocity, and acceleration.
4. Raw combined error heatmap using the sum of distance, velocity, and acceleration errors.
5. Hybrid heatmap where colors show normalized impact and text values show raw combined error.

Orientation error is computed and logged to Excel, but it is excluded from the main paper-quality plots and combined impact scores.

The figures are displayed but not automatically saved by `main_mean_attack_plot.m`.

## Command-Window Summary

At the end of the run, the script prints:

- Highest and lowest raw-error attack cases
- Most affected and most robust vehicles
- Average raw combined error by case and by vehicle
- Raw error severity classification
- Error component contribution percentages
- Normalized impact severity analysis
- Vehicle resilience ranking
- Excel output file name

## How to Run

Open MATLAB and set the working directory to the `test` folder. This matters because `Config.m` uses relative paths.

```matlab
cd('C:\Users\Quang Huy Nugyen\Desktop\PHD_paper\Simulation\HUY_ALL_TEST\Lane-Change-CBF_Huy\Lane-Change\test')
main_mean_attack_plot
```

Or run it explicitly:

```matlab
run('main_mean_attack_plot.m')
```

## Important Supporting Files

| File | Role |
| --- | --- |
| `test/main_mean_attack_plot.m` | Main batch simulation, logging, plotting, and summary analysis |
| `test/Config.m` | Simulation time step, observer settings, trust settings, road/lane setup, and path setup |
| `test/core/communication/Atk_Scenarios.m` | Attack-case definitions |
| `test/core/communication/Attack_module.m` | Applies attack behavior to communicated data |
| `test/core/communication/CenterCommunication.m` | Shared communication layer used during attacks |
| `test/core/Vehicle.m` | Vehicle model, controller assignment, observer, and trust logs |
| `test/core/Simulator.m` | Simulation loop |
| `test/Function/Weight_Trust_module.m` | Trust-weighted graph and neighbor weighting logic |

## Changing the Experiment

Common edits are made in `test/main_mean_attack_plot.m`.

Change attacker:

```matlab
attacker_vehicle_id = 1;
```

Change attack data channel:

```matlab
data_type_attack = "global"; % or "local"
```

Change attack family:

```matlab
attack_type = "Mix_test";
```

Change attack-case range:

```matlab
start_attack_senarios_index = 1;
last_attack_senarios_index = 6;
```

Enable per-case observer error plots:

```matlab
is_plot_each_case = true;
```

If you change `attack_type` or the case range, update `all_attack_descriptions` in the plotting section so the figure labels match the selected cases.

## Reproducibility Notes

- `Config.m` calls `clear`, `clc`, and `close all`, so do not define required variables before calling the main script.
- Random faulty attacks and measurement noise can produce different numerical results between runs. Add an `rng(...)` seed near the top of `main_mean_attack_plot.m` if repeatability is required.
- Close the Excel output file before running the script. MATLAB may fail to write if the workbook is open in Excel.
- The script stores complete trust logs in memory as `all_case_trust_logs`, `all_case_vehicles`, and `all_case_scenarios` for deeper post-run analysis.

