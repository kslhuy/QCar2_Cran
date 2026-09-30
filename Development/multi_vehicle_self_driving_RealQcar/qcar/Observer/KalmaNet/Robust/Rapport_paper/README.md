# Current RobustKLNet paper

The main manuscript is **robust_kalmannet_trust_report.tex**; its compiled version is **robust_kalmannet_trust_report.pdf**.

The paper describes the current `innovation_trust` simulation backend. It distinguishes the simulator's attack annotation, sensor availability, analytical consistency gate, learned reliability and applied trust. The old GRU description is no longer mixed into the active method.

## Evidence and rebuilding

Tables are pooled directly from the adjacent `../results/heldout.json`, `closed_loop.json`, `stress_mismatch.json` and `native_rate.json`. The `.npz` companions contain trajectories. Training history comes from `../models/innovation_trust_sim.history.json`. These are the saved experiments, not new training runs.

From this folder, with the Qcar environment activated:

```powershell
python build_paper_assets.py
pdflatex -interaction=nonstopmode -halt-on-error robust_kalmannet_trust_report.tex
pdflatex -interaction=nonstopmode -halt-on-error robust_kalmannet_trust_report.tex
```

The builder writes tables and vector plots to `paper_assets/`. It verifies that all result files identify the delivered checkpoint. Its `build_observer_figures.py` stage replays nine conditions for seed 101 with the EKF, analytical ablation and hybrid: all 27 trajectories must match the saved benchmark. It also checks that the recorded gain times residual reconstructs each correction, and that availability, analytical consistency and learned reliability reconstruct applied trust. Runtime publication through float32 is accounted for. Source hashes are recorded in `paper_assets/evidence_manifest.json` and `observer_diagnostics_manifest.json`.

## Visual evidence

Each expanded figure is available as a vector `.pdf` and reusable `.png` in `paper_assets/`:

| Figure file | What it demonstrates |
| --- | --- |
| `mask_channel_outputs` | GPS availability and four applied trust channels for eight fault conditions. |
| `trust_factor_comparison` | Analytical gate, learned reliability and applied mask for jump, ramp and tachometer scale corruption. |
| `gain_correction_comparison` | Raw versus applied gain, actual measurement-induced correction and resulting state error. |
| `gain_matrix_snapshot` | Which measurement residual directly changes each state, at attack onset; normalized matrix entries. |
| `observer_state_comparison` | All four observer states against true state and corrupted measurements, with error curves. |
| `performance_overview` | All 13 nominal conditions, three error metrics and four estimators; observed seed ranges. |
| `relative_performance` | Hybrid/EKF and hybrid/analytical RMSE ratios, separating total improvement from learning's contribution. |
| `attack_recovery_performance` | Attack-only and first-three-seconds-after-removal errors across all attacked conditions. |
| `control_stress_comparison` | Closed-loop driving performance and the limits under model mismatch. |

The diagnostic curves are from causal replay, not invented illustrative values. Attack intervals are annotations, never estimator inputs. The mask is reliability, not attack probability. Gains from different observers act on their own predictions and residuals, so gain magnitude alone is not an accuracy metric. The mask heatmaps use the nominal 10 Hz GPS schedule (including missing slots), and subsample the 50 Hz velocity trust to those display times.

`observer_diagnostics.npz` retains the plotted mask/gain/correction/state traces. `phase_metrics.json` retains the derived attack/post-attack metrics; the post-attack window is fixed at three seconds, not a claimed settling time. `relative_metrics.json` retains the full-drive comparison ratios. Ranges across seeds are descriptive ranges, not confidence intervals.

The manuscript describes the limits of that evidence: few simulation seeds, known fault families, no physical-car validation, no formal stability guarantee, and analytical safeguards accounting for much of the improvement. New experimental evidence requires updating the text as well as regenerating the assets.

The previous mixed report is preserved in `archive/` as a historical snapshot. The existing presentation and source reference PDFs were left untouched.

For training, headless tests, web animation and live fake-car setup, see [the workflow guide](../../../../../../../ROBUST_KALMANNET_WORKFLOW.md).
