# Robust KalmanNet: record, train, validate, run

This folder contains the offline workflow for the QCar Robust KalmanNet estimator.

The practical flow is:

1. Record clean driving data while `ekf` is the active local estimator.
2. Train `RobustStateNet` offline from the recorded `.npz` dataset files.
3. Validate the learned model against the baseline fallback estimator.
4. Enable `robust_kalman_net` in runtime and load the trained checkpoint.


## Folder layout

- Dataset recorder output: `Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/datasets/`
- Training script: `Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/train_robust_kalmannet.py`
- Validation script: `Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/validate_robust_kalmannet.py`
- Runtime config: `Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/config_local_estimators.yaml`


## What is inside one dataset

Each recorded `.npz` file contains synchronized samples:

- `x_gt`: target state `[x, y, theta, v, w]`
- `z`: measurement vector `[x, y, theta, v, w]`
- raw branches used by the network:
  - `ax`, `ay`, `wz`, `delta`, `vfl`, `vfr`, `vrl`, `vrr`
- metadata in the paired `.json` file

Important:

- For data collection, keep `local_estimator_type: ekf`.
- The recorder is designed to use the trusted EKF output as the training target.
- If you collect while `robust_kalman_net` is active, the dataset is not the intended clean target set.


## 1. Record a dataset

Before recording:

- In `config_local_estimators.yaml`, set `local_estimator_type: ekf`
- Start the vehicle and Ground Station normally

In the GUI:

- Open the `Offline RKNet Data` panel
- Click `Start`
- Drive the car through straights, turns, acceleration, braking, and mixed maneuvers
- Click `Save` when finished

Recorded files are saved under:

```text
Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/datasets/
```

Example files already present:

- `robust_kalmannet_dataset_V0_20260328_203148.npz`
- `robust_kalmannet_dataset_V0_20260328_211545.npz`
- `robust_kalmannet_dataset_V0_20260328_211604.npz`


## 2. Train the model

Recommended: run training from the `Robust` folder so relative output paths are obvious.

```powershell
cd Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust
python .\train_robust_kalmannet.py `
  .\datasets\robust_kalmannet_dataset_V0_20260328_203148.npz `
  --output models\robust_kalmannet.pt `
  --epochs 30 `
  --batch-size 64 `
  --sequence-length 20 `
  --val-split 0.2
```

You can also train on multiple datasets in one command:

```powershell
python .\train_robust_kalmannet.py `
  .\datasets\robust_kalmannet_dataset_V0_20260328_203148.npz `
  .\datasets\robust_kalmannet_dataset_V0_20260328_211545.npz `
  .\datasets\robust_kalmannet_dataset_V0_20260328_211604.npz `
  --output models\robust_kalmannet.pt `
  --epochs 30 `
  --batch-size 64 `
  --sequence-length 20 `
  --val-split 0.2
```

What the trainer does:

- merges one or more recorded datasets
- builds sliding windows of length `--sequence-length`
- uses a chronological validation split to reduce leakage
- applies sensor attack augmentation by default during training
- saves the best checkpoint when validation loss improves

To train on clean data only, disable augmentation explicitly:

```powershell
python .\train_robust_kalmannet.py `
  .\datasets\robust_kalmannet_dataset_V0_20260328_211604.npz `
  --output models\robust_kalmannet_clean.pt `
  --epochs 30 `
  --batch-size 64 `
  --sequence-length 20 `
  --val-split 0.2 `
  --no-augmentation
```

With `--no-augmentation`, the trainer uses the recorded dataset as-is and does not inject attack corruption into the sensor branches during training.

Training outputs:

- checkpoint: `models\robust_kalmannet.pt`
- training history: `models\robust_kalmannet.train_history.json`

Useful options:

- `--epochs 50`
- `--batch-size 128`
- `--sequence-length 30`
- `--lr 1e-4`
- `--device cpu`
- `--device cuda`
- `--no-augmentation`
- `--attack-prob 0.5`
- `--max-branches-attacked 1`

Notes:

- `--output` is resolved relative to the folder containing `train_robust_kalmannet.py`
- the dataset must contain at least `sequence_length` samples
- validation uses teacher forcing disabled, which is the realistic offline check

## How the current model learns

`RobustStateNet` has two logical parts:

- `predictor`: produces the prior state `x_{k|k-1}`
- `updater`: produces the corrected state `x_{k|k}`

At each timestep the flow is:

1. Build branch inputs from raw sensors and the current state estimate.
2. Predict the next state before correction.
3. Compare that prediction against the measurement vector `z_k`.
4. Learn a Kalman-like correction and apply it.

### Predictor

When `predictor_mode: nn`, the predictor is learned and contains:

- 3 branch LSTMs:
  - IMU branch
  - steering branch
  - wheel branch
- 1 predictor mask MLP:
  - receives the concatenated branch features
  - outputs a feature-wise mask to suppress unreliable branch features
- 1 motion regressor MLP:
  - maps masked fused features to motion terms
  - output is `[dxE, dyE, dpsi, v_next, w_next]`

Then a deterministic kinematic conversion maps that motion into the predicted state `x_{k|k-1}`.

When `predictor_mode: kinematic`, the learned predictor is bypassed. In that mode:

- the predictor LSTMs are not used
- the predictor mask is not used
- the prior state is produced by the analytical kinematic model
- the main learned part is the updater

This is the current recommended mode for this project.

### Updater

The updater is the learned correction block. It receives:

- `dx = x_{k|k-1} - x_{k-1|k-1}`
- innovation `dz_p = z_k - H x_{k|k-1}`
- measurement change `dz = z_k - z_{k-1}`
- `gps_valid`

It contains:

- 1 update mask MLP:
  - gates update features before they reach the recurrent layer
  - purpose: suppress weak or unreliable correction cues
- 1 GRU:
  - keeps temporal memory for the correction process
  - purpose: learn how the correction should depend on recent history, not only the current innovation
- 1 gain MLP:
  - maps the GRU hidden state to a full learned gain matrix `K`

The final update is Kalman-style:

```text
x_{k|k} = x_{k|k-1} + K_k (z_k - H x_{k|k-1})
```

### What LSTM, GRU, and MLP do here

- `LSTM`
  - used only in the learned predictor
  - learns temporal patterns in each sensor branch
- `GRU`
  - used only in the updater
  - learns temporal behavior of the correction step
- `MLP`
  - used for the predictor mask
  - used for the motion regressor
  - used for the update mask
  - used for mapping the GRU hidden state to the gain matrix

### Why `gps_valid` is part of the updater input

The measurement vector `z = [x, y, theta, v, w]` always has the same shape, even when GPS is missing.

When GPS is unavailable, the code fills the position channels with dead-reckoning so the tensor shape stays valid. That does not mean those channels should be trusted like real GPS. `gps_valid` tells the updater whether the position measurement is:

- real GPS-backed data
- or fallback pseudo-measurement

This lets the updater behave closer to a standard estimator that would skip or strongly downweight GPS correction when GPS is unavailable.

### Three-phase training

The trainer uses three phases:

1. Phase A: predictor only
   - teacher forcing on
   - updater frozen
   - trains `x_pred`
2. Phase B: updater only
   - teacher forcing on
   - predictor frozen
   - trains `x_upd`
3. Phase C: end-to-end
   - teacher forcing off
   - both parts train together over autoregressive rollouts

If `--predictor-mode kinematic` is used, Phase A is skipped automatically because the predictor has no trainable parameters.


## 3. Validate the trained checkpoint

Run the validator on the same or a separate recorded dataset:

```powershell
cd Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust
python .\validate_robust_kalmannet.py `
  .\datasets\robust_kalmannet_dataset_V0_20260328_211604.npz `
  --checkpoint .\models\robust_kalmannet.pt `
  --sequence-length 20 `
  --output validation_metrics.json
```

You can validate across multiple datasets too:

```powershell
python .\validate_robust_kalmannet.py `
  .\datasets\robust_kalmannet_dataset_V0_20260328_203148.npz `
  .\datasets\robust_kalmannet_dataset_V0_20260328_211545.npz `
  .\datasets\robust_kalmannet_dataset_V0_20260328_211604.npz `
  --checkpoint .\models\robust_kalmannet.pt `
  --sequence-length 20 `
  --output validation_metrics.json
```

The validator compares:

- `baseline`: fallback estimator only
- `learned`: trained Robust KalmanNet checkpoint

Validation outputs:

- metrics JSON: `validation_metrics.json`
- prediction arrays: `validation_metrics.predictions.npz`

Typical metric structure:

```json
{
  "baseline": {
    "rmse": [0.0, 0.0, 0.0, 0.0, 0.0],
    "rmse_mean": 0.0
  },
  "learned": {
    "rmse": [0.0, 0.0, 0.0, 0.0, 0.0],
    "rmse_mean": 0.0
  }
}
```

How to read it:

- smaller `rmse_mean` is better
- compare `learned.rmse_mean` against `baseline.rmse_mean`
- also inspect the five per-state RMSE values for `x`, `y`, `theta`, `v`, `w`


## 4. Enable runtime inference on the vehicle

After you have a checkpoint you trust, update:

`Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/config_local_estimators.yaml`

Set:

```yaml
local_estimator_type: robust_kalman_net

local:
  robust_kalman_net:
    use_model: true
    load_pretrained: true
    model_path: models/robust_kalmannet.pt
    use_fallback: true
    sequence_length: 20
    min_history: 5
    device: auto
```

Important runtime path rule:

- `model_path` is relative to `qcar/Observer/KalmaNet/Robust/`
- so `models/robust_kalmannet.pt` means:
  `Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/models/robust_kalmannet.pt`

Recommended runtime settings:

- keep `use_fallback: true`
- keep `load_pretrained: true`
- keep `allow_untrained_model: false`


## 5. Minimal end-to-end example

```powershell
cd Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust

python .\train_robust_kalmannet.py `
  .\datasets\robust_kalmannet_dataset_V0_20260328_211545.npz `
  --output models\robust_kalmannet.pt `
  --epochs 30 `
  --batch-size 64 `
  --sequence-length 20

python .\validate_robust_kalmannet.py `
  .\datasets\robust_kalmannet_dataset_V0_20260328_211604.npz `
  --checkpoint .\models\robust_kalmannet.pt `
  --sequence-length 20 `
  --output validation_metrics.json
```
## 6. Common mistakes

- Recording data while `robust_kalman_net` is active instead of `ekf`
- Using too little data to form sequence windows
- Forgetting that `--output` is relative to the training script folder
- Forgetting that `model_path` in YAML is relative to `KalmaNet/Robust/`
- Comparing only one state and ignoring full RMSE across all five states


## 7. Practical recommendation

For a first useful model:

- collect several runs with different maneuvers
- train on multiple `.npz` files together
- keep `sequence_length: 20`
- validate on one held-out run
- only switch runtime to `robust_kalman_net` after the learned model beats the baseline offline


# VALIDATION 
cd Development\multi_vehicle_self_driving_RealQcar\qcar\Observer\KalmaNet\Robust

python validate_robust_kalmannet.py `
  datasets\robust_kalmannet_dataset_V0_20260401_170829.npz `
  --checkpoint models\robust_kalmannet.pt `
  --output validation_metrics_best.json

python validate_robust_kalmannet.py `
  datasets\robust_kalmannet_dataset_V0_20260401_170829.npz `
  --checkpoint models\robust_kalmannet.phase2_final.pt `
  --output validation_metrics_phase2_final.json



Viewed KalmanNet_nn.py:1-293
Viewed robustKLnet.py:1-800
Viewed train_robust_kalmannet.py:1-800

Yes, your current **Robust KalmanNet** setup is conceptually the same as the original, but it is an evolved version specifically optimized for vehicle dynamics and robustness against sensor corruption.

### 1. Conceptual Commonalities
Both architectures are rooted in the same "Neural Kalman Filter" philosophy:
*   **Linear Update Structure**: Both preserve the standard Kalman Filter update equation: $\hat{x}_t = \check{x}_t + K_t(z_t - \check{z}_t)$.
*   **Learned Kalman Gain**: Both replace the traditional Riccati equation (which requires knowing $Q$ and $R$ noise matrices) with a Recurrent Neural Network (RNN) that learns to output $K_t$ directly from data.
*   **Hybrid Approach**: They both combine physical principles (motion and measurement models) with deep learning for adaptive sensor fusion.

### 2. Key Differences in Your "Robust" Setup
While the core concept is identical, your current implementation introduces several advanced features not found in the original "baseline" KalmanNet:

| Feature | Original KalmanNet (TSP) | Your Robust KalmanNet |
| :--- | :--- | :--- |
| **Architecture** | 3 separate GRUs (for $Q, \Sigma, S$) intended to mimic KF internals. | **Streamlined single-GRU** for the update logic, supplemented by **Tri-LSTM branches** for sensor processing. |
| **Robustness** | Assumes Gaussian-ish noise; no explicit defense against corruption. | **Learnable Masking**: Uses a secondary network to "mask out" features or sensors it detects as corrupted or attacked. |
| **Inputs** | Purely innovation and state differences ($z - \check{z}$). | **Asynchronous Flags**: Incorporates `gps_valid` and external signals to inform the gain logic about sensor outages. |
| **Training** | Typically one-shot end-to-end training. | **Three-Phase Training**: Separates prediction learning from update/gain learning to prevent "vanishing weight" issues on the motion model. |
| **Kinematics** | General signal processing focus. | **Vehicle Specific**: Implements mandatory **angle wrapping** ($\psi$) and **gain constraints** (tanh) to prevent real-time "snapping" or drift. |

### 3. Conclusion
Your current setup is a **domain-adapted, robust version** of the original. 
*   It is "conceptually the same" because it still tries to find the optimal $K_t$ to bridge the gap between a model prediction and a measurement.
*   It is "robust" because it adds an extra layer of intelligence (the Masking Net) to decide *how much to trust* that measurement before letting it influence the state.

**In summary: You are using the original KalmanNet "skeleton" but have replaced the "brain" with a more defensive architecture specifically tuned for the QCar's kinematics.**
