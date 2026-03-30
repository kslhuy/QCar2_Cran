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


 How the 2-Phase Training Adapts
The training script continues to use a two-phase approach (P1-TF and P2-E2E):

Phase 1: Teacher Forcing (Epochs 1-50)
What happens: The network is fed the Ground Truth state $x_{k-1}$ as the input to step $k$.
With Kinematic Mode: The kinematic model uses the ground truth previous state to predict pure physics. The difference between this physics prediction and the next ground truth state is passed to the 3-cascaded GRU.
Goal: This forces the $Q, \Sigma, S$ GRUs to learn how to perfectly calculate the Kalman Gain without having to worry about compounding errors from previous bad predictions.
Phase 2: End-to-End (Epochs 51-100)
What happens: Teacher forcing is disabled. The network uses its own past outputs $x_{k-1|k-1}$ for the next step.
With Kinematic Mode: The Cascaded GRUs learn to stabilize the physics model over rolling horizons.
💻 How to Run the Training
Ensure you are in the Robust directory inside your conda environment, and simply run the script.

To train the new Kinematic setup:

bash
python train_robust_kalmannet.py "C:\Users\Quang Huy Nugyen\Desktop\PHD_paper\Simulation\QCAR\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar\qcar\Observer\KalmaNet\Dataset_100ms.npz" --predictor-mode kinematic
(Note: Replace Dataset_100ms.npz with your actual dataset file name)

To train the original NN Predictor setup (it still works!):

bash
python train_robust_kalmannet.py "C:\Users\Quang Huy Nugyen\Desktop\PHD_paper\Simulation\QCAR\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar\qcar\Observer\KalmaNet\Dataset_100ms.npz"
Useful flags:
--epochs <number>: Adjust total epochs (default 100).
--attack-prob <float>: Adjust how frequently sensor attacks are injected during training (default 0.3).
--batch-size <number>: Lower this if you run into CUDA Out-Of-Memory errors due to the 3-GRU setup (default 64).

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


