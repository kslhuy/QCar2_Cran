# SCNN Parameters Reference

This document explains parameters used in:
- `scnn_wrapper.py`
- `QCar2_LaneNet_lane_estimation_new.py`

## 1. SCNNWrapper Constructor Parameters

| Parameter | Wrapper Default | Value in `QCar2_LaneNet_lane_estimation_new.py` | Valid Range | Meaning |
| :--- | :--- | :--- | :--- | :--- |
| `config_path` | auto default (`resnet18_tusimple.py`) | from selected preset | valid file path | Model config file (network definition). |
| `weight_path` | auto default (`model.pt`) | from selected preset | valid file path | Model weights file. |
| `strict_weights` | `True` | `True` | `True` or `False` | If `True`, fail on checkpoint/config mismatch. |
| `use_half` | `True` | `True` | `True` or `False` | Use FP16 on CUDA for faster inference. |
| `use_cuda_benchmark` | `True` | implicit default | `True` or `False` | Enables cuDNN autotune for fixed input size. |
| `input_size_override` | `None` | `(256, 640)` when `SPEED_MODE=True` | `(H, W)` tuple or `None` | Override model input resolution. |
| `disable_lane_head` | `False` | `True` when `SPEED_MODE=True` | `True` or `False` | Disable lane-existence head for speed. |
| `lane_prob_threshold` | config `thresh` (often `0.3`) | `0.22` | `0.0` to `1.0` | Pixel confidence threshold for lane mask. |
| `lane_exist_threshold` | `0.5` | `0.35` | `0.0` to `1.0` | Lane-level confidence threshold. |
| `temporal_alpha` | `0.6` | `0.45` | `0.0` to `0.99` | Temporal smoothing weight for previous frame. |
| `temporal_binary_threshold` | `110` | `100` | `1` to `254` | Threshold after temporal blend to produce final binary mask. |
| `morph_open_iterations` | `0` | `0` | integer `>= 0` | Opening pass count (remove small noise). |
| `morph_close_iterations` | `1` | `1` | integer `>= 0` | Closing pass count (fill small gaps). |
| `morph_dilate_iterations` | `1` | `1` | integer `>= 0` | Dilation pass count (thicken lane mask). |

## 2. Runtime Parameters (`set_runtime_params`)

These can be changed live:
- `lane_prob_threshold`
- `lane_exist_threshold`
- `temporal_alpha`
- `temporal_binary_threshold`
- `morph_open_iterations`
- `morph_close_iterations`
- `morph_dilate_iterations`
- `max_lane`

`max_lane` behavior:
- `max_lane > 0`: keep only top-k lanes by lane confidence.
- `max_lane == 0`: no top-k pruning.

`disable_lane_head=True` note:
- If lane head output is absent, wrapper assumes all lane classes exist.
- In that case, `lane_exist_threshold` has little or no effect.

## 3. Parameter Effects (Practical Meaning)

| Parameter | Increase Value | Decrease Value |
| :--- | :--- | :--- |
| `lane_prob_threshold` | Cleaner mask, fewer false positives, may miss weak lanes | More sensitive, more noise, better weak-lane recall |
| `lane_exist_threshold` | Fewer lane instances survive | More lane instances survive |
| `temporal_alpha` | More stable, slower reaction | Faster reaction, more flicker |
| `temporal_binary_threshold` | Stricter final mask | Easier mask activation |
| `morph_open_iterations` | More isolated noise removal | Preserve thin details and noise |
| `morph_close_iterations` | More gap filling | Less artificial connection |
| `morph_dilate_iterations` | Thicker lane mask | Thinner mask |
| `max_lane` | More allowed lane instances | Fewer lanes if set low |

## 4. Trackbar Mapping (`SCNN Controls`)

| Trackbar | Internal Parameter | Conversion |
| :--- | :--- | :--- |
| `LaneProb x100` | `lane_prob_threshold` | `value / 100` |
| `LaneExist x100` | `lane_exist_threshold` | `value / 100` |
| `TempAlpha x100` | `temporal_alpha` | `value / 100` |
| `TempBinThr` | `temporal_binary_threshold` | direct integer |
| `OpenIter` | `morph_open_iterations` | direct integer |
| `CloseIter` | `morph_close_iterations` | direct integer |
| `DilateIter` | `morph_dilate_iterations` | direct integer |
| `AdaptiveROI` | `adaptive_roi` | `0` or `1` |
| `TopCrop` | `top_crop` | direct integer (pixels) |
| `BottomCrop` | `bottom_crop` | direct integer (pixels) |
| `MinTop` | `min_top` | direct integer (pixels) |
| `MaxTop` | `max_top` | direct integer (pixels) |
| `ROIAdjPeriod` | `roi_adj_period` | direct integer, clamped to `>= 1` |

## 5. ROI and Throughput Parameters (Script Level)

| Parameter | Default | Meaning |
| :--- | :--- | :--- |
| `crop_bottom_px` | `60` | Pixels removed from image bottom. |
| `default_crop_start` | `max(90, crop_end - 300)` | Initial top of ROI crop. |
| `min_crop_start` | `70` | Minimum allowed top crop. |
| `max_crop_start` | `240` | Maximum allowed top crop. |
| `crop_adjust_period` | `5` | Frames between adaptive ROI updates. |
| `INFERENCE_EVERY_N` | `1` | Run model every N frames (reuse cached result otherwise). |
| `SPEED_MODE` | `True` | Enables fast settings block. |
| `INPUT_SIZE_OVERRIDE` | `(256, 640)` when speed mode | Faster but lower vertical detail than full size. |
| `DISABLE_LANE_HEAD_FOR_SPEED` | `True` when speed mode | Reduces compute by removing lane existence head. |

Adaptive ROI logic thresholds:
- move crop up by 4 if `top_lane_ratio > 0.012`
- move crop up by 2 if `lane_ratio < 0.002`
- move crop down by 2 if `top_lane_ratio < 0.001` and `bottom_lane_ratio > 0.01`

## 6. Color and Overlay Parameters

`self.colors` is a class-id to RGB lookup table used only for visualization (`instancePred`):
- class `0`: background
- classes `1..N`: lane instance colors

Important behavior:
- `_build_instance_mask` colors only where `(class_map == lane_id) AND (binary_mask > 0)`.
- It does not tint non-lane regions by itself.

## 7. Camera Color Order Parameter

| Parameter | Options | Meaning |
| :--- | :--- | :--- |
| `CAMERA_BUFFER_ORDER` | `"RGB"` or `"BGR"` | Declares actual incoming camera buffer order before preprocessing. |

If camera buffer is BGR:
- convert BGR to RGB before `predict()`
- convert RGB back to BGR before `cv2.imshow()`

## 8. Output Fields for Debug and Tuning

`SCNNWrapper.get_metrics()` returns:
- `lane_ratio`: lane-pixel fraction in full binary mask
- `top_lane_ratio`: lane-pixel fraction in top band
- `bottom_lane_ratio`: lane-pixel fraction in bottom band
- `lane_center_x`: lane center estimate in bottom band

Other outputs:
- `binaryPred`: binary lane mask (`0` or `255`)
- `instancePred`: RGB colored lane mask
- `fps`: lane model inference FPS
