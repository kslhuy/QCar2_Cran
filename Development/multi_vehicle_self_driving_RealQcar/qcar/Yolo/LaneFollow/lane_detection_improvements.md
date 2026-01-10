# Lane Detection Improvements for Black Roads

## Summary

Updated [QCar2_lane_following_new.py](file:///c:/Users/Quang%20Huy%20Nugyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/Yolo/LaneFollow/QCar2_lane_following_new.py) to handle black road surfaces with white dashed lines and yellow center lines, added image cropping, and implemented camera offset calibration.

---

## Changes Made

### 1. **Multi-Step Lane Detection for Black Roads**

Replaced the `preprocess()` method with a 6-step detection pipeline optimized for black road surfaces:

#### **Step 1: Detect Black Road Surface**
```python
black_lower = np.array([0, 0, 0])
black_upper = np.array([180, 255, 80])  # Dark surfaces
black_mask = cv2.inRange(hsv, black_lower, black_upper)
```
Creates a region of interest by identifying dark road surfaces (brightness 0-80 in HSV).

#### **Step 2-3: Detect Lane Markers**
- **White markers** (dashed lines): HSV range `[0, 0, 180]` to `[180, 60, 255]`
- **Yellow markers** (center line): HSV range `[15, 60, 120]` to `[40, 255, 255]`

#### **Step 4: Morphological Filtering**
```python
# Close gaps in dashed lines
white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
# Remove small noise
white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel, iterations=1)
```
- **MORPH_CLOSE**: Connects dashed line segments
- **MORPH_OPEN**: Removes small noise blobs

#### **Step 5: Road-Based Filtering** ⭐
```python
road_area = cv2.dilate(black_mask, kernel, iterations=5)
white_mask = cv2.bitwise_and(white_mask, road_area)
```
**Key improvement**: Only keeps lane markers that are within or near the black road area, filtering out white objects outside the lanes.

#### **Step 6: Combine Detections**
Merges white and yellow masks, plus gradient detection for additional robustness.

---

### 2. **Image Cropping**

Added 40-pixel bottom crop to remove noise/artifacts from the camera feed:

```python
# Crop 40 pixels from bottom to remove noise/artifacts
cropped_rgb = frame[:-40, :, :] if frame.shape[0] > 40 else frame
frame = cropped_rgb
```

Applied at line 465-467, before any image processing.

---

### 3. **Camera Offset Calibration**

Implemented compensation for camera being 0.032m to the left of vehicle center:

```python
# In __init__
self.camera_offset_m = 0.032  # meters to the left

# In calculate_curvature_and_offset
camera_offset_pixels = (self.camera_offset_m / 0.5) * lane_visible_width
car_position = (img_width / 2) - camera_offset_pixels
```

**How it works**:
- Assumes lane width ≈ 0.5m corresponds to `lane_visible_width` pixels
- Converts 0.032m offset to pixels: `(0.032 / 0.5) * lane_visible_width`
- Adjusts car position calculation to account for camera being left of center

---

## Key Benefits

### **Better Noise Rejection**
The road-based filtering (Step 5) prevents false detections from white objects outside the lane boundaries.

### **Robust to Lane Marker Types**
Handles both:
- White dashed lines (on turns/circles)
- Yellow center lines (separating lanes)

### **Improved Accuracy**
- Morphological operations connect dashed lines for better curve fitting
- Camera offset calibration corrects for physical camera placement

---

## Testing Recommendations

1. **Verify Detection Quality**
   - Check the "Binary Mask" window to see what's being detected
   - White markers should appear in cyan, yellow in yellow

2. **Tune HSV Thresholds** (if needed)
   - Adjust brightness threshold for black road (currently 80)
   - Adjust white detection range if too sensitive/insensitive
   - Modify yellow hue range (15-40) for different lighting

3. **Validate Camera Calibration**
   - Test on straight sections with visible center line
   - Verify car stays centered in lane
   - Adjust `camera_offset_m` if needed (currently 0.032m)

4. **Monitor Performance**
   - Ensure morphological operations don't introduce lag
   - Check if gradient detection helps or adds noise

---

## Configuration Parameters

The following trackbar parameters remain available for tuning:

| Parameter | Purpose | Default |
|-----------|---------|---------|
| `SX Min/Max` | Gradient edge detection thresholds | 20-100 |
| `Steer Gain` | Proportional control gain | 1.0 |
| `Curve Gain` | Curvature feedforward gain | 200.0 |

HSV thresholds are now hardcoded in the `preprocess()` method but can be exposed as parameters if needed.

---

## Update: Confidence Validation (Anti-Hallucination)

### Problem
The sliding window algorithm was **hallucinating lanes** in two scenarios:
1. **Off-road**: Car completely off the road, but algorithm still "finds" phantom lanes
2. **Sharp turns**: Only one lane marker visible, but algorithm creates a fake second lane

### Solution: Three-Layer Confidence Checks

#### **Check 1: Histogram Peak Validation**
```python
min_peak_strength = 100  # Minimum pixels in histogram peak
left_lane_detected = left_peak_strength > min_peak_strength
```
- Validates that histogram peaks are strong enough to indicate real lane markers
- Weak peaks (< 100 pixels) are rejected as noise

#### **Check 2: Minimum Pixel Count**
```python
min_pixels_for_lane = 200  # Need at least this many pixels to trust detection
if len(leftx) > min_pixels_for_lane and left_lane_detected:
    left_fit = np.polyfit(lefty, leftx, 2)
```
- Requires at least 200 pixels to fit a lane polynomial
- Prevents fitting curves to sparse noise

#### **Check 3: Lane Width Sanity Check**
```python
min_lane_width = 100  # pixels
max_lane_width = 600  # pixels
if lane_width < min_lane_width or lane_width > max_lane_width:
    # Reject the weaker detection
```
- When both lanes are detected, validates they're a reasonable distance apart
- Rejects unrealistic lane widths (too narrow or too wide)
- Keeps the stronger detection, discards the weaker one

### Single-Lane Handling
When only one lane is detected (e.g., on sharp turns):
- **Left lane only**: Follow at 250px offset to the right
- **Right lane only**: Follow at 250px offset to the left
- No longer creates phantom second lanes

### Visual Feedback
- Sliding window rectangles only drawn for **validated** lanes
- Green rectangles = confident detection
- Missing rectangles = lane rejected due to low confidence

