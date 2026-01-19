# Understanding the BEV Lane Following System

## Overview

Your lane following system uses a **4-stage pipeline** with multiple visualization windows to help you tune the parameters.

---

## The 4 Visualization Windows

### 1. **"Result" Window** - Main View
**What it shows:**
- Original camera image
- Green overlay showing detected lane area
- Red trapezoid showing the BEV transformation region
- Control information (offset, curvature, steering)

**Purpose:** See the final result and verify lane detection is working

---

### 2. **"Binary Mask" Window** - Preprocessing
**What it shows:**
- Black and white image
- White pixels = detected lane markers (white/yellow lines)
- Black pixels = everything else

**Purpose:** Verify that lane markers are being detected correctly
- Should show clear white lines where lanes are
- Minimal noise/clutter

**How to tune:**
- Adjust `S Min/Max` trackbars (saturation thresholds)
- Adjust `SX Min/Max` trackbars (gradient edge detection)

---

### 3. **"Warped / Sliding Windows" Window** - Bird's Eye View ⭐
**What it shows:**
- **Bird's Eye View transformation** of the binary mask
- Road appears as if looking straight down from above
- **Green rectangles** = sliding windows searching for lane pixels
- Parallel lane lines should appear as vertical lines

**Purpose:** This is the KEY window for tuning!
- Lane lines should be **PARALLEL** (vertical)
- If lines are tilted/converging, adjust warp parameters

**How to tune:**
- Adjust `Warp Top W`, `Warp Bot W`, `Warp H`, `Warp Y Off` trackbars
- Goal: Make lane lines appear parallel and vertical

---

### 4. **"Controls" Window** - Trackbar Panel
**What it shows:**
- All tunable parameters as sliders

---

## Complete Parameter Guide

### **Group 1: Bird's Eye View Transformation**

These control the perspective transformation (trapezoid → rectangle).

| Parameter | What it does | How to tune |
|-----------|-------------|-------------|
| **Warp Top W** | Width of trapezoid at top (far away) | Increase if lane lines converge too much at top |
| **Warp Bot W** | Width of trapezoid at bottom (near car) | Adjust to match lane width near car |
| **Warp H** | Height of trapezoid (how far ahead to look) | Increase to see further ahead, decrease for nearby focus |
| **Warp Y Off** | Vertical offset from bottom | Adjust if you want to exclude hood/bottom of image |

**Tuning Strategy:**
1. Look at the **red trapezoid** on the "Result" window
2. Adjust parameters so trapezoid covers the lane area
3. Check "Warped" window - lane lines should be **parallel and vertical**

---

### **Group 2: Lane Detection Thresholds**

These control what pixels are considered "lane markers" in the binary mask.

| Parameter | What it does | Default | How to tune |
|-----------|-------------|---------|-------------|
| **S Min** | Minimum saturation (color intensity) | 50 | Increase to ignore faded/gray areas |
| **S Max** | Maximum saturation | 255 | Usually keep at max |
| **SX Min** | Minimum gradient strength (edge detection) | 20 | Increase to ignore weak edges |
| **SX Max** | Maximum gradient strength | 100 | Decrease if detecting too many edges |

**Tuning Strategy:**
1. Look at "Binary Mask" window
2. White pixels should appear ONLY on lane markers
3. If too much noise: increase `S Min` or `SX Min`
4. If missing lanes: decrease `S Min` or `SX Min`

---

### **Group 3: Control Gains**

These control how aggressively the car steers.

| Parameter | What it does | Default | How to tune |
|-----------|-------------|---------|-------------|
| **Steer Gain** | Proportional gain for lane offset | 1.0 (×10) | Increase for sharper corrections, decrease if oscillating |
| **Curve Gain** | Feedforward gain for curvature | 200 | Increase for better curve handling |
| **Inv Curve** | Invert curvature term | 1 (on) | Toggle if car steers wrong direction on curves |

**Tuning Strategy:**
1. Start with low gains
2. Gradually increase until car follows lane smoothly
3. If car oscillates (wobbles), decrease gains

---

## Hardcoded Parameters (In Code)

These are set in the `preprocess()` and `find_lanes_sliding_window()` methods:

### **Lane Detection (HSV Thresholds)**
Located in `preprocess()` method around line 130-145:

```python
# Black road detection
black_upper = np.array([180, 255, 80])  # Brightness threshold

# White lane markers
white_lower = np.array([0, 0, 180])     # Brightness 180-255
white_upper = np.array([180, 60, 255])  # Low saturation

# Yellow lane markers
yellow_lower = np.array([15, 60, 120])  # Hue 15-40
yellow_upper = np.array([40, 255, 255])
```

**When to adjust:**
- If white lines not detected: lower `white_lower[2]` from 180 to 150
- If yellow lines not detected: widen hue range (e.g., 10-45)
- If detecting too much: increase thresholds

---

### **Confidence Validation**
Located in `find_lanes_sliding_window()` method around line 200-295:

```python
min_peak_strength = 100      # Histogram peak threshold
min_pixels_for_lane = 200    # Minimum pixels to fit lane
min_lane_width = 100         # Minimum lane width (pixels)
max_lane_width = 600         # Maximum lane width (pixels)
```

**When to adjust:**
- If rejecting valid lanes: decrease thresholds
- If hallucinating lanes: increase thresholds

---

### **Single-Lane Offset**
Located in `calculate_curvature_and_offset()` around line 342-349:

```python
lane_center = left_x + 250   # Follow 250px right of left lane
lane_center = right_x - 250  # Follow 250px left of right lane
```

**When to adjust:**
- If car drives too close to lane marker: increase offset (e.g., 300)
- If car drives too far from lane: decrease offset (e.g., 200)

---

## Tuning Workflow

### **Step 1: Get Binary Mask Right**
1. Run the program
2. Look at "Binary Mask" window
3. Adjust `S Min/Max` and `SX Min/Max` until only lane markers are white
4. Minimize noise

### **Step 2: Calibrate Bird's Eye View**
1. Look at "Result" window - adjust red trapezoid to cover lanes
2. Adjust `Warp Top W`, `Warp Bot W`, `Warp H`, `Warp Y Off`
3. Look at "Warped / Sliding Windows" - lane lines should be **parallel**
4. This is the MOST IMPORTANT step!

### **Step 3: Tune Control Gains**
1. Enable auto mode (press X)
2. Adjust `Steer Gain` and `Curve Gain`
3. Car should follow lane smoothly without oscillating

### **Step 4: Save Parameters**
1. Press **P** to save all trackbar settings
2. Settings saved to `lane_params_bev.json`
3. Auto-loaded next time you run the program

---

## Common Issues

### **Issue: Lane lines not parallel in "Warped" window**
**Solution:** Adjust warp parameters until lines are vertical and parallel

### **Issue: Too much noise in "Binary Mask"**
**Solution:** Increase `S Min` or `SX Min` thresholds

### **Issue: Missing lane markers in "Binary Mask"**
**Solution:** 
- Decrease `S Min` or `SX Min`
- Or adjust HSV thresholds in code (white_lower, yellow_lower)

### **Issue: Car oscillates/wobbles**
**Solution:** Decrease `Steer Gain`

### **Issue: Car doesn't turn enough on curves**
**Solution:** Increase `Curve Gain`

### **Issue: Hallucinating lanes when off-road**
**Solution:** Increase confidence thresholds in code:
- `min_peak_strength` (line 204)
- `min_pixels_for_lane` (line 254)

---

## Quick Reference: What Each Window Shows

```
┌─────────────────────┐  ┌─────────────────────┐
│   "Result"          │  │  "Binary Mask"      │
│                     │  │                     │
│  Camera view with   │  │  Black/white image  │
│  green lane overlay │  │  showing detected   │
│  and red trapezoid  │  │  lane markers       │
└─────────────────────┘  └─────────────────────┘

┌─────────────────────┐  ┌─────────────────────┐
│ "Warped / Sliding   │  │   "Controls"        │
│  Windows"           │  │                     │
│                     │  │  Trackbar sliders:  │
│  Bird's eye view    │  │  - Warp params      │
│  with green search  │  │  - Thresholds       │
│  windows            │  │  - Control gains    │
└─────────────────────┘  └─────────────────────┘
```
