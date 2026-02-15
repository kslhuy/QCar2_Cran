# How to Run YOLO Probing

The `multi_probing.py` script now monitors a **single vehicle** per instance. This allows you to open multiple terminals and arrange windows as you like.

## Usage

Run the script from the `python` directory or root, specifying the `--car` ID.

### Monitor Car 0
```powershell
python multi_probing.py --car 0
```

### Monitor Car 1 (in a new terminal)
```powershell
python multi_probing.py --car 1
```

### Options
- `--car [ID]`: The ID of the car to monitor (default: 0).
- `--scale [FACTOR]`: Scale the window size (e.g., `0.5` for half size, `2.0` for double).

### Example
To monitor Car 2 with a smaller window:
```powershell
python multi_probing.py --car 2 --scale 0.75
```
