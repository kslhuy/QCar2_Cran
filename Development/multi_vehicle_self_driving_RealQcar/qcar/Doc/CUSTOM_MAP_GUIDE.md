# Custom Roadmap Configuration Guide

## Problem Summary

You have a **custom roadmap with only nodes 0-10**, not the original SDCS roadmap (which has nodes up to 23).

## Issue Fixed

### Original Error
```
TypeError: 'NoneType' object is not subscriptable
```

This happened because:
1. Your `valid_nodes` list had invalid/duplicate nodes
2. `roadmap.generate_path()` couldn't create a path with those nodes
3. Returned `None` which caused the error when trying to get length

## Changes Made

### 1. Updated `config.py` (Lines 77-82)

**Before (Your version with duplicates):**
```python
if self.node_configuration == 0:
    return [10, 2, 4, 6, 8, 10, 0]  # ❌ Duplicate 10
else:
    return [10, 1, 5, 3, 8, 10]      # ❌ Duplicate 10
```

**After (Fixed for nodes 0-10):**
```python
if self.node_configuration == 0:
    return [10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0]  # ✅ All nodes 0-10
else:
    return [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]   # ✅ Alternative
```

### 2. Fixed `vehicle_controller.py` 

**Changed `num_nodes` back to original:**
```python
num_nodes = random.randint(6, 10)  # Was incorrectly changed to (5, 6)
```

**Added comprehensive validation:**
- Check if roadmap.generate_path() returns None
- Validate waypoint array type and shape
- Better error messages showing which nodes failed

### 3. Created Helper Script: `test_roadmap_nodes.py`

Run this to discover which nodes are actually valid in your roadmap:

```bash
python test_roadmap_nodes.py
```

This will:
- Test nodes 0-25 to see which exist
- Try different node sequences
- Tell you which sequences work
- Give you a list of valid nodes to use

## How to Configure for Your Custom Map

### Step 1: Discover Valid Nodes

```bash
cd qcar
python test_roadmap_nodes.py
```

Example output:
```
Valid nodes list: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
```

### Step 2: Test Path Generation

The script will test sequences like:
- `[0, 1, 2, 3, 0]` → ✓ SUCCESS
- `[0, 5, 10, 0]` → ✓ SUCCESS
- `[0, 2, 4, 6, 8, 10, 0]` → May fail if connections don't exist

### Step 3: Update `config.py`

Edit the `valid_nodes` property with nodes that work:

```python
@property
def valid_nodes(self) -> List[int]:
    if self.node_configuration == 0:
        # Use nodes that form valid paths in your map
        return [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    else:
        # Alternative configuration
        return [10, 8, 6, 4, 2, 0]
```

### Step 4: (Optional) Create Custom Config File

Use `config_custom_map.yaml` and specify:
```yaml
path:
  node_configuration: 0  # Which valid_nodes list to use
  calibration_pose: [0, 2, -1.5708]  # Adjust for your map
```

## Understanding Node Selection

### How It Works

1. **Get valid nodes**: From `config.path.valid_nodes`
2. **Shuffle**: Randomize order
3. **Select subset**: Pick 6-10 nodes randomly
4. **Add loop**: Append first node to end to close the path
5. **Generate waypoints**: Call `roadmap.generate_path(node_sequence)`

### Example

```python
valid_nodes = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0]
random.shuffle(valid_nodes)  # → [3, 7, 0, 10, 5, 2, 9, 1, 4, 8, 6]
num_nodes = 7  # Random 6-10
node_sequence = [3, 7, 0, 10, 5, 2, 9, 3]  # First 7 + close loop
```

## Important Notes

### 1. Not All Nodes Connect
Just because nodes 0-10 exist doesn't mean they all connect to each other. Your roadmap may only have specific connections.

### 2. Path Must Be Valid
The roadmap must have roads/connections between consecutive nodes in your sequence.

### 3. Test Before Deploying
Always run `test_roadmap_nodes.py` before using a new configuration.

## Troubleshooting

### Error: "Failed to generate path for node sequence"

**Cause**: The nodes you selected don't have valid connections in the roadmap.

**Solution**:
1. Run `test_roadmap_nodes.py`
2. Find working node sequences
3. Update `valid_nodes` in `config.py` with only working nodes

### Error: "Waypoint sequence has invalid shape"

**Cause**: `generate_path()` returned something but it's malformed.

**Solution**:
- Check your roadmap definition
- Verify node connections
- May need to recreate roadmap file

### Logs Show: "Valid nodes from config: [...]"

This is the list being used. Verify it matches your roadmap.

## Testing Your Configuration

### Test 1: Validate Nodes
```bash
python test_roadmap_nodes.py
```

### Test 2: Dry Run
```bash
python vehicle_control_refactored.py --config config_custom_map.yaml
```

Watch the logs for:
```
[INFO] Valid nodes from config: [...]
[INFO] Selected node sequence (X nodes): [...]
[INFO] Generated path with X waypoints
```

### Test 3: Single Vehicle
Start with one vehicle to verify configuration before multi-vehicle.

## Recommended Configuration for Nodes 0-10

Based on typical roadmap connectivity:

```python
# config.py - Option 1: Include all nodes (system will select subset)
@property
def valid_nodes(self) -> List[int]:
    if self.node_configuration == 0:
        return [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    else:
        return [10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0]

# config.py - Option 2: Only sequential nodes (safer)
@property
def valid_nodes(self) -> List[int]:
    if self.node_configuration == 0:
        return [0, 2, 4, 6, 8, 10]  # Even nodes only
    else:
        return [1, 3, 5, 7, 9]      # Odd nodes only
```

## Next Steps

1. ✅ Fixed configuration errors
2. ✅ Added validation and error handling
3. ⏳ **Run `test_roadmap_nodes.py` to find your valid nodes**
4. ⏳ Update `config.py` with working node list
5. ⏳ Test with single vehicle
6. ⏳ Deploy multi-vehicle

## Summary

The issue was that your `valid_nodes` configuration didn't match your actual roadmap. Now you have:

- ✅ Fixed node configuration for 0-10 range
- ✅ Better error handling and validation
- ✅ Tool to discover valid nodes (`test_roadmap_nodes.py`)
- ✅ Comprehensive logging to debug issues
- ✅ Custom config file template

Run the test script first to find which nodes work in your specific roadmap!
