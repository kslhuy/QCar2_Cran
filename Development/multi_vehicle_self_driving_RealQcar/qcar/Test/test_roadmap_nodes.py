"""
Test script to verify which nodes exist in your roadmap
Run this to find valid node sequences for your custom map
"""
from hal.products.mats import SDCSRoadMap
import numpy as np

print("="*70)
print(" Roadmap Node Validator")
print("="*70)

# Create roadmap
roadmap = SDCSRoadMap(leftHandTraffic=False)

print("\nTesting nodes 0-25...")
print("\nValid nodes (that can be retrieved):")

valid_nodes = []
for i in range(26):  # Test nodes 0-25
    try:
        pose = roadmap.get_node_pose(i)
        if pose is not None:
            valid_nodes.append(i)
            print(f"  Node {i:2d}: Position = [{pose[0][0]:.3f}, {pose[1][0]:.3f}], Orientation = {pose[2][0]:.3f}")
    except Exception as e:
        print(f"  Node {i:2d}: ✗ Not available ({type(e).__name__})")

print(f"\n{'='*70}")
print(f"Valid nodes list: {valid_nodes}")
print(f"Total valid nodes: {len(valid_nodes)}")

# Test path generation with different sequences
print(f"\n{'='*70}")
print("Testing path generation with different node sequences...")

test_sequences = [
    [10, 2, 4, 6, 8, 1],
    [10, 1, 5, 3, 8 , 10],
]

for seq in test_sequences:
    try:
        waypoints = roadmap.generate_path(seq)
        if waypoints is not None and isinstance(waypoints, np.ndarray):
            print(f"  {seq} → ✓ SUCCESS ({waypoints.shape[1]} waypoints)")
        else:
            print(f"  {seq} → ✗ FAILED (returned None or wrong type)")
    except Exception as e:
        print(f"  {seq} → ✗ ERROR ({type(e).__name__}: {e})")

print(f"\n{'='*70}")
print("Recommendations:")
print("1. Use only nodes from the 'Valid nodes list' above")
print("2. Test your specific sequence before using it in config")
print("3. Update config.py valid_nodes property with working nodes")
print("="*70)
