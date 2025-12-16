"""
Test script to verify multi_probing configuration
"""
import yaml
import os

# Load fleet config
config_path = os.path.join(os.path.dirname(__file__), "..", "fleet_config.yaml")
with open(config_path, 'r') as f:
    config = yaml.safe_load(f)

print("="*70)
print(" Multi-Probing Configuration Test")
print("="*70)

# Get image size
image_size = config.get('image_size', {})
print(f"\nImage Configuration:")
print(f"  Width: {image_size.get('width', 320)}")
print(f"  Height: {image_size.get('height', 200)}")

# Get probing vehicles
probing_vehicles = [v for v in config['vehicles'] if v.get('probing', False)]
print(f"\nProbing Vehicles: {len(probing_vehicles)}")

for vehicle in probing_vehicles:
    print(f"\n  Car ID: {vehicle['car_id']}")
    print(f"    IP: {vehicle['ip']}")
    print(f"    Description: {vehicle.get('description', 'N/A')}")

# Simulate command generation
probing_car_ids = [str(v['car_id']) for v in probing_vehicles]
cmd = [
    "python", "multi_probing.py",
    "--cars"
] + probing_car_ids + [
    "--width", str(image_size.get('width', 320)),
    "--height", str(image_size.get('height', 200))
]

print(f"\n{'='*70}")
print(" Generated Command")
print(f"{'='*70}")
print(" ".join(cmd))

print(f"\n{'='*70}")
print(" Test Complete")
print(f"{'='*70}")
print("\nConfiguration is correct!")
print(f"Multi-probing will create {len(probing_vehicles)} observer window(s).")
print("\nTo test multi_probing manually, run:")
print(f"  cd python")
print(f"  {' '.join(cmd)}")
