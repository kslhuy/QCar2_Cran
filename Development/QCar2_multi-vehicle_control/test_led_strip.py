"""
Test LED Strip Control on QCar2 (Simulation)

Requirements:
  - QLabs must be running with a QCar2 spawned
  - Conda environment: Qcar
  - Run AFTER initCars.py or initPlatoon.py has spawned the cars

This script demonstrates BOTH LED systems:
  1. LED Color Strip (underglow) - via QVL actor
  2. Signal LEDs (headlights, turn signals, brake, reverse) - via PAL QCar

Usage:
  conda activate Qcar
  python test_led_strip.py
"""

import time
import sys
import numpy as np

# =====================================================================
# PART 1: LED Color Strip (QVL — simulation only)
# =====================================================================
print("=" * 60)
print("PART 1: LED Color Strip Control (QVL)")
print("=" * 60)

try:
    from qvl.qlabs import QuanserInteractiveLabs
    from qvl.qcar2 import QLabsQCar2

    # Connect to QLabs
    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    qlabs.open("localhost")
    print("Connected!")

    # Get reference to an already-spawned QCar2 (actor number 0)
    car = QLabsQCar2(qlabs)
    car.actorNumber = 0  # Change this to match your spawned car

    # Color presets (values > 1 increase glow effect)
    COLORS = {
        "red":     [40, 0, 0],
        "green":   [0, 40, 0],
        "blue":    [0, 0, 40],
        "yellow":  [40, 40, 0],
        "cyan":    [0, 40, 40],
        "magenta": [40, 0, 40],
        "white":   [30, 30, 30],
        "off":     [0, 0, 0],
    }

    print("\n--- Cycling through LED strip colors ---")
    for name, color in COLORS.items():
        print(f"  Setting LED strip to: {name} = {color}")
        result = car.set_led_strip_uniform(color=color)
        print(f"  Result: {'OK' if result else 'FAILED'}")
        time.sleep(1.5)

    # Demo: individual LED control (rainbow effect)
    print("\n--- Individual LED control (gradient) ---")
    individual_colors = []
    for i in range(33):
        r = (i / 33.0) * 40
        g = ((33 - i) / 33.0) * 40
        b = 20
        individual_colors.append([r, g, b])

    result = car.set_led_strip_individual(color=individual_colors)
    print(f"  Rainbow gradient result: {'OK' if result else 'FAILED'}")
    time.sleep(2)

    # Reset to off
    car.set_led_strip_uniform(color=[0, 0, 0])
    print("\nLED strip test complete!")

    qlabs.close()

except ImportError as e:
    print(f"  QVL not available: {e}")
    print("  Skipping LED strip test (only works in simulation with QLabs)")
except Exception as e:
    print(f"  Error: {e}")
    print("  Make sure QLabs is running and a QCar2 is spawned")

print()

# =====================================================================
# PART 2: Signal LEDs (PAL QCar — works in both sim and physical)
# =====================================================================
print("=" * 60)
print("PART 2: Signal LEDs Control (PAL QCar)")
print("=" * 60)
print("""
The signal LEDs are controlled via QCar.write() or QCar.read_write_std().
The LEDs parameter is a numpy array of 8 values:

  Index  Function                    Digital Channels
  -----  --------------------------  ----------------
  [0]    Left front turn signal      ch 17
  [1]    Right front turn signal     ch 18
  [2]    Left rear turn signal       ch 25
  [3]    Right rear turn signal      ch 26
  [4]    Brake lights (x4)           ch 11,12,13,14
  [5]    Reverse lights (x2)         ch 15,16
  [6]    Left headlamps (x3)         ch 19,20,21
  [7]    Right headlamps (x3)        ch 22,23,24

Example usage in your code:

  from pal.products.qcar import QCar
  import numpy as np

  with QCar() as qcar:
      # Headlights ON
      LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
      qcar.read_write_std(throttle=0.0, steering=0.0, LEDs=LEDs)

      # Left turn signal
      LEDs = np.array([1, 0, 1, 0, 0, 0, 1, 1])
      qcar.read_write_std(throttle=0.0, steering=0.0, LEDs=LEDs)

      # Brake lights
      LEDs = np.array([0, 0, 0, 0, 1, 0, 1, 1])
      qcar.read_write_std(throttle=0.0, steering=0.0, LEDs=LEDs)

      # All OFF
      LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
      qcar.read_write_std(throttle=0.0, steering=0.0, LEDs=LEDs)
""")

print("=" * 60)
print("SUMMARY")
print("=" * 60)
print("""
LED Color Strip (underglow, 33 RGB LEDs):
  - Simulation:  QLabsQCar2.set_led_strip_uniform(color=[R, G, B])
  - Physical:    ROS2 param  ros2 param set /qcar2 led_color_id <0-5>
                 (0=red, 1=green, 2=blue, 3=yellow, 4=cyan, 5=magenta)

Signal LEDs (headlights, turn signals, brake, reverse):
  - Both:        QCar.write(throttle, steering, LEDs=np.array([...]))
                 QCar.read_write_std(throttle, steering, LEDs=np.array([...]))
""")
