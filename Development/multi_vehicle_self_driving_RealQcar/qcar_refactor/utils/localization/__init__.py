"""Platform-neutral localisation algorithms.

The package deliberately contains no PAL, CARLA, or ROS 2 imports.  Platform
resource contexts own native sensors and IO adapters publish core vehicle
contracts; algorithms can then be replayed and tested off-vehicle.
"""

from .localization_recording import plot_laser_scan_artifact, write_laser_scan_artifact
from .scan_matching import ReferenceScanLidarLocalizer, ScanMatchAlgorithm, ScanMatchingLidarOdometry

__all__ = [
    "ScanMatchAlgorithm",
    "ReferenceScanLidarLocalizer",
    "ScanMatchingLidarOdometry",
    "plot_laser_scan_artifact",
    "write_laser_scan_artifact",
]
