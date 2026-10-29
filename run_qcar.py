"""Launch QCar from a source checkout on Windows or Linux, from any directory."""
from pathlib import Path
import sys

if __name__ == "__main__":
    source = Path(__file__).resolve().parent / "Development" / "multi_vehicle_self_driving_RealQcar"
    sys.path.insert(0, str(source))
    from qcar.__main__ import main
    raise SystemExit(main())
