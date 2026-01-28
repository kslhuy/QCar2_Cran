
import sys
import os
import numpy as np
from pathlib import Path
from omegaconf import OmegaConf

# Add paths
current_dir = Path(__file__).parent.resolve()
sys.path.append(str(current_dir))

from vehiclemodels.vehicle_dynamics_qlpv import compute_tire_forces_pacejka, compute_tire_forces_linear, _compute_normal_loads
from vehiclemodels.vehicle_parameters import VehicleParameters

from vehiclemodels.vehicle_parameters import setup_vehicle_parameters


def test_stiffness():
    print("Checking Tire Stiffness Mismatch...")
    
    # Load parameters manually as in fake_vehicle_real_logic
    # params_dir = current_dir / "Development" / "multi_vehicle_self_driving_RealQcar" / "qcar" / "GUI" / "vehiclemodels" / "parameters"
    # params_dir = current_dir
    # qcar_conf = OmegaConf.load(str(params_dir / "parameters_qcar.yaml"))
    # tire_conf = OmegaConf.load(str(params_dir / "parameters_tire.yaml"))
    # structured_conf = OmegaConf.structured(VehicleParameters)
    # p = OmegaConf.to_object(OmegaConf.merge(structured_conf, tire_conf, qcar_conf))

    p = setup_vehicle_parameters(vehicle_id='qcar')
    
    print(f"Loaded Parameters:")
    print(f"  Cf (Linear): {p.Cf}")
    print(f"  p_ky1 (Pacejka Coeff): {p.tire.p_ky1}")
    print(f"  Mass: {p.m}")
    
    # Test conditions
    vx = 1.0
    alpha = 0.01 # Small slip angle (linear region)
    
    # Compute Normal Load (Static)
    Fz_f, Fz_r = _compute_normal_loads(p.m, p.a, p.b, p.h_cg, 0.0)
    print(f"  Fz_f: {Fz_f:.4f} N  and Fz_r: {Fz_r:.4f} N")
    
    # 1. Compute Linear Force using Cf
    Fy_linear_f, Fy_linear_r = compute_tire_forces_linear(alpha, alpha, p.Cf, p.Cr)
    linear_stiffness_f = Fy_linear_f / alpha
    linear_stiffness_r = Fy_linear_r / alpha
    print(f"  Linear Force (alpha={alpha}): {Fy_linear_f:.4f} N front and {Fy_linear_r:.4f} N rear")
    print(f"  Implied Linear Stiffness: {linear_stiffness_f:.4f} N/rad front and {linear_stiffness_r:.4f} N/rad rear")
    
    # 2. Compute Pacejka Force
    # We need to mock the p object to have 'tire' attribute if it doesn't - but OmegaConf object works
    Fy_pacejka_f, Fy_pacejka_r = compute_tire_forces_pacejka(alpha, alpha, p, vx, 0.0)
    pacejka_stiffness_f = Fy_pacejka_f / alpha
    pacejka_stiffness_r = Fy_pacejka_r / alpha
    print(f"  Pacejka Force (alpha={alpha}): {Fy_pacejka_f:.4f} N front and {Fy_pacejka_r:.4f} N rear")
    print(f"  Implied Pacejka Stiffness: {pacejka_stiffness_f:.4f} N/rad front and {pacejka_stiffness_r:.4f} N/rad rear")
    
    print("-" * 30)
    if abs(linear_stiffness_f - pacejka_stiffness_f) > 10.0:
        print("FAIL: Massive Discrepancy detected!")
        ratio = pacejka_stiffness_f / linear_stiffness_f
        print(f"Ratio (Pacejka/Linear): {ratio:.4f}")
    else:
        print("PASS: Stiffnesses match reasonably well.")

    if abs(linear_stiffness_r - pacejka_stiffness_r) > 10.0:
        print("FAIL: Massive Discrepancy detected!")
        ratio = pacejka_stiffness_r / linear_stiffness_r
        print(f"Ratio (Pacejka/Linear): {ratio:.4f}")
    else:
        print("PASS: Stiffnesses match reasonably well.")

if __name__ == "__main__":
    test_stiffness()
