
import sys
from pathlib import Path
import numpy as np

# Add paths to ensure imports work
current_dir = Path("c:/Users/Quang Huy Nugyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/LocalNeuralObs/2LayerObs")
sys.path.insert(0, str(current_dir))
sys.path.insert(0, str(current_dir.parent))

from Design_LMI_neural import NeuralQLPVGainScheduler
from qlpv_vehicle_dynamics_obs import get_default_vehicle_params

def test_scheduler_failure():
    print("Testing NeuralQLPVGainScheduler with params from yaml...")
    
    # Parameters mirroring neural_obs_params.yaml
    params = get_default_vehicle_params()
    
    # Configuration from yaml
    vx_range = (0.5, 2.0)
    delta_max = 0.5
    n_vx_vertices = 3
    n_delta_vertices = 3
    sample_time = 0.01
    contraction_rate = 0.99
    hinf_gamma = 50.0  # From yaml
    lmi_decay_rate = 0.5
    
    # "use_common_lyapunov": false in yaml, but let's check both or specifically the one failing
    use_common_lyapunov = False 
    disturbance_mode = 'general' # From yaml
    
    scheduler = NeuralQLPVGainScheduler(
        vehicle_params=params,
        vx_range=vx_range,
        delta_max=delta_max,
        n_vx_vertices=n_vx_vertices,
        n_delta_vertices=n_delta_vertices,
        decay_rate=lmi_decay_rate,
        lmi_method='qlpv_scheduled', 
        hinf_gamma=hinf_gamma,
        use_common_lyapunov=use_common_lyapunov,
        discrete=True,
        sample_time=sample_time,
        contraction_rate=contraction_rate,
        verbose=True,
        disturbance_mode=disturbance_mode
    )
    
    print(f"Scheduler configured. Computing gains...")
    try:
        success = scheduler.compute_gains_lmi()
        print(f"Success: {success}")
        if not success:
            print("Scheduler failed to compute gains (returned False).")
    except Exception as e:
        print(f"Scheduler raised exception: {e}")

if __name__ == "__main__":
    test_scheduler_failure()
