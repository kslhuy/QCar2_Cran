import numpy as np
import os
import sys

# Add path to the directory containing 'qcar' to imports
# Path structure: .../qcar/Observer/ShengyaObs
# We need to go up 3 levels
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
if project_root not in sys.path:
    sys.path.append(project_root)

from qcar.Observer.ShengyaObs.distributed_luenberger_estimator import DistributedLuenbergerEstimator
from qcar.Observer.fleet_state_estimators import ConsensusFleetEstimator

class MockLogger:
    def __init__(self):
        self.logger = self
    def info(self, msg): print(f"INFO: {msg}")
    def warning(self, msg): print(f"WARNING: {msg}")
    def log_error(self, msg, e): print(f"ERROR: {msg} - {e}")

def test_consensus_delegation():
    print("Testing Consensus Delegation...")
    
    # Initialize estimator for vehicle 99 (which has consensus config)
    fleet_size = 4
    mock_logger = MockLogger()
    estimator = DistributedLuenbergerEstimator(vehicle_id=0, fleet_size=fleet_size, logger=mock_logger)
    
    print(f"Estimator vehicle_id: {estimator.vehicle_id}")
    print(f"Consensus estimator instance: {estimator.consensus_estimator}")
    
    assert estimator.consensus_estimator is not None, "Consensus estimator should be initialized"
    assert isinstance(estimator.consensus_estimator, ConsensusFleetEstimator), "Should be ConsensusFleetEstimator"
    
    print(f"Consensus gain: {estimator.consensus_estimator.consensus_gain}")
    assert estimator.consensus_estimator.consensus_gain == 0.5, "Consensus gain should be 0.5"

    print(f"m_i: {estimator.m_i}")
    assert estimator.m_i == 0.5, "m_i should be 0.5"
    # m_i: 0.5
    # tau_i: 0.16
    # rho_i: 0.12
    # Cd_i: 0.035
    # AF_i: 0.22
    # mu_i: 0.01

    # Test update
    local_state = np.zeros(5)
    dt = 0.1
    current_time_ns = 1000
    control = np.array([0.0, 0.1])
    
    print("Running update...")
    fleet_states = estimator.update(local_state, dt, current_time_ns, control)
    
    print(f"Fleet states shape: {fleet_states.shape}")
    assert fleet_states.shape == (5, fleet_size), f"Wrong shape: {fleet_states.shape}"
    
    print("Test passed!")

if __name__ == "__main__":
    try:
        test_consensus_delegation()
    except Exception as e:
        print(f"Test failed: {e}")
        import traceback
        traceback.print_exc()
