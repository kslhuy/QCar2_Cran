#!/usr/bin/env python3
"""
Test script to verify that the consensus term is no longer always zero.
Tests the fix to _transfer_fleet_states_to_estimated_states function.
"""

import numpy as np
import sys
import os

# Add workspace to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'Development', 'multi_vehicle_self_driving_RealQcar'))

from qcar.Observer.ShengyaObs.distributed_luenberger_estimator import DistributedLuenbergerEstimator

def test_consensus_with_different_neighbor_states():
    """Test that consensus term is non-zero when neighbor states differ"""
    
    # Configuration for a 3-vehicle fleet
    config = {
        'observer_gain': np.array([
            [0.1, 0.0],
            [0.0, 0.1],
            [0.5, 0.5],
            [0.1, 0.0],
            [0.0, 0.1],
            [0.5, 0.5],
        ]),
        'consensus_gain': np.eye(6) * 0.2,
        'adjacency_matrix': [
            [0, 1, 0],
            [1, 0, 1],
            [0, 1, 0]
        ]
    }
    
    # Create observer for vehicle 2 in 3-vehicle fleet
    estimator = DistributedLuenbergerEstimator(
        vehicle_id=2,
        fleet_size=3,
        config=config
    )
    
    # Scenario 1: Self and neighbor have same states (should still give 0 consensus)
    fleet_states_same = np.array([
        [0.0, 1.0, 2.0],      # x
        [0.0, 0.0, 0.0],      # y
        [0.0, 0.0, 0.0],      # theta
        [0.0, 1.0, 1.5],      # v
        [0.0, 0.0, 0.0]       # a
    ])
    
    estimator.fleet_states = fleet_states_same.copy()
    
    # Get relative state for self
    x_vec_self, _ = estimator._transfer_fleet_states_to_estimated_states(
        fleet_states_same, 0
    )
    
    # Scenario 2: Neighbor has different velocities
    fleet_states_neighbor = np.array([
        [0.0, 0.5, 1.5],      # x (different positions)
        [0.0, 0.0, 0.0],      # y
        [0.0, 0.0, 0.0],      # theta
        [0.0, 0.8, 1.2],      # v (different velocities!)
        [0.0, 0.2, -0.1]      # a (different accelerations!)
    ])
    
    # Get relative state for neighbor
    x_vec_neighbor, _ = estimator._transfer_fleet_states_to_estimated_states(
        fleet_states_neighbor, 0
    )
    
    # Calculate consensus difference
    consensus_diff = x_vec_self - x_vec_neighbor
    consensus_diff_norm = np.linalg.norm(consensus_diff)
    
    print("=" * 70)
    print("CONSENSUS TERM FIX VERIFICATION TEST")
    print("=" * 70)
    print(f"\nScenario: Vehicle 2 in 3-vehicle fleet")
    print(f"Self state norm: {np.linalg.norm(x_vec_self):.6f}")
    print(f"Neighbor state norm: {np.linalg.norm(x_vec_neighbor):.6f}")
    print(f"\nConsensus difference norm: {consensus_diff_norm:.6f}")
    
    if consensus_diff_norm > 1e-6:
        print("\n✓ SUCCESS: Consensus difference is NON-ZERO")
        print(f"  This means the fix is working correctly!")
        print(f"  The function now properly uses the passed-in fleet_states parameter")
        return True
    else:
        print("\n✗ FAILURE: Consensus difference is still ZERO")
        print(f"  The function may still be ignoring the parameter")
        return False

if __name__ == "__main__":
    success = test_consensus_with_different_neighbor_states()
    sys.exit(0 if success else 1)
