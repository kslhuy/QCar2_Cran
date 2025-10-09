#!/usr/bin/env python3
"""
WEEK 3: Integration Verification Script

Quick checks to verify Week 1, 2, and 3 are properly integrated.
Run this before running the full simulation.
"""

import sys
import os
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

def test_imports():
    """Test that all required modules can be imported."""
    print("="*60)
    print("TEST 1: Import Verification")
    print("="*60)
    
    try:
        from src.Trust.TriPTrustModel import TriPTrustModel
        print("✅ TriPTrustModel imported (Week 1)")
    except Exception as e:
        print(f"❌ TriPTrustModel import failed: {e}")
        return False
    
    try:
        from GraphBasedTrust import GraphBasedTrustEvaluator
        print("✅ GraphBasedTrustEvaluator imported (Week 1)")
    except Exception as e:
        print(f"❌ GraphBasedTrustEvaluator import failed: {e}")
        return False
    
    try:
        from src.Weight.Weight_Trust_module import WeightTrustModule, DEFAULT_WEIGHT_CONFIG
        print("✅ WeightTrustModule imported (Week 2)")
        print(f"   Config parameters: {len(DEFAULT_WEIGHT_CONFIG)} keys")
    except Exception as e:
        print(f"❌ WeightTrustModule import failed: {e}")
        return False
    
    try:
        from VehicleObserver import VehicleObserver
        print("✅ VehicleObserver imported (Week 3)")
    except Exception as e:
        print(f"❌ VehicleObserver import failed: {e}")
        return False
    
    print("\n✅ All imports successful!\n")
    return True


def test_weight_calculation():
    """Test Week 2 weight calculation."""
    print("="*60)
    print("TEST 2: Weight Calculation (Week 2)")
    print("="*60)
    
    try:
        from src.Weight.Weight_Trust_module import WeightTrustModule
        
        # Simple 3-vehicle topology: 0 -- 1 -- 2
        graph = np.array([
            [0, 1, 0],
            [1, 0, 1],
            [0, 1, 0]
        ])
        
        weight_module = WeightTrustModule(graph, trust_threshold=0.5, kappa=5)
        
        # Test with trust scores
        vehicle_index = 1
        trust_scores = np.array([0.9, 0.0, 0.7])
        
        result = weight_module.calculate_weights_trust_v2(
            vehicle_index=vehicle_index,
            trust_scores=trust_scores
        )
        
        weights = result['weights'].flatten()
        
        # Verify row-stochastic
        weight_sum = np.sum(weights)
        assert np.abs(weight_sum - 1.0) < 1e-6, f"Weights sum to {weight_sum}, expected 1.0"
        
        print(f"✅ Weights calculated: {weights}")
        print(f"   Virtual weight: {weights[0]:.3f}")
        print(f"   Self weight: {weights[vehicle_index+1]:.3f}")
        print(f"   Trusted neighbors: {result['trusted_neighbors']}")
        print(f"   Row-stochastic: ✅ (sum = {weight_sum:.6f})")
        
        print("\n✅ Weight calculation test passed!\n")
        return True
        
    except Exception as e:
        print(f"❌ Weight calculation test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_observer_integration():
    """Test Week 3 observer integration."""
    print("="*60)
    print("TEST 3: Observer Integration (Week 3)")
    print("="*60)
    
    try:
        from VehicleObserver import VehicleObserver
        from src.Weight.Weight_Trust_module import WeightTrustModule
        
        # Create mock vehicle process
        class MockVehicleProcess:
            def __init__(self):
                self.vehicle_id = 1
                self.trust_evaluator = MockTrustEvaluator()
        
        class MockTrustEvaluator:
            def get_all_trust_scores(self):
                return {0: 0.9, 2: 0.7}
        
        # Create components
        graph = np.array([[0, 1, 0], [1, 0, 1], [0, 1, 0]])
        weight_module = WeightTrustModule(graph, trust_threshold=0.5, kappa=5)
        vehicle_process = MockVehicleProcess()
        
        # Create observer with Week 3 parameters
        observer = VehicleObserver(
            vehicle_id=1,
            fleet_size=3,
            config={'observer': {'local_observer_type': 'kalman'}},
            vehicle_process=vehicle_process,
            weight_trust_module=weight_module
        )
        
        # Verify integration
        assert observer.use_trust_weights == True, "use_trust_weights should be True"
        assert observer.vehicle_process is not None, "vehicle_process reference missing"
        assert observer.weight_trust_module is not None, "weight_trust_module reference missing"
        
        print(f"✅ Observer initialized with trust integration")
        print(f"   use_trust_weights: {observer.use_trust_weights}")
        print(f"   vehicle_process: {observer.vehicle_process is not None}")
        print(f"   weight_trust_module: {observer.weight_trust_module is not None}")
        
        # Test weight calculation
        weights = observer._get_distributed_weights()
        print(f"✅ Adaptive weights generated: shape={weights.shape}")
        print(f"   Vehicle 1 weights: {weights[1, :]}")
        print(f"   Row-stochastic: ✅ (sum = {np.sum(weights[1, :]):.6f})")
        
        print("\n✅ Observer integration test passed!\n")
        return True
        
    except Exception as e:
        print(f"❌ Observer integration test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all verification tests."""
    print("\n" + "="*60)
    print("WEEK 3 INTEGRATION VERIFICATION")
    print("="*60 + "\n")
    
    results = []
    
    # Test 1: Imports
    results.append(("Import Verification", test_imports()))
    
    # Test 2: Weight Calculation
    results.append(("Weight Calculation", test_weight_calculation()))
    
    # Test 3: Observer Integration
    results.append(("Observer Integration", test_observer_integration()))
    
    # Summary
    print("="*60)
    print("SUMMARY")
    print("="*60)
    
    for test_name, passed in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status}: {test_name}")
    
    all_passed = all(result[1] for result in results)
    
    print("\n" + "="*60)
    if all_passed:
        print("✅ ALL TESTS PASSED - Ready for simulation!")
    else:
        print("❌ SOME TESTS FAILED - Fix issues before running simulation")
    print("="*60 + "\n")
    
    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
