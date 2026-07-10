"""
Test script for the new unified NeuralObsRecorder
"""

import os
import sys
import time
import numpy as np
import shutil
from pathlib import Path

# Add parent directory to path to import LocalNeuralObs
current_dir = Path(__file__).parent.absolute()
parent_dir = current_dir.parent
sys.path.insert(0, str(parent_dir))

from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Observer.LocalNeuralObs import NeuralObsRecorder

def test_1layer_recording():
    print("\nTesting 1-layer recording...")
    output_dir = "test_recordings"
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    
    recorder = NeuralObsRecorder(output_dir=output_dir, name="test_1layer", mode='1layer')
    filepath = recorder.start()
    print(f"Started recording to: {filepath}")
    
    assert recorder.is_recording()
    assert recorder.get_mode() == '1layer'
    
    # Simulate some data
    for i in range(10):
        t = i * 0.1
        state = np.array([1.0, 0.0, 0.1*i, 0.0, float(i), 0.0]) # 6D
        unknown_input = np.array([0.5, -0.5]) # 2D
        measurements = {'vx': 1.0, 'X': float(i), 'Y': 0.0}
        
        recorder.record_1layer(
            t=t,
            state_6d=state,
            unknown_input=unknown_input,
            measurements=measurements,
            steering=0.1,
            throttle=0.2,
            gps_valid=True
        )
    
    count = recorder.stop()
    print(f"Recorded {count} samples")
    assert count == 10
    
    # Verify file content
    with open(filepath, 'r') as f:
        header = f.readline().strip()
        print(f"Header: {header}")
        assert 'w_r' in header
        assert 'w_f' in header
        assert 'w_r_nn' not in header # Should not be in 1-layer

def test_2layer_recording():
    print("\nTesting 2-layer recording...")
    output_dir = "test_recordings"
    
    recorder = NeuralObsRecorder(output_dir=output_dir, name="test_2layer", mode='2layer')
    filepath = recorder.start()
    print(f"Started recording to: {filepath}")
    
    assert recorder.is_recording()
    assert recorder.get_mode() == '2layer'
    
    # Simulate some data
    for i in range(10):
        t = i * 0.1
        state = np.array([1.0, 0.0, 0.1*i, 0.0, float(i), 0.0])
        nn_outputs = np.array([0.1, -0.1])
        uio_state = np.array([1.0, 0.0, 0.1*i, 0.0, float(i), 0.0])
        measurements = {'vx': 1.0, 'X': float(i), 'Y': 0.0}
        
        recorder.record_2layer(
            t=t,
            state_6d=state,
            measurements=measurements,
            nn_outputs=nn_outputs,
            uio_state=uio_state,
            steering=0.1,
            throttle=0.2,
            loss=0.01,
            gps_valid=True
        )
    
    count = recorder.stop()
    print(f"Recorded {count} samples")
    assert count == 10
    
    # Verify file content
    with open(filepath, 'r') as f:
        header = f.readline().strip()
        print(f"Header: {header}")
        assert 'w_r_nn' in header
        assert 'vx_uio' in header

if __name__ == "__main__":
    try:
        test_1layer_recording()
        test_2layer_recording()
        print("\nAll tests passed!")
    except Exception as e:
        print(f"\nTest failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
