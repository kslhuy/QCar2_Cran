
import numpy as np
import sys
from pathlib import Path

# Add path
current_file = Path(__file__).resolve()
sys.path.insert(0, str(current_file.parent))
sys.path.insert(0, str(current_file.parent.parent))

from firstLayerObserverBase import create_first_layer_observer

def test_z_observer():
    print("Testing ZLayer1Observer instantiation...")
    
    # Dummy gains (8 vertices, 3x2)
    L_vertices = [np.eye(3, 2)*0.1 for _ in range(8)]
    
    observer = create_first_layer_observer(
        'z_layer1', 
        sample_time=0.01,
        observer_gains={'L_vertices': L_vertices, 'tau': 10.0}
    )
    
    print("Observer instantiated successfully.")
    
    # Dummy measurement [vx, r, psi, X, Y, ay]
    meas = np.array([1.0, 0.1, 0.0, 0.0, 0.0, 0.0])
    
    # Dummy control [delta, a]
    u = np.array([0.05, 0.1])
    
    print("Running update step...")
    state, w = observer.update(meas, u)
    
    print(f"State: {state}")
    print(f"Unknown Input (w): {w}")
    
    assert state.shape == (6,)
    assert w.shape == (2,)
    
    print("Test passed!")

if __name__ == "__main__":
    test_z_observer()
