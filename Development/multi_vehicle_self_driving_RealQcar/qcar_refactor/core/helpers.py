import numpy as np

def copy_safe(data: dict) -> dict:
    """Shallow dict copy with deep copies of numpy arrays."""
    result = dict(data)
    for k, v in result.items():
        if isinstance(v, np.ndarray):
            result[k] = v.copy()
    return result