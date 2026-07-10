"""Lane-data normalization helpers for path-following states."""

from typing import Any, Dict, Optional


def extract_lane_data(
    yolo_data: Optional[Dict[str, Any]],
    min_confidence: float = 0.1,
) -> Dict[str, Any]:
    """
    Convert raw YOLO lane payload into a normalized dictionary.

    Args:
        yolo_data: Raw YOLO payload with lane_* keys.
        min_confidence: Confidence threshold to mark lane data as valid.

    Returns:
        Dict with normalized lane fields used by state logic.
    """
    if not yolo_data:
        return {
            "valid": False,
            "confidence": 0.0,
            "steering": 0.0,
            "curvature": 0.0,
            "offset": 0.0,
            "left_detected": False,
            "right_detected": False,
        }

    confidence = float(yolo_data.get("lane_confidence", 0.0))
    return {
        "valid": confidence > min_confidence,
        "confidence": confidence,
        "steering": float(yolo_data.get("lane_steering", 0.0)),
        "curvature": float(yolo_data.get("lane_slope", 0.0)),
        "offset": float(yolo_data.get("lane_intercept", 0.0)),
        "left_detected": bool(yolo_data.get("lane_left_detected", False)),
        "right_detected": bool(yolo_data.get("lane_right_detected", False)),
    }

