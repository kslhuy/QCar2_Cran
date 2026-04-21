import cv2
import numpy as np
import torch
from pit.YOLO.nets import YOLOv8, MASK_COLORS_RGB
from pit.YOLO.utils import TrafficLight, Obstacle


from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class DetectionBuffers:
    """Pre-allocated detection buffers for efficient packet building."""

    BUFFER_SIZE = 7
    CLASS_NAMES = ["stop_sign", "traffic", "car", "yield", "person", "lane", "obstacle"]

    # Obstacle type constants
    OBSTACLE_NONE: float = 0.0
    OBSTACLE_PERSON: float = 1.0  # Person in center box → stop & wait
    OBSTACLE_CONE: float = 2.0  # Cone / other in center box → replan path

    stop_sign: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    traffic: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    car: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    yield_sign: np.ndarray = field(
        default_factory=lambda: np.zeros(7, dtype=np.float64)
    )
    person: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    lane: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    # Obstacle buffer: [0]=count, [1]=closest_dist, [2]=obstacle_type, [3..6]=reserved
    obstacle: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))

    # Image dimensions for center-box and lane-side filtering
    image_width: int = 640
    image_height: int = 480
    # Horizontal offset sign convention:
    # +1.0 => positive offset is image-right, -1.0 => positive offset is image-left.
    offset_sign: float = 1.0
    # Stop/yield lane-side filter mode: "right" (default), "left", or "both".
    sign_filter_side: str = "right"

    # Center-box proportions (configurable)
    center_box_width_ratio: float = 0.40  # Middle 25% of image width
    center_box_height_ratio: float = 0.60  # Bottom 60% of image height
    # Ignore detections whose center lies in this bottom strip (hood/capo area).
    bottom_ignore_px: int = 0
    # Move center box upward by this many pixels.
    center_box_raise_px: int = 0
    # Positive values are subtracted from raw/fused distances before publishing.
    distance_offset_m: float = 0.0

    @classmethod
    def from_config(
        cls,
        image_width: int,
        image_height: int,
        offset_sign: float = 1.0,
        sign_filter_side: str = "right",
        config: Optional[Dict[str, Any]] = None,
    ) -> "DetectionBuffers":
        """Factory to build buffers from a runtime config section."""
        cfg = config or {}
        center_w = float(np.clip(cfg.get("center_box_width_ratio", 0.40), 0.05, 1.0))
        center_h = float(np.clip(cfg.get("center_box_height_ratio", 0.60), 0.05, 1.0))
        bottom_ignore_px = int(max(0, cfg.get("bottom_ignore_px", 0)))
        center_box_raise_px = int(max(0, cfg.get("center_box_raise_px", 0)))
        distance_offset_m = float(cfg.get("distance_offset_m", 0.0))
        return cls(
            image_width=int(image_width),
            image_height=int(image_height),
            offset_sign=float(offset_sign),
            sign_filter_side=str(sign_filter_side),
            center_box_width_ratio=center_w,
            center_box_height_ratio=center_h,
            bottom_ignore_px=bottom_ignore_px,
            center_box_raise_px=center_box_raise_px,
            distance_offset_m=distance_offset_m,
        )

    def reset(self):
        """Reset all buffers to zero."""
        self.stop_sign.fill(0)
        self.traffic.fill(0)
        self.car.fill(0)
        self.yield_sign.fill(0)
        self.person.fill(0)
        self.lane.fill(0)
        self.obstacle.fill(0)

    def _get_x_center(self, det, bbox=None) -> float:
        """Get detection center x using bbox when available."""
        if bbox is not None:
            return float((bbox[0] + bbox[2]) / 2.0)
        return float(det.x)

    def _is_right_side(self, det, bbox=None) -> bool:
        """Check if detection center is on the right half of the image.

        In right-hand traffic, signs on our current lane appear on the right side
        of the camera image. Signs on the left side are for the opposite lane.

        Args:
            det: Obstacle object with x attribute (top-left x of bbox)
            bbox: Optional [x1, y1, x2, y2] bounding box array for more accurate center
        """
        x_center = self._get_x_center(det, bbox)
        return x_center >= (self.image_width / 2.0)

    def _is_sign_on_allowed_side(self, det, bbox=None) -> bool:
        """Check stop/yield sign side filter based on configured traffic side."""
        side = str(self.sign_filter_side).lower()
        if side == "both":
            return True
        x_center = self._get_x_center(det, bbox)
        mid = self.image_width / 2.0
        if side == "left":
            return x_center <= mid
        # Default to right-side filtering for invalid values.
        return x_center >= mid

    def _is_in_center_box(self, bbox) -> bool:
        """Check if a bounding box overlaps the center-front region of the image.

        The center box represents the road directly ahead of the car:
        - Horizontally: middle `center_box_width_ratio` of the image
        - Vertically: bottom `center_box_height_ratio` of the image

        Args:
            bbox: [x1, y1, x2, y2] bounding box in pixel coordinates

        Returns:
            True if the bbox center falls inside the center-front box
        """
        if bbox is None:
            return False

        cx = (bbox[0] + bbox[2]) / 2
        cy = (bbox[1] + bbox[3]) / 2

        # Horizontal bounds: center portion of image
        margin_x = (1.0 - self.center_box_width_ratio) / 2
        x_min = self.image_width * margin_x
        x_max = self.image_width * (1.0 - margin_x)

        # Vertical bounds: bottom portion of image (road area)
        y_min = self.image_height * (1.0 - self.center_box_height_ratio)
        y_max = self.image_height - self.bottom_ignore_px
        # Shift center region upward when needed.
        y_min -= self.center_box_raise_px
        y_max -= self.center_box_raise_px
        y_min = max(0.0, y_min)
        y_max = min(float(self.image_height), y_max)
        if y_max <= y_min:
            return False

        return x_min <= cx <= x_max and y_min <= cy <= y_max

    def _is_in_bottom_ignore_strip(self, bbox) -> bool:
        """Return True if bbox center is in the ignored bottom strip."""
        if bbox is None or self.bottom_ignore_px <= 0:
            return False
        cy = (bbox[1] + bbox[3]) / 2.0
        return cy >= (self.image_height - self.bottom_ignore_px)

    def _correct_distance(self, distance) -> float:
        """Apply configured distance offset while preserving invalid zero values."""
        try:
            dist = float(np.asarray(distance).reshape(-1)[0])
        except Exception:
            return 0.0
        if not np.isfinite(dist) or dist <= 0.0:
            return 0.0
        return float(max(0.01, dist - float(self.distance_offset_m)))

    def fill_from_results(self, results: list, bounding_boxes=None):
        """Fill buffers from YOLO detection results.

        Applies lane-side filtering: stop sign and yield sign detections on the
        left half of the image (opposite lane) are filtered out.

        Also checks for center-box obstacles (person or cone-like objects
        directly in the car's path) and populates the obstacle buffer.

        Args:
            results: List of Obstacle/TrafficLight objects from post_processing
            bounding_boxes: Optional [N, 4] array of bounding boxes (xyxy format)
        """
        counts = {"car": 0, "stop_sign": 0, "traffic": 0, "yield": 0, "person": 0}

        # Center-box obstacle tracking
        obstacle_count = 0
        closest_obstacle_dist = 100.0
        closest_obstacle_type = self.OBSTACLE_NONE
        closest_obstacle_offset = 0.0

        # Names that are NOT treated as in-path obstacles (signs, traffic lights)
        sign_keywords = {
            "stop sign",
            "red",
            "green",
            "yellow",
            "idle",
            "yield",
            "traffic light",
        }

        traffic_vehicle_keywords = ("car", "truck", "bus", "motorcycle", "bicycle")
        cone_like_keywords = ("cone", "traffic cone", "pylon", "bollard", "fire hydrant")

        for idx, det in enumerate(results):
            name = det.name
            name_l = str(name).lower()
            corrected_dist = self._correct_distance(getattr(det, "distance", 0.0))
            if corrected_dist > 0.0:
                try:
                    det.distance = corrected_dist
                except Exception:
                    pass
            bbox = (
                bounding_boxes[idx]
                if bounding_boxes is not None and idx < len(bounding_boxes)
                else None
            )
            
            # Calculate horizontal offset if we have a bounding box
            offset = 0.0
            if bbox is not None:
                cx = (bbox[0] + bbox[2]) / 2.0
                # Offset normalized to [-1.0, 1.0], where 0 is center, -1 is left edge, 1 is right edge
                raw_offset = (cx - (self.image_width / 2.0)) / (self.image_width / 2.0)
                offset = float(
                    np.clip(raw_offset * float(self.offset_sign), -1.0, 1.0)
                )
            else:
                try:
                    raw_offset = (float(det.x) - (self.image_width / 2.0)) / (
                        self.image_width / 2.0
                    )
                    offset = float(
                        np.clip(raw_offset * float(self.offset_sign), -1.0, 1.0)
                    )
                except Exception:
                    offset = 0.0

            if any(kw in name_l for kw in traffic_vehicle_keywords):
                # Ignore hood/capo false positives near image bottom.
                if self._is_in_bottom_ignore_strip(bbox):
                    continue
                counts["car"] += 1
                dist = corrected_dist
                # Keep closest detection
                if counts["car"] == 1 or dist < self.car[1]:
                    self.car[1] = dist
                    self.car[2] = offset
            elif "stop sign" in name_l:
                # Lane-side filter for stop signs.
                if not self._is_sign_on_allowed_side(det, bbox):
                    continue
                counts["stop_sign"] += 1
                dist = corrected_dist
                if counts["stop_sign"] == 1 or dist < self.stop_sign[1]:
                    self.stop_sign[1] = dist
                    self.stop_sign[2] = offset
            elif "red" in name_l:
                counts["traffic"] += 1
                dist = corrected_dist
                if counts["traffic"] == 1 or dist < self.traffic[1]:
                    self.traffic[1] = dist
                    self.traffic[2] = offset
            elif "yield" in name_l:
                # Lane-side filter for yield signs.
                if not self._is_sign_on_allowed_side(det, bbox):
                    continue
                counts["yield"] += 1
                dist = corrected_dist
                if counts["yield"] == 1 or dist < self.yield_sign[1]:
                    self.yield_sign[1] = dist
                    self.yield_sign[2] = offset
            elif "person" in name_l:
                counts["person"] += 1
                dist = corrected_dist
                if counts["person"] == 1 or dist < self.person[1]:
                    self.person[1] = dist
                    self.person[2] = offset

            # --- Center-box obstacle check ---
            # Check if this detection is in the center-front box of the image.
            # Person → stop & wait; non-sign object → cone / replan.
            if bbox is not None and self._is_in_center_box(bbox):
                is_sign = any(kw in name_l for kw in sign_keywords)
                if not is_sign:
                    dist = corrected_dist
                    if dist <= 0.0:
                        dist = 100.0
                    if dist < closest_obstacle_dist:
                        closest_obstacle_dist = dist
                        closest_obstacle_offset = offset
                        if "person" in name_l:
                            closest_obstacle_type = self.OBSTACLE_PERSON
                        elif any(kw in name_l for kw in cone_like_keywords):
                            closest_obstacle_type = self.OBSTACLE_CONE
                        else:
                            closest_obstacle_type = self.OBSTACLE_CONE
                    obstacle_count += 1

        # Set counts in first element
        self.car[0] = counts["car"]
        self.traffic[0] = counts["traffic"]
        self.stop_sign[0] = counts["stop_sign"]
        self.yield_sign[0] = counts["yield"]
        self.person[0] = counts["person"]

        # Obstacle buffer: [0]=count, [1]=closest_dist, [2]=type (1=person, 2=cone)
        self.obstacle[0] = obstacle_count
        self.obstacle[1] = closest_obstacle_dist if obstacle_count > 0 else 0.0
        self.obstacle[2] = closest_obstacle_type
        # Preserve closest obstacle lateral offset for downstream path planning.
        self.obstacle[3] = closest_obstacle_offset if obstacle_count > 0 else 0.0

    def fill_lane(self, lane_result):
        """Fill lane buffer from lane detection result."""
        if lane_result is not None and lane_result.is_valid:
            self.lane[0] = lane_result.confidence
            self.lane[1] = lane_result.steering_correction
            self.lane[2] = lane_result.curvature if lane_result.curvature else 0.0
            self.lane[3] = (
                lane_result.lateral_offset if lane_result.lateral_offset else 0.0
            )
            self.lane[4] = 1.0 if lane_result.left_lane_detected else 0.0
            self.lane[5] = 1.0 if lane_result.right_lane_detected else 0.0
            self.lane[6] = 0.0  # Reserved

    def to_packet(self) -> np.ndarray:
        """Create send packet from all buffers. Shape: (7, 7)."""
        return np.vstack(
            (
                self.stop_sign,
                self.traffic,
                self.car,
                self.yield_sign,
                self.person,
                self.lane,
                self.obstacle,
            )
        )


class YOLOv8Wrapper_Huy(YOLOv8):
    """
    Enhanced YOLOv8 wrapper with unified rendering for both object detection and lane detection.

    Features:
    - Original YOLO post_processing and rendering
    - Integrated lane detection overlay
    - Unified post_process_render that combines both YOLO and lane visualization
    - Support for custom lane detector instances
    """

    def __init__(
        self,
        imageWidth=640,
        imageHeight=480,
        modelPath=None,
        runtime_config: Optional[Dict[str, Any]] = None,
    ):
        """Initialize the enhanced YOLOv8 wrapper"""
        super().__init__(imageWidth, imageHeight, modelPath)
        self.lane_result = None
        self.lane_detector = None
        self._tracking_initialized = False
        runtime = runtime_config or {}
        self.default_use_tracking = bool(runtime.get("use_tracking", True))
        self.default_classes = list(runtime.get("classes", [2, 9, 11, 33]))
        self.default_confidence = float(runtime.get("confidence", 0.3))
        self.default_half = bool(runtime.get("half", True))
        self.default_verbose = bool(runtime.get("verbose", False))
        tracker_cfg = runtime.get("tracker", {}) or {}
        self.tracker_name = str(tracker_cfg.get("name", "botsort.yaml"))
        self.tracker_persist = bool(tracker_cfg.get("persist", True))
        self.postprocess_clipping_distance_m = float(
            runtime.get("postprocess_clipping_distance_m", 10.0)
        )
        print("YOLOv8Wrapper_Huy initialized with unified rendering + tracking support")

    def track(
        self,
        inputImg,
        classes=None,
        confidence=None,
        verbose=None,
        half=None,
    ):
        """Use YOLOv8 tracking mode (BoT-SORT) for persistent object IDs.

        Same interface as predict() but uses ultralytics built-in tracker with
        persist=True for cross-frame ID continuity. Falls back to predict()
        if tracking fails (e.g., TensorRT engine compatibility).

        Args:
            inputImg: Pre-processed input image.
            classes: COCO class IDs to detect.
            confidence: Minimum detection confidence.
            verbose: Print inference logs.
            half: Use FP16 inference.

        Returns:
            Ultralytics Results object.
        """
        classes = self.default_classes if classes is None else classes
        confidence = self.default_confidence if confidence is None else confidence
        verbose = self.default_verbose if verbose is None else verbose
        half = self.default_half if half is None else half

        try:
            self.predictions = self.net.track(
                inputImg,
                verbose=verbose,
                imgsz=(self.imageHeight, self.imageWidth),
                classes=classes,
                conf=confidence,
                half=half,
                persist=self.tracker_persist,
                tracker=self.tracker_name,
            )
            self.objectsDetected = self.predictions[0].boxes.cls.cpu().numpy()
            self.FPS = 1000 / self.predictions[0].speed["inference"]
            self._tracking_initialized = True
            return self.predictions[0]
        except Exception as e:
            if not self._tracking_initialized:
                print(f"[YOLO] Tracking init failed, falling back to predict: {e}")
            # Fallback to standard predict
            return self.predict(
                inputImg=inputImg,
                classes=classes,
                confidence=confidence,
                verbose=verbose,
                half=half,
            )

    def run_inference(
        self,
        inputImg,
        use_tracking=None,
        classes=None,
        confidence=None,
        verbose=None,
        half=None,
    ):
        """Run tracking or prediction with configured defaults."""
        use_tracking = (
            self.default_use_tracking if use_tracking is None else bool(use_tracking)
        )
        classes = self.default_classes if classes is None else classes
        confidence = self.default_confidence if confidence is None else confidence
        verbose = self.default_verbose if verbose is None else verbose
        half = self.default_half if half is None else half

        if use_tracking:
            return self.track(
                inputImg=inputImg,
                classes=classes,
                confidence=confidence,
                verbose=verbose,
                half=half,
            )
        return self.predict(
            inputImg=inputImg,
            classes=classes,
            confidence=confidence,
            verbose=verbose,
            half=half,
        )

    def post_process_detections(self, alignedDepth=None, clippingDistance=None):
        """Run post-processing with configurable clipping distance."""
        clip = (
            self.postprocess_clipping_distance_m
            if clippingDistance is None
            else clippingDistance
        )
        return self.post_processing(alignedDepth=alignedDepth, clippingDistance=clip)

    def set_lane_detector(self, lane_detector):
        """Set the lane detector instance for integrated rendering"""
        self.lane_detector = lane_detector

    def set_lane_result(self, lane_result):
        """Set the current lane detection result for rendering"""
        self.lane_result = lane_result

    def post_process_render(
        self,
        showFPS=True,
        bbox_thickness=4,
        lane_result=None,
        lane_detector=None,
        show_lane_overlay=True,
    ):
        """
        Unified rendering method that combines YOLO detections and lane detection.

        Args:
            showFPS (bool): Display FPS in the rendered image
            bbox_thickness (int): Thickness of bounding box outlines
            lane_result: LaneDetectionResult object (uses self.lane_result if None)
            lane_detector: Lane detector instance (uses self.lane_detector if None)
            show_lane_overlay (bool): Whether to show lane detection overlay

        Returns:
            ndarray: Annotated image with both YOLO detections and lane overlay
        """
        # Use provided or stored lane detection info
        lane_result = lane_result or self.lane_result
        lane_detector = lane_detector or self.lane_detector

        # First, render YOLO detections (original implementation)
        if not self.processedResults:
            # No YOLO detections, just resize input image
            if self.img.shape[:2] != self.inputShape:
                output_img = cv2.resize(
                    self.img, (self.inputShape[1], self.inputShape[0])
                )
            else:
                output_img = self.img.copy()

            # Apply lane overlay even without YOLO detections
            if show_lane_overlay and lane_result is not None:
                if lane_detector is not None and hasattr(
                    lane_detector, "render_lane_overlay"
                ):
                    output_img = lane_detector.render_lane_overlay(
                        output_img, lane_result
                    )
                else:
                    output_img = self._render_basic_lane_overlay(
                        output_img, lane_result
                    )

            return output_img

        # Render YOLO detections with masks
        colors = []
        masks = None
        if self.predictions[0].masks is not None:
            masks = self.predictions[0].masks.data
            if torch.cuda.is_available():
                masks = masks.cuda()
            else:
                masks = masks.cpu()

            if masks.shape[1:] != self.img.shape[:2]:
                import torch.nn.functional as F

                masks = F.interpolate(
                    masks.unsqueeze(1).float(),
                    size=self.img.shape[:2],
                    mode="nearest",
                ).squeeze(1)

        boxes = self.predictions[0].boxes.xyxy.cpu().numpy().astype(int)
        imgClone = self.img.copy()

        render_count = min(len(self.objectsDetected), len(self.processedResults), len(boxes))
        for i in range(render_count):
            class_idx = int(self.objectsDetected[i]) % len(MASK_COLORS_RGB)
            color = tuple(int(channel) for channel in MASK_COLORS_RGB[class_idx][:3])
            colors.append(color)
            name = str(self.processedResults[i].name)
            bbox = np.asarray(boxes[i], dtype=int).reshape(-1)
            if bbox.size < 4:
                continue
            pt1 = (int(bbox[0]), int(bbox[1]))
            pt2 = (int(bbox[2]), int(bbox[3]))
            x = int(np.asarray(self.processedResults[i].x).reshape(-1)[0])
            y = int(np.asarray(self.processedResults[i].y).reshape(-1)[0])
            distance = float(self.processedResults[i].distance)

            cv2.rectangle(imgClone, pt1, pt2, color, int(bbox_thickness))
            cv2.putText(
                imgClone,
                name,
                (x, max(y - 30, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )

            if self._calc_distence:
                cv2.putText(
                    imgClone,
                    f"{distance:.2f} m",
                    (x, max(y - 10, 40)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2,
                )

        if showFPS:
            cv2.putText(
                imgClone,
                "Inference FPS: " + str(round(self.FPS)),
                (480, 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )

        # Apply mask coloring when segmentation masks are available.
        if masks is not None and len(colors) > 0:
            imgTensor = torch.from_numpy(imgClone)
            if torch.cuda.is_available():
                imgTensor = imgTensor.to("cuda:0")
            imgMask = self.mask_color(masks, imgTensor, colors)
        else:
            imgMask = imgClone

        if imgMask.shape[:2] != self.inputShape:
            imgMask = cv2.resize(imgMask, (self.inputShape[1], self.inputShape[0]))

        # Apply lane detection overlay on top of YOLO rendering
        if show_lane_overlay and lane_result is not None:
            if lane_detector is not None and hasattr(
                lane_detector, "render_lane_overlay"
            ):
                imgMask = lane_detector.render_lane_overlay(imgMask, lane_result)
            else:
                imgMask = self._render_basic_lane_overlay(imgMask, lane_result)

        return imgMask

    def _render_basic_lane_overlay(self, image: np.ndarray, lane_result) -> np.ndarray:
        """
        Render basic lane detection overlay when no detector instance is available.

        Args:
            image: Input image to overlay lane detection on
            lane_result: LaneDetectionResult object

        Returns:
            Image with lane detection overlay
        """
        overlay = image.copy()

        if not lane_result.is_valid:
            return overlay

        # Choose color based on confidence
        color = (0, 255, 0) if lane_result.confidence > 0.5 else (0, 255, 255)

        # Draw steering indicator arrow at bottom of image
        center_x = image.shape[1] // 2
        start_y = image.shape[0] - 30
        end_x = int(center_x + lane_result.steering_correction * 200)
        cv2.arrowedLine(
            overlay, (center_x, start_y), (end_x, start_y), color, 3, tipLength=0.3
        )

        # Draw lane detection information
        lane_info = f"Lane: Steer={lane_result.steering_correction:.3f} Conf={lane_result.confidence:.2f}"
        cv2.putText(
            overlay,
            lane_info,
            (10, image.shape[0] - 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )

        # Draw lateral offset if available
        if lane_result.lateral_offset is not None:
            offset_text = f"Offset: {lane_result.lateral_offset:.3f}"
            cv2.putText(
                overlay,
                offset_text,
                (10, image.shape[0] - 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

        # Draw lane detection indicators
        indicator_y = image.shape[0] - 100
        if lane_result.left_lane_detected:
            cv2.circle(overlay, (20, indicator_y), 8, (0, 255, 0), -1)
            cv2.putText(
                overlay,
                "L",
                (15, indicator_y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

        if lane_result.right_lane_detected:
            cv2.circle(overlay, (50, indicator_y), 8, (0, 255, 0), -1)
            cv2.putText(
                overlay,
                "R",
                (45, indicator_y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

        return overlay

    def render_combined(self, showFPS=True, show_yolo=True, show_lane=True):
        """
        Convenience method for rendering with flexible control over what to show.

        Args:
            showFPS: Show FPS counter
            show_yolo: Show YOLO detection overlays
            show_lane: Show lane detection overlays

        Returns:
            Rendered image
        """
        if show_yolo:
            return self.post_process_render(
                showFPS=showFPS, show_lane_overlay=show_lane
            )
        else:
            # Just show raw image with lane overlay
            img = self.img.copy()
            if show_lane and self.lane_result is not None:
                if self.lane_detector is not None and hasattr(
                    self.lane_detector, "render_lane_overlay"
                ):
                    img = self.lane_detector.render_lane_overlay(img, self.lane_result)
                else:
                    img = self._render_basic_lane_overlay(img, self.lane_result)
            return img
