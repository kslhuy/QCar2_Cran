import cv2
import numpy as np
import torch
from ultralytics import YOLO
import copy
from dataclasses import dataclass, field

# Top 80 colors from xkcd color survey
MASK_COLORS_HEX=[
    "7e1e9c", "15b01a", "0343df", "ff81c0", "653700", "e50000", "95d0fc", "029386", "f97306",
    "96f97b", "c20078", "ffff14", "75bbfd", "929591", "89fe05", "bf77f6", "9a0eea", "033500",
    "06c2ac", "c79fef", "00035b", "d1b26f", "00ffff", "13eac9", "06470c", "ae7181", "35063e",
    "01ff07", "650021", "6e750e", "ff796c", "e6daa6", "0504aa", "001146", "cea2fd", "000000",
    "ff028d", "ad8150", "c7fdb5", "ffb07c", "677a04", "cb416b", "8e82fe", "53fca1", "aaff32",
    "380282", "ceb301", "ffd1df", "cf6275", "0165fc", "0cff0c", "c04e01", "04d8b2", "01153e",
    "3f9b0b", "d0fefe", "840000", "be03fd", "c0fb2d", "a2cffe", "dbb40c", "8fff9f", "580f41",
    "4b006e", "8f1402", "014d4e", "610023", "aaa662", "137e6d", "7af9ab", "02ab2e", "9aae07",
    "8eab12", "b9a281", "341c02", "36013f", "c1f80a", "fe01b1", "fdaa48", "9ffeb0",
]
MASK_COLORS_RGB = [list(int(h[i:i+2], 16) for i in (0, 2, 4)) for h in MASK_COLORS_HEX]

class Obstacle:
    def __init__(self, name='Object', distance=0.0, conf=0.0, x=0, y=0):
        self.name = name
        self.distance = distance
        self.conf = conf
        self.x = x
        self.y = y

class TrafficLight(Obstacle):
    def __init__(self, color='idle', distance=0.0):
        super().__init__(name='traffic light', distance=distance)
        self.lightColor = color

@dataclass
class DetectionBuffers:
    """Pre-allocated detection buffers for efficient packet building. Matches what physical QCar requires."""
    
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
                dist = det.distance
                # Keep closest detection
                if counts["car"] == 1 or dist < self.car[1]:
                    self.car[1] = dist
                    self.car[2] = offset
            elif "stop sign" in name_l:
                # Lane-side filter for stop signs.
                if not self._is_sign_on_allowed_side(det, bbox):
                    continue
                counts["stop_sign"] += 1
                dist = det.distance
                if counts["stop_sign"] == 1 or dist < self.stop_sign[1]:
                    self.stop_sign[1] = dist
                    self.stop_sign[2] = offset
            elif "red" in name_l:
                counts["traffic"] += 1
                dist = det.distance
                if counts["traffic"] == 1 or dist < self.traffic[1]:
                    self.traffic[1] = dist
                    self.traffic[2] = offset
            elif "yield" in name_l:
                # Lane-side filter for yield signs.
                if not self._is_sign_on_allowed_side(det, bbox):
                    continue
                counts["yield"] += 1
                dist = det.distance
                if counts["yield"] == 1 or dist < self.yield_sign[1]:
                    self.yield_sign[1] = dist
                    self.yield_sign[2] = offset
            elif "person" in name_l:
                counts["person"] += 1
                dist = det.distance
                if counts["person"] == 1 or dist < self.person[1]:
                    self.person[1] = dist
                    self.person[2] = offset

            # --- Center-box obstacle check ---
            # Check if this detection is in the center-front box of the image.
            # Person → stop & wait; non-sign object → cone / replan.
            if bbox is not None and self._is_in_center_box(bbox):
                is_sign = any(kw in name_l for kw in sign_keywords)
                if not is_sign:
                    try:
                        dist = float(det.distance)
                    except (TypeError, ValueError):
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
            self.lane[2] = lane_result.curvature if hasattr(lane_result, 'curvature') and lane_result.curvature else 0.0
            self.lane[3] = lane_result.lateral_offset if hasattr(lane_result, 'lateral_offset') and lane_result.lateral_offset else 0.0
            self.lane[4] = 1.0 if lane_result.left_lane_detected else 0.0
            self.lane[5] = 1.0 if lane_result.right_lane_detected else 0.0
            self.lane[6] = 0.0  # Reserved

    def to_packet(self) -> np.ndarray:
        """Create send packet from all buffers."""
        return np.vstack((
            self.stop_sign, self.traffic, self.car,
            self.yield_sign, self.person, self.lane, self.obstacle
        ))

class LimoYOLO:
    """
    Standalone YOLO wrapper for Limo replacing Quanser's proprietary implementation.
    """
    def __init__(self, imageWidth=640, imageHeight=480, modelPath=None):
        # Force 640x640 for OpenVINO static model
        self.imageWidth = 640
        self.imageHeight = 640
        if modelPath is None:
            import os
            self.modelPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'yolo26n-seg_openvino_model')
        else:
            self.modelPath = modelPath
        self.net = YOLO(self.modelPath, task='segment')
        self.img = np.empty((self.imageHeight, self.imageWidth, 3), dtype=np.uint8)
        self._calc_distance = False
        self.FPS = 0.0
        self.lane_result = None
        self.lane_detector = None
        print(f"LimoYOLO initialized with model: {self.modelPath} explicitly using 640x640")

    def set_lane_detector(self, lane_detector):
        """Set the lane detector instance for integrated rendering"""
        self.lane_detector = lane_detector

    def set_lane_result(self, lane_result):
        """Set the current lane detection result for rendering"""
        self.lane_result = lane_result

    def pre_process(self, inputImg):
        self.inputShape = inputImg.shape[:2]
        if inputImg.shape[:2] != (self.imageHeight, self.imageWidth):
            inputImg = cv2.resize(inputImg, (self.imageWidth, self.imageHeight))
        self.img[:,:,:] = inputImg[:,:,:]
        return self.img

    def predict(self, inputImg, classes=[0, 2, 9, 11, 33], confidence=0.3, verbose=False, half=False):
        self.predictions = self.net.predict(
            inputImg,
            verbose=verbose,
            imgsz=640,
            classes=classes,  # 0: person, 2: car, 9: traffic light, 11: stop sign, 33: suitcase(yield?)
            conf=confidence,
            half=half
        )
        self.objectsDetected = self.predictions[0].boxes.cls.cpu().numpy()
        if 'inference' in self.predictions[0].speed:
             self.FPS = 1000.0 / max(self.predictions[0].speed['inference'], 1.0)
        else:
             self.FPS = 30.0 # Default fallback
        return self.predictions[0]

    def post_processing(self, alignedDepth=None, clippingDistance=10):
        self.processedResults = []
        self._calc_distance = False

        if len(self.objectsDetected) == 0:
            return self.processedResults
            
        self.bounding = self.predictions[0].boxes.xyxy.cpu().numpy().astype(int)
        
        if alignedDepth is not None:
            if alignedDepth.shape[:2] != (self.imageHeight, self.imageWidth):
                alignedDepth = cv2.resize(alignedDepth, (self.imageWidth, self.imageHeight))
            
            # alignedDepth comes in as millimeters or meters. 
            # If Astra, it's typically 16UC1 millimeters. Quanser assumes float meters or sets distant points to 0.
            # Convert explicitly if needed or handle distance directly
            depth3D = np.dstack((alignedDepth, alignedDepth, alignedDepth))
            
            # For 16UC1 mm, clipping of 10 meters means 10000 mm.
            # Let's check max value to assume mm vs m
            if np.max(alignedDepth) > 1000:
                # It's in millimeters
                clippingValue = clippingDistance * 1000
                scale_to_meters = 0.001
            else:
                # It's already in meters
                clippingValue = clippingDistance
                scale_to_meters = 1.0

            bgRemoved = np.where((depth3D > clippingValue) | (depth3D <= 0), 0, depth3D)
            self._calc_distance = True
            
            if torch.cuda.is_available():
                self.depthTensor = torch.as_tensor(bgRemoved, device="cuda:0")
            else:
                self.depthTensor = torch.as_tensor(bgRemoved)

        for i in range(len(self.objectsDetected)):
            cls_id = int(self.objectsDetected[i])
            if cls_id == 9: # Traffic Light
                trafficBox = self.bounding[i]
                trafficLightColor = self.check_traffic_light(trafficBox, self.img)
                result = TrafficLight(color=trafficLightColor)
                result.name += (' (' + trafficLightColor + ')')
            else:
                name = self.predictions[0].names[cls_id]
                result = Obstacle(name=name)

            if alignedDepth is not None and self.predictions[0].masks is not None:
                if torch.cuda.is_available():
                    mask = self.predictions[0].masks.data.cuda()[i]
                else:
                    mask = self.predictions[0].masks.data.cpu()[i]
                
                # Failsafe resize mask if it doesn't match depthTensor exact shape
                if mask.shape != self.depthTensor.shape[:2]:
                    import torch.nn.functional as F
                    mask = F.interpolate(mask.unsqueeze(0).unsqueeze(0).float(), size=self.depthTensor.shape[:2], mode='nearest').squeeze()

                distance_raw = self.check_distance(mask, self.depthTensor[:,:,:1])
                distance = distance_raw * scale_to_meters
                if torch.is_tensor(distance):
                     distance = distance.cpu().numpy()
                result.distance = float(np.round(distance, 3))
                
            points = self.predictions[0].boxes.xyxy.cpu()[i]
            conf = self.predictions[0].boxes.conf.cpu().numpy()[i]
            x = int(points.numpy()[0])
            y = int(points.numpy()[1])
            result.x = x
            result.y = y
            result.conf = float(conf)
            self.processedResults.append(result)
            
        return self.processedResults

    def check_traffic_light(self, traffic_box, im_cpu):
        """Simplistic traffic light color checking using HSV."""
        mask = np.zeros((self.imageHeight, self.imageWidth), dtype='uint8')
        x1, y1, x2, y2 = (traffic_box[0], traffic_box[1], traffic_box[2], traffic_box[3])
        d = 0.3 * (x2 - x1)
        if d <= 0: d = 1
        
        R_center = (int(x1/2 + x2/2), int(3*y1/4 + y2/4))
        Y_center = (int(x1/2 + x2/2), int(y1/2 + y2/2))
        G_center = (int(x1/2 + x2/2), int(y1/4 + 3*y2/4))
        
        maskR = cv2.circle(copy.deepcopy(mask), R_center, int(d/2), 1, -1)
        maskY = cv2.circle(copy.deepcopy(mask), Y_center, int(d/2), 1, -1)
        maskG = cv2.circle(copy.deepcopy(mask), G_center, int(d/2), 1, -1)
        
        im_hsv = cv2.cvtColor(im_cpu, cv2.COLOR_RGB2HSV)
        
        # Calculate mean V (Brightness) in these circular regions
        value_R = np.sum(im_hsv[:,:,2] * maskR) / max(np.count_nonzero(maskR), 1)
        value_Y = np.sum(im_hsv[:,:,2] * maskY) / max(np.count_nonzero(maskY), 1)
        value_G = np.sum(im_hsv[:,:,2] * maskG) / max(np.count_nonzero(maskG), 1)
        
        mean_val = (value_R + value_Y + value_G) / 3
        threshold_perc = 0.25
        vals = [value_R, value_Y, value_G]
        val_min, val_max = min(vals), max(vals)
        
        if (val_max - val_min) < 30:
            return 'idle'
            
        threshold = (val_max - val_min) * threshold_perc
        
        redOn = (value_R > mean_val) and (value_R - mean_val) > threshold
        yellowOn = (value_Y > mean_val) and (value_Y - mean_val) > threshold
        greenOn = (value_G > mean_val) and (value_G - mean_val) > threshold
        
        traffic_light_status = [redOn, yellowOn, greenOn]
        colors = ['red', 'yellow', 'green']
        traffic_light_color = ''
        
        for i in range(len(traffic_light_status)):
            if traffic_light_status[i]:
                traffic_light_color += colors[i] + ' '
                
        return traffic_light_color.strip()

    def check_distance(self, mask, depth_tensor):
        mask = mask.unsqueeze(2)
        isolated_depth = mask * depth_tensor
        nonzero = isolated_depth[isolated_depth.nonzero(as_tuple=True)]
        if len(nonzero) > 0:
            # We use float() here to return standard types instead of PyTorch tensors where possible
            return torch.median(nonzero).float()
        return torch.tensor(0.0)

    def mask_color(self, masks, im_gpu, colors, alpha=0.5):
        if not torch.cuda.is_available():
            colors = torch.tensor(colors, dtype=torch.float32) / 255.0
        else:
            colors = torch.tensor(colors, device="cuda:0", dtype=torch.float32) / 255.0
            
        colors = colors[:, None, None]
        masks = masks.unsqueeze(3)
        masks_color = masks * (colors * alpha)
        inv_alpha_masks = (1 - masks * alpha).cumprod(0)
        mcs = masks_color.max(dim=0).values
        im_gpu = im_gpu / 255.0
        im_gpu = im_gpu * inv_alpha_masks[-1] + mcs
        im_mask = im_gpu * 255.0
        return im_mask.squeeze().byte().cpu().numpy()

    def post_process_render(self, showFPS=True, bbox_thickness=4, show_lane_overlay=False, lane_result=None, lane_detector=None):
        lane_result = lane_result or self.lane_result
        lane_detector = lane_detector or self.lane_detector
        
        if not self.processedResults:
            # No YOLO detections, just resize input image
            if self.img.shape[:2] != self.inputShape:
                output_img = cv2.resize(self.img, (self.inputShape[1], self.inputShape[0]))
            else:
                output_img = self.img.copy()
                
            # Apply lane overlay even without YOLO detections
            if show_lane_overlay and lane_result is not None:
                if lane_detector is not None and hasattr(lane_detector, "render_lane_overlay"):
                    output_img = lane_detector.render_lane_overlay(output_img, lane_result)
                else:
                    output_img = self._render_basic_lane_overlay(output_img, lane_result)
            return output_img

        colors = []
        if torch.cuda.is_available():
             masks = self.predictions[0].masks.data.cuda() if self.predictions[0].masks is not None else None
        else:
             masks = self.predictions[0].masks.data.cpu() if self.predictions[0].masks is not None else None
             
        if masks is not None and masks.shape[1:] != self.img.shape[:2]:
            import torch.nn.functional as F
            masks = F.interpolate(masks.unsqueeze(1).float(), size=self.img.shape[:2], mode='nearest').squeeze(1)

        boxes = self.predictions[0].boxes.xyxy.cpu().numpy().astype(int)
        imgClone = self.img.copy()

        for i in range(len(self.objectsDetected)):
            colors.append(MASK_COLORS_RGB[int(self.objectsDetected[i]) % len(MASK_COLORS_RGB)])
            name = self.processedResults[i].name
            x = self.processedResults[i].x
            y = self.processedResults[i].y
            distance = self.processedResults[i].distance
            
            cv2.rectangle(imgClone, tuple(boxes[i, :2]), tuple(boxes[i, 2:4]), colors[i], bbox_thickness)
            cv2.putText(imgClone, name, (x, max(y - 30, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[i], 2)
            
            if self._calc_distance:
                cv2.putText(imgClone, f"{distance:.2f} m", (x, max(y - 10, 40)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[i], 2)

        if showFPS:
            cv2.putText(imgClone, f'Inference FPS: {int(self.FPS)}', 
                        (self.imageWidth - 160, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        if masks is not None:
            if torch.cuda.is_available():
                 imgTensor = torch.from_numpy(imgClone).to("cuda:0")
            else:
                 imgTensor = torch.from_numpy(imgClone)
                 
            imgMask = self.mask_color(masks, imgTensor, colors)
        else:
            imgMask = imgClone
        
        if imgMask.shape[:2] != self.inputShape:
            imgMask = cv2.resize(imgMask, (self.inputShape[1], self.inputShape[0]))
            
        # Apply lane detection overlay on top of YOLO rendering
        if show_lane_overlay and lane_result is not None:
            if lane_detector is not None and hasattr(lane_detector, "render_lane_overlay"):
                imgMask = lane_detector.render_lane_overlay(imgMask, lane_result)
            else:
                imgMask = self._render_basic_lane_overlay(imgMask, lane_result)

        return imgMask

    def _render_basic_lane_overlay(self, image: np.ndarray, lane_result) -> np.ndarray:
        """Render basic lane detection overlay when no detector instance is available."""
        overlay = image.copy()

        if not lane_result.is_valid:
            return overlay

        # Choose color based on confidence
        color = (0, 255, 0) if lane_result.confidence > 0.5 else (0, 255, 255)

        # Draw steering indicator arrow at bottom of image
        center_x = image.shape[1] // 2
        start_y = image.shape[0] - 30
        end_x = int(center_x + lane_result.steering_correction * 200)
        cv2.arrowedLine(overlay, (center_x, start_y), (end_x, start_y), color, 3, tipLength=0.3)

        # Draw lane detection information
        lane_info = f"Lane: Steer={lane_result.steering_correction:.3f} Conf={lane_result.confidence:.2f}"
        cv2.putText(overlay, lane_info, (10, image.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Draw lateral offset if available
        if lane_result.lateral_offset is not None:
            offset_text = f"Offset: {lane_result.lateral_offset:.3f}"
            cv2.putText(overlay, offset_text, (10, image.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Draw lane detection indicators
        indicator_y = image.shape[0] - 100
        if lane_result.left_lane_detected:
            cv2.circle(overlay, (20, indicator_y), 8, (0, 255, 0), -1)
            cv2.putText(overlay, "L", (15, indicator_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if lane_result.right_lane_detected:
            cv2.circle(overlay, (50, indicator_y), 8, (0, 255, 0), -1)
            cv2.putText(overlay, "R", (45, indicator_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return overlay

def detect_line_and_draw(image):
    """
    Detects a black line using HSV thresholding (V <= 46), 
    draws the target point, and computes steering command.
    Returns:
        steer_angle, linear_velocity
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 46])
    mask = cv2.inRange(hsv, lower_black, upper_black)

    h, w = image.shape[:2]
    search_top = 3 * h // 4
    search_bot = search_top + 20

    # Mask out everything except the narrow horizontal band
    mask[0:search_top, :] = 0
    mask[search_bot:h, :] = 0

    M = cv2.moments(mask)
    steering_angle = 0.0
    linear_x = 0.0

    if M['m00'] > 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1)
        
        # Calculate steering angle (proportional to offset)
        error = cx - w / 2.0
        # Formula from cpp: twist_angular_z = -error / 300 * 0.4
        # Since ackermann steering command is steering angle (radians), we use the same formula
        steering_angle = -error / 300.0 * 0.4
        linear_x = 0.1
        
        cv2.putText(image, f"LineCmd: V={linear_x:.2f}, Steer={steering_angle:.3f} rad", 
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(image, "Line: Not Found", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    return steering_angle, linear_x
