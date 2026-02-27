"""
YOLO virtual server configuration loader.

All tunable values are sourced from yolo_config.yaml and normalized here so the
server and helper classes can consume clean, typed settings.

Quick activation guide (in yolo_config.yaml):
- Lane detection:
  - `lane.enable_detection: true`
  - `lane.show_overlay: true` (only controls drawing)
- LiDAR fusion:
  - `lidar_fusion.enabled: true`
  - optional visuals: `lidar_fusion.show_overlay: true`, `lidar_fusion.show_bev: true`
- Note:
  - Fusion is also auto-enabled if either `show_overlay` or `show_bev` is true.
  - You can still override a few runtime values from CLI (`-idx`, `-p`, `-s`).
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import yaml


LEGACY_CLASS_IDS = [0, 2, 9, 11, 33]
TRAFFIC_PLUS_CLASS_IDS = [0, 1, 2, 3, 5, 7, 9, 10, 11, 33]
VALID_SIGN_SIDES = {"left", "right", "both"}

DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "yolo_config.yaml"
)


def parse_bool_string(raw: Optional[str]) -> Optional[bool]:
    """Parse text booleans used by legacy launcher args."""
    if raw is None:
        return None
    value = str(raw).strip().lower()
    if value in {"1", "true", "yes", "y", "on"}:
        return True
    if value in {"0", "false", "no", "n", "off"}:
        return False
    return None


def _as_dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _to_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _to_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _to_optional_float(value: Any) -> Optional[float]:
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _to_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        parsed = parse_bool_string(value)
        if parsed is not None:
            return parsed
    return default


def _to_float_list(value: Any, default: List[float], length: int) -> List[float]:
    if not isinstance(value, list) or len(value) != length:
        return default
    out: List[float] = []
    for idx in range(length):
        out.append(_to_float(value[idx], default[idx]))
    return out


def _to_matrix3(value: Any, default: List[List[float]]) -> List[List[float]]:
    if not isinstance(value, list) or len(value) != 3:
        return default
    rows: List[List[float]] = []
    for ridx in range(3):
        row_default = default[ridx]
        row = _to_float_list(value[ridx], row_default, 3)
        rows.append(row)
    return rows


def _normalize_weights(
    lidar_weight: float,
    depth_weight: float,
    default_lidar: float,
    default_depth: float,
) -> Tuple[float, float]:
    lw = _clamp(lidar_weight, 0.0, 1.0)
    dw = _clamp(depth_weight, 0.0, 1.0)
    total = lw + dw
    if total <= 1e-6:
        return default_lidar, default_depth
    return lw / total, dw / total


def _parse_class_ids(raw: Any) -> List[int]:
    if raw is None:
        return []

    if isinstance(raw, list):
        items = raw
    else:
        items = [token.strip() for token in str(raw).split(",")]

    out: List[int] = []
    for token in items:
        try:
            cid = int(token)
        except (TypeError, ValueError):
            continue
        if cid not in out:
            out.append(cid)
    return out


@dataclass
class ServerSettings:
    """General runtime toggles for the server process."""
    car_id: int = 0
    probing: bool = False
    show_image: bool = False
    show_lane_debug: bool = False
    show_obstacle_box: bool = False


@dataclass
class PortSettings:
    """Base ports; actual ports are computed as base + car_id."""
    data_port_base: int = 18660
    video_port_base: int = 18760

    def data_port(self, car_id: int) -> str:
        return str(int(self.data_port_base) + int(car_id))

    def video_port(self, car_id: int) -> str:
        return str(int(self.video_port_base) + int(car_id))


@dataclass
class CameraSettings:
    """Input image/alignment settings used by QCar2DepthAlignedCamera."""
    image_width: int = 640
    image_height: int = 480
    use_intrinsics: bool = False
    clipping_distance_m: float = 5.0
    load_settings: bool = True
    use_fast_alignment: bool = True


@dataclass
class YoloTrackerSettings:
    persist: bool = True
    name: str = "botsort.yaml"


@dataclass
class YoloPostprocessSettings:
    clipping_distance_m: float = 10.0


@dataclass
class YoloSettings:
    """YOLO inference and postprocess defaults."""
    model_path: str = ""
    use_tracking: bool = True
    class_ids: List[int] = field(default_factory=lambda: TRAFFIC_PLUS_CLASS_IDS.copy())
    confidence: float = 0.4
    half: bool = True
    verbose: bool = False
    tracker: YoloTrackerSettings = field(default_factory=YoloTrackerSettings)
    postprocess: YoloPostprocessSettings = field(default_factory=YoloPostprocessSettings)


@dataclass
class SignFilterSettings:
    """Traffic-side sign filtering and left-right offset convention."""
    invert_lr: bool = False
    side: str = "right"


@dataclass
class BufferSettings:
    """Detection packet center-box geometry."""
    center_box_width_ratio: float = 0.40
    center_box_height_ratio: float = 0.60
    bottom_ignore_px: int = 0
    center_box_raise_px: int = 0
    stop_sign_front_max_abs_offset: float = 0.70


@dataclass
class LaneSettings:
    """Lane pipeline controls.

    - `enable_detection`: run lane detector and publish lane state
    - `show_overlay`: draw lane overlay on rendered frame
    """
    enable_detection: bool = True
    show_overlay: bool = True


@dataclass
class LidarBlendSettings:
    trust_lidar_lidar_weight: float = 0.85
    trust_lidar_depth_weight: float = 0.15
    sparse_lidar_lidar_weight: float = 0.70
    sparse_lidar_depth_weight: float = 0.30


@dataclass
class LidarBevSettings:
    size_px: int = 520
    pixels_per_meter: float = 30.0
    max_range_m: float = 12.0


@dataclass
class LidarSensorSettings:
    num_measurements: int = 360
    ranging_distance_mode: int = 2
    interpolation_mode: int = 0


@dataclass
class LidarGeometrySettings:
    is_physical: bool = False
    camera_fov_h_deg: float = 68.0
    camera_fov_v_deg: float = 42.5
    image_width: int = 640
    image_height: int = 480
    fx: Optional[float] = None
    fy: Optional[float] = None
    cx: Optional[float] = None
    cy: Optional[float] = None
    scale: Optional[float] = None
    # Projection-range gate for raw LiDAR points before association.
    # Keep this broad (legacy behavior was ~0.1..40m in virtual mode).
    min_distance_lidar_m: float = 0.1
    max_distance_lidar_m: float = 40.0
    camera_position: List[float] = field(
        default_factory=lambda: [0.95, 0.32, 1.72]
    )
    camera_rotation: List[List[float]] = field(
        default_factory=lambda: [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    )
    lidar_position: List[float] = field(
        default_factory=lambda: [-0.12, 0.0, 1.93]
    )
    lidar_rotation: List[List[float]] = field(
        default_factory=lambda: [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    )


@dataclass
class LidarFusionSettings:
    """LiDAR-camera fusion controls and thresholds.

    To activate fusion set `enabled: true`. Rendering flags are separate:
    - `show_overlay`: draw projected LiDAR on image
    - `show_bev`: show top-down BEV window
    """
    enabled: bool = False
    show_overlay: bool = False
    show_bev: bool = False
    lidar_min_distance_m: float = 0.08
    lidar_max_distance_m: float = 2.2
    obstacle_distance_threshold_m: float = 1.0
    lidar_min_points_for_trust: int = 2
    max_depth_lidar_disagreement_ratio: float = 0.6
    lower_bbox_start_ratio: float = 0.40
    mask_erode_kernel_size: int = 3
    mask_erode_iterations: int = 1
    allow_upper_bbox_fallback: bool = True
    bbox_fallback_shrink_factor: float = -0.15
    bbox_overlay_shrink_factor: float = -0.20
    outlier_filter_method: str = "one_sigma"
    distance_selection_technique: str = "median"
    blend: LidarBlendSettings = field(default_factory=LidarBlendSettings)
    bev: LidarBevSettings = field(default_factory=LidarBevSettings)
    sensor: LidarSensorSettings = field(default_factory=LidarSensorSettings)
    geometry: LidarGeometrySettings = field(default_factory=LidarGeometrySettings)


@dataclass
class YoloServerConfig:
    """Top-level typed config consumed by yolo_server_virtual.py.

    YAML sections map 1:1 to dataclass fields:
    - `server`, `ports`, `camera`, `yolo`, `sign_filter`, `buffers`, `lane`, `lidar_fusion`
    """
    config_path: str = DEFAULT_CONFIG_PATH
    server: ServerSettings = field(default_factory=ServerSettings)
    ports: PortSettings = field(default_factory=PortSettings)
    camera: CameraSettings = field(default_factory=CameraSettings)
    yolo: YoloSettings = field(default_factory=YoloSettings)
    sign_filter: SignFilterSettings = field(default_factory=SignFilterSettings)
    buffers: BufferSettings = field(default_factory=BufferSettings)
    lane: LaneSettings = field(default_factory=LaneSettings)
    lidar: LidarFusionSettings = field(default_factory=LidarFusionSettings)

    @property
    def car_id(self) -> int:
        return self.server.car_id

    @property
    def probing(self) -> bool:
        return self.server.probing

    @property
    def show_image(self) -> bool:
        return self.server.show_image

    @property
    def show_lane_debug(self) -> bool:
        return self.server.show_lane_debug

    @property
    def show_obstacle_box(self) -> bool:
        return self.server.show_obstacle_box

    @property
    def image_width(self) -> int:
        return self.camera.image_width

    @property
    def image_height(self) -> int:
        return self.camera.image_height

    @property
    def model_path(self) -> str:
        return self.yolo.model_path

    @property
    def use_tracking(self) -> bool:
        return self.yolo.use_tracking

    @property
    def class_ids(self) -> List[int]:
        return self.yolo.class_ids

    @property
    def confidence(self) -> float:
        return self.yolo.confidence

    @property
    def invert_lr(self) -> bool:
        return self.sign_filter.invert_lr

    @property
    def sign_filter_side(self) -> str:
        return self.sign_filter.side

    @property
    def enable_lane_detection(self) -> bool:
        return self.lane.enable_detection

    @property
    def show_lane_overlay(self) -> bool:
        return self.lane.show_overlay

    @property
    def enable_lidar_fusion(self) -> bool:
        return self.lidar.enabled

    @property
    def show_lidar_overlay(self) -> bool:
        return self.lidar.show_overlay

    @property
    def show_lidar_bev(self) -> bool:
        return self.lidar.show_bev

    @property
    def lidar_min_distance_m(self) -> float:
        return self.lidar.lidar_min_distance_m

    @property
    def lidar_max_distance_m(self) -> float:
        return self.lidar.lidar_max_distance_m

    @property
    def obstacle_distance_threshold_m(self) -> float:
        return self.lidar.obstacle_distance_threshold_m

    @property
    def lidar_min_points_for_trust(self) -> int:
        return self.lidar.lidar_min_points_for_trust

    @property
    def max_depth_lidar_disagreement_ratio(self) -> float:
        return self.lidar.max_depth_lidar_disagreement_ratio

    @property
    def lidar_lower_bbox_start_ratio(self) -> float:
        return self.lidar.lower_bbox_start_ratio

    @property
    def lidar_mask_erode_kernel_size(self) -> int:
        return self.lidar.mask_erode_kernel_size

    @property
    def lidar_mask_erode_iterations(self) -> int:
        return self.lidar.mask_erode_iterations

    @property
    def lidar_allow_upper_bbox_fallback(self) -> bool:
        return self.lidar.allow_upper_bbox_fallback

    @property
    def lidar_bbox_fallback_shrink_factor(self) -> float:
        return self.lidar.bbox_fallback_shrink_factor

    @property
    def lidar_overlay_bbox_shrink_factor(self) -> float:
        return self.lidar.bbox_overlay_shrink_factor

    @property
    def lidar_outlier_filter_method(self) -> str:
        return self.lidar.outlier_filter_method

    @property
    def lidar_distance_selection_technique(self) -> str:
        return self.lidar.distance_selection_technique

    @property
    def lidar_blend_weights_trust(self) -> Tuple[float, float]:
        return (
            self.lidar.blend.trust_lidar_lidar_weight,
            self.lidar.blend.trust_lidar_depth_weight,
        )

    @property
    def lidar_blend_weights_sparse(self) -> Tuple[float, float]:
        return (
            self.lidar.blend.sparse_lidar_lidar_weight,
            self.lidar.blend.sparse_lidar_depth_weight,
        )

    @property
    def lidar_bev_size_px(self) -> int:
        return self.lidar.bev.size_px

    @property
    def lidar_bev_pixels_per_meter(self) -> float:
        return self.lidar.bev.pixels_per_meter

    @property
    def lidar_bev_max_range_m(self) -> float:
        return self.lidar.bev.max_range_m

    @property
    def data_port(self) -> str:
        return self.ports.data_port(self.car_id)

    @property
    def video_port(self) -> str:
        return self.ports.video_port(self.car_id)

    def apply_overrides(self, overrides: Dict[str, Any]) -> None:
        if not overrides:
            return
        if "car_id" in overrides and overrides["car_id"] is not None:
            self.server.car_id = _to_int(overrides["car_id"], self.server.car_id)
        if "probing" in overrides and overrides["probing"] is not None:
            self.server.probing = _to_bool(overrides["probing"], self.server.probing)
        if "show_image" in overrides and overrides["show_image"] is not None:
            self.server.show_image = _to_bool(
                overrides["show_image"], self.server.show_image
            )
        if "enable_lidar_fusion" in overrides and overrides["enable_lidar_fusion"] is not None:
            self.lidar.enabled = _to_bool(
                overrides["enable_lidar_fusion"], self.lidar.enabled
            )

    def to_yolo_runtime_config(self) -> Dict[str, Any]:
        return {
            "use_tracking": self.yolo.use_tracking,
            "classes": list(self.yolo.class_ids),
            "confidence": self.yolo.confidence,
            "half": self.yolo.half,
            "verbose": self.yolo.verbose,
            "tracker": {
                "persist": self.yolo.tracker.persist,
                "name": self.yolo.tracker.name,
            },
            "postprocess_clipping_distance_m": self.yolo.postprocess.clipping_distance_m,
        }

    def to_buffer_runtime_config(self) -> Dict[str, Any]:
        return {
            "center_box_width_ratio": self.buffers.center_box_width_ratio,
            "center_box_height_ratio": self.buffers.center_box_height_ratio,
            "bottom_ignore_px": self.buffers.bottom_ignore_px,
            "center_box_raise_px": self.buffers.center_box_raise_px,
            "stop_sign_front_max_abs_offset": self.buffers.stop_sign_front_max_abs_offset,
        }

    def to_lidar_geometry_runtime_config(self) -> Dict[str, Any]:
        g = self.lidar.geometry
        return {
            "is_physical": g.is_physical,
            "camera_fov_h_deg": g.camera_fov_h_deg,
            "camera_fov_v_deg": g.camera_fov_v_deg,
            "image_width": g.image_width,
            "image_height": g.image_height,
            "fx": g.fx,
            "fy": g.fy,
            "cx": g.cx,
            "cy": g.cy,
            "scale": g.scale,
            "min_distance_lidar_m": g.min_distance_lidar_m,
            "max_distance_lidar_m": g.max_distance_lidar_m,
            "camera_position": list(g.camera_position),
            "camera_rotation": [list(row) for row in g.camera_rotation],
            "lidar_position": list(g.lidar_position),
            "lidar_rotation": [list(row) for row in g.lidar_rotation],
        }

    @classmethod
    def from_yaml(
        cls, config_path: Optional[str] = None, overrides: Optional[Dict[str, Any]] = None
    ) -> "YoloServerConfig":
        """Load and normalize YAML into a typed runtime config.

        Feature toggles reference:
        - Lane detection: `lane.enable_detection` and `lane.show_overlay`
        - LiDAR fusion: `lidar_fusion.enabled`
          (or set `lidar_fusion.show_overlay/show_bev` to auto-enable fusion)
        """
        path = os.path.abspath(config_path or DEFAULT_CONFIG_PATH)
        raw: Dict[str, Any] = {}
        if os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    raw = yaml.safe_load(f) or {}
                print(f"[CONFIG] Loaded YOLO config: {path}")
            except Exception as exc:
                print(f"[CONFIG] Failed to load '{path}': {exc}. Using defaults.")
                raw = {}
        else:
            print(f"[CONFIG] Config file not found: {path}. Using defaults.")

        server_raw = _as_dict(raw.get("server"))
        ports_raw = _as_dict(raw.get("ports"))
        camera_raw = _as_dict(raw.get("camera"))
        yolo_raw = _as_dict(raw.get("yolo"))
        sign_raw = _as_dict(raw.get("sign_filter"))
        buffers_raw = _as_dict(raw.get("buffers"))
        lane_raw = _as_dict(raw.get("lane"))
        lidar_raw = _as_dict(raw.get("lidar_fusion"))

        preset = str(yolo_raw.get("classes_preset", "traffic_plus")).strip().lower()
        class_ids = _parse_class_ids(yolo_raw.get("classes"))
        if not class_ids:
            if preset == "legacy":
                class_ids = LEGACY_CLASS_IDS.copy()
            else:
                class_ids = TRAFFIC_PLUS_CLASS_IDS.copy()

        invert_lr = _to_bool(sign_raw.get("invert_lr"), False)
        sign_side = str(sign_raw.get("side", "right")).strip().lower()
        if sign_side not in VALID_SIGN_SIDES:
            sign_side = "right"
        if invert_lr and sign_side == "right":
            sign_side = "left"

        lidar_min = max(0.02, _to_float(lidar_raw.get("lidar_min_distance_m"), 0.08))
        lidar_max = max(
            lidar_min + 0.05,
            _to_float(lidar_raw.get("lidar_max_distance_m"), 2.2),
        )
        obstacle_threshold = _clamp(
            _to_float(lidar_raw.get("obstacle_distance_threshold_m"), 1.0),
            lidar_min,
            lidar_max,
        )
        lidar_min_points = max(
            1, _to_int(lidar_raw.get("lidar_min_points_for_trust"), 2)
        )

        blend_raw = _as_dict(lidar_raw.get("blend_weights"))
        trust_raw = _as_dict(blend_raw.get("trust_lidar"))
        sparse_raw = _as_dict(blend_raw.get("sparse_lidar"))
        trust_lidar_w, trust_depth_w = _normalize_weights(
            _to_float(trust_raw.get("lidar"), 0.85),
            _to_float(trust_raw.get("depth"), 0.15),
            0.85,
            0.15,
        )
        sparse_lidar_w, sparse_depth_w = _normalize_weights(
            _to_float(sparse_raw.get("lidar"), 0.70),
            _to_float(sparse_raw.get("depth"), 0.30),
            0.70,
            0.30,
        )

        bev_raw = _as_dict(lidar_raw.get("bev"))
        sensor_raw = _as_dict(lidar_raw.get("sensor"))
        geometry_raw = _as_dict(lidar_raw.get("geometry"))

        geometry_is_physical = _to_bool(
            geometry_raw.get("is_physical"),
            False,
        )
        default_scale = 0.1 if geometry_is_physical else 1.0
        geometry_scale = _to_optional_float(geometry_raw.get("scale"))
        if geometry_scale is None:
            geometry_scale = default_scale

        geometry_image_width = _to_int(
            geometry_raw.get("image_width"), _to_int(camera_raw.get("image_width"), 640)
        )
        geometry_image_height = _to_int(
            geometry_raw.get("image_height"), _to_int(camera_raw.get("image_height"), 480)
        )
        geometry_cx = _to_optional_float(geometry_raw.get("cx"))
        if geometry_cx is None:
            geometry_cx = geometry_image_width / 2.0
        geometry_cy = _to_optional_float(geometry_raw.get("cy"))
        if geometry_cy is None:
            geometry_cy = geometry_image_height / 2.0

        # Geometry projection range should be broad and independent from
        # fusion gates. Fusion gates are controlled by lidar_min/max above.
        geometry_min_dist = max(
            0.02,
            _to_float(
                geometry_raw.get("min_distance_lidar_m"),
                0.1 * geometry_scale,
            ),
        )
        geometry_max_dist = max(
            geometry_min_dist + 0.05,
            _to_float(
                geometry_raw.get("max_distance_lidar_m"),
                40.0 * geometry_scale,
            ),
        )

        config = cls(
            config_path=path,
            server=ServerSettings(
                car_id=_to_int(server_raw.get("car_id"), 0),
                probing=_to_bool(server_raw.get("probing"), False),
                show_image=_to_bool(server_raw.get("show_image"), False),
                show_lane_debug=_to_bool(server_raw.get("show_lane_debug"), False),
                show_obstacle_box=_to_bool(server_raw.get("show_obstacle_box"), False),
            ),
            ports=PortSettings(
                data_port_base=_to_int(ports_raw.get("data_port_base"), 18660),
                video_port_base=_to_int(ports_raw.get("video_port_base"), 18760),
            ),
            camera=CameraSettings(
                image_width=_to_int(camera_raw.get("image_width"), 640),
                image_height=_to_int(camera_raw.get("image_height"), 480),
                use_intrinsics=_to_bool(camera_raw.get("use_intrinsics"), False),
                clipping_distance_m=_to_float(
                    camera_raw.get("clipping_distance_m"), 5.0
                ),
                load_settings=_to_bool(camera_raw.get("load_settings"), True),
                use_fast_alignment=_to_bool(camera_raw.get("use_fast_alignment"), True),
            ),
            yolo=YoloSettings(
                model_path=str(yolo_raw.get("model_path", "") or ""),
                use_tracking=_to_bool(yolo_raw.get("use_tracking"), True),
                class_ids=class_ids,
                confidence=_clamp(_to_float(yolo_raw.get("confidence"), 0.4), 0.0, 1.0),
                half=_to_bool(yolo_raw.get("half"), True),
                verbose=_to_bool(yolo_raw.get("verbose"), False),
                tracker=YoloTrackerSettings(
                    persist=_to_bool(
                        _as_dict(yolo_raw.get("tracker")).get("persist"), True
                    ),
                    name=str(_as_dict(yolo_raw.get("tracker")).get("name", "botsort.yaml")),
                ),
                postprocess=YoloPostprocessSettings(
                    clipping_distance_m=_to_float(
                        _as_dict(yolo_raw.get("postprocess")).get(
                            "clipping_distance_m"
                        ),
                        10.0,
                    )
                ),
            ),
            sign_filter=SignFilterSettings(
                invert_lr=invert_lr,
                side=sign_side,
            ),
            buffers=BufferSettings(
                center_box_width_ratio=_clamp(
                    _to_float(buffers_raw.get("center_box_width_ratio"), 0.40),
                    0.05,
                    1.0,
                ),
                center_box_height_ratio=_clamp(
                    _to_float(buffers_raw.get("center_box_height_ratio"), 0.60),
                    0.05,
                    1.0,
                ),
                bottom_ignore_px=max(
                    0,
                    _to_int(
                        buffers_raw.get("bottom_ignore_px"),
                        0,
                    ),
                ),
                center_box_raise_px=max(
                    0,
                    _to_int(
                        buffers_raw.get("center_box_raise_px"),
                        0,
                    ),
                ),
                stop_sign_front_max_abs_offset=_clamp(
                    _to_float(
                        buffers_raw.get("stop_sign_front_max_abs_offset"),
                        0.70,
                    ),
                    0.05,
                    1.0,
                ),
            ),
            lane=LaneSettings(
                enable_detection=_to_bool(lane_raw.get("enable_detection"), True),
                show_overlay=_to_bool(lane_raw.get("show_overlay"), True),
            ),
            lidar=LidarFusionSettings(
                # Fusion master switch.
                # Keep behavior user-friendly: if any LiDAR visualization is enabled,
                # we automatically enable fusion even when `enabled` is omitted.
                enabled=_to_bool(
                    lidar_raw.get("enabled"),
                    False,
                )
                or _to_bool(lidar_raw.get("show_overlay"), False)
                or _to_bool(lidar_raw.get("show_bev"), False),
                # Visualization controls (do not change fusion math by themselves).
                show_overlay=_to_bool(lidar_raw.get("show_overlay"), False),
                show_bev=_to_bool(lidar_raw.get("show_bev"), False),
                lidar_min_distance_m=lidar_min,
                lidar_max_distance_m=lidar_max,
                obstacle_distance_threshold_m=obstacle_threshold,
                lidar_min_points_for_trust=lidar_min_points,
                max_depth_lidar_disagreement_ratio=_to_float(
                    lidar_raw.get("max_depth_lidar_disagreement_ratio"), 0.6
                ),
                lower_bbox_start_ratio=_clamp(
                    _to_float(lidar_raw.get("lower_bbox_start_ratio"), 0.40),
                    0.0,
                    1.0,
                ),
                mask_erode_kernel_size=max(
                    1, _to_int(lidar_raw.get("mask_erode_kernel_size"), 3)
                ),
                mask_erode_iterations=max(
                    1, _to_int(lidar_raw.get("mask_erode_iterations"), 1)
                ),
                allow_upper_bbox_fallback=_to_bool(
                    lidar_raw.get("allow_upper_bbox_fallback"), True
                ),
                bbox_fallback_shrink_factor=_to_float(
                    lidar_raw.get("bbox_fallback_shrink_factor"), -0.15
                ),
                bbox_overlay_shrink_factor=_to_float(
                    lidar_raw.get("bbox_overlay_shrink_factor"), -0.20
                ),
                outlier_filter_method=str(
                    lidar_raw.get("outlier_filter_method", "one_sigma")
                ),
                distance_selection_technique=str(
                    lidar_raw.get("distance_selection_technique", "median")
                ),
                blend=LidarBlendSettings(
                    trust_lidar_lidar_weight=trust_lidar_w,
                    trust_lidar_depth_weight=trust_depth_w,
                    sparse_lidar_lidar_weight=sparse_lidar_w,
                    sparse_lidar_depth_weight=sparse_depth_w,
                ),
                bev=LidarBevSettings(
                    size_px=max(128, _to_int(bev_raw.get("size_px"), 520)),
                    pixels_per_meter=max(
                        1.0, _to_float(bev_raw.get("pixels_per_meter"), 30.0)
                    ),
                    max_range_m=max(1.0, _to_float(bev_raw.get("max_range_m"), 12.0)),
                ),
                sensor=LidarSensorSettings(
                    num_measurements=max(
                        8, _to_int(sensor_raw.get("num_measurements"), 360)
                    ),
                    ranging_distance_mode=_to_int(
                        sensor_raw.get("ranging_distance_mode"), 2
                    ),
                    interpolation_mode=_to_int(
                        sensor_raw.get("interpolation_mode"), 0
                    ),
                ),
                geometry=LidarGeometrySettings(
                    is_physical=geometry_is_physical,
                    camera_fov_h_deg=_to_float(
                        geometry_raw.get("camera_fov_h_deg"), 68.0
                    ),
                    camera_fov_v_deg=_to_float(
                        geometry_raw.get("camera_fov_v_deg"), 42.5
                    ),
                    image_width=geometry_image_width,
                    image_height=geometry_image_height,
                    fx=_to_optional_float(geometry_raw.get("fx")),
                    fy=_to_optional_float(geometry_raw.get("fy")),
                    cx=geometry_cx,
                    cy=geometry_cy,
                    scale=geometry_scale,
                    min_distance_lidar_m=geometry_min_dist,
                    max_distance_lidar_m=geometry_max_dist,
                    camera_position=_to_float_list(
                        geometry_raw.get("camera_position"),
                        [0.95, 0.32, 1.72],
                        3,
                    ),
                    camera_rotation=_to_matrix3(
                        geometry_raw.get("camera_rotation"),
                        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
                    ),
                    lidar_position=_to_float_list(
                        geometry_raw.get("lidar_position"),
                        [-0.12, 0.0, 1.93],
                        3,
                    ),
                    lidar_rotation=_to_matrix3(
                        geometry_raw.get("lidar_rotation"),
                        [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
                    ),
                ),
            ),
        )

        config.apply_overrides(overrides or {})
        return config
