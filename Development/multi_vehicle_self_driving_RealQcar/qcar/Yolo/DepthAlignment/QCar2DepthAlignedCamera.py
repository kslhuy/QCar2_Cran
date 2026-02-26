"""
QCar2 Depth-RGB Aligned Camera Class
Provides aligned RGB and depth images from QCar RealSense camera.
"""

import numpy as np
import cv2
import json
import os, getpass
try:
    os.getlogin()
except OSError:
    os.getlogin = getpass.getuser
from pal.products.qcar import QCarRealSense

try:
    from numba import jit

    NUMBA_AVAILABLE = True
except ImportError:
    NUMBA_AVAILABLE = False

    # Dummy decorator if numba not available
    def jit(*args, **kwargs):
        def decorator(func):
            return func

        return decorator


class QCar2DepthAlignedCamera:
    """
    A class to capture and align RGB and Depth images from QCar RealSense camera.

    Usage:
        camera = QCar2DepthAlignedCamera(use_intrinsics=True)
        camera.read()
        rgb_image = camera.rgb
        depth_image = camera.depth  # Aligned depth in meters
    """
 git config --global user.email "huyq1471@gmail.com"
  git config --global user.name "HUY"
    def __init__(
        self,
        imageWidth=640,
        imageHeight=480,
        use_intrinsics=True,
        clipping_distance=3.0,
        video3dPort=18665,
        load_settings=True,
        use_fast_alignment=True,
    ):
        """
        Initialize the aligned camera.

        Args:
            imageWidth: Width of images
            imageHeight: Height of images
            use_intrinsics: If True, use intrinsic-based alignment. If False, use simple crop alignment.
            clipping_distance: Maximum depth in meters for visualization
            video3dPort: Port for RealSense camera
            load_settings: If True, load saved alignment settings from JSON
            use_fast_alignment: If True, use optimized vectorized alignment (recommended for speed)
        """
        self.imageWidth = imageWidth
        self.imageHeight = imageHeight
        self.use_intrinsics = use_intrinsics
        self.use_fast_alignment = use_fast_alignment
        self.clipping_distance = clipping_distance

        # Public properties for external access
        self.rgb = None
        self.depth = None  # Aligned depth in meters
        self.depth_colormap = None  # Colorized depth for visualization

        # Alignment parameters - Intrinsics mode
        self.focal_scale_rgb = 1.0
        self.focal_scale_depth = 1.0
        self.principal_point_offset_x = 0.0  # Offset for cx_depth
        self.principal_point_offset_y = 0.0  # Offset for cy_depth
        self.crop_scale_x = 1.0
        self.crop_scale_y = 1.0

        # Legacy crop offsets (kept for simple mode compatibility)
        self.crop_offset_x = 0
        self.crop_offset_y = 0

        # Alignment parameters - Simple mode
        self.crop_top = 65
        self.crop_bottom = 415
        self.crop_left = 105
        self.crop_right = 510

        # FOV data from camera specifications
        self.fov_rgb_h = np.deg2rad(69.4)
        self.fov_rgb_v = np.deg2rad(42.5)
        self.fov_depth_h = np.deg2rad(87.0)
        self.fov_depth_v = np.deg2rad(58.0)

        # Calculate focal lengths
        self._calculate_focal_lengths()

        # Initialize camera
        self._initialize_camera(video3dPort)

        # Load settings if requested
        if load_settings:
            self.load_alignment_settings()

        print("✓ QCar2DepthAlignedCamera initialized")
        print(f"  Mode: {'Intrinsics' if self.use_intrinsics else 'Simple'}")
        print(f"  Resolution: {self.imageWidth}x{self.imageHeight}")

    def _calculate_focal_lengths(self):
        """Calculate focal lengths from FOV."""
        fx_rgb = (
            (self.imageWidth / 2.0)
            / np.tan(self.fov_rgb_h / 2.0)
            * self.focal_scale_rgb
        )
        fy_rgb = (
            (self.imageHeight / 2.0)
            / np.tan(self.fov_rgb_v / 2.0)
            * self.focal_scale_rgb
        )
        fx_depth = (
            (self.imageWidth / 2.0)
            / np.tan(self.fov_depth_h / 2.0)
            * self.focal_scale_depth
        )
        fy_depth = (
            (self.imageHeight / 2.0)
            / np.tan(self.fov_depth_v / 2.0)
            * self.focal_scale_depth
        )

        cx_rgb = self.imageWidth / 2.0
        cy_rgb = self.imageHeight / 2.0
        cx_depth = self.imageWidth / 2.0 + self.principal_point_offset_x
        cy_depth = self.imageHeight / 2.0 + self.principal_point_offset_y

        self.fx_rgb = fx_rgb
        self.fy_rgb = fy_rgb
        self.fx_depth = fx_depth
        self.fy_depth = fy_depth
        self.cx_rgb = cx_rgb
        self.cy_rgb = cy_rgb
        self.cx_depth = cx_depth
        self.cy_depth = cy_depth

        # Build intrinsic matrices
        self.K_rgb = np.array(
            [[fx_rgb, 0, cx_rgb], [0, fy_rgb, cy_rgb], [0, 0, 1]], dtype=np.float64
        )

        self.K_depth = np.array(
            [[fx_depth, 0, cx_depth], [0, fy_depth, cy_depth], [0, 0, 1]],
            dtype=np.float64,
        )

    def _initialize_camera(self, video3dPort):
        """Initialize QCar RealSense camera."""

        # Check resolution to decide which parameters to use
        # Primary use case: D435 RealSense
        user_calib_used = False

        if self.imageWidth == 640 and self.imageHeight == 480:
            print("  Using User-Provided Calibration Parameters (D435)")
            # D435 RGB Intrinsic matrix
            self.K_rgb = np.array(
                [[455.2, 0.00, 308.53], [0.00, 459.43, 213.56], [0.00, 0.00, 1.00]],
                dtype=np.float64,
            )

            # D435 Depth Intrinsic matrix
            self.K_depth = np.array(
                [[385.6, 0.00, 321.9], [0.0, 385.6, 237.3], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )

            # D435 Extrinsics (User provided: RGB to Depth)
            # We need Depth to RGB for current alignment logic
            T_rgb_to_depth = np.array(
                [
                    [1, 0.004008, 0.0001655, -0.01474],
                    [-0.004007, 1, -0.003435, -0.0004152],
                    [-0.0001792, 0.003434, 1, -0.0002451],
                    [0, 0, 0, 1],
                ]
            )
            self.depth_to_rgb_transform = np.linalg.inv(T_rgb_to_depth)
            user_calib_used = True

        elif self.imageWidth == 820 and self.imageHeight == 410:
            # CSI camera intrinsic matrix (Provided by user)
            print("  Using User-Provided Calibration Parameters (CSI)")
            self.K_rgb = np.array(
                [[318.86, 0.00, 401.34], [0.00, 312.14, 201.50], [0.00, 0.00, 1.00]],
                dtype=np.float64,
            )

            # Use D435 Depth params as placeholder for depth
            self.K_depth = np.array(
                [[385.6, 0.00, 321.9], [0.0, 385.6, 237.3], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )

            print(
                "WARNING: Using CSI RGB params with default/D435 Depth params. Extrinsics might be invalid."
            )
            self.depth_to_rgb_transform = np.eye(4)  # Unknown extrinsics for CSI
            user_calib_used = True

        if user_calib_used:
            # Update member variables from the selected intrinsics
            self.fx_rgb = self.K_rgb[0, 0]
            self.fy_rgb = self.K_rgb[1, 1]
            self.cx_rgb = self.K_rgb[0, 2]
            self.cy_rgb = self.K_rgb[1, 2]

            self.fx_depth = self.K_depth[0, 0]
            self.fy_depth = self.K_depth[1, 1]
            self.cx_depth = self.K_depth[0, 2]
            self.cy_depth = self.K_depth[1, 2]

        # Initialize camera with the best available parameters
        self.camera = QCarRealSense(
            mode="RGB, Depth",
            focalLengthRGB=np.array(
                [[self.fx_rgb / 2], [self.fy_rgb / 2]], dtype=np.float64
            ),
            principlePointRGB=np.array(
                [[self.cx_rgb], [self.cy_rgb]], dtype=np.float64
            ),
            focalLengthDepth=np.array(
                [[self.fx_depth / 2], [self.fy_depth / 2]], dtype=np.float64
            ),
            principlePointDepth=np.array(
                [[self.cx_depth], [self.cy_depth]], dtype=np.float64
            ),
            skewDepth=0.0,
            skewRGB=0.0,
            video3dPort=video3dPort,
        )

        # If we didn't use user calibration, try to get from device as last resort
        if not user_calib_used:
            try:
                K_rgb_cam = self.camera.intrinsics_rgb()
                K_depth_cam = self.camera.intrinsics_depth()

                if not (np.any(np.isnan(K_rgb_cam)) or np.any(np.isnan(K_depth_cam))):
                    self.K_rgb = K_rgb_cam
                    self.K_depth = K_depth_cam
                    print("  Using QCarRealSense intrinsics")

                E_rgb = self.camera.extrinsics_rgb()
                E_depth = self.camera.extrinsics_depth()

                if np.any(np.isnan(E_rgb)) or np.any(np.isnan(E_depth)):
                    self.depth_to_rgb_transform = np.eye(4)
                else:
                    self.depth_to_rgb_transform = E_rgb @ np.linalg.inv(E_depth)
            except Exception:
                print("  Using calculated intrinsics from FOV")
                self.depth_to_rgb_transform = np.eye(4)
        # Update member variables for consistency
        self.fx_rgb = self.K_rgb[0, 0]
        self.fy_rgb = self.K_rgb[1, 1]
        self.cx_rgb = self.K_rgb[0, 2]
        self.cy_rgb = self.K_rgb[1, 2]

        self.fx_depth = self.K_depth[0, 0]
        self.fy_depth = self.K_depth[1, 1]
        self.cx_depth = self.K_depth[0, 2]
        self.cy_depth = self.K_depth[1, 2]

    def read(self):
        """
        Capture and align RGB and depth images.
        Updates self.rgb and self.depth properties.
        """
        # Read RGB and Depth frames
        self.camera.read_RGB()
        self.camera.read_depth(dataMode="PX")

        # Get images from camera buffers
        self.rgb = self.camera.imageBufferRGB.copy()
        depth_image_px = self.camera.imageBufferDepthPX.copy().squeeze()

        # Convert pixel depth to meters
        depth_scale = 0.01  # 1 pixel unit = 1cm = 0.01m (corrected from 0.001)
        depth_image_m = depth_image_px.astype(np.float32) * depth_scale

        # Align depth to RGB
        if self.use_intrinsics:
            #  # Crop depth to match RGB FOV first
            # depth_cropped = self._crop_depth_to_rgb_fov(depth_image_m)
            # # Then align using intrinsics
            # Pass raw depth directly to intrinsics-based alignment
            self.depth = self._align_depth_to_rgb_intrinsics(depth_image_m)
        else:
            # Simple crop and resize alignment
            self.depth = self._simple_align_depth_to_rgb(depth_image_m)

        # Create colormap for visualization
        self._create_depth_colormap()

    def _crop_depth_to_rgb_fov(self, depth_image_m):
        """Crop depth image to match RGB FOV."""
        fov_ratio_h = np.tan(self.fov_rgb_h / 2.0) / np.tan(self.fov_depth_h / 2.0)
        fov_ratio_v = np.tan(self.fov_rgb_v / 2.0) / np.tan(self.fov_depth_v / 2.0)

        crop_width = int(self.imageWidth * fov_ratio_h * self.crop_scale_x)
        crop_height = int(self.imageHeight * fov_ratio_v * self.crop_scale_y)
        crop_start_x = (self.imageWidth - crop_width) // 2
        crop_start_y = (self.imageHeight - crop_height) // 2

        # Apply user-adjustable crop offset
        actual_crop_x = crop_start_x + self.crop_offset_x
        actual_crop_y = crop_start_y + self.crop_offset_y

        # Ensure crop region is within bounds
        actual_crop_x = max(0, min(actual_crop_x, self.imageWidth - crop_width))
        actual_crop_y = max(0, min(actual_crop_y, self.imageHeight - crop_height))

        # Crop and resize
        depth_cropped = depth_image_m[
            actual_crop_y : actual_crop_y + crop_height,
            actual_crop_x : actual_crop_x + crop_width,
        ]

        depth_resized = cv2.resize(
            depth_cropped,
            (self.imageWidth, self.imageHeight),
            interpolation=cv2.INTER_LINEAR,
        )

        return depth_resized

    def _align_depth_to_rgb_intrinsics(self, depth_image):
        """Align depth to RGB using camera intrinsics."""
        if self.use_fast_alignment:
            return self._align_depth_to_rgb_intrinsics_fast(depth_image)
        else:
            return self._align_depth_to_rgb_intrinsics_legacy(depth_image)

    def _align_depth_to_rgb_intrinsics_fast(self, depth_image):
        """Fast vectorized depth alignment using numpy operations."""
        h_d, w_d = depth_image.shape[:2]
        h_rgb, w_rgb = self.imageHeight, self.imageWidth

        # Generate depth pixel coordinates (vectorized)
        u_d, v_d = np.meshgrid(np.arange(w_d), np.arange(h_d), indexing="xy")
        z_d = depth_image
        valid_mask = z_d > 0

        # Extract valid depth points
        u_d_valid = u_d[valid_mask]
        v_d_valid = v_d[valid_mask]
        z_d_valid = z_d[valid_mask]

        # Back-project to 3D (vectorized)
        K_depth_inv = np.linalg.inv(self.K_depth)
        pixels_depth = np.stack([u_d_valid, v_d_valid, np.ones_like(u_d_valid)], axis=0)
        points_3d_depth = (K_depth_inv @ pixels_depth) * z_d_valid

        # Transform to RGB camera frame
        points_3d_depth_homo = np.vstack(
            [points_3d_depth, np.ones((1, points_3d_depth.shape[1]))]
        )
        points_3d_rgb_homo = self.depth_to_rgb_transform @ points_3d_depth_homo
        points_3d_rgb = points_3d_rgb_homo[:3, :]

        # Project to RGB image plane
        z_proj = points_3d_rgb[2, :]
        pixels_rgb = self.K_rgb @ points_3d_rgb
        pixels_rgb = pixels_rgb[:2, :] / z_proj

        # Round to nearest pixel
        u_rgb = np.round(pixels_rgb[0, :]).astype(np.int32)
        v_rgb = np.round(pixels_rgb[1, :]).astype(np.int32)

        # Filter valid projections
        valid_proj = (
            (u_rgb >= 0)
            & (u_rgb < w_rgb)
            & (v_rgb >= 0)
            & (v_rgb < h_rgb)
            & (z_proj > 0)
        )
        u_rgb = u_rgb[valid_proj]
        v_rgb = v_rgb[valid_proj]
        z_rgb = z_proj[valid_proj]

        # Fast vectorized depth filling using numpy operations
        aligned_depth = np.zeros((h_rgb, w_rgb), dtype=np.float32)

        # Sort by depth to handle occlusions (closer points first)
        sort_idx = np.argsort(z_rgb)
        u_rgb_sorted = u_rgb[sort_idx]
        v_rgb_sorted = v_rgb[sort_idx]
        z_rgb_sorted = z_rgb[sort_idx]

        # Use numpy advanced indexing for fast assignment
        # Fill from farthest to closest, so closest points overwrite
        aligned_depth[v_rgb_sorted[::-1], u_rgb_sorted[::-1]] = z_rgb_sorted[::-1]

        # Fast hole filling using cv2 inpainting with smaller radius
        mask = (aligned_depth == 0).astype(np.uint8)
        hole_ratio = np.sum(mask) / mask.size

        if 0 < hole_ratio < 0.5:  # Only inpaint if reasonable number of holes
            # Use faster TELEA algorithm with smaller radius
            depth_uint16 = (aligned_depth * 1000).astype(np.uint16)
            depth_inpainted = cv2.inpaint(
                depth_uint16, mask, inpaintRadius=2, flags=cv2.INPAINT_TELEA
            )
            aligned_depth = depth_inpainted.astype(np.float32) / 1000.0

        return aligned_depth

    def _align_depth_to_rgb_intrinsics_legacy(self, depth_image):
        """Legacy depth alignment method (slower but more accurate for overlapping points)."""
        h_d, w_d = depth_image.shape[:2]
        h_rgb, w_rgb = self.imageHeight, self.imageWidth

        aligned_depth = np.zeros((h_rgb, w_rgb), dtype=np.float32)

        # Generate depth pixel coordinates
        u_d, v_d = np.meshgrid(np.arange(w_d), np.arange(h_d))
        z_d = depth_image.reshape(-1)
        valid_mask = z_d > 0

        u_d_flat = u_d.reshape(-1)[valid_mask]
        v_d_flat = v_d.reshape(-1)[valid_mask]
        z_d_flat = z_d[valid_mask]

        # Back-project to 3D
        K_depth_inv = np.linalg.inv(self.K_depth)
        pixels_depth = np.vstack([u_d_flat, v_d_flat, np.ones_like(u_d_flat)])
        points_3d_depth = K_depth_inv @ pixels_depth * z_d_flat

        # Transform to RGB camera frame
        points_3d_depth_homo = np.vstack(
            [points_3d_depth, np.ones((1, points_3d_depth.shape[1]))]
        )
        points_3d_rgb_homo = self.depth_to_rgb_transform @ points_3d_depth_homo
        points_3d_rgb = points_3d_rgb_homo[:3, :]

        # Project to RGB image plane
        pixels_rgb = self.K_rgb @ points_3d_rgb
        pixels_rgb = pixels_rgb[:2, :] / pixels_rgb[2, :]

        # Round to nearest pixel
        u_rgb = np.round(pixels_rgb[0, :]).astype(np.int32)
        v_rgb = np.round(pixels_rgb[1, :]).astype(np.int32)

        # Filter valid projections
        valid_proj = (u_rgb >= 0) & (u_rgb < w_rgb) & (v_rgb >= 0) & (v_rgb < h_rgb)
        u_rgb = u_rgb[valid_proj]
        v_rgb = v_rgb[valid_proj]
        z_rgb = points_3d_rgb[2, valid_proj]

        # Fill aligned depth with minimum depth per pixel
        for i in range(len(u_rgb)):
            if (
                aligned_depth[v_rgb[i], u_rgb[i]] == 0
                or z_rgb[i] < aligned_depth[v_rgb[i], u_rgb[i]]
            ):
                aligned_depth[v_rgb[i], u_rgb[i]] = z_rgb[i]

        # Fill holes using OpenCV inpainting
        mask = (aligned_depth == 0).astype(np.uint8)
        if np.sum(mask) > 0 and np.sum(mask) < mask.size:
            depth_uint16 = (aligned_depth * 1000).astype(np.uint16)
            depth_inpainted = cv2.inpaint(
                depth_uint16, mask, inpaintRadius=3, flags=cv2.INPAINT_NS
            )
            aligned_depth = depth_inpainted.astype(np.float32) / 1000.0

        return aligned_depth

    def _simple_align_depth_to_rgb(self, depth_image):
        """Simple alignment using crop and resize."""
        h, w = depth_image.shape[:2]
        top = max(0, min(self.crop_top, h))
        bottom = max(top + 1, min(self.crop_bottom, h))
        left = max(0, min(self.crop_left, w))
        right = max(left + 1, min(self.crop_right, w))

        depth_cropped = depth_image[top:bottom, left:right]
        depth_resized = cv2.resize(
            depth_cropped,
            (self.imageWidth, self.imageHeight),
            interpolation=cv2.INTER_NEAREST,
        )

        return depth_resized

    def _create_depth_colormap(self):
        """Create colorized depth image for visualization."""
        valid_depth = self.depth[self.depth > 0]
        if len(valid_depth) > 0:
            depth_min = valid_depth.min()
            depth_max = valid_depth.max()
            depth_normalized = np.zeros_like(self.depth, dtype=np.uint8)
            mask = self.depth > 0
            depth_normalized[mask] = np.clip(
                ((self.depth[mask] - depth_min) / (depth_max - depth_min + 1e-6)) * 255,
                0,
                255,
            ).astype(np.uint8)
        else:
            depth_normalized = np.zeros_like(self.depth, dtype=np.uint8)

        self.depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

    def load_alignment_settings(self):
        """Load alignment settings from JSON file."""
        filename = (
            "alignment_settings_intrinsics.json"
            if self.use_intrinsics
            else "alignment_settings_simple.json"
        )
        filepath = os.path.join(os.path.dirname(__file__), filename)

        if not os.path.exists(filepath):
            print(f"  No saved settings found: {filename}")
            return False

        try:
            with open(filepath, "r") as f:
                settings = json.load(f)

            mode_name = settings.get("mode", "unknown")

            if self.use_intrinsics and mode_name == "intrinsics":
                self.focal_scale_rgb = settings.get("focal_scale_rgb", 1.0)
                self.focal_scale_depth = settings.get("focal_scale_depth", 1.0)
                self.principal_point_offset_x = settings.get(
                    "principal_point_offset_x", 0.0
                )
                self.principal_point_offset_y = settings.get(
                    "principal_point_offset_y", 0.0
                )
                self.crop_scale_x = settings.get("crop_scale_x", 1.0)
                self.crop_scale_y = settings.get("crop_scale_y", 1.0)

                # Recalculate focal lengths and principal points
                self._calculate_focal_lengths()
                print(f"  Loaded intrinsics mode settings from {filename}")

            elif not self.use_intrinsics and mode_name == "simple":
                self.crop_top = settings.get("crop_top", 65)
                self.crop_bottom = settings.get("crop_bottom", 415)
                self.crop_left = settings.get("crop_left", 105)
                self.crop_right = settings.get("crop_right", 510)
                print(f"  Loaded simple mode settings from {filename}")
            else:
                print(
                    f"  Settings mode mismatch: expected {'intrinsics' if self.use_intrinsics else 'simple'}, got {mode_name}"
                )
                return False

            return True
        except Exception as e:
            print(f"  Error loading settings: {e}")
            return False

    def save_alignment_settings(self):
        """Save current alignment settings to JSON file."""
        filename = (
            "alignment_settings_intrinsics.json"
            if self.use_intrinsics
            else "alignment_settings_simple.json"
        )

        if self.use_intrinsics:
            settings = {
                "mode": "intrinsics",
                "focal_scale_rgb": self.focal_scale_rgb,
                "focal_scale_depth": self.focal_scale_depth,
                "principal_point_offset_x": self.principal_point_offset_x,
                "principal_point_offset_y": self.principal_point_offset_y,
                "crop_scale_x": self.crop_scale_x,
                "crop_scale_y": self.crop_scale_y,
            }
        else:
            settings = {
                "mode": "simple",
                "crop_top": self.crop_top,
                "crop_bottom": self.crop_bottom,
                "crop_left": self.crop_left,
                "crop_right": self.crop_right,
            }

        filepath = os.path.join(os.path.dirname(__file__), filename)
        with open(filepath, "w") as f:
            json.dump(settings, f, indent=4)

        print(f"Settings saved to: {filepath}")
        return filepath

    def terminate(self):
        """Clean up camera resources."""
        if hasattr(self, "camera"):
            self.camera.terminate()
        print("QCar2DepthAlignedCamera terminated")
