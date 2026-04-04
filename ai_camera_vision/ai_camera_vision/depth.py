"""Depth and spatial calculation utilities for 3D detections.

PURPOSE:
    Converts 2D detections + depth data into 3D positions and sizes.
    Used by the mower brain to know how far away obstacles are.

WHAT THIS FILE DOES:
    - BoundingBox2D: Normalized 2D bounding box
    - Detection3D: 3D detection with position and size in meters
    - sample_depth_at_bbox_center(): Get depth at detection center
    - compute_detection_3d(): Convert 2D detection to 3D

COORDINATE FRAME (camera optical frame):
    - +X: Right
    - +Y: Down
    - +Z: Forward (into scene)

KEY CONFIGURATION:
    - DEFAULT_FOCAL_LENGTH_PX = 600.0: Camera focal length (adjust for calibration)
    - MIN_VALID_DEPTH_M = 0.2: Below this, depth is unreliable
    - MAX_VALID_DEPTH_M = 10.0: Above this, depth is unreliable
    - TARGET_RANGE_M = 1.5: Optimal detection range for mower

HOW TO EDIT:
    - To change valid depth range: Edit MIN/MAX_VALID_DEPTH_M constants
    - To calibrate camera: Update DEFAULT_FOCAL_LENGTH_PX
    - To add confidence logic: Edit compute_depth_confidence()

DEPENDENCIES (imports from):
    - numpy: Array operations

USED BY:
    - visualization.py: _get_detection_distance() uses depth sampling
    - node.py: Publishes Detection3D to ROS2 topics
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


# =============================================================================
# Constants
# =============================================================================

# OAK-D Lite RGB camera approximate focal length (pixels at 1080p)
# This is an approximation; real value depends on calibration
DEFAULT_FOCAL_LENGTH_PX = 600.0

# Depth range limits (meters)
MIN_VALID_DEPTH_M = 0.2  # Below this, depth is unreliable
MAX_VALID_DEPTH_M = 10.0  # Above this, depth is unreliable
TARGET_RANGE_M = 1.5  # Per specs.md obstacle detection requirement

# Confidence thresholds
DEPTH_CONFIDENCE_HIGH = 0.9
DEPTH_CONFIDENCE_MEDIUM = 0.7
DEPTH_CONFIDENCE_LOW = 0.3


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class BoundingBox2D:
    """2D bounding box in normalized coordinates [0, 1].

    Attributes:
        x_min: Left edge (0 = image left)
        y_min: Top edge (0 = image top)
        x_max: Right edge (1 = image right)
        y_max: Bottom edge (1 = image bottom)
    """

    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def width(self) -> float:
        """Normalized width."""
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        """Normalized height."""
        return self.y_max - self.y_min

    @property
    def center_x(self) -> float:
        """Normalized center X coordinate."""
        return (self.x_min + self.x_max) / 2.0

    @property
    def center_y(self) -> float:
        """Normalized center Y coordinate."""
        return (self.y_min + self.y_max) / 2.0

    def to_pixels(self, frame_width: int, frame_height: int) -> tuple[int, int, int, int]:
        """Convert to pixel coordinates.

        Args:
            frame_width: Frame width in pixels.
            frame_height: Frame height in pixels.

        Returns:
            Tuple of (x_min, y_min, x_max, y_max) in pixels.
        """
        return (
            int(self.x_min * frame_width),
            int(self.y_min * frame_height),
            int(self.x_max * frame_width),
            int(self.y_max * frame_height),
        )

    def center_pixels(self, frame_width: int, frame_height: int) -> tuple[int, int]:
        """Get center in pixel coordinates.

        Args:
            frame_width: Frame width in pixels.
            frame_height: Frame height in pixels.

        Returns:
            Tuple of (center_x, center_y) in pixels.
        """
        return (
            int(self.center_x * frame_width),
            int(self.center_y * frame_height),
        )


@dataclass
class Detection3D:
    """3D detection with spatial position and size.

    Attributes:
        class_id: COCO class ID.
        class_label: Human-readable class name.
        confidence: Detection confidence [0, 1].
        bbox_2d: Original 2D bounding box (normalized).
        position_x: X position in meters (right = positive).
        position_y: Y position in meters (down = positive).
        position_z: Z position in meters (forward/depth).
        size_x: Estimated width in meters (None if unavailable).
        size_y: Estimated height in meters (None if unavailable).
        size_z: Estimated depth in meters (usually None - can't measure from single view).
        depth_valid: Whether depth measurement is valid.
        depth_confidence: Confidence in depth measurement [0, 1].
    """

    class_id: int
    class_label: str
    confidence: float

    # 2D bbox (normalized)
    bbox_2d: BoundingBox2D

    # 3D position (meters, camera frame)
    position_x: float  # Right (+) / Left (-)
    position_y: float  # Down (+) / Up (-)
    position_z: float  # Forward (depth)

    # 3D size estimates (meters), None if unavailable
    size_x: float | None = None  # Width
    size_y: float | None = None  # Height
    size_z: float | None = None  # Depth (usually None)

    # Quality flags
    depth_valid: bool = True
    depth_confidence: float = 1.0

    @property
    def distance(self) -> float:
        """Distance from camera (same as position_z for forward-facing camera)."""
        return self.position_z

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            "class_id": self.class_id,
            "class_label": self.class_label,
            "confidence": self.confidence,
            "bbox_2d": {
                "x_min": self.bbox_2d.x_min,
                "y_min": self.bbox_2d.y_min,
                "x_max": self.bbox_2d.x_max,
                "y_max": self.bbox_2d.y_max,
            },
            "position": {
                "x": self.position_x,
                "y": self.position_y,
                "z": self.position_z,
            },
            "size": {
                "x": self.size_x,
                "y": self.size_y,
                "z": self.size_z,
            },
            "depth_valid": self.depth_valid,
            "depth_confidence": self.depth_confidence,
        }


# =============================================================================
# Depth Sampling Functions
# =============================================================================


def sample_depth_at_point(
    depth_frame: np.ndarray,
    x: int,
    y: int,
) -> int:
    """Sample depth value at a specific pixel location.

    Args:
        depth_frame: Depth frame as numpy array (values in mm).
        x: X coordinate in pixels.
        y: Y coordinate in pixels.

    Returns:
        Depth value in millimeters, or 0 if out of bounds.
    """
    h, w = depth_frame.shape[:2]

    # Clamp to valid range
    x = max(0, min(x, w - 1))
    y = max(0, min(y, h - 1))

    return int(depth_frame[y, x])


def sample_depth_at_bbox_center(
    depth_frame: np.ndarray,
    bbox: BoundingBox2D,
    frame_shape: tuple[int, int] | None = None,
) -> int:
    """Sample depth at the center of a bounding box.

    Args:
        depth_frame: Depth frame as numpy array (values in mm).
        bbox: 2D bounding box (normalized coordinates).
        frame_shape: Optional (height, width) of depth frame.
                    If None, uses depth_frame.shape.

    Returns:
        Depth value in millimeters at bbox center.
    """
    if frame_shape is None:
        h, w = depth_frame.shape[:2]
    else:
        h, w = frame_shape

    cx, cy = bbox.center_pixels(w, h)
    return sample_depth_at_point(depth_frame, cx, cy)


def sample_depth_robust(
    depth_frame: np.ndarray,
    bbox: BoundingBox2D,
    frame_shape: tuple[int, int] | None = None,
    sample_count: int = 5,
) -> tuple[int, float]:
    """Sample depth using multiple points for robustness.

    Samples depth at center and nearby points, returns median.
    This helps with noisy depth or objects with varying depth.

    Args:
        depth_frame: Depth frame as numpy array (values in mm).
        bbox: 2D bounding box (normalized coordinates).
        frame_shape: Optional (height, width) of depth frame.
        sample_count: Number of points to sample (1, 5, or 9).

    Returns:
        Tuple of (median_depth_mm, confidence).
        Confidence is fraction of valid (non-zero) samples.
    """
    if frame_shape is None:
        h, w = depth_frame.shape[:2]
    else:
        h, w = frame_shape

    cx, cy = bbox.center_pixels(w, h)

    # Sample points
    samples = []
    offsets = [(0, 0)]  # Always include center

    if sample_count >= 5:
        # Add 4 corners of inner region (25% from center to edge)
        dx = int(bbox.width * w * 0.25)
        dy = int(bbox.height * h * 0.25)
        offsets.extend([(-dx, -dy), (dx, -dy), (-dx, dy), (dx, dy)])

    if sample_count >= 9:
        # Add 4 edge midpoints
        offsets.extend([(0, -dy), (0, dy), (-dx, 0), (dx, 0)])

    for ox, oy in offsets:
        x = max(0, min(cx + ox, w - 1))
        y = max(0, min(cy + oy, h - 1))
        depth = int(depth_frame[y, x])
        if depth > 0:
            samples.append(depth)

    if not samples:
        return 0, 0.0

    median_depth = int(np.median(samples))
    confidence = len(samples) / len(offsets)

    return median_depth, confidence


# =============================================================================
# Spatial Calculation Functions
# =============================================================================


def depth_mm_to_meters(depth_mm: int) -> float:
    """Convert depth from millimeters to meters.

    Args:
        depth_mm: Depth in millimeters.

    Returns:
        Depth in meters.
    """
    return depth_mm / 1000.0


def compute_depth_confidence(depth_m: float) -> float:
    """Compute confidence score based on depth value.

    Args:
        depth_m: Depth in meters.

    Returns:
        Confidence score [0, 1].
    """
    if depth_m <= 0:
        return 0.0
    if depth_m < MIN_VALID_DEPTH_M:
        return DEPTH_CONFIDENCE_LOW
    if depth_m > MAX_VALID_DEPTH_M:
        return DEPTH_CONFIDENCE_LOW
    if depth_m <= TARGET_RANGE_M:
        return DEPTH_CONFIDENCE_HIGH
    # Gradual decrease beyond target range
    return max(
        DEPTH_CONFIDENCE_MEDIUM,
        DEPTH_CONFIDENCE_HIGH - (depth_m - TARGET_RANGE_M) * 0.1,
    )


def compute_3d_position(
    bbox: BoundingBox2D,
    depth_m: float,
    frame_width: int,
    frame_height: int,
    focal_length: float = DEFAULT_FOCAL_LENGTH_PX,
) -> tuple[float, float, float]:
    """Compute 3D position from 2D bbox center and depth.

    Uses pinhole camera model to back-project 2D point to 3D.

    Args:
        bbox: 2D bounding box (normalized).
        depth_m: Depth in meters.
        frame_width: Frame width in pixels.
        frame_height: Frame height in pixels.
        focal_length: Camera focal length in pixels.

    Returns:
        Tuple of (x, y, z) position in meters.
        - x: Right (+) / Left (-)
        - y: Down (+) / Up (-)
        - z: Forward (depth)
    """
    # Get center in pixels
    cx_px, cy_px = bbox.center_pixels(frame_width, frame_height)

    # Image center (principal point, assumed at image center)
    cx_img = frame_width / 2.0
    cy_img = frame_height / 2.0

    # Back-project using pinhole model
    # X = (x_px - cx) * Z / f
    # Y = (y_px - cy) * Z / f
    x_m = (cx_px - cx_img) * depth_m / focal_length
    y_m = (cy_px - cy_img) * depth_m / focal_length
    z_m = depth_m

    return x_m, y_m, z_m


def compute_3d_size(
    bbox: BoundingBox2D,
    depth_m: float,
    frame_width: int,
    frame_height: int,
    focal_length: float = DEFAULT_FOCAL_LENGTH_PX,
) -> tuple[float | None, float | None, float | None]:
    """Estimate 3D size from 2D bbox and depth.

    Args:
        bbox: 2D bounding box (normalized).
        depth_m: Depth in meters.
        frame_width: Frame width in pixels.
        frame_height: Frame height in pixels.
        focal_length: Camera focal length in pixels.

    Returns:
        Tuple of (size_x, size_y, size_z) in meters.
        size_z is always None (can't measure depth dimension from single view).
    """
    if depth_m <= 0:
        return None, None, None

    # Bbox size in pixels
    width_px = bbox.width * frame_width
    height_px = bbox.height * frame_height

    # Convert to meters using similar triangles
    # real_size = (pixel_size * depth) / focal_length
    size_x = (width_px * depth_m) / focal_length
    size_y = (height_px * depth_m) / focal_length
    size_z = None  # Cannot estimate depth dimension

    return size_x, size_y, size_z


# =============================================================================
# Main Detection3D Computation
# =============================================================================


def compute_detection_3d(
    class_id: int,
    class_label: str,
    confidence: float,
    bbox: BoundingBox2D,
    depth_mm: int,
    frame_width: int,
    frame_height: int,
    *,
    focal_length: float = DEFAULT_FOCAL_LENGTH_PX,
) -> Detection3D:
    """Compute a full 3D detection from 2D detection and depth.

    Args:
        class_id: COCO class ID.
        class_label: Human-readable class name.
        confidence: Detection confidence [0, 1].
        bbox: 2D bounding box (normalized).
        depth_mm: Depth at bbox center in millimeters.
        frame_width: Depth/RGB frame width in pixels.
        frame_height: Depth/RGB frame height in pixels.
        focal_length: Camera focal length in pixels.

    Returns:
        Detection3D with 3D position and size estimates.
    """
    depth_m = depth_mm_to_meters(depth_mm)
    depth_valid = depth_mm > 0 and MIN_VALID_DEPTH_M <= depth_m <= MAX_VALID_DEPTH_M
    depth_conf = compute_depth_confidence(depth_m) if depth_valid else 0.0

    # Handle invalid depth
    if not depth_valid:
        return Detection3D(
            class_id=class_id,
            class_label=class_label,
            confidence=confidence,
            bbox_2d=bbox,
            position_x=0.0,
            position_y=0.0,
            position_z=-1.0,  # -1 indicates invalid
            size_x=None,
            size_y=None,
            size_z=None,
            depth_valid=False,
            depth_confidence=0.0,
        )

    # Compute 3D position
    x, y, z = compute_3d_position(
        bbox, depth_m, frame_width, frame_height, focal_length
    )

    # Compute 3D size
    size_x, size_y, size_z = compute_3d_size(
        bbox, depth_m, frame_width, frame_height, focal_length
    )

    return Detection3D(
        class_id=class_id,
        class_label=class_label,
        confidence=confidence,
        bbox_2d=bbox,
        position_x=x,
        position_y=y,
        position_z=z,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        depth_valid=True,
        depth_confidence=depth_conf,
    )


def compute_detections_3d_batch(
    detections_2d: list[dict[str, Any]],
    depth_frame: np.ndarray,
    frame_width: int | None = None,
    frame_height: int | None = None,
    *,
    focal_length: float = DEFAULT_FOCAL_LENGTH_PX,
    robust_sampling: bool = True,
) -> list[Detection3D]:
    """Compute 3D detections for a batch of 2D detections.

    Args:
        detections_2d: List of 2D detection dicts with keys:
            - class_id: int
            - class_label: str
            - confidence: float
            - bbox: dict with x_min, y_min, x_max, y_max (normalized)
        depth_frame: Depth frame as numpy array (values in mm).
        frame_width: Override frame width (uses depth_frame.shape if None).
        frame_height: Override frame height (uses depth_frame.shape if None).
        focal_length: Camera focal length in pixels.
        robust_sampling: Use multi-point depth sampling for robustness.

    Returns:
        List of Detection3D objects.
    """
    if frame_width is None or frame_height is None:
        h, w = depth_frame.shape[:2]
        frame_width = frame_width or w
        frame_height = frame_height or h

    results = []
    for det in detections_2d:
        bbox = BoundingBox2D(
            x_min=det["bbox"]["x_min"],
            y_min=det["bbox"]["y_min"],
            x_max=det["bbox"]["x_max"],
            y_max=det["bbox"]["y_max"],
        )

        if robust_sampling:
            depth_mm, sample_conf = sample_depth_robust(
                depth_frame, bbox, (frame_height, frame_width)
            )
        else:
            depth_mm = sample_depth_at_bbox_center(
                depth_frame, bbox, (frame_height, frame_width)
            )

        det_3d = compute_detection_3d(
            class_id=det["class_id"],
            class_label=det["class_label"],
            confidence=det["confidence"],
            bbox=bbox,
            depth_mm=depth_mm,
            frame_width=frame_width,
            frame_height=frame_height,
            focal_length=focal_length,
        )

        # Adjust confidence if using robust sampling
        if robust_sampling and det_3d.depth_valid:
            det_3d = Detection3D(
                class_id=det_3d.class_id,
                class_label=det_3d.class_label,
                confidence=det_3d.confidence,
                bbox_2d=det_3d.bbox_2d,
                position_x=det_3d.position_x,
                position_y=det_3d.position_y,
                position_z=det_3d.position_z,
                size_x=det_3d.size_x,
                size_y=det_3d.size_y,
                size_z=det_3d.size_z,
                depth_valid=det_3d.depth_valid,
                depth_confidence=det_3d.depth_confidence * sample_conf,
            )

        results.append(det_3d)

    return results

