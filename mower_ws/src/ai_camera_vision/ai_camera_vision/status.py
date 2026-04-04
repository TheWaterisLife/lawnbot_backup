"""Vision system status and health monitoring.

PURPOSE:
    Computes overall system health (OK/WARN/ERROR) based on FPS and drop rates.
    Published to ROS2 for the mower brain to monitor.

WHAT THIS FILE DOES:
    - StatusLevel: Enum (OK, WARN, ERROR, STALE)
    - VisionStatus: Dataclass with all health metrics
    - compute_status(): Computes status from RuntimeStats

STATUS LEVELS:
    - OK: All streams active, acceptable FPS
    - WARN: High drop rate or low FPS
    - ERROR: Critical failure or stream inactive
    - STALE: No data for >5 seconds

KEY CONFIGURATION (thresholds):
    - MIN_FPS_OK = 5.0: Below this → WARN
    - MIN_FPS_ERROR = 1.0: Below this → ERROR
    - MAX_DROP_RATE_OK = 50.0: Above this → WARN
    - STALE_TIMEOUT_SEC = 5.0: No data this long → STALE

HOW TO EDIT:
    - To change thresholds: Edit MIN_FPS_*, MAX_DROP_RATE_OK constants
    - To add new status fields: Add to VisionStatus dataclass

DEPENDENCIES (imports from):
    - ai_camera_vision.stats: RuntimeStats (TYPE_CHECKING only)
    - json, time, dataclasses, enum

USED BY:
    - node.py: Publishes VisionStatus to /vision/status
    - demo_live_view.py: Could use for status display
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ai_camera_vision.stats import RuntimeStats


class StatusLevel(str, Enum):
    """Health status levels."""
    OK = "OK"
    WARN = "WARN"
    ERROR = "ERROR"
    STALE = "STALE"


# Thresholds for status determination
MIN_FPS_OK = 5.0        # Below this → WARN
MIN_FPS_ERROR = 1.0     # Below this → ERROR
MAX_DROP_RATE_OK = 50.0 # Above this → WARN
STALE_TIMEOUT_SEC = 5.0 # No data for this long → STALE


@dataclass
class VisionStatus:
    """Vision system health status.
    
    Published to /vision/status for the mower brain to monitor.
    """
    timestamp_ns: int
    status: StatusLevel
    detection_fps: float
    segmentation_fps: float
    depth_fps: float
    preview_fps: float
    detection_active: bool
    segmentation_active: bool
    depth_active: bool
    drop_rate_detection: float
    drop_rate_segmentation: float
    last_error: str | None
    uptime_sec: float
    software_version: str = "1.0.0"
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            "timestamp_ns": self.timestamp_ns,
            "status": self.status.value,
            "detection_fps": round(self.detection_fps, 1),
            "segmentation_fps": round(self.segmentation_fps, 1),
            "depth_fps": round(self.depth_fps, 1),
            "preview_fps": round(self.preview_fps, 1),
            "detection_active": self.detection_active,
            "segmentation_active": self.segmentation_active,
            "depth_active": self.depth_active,
            "drop_rate_detection": round(self.drop_rate_detection, 1),
            "drop_rate_segmentation": round(self.drop_rate_segmentation, 1),
            "last_error": self.last_error,
            "uptime_sec": round(self.uptime_sec, 1),
            "software_version": self.software_version,
        }
    
    def to_json(self) -> str:
        """Convert to JSON string."""
        return json.dumps(self.to_dict())
    
    @property
    def is_healthy(self) -> bool:
        """Check if system is healthy (OK status)."""
        return self.status == StatusLevel.OK
    
    @property
    def is_critical(self) -> bool:
        """Check if system has critical error."""
        return self.status == StatusLevel.ERROR or not self.detection_active


def compute_status(
    stats: RuntimeStats | None,
    last_error: str | None = None,
    depth_enabled: bool = True,
) -> VisionStatus:
    """Compute current vision system status.
    
    Args:
        stats: RuntimeStats instance with current metrics
        last_error: Most recent error message (None if no error)
        depth_enabled: Whether depth stream is expected
    
    Returns:
        VisionStatus with current health information
    """
    timestamp_ns = int(time.time() * 1_000_000_000)
    
    # Default values if no stats
    if stats is None:
        return VisionStatus(
            timestamp_ns=timestamp_ns,
            status=StatusLevel.ERROR,
            detection_fps=0.0,
            segmentation_fps=0.0,
            depth_fps=0.0,
            preview_fps=0.0,
            detection_active=False,
            segmentation_active=False,
            depth_active=False,
            drop_rate_detection=0.0,
            drop_rate_segmentation=0.0,
            last_error="No statistics available",
            uptime_sec=0.0,
        )
    
    # Get stream stats
    det_stats = stats.get_stream("detection")
    seg_stats = stats.get_stream("segmentation")
    depth_stats = stats.get_stream("depth")
    preview_stats = stats.get_stream("preview")
    
    detection_fps = det_stats.fps if det_stats else 0.0
    segmentation_fps = seg_stats.fps if seg_stats else 0.0
    depth_fps = depth_stats.fps if depth_stats else 0.0
    preview_fps = preview_stats.fps if preview_stats else 0.0
    
    drop_rate_detection = det_stats.drop_rate if det_stats else 0.0
    drop_rate_segmentation = seg_stats.drop_rate if seg_stats else 0.0
    
    # Determine if streams are active
    detection_active = det_stats is not None and det_stats.frame_count > 0
    segmentation_active = seg_stats is not None and seg_stats.frame_count > 0
    depth_active = depth_stats is not None and depth_stats.frame_count > 0
    
    # Compute status level
    status = _compute_status_level(
        detection_fps=detection_fps,
        segmentation_fps=segmentation_fps,
        depth_fps=depth_fps,
        detection_active=detection_active,
        segmentation_active=segmentation_active,
        depth_active=depth_active,
        drop_rate_detection=drop_rate_detection,
        drop_rate_segmentation=drop_rate_segmentation,
        last_error=last_error,
        depth_enabled=depth_enabled,
    )
    
    return VisionStatus(
        timestamp_ns=timestamp_ns,
        status=status,
        detection_fps=detection_fps,
        segmentation_fps=segmentation_fps,
        depth_fps=depth_fps,
        preview_fps=preview_fps,
        detection_active=detection_active,
        segmentation_active=segmentation_active,
        depth_active=depth_active,
        drop_rate_detection=drop_rate_detection,
        drop_rate_segmentation=drop_rate_segmentation,
        last_error=last_error,
        uptime_sec=stats.uptime_sec,
    )


def _compute_status_level(
    detection_fps: float,
    segmentation_fps: float,
    depth_fps: float,
    detection_active: bool,
    segmentation_active: bool,
    depth_active: bool,
    drop_rate_detection: float,
    drop_rate_segmentation: float,
    last_error: str | None,
    depth_enabled: bool,
) -> StatusLevel:
    """Compute overall status level from metrics.
    
    Priority:
    1. ERROR if critical error or detection inactive
    2. WARN if low FPS or high drop rate
    3. OK if all checks pass
    """
    # Critical errors
    if last_error is not None:
        return StatusLevel.ERROR
    
    if not detection_active:
        return StatusLevel.ERROR
    
    if not segmentation_active:
        return StatusLevel.ERROR
    
    if depth_enabled and not depth_active:
        return StatusLevel.WARN  # Depth is optional, warn only
    
    # FPS checks
    if detection_fps < MIN_FPS_ERROR or segmentation_fps < MIN_FPS_ERROR:
        return StatusLevel.ERROR
    
    if detection_fps < MIN_FPS_OK or segmentation_fps < MIN_FPS_OK:
        return StatusLevel.WARN
    
    # Drop rate checks
    if drop_rate_detection > MAX_DROP_RATE_OK:
        return StatusLevel.WARN
    
    if drop_rate_segmentation > MAX_DROP_RATE_OK:
        return StatusLevel.WARN
    
    return StatusLevel.OK
