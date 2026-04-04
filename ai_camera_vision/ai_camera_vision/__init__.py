"""ai_camera_vision ROS2 package.

Main modules:
- pipeline: DepthAI dual-NN pipeline for OAK-D Lite
- detections: Detection data structures
- depth: 3D detection calculations
- zone_summary: Segmentation zone ratios
- stats: Runtime statistics
- status: Health monitoring
- visualization: Preview and overlays
- validation: Startup validation
- jsonl_writer: Debug logging
- node: ROS2 node entry point
"""

from ai_camera_vision.detections import (
    COCO_CLASSES,
    Detection2D,
    get_class_label,
    get_safety_category,
)
from ai_camera_vision.zone_summary import (
    ZoneSummary,
    compute_zone_ratios,
    decode_segmentation_mask,
)
from ai_camera_vision.stats import RuntimeStats, StreamStats
from ai_camera_vision.status import VisionStatus, StatusLevel, compute_status

__all__ = [
    # Detections
    "COCO_CLASSES",
    "Detection2D",
    "get_class_label",
    "get_safety_category",
    # Zone summary
    "ZoneSummary",
    "compute_zone_ratios",
    "decode_segmentation_mask",
    # Stats
    "RuntimeStats",
    "StreamStats",
    # Status
    "VisionStatus",
    "StatusLevel",
    "compute_status",
]
