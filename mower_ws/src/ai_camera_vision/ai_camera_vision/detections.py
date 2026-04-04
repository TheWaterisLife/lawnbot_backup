"""Detection data structures and helper functions.

PURPOSE:
    Provides the Detection2D dataclass for representing YOLO detection results,
    along with COCO class names and safety category mappings.

WHAT THIS FILE DOES:
    - Defines Detection2D: A structured representation of a detected object
    - Contains COCO_CLASSES: 80 class names used by YOLO models
    - Contains SAFETY_CATEGORY_MAP: Maps classes to safety levels (human/animal/vehicle/etc.)
    - Helper functions: get_class_label(), get_safety_category(), apply_nms()

HOW TO EDIT:
    - To add new safety categories: Edit SAFETY_CATEGORY_MAP dictionary
    - To change confidence thresholds: Modify in calling code (demo_live_view.py)
    - Detection2D is a frozen dataclass - add new fields at the end to maintain compatibility

DEPENDENCIES (imports from):
    - None (standalone module)

USED BY:
    - demo_live_view.py: parse_img_detections() creates Detection2D objects
    - pipeline.py: Returns detection results containing Detection2D
    - visualization.py: Draws Detection2D boxes on frames
    - depth.py: Uses Detection2D for 3D position calculation
"""

from __future__ import annotations

from dataclasses import dataclass

# =============================================================================
# COCO Class Names (80 classes used by YOLOv8)
# =============================================================================

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
]

# =============================================================================
# Safety Category Mapping (per schema-v1.md)
# =============================================================================

SAFETY_CATEGORY_MAP = {
    # Must-avoid (high priority)
    "person": "human",
    
    # Animals (avoid)
    "dog": "animal",
    "cat": "animal",
    "bird": "animal",
    "horse": "animal",
    "sheep": "animal",
    "cow": "animal",
    "elephant": "animal",
    "bear": "animal",
    "zebra": "animal",
    "giraffe": "animal",
    
    # Vehicles (avoid)
    "car": "vehicle",
    "truck": "vehicle",
    "bus": "vehicle",
    "motorcycle": "vehicle",
    "bicycle": "vehicle",
    "train": "vehicle",
    "airplane": "vehicle",
    "boat": "vehicle",
}


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class Detection2D:
    """A single 2D detection from YOLO.
    
    Attributes:
        class_id: COCO class ID (0-79)
        class_label: Human-readable class name
        confidence: Detection confidence [0, 1]
        x_min: Left edge of bbox, normalized [0, 1]
        y_min: Top edge of bbox, normalized [0, 1]
        x_max: Right edge of bbox, normalized [0, 1]
        y_max: Bottom edge of bbox, normalized [0, 1]
        safety_category: Category for mower brain (human/animal/vehicle/static_obstacle/unknown)
    """
    class_id: int
    class_label: str
    confidence: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    safety_category: str = "unknown"
    
    @property
    def width(self) -> float:
        """Normalized width of bounding box."""
        return self.x_max - self.x_min
    
    @property
    def height(self) -> float:
        """Normalized height of bounding box."""
        return self.y_max - self.y_min
    
    @property
    def center_x(self) -> float:
        """Normalized center X coordinate."""
        return (self.x_min + self.x_max) / 2.0
    
    @property
    def center_y(self) -> float:
        """Normalized center Y coordinate."""
        return (self.y_min + self.y_max) / 2.0
    
    def to_dict(self) -> dict:
        """Convert to dictionary for serialization (schema v1 format)."""
        return {
            "class_id": self.class_id,
            "class_label": self.class_label,
            "confidence": self.confidence,
            "bbox": {
                "x_min": self.x_min,
                "y_min": self.y_min,
                "x_max": self.x_max,
                "y_max": self.y_max,
            },
            "safety_category": self.safety_category,
        }
    
    def to_pixels(self, frame_width: int, frame_height: int) -> tuple[int, int, int, int]:
        """Convert normalized bbox to pixel coordinates.
        
        Args:
            frame_width: Frame width in pixels
            frame_height: Frame height in pixels
            
        Returns:
            Tuple of (x1, y1, x2, y2) in pixels
        """
        return (
            int(self.x_min * frame_width),
            int(self.y_min * frame_height),
            int(self.x_max * frame_width),
            int(self.y_max * frame_height),
        )


# =============================================================================
# Helper Functions
# =============================================================================

def get_class_label(class_id: int) -> str:
    """Get human-readable class label from COCO class ID.
    
    Args:
        class_id: COCO class ID (0-79)
        
    Returns:
        Class label string, or "class_N" if unknown
    """
    if 0 <= class_id < len(COCO_CLASSES):
        return COCO_CLASSES[class_id]
    return f"class_{class_id}"


def get_safety_category(class_label: str) -> str:
    """Map COCO class label to safety category.
    
    Args:
        class_label: COCO class name
        
    Returns:
        Safety category: human, animal, vehicle, static_obstacle, or unknown
    """
    return SAFETY_CATEGORY_MAP.get(class_label, "unknown")


def apply_nms(
    detections: list[Detection2D],
    iou_threshold: float = 0.45,
) -> list[Detection2D]:
    """Apply Non-Maximum Suppression to filter overlapping boxes.
    
    Args:
        detections: List of detections to filter
        iou_threshold: IoU threshold for suppression
        
    Returns:
        Filtered list of detections
    """
    if not detections:
        return []
    
    # Sort by confidence (highest first)
    detections = sorted(detections, key=lambda d: d.confidence, reverse=True)
    
    kept = []
    while detections:
        # Keep the highest confidence detection
        best = detections.pop(0)
        kept.append(best)
        
        # Filter out overlapping detections
        remaining = []
        for det in detections:
            iou = _compute_iou(best, det)
            if iou < iou_threshold:
                remaining.append(det)
        detections = remaining
    
    return kept


def _compute_iou(det1: Detection2D, det2: Detection2D) -> float:
    """Compute Intersection over Union between two detections."""
    # Intersection
    x1 = max(det1.x_min, det2.x_min)
    y1 = max(det1.y_min, det2.y_min)
    x2 = min(det1.x_max, det2.x_max)
    y2 = min(det1.y_max, det2.y_max)
    
    if x2 <= x1 or y2 <= y1:
        return 0.0
    
    intersection = (x2 - x1) * (y2 - y1)
    
    # Union
    area1 = det1.width * det1.height
    area2 = det2.width * det2.height
    union = area1 + area2 - intersection
    
    if union <= 0:
        return 0.0
    
    return intersection / union

# =============================================================================
# Note: Raw YOLO tensor decoding removed
# =============================================================================
# The decode_yolo_output() function was removed because we now use
# dai.node.YoloDetectionNetwork which performs on-device decoding.
# The output is already structured as dai.ImgDetections.

