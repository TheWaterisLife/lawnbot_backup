---
title: Story 2.7 — YOLO output decoder
description: Decode raw YOLO tensor output into Detection2DArray messages
date: 2026-01-14
epic: epic-02
status: DONE
priority: P0
---

# Story 2.7 — YOLO output decoder

## Objective

Create a decoder module that converts raw YOLO neural network tensor outputs into structured detection messages ready for ROS2 publishing.

## Background

The pipeline (Story 2.2) returns raw `dai.NNData` objects containing YOLO tensor outputs. These must be decoded into bounding boxes with class labels before they can be published to `/vision/detections`.

**Current gap**: The pipeline works, but nothing decodes the output tensors into detections.

## Acceptance Criteria

- [x] Create `src/ai_camera_vision/ai_camera_vision/detections.py` module
- [x] Decode YOLO output tensors into list of detections
- [x] Each detection has: class_id, class_label, confidence, bbox (normalized)
- [x] Apply Non-Maximum Suppression (NMS) to filter overlapping boxes
- [x] Configurable confidence threshold (default 0.25)
- [x] Unit tests verify decoder works with known tensor data (35 tests passing)
- [x] Works with `yolov8n_coco_416x416.blob` output format

## Technical Design

### YOLO Output Format

The YOLOv8n blob outputs 3 feature maps at different scales:
- `output1_yolov6r2`: 229,840 values (large feature map)
- `output2_yolov6r2`: 57,460 values (medium feature map)
- `output3_yolov6r2`: 14,365 values (small feature map)

Each detection contains 85 values:
- 4 bbox values: x_center, y_center, width, height (normalized 0-1)
- 1 objectness score
- 80 class scores (COCO classes)

### Module Interface

```python
# src/ai_camera_vision/ai_camera_vision/detections.py

from dataclasses import dataclass
from typing import Any

@dataclass
class Detection2D:
    """A single 2D detection."""
    class_id: int
    class_label: str
    confidence: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    safety_category: str = "unknown"

def decode_yolo_output(
    nn_data: Any,  # dai.NNData
    confidence_threshold: float = 0.25,
    nms_threshold: float = 0.45,
) -> list[Detection2D]:
    """Decode YOLO tensor output to detection list."""
    ...

def get_safety_category(class_label: str) -> str:
    """Map COCO class to safety category (human/animal/vehicle/unknown)."""
    ...
```

### Safety Category Mapping

Per schema-v1.md safety-critical classes:

| COCO Class | Safety Category |
|------------|-----------------|
| person | human |
| dog, cat, bird, horse, cow, sheep | animal |
| car, truck, bus, motorcycle, bicycle | vehicle |
| chair, bench, potted plant | static_obstacle |
| (others) | unknown |

## Files to Create

- `src/ai_camera_vision/ai_camera_vision/detections.py`
- `tests/unit/test_story_2_7_detections.py`

## Files to Modify

- `src/ai_camera_vision/ai_camera_vision/__init__.py` (export new module)

## Test Plan

### Unit Tests (no hardware)

1. Decode synthetic YOLO-format tensor data
2. NMS filters overlapping boxes correctly
3. Confidence threshold filtering
4. Safety category mapping for all categories
5. Empty output handling (no detections)
6. Invalid tensor format handling

### Integration Tests (hardware)

1. Run pipeline, decode output, verify detections make sense
2. Point camera at person, verify person detected

## Dependencies

- Story 2.2 (pipeline provides raw NN output)
- Story 1.1 (schema defines detection format)

## Notes

- This is a **critical missing piece** - without it, the pipeline outputs can't be used
- The `demo_live_view.py` has a prototype decoder that needs to be moved to `src/`
- Once implemented, Story 2.8 will use this to publish to ROS2 topics

## Implementation Notes (2026-01-14)

**Implemented in**: `src/ai_camera_vision/ai_camera_vision/detections.py`

**Key features**:
- `Detection2D` dataclass with full schema v1 fields
- `decode_yolo_output()` handles both 84 and 85-value tensor formats
- `apply_nms()` for Non-Maximum Suppression
- Safety category mapping for all COCO classes
- 35 unit tests passing in `tests/unit/test_story_2_7_detections.py`

**FOV Change**: Pipeline now uses 16:9 preview (640×352) with resizing/letterboxing to 416x416 for NN. The demo includes `adjust_bbox_from_letterbox()` to compensate.

