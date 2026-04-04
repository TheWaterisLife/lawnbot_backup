---
title: Story 3.2 — Overlay YOLO detections (boxes + labels)
description: Draw bounding boxes and class labels on the RGB preview
date: 2026-01-14
epic: epic-03
status: DONE
---

# Story 3.2 — Overlay YOLO detections (boxes + labels)

## Objective

Render YOLO detection bounding boxes and class labels on the RGB preview to visually validate detection correctness.

## Acceptance Criteria

- [ ] Boxes and labels render in correct positions relative to the preview
- [ ] Confidence threshold can be configured
- [ ] Overlay uses the coordinate convention from ADR-003 (normalized bbox)
- [ ] Different colors for different safety categories
- [ ] Confidence score displayed with label

## Technical Design

### Bounding Box Coordinate Conversion

Per schema-v1.md, bbox is normalized `[0, 1]`. Convert to pixel coordinates:

```python
def bbox_to_pixels(bbox: dict, frame_width: int, frame_height: int) -> tuple:
    """Convert normalized bbox to pixel coordinates.
    
    Args:
        bbox: { x_min, y_min, x_max, y_max } normalized [0,1]
        frame_width: Preview frame width in pixels
        frame_height: Preview frame height in pixels
    
    Returns:
        (x1, y1, x2, y2) in pixels
    """
    x1 = int(bbox['x_min'] * frame_width)
    y1 = int(bbox['y_min'] * frame_height)
    x2 = int(bbox['x_max'] * frame_width)
    y2 = int(bbox['y_max'] * frame_height)
    return (x1, y1, x2, y2)
```

### Color Scheme by Safety Category

| Safety Category | Color (BGR) | Hex |
|-----------------|-------------|-----|
| human | Red | (0, 0, 255) |
| animal | Orange | (0, 165, 255) |
| vehicle | Blue | (255, 0, 0) |
| static_obstacle | Yellow | (0, 255, 255) |
| unknown | Gray | (128, 128, 128) |

### Drawing Function

```python
def draw_detections(
    frame: np.ndarray,
    detections: list[Detection],
    confidence_threshold: float = 0.5,
) -> np.ndarray:
    """Draw detection boxes and labels on frame.
    
    Args:
        frame: BGR image (will be modified in place)
        detections: List of Detection objects
        confidence_threshold: Skip detections below this confidence
    
    Returns:
        Frame with overlays drawn
    """
    h, w = frame.shape[:2]
    
    for det in detections:
        if det.confidence < confidence_threshold:
            continue
        
        # Get pixel coordinates
        x1, y1, x2, y2 = bbox_to_pixels(det.bbox, w, h)
        
        # Get color based on safety category
        color = SAFETY_COLORS.get(det.safety_category, (128, 128, 128))
        
        # Draw rectangle
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        
        # Draw label with confidence
        label = f"{det.class_label}: {det.confidence:.2f}"
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    return frame
```

### Label Positioning

- Label above box by default
- If box is near top edge, label inside box
- Background rectangle for better readability (optional)

## Implementation

### Files Modified

- `src/ai_camera_vision/ai_camera_vision/visualization.py`
  - Add `draw_detections()` function
  - Add color mapping for safety categories

### Files Added

- `tests/unit/test_story_3_2_detection_overlay.py`
- `bmad/stories/story-3-2-detection-overlay.md` (this file)

## Test Plan

### Unit Tests (no hardware)

1. Coordinate conversion from normalized to pixels
2. Color mapping for all safety categories
3. Confidence threshold filtering
4. Label text formatting
5. Edge case: detection at frame boundary

### Manual Tests (OAK-D Lite)

1. Boxes align with detected objects
2. Labels are readable
3. Colors match safety categories
4. Low-confidence detections filtered out

## Dependencies

- Story 3.1 (RGB preview window)
- Story 1.1 (schema defines bbox format)
- Story 2.2 (pipeline provides detections)

## Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `detection_confidence_threshold` | float | 0.5 | Skip detections below this |
| `show_detection_boxes` | bool | true | Enable/disable box overlay |

## Notes

- YOLO model input is 416×416, but preview may be different size
- Must account for any letterboxing/padding in coordinate mapping
- Consider adding 3D distance label when depth is available (Story 2.5)

