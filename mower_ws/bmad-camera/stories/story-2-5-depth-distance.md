---
title: Story 2.5 — Depth-based distance output (3D detections)
description: Add stereo depth to pipeline and compute 3D positions for detected obstacles
date: 2026-01-12
epic: epic-02
status: DONE
---

# Story 2.5 — Depth-based distance output (3D detections)

## Objective

Extend the pipeline to produce depth-aligned to RGB and compute 3D spatial positions for each detection, enabling the mower brain to know how far obstacles are.

## Acceptance Criteria

- [x] Pipeline produces depth aligned to the chosen RGB reference frame
- [x] For each obstacle-like detection, can compute distance in meters
- [x] Target usable range: 1.5 m (per specs.md obstacle detection requirement)
- [x] Size estimates (x/y/z dimensions) when feasible
- [x] Document assumptions and limitations

## Technical Design

### OAK-D Lite Stereo Setup

The OAK-D Lite has three cameras:
- **CAM_A (IMX214)**: RGB color camera (4208×3120, used at 1080p)
- **CAM_B (OV7251)**: Left mono camera (640×480)
- **CAM_C (OV7251)**: Right mono camera (640×480)

Baseline (stereo separation): ~75mm

### Updated Pipeline Architecture

```
MonoCamera (Left)  ─┐
                    ├──► StereoDepth ──► XLinkOut (depth)
MonoCamera (Right) ─┘         │
                              │ (aligned to RGB)
                              ▼
ColorCamera (RGB) ──► ImageManip ──► NeuralNetwork (YOLO) ──► XLinkOut
         │
         └──► ImageManip ──► NeuralNetwork (Seg) ──► XLinkOut
```

### Depth Configuration

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Resolution | 400p (640×400) | Balance quality vs. performance |
| Median Filter | 7×7 | Reduce noise |
| Depth Align | RGB | Match depth pixels to color pixels |
| Extended Disparity | Off | Not needed for close range |
| Subpixel | Off | Performance over precision |
| LR Check | On | Filter bad matches |

### Distance Calculation

For each detection bounding box:

1. **Sample depth at center**: `depth_mm = depth_frame[center_y, center_x]`
2. **Convert to meters**: `distance_m = depth_mm / 1000.0`
3. **Handle invalid depth**: If depth is 0 or >10m, mark as invalid

### Size Estimation

Given:
- Detection bbox: `(x_min, y_min, x_max, y_max)` normalized [0,1]
- Distance: `Z` meters
- Camera FOV: ~73° horizontal for RGB

Approximate size:
```python
# Convert normalized bbox to pixels (at depth frame resolution)
width_px = (x_max - x_min) * frame_width
height_px = (y_max - y_min) * frame_height

# Use pinhole camera model: size = (pixels * distance) / focal_length
# For OAK-D Lite RGB: fx ≈ 600 (approximate)
width_m = (width_px * distance_m) / focal_length
height_m = (height_px * distance_m) / focal_length
```

### Detection3D Structure

```python
@dataclass
class Detection3D:
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
    size_x: float | None  # Width
    size_y: float | None  # Height
    size_z: float | None  # Depth (usually unknown from mono view)
    
    # Quality flags
    depth_valid: bool
    depth_confidence: float  # 0-1
```

### Coordinate Frame

Per schema v1, use **camera optical frame** convention:
- **+X**: Right
- **+Y**: Down
- **+Z**: Forward (into scene)

Frame ID: `oak_rgb_optical_frame`

## Assumptions and Limitations

### Assumptions

1. **Flat ground plane**: Size estimation assumes objects are upright
2. **Detection center = object center**: Depth sampled at bbox center
3. **Rigid objects**: Size doesn't change with viewing angle
4. **Good lighting**: Stereo matching requires texture and contrast

### Limitations

1. **Close range accuracy**: < 0.5m depth becomes unreliable
2. **Far range**: > 3m depth increasingly noisy
3. **Target range**: 1.5m (per specs) is well within reliable range
4. **Textureless surfaces**: Stereo fails on uniform surfaces (white walls)
5. **Reflective surfaces**: Glass, mirrors, water give wrong depth
6. **Occlusion**: Partially hidden objects have unreliable depth
7. **Size Z-dimension**: Cannot estimate depth-dimension of object (only width/height)
8. **Moving objects**: Fast motion causes stereo mismatch
9. **Performance**: Running YOLOv8n + Segmentation + Depth concurrently pushes OAK-D Lite to limits. May cause instability/stalling.

### Edge Cases

| Scenario | Behavior |
|----------|----------|
| Invalid depth (0) | `depth_valid = False`, distance set to -1 |
| Depth > 10m | Clamp to 10m, mark low confidence |
| Depth < 0.2m | Mark low confidence (too close) |
| Bbox at image edge | Depth may be unreliable, mark low confidence |
| Multiple objects overlapping | Each detection uses its own bbox center |

## Implementation

### Files Modified

- `src/ai_camera_vision/ai_camera_vision/pipeline.py`
  - Add `enable_depth` parameter to `create_pipeline()`
  - Add MonoCamera and StereoDepth nodes
  - Add depth output stream
  - Add `get_depth_frame()` to PipelineRunner

### Files Added

- `src/ai_camera_vision/ai_camera_vision/depth.py` — Spatial calculation utilities
- `bmad/stories/story-2-5-depth-distance.md` (this file)
- `tests/unit/test_story_2_5_depth.py`

## Test Plan

### Unit Tests (no hardware)

1. Distance calculation from depth value
2. Size estimation from bbox + distance
3. Detection3D dataclass construction
4. Coordinate frame conventions
5. Edge case handling (invalid depth, out of range)

### Hardware Tests (OAK-D Lite)

1. Depth stream produces valid frames
2. Depth values are in expected range (mm)
3. Known-distance object test (place object at 1m, verify)
4. Full pipeline: detection + depth integration

## Dependencies

- Story 2.2 (base pipeline)
- Story 2.3 (timing metadata for frame correlation)

## Notes

- USB3 mode provides better depth frame rate (~19 FPS on dev machine)
- Depth frame may have different sequence numbers than RGB/NN frames
- Consider adding depth confidence map in future iteration

