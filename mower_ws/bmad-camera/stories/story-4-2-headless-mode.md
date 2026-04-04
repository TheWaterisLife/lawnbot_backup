---
title: Story 4.2 — Headless mode is default and stable
description: Ensure application runs without GUI dependencies when visualization is disabled
date: 2026-01-14
epic: epic-04
status: DONE
---

# Story 4.2 — Headless mode is default and stable

## Objective

Ensure the AI Camera Vision node runs reliably without any GUI/display dependencies when visualization is disabled, which is the expected deployment mode on Raspberry Pi.

## Acceptance Criteria

- [ ] App runs without any GUI dependencies when visualization is off
- [ ] Output publishing works in headless mode (JSONL files created)
- [ ] ROS 2 topics publish correctly in headless mode
- [ ] No X11/display errors when running on headless system

## Technical Design

### Visualization Parameter

The `visualization_enabled` parameter (from Story 1.2) controls GUI mode:

```python
# Default is False (headless)
self.declare_parameter('visualization_enabled', False)
```

### Headless Requirements

When `visualization_enabled=False`:
1. No OpenCV window operations (`imshow`, `waitKey`, `namedWindow`)
2. No matplotlib or other GUI libraries
3. No X11 display required
4. All outputs via ROS 2 topics and/or JSONL files

### Conditional Imports

```python
# Only import GUI-related code when needed
if self.visualization_enabled:
    from .visualization import PreviewWindow, draw_detections
```

### JSONL Output in Headless Mode

Per Story 1.3, JSONL logging is optional and independent of visualization:

```python
# Enable JSONL logging (works in headless mode)
self.declare_parameter('jsonl_logging_enabled', False)
self.declare_parameter('jsonl_output_dir', '.')
```

### ROS 2 Topic Publishing

All topics work regardless of visualization mode:
- `/vision/detections`
- `/vision/detections_3d`
- `/vision/segmentation/mask`
- `/vision/segmentation/zone_summary`

### Error Handling for Missing Display

```python
try:
    if self.visualization_enabled:
        cv2.namedWindow("Preview")
except cv2.error as e:
    self.get_logger().warning(f"Cannot create display window: {e}")
    self.get_logger().warning("Disabling visualization")
    self.visualization_enabled = False
```

## Implementation

### Files Modified

- `src/ai_camera_vision/ai_camera_vision/node.py`
  - Ensure visualization code is conditional
  - Handle missing display gracefully
  - Verify JSONL works in headless mode

### Files Added

- `tests/unit/test_story_4_2_headless.py`
- `bmad/stories/story-4-2-headless-mode.md` (this file)

## Test Plan

### Unit Tests (no hardware)

1. Node initializes with visualization_enabled=False
2. No GUI imports when headless
3. JSONL writer works without display
4. ROS 2 publishers created in headless mode

### Integration Tests (Raspberry Pi)

1. Run node via SSH (no display)
2. Verify topics publishing
3. Verify JSONL files created
4. Verify no X11 errors in logs

### Test Command

```bash
# Test headless mode on Pi (via SSH)
ros2 run ai_camera_vision ai_camera_vision_node --ros-args \
  -p visualization_enabled:=false \
  -p jsonl_logging_enabled:=true \
  -p jsonl_output_dir:=/tmp/vision_logs

# Verify output
ls -la /tmp/vision_logs/
ros2 topic list | grep vision
ros2 topic hz /vision/detections
```

## Dependencies

- Story 1.2 (node skeleton with visualization parameter)
- Story 1.3 (JSONL writer)
- Story 2.2 (pipeline)

## Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `visualization_enabled` | bool | **false** | GUI mode (default off for headless) |
| `jsonl_logging_enabled` | bool | false | Enable file logging |
| `jsonl_output_dir` | string | `.` | JSONL output directory |

## Notes

- Default `visualization_enabled=false` ensures safe headless deployment
- OpenCV can be imported for image processing even in headless mode
- Only `cv2.imshow()` and related functions require a display
- Consider adding `DISPLAY` environment variable check
- Raspberry Pi OS Lite has no display server by default
