---
title: Story 3.1 — Render RGB preview with clean exit
description: Display live RGB preview window with graceful exit mechanism
date: 2026-01-14
epic: epic-03
status: DONE
---

# Story 3.1 — Render RGB preview with clean exit

## Objective

Provide a visualization mode that displays the live RGB camera feed from the OAK-D Lite, with a clean way to exit the application.

## Acceptance Criteria

- [ ] Visualization mode displays an RGB preview window
- [ ] The app can exit gracefully via keyboard command (documented)
- [ ] Headless mode remains supported (no window when visualization disabled)
- [ ] Window title identifies the application
- [ ] Preview updates at camera frame rate

## Technical Design

### Existing Implementation

`demo_live_view.py` already provides this functionality:
- Uses OpenCV `cv2.imshow()` for display
- Exits on 'q' key press
- Shows FPS overlay

### Integration with ROS2 Node

The ROS2 node needs a `visualization_enabled` parameter (already defined in Story 1.2):

```python
# In node.py
self.declare_parameter('visualization_enabled', False)
self.visualization_enabled = self.get_parameter('visualization_enabled').value

if self.visualization_enabled:
    # Create preview window
    cv2.namedWindow("AI Camera Vision", cv2.WINDOW_AUTOSIZE)
```

### Exit Mechanism

| Key | Action |
|-----|--------|
| `q` | Quit application gracefully |
| `ESC` | Alternative quit |

### Window Management

```python
def show_preview(self, frame: np.ndarray) -> bool:
    """Display preview and check for exit.
    
    Returns:
        False if user requested exit, True otherwise.
    """
    cv2.imshow("AI Camera Vision", frame)
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('q') or key == 27:  # 'q' or ESC
        return False
    return True
```

### Headless Mode

When `visualization_enabled=False`:
- No OpenCV window created
- No `cv2.imshow()` calls
- No `cv2.waitKey()` calls (which would block)
- Application runs without any GUI dependencies

## Implementation

### Files Modified

- `src/ai_camera_vision/ai_camera_vision/node.py`
  - Add preview display in main loop
  - Handle exit key detection
  - Respect `visualization_enabled` parameter

### Files Added

- `src/ai_camera_vision/ai_camera_vision/visualization.py`
  - `PreviewWindow` class
  - Clean resource management
- `tests/unit/test_story_3_1_preview.py`
- `bmad/stories/story-3-1-rgb-preview.md` (this file)

## Test Plan

### Unit Tests (no hardware)

1. PreviewWindow class can be instantiated
2. Headless mode doesn't create window
3. Exit key detection logic
4. Window cleanup on shutdown

### Manual Tests (OAK-D Lite)

1. Window appears when visualization_enabled=True
2. Preview shows live camera feed
3. 'q' key exits cleanly
4. No window when visualization_enabled=False

## Dependencies

- Story 1.2 (node skeleton with visualization_enabled parameter)
- Story 2.2 (pipeline provides RGB preview stream)

## Notes

- OpenCV must be installed (`opencv-python` in requirements.txt)
- On headless systems (no display), visualization must be disabled
- Frame rate depends on USB mode (USB3: ~19 FPS, USB2: ~3-5 FPS)
- `demo_live_view.py` can serve as reference implementation

## Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `visualization_enabled` | bool | false | Show preview window |
