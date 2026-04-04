---
title: Story 4.4 — Publish ROS2 status/diagnostics for health monitoring
description: Publish periodic health status for monitoring by the mower brain
date: 2026-01-14
epic: epic-04
status: DONE
---

# Story 4.4 — Publish ROS2 status/diagnostics for health monitoring

## Objective

Publish a periodic status message so the mower brain can monitor vision system health and detect faults.

## Acceptance Criteria

- [ ] Node publishes a periodic status signal (e.g., `/vision/status` or `/diagnostics`)
- [ ] Status includes at least: FPS (or publish rate), last error state, and whether depth/NN streams are active
- [ ] Documentation explains how the "brain" or operators can use the status to detect faults

## Technical Design

### Status Topic

Publish to `/vision/status` using `std_msgs/msg/String` with JSON payload, or use standard `diagnostic_msgs/msg/DiagnosticArray`.

**Option 1: Simple JSON status** (recommended for MVP)

Topic: `/vision/status`
Type: `std_msgs/msg/String`

```json
{
  "timestamp_ns": 1736700000000000000,
  "status": "OK",
  "detection_fps": 10.1,
  "segmentation_fps": 10.2,
  "depth_fps": 18.9,
  "preview_fps": 18.9,
  "detection_active": true,
  "segmentation_active": true,
  "depth_active": true,
  "drop_rate_detection": 62.6,
  "drop_rate_segmentation": 30.2,
  "last_error": null,
  "uptime_sec": 3600.5
}
```

**Option 2: ROS 2 Diagnostics** (standard approach)

Topic: `/diagnostics`
Type: `diagnostic_msgs/msg/DiagnosticArray`

### Status Levels

| Status | Meaning |
|--------|---------|
| `OK` | All streams active, acceptable FPS |
| `WARN` | High drop rate (>50%) or low FPS |
| `ERROR` | Stream inactive or critical failure |
| `STALE` | No recent data from stream |

### Health Check Logic

```python
def compute_status(self) -> str:
    """Determine overall health status."""
    if self.last_error is not None:
        return "ERROR"
    
    if not self.detection_active or not self.segmentation_active:
        return "ERROR"
    
    if self.detection_fps < 5.0 or self.segmentation_fps < 5.0:
        return "WARN"
    
    if self.drop_rate_detection > 50.0 or self.drop_rate_segmentation > 50.0:
        return "WARN"
    
    return "OK"
```

### Publishing Rate

Publish status every **1 second** (1 Hz) to minimize bandwidth while providing timely health updates.

### Fault Detection for Brain

The mower brain should monitor:

1. **No status message** for >5 seconds → Vision node dead
2. **status = "ERROR"** → Critical fault, stop mower
3. **status = "WARN"** → Degraded mode, proceed with caution
4. **detection_active = false** → No obstacle detection, stop mower
5. **depth_active = false** → No distance info, reduce speed

### Example Brain Logic

```python
def vision_status_callback(self, msg: String):
    status = json.loads(msg.data)
    
    if status['status'] == 'ERROR':
        self.emergency_stop("Vision system error")
    elif not status['detection_active']:
        self.emergency_stop("Detection stream inactive")
    elif status['status'] == 'WARN':
        self.reduce_speed(0.5)  # Half speed in degraded mode
```

## Implementation

### Files Modified

- `src/ai_camera_vision/ai_camera_vision/node.py`
  - Add status publisher
  - Compute and publish status periodically
  - Integrate with RuntimeStats

### Files Added

- `src/ai_camera_vision/ai_camera_vision/status.py`
  - `VisionStatus` dataclass
  - Status computation logic
- `tests/unit/test_story_4_4_status.py`
- `bmad/stories/story-4-4-ros2-status.md` (this file)

## Test Plan

### Unit Tests (no hardware)

1. Status JSON serialization
2. Health level computation
3. Stream active detection
4. FPS threshold logic

### Integration Tests (Raspberry Pi)

1. Status publishes at 1 Hz
2. Status reflects actual stream states
3. Brain can subscribe and parse status

### Test Commands

```bash
# Monitor status
ros2 topic echo /vision/status

# Check publish rate
ros2 topic hz /vision/status
```

## Dependencies

- Story 2.2 (pipeline streams to monitor)
- Story 3.4 (RuntimeStats for FPS data)
- Story 2.4 (BackpressureMonitor for drop rates)

## Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `status_topic` | string | `/vision/status` | Status topic name |
| `status_rate_hz` | float | 1.0 | Status publish rate |
| `status_enabled` | bool | true | Enable status publishing |

## Notes

- Status is critical for autonomous operation safety
- Brain should have watchdog timeout for status messages
- Consider using `diagnostic_msgs` for integration with ROS 2 tooling
- Status message should be small (<1KB) for low bandwidth
- Include software version in status for debugging
