---
title: Story 3.4 — Runtime stats (FPS/latency) surfaced
description: Log and display FPS and latency measurements for performance monitoring
date: 2026-01-14
epic: epic-03
status: DONE
---

# Story 3.4 — Runtime stats (FPS/latency) surfaced

## Objective

Surface runtime performance statistics (FPS, latency) via logging and optional UI overlay for debugging and performance validation.

## Acceptance Criteria

- [ ] Logs include FPS and basic latency measurements
- [ ] Stats are available in headless mode (not only UI overlay)
- [ ] Stats logged at configurable interval (not every frame)
- [ ] Per-stream statistics (detection, segmentation, depth, preview)

## Technical Design

### Metrics to Track

| Metric | Description | Unit |
|--------|-------------|------|
| Preview FPS | RGB preview frame rate | frames/sec |
| Detection FPS | YOLO inference throughput | frames/sec |
| Segmentation FPS | DeepLabV3+ inference throughput | frames/sec |
| Depth FPS | Stereo depth frame rate | frames/sec |
| Detection latency | Time from capture to detection output | ms |
| Segmentation latency | Time from capture to segmentation output | ms |
| Drop rate | Percentage of frames dropped per stream | % |

### Stats Collection

```python
@dataclass
class StreamStats:
    """Statistics for a single stream."""
    name: str
    frame_count: int = 0
    start_time: float = 0.0
    last_timestamp_ns: int = 0
    total_latency_ms: float = 0.0
    
    @property
    def fps(self) -> float:
        elapsed = time.time() - self.start_time
        return self.frame_count / elapsed if elapsed > 0 else 0.0
    
    @property
    def avg_latency_ms(self) -> float:
        return self.total_latency_ms / self.frame_count if self.frame_count > 0 else 0.0

class RuntimeStats:
    """Aggregate runtime statistics."""
    
    def __init__(self):
        self.streams = {
            'preview': StreamStats('preview'),
            'detection': StreamStats('detection'),
            'segmentation': StreamStats('segmentation'),
            'depth': StreamStats('depth'),
        }
        self.log_interval_sec = 10.0
        self.last_log_time = time.time()
    
    def record_frame(self, stream: str, timestamp_ns: int):
        """Record a frame received for a stream."""
        stats = self.streams[stream]
        if stats.frame_count == 0:
            stats.start_time = time.time()
        stats.frame_count += 1
        stats.last_timestamp_ns = timestamp_ns
    
    def maybe_log(self) -> bool:
        """Log stats if interval elapsed. Returns True if logged."""
        now = time.time()
        if now - self.last_log_time >= self.log_interval_sec:
            self.log_summary()
            self.last_log_time = now
            return True
        return False
    
    def log_summary(self):
        """Log current statistics."""
        logger.info("Runtime Stats:")
        for name, stats in self.streams.items():
            if stats.frame_count > 0:
                logger.info(f"  {name}: {stats.fps:.1f} FPS")
```

### Logging Format

```
[INFO] Runtime Stats:
[INFO]   preview: 18.9 FPS
[INFO]   detection: 10.1 FPS (avg latency: 45ms)
[INFO]   segmentation: 10.2 FPS (avg latency: 38ms)
[INFO]   depth: 18.9 FPS
[INFO]   drops: detection=62.6%, segmentation=30.2%
```

### UI Overlay (when visualization enabled)

Display in corner of preview window:

```
┌─────────────────────────┐
│ Preview:      18.9 FPS  │
│ Detection:    10.1 FPS  │
│ Segmentation: 10.2 FPS  │
│ Depth:        18.9 FPS  │
└─────────────────────────┘
```

This is already implemented in `demo_live_view.py`.

## Implementation

### Files Modified

- `src/ai_camera_vision/ai_camera_vision/node.py`
  - Integrate RuntimeStats
  - Log periodically
- `src/ai_camera_vision/ai_camera_vision/visualization.py`
  - Add stats overlay drawing

### Files Added

- `src/ai_camera_vision/ai_camera_vision/stats.py`
  - `StreamStats` dataclass
  - `RuntimeStats` class
- `tests/unit/test_story_3_4_runtime_stats.py`
- `bmad/stories/story-3-4-runtime-stats.md` (this file)

## Test Plan

### Unit Tests (no hardware)

1. FPS calculation accuracy
2. Latency calculation
3. Stats reset functionality
4. Log interval timing
5. Stats formatting

### Manual Tests (OAK-D Lite)

1. Stats logged at correct interval in headless mode
2. UI overlay shows correct values
3. Stats match actual performance

## Dependencies

- Story 2.3 (timestamps for latency calculation)
- Story 2.4 (drop statistics from BackpressureMonitor)

## Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `stats_log_interval_sec` | float | 10.0 | Seconds between log outputs |
| `stats_enabled` | bool | true | Enable stats collection |

## Notes

- Existing `demo_live_view.py` has much of this implemented
- Can integrate with BackpressureMonitor for drop statistics
- Consider publishing stats to ROS2 topic for monitoring (see Story 4.4)
- Latency measurement requires comparing device timestamp to host time
