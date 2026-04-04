---
title: Story 2.4 — Backpressure strategy and bounded queues
description: Ensure all queues are bounded and document the drop policy with logging
date: 2026-01-12
epic: epic-02
status: IN_PROGRESS
---

# Story 2.4 — Backpressure strategy and bounded queues

## Objective

Ensure the pipeline handles backpressure gracefully by using bounded queues, implementing an explicit drop policy, and logging when frame drops occur.

## Acceptance Criteria

- [x] All queues are bounded or otherwise protected against unbounded growth
- [x] Drop policy is explicit and documented (what drops first and why)
- [x] Logs indicate when drops occur

## Technical Design

### Queue Architecture

The pipeline has queues at three levels:

```
Camera → ImageManip → NN Input Queue → Neural Network → XLinkOut Queue → Host Queue
                      (on device)                       (on device)      (USB→host)
```

### Current Queue Settings

| Queue Location | Size | Blocking | Behavior |
|----------------|------|----------|----------|
| NN input (YOLO) | 1 | False | Drop oldest if full |
| NN input (Seg) | 1 | False | Drop oldest if full |
| XLinkOut (YOLO) | 1 | False | Drop oldest if full |
| XLinkOut (Seg) | 1 | False | Drop oldest if full |
| XLinkOut (RGB) | 1 | False | Drop oldest if full |
| Host queue (all) | 4 | False | Drop oldest if full |

### Drop Policy (Explicit)

**Policy: Drop Oldest, Keep Latest**

When a queue is full and a new frame arrives:
1. The **oldest** frame in the queue is dropped
2. The **newest** frame is enqueued
3. Consumer always gets the most recent available data

**Rationale:**
- Real-time systems need fresh data, not stale queued data
- A mower brain making decisions needs current obstacle positions
- Latency is more important than completeness for safety

### Drop Detection Strategy

Drops are detected by monitoring **sequence number gaps**:

```python
# If we received seq 100 last time and now receive seq 103,
# we know frames 101 and 102 were dropped (2 frames lost)
dropped = current_seq - last_seq - 1
```

### Logging Behavior

When drops are detected:
- Log at WARNING level (not ERROR — drops are expected under load)
- Include: stream name, frames dropped count, current sequence
- Log periodically (not every frame) to avoid log spam

Example log output:
```
[WARN] Backpressure: 3 detection frames dropped (seq 100→104)
[WARN] Backpressure: 2 segmentation frames dropped (seq 150→153)
```

### Drop Statistics

`QueueStats` tracks per-stream:
- `total_received`: Total frames received
- `total_dropped`: Total frames dropped (detected via sequence gaps)
- `last_sequence`: Last sequence number seen
- `drop_rate`: Percentage of frames dropped

## Implementation

### Files Modified

- `src/ai_camera_vision/ai_camera_vision/pipeline.py`
  - Add `QueueStats` dataclass
  - Add `BackpressureMonitor` class
  - Update `PipelineRunner` with drop tracking and logging

### Files Added

- `bmad/stories/story-2-4-backpressure.md` (this file)
- `tests/unit/test_story_2_4_backpressure.py`

## Test Plan

### Unit Tests (no hardware)

1. `QueueStats` dataclass construction and updates
2. Drop detection from sequence gaps
3. Drop rate calculation
4. Log message formatting

### Hardware Tests (OAK-D Lite)

1. Verify drops are detected under artificial slow consumer
2. Verify stats accumulate correctly over time
3. Verify no false positives (no drops reported when consumer keeps up)

## Dependencies

- Story 2.2 (pipeline must be running)
- Story 2.3 (need sequence numbers for drop detection)

## Notes

- Queue sizes of 1 on device minimize latency but increase drop probability
- Host queue size of 4 provides small buffer for consumer jitter
- Drop rate under normal operation should be < 5%
- High drop rate (> 20%) may indicate consumer is too slow

