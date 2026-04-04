---
title: Story 2.3 — Add timing metadata and sequence numbers
description: Extend pipeline outputs to include device timestamps and sequence numbers for stream correlation
date: 2026-01-12
epic: epic-02
status: IN_PROGRESS
---

# Story 2.3 — Add timing metadata and sequence numbers

## Objective

Extend the dual-NN pipeline outputs to include device timestamps and sequence numbers, enabling the host to correlate detection and segmentation results from the same camera frame.

## Acceptance Criteria

- [x] Detection and segmentation outputs include timestamps and sequence numbers
- [x] Timestamps extracted from device clock (nanoseconds since boot)
- [x] Sequence numbers are monotonically increasing per stream
- [x] Documentation explains how to associate results across streams

## Technical Design

### DepthAI NNData Timing API

The `dai.NNData` object from DepthAI provides:

```python
nn_data.getTimestamp()      # datetime.timedelta from device boot
nn_data.getSequenceNum()    # int, monotonic per stream
```

### Conversion to Schema v1 Format

Device timestamp (timedelta) → nanoseconds:

```python
timestamp_ns = int(nn_data.getTimestamp().total_seconds() * 1e9)
```

### Data Classes

New `InferenceResult` dataclass encapsulates raw NN output plus metadata:

```python
@dataclass(frozen=True)
class InferenceResult:
    """Inference result with timing metadata."""
    raw_data: dai.NNData          # Original NNData for tensor access
    timestamp_ns: int             # Device timestamp in nanoseconds
    sequence_num: int             # Monotonic sequence number
    timestamp_source: str = "device"
```

### Stream Correlation Strategy

Detection and segmentation streams run asynchronously at different rates:
- YOLO: ~14 FPS
- Segmentation: ~15 FPS

**Correlation approaches:**

1. **Sequence Number Matching** (exact match)
   - Both streams share the same camera input
   - Same `sequence_num` = same input frame
   - Use when exact frame pairing is required

2. **Timestamp Proximity** (nearest match)
   - Find the closest timestamps within a tolerance window
   - Use when exact pairing is not critical
   - Recommended tolerance: ±50ms

3. **Latest Available** (fire-and-forget)
   - Use whatever is newest on each stream
   - Simplest but may mix frames
   - Acceptable for low-latency display

**Recommendation:** Use sequence number matching for schema v1 messages and JSONL logging, as it guarantees frame correspondence.

## Implementation

### Files Modified

- `src/ai_camera_vision/ai_camera_vision/pipeline.py`
  - Add `InferenceResult` dataclass
  - Add `get_detection_result()` method
  - Add `get_segmentation_result()` method
  - Keep existing `get_detections()` / `get_segmentation()` for raw access

### Files Added

- `tests/unit/test_story_2_3_timing.py` — Unit tests for timing metadata

## Test Plan

### Unit Tests (no hardware)

1. `InferenceResult` dataclass construction
2. Timestamp conversion from timedelta to nanoseconds
3. Mock NNData with timing methods

### Hardware Tests (OAK-D Lite)

1. Verify real timestamps are positive and increasing
2. Verify sequence numbers are monotonic
3. Verify both streams produce timing metadata

## Dependencies

- Story 2.2 (dual-NN pipeline must be running)
- Story 1.1 (schema v1 defines timestamp/sequence format)

## Notes

- Device timestamps are relative to device boot, not wall clock
- For ROS2 integration, convert device time to ROS time using a synchronized offset
- The `frame_id` should be `"oak_rgb_optical_frame"` per schema v1

