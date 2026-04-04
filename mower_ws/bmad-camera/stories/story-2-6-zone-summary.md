---
title: Story 2.6 — Publish segmentation zone summary
description: Compute and publish green/yellow/red ratios from segmentation mask for mower brain planning
date: 2026-01-14
epic: epic-02
status: DONE
---

# Story 2.6 — Publish segmentation zone summary

## Objective

Compute green/yellow/red zone ratios from each segmentation mask and publish them as a lightweight summary for the mower brain to make fast planning decisions.

## Acceptance Criteria

- [ ] For each segmentation mask update, publish `/vision/segmentation/zone_summary` as `std_msgs/msg/Float32MultiArray`
- [ ] The array uses the field order documented in `bmad/schema-v1.md` (green, yellow, red ratios)
- [ ] Ratios are computed from a documented ROI policy (default: full mask)
- [ ] Publishing can be disabled via configuration if needed
- [ ] Invalid/unknown pixels (class 0) are excluded from ratio calculations

## Technical Design

### Zone Class Mapping

The zone summary supports two segmentation models:

#### PASCAL VOC Model (DeepLabV3+ - Legacy)

| Model Output | Class | Zone |
|--------------|-------|------|
| 0 | background | GREEN (grass) |
| 15 | person | RED (no-go) |
| 1-14, 16-20 | other objects | YELLOW (caution) |

#### ADAS Model (semantic-segmentation-adas-0001 - Current Default)

| Class ID | Class | Zone | Rationale |
|----------|-------|------|-----------|
| 0 | road | YELLOW | Safe to drive, not grass |
| 1 | sidewalk | YELLOW | Safe to drive, not grass |
| 2-7 | building, wall, fence, pole, traffic light/sign | RED | Obstacles |
| 8 | vegetation | GREEN | Grass/plants = safe to cut |
| 9 | terrain | GREEN | Ground/dirt = safe to drive |
| 10 | sky | *(ignored)* | Not ground-level |
| 11 | person | RED | Human = STOP |
| 12 | rider | RED | Human on vehicle = STOP |
| 13-18 | car, truck, bus, train, motorcycle, bicycle | RED | Vehicles |
| 19 | ego-vehicle | *(ignored)* | The mower itself |

### Output Format

Per schema-v1.md, publish `std_msgs/msg/Float32MultiArray` with 3 floats:

```
[green_ratio, yellow_ratio, red_ratio]
```

Each ratio is in `[0.0, 1.0]` representing fraction of classified pixels.

### Ratio Calculation

```python
def compute_zone_ratios(
    mask: np.ndarray,
    roi: tuple[int, int, int, int] | None = None,
    model_type: str = MODEL_ADAS,
) -> ZoneSummary:
    """Compute zone ratios from segmentation mask.
    
    Supports MODEL_PASCAL_VOC (legacy) and MODEL_ADAS (default).
    ADAS model excludes sky and ego-vehicle from calculations.
    """
    ...
```

### ROI Policy

**Default**: Full mask (all pixels considered)

**Future enhancement**: Front-of-mower wedge/strip ROI for more relevant planning data. This can be configured via:
- `roi_policy` parameter: `"full"` (default) or `"front_wedge"`
- `roi_width_ratio` and `roi_height_ratio` for custom ROI

### Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `zone_summary_enabled` | bool | true | Enable/disable zone summary publishing |
| `zone_summary_topic` | string | `/vision/segmentation/zone_summary` | Topic name |
| `roi_policy` | string | `"full"` | ROI policy for ratio calculation |

### Pipeline Integration

The zone summary is computed from the same segmentation output used for the mask:

```
Segmentation NN → XLinkOut → Host
                              │
                              ├──► Mask Publisher (/vision/segmentation/mask)
                              │
                              └──► Zone Summary (/vision/segmentation/zone_summary)
```

## Implementation

### Files Modified

- `src/ai_camera_vision/ai_camera_vision/node.py`
  - Add zone summary publisher
  - Add configuration parameters
  - Compute and publish ratios on each segmentation frame

### Files Added

- `src/ai_camera_vision/ai_camera_vision/zone_summary.py`
  - `compute_zone_ratios()` function
  - `ZoneSummary` dataclass
  - ROI handling utilities
- `tests/unit/test_story_2_6_zone_summary.py`
- `bmad/stories/story-2-6-zone-summary.md` (this file)

## Test Plan

### Unit Tests (no hardware)

1. Ratio calculation with all green pixels
2. Ratio calculation with mixed zones
3. Ratio calculation with all unknown pixels (edge case)
4. ROI cropping logic
5. Float32MultiArray message construction

### Hardware Tests (OAK-D Lite)

1. Zone summary published at expected rate
2. Ratios change when camera views different scenes
3. Publishing can be disabled via parameter

## Dependencies

- Story 2.2 (pipeline running segmentation)
- Story 1.1 (schema-v1 defines zone semantics)

## Notes

- Zone summary is much smaller than full mask (~12 bytes vs ~65KB)
- Useful for quick "is it safe?" decisions without processing full mask
- **ADAS model** (semantic-segmentation-adas-0001) is now the default, providing 20-class outdoor segmentation
- **PASCAL VOC model** (DeepLabV3+) is still supported via `model_type=MODEL_PASCAL_VOC`
- ADAS model correctly maps vegetation→GREEN, road/sidewalk→YELLOW, obstacles/humans/vehicles→RED

## Edge Cases

| Scenario | Behavior |
|----------|----------|
| All pixels sky/ego-vehicle (ADAS) | Return (0.0, 0.0, 0.0) with total_pixels=0 |
| All pixels background (PASCAL VOC) | Return (1.0, 0.0, 0.0) |
| Empty mask | Return (0.0, 0.0, 0.0) |
| Single class only | That class = 1.0, others = 0.0 |
| Zone summary disabled | Don't create publisher, skip computation |
