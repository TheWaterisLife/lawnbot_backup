---
title: Story 3.3 — Overlay segmentation mask
description: Render segmentation mask as transparent overlay on RGB preview
date: 2026-01-14
epic: epic-03
status: DONE
---

# Story 3.3 — Overlay segmentation mask

## Objective

Display the segmentation mask as a semi-transparent colored overlay on the RGB preview to visualize terrain classification.

## Acceptance Criteria

- [ ] Mask is rendered as a transparent overlay on preview
- [ ] Class mapping is documented and visible (legend or label map file)
- [ ] Mask alignment (scale/letterbox assumptions) is documented
- [ ] Zone colors match schema-v1.md semantics (green/yellow/red)

## Technical Design

### Color Mapping (Zone Semantics)

Per schema-v1.md zone definitions:

| Class ID | Zone | Color (BGR) | Meaning |
|----------|------|-------------|---------|
| 0 | unknown | Transparent | Not classified |
| 1 | green | (0, 255, 0) | Safe to cut/drive |
| 2 | yellow | (0, 255, 255) | Drive only, no cut |
| 3 | red | (0, 0, 255) | No-go zone |

### Overlay Blending

```python
def overlay_segmentation(
    frame: np.ndarray,
    mask: np.ndarray,
    alpha: float = 0.4,
) -> np.ndarray:
    """Overlay segmentation mask on RGB frame.
    
    Args:
        frame: BGR image (H, W, 3)
        mask: Class ID mask (H_mask, W_mask) - will be resized
        alpha: Transparency (0=invisible, 1=opaque)
    
    Returns:
        Frame with mask overlay
    """
    h, w = frame.shape[:2]
    
    # Resize mask to frame size
    mask_resized = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
    
    # Create color overlay
    overlay = np.zeros_like(frame)
    overlay[mask_resized == 1] = (0, 255, 0)    # Green
    overlay[mask_resized == 2] = (0, 255, 255)  # Yellow
    overlay[mask_resized == 3] = (0, 0, 255)    # Red
    
    # Create alpha mask (only color where class > 0)
    alpha_mask = (mask_resized > 0).astype(np.float32) * alpha
    alpha_mask = np.stack([alpha_mask] * 3, axis=-1)
    
    # Blend
    result = frame.astype(np.float32) * (1 - alpha_mask) + overlay.astype(np.float32) * alpha_mask
    return result.astype(np.uint8)
```

### Mask Alignment

The segmentation model input is 256×256, while the RGB preview is 640×352.

**Alignment strategy**:
1. Resize mask from 256×256 to preview size using `INTER_NEAREST` (preserves class IDs)
2. No letterboxing compensation needed if both use same crop/resize strategy

### Legend Display

Optional: Show color legend in corner of preview:

```
┌─────────────────┐
│ ■ Green: Safe   │
│ ■ Yellow: Drive │
│ ■ Red: No-go    │
└─────────────────┘
```

## Implementation

### Files Modified

- `src/ai_camera_vision/ai_camera_vision/visualization.py`
  - Add `overlay_segmentation()` function
  - Add `draw_legend()` function
  - Add zone color constants

### Files Added

- `tests/unit/test_story_3_3_segmentation_overlay.py`
- `bmad/stories/story-3-3-segmentation-overlay.md` (this file)

## Test Plan

### Unit Tests (no hardware)

1. Color mapping for all zone classes
2. Mask resize to different frame sizes
3. Alpha blending calculation
4. Unknown pixels (class 0) remain transparent
5. Edge case: all same class

### Manual Tests (OAK-D Lite)

1. Overlay aligns with visible scene
2. Colors are distinguishable
3. Transparency allows seeing underlying image
4. Legend is readable (if implemented)

## Dependencies

- Story 3.1 (RGB preview window)
- Story 1.1 (schema defines zone semantics)
- Story 2.2 (pipeline provides segmentation mask)

## Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `show_segmentation_overlay` | bool | true | Enable/disable mask overlay |
| `segmentation_alpha` | float | 0.4 | Overlay transparency |
| `show_legend` | bool | true | Show color legend |

## Notes

- DeepLabV3+ model is trained for person segmentation, not terrain
- Actual zone classification may require different model or post-processing
- For MVP, overlay shows raw model output (person vs background)
- `INTER_NEAREST` interpolation preserves class ID values (don't use `INTER_LINEAR`)

## Model Output Mapping

Current DeepLabV3+ outputs:
- Class 0: Background
- Class 15: Person (PASCAL VOC class)

Mapping to zones for visualization:
- Background → Unknown (transparent)
- Person → Red (obstacle/no-go)

This can be refined with a terrain-specific model later.

