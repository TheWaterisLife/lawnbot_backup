---
title: Epic 03 — Host Visualization and Validation Tools
description: Render overlays for boxes and segmentation, and provide runtime stats for debugging
date: 2026-01-12
---

# Epic 03 — Host Visualization and Validation Tools

## Objective

Provide a developer-friendly visualization mode that makes it easy to validate detection and segmentation correctness and diagnose performance issues.

## Scope

- RGB preview display
- Bounding box overlay + labels
- Segmentation overlay with legend/class mapping
- Runtime stats display/logging

## Stories

### Story 3.1 — Render RGB preview with clean exit

- **Priority**: P0
- **Acceptance criteria**
  - [ ] Visualization mode displays an RGB preview window
  - [ ] The app can exit gracefully via keyboard command (documented)
  - [ ] Headless mode remains supported
- **Dependencies**
  - Story 1.2
  - Story 2.2

### Story 3.2 — Overlay YOLO detections (boxes + labels)

- **Priority**: P0
- **Acceptance criteria**
  - [ ] Boxes and labels render in correct positions relative to the preview
  - [ ] Confidence threshold can be configured
  - [ ] Overlay uses the coordinate convention from ADR-003
- **Dependencies**
  - Story 3.1
  - Story 1.1

### Story 3.3 — Overlay segmentation mask

- **Priority**: P0
- **Acceptance criteria**
  - [ ] Mask is rendered as a transparent overlay on preview
  - [ ] Class mapping is documented and visible (legend or label map file)
  - [ ] Mask alignment (scale/letterbox assumptions) is documented
- **Dependencies**
  - Story 3.1
  - Story 1.1

### Story 3.4 — Runtime stats (FPS/latency) surfaced

- **Priority**: P1
- **Acceptance criteria**
  - [ ] Logs include FPS and basic latency measurements
  - [ ] Stats are available in headless mode (not only UI overlay)
- **Dependencies**
  - Story 2.3


