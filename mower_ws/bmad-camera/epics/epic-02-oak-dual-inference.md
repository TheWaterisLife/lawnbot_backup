---
title: Epic 02 — OAK-D Lite Dual-Model Inference Pipeline
description: Build the DepthAI pipeline to run YOLO and segmentation concurrently and stream results
date: 2026-01-12
---

# Epic 02 — OAK-D Lite Dual-Model Inference Pipeline

## Objective

Run YOLOv8n and DeepLabV3+ concurrently on OAK-D Lite and deliver both result streams to the host app reliably with clear timing metadata.

## Scope

- DepthAI pipeline creation
- Stream definitions and metadata
- Minimal robustness (startup validation, failure behavior)

## Stories

### Story 2.1 — Device + blob validation on startup

- **Priority**: P0
- **Acceptance criteria**
  - [ ] Startup checks for OAK device presence
  - [ ] Startup checks blob file paths exist and are readable
  - [ ] On failure, prints actionable error and exits non-zero

### Story 2.2 — Build and start dual-NN DepthAI pipeline

- **Priority**: P0
- **Acceptance criteria**
  - [ ] Pipeline creates an RGB stream and two NN nodes (YOLO + segmentation)
  - [ ] Both NN nodes run during the same session
  - [ ] Host receives both output queues
- **Dependencies**
  - Story 2.1
  - Story 1.2

### Story 2.3 — Add timing metadata and sequence numbers

- **Priority**: P0
- **Acceptance criteria**
  - [ ] Detection and segmentation outputs include timestamps and sequence numbers
  - [ ] Documentation explains how to associate results across streams
- **Dependencies**
  - Story 2.2
  - Story 1.1

### Story 2.4 — Backpressure strategy and bounded queues

- **Priority**: P1
- **Acceptance criteria**
  - [ ] All queues are bounded or otherwise protected against unbounded growth
  - [ ] Drop policy is explicit and documented (what drops first and why)
  - [ ] Logs indicate when drops occur
- **Dependencies**
  - Story 2.2

### Story 2.5 — Depth-based distance output (3D detections for the brain)

- **Priority**: P0
- **Acceptance criteria**
  - [ ] Pipeline produces depth aligned to the chosen RGB reference frame
  - [ ] For each obstacle-like detection, publish a `vision_msgs/msg/Detection3DArray` message on `/vision/detections_3d`
  - [ ] Each published 3D detection includes a distance estimate in meters in the ROS frame (target usable range: 1.5 m per `specs.md`)
  - [ ] Each published 3D detection includes size estimates when feasible (x/y/z dimensions)
  - [ ] Document assumptions and limitations (occlusion, invalid depth, edge cases)
- **Dependencies**
  - Story 2.2

### Story 2.6 — Publish segmentation zone summary (green/yellow/red ratios)

- **Priority**: P1
- **Acceptance criteria**
  - [ ] For each segmentation mask update, publish `/vision/segmentation/zone_summary` as `std_msgs/msg/Float32MultiArray`
  - [ ] The array uses the field order documented in `bmad/schema-v1.md` (green, yellow, red ratios)
  - [ ] Ratios are computed from a documented ROI policy (default can be full mask)
  - [ ] Publishing can be disabled via configuration if needed
- **Dependencies**
  - Story 2.2

### Story 2.7 — YOLO output decoder (raw tensors → Detection2DArray)

- **Priority**: P0
- **Acceptance criteria**
  - [ ] Create `detections.py` module in src/
  - [ ] Decode YOLO tensor output to structured detections
  - [ ] Apply NMS to filter overlapping boxes
  - [ ] Map COCO classes to safety categories
  - [ ] Unit tests verify decoder correctness
- **Dependencies**
  - Story 2.2
  - Story 1.1 (schema defines detection format)

### Story 2.8 — Connect pipeline to ROS2 publishers

- **Priority**: P0
- **Acceptance criteria**
  - [ ] ROS2 node starts DepthAI pipeline on initialization
  - [ ] Detection results published to `/vision/detections`
  - [ ] 3D detections published to `/vision/detections_3d`
  - [ ] Segmentation published to `/vision/segmentation/mask`
  - [ ] Topics verifiable with `ros2 topic echo`
- **Dependencies**
  - Story 2.7 (decoder)
  - Story 2.5 (depth utilities)
  - Story 1.2 (node skeleton)


