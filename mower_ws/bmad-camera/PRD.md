---
title: PRD — AI Camera Vision System
description: Product requirements for the OAK-D Lite dual-model vision subsystem
date: 2026-01-12
---

# PRD — AI Camera Vision System

## Executive summary

Deliver a deployable vision subsystem using OAK-D Lite that provides real-time **YOLO detections** and **semantic segmentation** computed on-device, and streams structured outputs to a host for visualization and robotics integration.

## Personas

- **P1: Robotics integrator**
  - Needs reliable, parseable outputs and stable runtime behavior.
  - Cares about coordinate consistency and latency/throughput.
- **P2: Developer/operator**
  - Needs tooling for running locally, visual validation, and debugging.
  - Cares about logs, repeatability, and easy deployment steps.

## In scope

- DepthAI pipeline configuration for dual-model inference
- Host-side receiver that:
  - visualizes outputs for validation
  - emits outputs in a documented schema
- Basic run controls and observability (logs, stats)
- Raspberry Pi deployment path (how to run on boot is acceptable if scripted)

## Out of scope (for this PRD)

- Model training/fine-tuning
- Custom datasets / labeling pipelines
- Advanced tracking (multi-object tracking) unless required for stability
- Full robot autonomy behaviors

## Functional requirements (FR)

### FR-001 — Dual-model pipeline runs concurrently

- The system runs **YOLO detection** and **semantic segmentation** concurrently on the OAK device.
- The host receives both result streams during the same runtime session.

### FR-002 — Detection results are exposed with stable schema

- For each frame (or timestamped inference result), emit:
  - list of detections: class label (or ID), confidence, bounding box coordinates
  - metadata: timestamp, inference sequence number, model input size, source stream identifier

### FR-003 — Segmentation results are exposed with stable schema

- For each frame (or timestamped inference result), emit:
  - segmentation mask (class IDs) with documented shape and coordinate mapping
  - metadata: timestamp, inference sequence number, model input size, class map/version
  - the class map MUST support mower zones (green/yellow/red) as defined in `bmad/schema-v1.md`
  - an optional low-bandwidth zone summary (green/yellow/red ratios) for fast planning, per `bmad/schema-v1.md`

### FR-004 — Result association and timing metadata

- Outputs include enough metadata to associate detection and segmentation outputs that correspond to the same scene time.
- If perfect sync is not possible, outputs must clearly indicate their independent timestamps and sequence numbers.

### FR-005 — Visualization mode for validation

- Provide a mode that renders:
  - RGB preview
  - bounding boxes and labels
  - segmentation overlay (with legend or class mapping)
- Visualization can be disabled for headless deployment.

### FR-006 — Configuration

- Support configuration of at least:
  - model blob paths
  - confidence threshold(s)
  - visualization toggle
  - output transport selection (see architecture decisions)

### FR-007 — Export / publish outputs for integration

- Provide an integration-friendly output mechanism suitable for robotics:
  - ROS2 topics (primary), and
  - optional file logging (JSONL) for debugging and replay

### FR-009 — Provide obstacle distance (and size when possible) for avoidance planning

- For each obstacle-like detection, publish a ROS2 3D detection output that provides:
  - distance from the robot/camera frame, and
  - size estimates when feasible (preferred as 3D box dimensions, not a single “volume” number)
- The mapping uses standard message types (see `bmad/schema-v1.md`).
- Obstacle distance must be usable at least out to the required range in `specs.md` (Obstacle Detection Range: 1.5 m).

### FR-008 — Health and failure behavior

- On startup, validate:
  - device connection
  - blob file presence
  - required model metadata (input size assumptions)
- On failure, exit with a clear error message and non-zero status code.

## Non-functional requirements (NFR)

### NFR-001 — Performance

- The system sustains real-time processing suitable for robotics use.
- The system exposes runtime performance stats (FPS, latency, queue depth where possible).
 - Target throughput for validation: ~20 FPS is desirable; the system must remain usable at lower rates by trading input resolution/model size if needed.

### NFR-002 — Resource constraints

- Host CPU usage remains low in headless mode (inference is on-device).
- The system avoids unbounded memory growth (queues bounded, backpressure defined).

### NFR-003 — Reliability

- The system handles intermittent device disconnects with a clear failure mode (fail-fast or explicit reconnect strategy per architecture).

### NFR-004 — Portability

- Runs on development PC and Raspberry Pi with minimal environment differences.

### NFR-005 — Observability

- Logs are structured enough to debug typical failures (device missing, blob incompatible, queue overflow).
 - Default deployment uses ROS2 logging only; optional file-based debug logging can be enabled when needed.

### NFR-006 — Maintainability

- Code organization supports adding future outputs (e.g., depth, tracking) without major rewrites.

## Constraints

- Inference must run on OAK-D Lite (Myriad X).
- Models are provided as `.blob` files compiled for the device.
- Robotics integration target: Raspberry Pi 5 running Ubuntu 24.04 (ROS 2 Jazzy).

## Success metrics

- Demonstrated end-to-end demo: live camera → dual inference → overlay visualization
- Documented and validated output schema used by a small consumer program
- Basic deployment on Raspberry Pi demonstrated in a repeatable way

## Risks and mitigations

- **Dual inference throughput risk**: measure early; add configuration for preview size / queue depths; allow disabling visualization.
- **Output alignment risk**: define coordinate conventions and include explicit metadata in each message.
- **Integration risk**: choose one “default transport” and treat others as optional extensions.


