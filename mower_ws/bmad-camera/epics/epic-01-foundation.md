---
title: Epic 01 — Foundation and Repo Setup
description: Establish runnable skeleton, configuration, and basic output contracts
date: 2026-01-12
---

# Epic 01 — Foundation and Repo Setup

## Objective

Create a maintainable project skeleton with a clear run entrypoint, configuration, logging, and documented output schema v1.

## Scope

- Repo structure for Python app code
- Configuration and logging conventions
- Output schema docs (v1)

## Stories

### Story 1.1 — Define output schema v1 (detections + segmentation)

- **Priority**: P0
- **Acceptance criteria**
  - [ ] A markdown doc defines schema v1 for detections and segmentation messages
  - [ ] Schema includes `schema_version`, timestamps, and model metadata
  - [ ] Bounding box coordinate convention matches ADR-003
  - [ ] Doc includes ROS2 topic + message type mapping (treat subsystem as a sensor)
  - [ ] Example messages are included

### Story 1.2 — Add minimal runnable host app skeleton

- **Priority**: P0
- **Acceptance criteria**
  - [ ] A ROS2 Jazzy Python node (`rclpy`) exists that starts and exits cleanly
  - [ ] Configuration supports toggling visualization vs headless (ROS params)
  - [ ] Logs are emitted with a consistent format
- **Dependencies**
  - Story 1.1

### Story 1.3 — JSONL publisher for schema v1

- **Priority**: P0
- **Acceptance criteria**
  - [ ] Host app can write detection messages to `detections.jsonl`
  - [ ] Host app can write segmentation messages to `segmentation.jsonl`
  - [ ] Files are append-only and flush periodically or on each line (document behavior)
- **Dependencies**
  - Story 1.1
  - Story 1.2


