---
title: BMAD Agent Context — Capstone Robot (Camera Subsystem)
description: Canonical project context and answers for BMAD agents (PM/Architect/SM/DEV/TEA)
date: 2026-01-12
---

# BMAD Agent Context — Capstone Robot (Camera Subsystem)

Use this file as the **single source of truth** when running BMAD workflows. It captures decisions and answers “on behalf of the camera subsystem owner”.

## Project summary

We are building an **autonomous mower**. The camera subsystem is an **OAK-D Lite** vision sensor that performs:

- **Object detection** (YOLOv8n, 640×352) for obstacle detection and classification
- **Semantic segmentation** (DeepLabV3+, 256×256) for terrain/zones
- **Stereo depth** (OAK-D Lite) to estimate obstacle **distance + size** for planning

The mower “brain” runs on **Raspberry Pi 5** with **Ubuntu 24.04.x LTS** and **ROS 2 Jazzy**.

## System boundary (who owns what)

- **Camera node (this repo)**: publishes perception observations as ROS2 topics (acts like a sensor).
- **Brain (other repo/subsystem on Pi)**:
  - maintains the user-defined boundary/map (geofence)
  - fuses perception outputs with other sensors (RTK GPS, etc.)
  - decides motion + blades + safety behavior
  - may update the map based on perception (e.g., mark plant zones as no-go)

## Segmentation meaning (zones)

Segmentation is used to classify operational zones:

- **Green**: safe-to-cut / safe-to-drive
- **Yellow**: safe-to-drive but **do not cut**
- **Red**: interdiction (do not cut, do not drive)

Zone IDs are defined in `bmad/schema-v1.md` (class_map_version v1).

### Map override

User-defined map/geofence overrides vision decisions. The camera does not need access to the map to publish its outputs.

## ROS2 interface (primary)

Canonical interfaces are defined in `bmad/schema-v1.md`.

### Topics (defaults)

- `/vision/detections` — `vision_msgs/msg/Detection2DArray`
- `/vision/detections_3d` — `vision_msgs/msg/Detection3DArray` (distance + size)
- `/vision/segmentation/mask` — `sensor_msgs/msg/Image` (`mono8` class IDs)
- `/vision/segmentation/zone_summary` — `std_msgs/msg/Float32MultiArray` order: `[green_ratio, yellow_ratio, red_ratio]`

### Frame/TF strategy

- Publish messages in the **camera frame** (default `oak_rgb_optical_frame`) via `header.frame_id`.
- The robot stack provides TF (URDF/robot_state_publisher or static transform). The camera node does **not** own TF.

## Safety-critical object policy (default)

YOLO classes are mapped to simple planning categories (documented in `bmad/schema-v1.md`).

Default “avoid” policy:

- Must-avoid: `person`
- Avoid: `dog`, `cat`, `bicycle`, `motorcycle`, `car`, `truck`, `bus`
- Obstacle-like: `potted plant`, `chair`, `bench`, `stop sign`, `fire hydrant`

Everything else may be published as `unknown`.

## Performance targets

- Desired: ~20 FPS (stable lower FPS acceptable if it remains usable for the mower’s speed).
- We will measure on real hardware and trade input resolution/model size as needed.

## Depth requirement and range

- Stereo depth is **required for V1** to provide real distance estimates.
- Required obstacle detection range: **1.5 m** (from `specs.md`).

## Deployment defaults (Pi)

- Autostart on boot: **yes** (systemd service option).
- Logging default: **ROS logs only** (file logging optional debug mode).
- “Telemetry”: local ROS2 status/diagnostics topics only (no cloud).

## References

- Overall camera overview: `PROJECT_OVERVIEW.md`
- Mower requirements/specs: `specs.md`
- PRD: `bmad/PRD.md`
- Architecture + ADRs: `bmad/architecture.md`
- Interfaces: `bmad/schema-v1.md`
- Backlog: `bmad/epics/` and `sprint-status.yaml`



