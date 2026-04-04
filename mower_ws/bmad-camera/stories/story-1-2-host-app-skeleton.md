---
title: Story 1.2 — Minimal runnable host ROS2 node skeleton (rclpy)
description: Create a ROS2 Jazzy Python package that starts cleanly and is ready to connect to OAK streams
date: 2026-01-12
---

# Story 1.2 — Minimal runnable host ROS2 node skeleton (rclpy)

## Objective

Create a minimal, runnable ROS2 Jazzy **Python** package that provides the host-side “sensor node” skeleton:

- starts and shuts down cleanly
- supports headless vs visualization configuration
- is ready to publish the topics defined in `bmad/schema-v1.md`

## Scope

- ROS2 Python package scaffolding (`rclpy`)
- Node entrypoint and basic parameters
- Logging conventions
- Placeholders for publishers (no need for full OAK integration in this story)

## Out of scope

- Full DepthAI pipeline creation (Epic 02)
- Real inference parsing and overlays (Epic 02/03)

## Acceptance criteria

- [ ] A ROS2 Jazzy Python package exists in the repo and builds/runs on Ubuntu 24.04
- [ ] Running the node starts successfully and logs startup configuration
- [ ] The node exits cleanly on Ctrl+C (or ROS shutdown) without hanging
- [ ] Node supports parameters (at minimum):
  - `visualization_enabled` (bool)
  - `yolo_blob_path` (string)
  - `seg_blob_path` (string)
  - `detections_topic` (string, default `/vision/detections`)
  - `segmentation_topic` (string, default `/vision/segmentation/mask`)
- [ ] Node declares publishers using standard message types:
  - detections: `vision_msgs/msg/Detection2DArray`
  - segmentation mask: `sensor_msgs/msg/Image`
- [ ] A short README documents how to run the node and configure parameters

## Technical notes

- Follow `bmad/schema-v1.md` for topic names and message type decisions.
- This story should create the structure to later plug in:
  - DepthAI stream receiver
  - message conversion (bbox normalization → pixels, mask encoding)
  - optional visualization pipeline

## Suggested repo structure (implementation guidance)

- `src/ai_camera_vision/` (ament_python package)
  - `package.xml`, `setup.py`, `setup.cfg`
  - `ai_camera_vision/node.py` (ROS2 node class)
  - `README.md` (package-specific)

## Definition of Done

- [ ] Acceptance criteria met
- [ ] Code is readable and organized for future stories
- [ ] No hard-coded absolute paths (use ROS params)


