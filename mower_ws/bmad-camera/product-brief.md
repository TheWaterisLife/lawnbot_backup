---
title: Product Brief — AI Camera Vision System
description: Product vision and MVP boundary for the OAK-D Lite dual-model on-device inference subsystem
date: 2026-01-12
---

# Product Brief — AI Camera Vision System

## Executive summary

Build a vision subsystem using **OAK-D Lite** that runs **object detection** (YOLOv8n) and **semantic segmentation** (DeepLabV3+ MobileNetV2) **in parallel on-device**, and streams structured results to a host (PC / Raspberry Pi) for visualization and robotics integration.

## Problem statement

Robotics and autonomy stacks often struggle to run multiple real-time vision models on small host computers (e.g., Raspberry Pi) due to limited CPU/GPU resources and power constraints. The project needs a solution that:

- Provides **rich scene understanding** (objects + per-pixel classes) with **low host CPU load**
- Runs reliably in real time using a hardware-constrained embedded platform
- Produces outputs that are easy to integrate into a larger autonomy system

## Target users

- **Robotics integrator (primary)**: wants detections/segmentation outputs in a clean schema to feed navigation or decision-making.
- **Developer/operator (secondary)**: wants to run, visualize, debug, and validate the pipeline on a development PC, then deploy to a Raspberry Pi.

## Proposed solution

- On OAK-D Lite, run two neural networks concurrently:
  - **YOLOv8n (640×352)** for bounding boxes and class labels
  - **DeepLabV3+ (256×256)** for semantic segmentation masks
- On the host:
  - Receive results (and optionally preview images)
  - Provide visualization overlays for debugging/validation
  - Publish structured outputs to downstream consumers (robotics stack)

## MVP scope (must-have)

- Dual-model DepthAI pipeline that runs both models concurrently
- Stable real-time stream of:
  - YOLO detections (bbox, label, confidence)
  - Segmentation mask (class IDs) aligned to a defined coordinate space
- Host-side visualization to validate correctness (overlay boxes + mask)
- Deterministic output schema suitable for integration (documented)
- Basic operational robustness:
  - Clean startup/shutdown
  - Reasonable logging
  - Clear error messages when device/model is missing or incompatible

## Non-goals (explicitly out of scope for MVP)

- Training or fine-tuning the models
- Multi-camera support
- Cloud services or remote dashboards
- Full autonomy behaviors (path planning / control)

## Differentiators / why this approach

- **Edge-first**: inference is on the OAK device, not the host.
- **Dual signal**: combines object-level and pixel-level understanding.
- **Integration-ready**: outputs designed to be consumed by robotics systems.

## Key assumptions

- OAK-D Lite can load and execute both `.blob` models concurrently within acceptable resource limits.
- The host can receive results over USB reliably (development PC and Raspberry Pi).
- COCO labels and the chosen segmentation classes are acceptable for the initial use cases.

## Risks and open questions

- **Performance risk**: dual model inference may reduce FPS or increase latency beyond requirements.
- **Alignment risk**: segmentation output resolution/coordinate mapping might be confusing or inconsistent if not specified precisely.
- **Integration risk**: downstream robotics stack interface (ROS2, MQTT, ZeroMQ, file logging) must be chosen and documented.
- **Operational risk**: USB bandwidth/cable quality can cause intermittent disconnects.
- **Model risk**: blob compatibility with the target DepthAI version / device firmware.

## Success metrics

- Stable runtime without crashes during an extended run on dev PC and Raspberry Pi
- Measured end-to-end latency and throughput meet the project’s real-time needs
- Validated output correctness (boxes and masks align with the visual scene)
- Downstream consumer can parse and use the output schema without custom hacks



