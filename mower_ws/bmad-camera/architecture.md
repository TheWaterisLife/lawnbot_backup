---
title: Architecture — AI Camera Vision System
description: Decision-focused system architecture and ADRs for OAK-D Lite dual-model inference
date: 2026-01-12
---

# Architecture — AI Camera Vision System

## Architecture overview

This system is split into two major domains:

- **On-device (OAK-D Lite / Myriad X)**: camera capture and dual-model inference (YOLO + segmentation).
- **Host (PC / Raspberry Pi)**: receives results, visualizes for validation, and publishes structured outputs for integration.

### Principles

- **Inference on-device**: host should not do heavy ML inference.
- **Explicit coordinate conventions**: every output message must define its coordinate space.
- **Backpressure over memory growth**: bounded queues; drop policy is explicit.
- **Integration-first outputs**: results are structured and versioned.

## System context and data flow

```mermaid
flowchart TD
    Cam[OAK-D Lite RGB Sensor] --> Pipe[DepthAI Pipeline]
    Pipe --> Yolo[YOLOv8n .blob]
    Pipe --> Seg[DeepLabV3+ .blob]
    Pipe --> Depth[Stereo Depth]
    Yolo --> RawDet[Raw Tensors]
    Seg --> RawSeg[Raw Tensors]
    Depth --> RawDepth[Depth Frame]
    RawDet --> Decoder[YOLO Decoder]
    Decoder --> Det2D[Detection2DArray]
    RawDepth --> Depth3D[3D Calculator]
    Det2D --> Depth3D
    Depth3D --> Det3D[Detection3DArray]
    RawSeg --> ZoneCalc[Zone Calculator]
    ZoneCalc --> Zones[Zone Summary]
    Det2D --> Pub[ROS2 Publisher]
    Det3D --> Pub
    Zones --> Pub
    Pub --> Topics[/vision/* Topics]
    Det2D --> Viz[Optional Visualization]
    RawSeg --> Viz
    RawDepth --> Viz
```

### Key Processing Steps

1. **DepthAI Pipeline** runs on OAK-D Lite (Myriad X VPU)
2. **Raw Tensors** are sent to host via USB XLink
3. **YOLO Decoder** (host) converts tensors → Detection2DArray
4. **3D Calculator** (host) combines detections + depth → Detection3DArray
5. **Zone Calculator** (host) converts segmentation mask → zone ratios
6. **ROS2 Publisher** sends to topics for mower brain consumption

## Key components

### OAK device pipeline

- **RGB camera node**
- **Neural network nodes**
  - YOLO NN node
  - Segmentation NN node
- **XLink output streams**
  - detections stream
  - segmentation stream
  - optional preview stream (for visualization only)

### Host application

- **Receiver**: reads streams, timestamps, converts to a stable schema.
- **Visualization (optional)**: overlays boxes + mask on preview.
- **Publisher**: emits results to downstream consumers (file / transport).
- **Stats + logging**: performance metrics, error reporting.

## Data contracts (high-level)

### Detection message (schema v1)

- `schema_version`
- `timestamp` (monotonic or device timestamp, but documented)
- `sequence`
- `model`: `{ name, input_width, input_height }`
- `detections[]`: `{ class_id, class_label, confidence, bbox }`
- `bbox`: explicit convention (see ADR-003)

### Segmentation message (schema v1)

- `schema_version`
- `timestamp`
- `sequence`
- `model`: `{ name, input_width, input_height }`
- `mask`: either:
  - raw class-id tensor (with shape), or
  - encoded representation (with encoding info)
- `class_map_version`

## Deployment architecture

- **Development PC**: run visualization mode for rapid validation.
- **Raspberry Pi**: run headless mode by default; visualization optional.

## Failure modes

- Missing device: fail fast with actionable error.
- Missing/incompatible blob: fail fast with actionable error.
- Stream backpressure: apply explicit drop strategy and log events.

## ADRs (Architecture Decision Records)

### ADR-001: Use DepthAI pipelines for on-device inference

- **Status**: Accepted
- **Context**: The system must execute neural networks on OAK-D Lite hardware.
- **Decision**: Use DepthAI pipeline primitives (camera + NN nodes + XLink outputs).
- **Consequences**
  - Positive: native execution model for OAK; stable SDK support
  - Negative: hardware-in-the-loop is required for full end-to-end testing

### ADR-002: Provide two host modes: visualization and headless

- **Status**: Accepted
- **Context**: Developers need overlays; Raspberry Pi deployments often need low overhead.
- **Decision**: Implement a visualization toggle; headless mode focuses on publishing.
- **Consequences**
  - Positive: preserves performance in deployment
  - Negative: requires careful testing of both modes

### ADR-003: Bounding box coordinate convention is normalized (0..1) with explicit origin

- **Status**: Accepted
- **Context**: Different consumers expect different coordinate spaces; ambiguity creates integration bugs.
- **Decision**: Emit `bbox` as normalized floats in `[0,1]` relative to the model input frame, with:
  - origin at top-left
  - `bbox = { x_min, y_min, x_max, y_max }`
- **Consequences**
  - Positive: easy to convert to pixels for any resolution
  - Negative: consumers must know the frame used for normalization (included in metadata)

### ADR-004: Default integration output is JSON Lines (JSONL) + optional transport later

- **Status**: Accepted
- **Context**: Logging and replay are critical for debugging; robotics control needs real-time messaging.
- **Decision**: JSONL is supported as a **logging/replay** mechanism (`detections.jsonl`, `segmentation.jsonl`) using schema v1.
- **Consequences**
  - Positive: simple, inspectable, easy to replay for tests
  - Negative: file I/O and log rotation must be handled by the host app

### ADR-005: Schema versioning is mandatory for published outputs

- **Status**: Accepted
- **Context**: As the project evolves, fields will change and consumers must not break silently.
- **Decision**: Include `schema_version` in every published message and document changes in a changelog.

### ADR-006: Primary real-time integration is ROS2 topics (Raspberry Pi “sensor” node)

- **Status**: Accepted
- **Context**: The Raspberry Pi runs the robotics “brain” in ROS2. The camera subsystem should behave like a sensor that publishes perception outputs.
- **Decision**: The host application (running on the Raspberry Pi) publishes:
  - detections on a ROS2 topic
  - segmentation mask on a ROS2 topic
  - optional debug preview image on a ROS2 topic
  Canonical schema v1 is defined in `schema-v1.md` and targets ROS 2 Jazzy on Ubuntu 24.04.
- **Consequences**
  - Positive: clean integration with ROS2 consumers (controllers, fusion nodes, logging)
  - Negative: adds ROS2 packaging and message-type dependency decisions (we standardize on `vision_msgs` for detections)

### ADR-007: Implement the ROS2 node in Python (rclpy) for initial delivery

- **Status**: Accepted
- **Context**: The project’s primary implementation language is Python, and DepthAI integration is straightforward in Python. The initial goal is rapid iteration and validation on the Pi.
- **Decision**: Implement the first ROS2 node as a Python package using `rclpy` on ROS 2 Jazzy.
- **Consequences**
  - Positive: faster iteration, easier debugging, strong ecosystem support for this project’s stack
  - Negative: may need optimization or a C++ port later if latency/throughput become limiting

### ADR-008: Use OAK-D Lite stereo depth to produce distance/3D obstacle outputs

- **Status**: Accepted
- **Context**: The robot brain expects obstacle information including distance and size/volume-like estimates for avoidance. OAK-D Lite supports stereo depth on-device.
- **Decision**: Add a depth pipeline (stereo depth) and use it on the host to derive:
  - per-detection distance estimates, and
  - optional 3D bounding box size estimates
  published as `vision_msgs/msg/Detection3DArray` on `/vision/detections_3d`.
- **Consequences**
  - Positive: enables avoidance planning using a standard ROS2 interface
  - Negative: increases pipeline complexity and requires careful calibration/alignment

### ADR-009: Deployment defaults on Raspberry Pi (autostart + ROS logging)

- **Status**: Accepted
- **Context**: The mower should behave like an appliance. The vision node must start reliably at boot and not degrade real-time performance with heavy file I/O by default.
- **Decision**
  - Autostart: provide a `systemd` service option to launch the ROS2 node on boot
  - Logging default: ROS2 logging only (no JSONL file logging by default)
  - Optional debug logging: JSONL can be enabled via a parameter/flag for development and replay
- **Consequences**
  - Positive: reliable startup behavior; minimal I/O overhead by default
  - Negative: less offline replay data unless debug logging is explicitly enabled

### ADR-010: “Telemetry” is local ROS2 status/diagnostics topics (not cloud)

- **Status**: Accepted
- **Context**: The Pi is wired to the camera; “telemetry” should not imply cloud or remote networking. The brain and tools on the Pi still need health/performance signals.
- **Decision**: Publish node health/performance as local ROS2 topics (e.g., `/diagnostics` or `/vision/status`) so other nodes can monitor FPS, latency, and error states.
- **Consequences**
  - Positive: integrates cleanly with ROS monitoring; no external infrastructure required
  - Negative: requires defining a small status message contract and keeping it stable

### ADR-011: Map/geofence is owned by the brain; camera publishes observations only

- **Status**: Accepted
- **Context**: The user defines boundaries by driving the mower around the desired cut area. The brain maintains that map and may update it based on perception (e.g., mark plant areas as no-go).
- **Decision**
  - The camera node publishes observations (detections, segmentation mask, depth-derived 3D detections).
  - The brain fuses observations with the boundary/map and decides “green/yellow/red” actions in the world/map frame.
  - The camera node does not store or require the map to operate.
- **Consequences**
  - Positive: clean separation of concerns; camera stays a sensor
  - Negative: requires good timestamping/TF so the brain can fuse reliably

### ADR-012: TF strategy — publish messages with frame_id; TF is provided by the robot stack

- **Status**: Accepted
- **Context**: Correct fusion requires a TF tree (e.g., `base_link` → `camera_link` → `camera_optical_frame`). Publishing TF inside the camera node can create conflicts with robot-wide TF ownership.
- **Decision**
  - Camera node sets `header.frame_id` consistently (default `oak_rgb_optical_frame`).
  - TF transforms are provided by the robot stack (static transform publisher or URDF/robot_state_publisher).
- **Consequences**
  - Positive: avoids TF conflicts; aligns with ROS conventions
  - Negative: requires a separate TF setup step during integration

### ADR-013: Camera FOV and aspect ratio handling

- **Status**: Accepted
- **Context**: YOLO models expect a fixed input size (640×352). The OAK-D Lite RGB camera is 16:9 (1920×1080). Options: center-crop (loses FOV) or letterbox (preserves FOV with black bars).
- **Decision**
  - Default: Use **letterboxing** to preserve the full ~81° horizontal FOV
  - YOLO input: 640×352 with minimal letterboxing to preserve FOV
  - Preview: 640×360 (16:9) for full FOV visualization
  - Bounding box coordinates are normalized to the actual image region (not including letterbox)
- **Consequences**
  - Positive: mower can detect obstacles approaching from sides; safer operation
  - Negative: slightly more processing; black bars in preview; must handle letterbox offset in coordinate conversion
- **Note**: If center-crop is preferred (simpler but less safe), set `CAM_PREVIEW_SIZE = (640, 352)` in pipeline.py to match the YOLO input.

### ADR-014: USB mode selection strategy

- **Status**: Accepted
- **Context**: OAK-D Lite can run in USB2 (~480 Mbps) or USB3 (~5 Gbps) mode. USB3 provides higher bandwidth but may cause "brownout" resets on some systems due to power draw.
- **Decision**
  - Development machine: **USB3** by default (higher FPS)
  - Raspberry Pi: **USB2** recommended (more stable power)
  - Configurable via `force_usb2` parameter in PipelineRunner
- **Consequences**
  - Positive: best performance on capable hardware; stable fallback available
  - Negative: must test on target platform; FPS varies by USB mode



