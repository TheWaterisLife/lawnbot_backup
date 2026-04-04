---
title: Output Schema v1 — AI Camera Vision System
description: Canonical output schema (file + ROS2 mapping) for detections and semantic segmentation
date: 2026-01-12
---

# Output Schema v1 — AI Camera Vision System

This document defines the **canonical output schema v1** for the OAK-D Lite dual-model pipeline.

The schema is designed to support:

- **Real-time integration** via ROS2 topics (primary)
- **Logging and replay** via JSON Lines (JSONL) files (optional)

## Coordinate conventions

### Bounding boxes

Per ADR-003, `bbox` is **normalized** to the model input frame:

- Origin: **top-left**
- Range: each value is in \([0, 1]\)
- Format: `bbox = { x_min, y_min, x_max, y_max }`

### Segmentation mask

The segmentation mask is a 2D image of **class IDs** where each pixel is an integer class ID.

- Shape: `(height, width)` equals the segmentation model input (e.g., 256×256) unless otherwise documented
- Alignment: any scaling/letterboxing strategy MUST be documented by the implementation and reflected in metadata

## Segmentation semantics (zones)

The segmentation output is used to classify terrain into operational "zones" for the mower brain:

- **Green**: safe-to-cut / safe-to-drive
- **Yellow**: safe-to-drive but **do not cut**
- **Red**: interdiction (do not cut, do not drive)

### Zone class IDs (class_map_version: v1)

- `0`: unknown
- `1`: green (grass / cuttable)
- `2`: yellow (drive-only)
- `3`: red (no-go)

### Current Model Output (DeepLabV3+ Person)

**Important**: The current `deeplab_v3_mnv2_256x256.blob` model outputs **PASCAL VOC classes**, not terrain zones directly:

| Model Output | PASCAL VOC Class | Mapped Zone |
|--------------|------------------|-------------|
| 0 | background | green (grass) |
| 15 | person | **red** (no-go) |
| 1-14, 16-20 | other objects | yellow (caution) |

The zone summary computation (Story 2.6) maps model output to zones:
- Background → Green (safe)
- Person → Red (danger!)
- Other detected objects → Yellow (caution)

### ADAS Model Output (semantic-segmentation-adas-0001)

The ADAS model provides **20 outdoor/driving scene classes** and is the **recommended default** for lawn mower operation:

| Class ID | Class | Zone | Rationale |
|----------|-------|------|-----------|
| 0 | road | YELLOW | Safe to drive, not grass |
| 1 | sidewalk | YELLOW | Safe to drive, not grass |
| 2-7 | building, wall, fence, pole, traffic light/sign | RED | Obstacles |
| 8 | vegetation | **GREEN** | Grass/plants = safe to cut |
| 9 | terrain | **GREEN** | Ground/dirt = safe to drive |
| 10 | sky | *(ignored)* | Not ground-level |
| 11 | person | **RED** | Human = STOP |
| 12 | rider | **RED** | Human on vehicle = STOP |
| 13-18 | car, truck, bus, train, motorcycle, bicycle | RED | Vehicles |
| 19 | ego-vehicle | *(ignored)* | The mower itself |

The zone summary supports both models via a `model_type` parameter:
- `MODEL_ADAS` (default): Uses ADAS class mapping above
- `MODEL_PASCAL_VOC`: Uses legacy DeepLabV3+ mapping

**Note**: ADAS model excludes sky (class 10) and ego-vehicle (class 19) from ratio calculations.

### Interaction with the mower map (geofence)

The mower “brain” (Raspberry Pi) maintains the user-defined boundary/map and applies overrides:

- The camera node publishes **observations** (mask + detections).
- The brain may fuse these observations with the user-defined map to mark regions as yellow/red.
- The camera node does **not** need the map to publish its outputs.

## Common fields (all messages)

- `schema_version` (string): always `"v1"`
- `timestamp`
  - `source` (string): `"device"` or `"host"`
  - `t_ns` (integer): timestamp in nanoseconds
- `sequence` (integer): monotonically increasing per stream
- `frame_id` (string): coordinate frame identifier (ROS-style, e.g., `"oak_rgb_optical_frame"`)
- `model`
  - `name` (string)
  - `input_width` (integer)
  - `input_height` (integer)

## Detection message (schema v1)

### Fields

- All **common fields**
- `detections` (array)
  - `class_id` (integer)
  - `class_label` (string)
  - `confidence` (number)
  - `bbox` (object): `{ x_min, y_min, x_max, y_max }` normalized
  - `safety_category` (string, optional): `"human" | "animal" | "vehicle" | "static_obstacle" | "unknown"`

### Example (JSON object)

```json
{
  "schema_version": "v1",
  "timestamp": { "source": "device", "t_ns": 1736700000000000000 },
  "sequence": 42,
  "frame_id": "oak_rgb_optical_frame",
  "model": { "name": "yolov8n_coco_416x416", "input_width": 416, "input_height": 416 },
  "detections": [
    {
      "class_id": 0,
      "class_label": "person",
      "confidence": 0.87,
      "bbox": { "x_min": 0.31, "y_min": 0.18, "x_max": 0.55, "y_max": 0.92 }
    }
  ]
}
```

## Segmentation message (schema v1)

### Fields

- All **common fields**
- `class_map_version` (string): identifies which label map is in use
- `mask`
  - `encoding` (string): `"mono8"` (preferred for class IDs 0–255) or another documented encoding
  - `width` (integer)
  - `height` (integer)
  - `data_b64` (string): base64-encoded bytes of the mask image row-major

### Example (JSON object)

```json
{
  "schema_version": "v1",
  "timestamp": { "source": "device", "t_ns": 1736700000000000000 },
  "sequence": 42,
  "frame_id": "oak_rgb_optical_frame",
  "model": { "name": "deeplabv3p_256x256", "input_width": 256, "input_height": 256 },
  "class_map_version": "v1",
  "mask": { "encoding": "mono8", "width": 256, "height": 256, "data_b64": "<base64-bytes>" }
}
```

## Depth / obstacle metrics (for “brain” planning)

To support obstacle avoidance planning, the system additionally publishes **3D detections** in ROS2 using standard message types. This is the preferred real-time interface for distance/volume-like reasoning.

### Distance + size (recommended)

The minimum recommended outputs per obstacle are:

- **Distance** (meters): forward range from the camera/robot frame
- **Size** (meters): approximate dimensions when feasible (x/y/z)

## Safety-critical object classes (default policy)

YOLO provides many classes; the brain needs a simpler “what do I avoid?” signal. Default policy:

- Treat these as **must-avoid** (high priority): `person`
- Treat these as **avoid**: `dog`, `cat`, `bicycle`, `motorcycle`, `car`, `truck`, `bus`
- Treat these as **obstacles** (avoid if detected and confident): `potted plant`, `chair`, `bench`, `stop sign`, `fire hydrant`

Everything else can default to `unknown` and still be published; the brain can decide whether to ignore it.

## ROS2 mapping (primary integration)

### Target platform

- OS: Ubuntu 24.04.x LTS (Raspberry Pi 5)
- ROS 2 distro: Jazzy Jalisco

### Topics

- Detections: `/vision/detections` (recommended)
- Detections (3D): `/vision/detections_3d` (recommended)
- Segmentation mask: `/vision/segmentation/mask` (recommended)
- Segmentation zone summary: `/vision/segmentation/zone_summary` (recommended)
- Optional debug preview image: `/vision/rgb/preview` (optional)

### Message types (recommended)

- **Detections**: `vision_msgs/msg/Detection2DArray` (standard)
  - Each detection becomes a `Detection2D` with:
    - `bbox` converted to pixel coordinates of the chosen reference image
    - `results[]` contains `{ id, score }` (map `class_id` and `confidence`)
- **Segmentation mask**: `sensor_msgs/msg/Image`
  - `encoding`: `"mono8"` (class IDs)
  - `width/height`: mask dimensions
  - `data`: raw bytes (no base64 in ROS)
- **Segmentation zone summary**: `std_msgs/msg/Float32MultiArray`
  - A small, low-bandwidth summary derived from the mask for fast planning.
  - Default field order (3 floats, each in \([0,1]\)):
    1. `green_ratio`
    2. `yellow_ratio`
    3. `red_ratio`
  - Ratios represent the fraction of pixels in the evaluated region-of-interest (ROI).
  - ROI policy (default): full mask. (May be updated later to “front-of-mower wedge/strip” for better control decisions.)
- **Detections (3D)**: `vision_msgs/msg/Detection3DArray` (standard)
  - Each detection includes a 3D position (distance) and 3D bounding box when available.
  - Recommended convention:
    - `bbox.center.position.z` represents forward distance in meters
    - If “volume” is needed, prefer publishing `bbox.size` (x/y/z) rather than a single derived scalar.
- **Class map**: publish once on startup (recommended) as:
  - `/vision/segmentation/class_map` using `std_msgs/msg/String` containing JSON, or
  - document it as a static file installed with the node

### Required ROS header fields

- `header.stamp` comes from the same timestamp used in the canonical schema
- `header.frame_id` matches `frame_id` above

### Frame strategy (recommended default)

- Publish messages in the **camera frame** (e.g., `oak_rgb_optical_frame`).
- World/map-frame projection and “edit the map” logic belong in the brain (localization + TF + planning).

## JSONL mapping (optional logging)

If JSONL logging is enabled:

- Write one **detection message JSON object** per line to `detections.jsonl`
- Write one **segmentation message JSON object** per line to `segmentation.jsonl`
- Every line must include `schema_version: "v1"`



