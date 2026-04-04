---
title: Story 2.2 — Build and start dual-NN DepthAI pipeline
description: Create the core DepthAI pipeline running YOLO and segmentation concurrently on OAK-D Lite
date: 2026-01-12
---

# Story 2.2 — Build and start dual-NN DepthAI pipeline

## Objective

Create the core DepthAI pipeline that runs YOLOv8n detection and DeepLabV3+ segmentation **concurrently** on the OAK-D Lite camera, streaming results to the host.

## Scope

- DepthAI pipeline creation with RGB camera + 2 NN nodes
- XLink output queues for both model results
- Integration with the ROS2 node

## Out of scope

- Timing metadata (Story 2.3)
- Backpressure/queue management (Story 2.4)
- Depth/3D detections (Story 2.5)
- Zone summary (Story 2.6)

## Acceptance criteria

- [x] Pipeline creates an RGB stream and two NN nodes (YOLO + segmentation)
- [x] Both NN nodes run during the same session
- [x] Host receives both output queues
- [x] Pipeline can be started and stopped cleanly
- [x] Results can be retrieved from the queues

## Technical notes

### Pipeline architecture

```
ColorCamera (RGB)
     │
     ├──► ImageManip (resize 416x416) ──► NeuralNetwork (YOLO) ──► XLinkOut (detections)
     │
     └──► ImageManip (resize 256x256) ──► NeuralNetwork (Seg)  ──► XLinkOut (segmentation)
```

### Key DepthAI components

- `dai.node.ColorCamera` — RGB sensor
- `dai.node.ImageManip` — resize for each model's input size
- `dai.node.NeuralNetwork` — load and run .blob models
- `dai.node.XLinkOut` — stream results to host

### Model input sizes

- YOLO: 416×416
- Segmentation: 256×256

## Suggested implementation

1. Create `ai_camera_vision/pipeline.py`:
   - `create_pipeline(yolo_blob, seg_blob)` — builds the dai.Pipeline
   - `PipelineRunner` class — manages device lifecycle and queue access

2. Integrate into `node.py`:
   - Start pipeline after validation passes
   - Poll queues in a timer or thread
   - Clean shutdown on node destroy

## Definition of Done

- [x] Acceptance criteria met
- [x] Pipeline runs with real hardware (OAK-D Lite, USB3 mode)
- [x] Both detection and segmentation queues produce data
- [x] Clean startup and shutdown

## Implementation Notes (2026-01-12)

- **DepthAI Version**: 2.28.0.0 (NOT 3.x - different API)
- **Python Version**: 3.11.9 (required for depthai 2.x wheels)
- **USB Mode**: USB3 works on dev machine; use `force_usb2=True` if brownout occurs
- **Test Results**: Confirmed detections with `yolov8n_coco_416x416.blob`
- **YOLO Output Layers**: `output0` (concatenated) or separate heads depending on export (currently using `yolov8n` blob format)
- **Segmentation Output Layer**: `Output/Transpose` (for `deeplab_v3_mnv2_256x256`)


