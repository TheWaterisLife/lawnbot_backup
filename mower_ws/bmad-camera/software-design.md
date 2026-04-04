# Software Design — AI Camera Vision (`ai_camera_vision`)

## 1. Overview

The `ai_camera_vision` package implements a dual-model neural network inference pipeline on the OAK-D Lite stereo camera. The DepthAI pipeline runs YOLOv4-tiny object detection and DeepLabV3+ semantic segmentation on the Myriad X VPU, while the host (Raspberry Pi) decodes raw tensors, computes 3D obstacle distances using stereo depth, and publishes results as ROS 2 topics.

### 1.1 Design Rationale

All neural network inference runs on-device (Myriad X VPU) to keep the Raspberry Pi CPU free for control logic. The host only performs lightweight post-processing: YOLO tensor decoding, depth sampling, and zone calculation. Two deployment modes (visualization for development, headless for production) are controlled by a single toggle, ensuring minimal overhead in the field.

### 1.2 Key Responsibilities

- Construct and deploy DepthAI pipeline (RGB camera + YOLO NN + segmentation NN + stereo depth)
- Decode raw YOLO tensors into Detection2D structures with letterbox-corrected bounding boxes
- Sample stereo depth within detection bounding boxes for 3D distance/bearing estimation
- Compute segmentation zone ratios (grass %, obstacle %) from the per-pixel class mask
- Publish detections, 3D detections, and zone summaries on `/vision/*` topics
- Optional OpenCV visualization overlay for development

---

## 2. Class Diagram

```plantuml
@startuml
skinparam classAttributeIconSize 0

package "ai_camera_vision" {

  class PipelineRunner {
    -device : dai.Device
    -pipeline : dai.Pipeline
    -force_usb2 : bool
    -cam_preview_size : Tuple[int, int]
    +build_pipeline() : dai.Pipeline
    +start() : void
    +stop() : void
    +get_detection_queue() : DataOutputQueue
    +get_segmentation_queue() : DataOutputQueue
    +get_depth_queue() : DataOutputQueue
    +get_preview_queue() : DataOutputQueue
  }

  class YOLODecoder {
    -model_input_w : int
    -model_input_h : int
    -confidence_threshold : float
    -nms_threshold : float
    -class_labels : List[str]
    +decode(raw_tensor: NNData) : List[Detection2D]
    +adjust_bbox_from_letterbox(bbox: BBox) : BBox
  }

  class Depth3DCalculator {
    -hfov_deg : float
    -depth_frame_w : int
    -depth_frame_h : int
    +sample_depth_robust(depth_frame, bbox) : float
    +compute_bearing(bbox_center_x: float) : float
    +to_detection_3d(det2d, depth_mm) : Detection3D
  }

  class ZoneCalculator {
    -zone_config : dict
    -class_map : Dict[int, str]
    +compute_zones(mask: ndarray) : ZoneSummary
  }

  class CameraVisionNode {
    -pipeline_runner : PipelineRunner
    -yolo_decoder : YOLODecoder
    -depth_calc : Depth3DCalculator
    -zone_calc : ZoneCalculator
    -pub_det2d : Publisher
    -pub_det3d : Publisher
    -pub_zones : Publisher
    -pub_preview : Publisher
    -headless : bool
    -frame_id : str
    +run_loop() : void
    +process_frame() : void
  }

  class Detection2D <<dataclass>> {
    +class_id : int
    +class_label : str
    +confidence : float
    +bbox : BBox
  }

  class Detection3D <<dataclass>> {
    +class_id : int
    +class_label : str
    +confidence : float
    +distance_mm : float
    +bearing_deg : float
    +bbox : BBox
  }

  class BBox <<dataclass>> {
    +x_min : float
    +y_min : float
    +x_max : float
    +y_max : float
  }

  class ZoneSummary <<dataclass>> {
    +grass_ratio : float
    +obstacle_ratio : float
    +unknown_ratio : float
    +timestamp : float
  }

  CameraVisionNode "1" *-- "1" PipelineRunner
  CameraVisionNode "1" *-- "1" YOLODecoder
  CameraVisionNode "1" *-- "1" Depth3DCalculator
  CameraVisionNode "1" *-- "1" ZoneCalculator
  YOLODecoder ..> Detection2D : produces
  Depth3DCalculator ..> Detection3D : produces
  ZoneCalculator ..> ZoneSummary : produces
  Detection2D "1" *-- "1" BBox
  Detection3D "1" *-- "1" BBox
}

package "depthai" <<external>> {
  class Device
  class Pipeline
  class NNData
}

package "rclpy" <<external>> {
  class Node
}

CameraVisionNode --|> Node
PipelineRunner "1" *-- "1" Device
PipelineRunner "1" *-- "1" Pipeline

@enduml
```

---

## 3. Sequence Diagram

```plantuml
@startuml
skinparam sequenceArrowThickness 2

participant "CameraVisionNode" as Node
participant "PipelineRunner" as Runner
participant "OAK-D Lite\n(Myriad X)" as OAK
participant "YOLODecoder" as YOLO
participant "Depth3DCalculator" as Depth
participant "ZoneCalculator" as Zone
participant "ROS 2 Topics" as Topics

== Initialization ==
Node -> Runner : build_pipeline()
Runner -> OAK : deploy pipeline via USB XLink
note right of OAK : 3 streams:\n1. YOLO NN output\n2. Segmentation NN output\n3. Stereo depth frame
Runner --> Node : pipeline ready

== Frame Processing Loop ==
loop each camera frame
  OAK -> Runner : YOLO raw tensors (XLink)
  OAK -> Runner : Segmentation mask (XLink)
  OAK -> Runner : Depth frame (XLink)

  Node -> YOLO : decode(raw_tensor)
  YOLO -> YOLO : parse grid cells, apply NMS
  YOLO -> YOLO : adjust_bbox_from_letterbox()
  note right : 640×352 YOLO input\nvs 640×360 preview\nletterbox offset correction
  YOLO --> Node : List[Detection2D]

  Node -> Topics : publish Detection2DArray\non /vision/detections_2d

  loop each Detection2D
    Node -> Depth : sample_depth_robust(depth_frame, bbox)
    Depth -> Depth : median of samples within bbox
    Depth --> Node : distance_mm
    Node -> Depth : compute_bearing(bbox_center_x)
    Depth --> Node : bearing_deg (from 73° HFOV)
    Node -> Depth : to_detection_3d(det2d, distance_mm)
    Depth --> Node : Detection3D
  end

  Node -> Topics : publish Detection3DArray\non /vision/detections_3d

  Node -> Zone : compute_zones(segmentation_mask)
  Zone -> Zone : count per-pixel class IDs
  Zone -> Zone : compute grass/obstacle ratios
  Zone --> Node : ZoneSummary

  Node -> Topics : publish ZoneSummary\non /vision/zones

  opt headless == false
    Node -> Node : overlay bboxes + mask\non preview (OpenCV)
  end
end

@enduml
```

---

## 4. System Context Diagram

```plantuml
@startuml
!include <C4/C4_Context>

title System Context — AI Camera Vision

System(camera, "ai_camera_vision", "Dual-model inference\non OAK-D Lite\n+ ROS 2 publisher")
System(autonomy, "autonomy_node", "Consumes obstacle\ndetections for avoidance")
System(obstacle, "obstacle_detection", "Standalone YOLO-based\nobstacle avoidance")
System_Ext(oakd, "OAK-D Lite Camera", "RGB + Stereo depth\nMyriad X VPU\nUSB connection")
System_Ext(yolo_blob, "YOLOv4-tiny .blob", "640×352 input\n6 shaves, COCO 80 classes")
System_Ext(seg_blob, "DeepLabV3+ .blob", "Semantic segmentation\nper-pixel class mask")

oakd --> camera : USB XLink streams\n(detections, segmentation, depth)
yolo_blob --> oakd : deployed to VPU
seg_blob --> oakd : deployed to VPU
camera --> autonomy : /vision/detections_3d\n(Detection3DArray)
camera --> autonomy : /vision/zones\n(ZoneSummary)
camera --> obstacle : shared OAK-D pipeline\n(when running standalone)

note bottom of camera
  header.frame_id = "oak_rgb_optical_frame"
  TF transforms managed by robot stack
  Modes: headless (Pi) / visualization (dev)
end note

@enduml
```

---

## 5. Detailed Design

### 5.1 DepthAI Pipeline Architecture

The pipeline deployed to the OAK-D Lite consists of three parallel processing streams sharing a single RGB camera input:

| Stream | Model | Input Size | Output | Shaves |
|---|---|---|---|---|
| YOLO Detection | YOLOv4-tiny (.blob) | 640×352 | Raw detection tensors | 6 |
| Segmentation | DeepLabV3+ (.blob) | Model-specific | Per-pixel class-id mask | Shared |
| Stereo Depth | Built-in stereo | Camera native | Depth frame (mm) | N/A |

### 5.2 Letterbox Handling

The YOLO model expects 640×352 input while the camera produces 640×360 (16:9). Letterboxing preserves the full ~81° horizontal FOV with 4-pixel black bars at top and bottom. The `adjust_bbox_from_letterbox()` function corrects bounding box y-coordinates by subtracting the letterbox offset, ensuring accurate pixel-to-world coordinate mapping.

### 5.3 Depth Sampling

For each YOLO detection, `sample_depth_robust()` takes the median of multiple depth samples within the bounding box region. The median rejects outlier depth values (e.g., background pixels at bbox edges). Depth in millimeters is combined with the detection's horizontal pixel position and the camera's 73° HFOV to compute bearing angle:

```
bearing = (bbox_center_x / frame_width - 0.5) × HFOV
```

### 5.4 Segmentation Zones

The `ZoneCalculator` converts the per-pixel class-id mask into zone ratios by counting pixel class frequencies. The output `ZoneSummary` reports the fraction of the frame occupied by grass, obstacles, and unknown classes.

### 5.5 Deployment Modes

| Mode | OpenCV | File I/O | Use Case |
|---|---|---|---|
| Headless | Disabled | ROS 2 logging only | Production on Raspberry Pi |
| Visualization | Enabled (overlay) | Optional JSONL | Development and debugging |

### 5.6 USB Mode Selection

| Platform | USB Mode | Rationale |
|---|---|---|
| Development PC | USB3 (5 Gbps) | Higher FPS for rapid iteration |
| Raspberry Pi | USB2 (480 Mbps) | More stable power, avoids brownout resets |

Configurable via the `force_usb2` parameter in `PipelineRunner`.

### 5.7 Published Topics

| Topic | Message Type | Content |
|---|---|---|
| `/vision/detections_2d` | `vision_msgs/Detection2DArray` | 2D bounding boxes with class and confidence |
| `/vision/detections_3d` | `vision_msgs/Detection3DArray` | 3D detections with distance and bearing |
| `/vision/zones` | Custom `ZoneSummary` | Grass/obstacle/unknown ratios |
| `/vision/status` | Diagnostics | FPS, latency, error states |

### 5.8 Schema Versioning

All published messages include `schema_version` in metadata (ADR-005). The canonical schema is defined in `schema-v1.md`. Detection messages include model name, input dimensions, and sequence number for replay and debugging.
