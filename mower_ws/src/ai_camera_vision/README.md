# AI Camera Vision

OAK-D Lite camera pipeline for YOLO object detection and DeepLabV3 segmentation.

## Quick Start (Raspberry Pi)

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Run as ROS2 Node
```bash
# Build the ROS2 package
colcon build --packages-select ai_camera_vision

# Source the workspace
source install/setup.bash

# Run the vision node
ros2 run ai_camera_vision ai_camera_vision_node --ros-args \
  -p yolo_blob_path:="models/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob" \
  -p seg_blob_path:="models/deeplab_v3_mnv2_256x256.blob" \
  -p visualization_enabled:=false
```

### Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `yolo_blob_path` | string | required | Path to YOLO detection model blob |
| `seg_blob_path` | string | required | Path to segmentation model blob |
| `visualization_enabled` | bool | false | Enable OpenCV preview window |

### 3. Run Demo (Standalone)
```python
from ai_camera_vision.pipeline import create_pipeline, PipelineRunner

# Paths to model files
yolo_blob = "models/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"
seg_blob = "models/deeplab_v3_mnv2_256x256.blob"

# Create and run pipeline
pipeline = create_pipeline(yolo_blob, seg_blob, enable_depth=True)

with PipelineRunner(pipeline) as runner:
    while True:
        # Get detection results
        det_result = runner.get_detection_result(timeout_ms=100)
        if det_result:
            print(f"Detection frame #{det_result.sequence_num}")
        
        # Get segmentation results
        seg_result = runner.get_segmentation_result(timeout_ms=100)
        if seg_result:
            print(f"Segmentation frame #{seg_result.sequence_num}")
```

---

## Package Structure

```
ai_camera_vision/
├── models/                     # Neural network blobs
│   ├── yolo-v3-tiny-tf_....blob   # YOLO detection (~17MB)
│   └── deeplab_v3_mnv2_256x256.blob  # Segmentation (~7MB)
├── ai_camera_vision/           # Python package
│   ├── pipeline.py             # Core camera pipeline
│   ├── detections.py           # Detection2D dataclass
│   ├── depth.py                # 3D position calculations
│   ├── zone_summary.py         # Segmentation zone ratios
│   ├── visualization.py        # OpenCV overlays
│   ├── stats.py                # FPS/latency tracking
│   └── status.py               # Health monitoring
├── requirements.txt            # Python dependencies
└── README.md                   # This file
```

---

## Module Reference

### pipeline.py - Camera Pipeline

**Main functions:**
| Function | Description |
|----------|-------------|
| `create_pipeline(yolo_blob, seg_blob)` | Creates DepthAI pipeline |
| `PipelineRunner(pipeline)` | Context manager to run pipeline |

**Key classes:**
- `InferenceResult`: Detection/segmentation result with timestamp
- `ImageResult`: RGB/depth frame with timestamp

**Configuration to edit:**
- Line ~416: `CAM_PREVIEW_SIZE` - Camera resolution
- Line ~504: Confidence threshold for YOLO

---

### detections.py - Detection Data

**Classes:**
| Class | Description |
|-------|-------------|
| `Detection2D` | Single detection (class, confidence, bbox) |

**Constants:**
- `COCO_CLASSES`: 80 YOLO class names
- `SAFETY_CATEGORY_MAP`: Maps classes to human/animal/vehicle

**Helper functions:**
- `get_class_label(class_id)` → class name
- `get_safety_category(class_label)` → safety category

---

### zone_summary.py - Segmentation Zones

**Classes:**
| Class | Description |
|-------|-------------|
| `ZoneSummary` | Green/yellow/red ratios |

**Functions:**
- `compute_zone_ratios(mask)` → ZoneSummary
- `decode_segmentation_mask(seg_data)` → numpy mask

**Zone colors:**
- 🟢 GREEN: Safe (background)
- 🟡 YELLOW: Caution (objects)
- 🔴 RED: Stop (person)

---

### depth.py - 3D Calculations

**Classes:**
| Class | Description |
|-------|-------------|
| `BoundingBox2D` | Normalized bbox |
| `Detection3D` | 3D position + size |

**Functions:**
- `sample_depth_at_bbox_center(depth_frame, bbox)` → depth_mm
- `compute_detection_3d(...)` → Detection3D

---

### visualization.py - Display

**Classes:**
| Class | Description |
|-------|-------------|
| `PreviewWindow` | OpenCV window manager |

**Functions:**
- `draw_detections(frame, detections)` → frame with boxes
- `overlay_segmentation(frame, mask)` → frame with overlay
- `draw_zone_legend(frame)` → frame with legend

---

## Hardware Requirements

| Component | Requirement |
|-----------|-------------|
| Camera | OAK-D Lite |
| USB | USB3 preferred (USB2 works with `force_usb2=True`) |
| RAM | 4GB minimum |
| Python | 3.11 |

---

## Troubleshooting

### Camera not found
```bash
# Check USB connection
lsusb | grep Luxonis
```

### USB brownout (camera resets)
Use USB2 mode:
```python
with PipelineRunner(pipeline, force_usb2=True) as runner:
    ...
```

### Low FPS
- Use smaller model input size
- Reduce queue sizes
- Close other camera applications
