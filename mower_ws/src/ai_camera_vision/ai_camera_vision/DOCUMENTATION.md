# AI Camera Vision - API Documentation

Complete function reference for all modules.

---

## Table of Contents
1. [pipeline.py](#pipelinepy)
2. [detections.py](#detectionspy)
3. [zone_summary.py](#zone_summarypy)
4. [depth.py](#depthpy)
5. [visualization.py](#visualizationpy)
6. [stats.py](#statspy)
7. [status.py](#statuspy)

---

## pipeline.py

Core DepthAI pipeline for OAK-D Lite camera.

### `create_pipeline(yolo_blob_path, seg_blob_path, *, enable_rgb_preview=False, enable_depth=False)`

Creates a DepthAI pipeline with dual-model inference.

**Parameters:**
| Name | Type | Default | Description |
|------|------|---------|-------------|
| `yolo_blob_path` | `str \| Path` | required | Path to YOLO .blob file |
| `seg_blob_path` | `str \| Path` | required | Path to segmentation .blob file |
| `enable_rgb_preview` | `bool` | `False` | Output RGB preview for visualization |
| `enable_depth` | `bool` | `False` | Enable stereo depth output |

**Returns:** `dai.Pipeline` - Configured pipeline

**Example:**
```python
from ai_camera_vision.pipeline import create_pipeline

pipeline = create_pipeline(
    "models/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob",
    "models/deeplab_v3_mnv2_256x256.blob",
    enable_depth=True
)
```

---

### `class PipelineRunner`

Context manager to run pipeline on device.

**Constructor:**
```python
PipelineRunner(pipeline, *, force_usb2=False)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pipeline` | `dai.Pipeline` | required | Pipeline from `create_pipeline()` |
| `force_usb2` | `bool` | `False` | Force USB2 mode (for Pi) |

**Methods:**

#### `get_detection_result(timeout_ms=100)`
Get next YOLO detection result.

| Parameter | Type | Default |
|-----------|------|---------|
| `timeout_ms` | `int` | `100` |

**Returns:** `InferenceResult | None`

#### `get_segmentation_result(timeout_ms=100)`
Get next segmentation result.

**Returns:** `InferenceResult | None`

#### `get_depth_result(timeout_ms=100)`
Get next depth frame.

**Returns:** `ImageResult | None`

#### `get_rgb_result(timeout_ms=100)`
Get next RGB preview frame.

**Returns:** `ImageResult | None`

**Example:**
```python
with PipelineRunner(pipeline, force_usb2=True) as runner:
    det = runner.get_detection_result(timeout_ms=0)  # Non-blocking
    if det:
        print(f"Frame {det.sequence_num} at {det.timestamp_ns}")
```

---

### `class InferenceResult`

Dataclass for NN inference results.

| Attribute | Type | Description |
|-----------|------|-------------|
| `raw_data` | `dai.NNData` | Raw neural network output |
| `timestamp_ns` | `int` | Device timestamp (nanoseconds) |
| `sequence_num` | `int` | Frame sequence number |

---

### `class ImageResult`

Dataclass for image frame results.

| Attribute | Type | Description |
|-----------|------|-------------|
| `raw_frame` | `dai.ImgFrame` | Raw image frame |
| `timestamp_ns` | `int` | Device timestamp |
| `sequence_num` | `int` | Frame sequence number |

---

## detections.py

Detection data structures and helpers.

### `class Detection2D`

Single 2D detection from YOLO.

| Attribute | Type | Description |
|-----------|------|-------------|
| `class_id` | `int` | COCO class ID (0-79) |
| `class_label` | `str` | Human-readable class name |
| `confidence` | `float` | Detection confidence [0, 1] |
| `x_min` | `float` | Left edge, normalized [0, 1] |
| `y_min` | `float` | Top edge, normalized [0, 1] |
| `x_max` | `float` | Right edge, normalized [0, 1] |
| `y_max` | `float` | Bottom edge, normalized [0, 1] |
| `safety_category` | `str` | human/animal/vehicle/unknown |

**Properties:**
- `width` → `float` - Normalized width
- `height` → `float` - Normalized height
- `center_x` → `float` - Normalized center X
- `center_y` → `float` - Normalized center Y

**Methods:**

#### `to_pixels(frame_width, frame_height)`
Convert to pixel coordinates.

**Returns:** `tuple[int, int, int, int]` - (x1, y1, x2, y2)

#### `to_dict()`
Convert to dictionary for JSON.

**Returns:** `dict`

---

### `get_class_label(class_id)`

Get class name from COCO class ID.

| Parameter | Type | Description |
|-----------|------|-------------|
| `class_id` | `int` | COCO class ID (0-79) |

**Returns:** `str` - Class name or "class_N"

---

### `get_safety_category(class_label)`

Map class to safety category.

| Parameter | Type | Description |
|-----------|------|-------------|
| `class_label` | `str` | Class name |

**Returns:** `str` - "human", "animal", "vehicle", or "unknown"

---

### `COCO_CLASSES`

List of 80 COCO class names.

```python
COCO_CLASSES[0]   # "person"
COCO_CLASSES[2]   # "car"
COCO_CLASSES[16]  # "dog"
```

---

### `SAFETY_CATEGORY_MAP`

Dictionary mapping class names to safety categories.

```python
SAFETY_CATEGORY_MAP["person"]  # "human"
SAFETY_CATEGORY_MAP["car"]     # "vehicle"
SAFETY_CATEGORY_MAP["dog"]     # "animal"
```

---

## zone_summary.py

Segmentation zone ratio computation.

### `class ZoneSummary`

Zone ratios from segmentation mask.

| Attribute | Type | Description |
|-----------|------|-------------|
| `green_ratio` | `float` | Safe area fraction [0, 1] |
| `yellow_ratio` | `float` | Caution area fraction [0, 1] |
| `red_ratio` | `float` | No-go area fraction [0, 1] |
| `total_pixels` | `int` | Total pixels analyzed |
| `classified_pixels` | `int` | Non-background pixels |

**Properties:**
- `is_safe` → `bool` - True if mostly green
- `has_person` → `bool` - True if any red
- `recommendation` → `str` - "GO", "SLOW", or "STOP"

**Methods:**
- `to_array()` → `list[float]` - [green, yellow, red]
- `to_dict()` → `dict` - For JSON

---

### `compute_zone_ratios(mask, roi=None, model_type=MODEL_PASCAL_VOC)`

Compute zone ratios from segmentation mask.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `mask` | `np.ndarray` | required | 2D class ID mask |
| `roi` | `tuple[int,int,int,int]` | `None` | (x, y, w, h) region |
| `model_type` | `str` | `MODEL_PASCAL_VOC` | Model type |

**Returns:** `ZoneSummary`

**Example:**
```python
from ai_camera_vision.zone_summary import compute_zone_ratios

summary = compute_zone_ratios(seg_mask)
print(f"Safe: {summary.green_ratio:.1%}")
print(f"Action: {summary.recommendation}")
```

---

### `decode_segmentation_mask(seg_data, expected_shape=(256, 256))`

Decode raw NN output to class ID mask.

| Parameter | Type | Default |
|-----------|------|---------|
| `seg_data` | `np.ndarray` | required |
| `expected_shape` | `tuple[int,int]` | `(256, 256)` |

**Returns:** `np.ndarray | None` - 2D uint8 mask

---

## depth.py

3D position calculations from depth data.

### `class Detection3D`

3D detection with spatial position.

| Attribute | Type | Description |
|-----------|------|-------------|
| `class_id` | `int` | COCO class ID |
| `class_label` | `str` | Class name |
| `confidence` | `float` | Detection confidence |
| `bbox_2d` | `BoundingBox2D` | Original 2D bbox |
| `position_x` | `float` | X position (meters, right+) |
| `position_y` | `float` | Y position (meters, down+) |
| `position_z` | `float` | Z position (meters, forward) |
| `size_x` | `float \| None` | Width in meters |
| `size_y` | `float \| None` | Height in meters |
| `depth_valid` | `bool` | Is depth measurement valid |
| `depth_confidence` | `float` | Confidence in depth [0-1] |

**Properties:**
- `distance` → `float` - Distance from camera (same as position_z)

---

### `sample_depth_at_bbox_center(depth_frame, bbox, frame_shape=None)`

Get depth at detection center.

| Parameter | Type | Description |
|-----------|------|-------------|
| `depth_frame` | `np.ndarray` | Depth frame (mm) |
| `bbox` | `BoundingBox2D` | Normalized bbox |
| `frame_shape` | `tuple[int,int]` | (height, width) |

**Returns:** `int` - Depth in millimeters

---

### `compute_detection_3d(...)`

Convert 2D detection to 3D.

**Returns:** `Detection3D`

---

## visualization.py

OpenCV visualization utilities.

### `class PreviewWindow`

OpenCV window with exit handling.

**Constructor:**
```python
PreviewWindow(title="AI Camera Vision", *, scale=1.0)
```

**Properties:**
- `is_open` → `bool` - Window still open?

**Methods:**

#### `show(frame)`
Display frame and check for exit key.

**Returns:** `bool` - True to continue, False to exit

#### `close()`
Close window and cleanup.

**Example:**
```python
from ai_camera_vision.visualization import PreviewWindow

with PreviewWindow("My Window", scale=1.5) as preview:
    while preview.is_open:
        preview.show(frame)
```

---

### `draw_detections(frame, detections, *, confidence_threshold=0.5)`

Draw bounding boxes on frame.

| Parameter | Type | Default |
|-----------|------|---------|
| `frame` | `np.ndarray` | required |
| `detections` | `list[Detection2D]` | required |
| `confidence_threshold` | `float` | `0.5` |

**Returns:** `np.ndarray` - Frame with boxes

---

### `overlay_segmentation(frame, mask, *, alpha=0.4)`

Overlay colored mask on frame.

| Parameter | Type | Default |
|-----------|------|---------|
| `frame` | `np.ndarray` | required |
| `mask` | `np.ndarray` | required |
| `alpha` | `float` | `0.4` |

**Returns:** `np.ndarray` - Frame with overlay

---

## stats.py

Runtime statistics tracking.

### `class RuntimeStats`

Aggregate statistics manager.

**Constructor:**
```python
RuntimeStats(log_interval_sec=10.0)
```

**Methods:**

#### `record_frame(stream, timestamp_ns, host_time=None, sequence_num=None)`
Record a frame for statistics.

| Parameter | Type | Description |
|-----------|------|-------------|
| `stream` | `str` | "detection", "segmentation", etc |
| `timestamp_ns` | `int` | Device timestamp |
| `sequence_num` | `int` | For drop detection |

#### `get_summary()`
**Returns:** `str` - Formatted summary

#### `detection_fps`
**Returns:** `float` - Detection stream FPS

---

## status.py

System health monitoring.

### `class VisionStatus`

Health status dataclass.

| Attribute | Type |
|-----------|------|
| `status` | `StatusLevel` |
| `detection_fps` | `float` |
| `segmentation_fps` | `float` |
| `detection_active` | `bool` |
| `last_error` | `str \| None` |

**Properties:**
- `is_healthy` → `bool`
- `is_critical` → `bool`

---

### `class StatusLevel`

Status enum.

| Value | Meaning |
|-------|---------|
| `OK` | All good |
| `WARN` | Low FPS or high drops |
| `ERROR` | Critical failure |
| `STALE` | No recent data |

---

### `compute_status(stats, last_error=None, depth_enabled=True)`

Compute current system status.

**Returns:** `VisionStatus`
