"""Live demo of the AI Camera Vision pipeline with full visualization.

This script shows what's happening in real-time with VISUAL FEEDBACK:
- RGB camera feed with detection boxes and labels
- Depth colormap (red=close, blue=far)
- Segmentation mask overlay (person highlighted)
- Frame rate statistics

TERMINAL OUTPUT (Press 't' to toggle):
Shows exactly what data would be published to ROS2 topics on the Raspberry Pi:
- /vision/detections         → Detection2DArray (objects found)
- /vision/detections_3d      → Detection3DArray (with distance)
- /vision/segmentation/zones → Zone ratios (green/yellow/red)
- /vision/status             → Health/FPS data

Run with: py -3.11 demo_live_view.py

Controls:
  q     - Quit
  t     - Toggle terminal data output (what Pi receives)
  d     - Toggle depth view
  s     - Toggle segmentation overlay
  b     - Toggle detection boxes
  1-3   - Switch display mode (1=RGB, 2=Depth, 3=Side-by-side)
  h     - Show/hide help
"""

import sys
import time
import json
from pathlib import Path
from datetime import datetime

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

# CRITICAL: Import depthai (via pipeline) BEFORE opencv to avoid Windows crash
# See: https://discuss.luxonis.com/d/1089-importerror-dll-load-failed
from ai_camera_vision.pipeline import (
    PipelineRunner,
    create_pipeline,
    CAM_PREVIEW_SIZE,
    YOLO_INPUT_SIZE,
)
from ai_camera_vision.depth import BoundingBox2D, compute_detection_3d, sample_depth_robust
from ai_camera_vision.detections import Detection2D

# OpenCV must be imported AFTER depthai
import cv2
import numpy as np



# =============================================================================
# COCO Class Names (YOLOv8 uses COCO 80 classes)
# =============================================================================

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
]

# Safety categories for different objects
SAFETY_CATEGORIES = {
    "person": "human",
    "dog": "animal", "cat": "animal", "bird": "animal", "horse": "animal",
    "cow": "animal", "sheep": "animal", "elephant": "animal", "bear": "animal",
    "zebra": "animal", "giraffe": "animal",
    "car": "vehicle", "truck": "vehicle", "bus": "vehicle", "motorcycle": "vehicle",
    "bicycle": "vehicle", "train": "vehicle", "airplane": "vehicle", "boat": "vehicle",
}

# Colors for safety categories (BGR format for OpenCV)
CATEGORY_COLORS = {
    "human": (0, 0, 255),       # Red - DANGER
    "animal": (0, 165, 255),    # Orange - CAUTION
    "vehicle": (255, 0, 0),     # Blue - OBSTACLE
    "static_obstacle": (0, 255, 255),  # Yellow
    "unknown": (128, 128, 128), # Gray
}


# =============================================================================
# YOLO Output Decoding
# =============================================================================

# Debug flag - set to True to see raw NN output info (press 'x' to toggle)
# Using a list to allow modification from inside functions
DEBUG_FLAGS = {"yolo": False}

# Display scale - increase to make window bigger (1.0 = original, 2.0 = double)
DISPLAY_SCALE = 1.5

# Letterboxing compensation
# Use generic math to handle any NN input size vs preview size.


def parse_img_detections(img_detections) -> list:
    """Parse ImgDetections from YoloDetectionNetwork into Detection2D list.
    
    YoloDetectionNetwork outputs dai.ImgDetections with on-device decoded boxes.
    Each detection has: .label, .confidence, .xmin, .ymin, .xmax, .ymax
    """
    from ai_camera_vision.detections import Detection2D, get_safety_category
    
    detections = []
    for det in img_detections.detections:
        class_id = det.label
        class_label = COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else f"class_{class_id}"
        safety_category = SAFETY_CATEGORIES.get(class_label, "unknown")
        
        detections.append(Detection2D(
            class_id=class_id,
            class_label=class_label,
            confidence=det.confidence,
            x_min=det.xmin,
            y_min=det.ymin,
            x_max=det.xmax,
            y_max=det.ymax,
            safety_category=safety_category,
        ))
    return detections


def adjust_bbox_from_letterbox(det: Detection2D) -> Detection2D:
    """Adjust detection bbox from letterboxed NN space to preview space."""
    # bbox = det.bbox  # REMOVED
    nn_w, nn_h = YOLO_INPUT_SIZE
    preview_w, preview_h = CAM_PREVIEW_SIZE
    
    # Calculate letterbox scaling factor and padding
    scale = min(nn_w / preview_w, nn_h / preview_h)
    padded_w = preview_w * scale
    padded_h = preview_h * scale
    
    offset_x = (nn_w - padded_w) / 2 / nn_w
    offset_y = (nn_h - padded_h) / 2 / nn_h
    
    scale_x = nn_w / padded_w
    scale_y = nn_h / padded_h
    
    # Adjust bbox coordinates using direct attributes
    x_min_adj = (det.x_min - offset_x) * scale_x
    y_min_adj = (det.y_min - offset_y) * scale_y
    x_max_adj = (det.x_max - offset_x) * scale_x
    y_max_adj = (det.y_max - offset_y) * scale_y
    
    # Clamp to [0, 1]
    x_min_adj = max(0.0, min(1.0, x_min_adj))
    y_min_adj = max(0.0, min(1.0, y_min_adj))
    x_max_adj = max(0.0, min(1.0, x_max_adj))
    y_max_adj = max(0.0, min(1.0, y_max_adj))
    
    # Create new Detection2D with adjusted coordinates
    return Detection2D(
        class_id=det.class_id,
        class_label=det.class_label,
        confidence=det.confidence,
        x_min=x_min_adj,
        y_min=y_min_adj,
        x_max=x_max_adj,
        y_max=y_max_adj,
        safety_category=det.safety_category,
    )





# =============================================================================
# Terminal Output - What Raspberry Pi Receives
# =============================================================================

def print_ros2_topic_data(
    detections: list,
    depth_frame: np.ndarray | None,
    seg_mask: np.ndarray | None,
    timestamp_ns: int,
    sequence_num: int,
    fps_data: dict,
):
    """Print what would be published to ROS2 topics.
    
    This shows exactly what data the Raspberry Pi mower brain receives.
    """
    print("\n" + "=" * 80)
    print(f"  📡 ROS2 TOPIC DATA — What the Raspberry Pi Receives")
    print(f"  Timestamp: {timestamp_ns / 1e9:.3f}s | Sequence: {sequence_num}")
    print("=" * 80)
    
    # ─────────────────────────────────────────────────────────────────────────
    # /vision/detections (Detection2DArray)
    # ─────────────────────────────────────────────────────────────────────────
    print("\n┌─ /vision/detections (Detection2DArray)")
    print("│")
    if detections:
        for i, det in enumerate(detections):
            # bbox = det.bbox  # REMOVED
            print(f"│  [{i}] {det.class_label}")
            print(f"│      confidence: {det.confidence:.1%}")
            print(f"│      bbox: x=[{det.x_min:.3f}, {det.x_max:.3f}] y=[{det.y_min:.3f}, {det.y_max:.3f}]")
            print(f"│      safety: {det.safety_category}")
    else:
        print("│  (no detections)")
    print("│")
    
    # ─────────────────────────────────────────────────────────────────────────
    # /vision/detections_3d (Detection3DArray) - with distance
    # ─────────────────────────────────────────────────────────────────────────
    print("├─ /vision/detections_3d (Detection3DArray)")
    print("│")
    if detections and depth_frame is not None:
        dh, dw = depth_frame.shape[:2]
        for i, det in enumerate(detections):
            # bbox = det.bbox  # REMOVED
            # Use robust multi-point depth sampling
            bbox_obj = BoundingBox2D(det.x_min, det.y_min, det.x_max, det.y_max)
            
            depth_mm, sample_conf = sample_depth_robust(
                depth_frame, bbox_obj, frame_shape=(dh, dw), sample_count=5
            )
            depth_m = depth_mm / 1000.0

            
            
            # Compute 3D position
            # Construct BoundingBox2D on the fly
            bbox_obj = BoundingBox2D(det.x_min, det.y_min, det.x_max, det.y_max)
            
            det_3d = compute_detection_3d(
                class_id=det.class_id,
                class_label=det.class_label,
                confidence=det.confidence,
                bbox=bbox_obj,
                depth_mm=depth_mm,
                frame_width=dw,
                frame_height=dh,
            )
            
            status = "✓ VALID" if det_3d.depth_valid else "✗ INVALID"
            print(f"│  [{i}] {det.class_label} — Distance: {depth_m:.2f}m {status}")
            print(f"│      position: x={det_3d.position_x:+.2f}m, y={det_3d.position_y:+.2f}m, z={det_3d.position_z:.2f}m")
            if det_3d.size_x and det_3d.size_y:
                print(f"│      size: {det_3d.size_x:.2f}m × {det_3d.size_y:.2f}m")
            print(f"│      depth_confidence: {det_3d.depth_confidence:.1%}")
    elif depth_frame is None:
        print("│  (depth data unavailable)")
    else:
        print("│  (no detections)")
    print("│")
    
    # ─────────────────────────────────────────────────────────────────────────
    # /vision/segmentation/zone_summary (Float32MultiArray)
    # ─────────────────────────────────────────────────────────────────────────
    print("├─ /vision/segmentation/zone_summary (Float32MultiArray)")
    print("│")
    if seg_mask is not None:
        # GREEN: background (0)
        green_pixels = np.sum(seg_mask == 0)
        
        # RED: everything else (objects)
        red_pixels = seg_mask.size - green_pixels
        
        # YELLOW: Not used in PASCAL VOC (no road class)
        yellow_pixels = 0
        
        total_counted = seg_mask.size
        
        if total_counted > 0:
            green_ratio = green_pixels / total_counted
            yellow_ratio = yellow_pixels / total_counted
            red_ratio = red_pixels / total_counted
        else:
            green_ratio = yellow_ratio = red_ratio = 0.0
        
        print(f"│  [green_ratio, yellow_ratio, red_ratio]")
        print(f"│  [{green_ratio:.3f}, {yellow_ratio:.3f}, {red_ratio:.3f}]")
    else:
        print("│  (segmentation data unavailable)")
    print("│")
    
    # ─────────────────────────────────────────────────────────────────────────
    # /vision/status (Health monitoring)
    # ─────────────────────────────────────────────────────────────────────────
    print("├─ /vision/status (JSON String)")
    print("│")
    status_data = {
        "timestamp_ns": timestamp_ns,
        "status": "OK" if fps_data.get("det_fps", 0) > 5 else "WARN",
        "detection_fps": round(fps_data.get("det_fps", 0), 1),
        "segmentation_fps": round(fps_data.get("seg_fps", 0), 1),
        "depth_fps": round(fps_data.get("depth_fps", 0), 1),
        "detection_count": len(detections),
        "depth_available": depth_frame is not None,
    }
    print(f"│  {json.dumps(status_data, indent=2).replace(chr(10), chr(10) + '│  ')}")
    print("│")
    print("└" + "─" * 79)


# =============================================================================
# Visualization Functions
# =============================================================================

def draw_detections(frame, detections, depth_frame=None):
    """Draw detection boxes with labels and distances on frame."""
    h, w = frame.shape[:2]
    
    for det in detections:
        # bbox = det.bbox  # REMOVED
        class_label = det.class_label
        confidence = det.confidence
        
        # Detection2D objects have a to_pixels method
        x1, y1, x2, y2 = det.to_pixels(w, h)
        
        category = det.safety_category
        color = CATEGORY_COLORS.get(category, (128, 128, 128))
        
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        
        label = f"{class_label}: {confidence:.0%}"
        
        if depth_frame is not None:
            try:
                dh, dw = depth_frame.shape[:2]
                # cx, cy = bbox.center_pixels(dw, dh) # REMOVED
                cx = int(det.center_x * dw)
                cy = int(det.center_y * dh)
                
                # Check bounds
                cx = max(0, min(dw - 1, cx))
                cy = max(0, min(dh - 1, cy))
                
                depth_mm = int(depth_frame[cy, cx])
                if depth_mm > 0:
                    distance_m = depth_mm / 1000.0
                    label += f" [{distance_m:.2f}m]"
            except:
                pass
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, thickness)
        
        label_y = y1 - 5 if y1 > 25 else y1 + text_h + 5
        
        cv2.rectangle(frame, (x1, label_y - text_h - 5), (x1 + text_w + 4, label_y + 2), color, -1)
        cv2.putText(frame, label, (x1 + 2, label_y - 2), font, font_scale, (255, 255, 255), thickness)
    
    return frame


def create_depth_colormap(depth_frame, max_depth_mm=5000):
    """Create a colorized depth visualization."""
    depth_normalized = np.clip(depth_frame.astype(np.float32) / max_depth_mm * 255, 0, 255).astype(np.uint8)
    depth_colored = cv2.applyColorMap(255 - depth_normalized, cv2.COLORMAP_JET)
    depth_colored[depth_frame == 0] = [0, 0, 0]
    return depth_colored



# PASCAL VOC Class Names (21 classes)
# Used by deeplab_v3_mnv2_256x256
PASCAL_VOC_CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
    "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike",
    "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

# Zone colors (BGR)
ZONE_COLORS = {
    "green": (0, 200, 0),     # Safe (background)
    "red": (0, 0, 255),       # Danger (objects)
}

# Map PASCAL classes to zones
# 0 (background) -> green (safe-ish, assumes lawn)
# 1-20 (objects) -> red (obstacles)
PASCAL_CLASS_TO_ZONE = {
    0: "green",
}
for i in range(1, 21):
    PASCAL_CLASS_TO_ZONE[i] = "red"


def create_segmentation_overlay(frame, seg_mask, alpha=0.4):
    """Overlay segmentation mask on RGB frame using ADAS class colors."""
    if seg_mask is None:
        return frame
    
    h, w = frame.shape[:2]
    
    try:
        mask_resized = cv2.resize(seg_mask.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
    except Exception:
        return frame
    
    # Create colored overlay using ADAS zone colors
    overlay = np.zeros_like(frame)
    
    for class_id, zone in PASCAL_CLASS_TO_ZONE.items():
        class_mask = mask_resized == class_id
        if np.any(class_mask):
            color = ZONE_COLORS.get(zone, (128, 128, 128))
            overlay[class_mask] = color
    
    # Check if there's any segmentation to show
    has_segmentation = mask_resized > 0
    if not np.any(has_segmentation):
        return frame
    
    # Blend using addWeighted on full frame
    result = cv2.addWeighted(frame, 1.0, overlay, alpha, 0)
    
    return result


def draw_info_panel(frame, info_dict, position="top-left"):
    """Draw an info panel with statistics."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 1
    line_height = 22
    padding = 10
    
    max_width = 0
    for label, value in info_dict.items():
        text = f"{label}: {value}"
        (text_w, _), _ = cv2.getTextSize(text, font, font_scale, thickness)
        max_width = max(max_width, text_w)
    
    panel_w = max_width + padding * 2
    panel_h = len(info_dict) * line_height + padding * 2
    
    h, w = frame.shape[:2]
    if "left" in position:
        x = 5
    else:
        x = w - panel_w - 5
    if "top" in position:
        y = 5
    else:
        y = h - panel_h - 5
    
    cv2.rectangle(frame, (x, y), (x + panel_w, y + panel_h), (0, 0, 0), -1)
    cv2.rectangle(frame, (x, y), (x + panel_w, y + panel_h), (0, 255, 0), 1)
    
    text_y = y + padding + 15
    for label, value in info_dict.items():
        text = f"{label}: {value}"
        cv2.putText(frame, text, (x + padding, text_y), font, font_scale, (255, 255, 255), thickness)
        text_y += line_height


# HUD removed



def draw_zone_legend(frame, position="bottom-left"):
    """Draw a compact zone color legend."""
    h, w = frame.shape[:2]
    
    legend_items = [
        ("GREEN", (0, 200, 0), "Safe (Background)"),
        ("RED", (0, 0, 255), "Obstacle (Object)"),
    ]
    
    box_size = 12
    padding = 8
    item_h = 18
    legend_h = len(legend_items) * item_h + padding * 2
    legend_w = 85
    
    x = 10 if "left" in position else w - legend_w - 10
    y = h - legend_h - 10 if "bottom" in position else 10
    
    # Background
    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (x + legend_w, y + legend_h), (20, 20, 30), -1)
    cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)
    
    # Items
    text_y = y + padding + 12
    for label, color, hint in legend_items:
        cv2.rectangle(frame, (x + padding, text_y - 10), 
                      (x + padding + box_size, text_y - 10 + box_size), color, -1)
        cv2.putText(frame, hint, (x + padding + box_size + 5, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1, cv2.LINE_AA)
        text_y += item_h
    
    return frame


def draw_help(frame):
    """Draw help text on frame."""
    h, w = frame.shape[:2]
    help_text = [
        "Controls:",
        "  q - Quit",
        "  t - Terminal data (Pi)",
        "  x - Debug NN output",
        "  d - Depth view",
        "  s - Segmentation",
        "  b - Detection boxes",
        "  1 - RGB only",
        "  2 - Depth only",
        "  3 - Side by side",
        "  h - Hide this help",
    ]
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 1
    line_height = 20
    
    panel_h = len(help_text) * line_height + 20
    panel_w = 200
    cv2.rectangle(frame, (w//2 - panel_w//2, h//2 - panel_h//2), 
                  (w//2 + panel_w//2, h//2 + panel_h//2), (0, 0, 0), -1)
    cv2.rectangle(frame, (w//2 - panel_w//2, h//2 - panel_h//2), 
                  (w//2 + panel_w//2, h//2 + panel_h//2), (0, 255, 0), 2)
    
    y = h//2 - panel_h//2 + 22
    for line in help_text:
        cv2.putText(frame, line, (w//2 - panel_w//2 + 10, y), font, font_scale, (255, 255, 255), thickness)
        y += line_height


# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 80)
    print("  AI Camera Vision - Live Demo")
    print("  Shows what data the Raspberry Pi mower brain receives")
    print("=" * 80)
    print()
    print("Controls:")
    print("  q - Quit")
    print("  t - Toggle terminal data output (what Pi receives)")
    print("  d - Toggle depth colormap")
    print("  s - Toggle segmentation overlay")
    print("  b - Toggle detection boxes")
    print("  h - Show/hide help")
    print()

    # Paths to models (inside models/ subdirectory)
    models_dir = Path(__file__).parent / "models"
    yolo_blob = models_dir / "yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"
    # NOTE: ADAS model (2048x1024) is too large for OAK-D Lite (needs ~200MB, only 15MB available)
    # Using person segmentation for now - TODO: recompile ADAS at lower resolution
    seg_blob = models_dir / "deeplab_v3_mnv2_256x256.blob"


    if not yolo_blob.exists() or not seg_blob.exists():
        print("ERROR: Model blobs not found in models/")
        return 1

    print(f"YOLO model:         {yolo_blob.name}")
    print(f"Segmentation model: {seg_blob.name}")
    print()
    print("Starting pipeline (USB2 mode for stability)...")
    print()

    # Create pipeline
    pipeline = create_pipeline(
        yolo_blob, seg_blob, enable_rgb_preview=True, enable_depth=True
    )

    # Visualization toggles
    show_boxes = True
    show_depth = True
    show_segmentation = True
    show_help = True
    show_terminal_data = False  # Toggle with 't' key
    view_mode = 3  # 1=RGB, 2=Depth, 3=Side-by-side

    # Stats tracking
    det_count = 0
    seg_count = 0
    depth_count = 0
    frame_count = 0
    start_time = time.time()
    last_terminal_print = 0
    
    # Current data
    last_detections = []
    last_seg_mask = None
    last_depth_frame = None
    last_timestamp_ns = 0
    last_sequence_num = 0

    # NOTE: Using USB2 mode to prevent camera firmware crashes on Windows
    with PipelineRunner(pipeline, force_usb2=True) as runner:
        print("Pipeline running! Press 't' to see data sent to Pi, 'q' to quit.")
        print("-" * 80)

        while True:
            # Get RGB preview frame
            rgb_result = runner.get_rgb_preview_result(timeout_ms=100)

            if rgb_result is not None:
                frame_count += 1
                frame = rgb_result.raw_frame.getCvFrame()
                last_timestamp_ns = rgb_result.timestamp_ns
                last_sequence_num = rgb_result.sequence_num

                # Get detection result (non-blocking)
                # Now using YoloDetectionNetwork which outputs ImgDetections (on-device decoded)
                det_result = runner.get_detection_result(timeout_ms=0)
                if det_result is not None:
                    det_count += 1
                    # Parse ImgDetections from on-device YOLO decoder
                    raw_detections = parse_img_detections(det_result.raw_data)
                    # Adjust bbox coordinates from letterboxed NN space to preview space
                    last_detections = [adjust_bbox_from_letterbox(d) for d in raw_detections]

                # Get segmentation result (non-blocking)
                seg_result = runner.get_segmentation_result(timeout_ms=0)
                if seg_result is not None:
                    seg_count += 1
                    try:
                        # Try different methods to get the segmentation data
                        seg_data = None
                        layer_names = seg_result.raw_data.getAllLayerNames()
                        
                        if DEBUG_FLAGS["yolo"]:  # Reuse debug flag for segmentation too
                            print(f"[DEBUG SEG] Layers: {layer_names}")
                        
                        for layer_name in layer_names:
                            # Try FP16 first (common for DeepLabV3)
                            try:
                                seg_data = seg_result.raw_data.getLayerFp16(layer_name)
                            except:
                                pass
                            
                            if seg_data is None or len(seg_data) == 0:
                                # Try Int32
                                try:
                                    seg_data = seg_result.raw_data.getLayerInt32(layer_name)
                                except:
                                    pass
                            
                            if seg_data is not None and len(seg_data) > 0:
                                seg_array = np.array(seg_data)
                                
                                if DEBUG_FLAGS["yolo"]:
                                    print(f"[DEBUG SEG] {layer_name}: len={len(seg_array)}, "
                                          f"min={seg_array.min()}, max={seg_array.max()}")
                                
                                # DeepLabV3+ outputs: (1, num_classes, H, W) or (H, W)
                                # For 256x256 input: 65536 pixels
                                if len(seg_array) == 256 * 256:
                                    # Direct class IDs
                                    last_seg_mask = seg_array.reshape(256, 256).astype(np.uint8)
                                    break
                                elif len(seg_array) == 21 * 256 * 256:
                                    # Softmax output: (21 classes, 256, 256) for PASCAL VOC
                                    seg_array = seg_array.reshape(21, 256, 256)
                                    last_seg_mask = np.argmax(seg_array, axis=0).astype(np.uint8)
                                    break
                                elif len(seg_array) % (256 * 256) == 0:
                                    # Some other multi-channel format
                                    num_channels = len(seg_array) // (256 * 256)
                                    seg_array = seg_array.reshape(num_channels, 256, 256)
                                    last_seg_mask = np.argmax(seg_array, axis=0).astype(np.uint8)
                                    break
                    except Exception as e:
                        if DEBUG_FLAGS["yolo"]:
                            print(f"[DEBUG SEG] Error: {e}")

                # Get depth result (non-blocking)
                depth_result = runner.get_depth_result(timeout_ms=0)
                if depth_result is not None:
                    depth_count += 1
                    last_depth_frame = depth_result.raw_frame.getFrame()

                # Calculate FPS
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0
                det_fps = det_count / elapsed if elapsed > 0 else 0
                seg_fps = seg_count / elapsed if elapsed > 0 else 0
                depth_fps = depth_count / elapsed if elapsed > 0 else 0

                # === TERMINAL OUTPUT (What Pi receives) ===
                if show_terminal_data and time.time() - last_terminal_print > 1.0:
                    last_terminal_print = time.time()
                    print_ros2_topic_data(
                        detections=last_detections,
                        depth_frame=last_depth_frame,
                        seg_mask=last_seg_mask,
                        timestamp_ns=last_timestamp_ns,
                        sequence_num=last_sequence_num,
                        fps_data={
                            "det_fps": det_fps,
                            "seg_fps": seg_fps,
                            "depth_fps": depth_fps,
                        },
                    )

                # === BUILD VISUALIZATION ===
                display_frame = frame.copy()
                
                if show_segmentation and last_seg_mask is not None:
                    display_frame = create_segmentation_overlay(display_frame, last_seg_mask)
                
                if show_boxes and last_detections:
                    draw_detections(display_frame, last_detections, last_depth_frame)
                
                # Calculate zone ratios from segmentation mask for HUD
                zone_ratios = {"green": 0.0, "yellow": 0.0, "red": 0.0}
                if last_seg_mask is not None:
                    total_pixels = last_seg_mask.size
                    if total_pixels > 0:
                        # Count pixels per ADAS zone (excluding ignored: sky=10, ego=19)
                        green_pixels = np.sum((last_seg_mask == 8) | (last_seg_mask == 9))
                        yellow_pixels = np.sum((last_seg_mask == 0) | (last_seg_mask == 1))
                        red_mask = (
                            (last_seg_mask == 11) | (last_seg_mask == 12) |  # person, rider
                            np.isin(last_seg_mask, [2, 3, 4, 5, 6, 7, 13, 14, 15, 16, 17, 18])  # obstacles, vehicles
                        )
                        red_pixels = np.sum(red_mask)
                        ignored_pixels = np.sum((last_seg_mask == 10) | (last_seg_mask == 19))
                        
                        total_valid = total_pixels - ignored_pixels
                        if total_valid > 0:
                            zone_ratios["green"] = green_pixels / total_valid
                            zone_ratios["yellow"] = yellow_pixels / total_valid
                            zone_ratios["red"] = red_pixels / total_valid
                
                # Detection labels for HUD
                det_labels = [d.class_label for d in last_detections] if last_detections else []
                
                # Premium HUD overlay (REMOVED)
                # draw_premium_hud(display_frame, fps, len(last_detections), zone_ratios, det_labels)
                
                # Zone legend in bottom-left corner
                if show_segmentation:
                    draw_zone_legend(display_frame, "bottom-left")

                
                # Depth visualization
                if show_depth and last_depth_frame is not None:
                    depth_viz = create_depth_colormap(last_depth_frame)
                    depth_viz = cv2.resize(depth_viz, (frame.shape[1], frame.shape[0]))
                    cv2.putText(depth_viz, "CLOSE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.putText(depth_viz, "FAR", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                
                # Combine views
                if view_mode == 1 or last_depth_frame is None:
                    final_display = display_frame
                elif view_mode == 2 and last_depth_frame is not None:
                    final_display = depth_viz
                else:
                    if last_depth_frame is not None and show_depth:
                        final_display = np.hstack([display_frame, depth_viz])
                    else:
                        final_display = display_frame
                
                if show_help:
                    draw_help(final_display)
                
                # Scale up for bigger display
                if DISPLAY_SCALE != 1.0:
                    h, w = final_display.shape[:2]
                    new_size = (int(w * DISPLAY_SCALE), int(h * DISPLAY_SCALE))
                    final_display = cv2.resize(final_display, new_size)
                
                cv2.imshow("AI Camera Vision - Live Demo", final_display)

                # Simple terminal stats (when terminal data is off)
                if not show_terminal_data and frame_count % 60 == 0:
                    det_names = [d.class_label for d in last_detections[:3]]
                    det_str = ", ".join(det_names) if det_names else "none"
                    print(f"[{elapsed:5.0f}s] FPS: {fps:.1f} | Detections: {det_str}")

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("\nQuitting...")
                break
            elif key == ord("t"):
                show_terminal_data = not show_terminal_data
                if show_terminal_data:
                    print("\n" + "=" * 80)
                    print("  TERMINAL DATA OUTPUT: ON")
                    print("  Showing what data the Raspberry Pi mower brain receives...")
                    print("=" * 80)
                else:
                    print("\n  TERMINAL DATA OUTPUT: OFF")
            elif key == ord("d"):
                show_depth = not show_depth
                print(f"Depth view: {'ON' if show_depth else 'OFF'}")
            elif key == ord("s"):
                show_segmentation = not show_segmentation
                print(f"Segmentation overlay: {'ON' if show_segmentation else 'OFF'}")
            elif key == ord("b"):
                show_boxes = not show_boxes
                print(f"Detection boxes: {'ON' if show_boxes else 'OFF'}")
            elif key == ord("h"):
                show_help = not show_help
            elif key == ord("1"):
                view_mode = 1
                print("View mode: RGB only")
            elif key == ord("2"):
                view_mode = 2
                print("View mode: Depth only")
            elif key == ord("3"):
                view_mode = 3
                print("View mode: Side by side")
            elif key == ord("x"):
                DEBUG_FLAGS["yolo"] = not DEBUG_FLAGS["yolo"]
                status = "ON" if DEBUG_FLAGS["yolo"] else "OFF"
                print(f"\n[DEBUG MODE: {status}]")
                if DEBUG_FLAGS["yolo"]:
                    print("Will show raw NN output info for next frames...")

    cv2.destroyAllWindows()

    # Final stats
    elapsed = time.time() - start_time
    print()
    print("=" * 80)
    print("  Session Summary")
    print("=" * 80)
    print(f"  Duration:          {elapsed:.1f} seconds")
    print(f"  Preview frames:    {frame_count} ({frame_count/elapsed:.1f} FPS)")
    print(f"  Detection frames:  {det_count} ({det_count/elapsed:.1f} FPS)")
    print(f"  Segmentation:      {seg_count} ({seg_count/elapsed:.1f} FPS)")
    print(f"  Depth frames:      {depth_count} ({depth_count/elapsed:.1f} FPS)")
    print("=" * 80)

    return 0


if __name__ == "__main__":
    sys.exit(main())
