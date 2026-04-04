#!/usr/bin/env python3
"""Headless demo for Raspberry Pi - prints ROS2 topic data to terminal."""

import sys
import time
import json
from pathlib import Path

import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from ai_camera_vision.pipeline import create_pipeline, PipelineRunner
from ai_camera_vision.depth import BoundingBox2D, compute_detection_3d, sample_depth_robust
from ai_camera_vision.detections import Detection2D

# Model paths (relative to this script)
SCRIPT_DIR = Path(__file__).parent
YOLO_BLOB = SCRIPT_DIR / "models" / "yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"
SEG_BLOB = SCRIPT_DIR / "models" / "deeplab_v3_mnv2_256x256.blob"

# COCO class names for YOLO
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

SAFETY_CATEGORIES = {
    "person": "human",
    "dog": "animal", "cat": "animal", "bird": "animal", "horse": "animal",
    "cow": "animal", "sheep": "animal", "elephant": "animal", "bear": "animal",
    "zebra": "animal", "giraffe": "animal",
    "car": "vehicle", "truck": "vehicle", "bus": "vehicle", "motorcycle": "vehicle",
    "bicycle": "vehicle", "train": "vehicle", "airplane": "vehicle", "boat": "vehicle",
}


def print_ros2_topic_data(
    detections: list,
    depth_frame,
    seg_mask,
    timestamp_ns: int,
    sequence_num: int,
    fps_data: dict,
):
    """Print what would be published to ROS2 topics."""
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
            bbox_obj = BoundingBox2D(det.x_min, det.y_min, det.x_max, det.y_max)
            depth_mm, sample_conf = sample_depth_robust(
                depth_frame, bbox_obj, frame_shape=(dh, dw), sample_count=5
            )
            depth_m = depth_mm / 1000.0
            
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
        green_pixels = np.sum(seg_mask == 0)
        red_pixels = seg_mask.size - green_pixels
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


def parse_img_detections(img_detections) -> list:
    """Parse ImgDetections from YoloDetectionNetwork into Detection2D list."""
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


def main():
    print("=" * 60)
    print("AI Camera Vision - Headless Demo (Raspberry Pi)")
    print("=" * 60)
    
    # Check model files exist
    if not YOLO_BLOB.exists():
        print(f"ERROR: YOLO model not found: {YOLO_BLOB}")
        return
    if not SEG_BLOB.exists():
        print(f"ERROR: Segmentation model not found: {SEG_BLOB}")
        return
    
    print(f"YOLO model: {YOLO_BLOB}")
    print(f"Seg model:  {SEG_BLOB}")
    print("-" * 60)
    
    # Create pipeline
    print("Creating pipeline...")
    pipeline = create_pipeline(
        str(YOLO_BLOB),
        str(SEG_BLOB),
        enable_depth=True,
    )
    
    print("Starting camera (USB3 mode for Pi)...")
    print("Press Ctrl+C to stop")
    print("-" * 60)
    print()
    
    try:
        with PipelineRunner(pipeline, force_usb2=False) as runner:
            det_count = 0
            seg_count = 0
            depth_count = 0
            start_time = time.time()
            last_print = time.time()
            
            # Current data
            last_detections = []
            last_seg_mask = None
            last_depth_frame = None
            last_timestamp_ns = 0
            last_sequence_num = 0
            
            print("Waiting for frames...")
            
            while True:
                # Get detection result
                det_result = runner.get_detection_result(timeout_ms=100)
                if det_result:
                    det_count += 1
                    last_timestamp_ns = det_result.timestamp_ns
                    last_sequence_num = det_result.sequence_num
                    last_detections = parse_img_detections(det_result.raw_data)
                
                # Get segmentation result
                seg_result = runner.get_segmentation_result(timeout_ms=0)
                if seg_result:
                    seg_count += 1
                    try:
                        layer_names = seg_result.raw_data.getAllLayerNames()
                        for layer_name in layer_names:
                            try:
                                seg_data = seg_result.raw_data.getLayerFp16(layer_name)
                            except:
                                seg_data = None
                            
                            if seg_data is None or len(seg_data) == 0:
                                try:
                                    seg_data = seg_result.raw_data.getLayerInt32(layer_name)
                                except:
                                    pass
                            
                            if seg_data is not None and len(seg_data) > 0:
                                seg_array = np.array(seg_data)
                                if len(seg_array) == 256 * 256:
                                    last_seg_mask = seg_array.reshape(256, 256).astype(np.uint8)
                                    break
                                elif len(seg_array) == 21 * 256 * 256:
                                    seg_array = seg_array.reshape(21, 256, 256)
                                    last_seg_mask = np.argmax(seg_array, axis=0).astype(np.uint8)
                                    break
                                elif len(seg_array) % (256 * 256) == 0:
                                    num_channels = len(seg_array) // (256 * 256)
                                    seg_array = seg_array.reshape(num_channels, 256, 256)
                                    last_seg_mask = np.argmax(seg_array, axis=0).astype(np.uint8)
                                    break
                    except Exception:
                        pass
                
                # Get depth result  
                depth_result = runner.get_depth_result(timeout_ms=0)
                if depth_result:
                    depth_count += 1
                    last_depth_frame = depth_result.raw_frame.getFrame()
                
                # Print ROS2 topic data every second
                now = time.time()
                if now - last_print > 1.0:
                    elapsed = now - start_time
                    fps_data = {
                        "det_fps": det_count / elapsed if elapsed > 0 else 0,
                        "seg_fps": seg_count / elapsed if elapsed > 0 else 0,
                        "depth_fps": depth_count / elapsed if elapsed > 0 else 0,
                    }
                    
                    print_ros2_topic_data(
                        detections=last_detections,
                        depth_frame=last_depth_frame,
                        seg_mask=last_seg_mask,
                        timestamp_ns=last_timestamp_ns,
                        sequence_num=last_sequence_num,
                        fps_data=fps_data,
                    )
                    last_print = now
                    
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
