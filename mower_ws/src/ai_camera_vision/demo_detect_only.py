#!/usr/bin/env python3
"""Detection-only headless demo for Raspberry Pi.

Runs ONLY YOLO object detection (no segmentation, no depth).
Lighter pipeline = less power draw = no brownout crashes.

Run on Pi:
    cd ~/mower_ws/src/ai_camera_vision
    source ~/ai_camera_vision/venv/bin/activate
    python3 demo_detect_only.py
"""

import sys
import time
import json
from pathlib import Path

import numpy as np
import depthai as dai

# Model path
SCRIPT_DIR = Path(__file__).parent
YOLO_BLOB = SCRIPT_DIR / "models" / "yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"

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


def main():
    print("=" * 60)
    print("  AI Camera Vision — Detection Only (Headless)")
    print("=" * 60)

    # Check model file
    if not YOLO_BLOB.exists():
        print(f"ERROR: YOLO model not found: {YOLO_BLOB}")
        return 1

    print(f"YOLO model: {YOLO_BLOB.name}")
    print("-" * 60)

    # ── Build a minimal pipeline: Camera → YOLO only ──
    print("Building detection-only pipeline...")
    pipeline = dai.Pipeline()

    # RGB Camera
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setFps(15)
    cam.setPreviewSize(416, 416)  # Match YOLO input directly (no letterbox needed)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)

    # YOLO Detection Network
    nn = pipeline.create(dai.node.YoloDetectionNetwork)
    nn.setBlobPath(str(YOLO_BLOB))
    nn.setNumInferenceThreads(2)
    nn.input.setBlocking(False)
    nn.input.setQueueSize(1)

    # YOLO v3-tiny config
    nn.setNumClasses(80)
    nn.setCoordinateSize(4)
    nn.setAnchors([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319])
    nn.setAnchorMasks({"side26": [0, 1, 2], "side13": [3, 4, 5]})
    nn.setIouThreshold(0.5)
    nn.setConfidenceThreshold(0.4)  # Slightly lower threshold to catch more objects

    # Link camera → YOLO
    cam.preview.link(nn.input)

    # Output
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("detections")
    xout.input.setBlocking(False)
    xout.input.setQueueSize(1)
    nn.out.link(xout.input)

    print("Pipeline built ✓")
    print("Starting camera (USB2 mode)...")
    print("Press Ctrl+C to stop")
    print("-" * 60)

    try:
        with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
            print(f"Device: {device.getMxId()} | USB: {device.getUsbSpeed().name}")
            q = device.getOutputQueue("detections", maxSize=4, blocking=False)

            det_count = 0
            frame_count = 0
            start_time = time.time()
            last_print = time.time()

            print("\nWaiting for detections...\n")

            while True:
                img_dets = q.tryGet()

                if img_dets is not None:
                    frame_count += 1
                    detections = img_dets.detections

                    # Print every second
                    now = time.time()
                    if now - last_print > 1.0:
                        elapsed = now - start_time
                        fps = frame_count / elapsed if elapsed > 0 else 0

                        print("=" * 60)
                        print(f"  Frame #{frame_count} | FPS: {fps:.1f} | "
                              f"Elapsed: {elapsed:.0f}s")
                        print("=" * 60)

                        if detections:
                            for i, det in enumerate(detections):
                                class_id = det.label
                                label = COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else f"class_{class_id}"
                                safety = SAFETY_CATEGORIES.get(label, "unknown")
                                conf = det.confidence

                                print(f"  [{i}] {label}")
                                print(f"      confidence: {conf:.1%}")
                                print(f"      bbox: x=[{det.xmin:.3f}, {det.xmax:.3f}] "
                                      f"y=[{det.ymin:.3f}, {det.ymax:.3f}]")
                                print(f"      safety: {safety}")
                        else:
                            print("  (no objects detected)")

                        print()
                        last_print = now
                else:
                    time.sleep(0.01)  # Small sleep when no data

    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        print(f"\n\nStopped. {frame_count} frames in {elapsed:.1f}s "
              f"({frame_count/elapsed:.1f} FPS)" if elapsed > 0 else "")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
