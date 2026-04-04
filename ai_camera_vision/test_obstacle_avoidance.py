#!/usr/bin/env python3
"""Headless Obstacle Avoidance Test for Raspberry Pi.

This script runs on the Pi with USB3 for higher FPS.
No OpenCV visualization - just prints obstacle data and avoidance actions to terminal.

Usage:
    python3 test_obstacle_avoidance.py

Expected output:
    - Obstacle detections with distances
    - Avoidance actions (CLEAR, TURN_LEFT, TURN_RIGHT, SLOW_DOWN, STOP)
    - Real-time FPS
"""

import sys
import time
import math
from pathlib import Path
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

import numpy as np
import depthai as dai

# =============================================================================
# Path setup - import from ai_camera_vision package
# =============================================================================
# This script is in ai_camera_vision folder
SCRIPT_DIR = Path(__file__).parent
sys.path.insert(0, str(SCRIPT_DIR))

from ai_camera_vision.depth import BoundingBox2D, sample_depth_robust
from ai_camera_vision.detections import Detection2D

# Model paths
MODELS_DIR = SCRIPT_DIR / "models"
YOLO_BLOB = MODELS_DIR / "yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"


# =============================================================================
# Obstacle Avoidance Classes
# =============================================================================

class AvoidanceAction(Enum):
    """Actions to take for obstacle avoidance."""
    NONE = "CLEAR"
    SLOW_DOWN = "SLOW_DOWN"
    STOP = "STOP"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"


@dataclass
class Obstacle:
    """Detected obstacle information."""
    type: str
    distance: float          # meters
    angle: float             # radians from center (0 = straight ahead)
    width: float = 0.3       # estimated width in meters
    confidence: float = 0.5
    
    @property
    def is_in_path(self) -> bool:
        """Check if obstacle is in robot's path."""
        robot_half_width = 0.25  # meters
        lateral_distance = abs(self.distance * math.sin(self.angle))
        return lateral_distance < (robot_half_width + self.width / 2)


class ObstacleHandler:
    """Simplified obstacle handler."""
    
    def __init__(self):
        self.critical_distance = 0.5   # meters - stop
        self.warning_distance = 1.2    # meters - slow/turn
        self.obstacles: List[Obstacle] = []
    
    def update_obstacles(self, obstacles: List[Obstacle]) -> None:
        self.obstacles = obstacles
    
    def get_closest_in_path(self) -> Optional[Obstacle]:
        in_path = [o for o in self.obstacles if o.is_in_path]
        if not in_path:
            return None
        return min(in_path, key=lambda o: o.distance)
    
    def get_avoidance_action(self) -> AvoidanceAction:
        closest = self.get_closest_in_path()
        if closest is None:
            return AvoidanceAction.NONE
        
        if closest.distance < self.critical_distance:
            return AvoidanceAction.STOP
        
        if closest.distance < self.warning_distance:
            if closest.angle > 0.15:
                return AvoidanceAction.TURN_LEFT
            elif closest.angle < -0.15:
                return AvoidanceAction.TURN_RIGHT
            else:
                return AvoidanceAction.SLOW_DOWN
        
        return AvoidanceAction.NONE
    
    def get_speed_factor(self) -> float:
        closest = self.get_closest_in_path()
        if closest is None:
            return 1.0
        if closest.distance < self.critical_distance:
            return 0.0
        if closest.distance < self.warning_distance:
            ratio = (closest.distance - self.critical_distance) / \
                    (self.warning_distance - self.critical_distance)
            return max(0.3, ratio)
        return 1.0


# =============================================================================
# COCO Class Names
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

SAFETY_CATEGORIES = {
    "person": "human", "dog": "animal", "cat": "animal", "bird": "animal",
    "horse": "animal", "cow": "animal", "sheep": "animal",
    "car": "vehicle", "truck": "vehicle", "bus": "vehicle", 
    "motorcycle": "vehicle", "bicycle": "vehicle",
}


# =============================================================================
# Pipeline Creation (YOLO + Depth only, no segmentation)
# =============================================================================

def create_fast_pipeline(yolo_blob_path: Path) -> dai.Pipeline:
    """Create a FAST pipeline with only YOLO detection + depth."""
    pipeline = dai.Pipeline()
    
    # Color Camera
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setFps(30)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setPreviewSize(640, 360)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    
    # YOLO Detection
    manip_yolo = pipeline.create(dai.node.ImageManip)
    manip_yolo.initialConfig.setResize(416, 416)
    manip_yolo.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
    manip_yolo.initialConfig.setKeepAspectRatio(True)
    manip_yolo.setMaxOutputFrameSize(416 * 416 * 3)
    
    nn_yolo = pipeline.create(dai.node.YoloDetectionNetwork)
    nn_yolo.setBlobPath(str(yolo_blob_path))
    nn_yolo.setNumInferenceThreads(2)
    nn_yolo.input.setBlocking(False)
    nn_yolo.input.setQueueSize(1)
    nn_yolo.setNumClasses(80)
    nn_yolo.setCoordinateSize(4)
    nn_yolo.setAnchors([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319])
    nn_yolo.setAnchorMasks({"side26": [0, 1, 2], "side13": [3, 4, 5]})
    nn_yolo.setIouThreshold(0.5)
    nn_yolo.setConfidenceThreshold(0.5)
    
    cam_rgb.preview.link(manip_yolo.inputImage)
    manip_yolo.out.link(nn_yolo.input)
    
    # Detections output
    xout_yolo = pipeline.create(dai.node.XLinkOut)
    xout_yolo.setStreamName("detections")
    xout_yolo.input.setBlocking(False)
    xout_yolo.input.setQueueSize(1)
    nn_yolo.out.link(xout_yolo.input)
    
    # Depth (stereo)
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_left.setFps(30)
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    
    mono_right = pipeline.create(dai.node.MonoCamera)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setFps(30)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.initialConfig.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_7x7)
    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    xout_depth.input.setBlocking(False)
    xout_depth.input.setQueueSize(1)
    stereo.depth.link(xout_depth.input)
    
    return pipeline


# =============================================================================
# Detection Parsing
# =============================================================================

def parse_img_detections(img_detections) -> list:
    """Parse ImgDetections from YoloDetectionNetwork."""
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


def detections_to_obstacles(detections: list, depth_frame) -> List[Obstacle]:
    """Convert camera detections to obstacles with depth."""
    obstacles = []
    
    if depth_frame is None:
        return obstacles
    
    dh, dw = depth_frame.shape[:2]
    
    for det in detections:
        bbox = BoundingBox2D(det.x_min, det.y_min, det.x_max, det.y_max)
        depth_mm, confidence = sample_depth_robust(
            depth_frame, bbox, frame_shape=(dh, dw), sample_count=5
        )
        
        if depth_mm <= 0:
            continue
        
        distance = depth_mm / 1000.0
        center_x = (det.x_min + det.x_max) / 2
        angle = (center_x - 0.5) * math.radians(60)
        width = (det.x_max - det.x_min) * distance
        
        obstacles.append(Obstacle(
            type=det.class_label,
            distance=distance,
            angle=angle,
            width=width,
            confidence=det.confidence,
        ))
    
    return obstacles


# =============================================================================
# Terminal Output
# =============================================================================

def print_status(
    action: AvoidanceAction,
    speed_factor: float,
    obstacles: List[Obstacle],
    closest: Optional[Obstacle],
    fps: float,
):
    """Print status to terminal."""
    # Clear previous output
    print("\033[2J\033[H", end="")  # Clear screen and move cursor to top
    
    print("=" * 60)
    print("  OBSTACLE AVOIDANCE TEST - Raspberry Pi (USB3)")
    print("=" * 60)
    print()
    
    # Avoidance action with color
    action_colors = {
        AvoidanceAction.NONE: "\033[92m",       # Green
        AvoidanceAction.SLOW_DOWN: "\033[93m",   # Yellow
        AvoidanceAction.TURN_LEFT: "\033[96m",   # Cyan
        AvoidanceAction.TURN_RIGHT: "\033[96m",  # Cyan
        AvoidanceAction.STOP: "\033[91m",        # Red
    }
    color = action_colors.get(action, "\033[0m")
    reset = "\033[0m"
    
    print(f"  ACTION: {color}>>> {action.value} <<<{reset}")
    print(f"  SPEED:  {int(speed_factor * 100)}%")
    print()
    
    # Closest obstacle
    if closest:
        side = "LEFT" if closest.angle < 0 else "RIGHT" if closest.angle > 0 else "CENTER"
        print(f"  Closest in path: {closest.type}")
        print(f"    Distance: {closest.distance:.2f}m")
        print(f"    Position: {side} ({abs(math.degrees(closest.angle)):.0f}°)")
    else:
        print("  \033[92mPath is CLEAR\033[0m")
    print()
    
    # All obstacles
    print(f"  Detected obstacles: {len(obstacles)}")
    for i, obs in enumerate(obstacles[:5]):  # Show max 5
        in_path = "⚠️" if obs.is_in_path else "  "
        print(f"    {in_path} [{i}] {obs.type}: {obs.distance:.2f}m")
    if len(obstacles) > 5:
        print(f"    ... and {len(obstacles) - 5} more")
    print()
    
    # FPS
    print(f"  FPS: {fps:.1f}")
    print()
    print("-" * 60)
    print("  Press Ctrl+C to stop")


# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 60)
    print("  OBSTACLE AVOIDANCE TEST")
    print("  For Raspberry Pi with USB3")
    print("=" * 60)
    print()
    
    # Check model exists
    if not YOLO_BLOB.exists():
        print(f"ERROR: YOLO model not found at: {YOLO_BLOB}")
        print("Make sure to copy the models/ folder to the Pi.")
        return 1
    
    print(f"YOLO model: {YOLO_BLOB.name}")
    print()
    print("Creating pipeline...")
    
    pipeline = create_fast_pipeline(YOLO_BLOB)
    
    # USB3 mode for Raspberry Pi (faster FPS)
    print("Starting camera (USB3 mode)...")
    print()
    
    handler = ObstacleHandler()
    last_depth_frame = None
    frame_count = 0
    start_time = time.time()
    
    try:
        # USB3 mode (no maxUsbSpeed specified = USB3)
        with dai.Device(pipeline) as device:
            print("Pipeline running!")
            print("Waiting for data...")
            print()
            
            q_det = device.getOutputQueue("detections", maxSize=4, blocking=False)
            q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)
            
            while True:
                # Get depth
                depth_data = q_depth.tryGet()
                if depth_data is not None:
                    last_depth_frame = depth_data.getFrame()
                
                # Get detections
                det_data = q_det.tryGet()
                if det_data is not None:
                    frame_count += 1
                    
                    raw_detections = parse_img_detections(det_data)
                    obstacles = detections_to_obstacles(raw_detections, last_depth_frame)
                    handler.update_obstacles(obstacles)
                    
                    action = handler.get_avoidance_action()
                    speed_factor = handler.get_speed_factor()
                    closest = handler.get_closest_in_path()
                    
                    # Calculate FPS
                    elapsed = time.time() - start_time
                    fps = frame_count / elapsed if elapsed > 0 else 0
                    
                    # Print status
                    print_status(action, speed_factor, obstacles, closest, fps)
                
                time.sleep(0.01)  # Small delay to reduce CPU usage
                
    except KeyboardInterrupt:
        print("\n\nStopping...")
    
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
