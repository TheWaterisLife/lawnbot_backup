"""Obstacle Avoidance Demo - Visual Arrow Display.

Shows real-time obstacle detection with directional arrows indicating 
where the mower should move to avoid obstacles.

Visual Feedback:
  - Large directional arrow showing avoidance direction
  - Action label: TURN_LEFT, TURN_RIGHT, SLOW_DOWN, STOP, CLEAR
  - Speed factor bar showing current speed reduction
  - Detection boxes with distance labels

Run with: py -3.11 demo_obstacle_avoidance.py

Controls:
  q - Quit
  h - Toggle help overlay
"""

import sys
import time
import math
from pathlib import Path
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

# =============================================================================
# Path setup - import from ai_camera_vision package
# =============================================================================
# This script is in mower_navigation, but needs ai_camera_vision modules
SCRIPT_DIR = Path(__file__).parent
PROJECT_ROOT = SCRIPT_DIR.parent  # Go up to src/
AI_CAMERA_DIR = PROJECT_ROOT / "ai_camera_vision"
sys.path.insert(0, str(AI_CAMERA_DIR))

# Import depthai BEFORE opencv to avoid Windows crash
from ai_camera_vision.pipeline import (
    CAM_PREVIEW_SIZE,
    YOLO_INPUT_SIZE,
)
from ai_camera_vision.depth import BoundingBox2D, sample_depth_robust
from ai_camera_vision.detections import Detection2D

# OpenCV must be imported AFTER depthai
import cv2
import numpy as np
import depthai as dai


def create_yolo_only_pipeline(
    yolo_blob_path: Path,
    *,
    enable_rgb_preview: bool = True,
    enable_depth: bool = True,
) -> dai.Pipeline:
    """Create a FAST pipeline with only YOLO detection (no segmentation).
    
    This is ~2x faster than the full pipeline since we skip segmentation.
    """
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
    
    # RGB preview
    if enable_rgb_preview:
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb_preview")
        xout_rgb.input.setBlocking(False)
        xout_rgb.input.setQueueSize(1)
        cam_rgb.preview.link(xout_rgb.input)
    
    # Depth (stereo)
    if enable_depth:
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setFps(30)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        
        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setFps(30)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
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
# Obstacle Handler Classes (simplified from mower_navigation)
# =============================================================================

class AvoidanceAction(Enum):
    """Actions to take for obstacle avoidance."""
    NONE = "CLEAR"
    SLOW_DOWN = "SLOW_DOWN"
    STOP = "STOP"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    PIVOT_LEFT = "PIVOT_LEFT"    # Tank turn in place
    PIVOT_RIGHT = "PIVOT_RIGHT"  # Tank turn in place
    PIVOT_180 = "PIVOT_180"      # Full 180° turn


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


class SimpleObstacleHandler:
    """Simplified obstacle handler for demo with recovery logic."""
    
    def __init__(self):
        self.critical_distance = 0.5   # meters - stop
        self.warning_distance = 1.2    # meters - slow/turn
        self.obstacles: List[Obstacle] = []
        
        # Recovery state
        self._stop_start_time: float = 0.0
        self._stop_timeout: float = 2.0  # seconds before pivot
        self._in_recovery: bool = False
        
        # Boundary constraints (for testing - can be toggled)
        self.left_boundary_blocked: bool = False
        self.right_boundary_blocked: bool = False
    
    def update_obstacles(self, obstacles: List[Obstacle]) -> None:
        self.obstacles = obstacles
    
    def get_closest_in_path(self) -> Optional[Obstacle]:
        in_path = [o for o in self.obstacles if o.is_in_path]
        if not in_path:
            return None
        return min(in_path, key=lambda o: o.distance)
    
    def _choose_pivot_direction(self, obstacle: Optional[Obstacle]) -> AvoidanceAction:
        """Choose pivot direction considering boundary constraints."""
        # Both blocked - 180
        if self.left_boundary_blocked and self.right_boundary_blocked:
            return AvoidanceAction.PIVOT_180
        
        # Left blocked - go right
        if self.left_boundary_blocked:
            return AvoidanceAction.PIVOT_RIGHT
        
        # Right blocked - go left
        if self.right_boundary_blocked:
            return AvoidanceAction.PIVOT_LEFT
        
        # Turn away from obstacle
        if obstacle is not None:
            if obstacle.angle > 0.1:
                return AvoidanceAction.PIVOT_LEFT
            elif obstacle.angle < -0.1:
                return AvoidanceAction.PIVOT_RIGHT
            else:
                return AvoidanceAction.PIVOT_180
        
        return AvoidanceAction.PIVOT_LEFT
    
    def get_avoidance_action(self) -> AvoidanceAction:
        closest = self.get_closest_in_path()
        if closest is None:
            self._stop_start_time = 0.0
            self._in_recovery = False
            return AvoidanceAction.NONE
        
        # Critical zone - stop or recover
        if closest.distance < self.critical_distance:
            if not self._in_recovery:
                if self._stop_start_time == 0:
                    self._stop_start_time = time.time()
                
                # Check timeout
                if time.time() - self._stop_start_time > self._stop_timeout:
                    self._in_recovery = True
                    self._stop_start_time = 0.0
                    return self._choose_pivot_direction(closest)
                
                return AvoidanceAction.STOP
            else:
                return self._choose_pivot_direction(closest)
        
        # Warning zone
        if closest.distance < self.warning_distance:
            self._stop_start_time = 0.0
            
            if closest.angle > 0.15:
                if not self.left_boundary_blocked:
                    return AvoidanceAction.TURN_LEFT
                elif not self.right_boundary_blocked:
                    return AvoidanceAction.TURN_RIGHT
                else:
                    return AvoidanceAction.SLOW_DOWN
            elif closest.angle < -0.15:
                if not self.right_boundary_blocked:
                    return AvoidanceAction.TURN_RIGHT
                elif not self.left_boundary_blocked:
                    return AvoidanceAction.TURN_LEFT
                else:
                    return AvoidanceAction.SLOW_DOWN
            else:
                return AvoidanceAction.SLOW_DOWN
        
        self._in_recovery = False
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
# Visualization Functions
# =============================================================================

def draw_large_arrow(frame, action: AvoidanceAction, speed_factor: float):
    """Draw a VERY large directional arrow in center of frame."""
    h, w = frame.shape[:2]
    center_x = w // 2
    center_y = h // 2 + 50  # Slightly below center
    
    # BIG arrow parameters
    arrow_length = 200
    thickness = 20
    
    # Colors based on action
    colors = {
        AvoidanceAction.NONE: (0, 255, 0),       # Green - clear
        AvoidanceAction.SLOW_DOWN: (0, 200, 255), # Orange - caution
        AvoidanceAction.TURN_LEFT: (255, 200, 0), # Cyan - turn
        AvoidanceAction.TURN_RIGHT: (255, 200, 0),
        AvoidanceAction.STOP: (0, 0, 255),        # Red - stop
    }
    color = colors.get(action, (128, 128, 128))
    
    if action == AvoidanceAction.NONE:
        # Big UP arrow (go forward)
        pt1 = (center_x, center_y + 80)
        pt2 = (center_x, center_y - 80)
        cv2.arrowedLine(frame, pt1, pt2, color, thickness, tipLength=0.35)
        
    elif action == AvoidanceAction.TURN_LEFT:
        # Big LEFT arrow
        pt1 = (center_x + 100, center_y)
        pt2 = (center_x - 100, center_y)
        cv2.arrowedLine(frame, pt1, pt2, color, thickness, tipLength=0.35)
        
    elif action == AvoidanceAction.TURN_RIGHT:
        # Big RIGHT arrow
        pt1 = (center_x - 100, center_y)
        pt2 = (center_x + 100, center_y)
        cv2.arrowedLine(frame, pt1, pt2, color, thickness, tipLength=0.35)
        
    elif action == AvoidanceAction.PIVOT_LEFT:
        # Curved left arrow (tank turn) - magenta
        color = (255, 0, 255)
        cv2.ellipse(frame, (center_x, center_y), (80, 80), 0, 45, 225, color, thickness)
        # Arrow tip
        pt1 = (center_x - 57, center_y - 57)
        pt2 = (center_x - 75, center_y - 35)
        cv2.arrowedLine(frame, pt1, pt2, color, thickness, tipLength=0.5)
        
    elif action == AvoidanceAction.PIVOT_RIGHT:
        # Curved right arrow (tank turn) - magenta
        color = (255, 0, 255)
        cv2.ellipse(frame, (center_x, center_y), (80, 80), 0, -45, 135, color, thickness)
        # Arrow tip
        pt1 = (center_x + 57, center_y - 57)
        pt2 = (center_x + 75, center_y - 35)
        cv2.arrowedLine(frame, pt1, pt2, color, thickness, tipLength=0.5)
        
    elif action == AvoidanceAction.PIVOT_180:
        # Full U-turn symbol - magenta
        color = (255, 0, 255)
        cv2.ellipse(frame, (center_x, center_y - 20), (60, 60), 0, 0, 180, color, thickness)
        # Left leg
        cv2.line(frame, (center_x - 60, center_y - 20), (center_x - 60, center_y + 60), color, thickness)
        # Right leg with arrow
        cv2.arrowedLine(frame, (center_x + 60, center_y - 20), (center_x + 60, center_y + 80), color, thickness, tipLength=0.3)
        
    elif action == AvoidanceAction.SLOW_DOWN:
        # Slow arrow (smaller up with dots)
        pt1 = (center_x, center_y + 60)
        pt2 = (center_x, center_y - 40)
        cv2.arrowedLine(frame, pt1, pt2, color, thickness, tipLength=0.4)
        # Dots below
        for i in range(3):
            cv2.circle(frame, (center_x - 40 + i*40, center_y + 100), 10, color, -1)
        
    elif action == AvoidanceAction.STOP:
        # Big STOP circle with X
        cv2.circle(frame, (center_x, center_y), 100, color, -1)
        cv2.line(frame, (center_x - 50, center_y - 50), (center_x + 50, center_y + 50), (255, 255, 255), 12)
        cv2.line(frame, (center_x + 50, center_y - 50), (center_x - 50, center_y + 50), (255, 255, 255), 12)


def draw_action_label(frame, action: AvoidanceAction, speed_factor: float):
    """Draw HUGE action label at top of screen."""
    h, w = frame.shape[:2]
    
    # Action text and color
    action_text = action.value
    colors = {
        AvoidanceAction.NONE: (0, 255, 0),
        AvoidanceAction.SLOW_DOWN: (0, 200, 255),
        AvoidanceAction.TURN_LEFT: (255, 200, 0),
        AvoidanceAction.TURN_RIGHT: (255, 200, 0),
        AvoidanceAction.PIVOT_LEFT: (255, 0, 255),   # Magenta
        AvoidanceAction.PIVOT_RIGHT: (255, 0, 255),  # Magenta
        AvoidanceAction.PIVOT_180: (255, 0, 255),    # Magenta
        AvoidanceAction.STOP: (0, 0, 255),
    }
    color = colors.get(action, (128, 128, 128))
    
    # Draw semi-transparent background bar (simplified - no blend)
    cv2.rectangle(frame, (0, 0), (w, 80), (20, 20, 30), -1)
    
    # Draw BIG action text
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 3.0
    thickness = 6
    (text_w, text_h), _ = cv2.getTextSize(action_text, font, font_scale, thickness)
    text_x = (w - text_w) // 2
    text_y = 60
    
    # Shadow
    cv2.putText(frame, action_text, (text_x + 4, text_y + 4), font, font_scale, (0, 0, 0), thickness + 3)
    cv2.putText(frame, action_text, (text_x, text_y), font, font_scale, color, thickness)


def draw_speed_bar(frame, speed_factor: float):
    """Draw speed factor bar."""
    h, w = frame.shape[:2]
    
    bar_width = 300
    bar_height = 30
    bar_x = w - bar_width - 30
    bar_y = h - 60
    
    # Background
    cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), (50, 50, 50), -1)
    cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), (100, 100, 100), 2)
    
    # Speed fill
    fill_width = int(bar_width * speed_factor)
    if speed_factor > 0.7:
        fill_color = (0, 255, 0)
    elif speed_factor > 0.3:
        fill_color = (0, 200, 255)
    else:
        fill_color = (0, 0, 255)
    cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill_width, bar_y + bar_height), fill_color, -1)
    
    # Label
    label = f"SPEED: {int(speed_factor * 100)}%"
    cv2.putText(frame, label, (bar_x, bar_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


def draw_obstacle_info(frame, obstacles: List[Obstacle], closest: Optional[Obstacle]):
    """Draw obstacle info panel."""
    h, w = frame.shape[:2]
    
    # Info panel on left side
    panel_x = 20
    panel_y = 140
    panel_w = 280
    panel_h = 150
    
    # Background (simplified - no blend)
    cv2.rectangle(frame, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), (20, 20, 30), -1)
    cv2.rectangle(frame, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), (80, 80, 100), 2)
    
    # Title
    cv2.putText(frame, "OBSTACLE INFO", (panel_x + 10, panel_y + 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
    
    y = panel_y + 55
    
    # Obstacle count
    cv2.putText(frame, f"Detected: {len(obstacles)}", (panel_x + 15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    y += 30
    
    # Closest obstacle info
    if closest:
        cv2.putText(frame, f"Closest: {closest.type}", (panel_x + 15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        y += 25
        cv2.putText(frame, f"Distance: {closest.distance:.2f}m", (panel_x + 15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        y += 25
        angle_deg = math.degrees(closest.angle)
        side = "LEFT" if angle_deg < 0 else "RIGHT" if angle_deg > 0 else "CENTER"
        cv2.putText(frame, f"Position: {side} ({abs(angle_deg):.0f}°)", (panel_x + 15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    else:
        cv2.putText(frame, "Path: CLEAR", (panel_x + 15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)


def draw_detection_boxes(frame, detections, obstacles, depth_frame):
    """Draw detection boxes with distance."""
    h, w = frame.shape[:2]
    
    for i, det in enumerate(detections):
        x1, y1, x2, y2 = det.to_pixels(w, h)
        
        # Color based on safety
        category = det.safety_category
        if category == "human":
            color = (0, 0, 255)  # Red
        elif category == "animal":
            color = (0, 165, 255)  # Orange
        elif category == "vehicle":
            color = (255, 0, 0)  # Blue
        else:
            color = (128, 128, 128)  # Gray
        
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        
        # Distance label
        if i < len(obstacles):
            dist = obstacles[i].distance
            label = f"{det.class_label}: {dist:.1f}m"
        else:
            label = det.class_label
        
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def draw_help(frame):
    """Draw small help text at bottom."""
    h, w = frame.shape[:2]
    
    help_text = "Press 'h' for help | 'q' to quit"
    cv2.putText(frame, help_text, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)


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
        # Get depth at detection center
        bbox = BoundingBox2D(det.x_min, det.y_min, det.x_max, det.y_max)
        depth_mm, confidence = sample_depth_robust(
            depth_frame, bbox, frame_shape=(dh, dw), sample_count=5
        )
        
        if depth_mm <= 0:
            continue
        
        distance = depth_mm / 1000.0  # Convert to meters
        
        # Calculate angle from center (assume 60° FOV)
        center_x = (det.x_min + det.x_max) / 2
        angle = (center_x - 0.5) * math.radians(60)
        
        # Estimate width
        width = (det.x_max - det.x_min) * distance
        
        obstacles.append(Obstacle(
            type=det.class_label,
            distance=distance,
            angle=angle,
            width=width,
            confidence=det.confidence,
        ))
    
    return obstacles


def adjust_bbox_from_letterbox(det: Detection2D) -> Detection2D:
    """Adjust detection bbox from letterboxed NN space to preview space."""
    nn_w, nn_h = YOLO_INPUT_SIZE
    preview_w, preview_h = CAM_PREVIEW_SIZE
    
    scale = min(nn_w / preview_w, nn_h / preview_h)
    padded_w = preview_w * scale
    padded_h = preview_h * scale
    
    offset_x = (nn_w - padded_w) / 2 / nn_w
    offset_y = (nn_h - padded_h) / 2 / nn_h
    
    scale_x = nn_w / padded_w
    scale_y = nn_h / padded_h
    
    x_min_adj = max(0.0, min(1.0, (det.x_min - offset_x) * scale_x))
    y_min_adj = max(0.0, min(1.0, (det.y_min - offset_y) * scale_y))
    x_max_adj = max(0.0, min(1.0, (det.x_max - offset_x) * scale_x))
    y_max_adj = max(0.0, min(1.0, (det.y_max - offset_y) * scale_y))
    
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
# Main
# =============================================================================

def main():
    print("=" * 70)
    print("  OBSTACLE AVOIDANCE DEMO")
    print("  Shows directional arrows for mower navigation")
    print("=" * 70)
    print()
    print("Controls: q=Quit, h=Toggle help")
    print()

    # Model paths - look in ai_camera_vision/models
    models_dir = AI_CAMERA_DIR / "models"
    yolo_blob = models_dir / "yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"

    if not yolo_blob.exists():
        print(f"ERROR: YOLO model blob not found at: {yolo_blob}")
        return 1

    print(f"YOLO model: {yolo_blob.name}")
    print("(Using FAST mode - YOLO only, no segmentation)")
    print()
    print("Starting pipeline (USB2 mode)...")

    # Create FAST pipeline (YOLO + depth only, no segmentation)
    pipeline = create_yolo_only_pipeline(yolo_blob)

    # Obstacle handler
    handler = SimpleObstacleHandler()
    
    # State - help OFF by default for clean view
    show_help = False
    last_detections = []
    last_obstacles = []
    last_depth_frame = None
    frame_count = 0
    start_time = time.time()

    # Display scale - 1.5x is a good balance (was 2.0 which killed FPS)
    DISPLAY_SCALE = 1.5
    
    # Frame skip for visualization (process every Nth frame for display)
    SKIP_FRAMES = 1  # Set to 2 to double FPS, 1 = no skip

    # Use direct device connection (not PipelineRunner which expects segmentation)
    with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
        print("Pipeline running! Press 'h' for help, 'q' to quit.")
        print("-" * 70)
        
        # Get queues
        q_rgb = device.getOutputQueue("rgb_preview", maxSize=4, blocking=False)
        q_det = device.getOutputQueue("detections", maxSize=4, blocking=False)
        q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)

        while True:
            # Get RGB frame
            rgb_frame = q_rgb.tryGet()

            if rgb_frame is not None:
                frame_count += 1
                frame = rgb_frame.getCvFrame()

                # Get depth
                depth_data = q_depth.tryGet()
                if depth_data is not None:
                    last_depth_frame = depth_data.getFrame()

                # Get detections
                det_data = q_det.tryGet()
                if det_data is not None:
                    raw_detections = parse_img_detections(det_data)
                    last_detections = [adjust_bbox_from_letterbox(d) for d in raw_detections]
                    
                    # Convert to obstacles
                    last_obstacles = detections_to_obstacles(last_detections, last_depth_frame)
                    handler.update_obstacles(last_obstacles)

                # Get avoidance action
                action = handler.get_avoidance_action()
                speed_factor = handler.get_speed_factor()
                closest = handler.get_closest_in_path()

                # === BUILD VISUALIZATION ===
                display = frame.copy()
                
                # Draw detection boxes
                draw_detection_boxes(display, last_detections, last_obstacles, last_depth_frame)
                
                # Draw action label at top
                draw_action_label(display, action, speed_factor)
                
                # Draw large directional arrow
                draw_large_arrow(display, action, speed_factor)
                
                # Draw speed bar
                draw_speed_bar(display, speed_factor)
                
                # Draw obstacle info panel
                draw_obstacle_info(display, last_obstacles, closest)
                
                # Draw help if enabled
                if show_help:
                    draw_help(display)
                
                # Scale up for big display (use fast interpolation)
                if DISPLAY_SCALE != 1.0:
                    new_h = int(display.shape[0] * DISPLAY_SCALE)
                    new_w = int(display.shape[1] * DISPLAY_SCALE)
                    display = cv2.resize(display, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
                
                cv2.imshow("Obstacle Avoidance Demo", display)

                # FPS output
                if frame_count % 60 == 0:
                    elapsed = time.time() - start_time
                    fps = frame_count / elapsed
                    print(f"FPS: {fps:.1f} | Action: {action.value} | Speed: {speed_factor*100:.0f}% | Obstacles: {len(last_obstacles)}")

            # Keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("\nQuitting...")
                break
            elif key == ord("h"):
                show_help = not show_help

    cv2.destroyAllWindows()
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
