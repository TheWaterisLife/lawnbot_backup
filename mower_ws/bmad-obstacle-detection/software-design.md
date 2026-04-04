# Obstacle Detection & Avoidance: Software Design

## 1. System / Component Diagram

The system diagram illustrates the dual vision models running on the camera's VPU and how the host process interacts with the mower's motor server.

```plantuml
@startuml
!include <C4/C4_Context>

title System Context — Obstacle Detection & Avoidance

System_Ext(oakd, "OAK-D Lite (VPU)", "Runs stereo depth\nYOLOv3 detection\nDeepLabV3+ segmentation")
System(obstacle_det, "obstacle_detection", "Standalone Process\nMerges vision & depth\nDecides evasive actions")
System(motors, "lawnbot_motors", "WebSocket daemon\nExecutes direct motor commands\nws://127.0.0.1:8766")

oakd --> obstacle_det : USB3 (XLink)\nRaw tensors & depth map
obstacle_det --> motors : WebSocket JSON\n(e.g., 'pivot_left')

note bottom of obstacle_det
  Internal Pipeline:
  - Detection Parser
  - Segmentation Parser
  - TrackedObstacle Builder
  - Action Decider
end note
@enduml
```

### Component Details
1. **OAK-D Lite VPU Pipeline**: The intensive ML work is offloaded to the camera. It runs three parallel nodes: Stereo Depth, YOLO object detection (people, animals, etc.), and DeepLabV3+ semantic segmentation (drivable vs. non-drivable surfaces).
2. **Host Processing (`obstacle_avoidance_full.py`)**: 
   - **Detection Parser**: Decodes raw YOLO outputs.
   - **Segmentation Parser**: Evaluates the bottom 60% of the image (the immediate path) by splitting it into Left, Center, Right columns and counting "red" (non-drivable) pixels.
   - **TrackedObstacle Builder**: Merges YOLO objects and Segmentation walls, using depth map values.
3. **Action Decider**: Distance-layered decision engine that chooses navigation commands based on the closest obstacle.

---

## 2. Class Diagram

```plantuml
@startuml
!theme toy

class Detection2D {
    + class_id: int
    + class_label: string
    + confidence: float
    + x_min: float
    + y_min: float
    + x_max: float
    + y_max: float
    + safety_category: string
}

class TrackedObstacle {
    + label: string
    + distance: float
    + angle_rad: float
    + lateral: float
    + width: float
    + in_path: bool
    + __init__(label, distance, angle_rad, width)
}

class MotorCommander {
    - ws_uri: string
    - ws_client: WebSocket
    - is_connected: bool
    + ensure_connected()
    + execute(action, speed_factor)
}

package "Decision Engine" <<Rectangle>> {
    class "AvoidanceLogic" {
        + decide_action(obstacles, stop_start): Tuple[string, string]
        - _best_escape_dir(nearest, all_obs): string
    }
}

package "Vision Pipeline" <<Rectangle>> {
    class "DepthAIEngine" {
        + create_pipeline(yolo_blob, seg_blob): dai.Pipeline
        + parse_img_detections(img_detections): List[Detection2D]
        + build_obstacles(detections, depth_frame): List[TrackedObstacle]
        + seg_wall_obstacle(seg_mask, depth_frame): List[TrackedObstacle]
    }
}

Detection2D -down-> TrackedObstacle : projected using Depth
TrackedObstacle "many" --> "AvoidanceLogic" : evaluated by
"AvoidanceLogic" --> MotorCommander : determines action

@enduml
```

### Class Details
- **TrackedObstacle**: Represents a physical object mapped into 3D space. It calculates its `lateral` displacement from the camera origin and determines if it is `in_path`.

---

## 3. Sequence Diagram

This sequence illustrates a single loop where an obstacle is detected, merged, and evaded.

```plantuml
@startuml
!theme toy

actor "Camera/VPU" as VPU
participant "Main Loop" as Loop
participant "Vision Pipeline" as Vision
participant "Avoidance Logic" as Logic
participant "MotorCommander" as Motor
participant "lawnbot_motors" as Motors

loop Async Event Loop
    VPU -> Loop: Queue Events (det, seg, depth)
    
    Loop -> Vision: parse_img_detections()
    Vision --> Loop: List<Detection2D>
    
    Loop -> Vision: decode_segmentation_mask()
    Vision --> Loop: 2D Numpy Mask
    
    Loop -> Vision: build_obstacles(detections, depth)
    Vision --> Loop: List<TrackedObstacle> (YOLO objects)
    
    Loop -> Vision: seg_wall_obstacle(mask, depth)
    Vision --> Loop: List<TrackedObstacle> ("wall_center")
    
    Loop -> Loop: all_obs = yolo_obs + seg_wall_obs
    
    Loop -> Logic: decide_action(all_obs)
    
    opt Nearest Obstacle < 0.25m
        Logic -> Logic: _best_escape_dir() -> "left"
        Logic --> Loop: action="pivot_left"
    end
    
    opt Nearest Obstacle > 0.50m
        Logic --> Loop: action="forward"
    end
    
    Loop -> Motor: execute(action="pivot_left")
    Motor -> Motors: WS Send (JSON command)
end

@enduml
```

### Sequence Flow Details
- **Data Acquisition**: Non-blocking extraction of asynchronous neural network results and depth frames.
- **YOLO & Segmentation Resolution**: Objects and Semantic boundaries are independently resolved using the depth mask.
- **Action Determination**: Merged list is prioritized by distance to determine immediate safe vectors.
