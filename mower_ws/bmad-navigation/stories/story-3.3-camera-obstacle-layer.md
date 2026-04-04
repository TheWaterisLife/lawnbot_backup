# Story 3.3: Camera Obstacle Layer

## Status: ✅ Complete

## Description

As a mower system, I need to project camera obstacle detections into Nav2's costmap so that the local planner can route around obstacles in real-time.

## Acceptance Criteria

- [x] Camera detections converted to PointCloud2 format
- [x] Published to Nav2-compatible topic
- [x] Costmap updated at 10 Hz
- [x] Visualization available in RViz

## Technical Implementation

### Files Created
- `src/mower_navigation/mower_navigation/camera_obstacle_publisher.py`

### ROS2 Node: `camera_obstacle_publisher`

**Published Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/vision/obstacles` | `PointCloud2` | Obstacle points for Nav2 |
| `/vision/obstacles_viz` | `MarkerArray` | RViz visualization |

**Subscribed Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/vision/detections_3d` | `PointCloud2` | 3D detections from camera |

### Zone Cost Mapping
```python
ZONE_COSTS = {
    'green': 0,      # FREE_SPACE
    'yellow': 200,   # High cost but passable
    'red': 254,      # LETHAL_OBSTACLE
}
```

### Nav2 Configuration
```yaml
# nav2_params.yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  observation_sources: camera_detections
  camera_detections:
    topic: /vision/obstacles
    data_type: "PointCloud2"
    max_obstacle_height: 2.0
    clearing: true
    marking: true
```

### Key Class
```python
class CameraObstaclePublisher(Node):
    def add_obstacle(self, x, y, z=0.0, cost=254):
        """Add an obstacle point to be published."""
    
    def add_obstacles_from_zones(self, zone_points):
        """Add obstacles from semantic zones (red/yellow/green)."""
```

## Test Cases

1. Single obstacle in path
2. Multiple obstacles
3. Moving obstacle (tracks correctly)
4. Obstacle cleared when removed
5. High-frequency updates don't cause issues

## Dependencies

- bmad-camera: AI Camera Vision (provides detections)
- Nav2: Costmap2D (consumes obstacles)

## Related Stories

- Story 3.5: Boundary Virtual Fence
