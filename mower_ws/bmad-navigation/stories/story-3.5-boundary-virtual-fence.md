# Story 3.5: Boundary Virtual Fence

## Status: ✅ Complete

## Description

As a mower system, I need to create an invisible fence along the boundary perimeter so that the Nav2 planner never generates paths outside the lawn area.

## Acceptance Criteria

- [x] Fence points generated along boundary perimeter
- [x] Configurable fence thickness
- [x] Published as PointCloud2 for Nav2 ObstacleLayer
- [x] Polygon visualization available

## Technical Implementation

### Files Created
- `src/mower_navigation/mower_navigation/boundary_costmap_publisher.py`

### ROS2 Node: `boundary_costmap_publisher`

**Published Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/boundary/fence` | `PointCloud2` | Fence obstacle points |
| `/boundary/polygon` | `PolygonStamped` | Boundary visualization |

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `boundary_file` | `` | Path to boundary JSON |
| `boundary_name` | `default` | Name of boundary |
| `fence_thickness` | `0.5` | Fence width in meters |
| `fence_resolution` | `0.1` | Point spacing in meters |
| `publish_rate` | `1.0` | Hz (slow, boundary is static) |

### Algorithm
```python
def _generate_fence_points(self):
    """Generate fence points along boundary perimeter."""
    for each edge (p1, p2) in polygon:
        # Calculate edge length and normal vector
        dx, dy = p2.x - p1.x, p2.y - p1.y
        length = sqrt(dx**2 + dy**2)
        nx, ny = -dy/length, dx/length  # Outward normal
        
        # Generate points along edge
        for t in range(0, 1, resolution/length):
            x = p1.x + t * dx
            y = p1.y + t * dy
            
            # Add points at boundary and offset (fence thickness)
            add_point(x, y)
            for offset in [0.1, 0.25, 0.5]:
                add_point(x + nx*offset, y + ny*offset)
```

### Nav2 Configuration
```yaml
# nav2_params.yaml
boundary_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  observation_sources: boundary_fence
  boundary_fence:
    topic: /boundary/fence
    data_type: "PointCloud2"
    clearing: false  # Never clear boundary
    marking: true
    obstacle_max_range: 10.0
```

### Key Features
- **Static boundary**: Published at low rate (1 Hz) since boundary doesn't change
- **Thick fence**: Multiple layers of points prevent any path through
- **Normal calculation**: Points placed outside polygon (CCW winding assumed)
- **Coordinate Agnostic**: Works with both GPS-derived and Local-only polygons

## Test Cases

1. Rectangular boundary
2. Concave boundary
3. Robot approaches boundary edge
4. Nav2 avoids boundary in path planning

## Dependencies

- Story 1.3: Boundary Storage (provides boundary data)
- Nav2: Costmap2D (consumes fence)

## Related Stories

- Story 3.3: Camera Obstacle Layer
