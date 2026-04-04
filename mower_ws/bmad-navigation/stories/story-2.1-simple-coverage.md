# Story 2.1: Simple Polygon Coverage

**Epic**: 2 - Coverage Path Planning  
**Status**: Not Started  
**Priority**: Must  

## User Story

**As a** navigation system  
**I want** a coverage path for convex polygons  
**So that** I can mow simple lawn shapes

## Acceptance Criteria

- [ ] Accept polygon as input
- [ ] Generate parallel sweep lines at row_spacing
- [ ] Clip lines to polygon boundary
- [ ] Connect into continuous path
- [ ] Path covers >= 95% of polygon area
- [ ] Output as nav_msgs/Path

## Technical Details

### Algorithm Overview

```
1. Input: Polygon vertices, row_spacing (0.30m)

2. Find bounding box of polygon

3. Determine sweep direction (default: along longest edge)

4. Generate parallel lines:
   - Start at bounding box edge
   - Space lines by row_spacing
   - Continue until past opposite edge

5. Clip lines to polygon:
   - For each line, find intersection with polygon edges
   - Create line segments inside polygon

6. Connect segments:
   - Start from one end
   - Alternate direction each row (boustrophedon)
   - Add turn waypoints at endpoints

7. Output: Ordered list of waypoints
```

### Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| row_spacing | 0.30 m | Mowing width |
| turn_radius | 0.20 m | Minimum turn at row ends |
| edge_offset | 0.15 m | Stay away from boundary |

### Class Interface

```python
@dataclass
class CoverageConfig:
    row_spacing: float = 0.30  # meters
    turn_radius: float = 0.20  # meters
    edge_offset: float = 0.15  # meters
    sweep_angle: Optional[float] = None  # radians, None = auto

class SimpleCoveragePlanner:
    def __init__(self, config: CoverageConfig):
        pass
    
    def set_polygon(self, vertices: List[Tuple[float, float]]) -> bool:
        """Set polygon to cover. Returns False if invalid."""
        pass
    
    def plan(self) -> List[Tuple[float, float]]:
        """Generate coverage path. Returns waypoints."""
        pass
    
    def get_total_length(self) -> float:
        """Get total path length in meters."""
        pass
    
    def get_estimated_time(self, velocity: float = 0.45) -> float:
        """Get estimated completion time in seconds."""
        pass
    
    def get_coverage_percentage(self) -> float:
        """Calculate coverage percentage of polygon area."""
        pass
```

### Visualization

```
Input Polygon:           Coverage Path:
+----------+             +----------+
|          |             |→→→→→→→→→→|
|          |             |←←←←←←←←←←|
|          |       =>    |→→→→→→→→→→|
|          |             |←←←←←←←←←←|
|          |             |→→→→→→→→→→|
+----------+             +----------+
```

## Test File

`Tests/unit/navigation/test_coverage_2_1_simple.py`

## Test Cases

1. Square polygon generates correct number of rows
2. Rectangle polygon (10x8m) with 0.3m spacing -> 27 rows
3. Path alternates direction each row
4. Path stays within polygon boundary
5. Coverage >= 95% of area
6. Edge offset is respected
7. Total length calculation is accurate
8. Empty polygon returns empty path
9. Single-point polygon rejected
10. Path connects all rows

## Definition of Done

- [ ] SimpleCoveragePlanner class implemented
- [ ] All unit tests passing
- [ ] Coverage >= 95% verified
- [ ] Code reviewed
- [ ] Documented in code
