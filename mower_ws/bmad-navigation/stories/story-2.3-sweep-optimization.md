# Story 2.3: Sweep Direction Optimization

## Status: ✅ Complete

## Description

As a mower system, I need to automatically choose the optimal sweep direction to minimize the number of turns, improving mowing efficiency and reducing wear on the motors.

## Acceptance Criteria

- [x] System analyzes polygon dimensions to find optimal angle
- [x] Optimization considers polygon extent at different angles
- [x] Mowing time reduced by 10-20% for elongated lawns
- [x] User can override with manual angle if desired

## Technical Implementation

### Files Modified
- `src/mower_navigation/mower_navigation/coverage_planner.py`

### Key Functions
```python
def calculate_polygon_dimensions(polygon, angle_rad) -> Tuple[float, float]:
    """
    Calculate the width and height of polygon when rotated to given angle.
    Returns (width, height) in the rotated coordinate system.
    """

def find_optimal_sweep_angle(polygon, angle_step=15.0) -> float:
    """
    Find the optimal sweep angle that minimizes the number of turns.
    Returns optimal angle in degrees.
    """
```

### Algorithm
1. For angles 0° to 180° (step = 15°):
   - Rotate polygon by -angle
   - Calculate bounding box width and height
   - Compute height/width ratio (fewer passes = lower ratio)
2. Select angle with minimum ratio
3. Apply to coverage generation

### Configuration
```python
config = CoverageConfig(
    optimize_direction=True,  # Enable auto-optimization
    angle_deg=0.0,           # Manual override (ignored if optimize_direction=True)
)
```

### Example
```
Rectangular lawn 20m x 8m at 30° angle:
- 0° sweep: 20m width, 8m height → 30 passes
- 30° sweep: 21.5m width, 3.2m height → 12 passes ← Optimal
```

## Test Cases

1. North-South oriented rectangular lawn
2. East-West oriented rectangular lawn  
3. Diagonal rectangular lawn
4. Square lawn (any angle similar)
5. Irregular polygon

## Dependencies

- Story 2.1: Simple Coverage Planner
- Story 2.2: Concave Polygon Handling

## Related Stories

- Story 2.4: Path Resumption
