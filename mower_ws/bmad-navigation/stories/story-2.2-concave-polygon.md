# Story 2.2: Concave Polygon Handling

## Status: ✅ Complete

## Description

As a mower system, I need to handle concave lawn shapes (L-shaped, T-shaped, or irregular boundaries) so that I can mow any residential lawn without leaving uncovered areas.

## Acceptance Criteria

- [x] System correctly identifies all intersection points between sweep lines and polygon edges
- [x] Coverage path skips areas outside the boundary for concave shapes
- [x] No gaps in coverage for L-shaped or T-shaped lawns
- [x] Path generation completes in < 1 second for typical lawn sizes

## Technical Implementation

### Files Modified
- `src/mower_navigation/mower_navigation/coverage_planner.py`

### Key Functions
```python
def find_polygon_intersections(y, polygon, min_x, max_x) -> List[float]:
    """
    Find all X coordinates where a horizontal line at Y intersects the polygon.
    Returns sorted list of intersection points.
    """

def find_polygon_intersections_vertical(x, polygon, min_y, max_y) -> List[float]:
    """
    Find all Y coordinates where a vertical line at X intersects the polygon.
    """
```

### Algorithm
1. For each sweep line (horizontal or vertical), find all intersection points with polygon edges
2. Sort intersections and pair them (entry/exit points)
3. Generate coverage passes only within valid polygon segments
4. Handle odd number of intersections gracefully (edge cases)

### Example: L-Shaped Lawn
```
+-------+
|   1   |
|       +---+
|     2     |
+-----------+

Sweep at y=1.5:
- Intersections: [0, 3, 5, 10]
- Passes: (0,3) and (5,10)
```

## Test Cases

1. Rectangular lawn (baseline)
2. L-shaped lawn
3. T-shaped lawn
4. Lawn with narrow passage
5. Lawn with internal exclusion zone

## Dependencies

- Story 2.1: Simple Coverage Planner (base implementation)

## Related Stories

- Story 2.3: Sweep Direction Optimization
- Story 2.4: Path Resumption
