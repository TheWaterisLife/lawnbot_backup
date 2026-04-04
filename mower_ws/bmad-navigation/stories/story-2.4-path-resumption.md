# Story 2.4: Path Resumption

## Status: ✅ Complete

## Description

As a mower system, I need to save my progress and resume mowing after interruptions (battery recharge, rain, user pause) so that I don't re-mow areas already completed.

## Acceptance Criteria

- [x] Progress saved to persistent storage (JSON file)
- [x] Progress includes: waypoint index, position, timestamp, completion percentage
- [x] Resume from saved progress on restart
- [x] Handle corrupted progress files gracefully
- [x] Clear progress after task completion
- [ ] **Local Mode Warning**: Resumption requires precise origin reset if GPS unavailable

## Technical Implementation

### Files Modified
- `src/mower_navigation/mower_navigation/coverage_planner.py`

### Key Classes
```python
@dataclass
class Waypoint:
    x: float
    y: float
    heading: float
    index: int = 0  # Position in path for resumption

@dataclass  
class CoverageProgress:
    boundary_name: str
    total_waypoints: int
    completed_waypoints: int
    current_index: int
    last_position: Optional[Tuple[float, float]]
    start_time: Optional[str]
    last_update_time: Optional[str]
    is_complete: bool = False
```

### Key Methods
```python
class CoveragePlanner:
    def update_progress(self, completed_index: int, position: Tuple[float, float]) -> None:
        """Update progress after completing a waypoint."""
    
    def save_progress(self, filename: Optional[str] = None) -> bool:
        """Save current progress to file."""
    
    def load_progress(self, filename: str) -> bool:
        """Load progress from file."""
    
    def get_remaining_path(self, boundary: Boundary, from_index: Optional[int] = None) -> List[Waypoint]:
        """Get remaining path from a specific index."""
```

### File Format
```json
{
  "progress": {
    "boundary_name": "front_yard",
    "total_waypoints": 200,
    "completed_waypoints": 127,
    "current_index": 127,
    "last_position": [5.2, 3.8],
    "start_time": "2026-02-01T10:30:00",
    "last_update_time": "2026-02-01T11:45:00",
    "is_complete": false
  },
  "path": [
    {"x": 0.0, "y": 0.0, "heading": 0.0, "index": 0},
    ...
  ]
}
```

### Storage Location
`config/coverage_progress/{boundary_name}_progress.json`

## Usage Example
```python
# Save progress during mowing
planner.update_progress(completed_index=50, position=(5.2, 3.8))
planner.save_progress()

# Resume after restart
planner.load_progress("front_yard")
remaining = planner.get_remaining_path(boundary)
# Continues from waypoint 51
```

## Test Cases

1. Save and load progress
2. Resume from mid-path
3. Handle missing progress file
4. Handle corrupted progress file
5. Clear progress after completion

## Local Mode Constraints (No GPS)
In GPS-denied mode, `last_position` is relative to the startup origin.
To resume correctly:
1. User must place robot at **exact original "Home" position/heading**.
2. Robot restores odometry to (0,0).
3. Robot navigates to saved waypoint.
*Warning: Accumulating drift may make resumption inaccurate without GPS.*

## Dependencies

- Story 2.1: Simple Coverage Planner

## Related Stories

- Story 2.6: Nav2 Planner Plugin (uses resumption)
