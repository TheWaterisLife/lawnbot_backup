# Epic 2: Coverage Path Planning

## Goal
Implement a boustrophedon (back-and-forth) path planner that generates an efficient coverage path for any polygon boundary.

## Background
Boustrophedon is the most efficient coverage pattern for convex areas. For concave polygons, the area must be decomposed into convex cells. The mower follows parallel lines (rows) spaced by the mowing width (30 cm), ensuring complete coverage with minimal overlap.

## Coverage Algorithm Overview

```
Input: Polygon boundary, row_spacing (30cm)
Output: Ordered list of waypoints

1. Determine optimal sweep direction
   - Minimize number of turns
   - Usually aligned with longest edge

2. Generate sweep lines
   - Parallel lines at row_spacing intervals
   - Perpendicular to sweep direction

3. Clip lines to polygon
   - Find intersection with boundary
   - Create line segments inside polygon

4. Connect segments into path
   - Alternate direction each row
   - Add turn waypoints at ends

5. Handle concave polygons
   - Decompose into convex cells
   - Plan each cell
   - Optimize cell ordering
```

## Stories

### Story 2.1: Simple Polygon Coverage
**As a** navigation system  
**I want** a coverage path for convex polygons  
**So that** I can mow simple lawn shapes

**Acceptance Criteria:**
- [ ] Accept polygon as input
- [ ] Generate parallel sweep lines at row_spacing
- [ ] Clip lines to polygon boundary
- [ ] Connect into continuous path
- [ ] Path covers >= 95% of polygon area
- [ ] Output as nav_msgs/Path

**Parameters:**
- row_spacing: 0.30 m (mowing width)
- turn_radius: 0.20 m (minimum turn radius)

---

### Story 2.2: Concave Polygon Handling
**As a** navigation system  
**I want** coverage paths for concave polygons  
**So that** I can mow L-shaped or complex lawns

**Acceptance Criteria:**
- [ ] Detect concave polygons
- [ ] Decompose into convex cells (trapezoidal decomposition)
- [ ] Plan coverage for each cell
- [ ] Connect cells with transition paths
- [ ] Optimize cell order to minimize travel

**Algorithm:**
- Use trapezoidal decomposition or Boustrophedon Cell Decomposition
- Connect adjacent cells through shared edges
- TSP-style optimization for cell ordering

---

### Story 2.3: Sweep Direction Optimization
**As a** navigation system  
**I want** the optimal sweep direction  
**So that** I minimize the number of turns

**Acceptance Criteria:**
- [ ] Calculate optimal direction for minimum turns
- [ ] Consider polygon shape (align with longest edge)
- [ ] Allow manual override via parameter
- [ ] Report chosen direction and turn count

---

### Story 2.4: Path Resumption
**As a** mower returning from charging  
**I want** to resume from where I stopped  
**So that** I don't redo completed areas

**Acceptance Criteria:**
- [ ] Save current waypoint index on pause/stop
- [ ] Store progress in state file
- [ ] Resume from saved position
- [ ] Handle position drift during charge
- [ ] Validate we're near the resume point

**State File:**
```json
{
  "zone_id": "front_yard",
  "waypoint_index": 127,
  "total_waypoints": 450,
  "last_position": {"x": 12.5, "y": 8.3},
  "paused_at": "2026-01-26T11:45:00Z"
}
```

---

### Story 2.5: Coverage Estimation
**As a** user  
**I want** to know estimated mowing time  
**So that** I can plan accordingly

**Acceptance Criteria:**
- [ ] Calculate total path length
- [ ] Estimate time based on speed (0.45 m/s)
- [ ] Account for turns (slower)
- [ ] Report percentage complete during mowing
- [ ] Display remaining time estimate

---

### Story 2.6: Nav2 Planner Plugin
**As a** Nav2 stack  
**I want** coverage planning as a planner plugin  
**So that** it integrates with the navigation framework

**Acceptance Criteria:**
- [ ] Implement nav2_core::GlobalPlanner interface
- [ ] Accept start pose and goal (ignored for coverage)
- [ ] Return coverage path as nav_msgs/Path
- [ ] Support replanning on request
- [ ] Register plugin in Nav2 config

## Definition of Done
- All stories completed
- Coverage path generated for test polygons
- Handles both convex and concave shapes
- Path resumption works after simulated charge
- Integrated with Nav2 as plugin
- Tests passing
- Code reviewed
