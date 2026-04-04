# Product Requirements Document: Navigation Subsystem

## 1. Overview

### 1.1 Purpose
This document defines the requirements for the Navigation subsystem, which enables the autonomous lawn mower to plan and execute complete lawn coverage while respecting boundaries and avoiding obstacles.

### 1.2 Scope
- Boundary recording and management (via mower_mapping / boundary_mapper)
- Coverage path planning (boustrophedon)
- Nav2 integration for execution
- Obstacle-aware local planning
- Motor command interface
- **GPS-Denied / Local-Only Mode support**

### 1.3 Definitions

| Term | Definition |
|------|------------|
| Boustrophedon | Back-and-forth pattern like plowing a field |
| Costmap | 2D grid representing obstacle costs |
| Global Planner | Plans overall path through environment |
| Local Planner | Executes path while avoiding obstacles |
| DWB | Dynamic Window B - Nav2 local planner |

---

## 2. Functional Requirements

### 2.1 Boundary Management (FR-BND)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-BND-01 | Record boundary by driving perimeter | Must |
| FR-BND-02 | Store boundary in JSON format | Must |
| FR-BND-03 | Store local coordinates (GPS optional) | Must |
| FR-BND-04 | Support up to 5 separate zones | Must |
| FR-BND-05 | Load boundary on startup (Use GPS origin if available, else Manual) | Must |
| FR-BND-06 | Publish boundary as polygon | Should |
| FR-BND-07 | Validate boundary is closed polygon | Must |

**Boundary JSON Schema:**
```json
{
  "version": "1.0",
  "zone_id": "string",
  "zone_name": "string",
  "created_at": "ISO8601",
  "coordinate_system": "WGS84",
  "boundary": {
    "type": "Polygon",
    "coordinates_gps": [{"lat": float, "lon": float}],
    "coordinates_local": [{"x": float, "y": float}]
  },
  "origin": {
    "lat": float, 
    "lon": float,
    "manual_origin_id": "string (optional)"
  }
}
```

**Local Mode (No GPS) Workflow:**
1. User marks "Home" location physically.
2. Robot starts at "Home".
3. Local boundary defined relative to (0,0).
4. **Drift Warning**: Without GPS, long-term accuracy depends on Odom/IMU quality.

**Acceptance Criteria:**
- Boundary recorded while user drives perimeter
- Points captured every ~1 meter
- Polygon area calculated and displayed
- Boundary loads correctly after restart

### 2.2 Coverage Planning (FR-COV)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-COV-01 | Generate boustrophedon path for polygon | Must |
| FR-COV-02 | Path row spacing = mowing width (20 cm) | Must |
| FR-COV-03 | Handle concave polygons | Should |
| FR-COV-04 | Optimize path start/end near home | Should |
| FR-COV-05 | Support path resumption | Must |
| FR-COV-06 | Estimate coverage time | Should |

**Boustrophedon Algorithm:**
1. Decompose polygon into cells (if concave)
2. For each cell, generate parallel lines at row spacing
3. Connect lines into continuous path
4. Add turns at row ends
5. Order cells for minimal travel

**Acceptance Criteria:**
- Path covers >= 95% of polygon interior
- Row spacing configurable (default 20 cm)
- Path avoids self-intersection
- Total path length is near-optimal

### 2.3 Nav2 Integration (FR-NAV)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-NAV-01 | Use Nav2 for path following | Must |
| FR-NAV-02 | Subscribe to /odometry/filtered for pose | Must |
| FR-NAV-03 | Build costmap from camera detections | Must |
| FR-NAV-04 | Use DWB local planner | Must |
| FR-NAV-05 | Respect max velocity = 0.45 m/s | Must |
| FR-NAV-06 | Stop for human/animal detections | Must |

**Costmap Configuration:**
- Resolution: 5 cm/cell
- Obstacle sources: /vision/detections
- Inflation radius: 30 cm
- Update rate: 5 Hz

**Acceptance Criteria:**
- Robot follows coverage path smoothly
- Deviates to avoid obstacles
- Returns to path after obstacle clears
- Stops immediately for safety-critical obstacles

### 2.4 Motor Control Interface (FR-MOT)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-MOT-01 | Subscribe to /cmd_vel | Must |
| FR-MOT-02 | Convert twist to left/right track speeds | Must |
| FR-MOT-03 | Control BTS7960 H-Bridge via gpiozero PWM | Must |
| FR-MOT-04 | Respect velocity limits | Must |
| FR-MOT-05 | Smooth acceleration/deceleration (slew-rate ramping) | Should |

**Differential Drive Conversion:**
```
v_left  = vx - (wz * track_width / 2)
v_right = vx + (wz * track_width / 2)
pwm_left  = velocity_to_pwm(v_left)
pwm_right = velocity_to_pwm(v_right)
```

**Acceptance Criteria:**
- Smooth motor response to cmd_vel
- No abrupt direction changes
- Stops within 1.5 seconds on E-stop

### 2.5 Safety Behaviors (FR-SAF)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-SAF-01 | Stop immediately if human detected < 2m | Must |
| FR-SAF-02 | Stop if animal detected < 1.5m | Must |
| FR-SAF-03 | Slow down near boundary (< 50cm) | Should |
| FR-SAF-04 | Hard stop at boundary | Must |
| FR-SAF-05 | Pause if pose uncertainty too high | Should |

---

## 3. Non-Functional Requirements

### 3.1 Performance (NFR-PERF)

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-PERF-01 | Coverage planner computation | < 5 sec for 1000 m^2 |
| NFR-PERF-02 | Nav2 cmd_vel output rate | >= 20 Hz |
| NFR-PERF-03 | Obstacle costmap update rate | >= 5 Hz |
| NFR-PERF-04 | Total CPU usage | < 40% (with sensors) |

### 3.2 Reliability (NFR-REL)

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-REL-01 | Session completion rate | >= 80% |
| NFR-REL-02 | Boundary violation rate | < 1% |
| NFR-REL-03 | Collision rate | 0% (detected obstacles) |

### 3.3 Usability (NFR-USE)

| ID | Requirement | Priority |
|----|-------------|----------|
| NFR-USE-01 | Boundary recording < 10 min for 500 m^2 | Should |
| NFR-USE-02 | Clear status feedback during mowing | Must |
| NFR-USE-03 | Progress percentage reported | Should |

---

## 4. Interface Specifications

### 4.1 Subscribed Topics

| Topic | Message Type | Source | Purpose |
|-------|--------------|--------|---------|
| /odometry/filtered | nav_msgs/Odometry | Sensor Integration | Robot pose |
| /vision/detections | Detection2DArray | AI Camera | Obstacles |
| /vision/detections_3d | Detection3DArray | AI Camera | 3D obstacles |

### 4.2 Published Topics

| Topic | Message Type | Rate | Purpose |
|-------|--------------|------|---------|
| /cmd_vel | geometry_msgs/Twist | 20 Hz | Motor commands |
| /boundary/polygon | geometry_msgs/PolygonStamped | 1 Hz | Current boundary |
| /nav/path | nav_msgs/Path | On change | Coverage path |
| /mower/status | std_msgs/String | 1 Hz | Status text |

### 4.3 Services

| Service | Type | Purpose |
|---------|------|---------|
| /mower/start | std_srvs/Trigger | Begin mowing |
| /mower/pause | std_srvs/Trigger | Pause mowing |
| /mower/stop | std_srvs/Trigger | Stop and reset |

> **Note:** Boundary recording services (`map_start`, `map_stop`, `map_save`, `map_load`, `map_list`) are handled by `mower_mapping` via WebSocket on port 8770. See [bmad-mower-mapping](../bmad-mower-mapping/README.md).

---

## 5. Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| nav2_bringup | Jazzy | Nav2 launch |
| nav2_bt_navigator | Jazzy | Behavior trees |
| nav2_controller | Jazzy | Path following |
| nav2_planner | Jazzy | Global planning |
| nav2_costmap_2d | Jazzy | Obstacle mapping |
| nav2_dwb_controller | Jazzy | Local planner |

---

## 6. Testing Strategy

### 6.1 Unit Tests
- Boundary polygon validation
- Boustrophedon path generation
- Twist to motor conversion

### 6.2 Integration Tests
- Coverage planner with sample polygons
- Nav2 with simulated obstacles
- End-to-end path following

### 6.3 Field Tests
- Boundary recording accuracy
- Coverage completeness (drone photo)
- Obstacle avoidance with mannequin

---

## 7. Acceptance Criteria Summary

The Navigation subsystem is complete when:

1. User can record and save boundary by driving perimeter
2. Coverage planner generates valid boustrophedon path
3. Robot follows path within +/- 20 cm
4. Robot avoids all detected obstacles
5. Robot stops for human/animal detections
6. Coverage efficiency >= 90%
7. Documentation and tests complete
