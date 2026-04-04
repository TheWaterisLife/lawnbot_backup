# Epic 3: Nav2 Integration

## Goal
Configure and integrate the Nav2 navigation stack for path following with obstacle avoidance, using the fused pose from Sensor Integration and obstacle data from AI Camera Vision.

## Background
Nav2 is the ROS 2 navigation stack that provides:
- Behavior Tree-based task execution
- Controller for path following (DWB)
- Costmap for obstacle representation
- Recovery behaviors

We configure Nav2 to follow coverage paths while avoiding obstacles detected by the camera.

## Stories

### Story 3.1: Nav2 Basic Configuration
**As a** developer  
**I want** Nav2 configured for the mower  
**So that** it can follow paths

**Acceptance Criteria:**
- [ ] nav2_params.yaml configured for mower specs
- [ ] Robot footprint defined (42x28 cm)
- [ ] Velocity limits set (max 0.45 m/s)
- [ ] Acceleration limits set
- [ ] Launch file starts Nav2 stack

**Key Parameters:**
```yaml
robot_radius: 0.20  # Inscribed
robot_base_frame: base_link
odom_topic: /odom_filtered
max_vel_x: 0.45
max_vel_theta: 1.0
```

---

### Story 3.2: Local Costmap Configuration
**As a** navigation system  
**I want** a local costmap around the robot  
**So that** I can avoid nearby obstacles

**Acceptance Criteria:**
- [ ] Rolling costmap (4m x 4m)
- [ ] 5 cm resolution
- [ ] Updates at 5 Hz
- [ ] Clears automatically when obstacles move
- [ ] Inflation layer for safety margin

**Costmap Layers:**
1. Obstacle Layer (camera detections)
2. Inflation Layer (30 cm radius)

---

### Story 3.3: Camera Obstacle Layer
**As a** costmap  
**I want** obstacles from camera detections  
**So that** the planner avoids them

**Acceptance Criteria:**
- [ ] Subscribe to /vision/detections_3d
- [ ] Mark obstacle cells at detection positions
- [ ] Obstacle size from bounding box
- [ ] Safety category affects cost (human > animal > object)
- [ ] Obstacles clear after timeout (2 sec)

**Cost Values:**
| Detection | Cost | Inflation |
|-----------|------|-----------|
| Human | 254 (lethal) | 50 cm |
| Animal | 254 (lethal) | 40 cm |
| Vehicle | 254 (lethal) | 50 cm |
| Other | 200 (high) | 30 cm |

---

### Story 3.4: DWB Controller Tuning
**As a** mower  
**I want** smooth, precise path following  
**So that** I mow in straight lines

**Acceptance Criteria:**
- [ ] Smooth velocity profiles (no jerking)
- [ ] Stays within 10 cm of planned path
- [ ] Handles sharp turns at row ends
- [ ] Slows for obstacles, speeds up when clear
- [ ] Goal tolerance: 10 cm position, 0.1 rad heading

**Tuning Parameters:**
- PathDistCritic weight: high (stay on path)
- GoalDistCritic weight: medium
- ObstacleFootprintCritic weight: high
- RotateToGoalCritic: enabled for row ends

---

### Story 3.5: Boundary Virtual Fence
**As a** safety system  
**I want** the boundary as a costmap obstacle  
**So that** the mower cannot leave the lawn

**Acceptance Criteria:**
- [ ] Boundary polygon marked as lethal in costmap
- [ ] 20 cm buffer inside boundary
- [ ] Updates when boundary changes
- [ ] Hard stop if within 10 cm of boundary

**Implementation:**
- Add boundary as static obstacle layer
- Inflate inward (not outward)
- High cost approaching boundary

---

### Story 3.6: Recovery Behaviors
**As a** mower stuck on an obstacle  
**I want** recovery behaviors  
**So that** I can continue mowing

**Acceptance Criteria:**
- [ ] Spin in place (look for clear path)
- [ ] Back up and retry
- [ ] Wait for obstacle to move (30 sec timeout)
- [ ] If all fail, pause and alert user
- [ ] Never cross boundary during recovery

**Recovery Sequence:**
1. Spin 90 degrees, replan
2. Back up 30 cm, replan
3. Wait 30 seconds, replan
4. Pause and request help

---

### Story 3.7: Path Following Launch
**As a** developer  
**I want** a launch file for path following  
**So that** I can test navigation independently

**Acceptance Criteria:**
- [ ] Launch Nav2 with mower config
- [ ] Accept path from coverage planner
- [ ] Execute path with obstacle avoidance
- [ ] Report progress and completion

## Definition of Done
- All stories completed
- Nav2 follows coverage path
- Avoids camera-detected obstacles
- Stays within boundary
- Recovery behaviors work
- Tests passing
- Code reviewed
