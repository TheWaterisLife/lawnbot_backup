# Architecture: Navigation Subsystem

## 1. System Context

```
+------------------------------------------------------------------+
|                    Autonomous Lawn Mower                         |
+------------------------------------------------------------------+
|                                                                  |
|  +------------------+     +------------------+                   |
|  |Sensor Integration|     | AI Camera Vision |                   |
|  |/odometry/filtered|     |/vision/detections|                   |
|  +--------+---------+     +---------+--------+                   |
|           |                         |                            |
|           v                         v                            |
|  +--------------------------------------------------+           |
|  |            NAVIGATION SUBSYSTEM                   |           |
|  |                (this project)                     |           |
|  |                                                   |           |
|  |  +------------+    +-----------+    +---------+  |           |
|  |  |  Boundary  |--->|  Coverage |--->|  Nav2   |  |           |
|  |  |  Manager   |    |  Planner  |    |  Stack  |  |           |
|  |  +------------+    +-----------+    +----+----+  |           |
|  |                                          |       |           |
|  |                                          v       |           |
|  |                                    +---------+   |           |
|  |                                    |  Motor  |   |           |
|  |                                    | Control |   |           |
|  |                                    +----+----+   |           |
|  +--------------------------------------------------+           |
|                                            |                     |
+--------------------------------------------+---------------------+
                                             |
                                             v
                                    +----------------+
                                    | Motor Drivers  |
                                    | (H-Bridge PWM) |
                                    +----------------+
```

## 2. Component Architecture

### 2.1 Package Structure

> **Note:** Boundary recording and management has been moved to the `mower_mapping` package.
> See [bmad-mower-mapping](../bmad-mower-mapping/README.md) for details.

```
src/mower_navigation/
|-- mower_navigation/
|   |-- __init__.py
|   |-- coverage_planner.py       # Boustrophedon path generation (concave support)
|   |-- coverage_planner_node.py  # ROS2 node wrapper for coverage planner
|   |-- motor_controller.py       # PWM + MCP23017 motor control
|   |-- motor_controller_node.py  # ROS2 node wrapper
|   |-- motor_diagnostics.py      # Motor health monitoring
|   |-- obstacle_handler.py       # Obstacle detection handling
|   |-- obstacle_handler_node.py  # ROS2 node wrapper
|   |-- navigation_controller.py  # High-level state machine
|   |-- navigation_controller_node.py
|   |-- camera_obstacle_publisher.py   # Camera → Nav2 costmap bridge
|   +-- boundary_costmap_publisher.py  # Boundary virtual fence (uses mower_mapping)
|
|-- config/
|   |-- nav2_params.yaml          # Nav2 configuration
|   +-- motor_calibration.yaml    # Motor calibration data
|
|-- docs/
|   +-- MOTOR_CALIBRATION.md      # Motor calibration guide
|
|-- launch/
|   |-- navigation.launch.py      # Full navigation stack
|   +-- mow.launch.py             # Complete mowing session
|
|-- test/
|   |-- test_coverage.py
|   +-- test_motor_control.py
|
|-- package.xml
|-- setup.py
+-- setup.cfg
```

### 2.2 Node Descriptions

#### ~~Boundary Node~~ (Moved to mower_mapping)

> ⚠️ **DEPRECATED**: The boundary recording node has been moved to the `mower_mapping` package.
> See [bmad-mower-mapping/architecture.md](../bmad-mower-mapping/architecture.md) for the current design.

#### Coverage Planner (`coverage_planner.py`)

**Responsibility:** Generate boustrophedon coverage path for polygon

**Class Diagram:**
```
+---------------------------+
|   CoveragePlanner         |
+---------------------------+
| - polygon: Polygon        |
| - row_spacing: float      |
| - path: list[Point]       |
+---------------------------+
| + set_polygon(polygon)    |
| + plan_coverage()         |
| + get_path()              |
| + get_progress()          |
| + save_progress()         |
| + resume_from(waypoint)   |
+---------------------------+
           |
           | uses
           v
+---------------------------+
|   BoustrophedonGenerator  |
+---------------------------+
| + decompose(polygon)      |
| + generate_rows(cell)     |
| + connect_rows(rows)      |
| + optimize_order(cells)   |
+---------------------------+
```

**Algorithm:**
1. Find optimal sweep direction (minimize turns)
2. Generate parallel lines at row_spacing intervals
3. Clip lines to polygon boundary
4. Connect line endpoints (alternating direction)
5. Add smooth turns at endpoints

#### Motor Control Node (`motor_controller_node.py`)

**Responsibility:** Convert /cmd_vel to motor speeds via WebSocket to lawnbot_motors server

> **Note:** Motor control is delegated to the `lawnbot_motors` package (standalone WebSocket server on port 8766). The navigation node connects as a WebSocket client and sends `drive`/`stop` commands. See [bmad-lawnbot-motors](../bmad-lawnbot-motors/architecture.md) for hardware details.

**Class Diagram:**
```
+---------------------------+
|   MotorController         |
|   (WebSocket Client)      |
+---------------------------+
| - ws_host: str            |
| - ws_port: int            |
| - _ws: WebSocket          |
| - _connected: bool        |
| - track_width: float      |
+---------------------------+
| + start() -> connect WS   |
| + stop() -> send stop cmd |
| + set_speeds(left, right) |
| + set_velocity(vx, wz)    |
| + emergency_stop()        |
+---------------------------+
         |
         | WebSocket (ws://localhost:8766)
         v
+---------------------------+
| lawnbot_motors server     |
| (ws_motor_server.py)      |
+---------------------------+
| BTS7960Motor (gpiozero)   |
+---------------------------+
```

**WebSocket Commands Used:**
| Command | Payload | When |
|---------|---------|------|
| `drive` | `{"cmd":"drive","left":0.5,"right":0.5}` | set_speeds() |
| `stop` | `{"cmd":"stop"}` | stop(), emergency_stop() |

#### Motor Diagnostics Node (`motor_diagnostics.py`)

**Responsibility:** Monitor motor health and publish to /diagnostics

**Features:**
- Stall detection (motor commanded but not moving)
- Speed mismatch warning
- BTS7960 / gpiozero GPIO health check
- ROS2 DiagnosticArray publishing

## 3. Data Flow

### 3.1 Mowing Session Sequence

```
    User         BoundaryNode   CoveragePlanner   Nav2      MotorControl
      |               |               |             |             |
      |--start_mow--->|               |             |             |
      |               |--load_zone--->|             |             |
      |               |               |             |             |
      |               |<--polygon-----|             |             |
      |               |               |             |             |
      |               |---plan_path-->|             |             |
      |               |               |--path------>|             |
      |               |               |             |             |
      |               |               |             |--follow---->|
      |               |               |             |             |
      |               |               |             |<-cmd_vel----|
      |               |               |             |             |
      |               |               |             |---/cmd_vel->|
      |               |               |             |             |--PWM-->
```

### 3.2 Obstacle Avoidance Flow

```
    Camera       ObstacleCostmap    Nav2Costmap    DWBPlanner   MotorControl
      |               |                 |              |             |
      |--detections-->|                 |              |             |
      |               |--mark_obstacle->|              |             |
      |               |                 |--update----->|             |
      |               |                 |              |--replan---->|
      |               |                 |              |             |
      |               |                 |              |<-new_cmd----|
      |               |                 |              |             |
      |               |                 |              |---cmd_vel-->|
```

## 4. Nav2 Configuration

### 4.1 Navigation Stack Components

```
+----------------------------------------------------------+
|                     Nav2 Stack                            |
+----------------------------------------------------------+
|                                                          |
|  +------------+    +-------------+    +--------------+   |
|  | BT Navigator|-->| Controller  |--->| FollowPath   |   |
|  |            |    | Server      |    | (DWB)        |   |
|  +------------+    +-------------+    +--------------+   |
|       |                                      |           |
|       v                                      v           |
|  +------------+                       +--------------+   |
|  | Planner    |                       |  Costmap2D   |   |
|  | Server     |                       |  (Local)     |   |
|  +------------+                       +--------------+   |
|       |                                      ^           |
|       v                                      |           |
|  +------------------+              +------------------+  |
|  | Coverage Planner |              | Obstacle Layer   |  |
|  | (Custom Plugin)  |              | (Camera Feed)    |  |
|  +------------------+              +------------------+  |
|                                                          |
+----------------------------------------------------------+
```

### 4.2 Costmap Layers

| Layer | Purpose | Update Rate |
|-------|---------|-------------|
| Static | Boundary (virtual fence) | Once |
| Obstacle | Camera detections | 5 Hz |
| Inflation | Safety buffer | On obstacle change |

### 4.3 DWB Controller Tuning

```yaml
# For slow, precise lawn mowing
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.45
      min_vel_x: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.1
      max_speed_xy: 0.45
      acc_lim_x: 0.5
      acc_lim_theta: 1.5
      decel_lim_x: -1.0
      xy_goal_tolerance: 0.10
      yaw_goal_tolerance: 0.1
```

## 5. State Machine

### 5.1 Mower States

```
                    +-------+
                    | IDLE  |
                    +---+---+
                        |
              start_mow |
                        v
                +-------+-------+
                | LOADING_ZONE  |
                +-------+-------+
                        |
                        v
                +-------+-------+
                | PLANNING_PATH |
                +-------+-------+
                        |
                        v
               +--------+--------+
        +----->|     MOWING      |<----+
        |      +--------+--------+     |
        |               |              |
        | resume        | obstacle     | clear
        |               v              |
        |      +--------+--------+     |
        +------+     PAUSED      +-----+
               +--------+--------+
                        |
                        | stop / complete
                        v
                +-------+-------+
                | RETURNING_HOME|
                +-------+-------+
                        |
                        v
                    +---+---+
                    | IDLE  |
                    +-------+
```

### 5.2 Safety State Overrides

| Detection | Action | Return Condition |
|-----------|--------|------------------|
| Human < 2m | EMERGENCY_STOP | Human leaves FOV |
| Animal < 1.5m | PAUSE | Animal > 2m away |
| Unknown obstacle | SLOW + DEVIATE | Clear path found |
| Boundary < 20cm | STOP + TURN | Heading inward |
| Pose uncertain | PAUSE | EKF recovers |

## 6. Boundary File Management

### 6.1 File Structure

The `mower_mapping` package manages map files in the `~/mower_ws/maps/` directory.

```
~/mower_ws/maps/
|-- front_yard_20260210_183000.json
|-- back_yard_20260210_190000.json
+-- side_strip_20260211_100000.json
```

See [bmad-mower-mapping/architecture.md](../bmad-mower-mapping/architecture.md) for details on the file format and management.

## 7. Performance Considerations

### 7.1 CPU Budget (with Sensor Integration)

| Component | Target CPU | Notes |
|-----------|------------|-------|
| Sensor Integration | 16% | IMU, GPS, Odom, EKF |
| AI Camera Vision | 15% | On VPU, minimal host |
| Nav2 Stack | 15% | Costmap, planner, controller |
| Motor Control | 2% | Simple PWM |
| Coverage Planner | 2% | One-time computation |
| **Total** | **50%** | Headroom for peaks |

### 7.2 Memory Budget

| Component | Target RAM |
|-----------|------------|
| Sensor Integration | 120 MB |
| AI Camera Vision | 100 MB |
| Nav2 Stack | 200 MB |
| Motor/Coverage | 30 MB |
| **Total** | **450 MB** |

Raspberry Pi 5 has 8 GB RAM - plenty of headroom.

## 8. Launch Configuration

### 8.1 Full Mowing Launch

```python
# mow.launch.py
def generate_launch_description():
    return LaunchDescription([
        # Sensor Integration (from other package)
        IncludeLaunchDescription(
            'sensor_integration', 'sensors.launch.py'
        ),
        
        # AI Camera Vision (existing)
        IncludeLaunchDescription(
            'ai_camera_vision', 'camera.launch.py'
        ),
        
        # Mapping server (boundary recording/loading)
        Node(
            package='mower_mapping',
            executable='map_server',
        ),
        
        # Navigation controller
        Node(
            package='mower_navigation',
            executable='motor_control_node',
        ),
        
        # Nav2
        IncludeLaunchDescription(
            'nav2_bringup', 'navigation_launch.py',
            launch_arguments={
                'params_file': nav2_params_file,
            }.items()
        ),
    ])
```

> **Note:** `lawnbot_motors` (motor WebSocket server on port 8766) runs as a standalone process, not a ROS2 node. Start separately via `python3 ws_motor_server.py` or a systemd service.
