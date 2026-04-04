# Mower Navigation Package

ROS 2 package for autonomous lawn mower navigation.

## Overview

This package provides:
- **Boundary Management** - Learn and store lawn boundaries
- **Coverage Planning** - Boustrophedon mowing pattern
- **Motor Control** - PWM control for drive motors
- **Obstacle Handling** - Integration with camera vision
- **Navigation Control** - Nav2 integration for path following

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select mower_navigation
source install/setup.bash
```

### Nav2 Dependencies

```bash
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs
```

## Usage

### Start Navigation (All Nodes)

```bash
ros2 launch mower_navigation navigation.launch.py
```

### Start Nav2 Stack (Separate)

```bash
ros2 launch mower_navigation nav2.launch.py
```

### Learn Boundary

```bash
# Start learning mode
ros2 service call /boundary/start_learning std_srvs/srv/SetBool "{data: true}"

# Drive the mower around the lawn perimeter
# Points are automatically recorded from /gps/fix

# Finish and save
ros2 service call /boundary/finish_learning std_srvs/srv/Trigger
```

### Generate Coverage Path

```bash
ros2 service call /coverage/generate std_srvs/srv/Trigger
```

### Start Autonomous Mowing

```bash
# Enable motors
ros2 service call /motors/enable std_srvs/srv/Trigger

# Start navigation (follows coverage path via Nav2)
ros2 service call /navigation/start std_srvs/srv/Trigger
```

### Emergency Stop

```bash
ros2 service call /motors/emergency_stop std_srvs/srv/Trigger
```

## Topics

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/gps/fix` | NavSatFix | GPS for boundary learning |
| `/cmd_vel` | Twist | Velocity commands |
| `/cmd_vel_raw` | Twist | Raw velocity (filtered by obstacles) |
| `/vision/detections` | Detection2DArray | Camera detections |

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/boundary/polygon` | PolygonStamped | Current boundary |
| `/boundary/status` | String | JSON status |
| `/coverage/path` | Path | Coverage path |
| `/coverage/status` | String | JSON status |
| `/motors/status` | String | JSON status |
| `/motors/speeds` | Float32MultiArray | [left, right] speeds |
| `/obstacles/alert` | Bool | Obstacle in path |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/boundary/start_learning` | SetBool | Start boundary learning |
| `/boundary/finish_learning` | Trigger | Finish and save |
| `/boundary/cancel_learning` | Trigger | Cancel learning |
| `/coverage/generate` | Trigger | Generate path |
| `/coverage/start` | Trigger | Start mowing |
| `/coverage/pause` | Trigger | Pause/resume |
| `/coverage/stop` | Trigger | Stop mowing |
| `/motors/enable` | Trigger | Enable motors |
| `/motors/disable` | Trigger | Disable motors |
| `/motors/emergency_stop` | Trigger | Emergency stop |

## Parameters

See `config/navigation.yaml` for all parameters.

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cutting_width` | 0.30 | Cutting deck width (m) |
| `overlap_ratio` | 0.10 | Overlap between passes |
| `track_width` | 0.24 | Robot track width (m) |
| `max_linear_velocity` | 0.8 | Max speed (m/s) |
| `critical_distance` | 0.3 | Emergency stop distance (m) |

## Obstacle Avoidance

The obstacle handler provides intelligent avoidance with recovery logic:

### Avoidance Actions

| Action | Description | When Triggered |
|--------|-------------|----------------|
| `NONE` | Path clear, proceed | No obstacles in path |
| `SLOW_DOWN` | Reduce speed | Obstacle straight ahead in warning zone |
| `TURN_LEFT` | Gentle turn left | Obstacle to the right, left is clear |
| `TURN_RIGHT` | Gentle turn right | Obstacle to the left, right is clear |
| `PIVOT_LEFT` | Tank turn in place | Recovery mode, pivot left 90° |
| `PIVOT_RIGHT` | Tank turn in place | Recovery mode, pivot right 90° |
| `PIVOT_180` | Full U-turn in place | Both sides blocked, turn around |
| `STOP` | Emergency stop | Obstacle in critical zone |

### Recovery Logic

1. When stopped for > 2 seconds, enters **recovery mode**
2. Recovery mode triggers a pivot turn to avoid obstacle
3. Turn direction considers **boundary constraints** first
4. If both boundaries blocked → PIVOT_180 (turn around, skip to next line)

### Boundary Integration

The navigation system should call:
```python
handler.set_boundary_constraints(left_blocked=True, right_blocked=False)
```
This prevents turning into a boundary edge.

## Integration with Camera

```
/vision/detections → obstacle_handler_node → /cmd_vel (filtered)
```

Critical objects (person, animal, vehicle) trigger emergency stop.

## Visualization (RViz)

Add these displays:
- `Polygon`: `/boundary/polygon`
- `Path`: `/coverage/path`
- `MarkerArray`: `/boundary/markers`, `/coverage/markers`

