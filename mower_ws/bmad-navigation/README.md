# BMAD: Navigation Subsystem

## Overview

This BMAD project covers the **Navigation** subsystem of the autonomous lawn mower. It consumes the fused pose from the Integration subsystem and obstacle data from the Camera subsystem to plan and execute complete lawn coverage.

## Scope

**In Scope:**
- Boustrophedon coverage path planning (with concave polygon support)
- Nav2 integration for obstacle avoidance
- Motor command interface (/cmd_vel → WebSocket to lawnbot_motors)
- Blade motor control
- Path resumption after interruption
- Motor diagnostics and health monitoring

**Out of Scope:**
- Sensor fusion (see [bmad-integration](../bmad-integration/README.md))
- Boundary recording (see [bmad-mower-mapping](../bmad-mower-mapping/README.md))
- Camera/obstacle detection (see bmad-camera)
- Mobile app (separate project)

## Dependencies

| Subsystem | Package | Topics Used |
|-----------|---------|-------------|
| Integration | sensor_integration | `/odometry/filtered` |
| Camera | ai_camera_vision | `/vision/detections`, `/vision/detections_3d` |

## Hardware Reference

For sensor wiring and hardware pinouts (Encoders, IMU, GPS), please refer to [bmad-integration/README.md](../bmad-integration/README.md).

### Motor Hardware Configuration (BTS7960 H-Bridge via gpiozero)

| Motor | R_EN | L_EN | R_PWM | L_PWM | Max Duty |
|-------|------|------|-------|-------|----------|
| **Right** | GPIO 5 | GPIO 6 | GPIO 19 | GPIO 12 | 1.0 |
| **Left** | GPIO 23 | GPIO 24 | GPIO 18 | GPIO 13 | 1.0 |
| **Blade** | GPIO 4 | GPIO 21 | GPIO 16 | GPIO 20 | 1.0 |

> **Note:** Motor control is handled by the `lawnbot_motors` package (standalone WebSocket server on port 8766). Uses `gpiozero` (`PWMOutputDevice` + `DigitalOutputDevice`) for Pi 5 compatibility. See [bmad-lawnbot-motors](../bmad-lawnbot-motors/README.md) for details.

## BMAD Documents

| Document | Status | Description |
|----------|--------|-------------|
| [product-brief.md](product-brief.md) | ✅ Done | High-level vision |
| [PRD.md](PRD.md) | ✅ Done | Requirements & acceptance criteria |
| [architecture.md](architecture.md) | ✅ Done | Technical design |
| [epics/](epics/) | ✅ Done | Feature groupings |
| [stories/](stories/) | ✅ Done | Implementation tasks |

## ROS2 Nodes

| Node | Description | Topics |
|------|-------------|--------|
| `coverage_planner_node` | Coverage path generation | `/coverage/path` |
| `motor_controller_node` | Motor control via WebSocket to lawnbot_motors | `/cmd_vel` |
| `obstacle_handler_node` | Obstacle detection handling | `/vision/detections` |
| `navigation_controller_node` | High-level mowing state machine | - |
| `camera_obstacle_publisher` | Camera to Nav2 costmap bridge | `/vision/obstacles` |
| `boundary_costmap_publisher` | Boundary virtual fence (uses mower_mapping) | `/boundary/fence` |
| `motor_diagnostics_node` | Motor health monitoring | `/diagnostics` |

## Quick Start

```bash
# Build the packages
cd ~/mower_ws
colcon build --packages-select mower_mapping mower_navigation

# Record a boundary (see bmad-mower-mapping)
ros2 run mower_mapping map_server

# Run autonomous mowing (loads boundary from boundary_mapper)
ros2 launch mower_navigation mow.launch.py zone:=front_yard

# Monitor motor health
ros2 run mower_navigation motor_diagnostics_node
```

## Key Features

### 1. Coverage Planning
- Boustrophedon (back-and-forth) pattern
- **Concave polygon handling** (Story 2.2)
- **Sweep direction optimization** (Story 2.3)
- **Path resumption after battery recharge** (Story 2.4)
- Configurable row spacing (mowing width)

### 3. Obstacle Avoidance
- Real-time costmap from camera detections
- **Camera obstacle layer** (Story 3.3)
- **Boundary virtual fence** (Story 3.5)
- Nav2 local planner (DWB)
- Safety stop for humans/animals

### 4. Motor Control
- Delegated to `lawnbot_motors` WebSocket server (port 8766)
- Navigation node connects as WebSocket client
- **Motor calibration** (Story 4.6)
- **Motor diagnostics** (Story 4.7) - stall detection, health monitoring

## Implementation Status

| Epic | Stories | Status |
|------|---------|--------|
| **Epic 1: Boundary Management** | - | ➡️ Moved to [bmad-mower-mapping](../bmad-mower-mapping/README.md) |
| **Epic 2: Coverage Planning** | 2.1-2.6 | ✅ Complete |
| **Epic 3: Nav2 Integration** | 3.1-3.7 | ✅ Complete |
| **Epic 4: Motor Control** | 4.1-4.7 | ✅ Complete |

## Success Criteria

1. ✅ Complete 90%+ lawn coverage
2. ✅ Stay within boundary (+/- 20 cm)
3. ✅ Avoid all detected obstacles
4. ✅ Resume mowing after interruption
5. ✅ Handle concave lawn shapes
6. ✅ Monitor motor health
