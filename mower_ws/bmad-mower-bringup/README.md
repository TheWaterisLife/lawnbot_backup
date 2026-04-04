# BMAD: Mower Bringup Subsystem

## Overview

This BMAD project covers the **Mower Bringup** package — the top-level launch orchestrator for the autonomous lawn mower. It brings up all subsystems (sensors, motors, navigation, camera, comms) in the correct order with proper configuration.

## Scope

**In Scope:**
- Top-level launch files to start the complete mower system
- Launch configuration for each subsystem
- System-wide parameter management
- Startup sequencing and dependency ordering

**Out of Scope:**
- Individual subsystem logic (each handled by its own package)
- Motor hardware control (see [bmad-lawnbot-motors](../bmad-lawnbot-motors/README.md))
- Sensor fusion (see [bmad-integration](../bmad-integration/README.md))
- Navigation (see [bmad-navigation](../bmad-navigation/README.md))

## Current Status

> [!NOTE]
> This package is currently a **skeleton** — the launch files and orchestration logic are planned but not yet implemented.

## Subsystems to Orchestrate

| Subsystem | Package | Launch Requirement |
|-----------|---------|-------------------|
| RTK GPS | `rtk_reader` | Start first (GPS fix needed) |
| Sensor Integration | `sensor_integration` | Start after RTK (needs `/rtk/fix`) |
| Motor Server | `lawnbot_motors` | Standalone WebSocket (port 8766) |
| Camera Vision | `ai_camera_vision` | Independent (OAK-D) |
| Comms Bridge | `lawnbot_comms` | Needs ROS2 topics active |
| Navigation | `mower_navigation` | Start last (needs all inputs) |

## BMAD Documents

| Document | Status | Description |
|----------|--------|-------------|
| [product-brief.md](product-brief.md) | ✅ Done | High-level vision |
| [PRD.md](PRD.md) | ✅ Done | Requirements & acceptance criteria |
| [architecture.md](architecture.md) | ✅ Done | Technical design |

## Planned Quick Start

```bash
# Build all packages
cd ~/mower_ws
colcon build

# Launch everything
ros2 launch mower_bringup full_system.launch.py

# Launch sensors only (testing)
ros2 launch mower_bringup sensors_only.launch.py

# Launch teleop mode (manual drive)
ros2 launch mower_bringup teleop.launch.py
```

## Related Subsystems

| Subsystem | Description | Link |
|-----------|-------------|------|
| RTK Reader | GPS positioning | [bmad-rtk-reader](../bmad-rtk-reader/README.md) |
| Integration | Sensor fusion | [bmad-integration](../bmad-integration/README.md) |
| Motors | Motor control | [bmad-lawnbot-motors](../bmad-lawnbot-motors/README.md) |
| Comms | App bridge | [bmad-lawnbot-comms](../bmad-lawnbot-comms/README.md) |
| Navigation | Path planning | [bmad-navigation](../bmad-navigation/README.md) |
| Camera | AI vision | [bmad-camera](../bmad-camera/README.md) |
