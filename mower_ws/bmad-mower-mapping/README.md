# BMAD: Mower Mapping Subsystem

## Overview

This BMAD project covers the **Mower Mapping** subsystem — a ROS2 node with a WebSocket interface that records lawn boundaries using RTK GPS. The operator walks the perimeter while the system records GPS points, simplifies the path, and saves the boundary as a JSON map file.

## Scope

**In Scope:**
- GPS boundary recording from `/rtk/fix`
- Adaptive point sampling (distance, heading, time triggers)
- Ramer-Douglas-Peucker (RDP) path simplification
- Loop closure detection
- Auto-pause/resume on GPS loss
- WebSocket server for app control (port 8770)
- Map save/load/list to `~/mower_ws/maps/`
- Real-time preview broadcast to connected clients

**Out of Scope:**
- GPS hardware (see [bmad-rtk-reader](../bmad-rtk-reader/README.md))
- Navigation within boundaries (see [bmad-navigation](../bmad-navigation/README.md))
- Costmap boundary fence (see [bmad-navigation](../bmad-navigation/README.md))

## BMAD Documents

| Document | Status | Description |
|----------|--------|-------------|
| [product-brief.md](product-brief.md) | ✅ Done | High-level vision |
| [PRD.md](PRD.md) | ✅ Done | Requirements & acceptance criteria |
| [architecture.md](architecture.md) | ✅ Done | Technical design |

## Quick Start

```bash
# Build
cd ~/mower_ws
colcon build --packages-select mower_mapping

# Run (requires rtk_reader publishing /rtk/fix)
ros2 run mower_mapping map_server
```

Connect from mobile app to `ws://<pi-ip>:8770`.

## Mapping Workflow

1. **Start**: App sends `map_start` → operator walks perimeter
2. **Record**: GPS points sampled adaptively as you walk
3. **Stop**: App sends `map_stop` → path simplified with RDP
4. **Save**: App sends `map_save` with a name → JSON saved to disk
5. **Load**: App sends `map_load` → boundary retrieved for navigation

## WebSocket Commands

| Command | Description |
|---------|-------------|
| `map_start` | Begin recording boundary |
| `map_stop` | Stop recording, build simplified boundary |
| `map_save` | Save last built map to disk |
| `map_list` | List all saved maps |
| `map_load` | Load a saved map by ID |
| `map_cancel` | Cancel recording, discard points |

## Sampling Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `MIN_DIST_M` | 0.25 m | Min distance to record new point |
| `MIN_HEADING_DEG` | 8° | Heading change triggers new point |
| `MAX_TIME_S` | 1.0 s | Max time between points |
| `MIN_TIME_S` | 0.2 s | Min time between points |
| `RDP_EPS_M` | 0.08 m | RDP simplification tolerance |
| `CLOSE_LOOP_DIST_M` | 0.50 m | Auto-close loop distance |
| `MIN_POINTS` | 20 | Minimum points for valid boundary |

## Related Subsystems

| Subsystem | Description | Link |
|-----------|-------------|------|
| RTK Reader | GPS data source | [bmad-rtk-reader](../bmad-rtk-reader/README.md) |
| Navigation | Costmap virtual fence | [bmad-navigation](../bmad-navigation/README.md) |
| Navigation | Path planning | [bmad-navigation](../bmad-navigation/README.md) |
