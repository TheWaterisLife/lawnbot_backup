# BMAD: Lawnbot Comms Subsystem

## Overview

This BMAD project covers the **Lawnbot Comms** subsystem — a WebSocket bridge that connects the mobile app to the ROS2 system. It enables remote teleoperation, mode switching, and real-time telemetry streaming.

## Scope

**In Scope:**
- WebSocket server on port 9002
- Mobile app → ROS2: heartbeat, mode commands, teleop joystick
- ROS2 → Mobile app: mower state, telemetry data
- Single-client connection management
- Fake telemetry node for testing

**Out of Scope:**
- Mobile app UI (separate project)
- Motor control logic (see [bmad-navigation](../bmad-navigation/README.md))
- Sensor data (see [bmad-integration](../bmad-integration/README.md))

## BMAD Documents

| Document | Status | Description |
|----------|--------|-------------|
| [product-brief.md](product-brief.md) | ✅ Done | High-level vision |
| [PRD.md](PRD.md) | ✅ Done | Requirements & acceptance criteria |
| [architecture.md](architecture.md) | ✅ Done | Technical design |

## ROS2 Nodes

| Node | Executable | Description |
|------|------------|-------------|
| `lawnbot_bridge` | `bridge_node` | WebSocket ↔ ROS2 bridge |
| `fake_telemetry` | `fake_telemetry_node` | Test telemetry publisher (5 Hz) |

## Topics

### Published (Phone → ROS2)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/app/heartbeat` | `std_msgs/Empty` | App connection keepalive |
| `/app/mode` | `std_msgs/String` | Mode command (IDLE, TELEOP, AUTO, etc.) |
| `/app/teleop_cmd` | `geometry_msgs/Twist` | Joystick control (linear.x, angular.z) |

### Subscribed (ROS2 → Phone)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/mower/state` | `std_msgs/String` | Mower state (mode, armed, estop) |
| `/mower/telemetry` | `std_msgs/String` | Telemetry data (wifi_ip, etc.) |

## Quick Start

```bash
# Build
cd ~/mower_ws
colcon build --packages-select lawnbot_comms

# Run the bridge
ros2 run lawnbot_comms bridge_node

# Test with fake telemetry
ros2 run lawnbot_comms fake_telemetry_node
```

Connect from mobile app to `ws://<pi-ip>:9002`.

## WebSocket Protocol

### Phone → Mower (JSON)

| Type | Fields | Description |
|------|--------|-------------|
| `hello` | — | Handshake |
| `heartbeat` | — | Keepalive |
| `mode` | `value` | Set mode (e.g., "TELEOP") |
| `teleop` | `linear`, `angular` | Joystick command |

### Mower → Phone (JSON)

| Type | Fields | Description |
|------|--------|-------------|
| `ack` | `msg`, `version` | Connection ack |
| `state` | `data` | Mower state string |
| `telemetry` | `data` | Telemetry string |
| `comms` | `heartbeat_age_ms` | Connection health |

## Related Subsystems

| Subsystem | Description | Link |
|-----------|-------------|------|
| Navigation | Consumes teleop commands | [bmad-navigation](../bmad-navigation/README.md) |
| Integration | Sensor data source | [bmad-integration](../bmad-integration/README.md) |
