# BMAD: Lawnbot Motors Subsystem

## Overview

This BMAD project covers the **Lawnbot Motors** subsystem — a standalone WebSocket server that directly controls the BTS7960 H-Bridge motor drivers via gpiozero GPIO on the Raspberry Pi 5. It handles left/right track motors and the blade motor.

## Scope

**In Scope:**
- BTS7960 H-Bridge motor control via gpiozero PWM
- WebSocket command server on port 8766
- Teleop drive with deadzone, clamping, and slew-rate ramping
- Blade motor independent control
- Watchdog timeout (auto-stop on connection loss)
- YAML-based motor configuration

**Out of Scope:**
- ROS2 integration (this is a standalone server, not a ROS2 node)
- Navigation path following (see [bmad-navigation](../bmad-navigation/README.md))
- Encoder reading (see [bmad-integration](../bmad-integration/README.md))

## Hardware

### Motor Pin Configuration (BTS7960)

| Motor | R_EN | L_EN | R_PWM | L_PWM | Max Duty |
|-------|------|------|-------|-------|----------|
| **Right** | GPIO 5 | GPIO 6 | GPIO 19 | GPIO 12 | 1.0 |
| **Left** | GPIO 23 | GPIO 24 | GPIO 18 | GPIO 13 | 1.0 |
| **Blade** | GPIO 4 | GPIO 21 | GPIO 16 | GPIO 20 | 1.0 |

> **Note:** Uses `gpiozero` (`PWMOutputDevice` + `DigitalOutputDevice`) for Pi 5 compatibility.

## BMAD Documents

| Document | Status | Description |
|----------|--------|-------------|
| [product-brief.md](product-brief.md) | ✅ Done | High-level vision |
| [PRD.md](PRD.md) | ✅ Done | Requirements & acceptance criteria |
| [architecture.md](architecture.md) | ✅ Done | Technical design |

## Quick Start

```bash
# SSH to Raspberry Pi
cd ~/mower_ws/src/lawnbot_motors

# Run motor server (standalone, no ROS2 needed)
python3 ws_motor_server.py
```

Connect via WebSocket to `ws://<pi-ip>:8766`.

## WebSocket Commands

| Command | Payload | Description |
|---------|---------|-------------|
| `stop` | `{"cmd":"stop"}` | Stop all motors |
| `on` | `{"cmd":"on"}` | Default profile (right:0.6, left:0.6, blade:0.1) |
| `drive` | `{"cmd":"drive","left":0.3,"right":0.3}` | Teleop joystick |
| `blade` | `{"cmd":"blade","speed":0.5}` | Blade motor only |
| `set` | `{"cmd":"set","right":0.4,"left":0.4}` | Manual speed set |

## Teleop Tuning

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_wheel` | 0.5 | Max manual speed limit |
| `deadzone` | 0.08 | Joystick noise filter |
| `ramp_per_sec` | 1.5 | Slew-rate limiter (smoothing) |

## Related Subsystems

| Subsystem | Description | Link |
|-----------|-------------|------|
| Integration | Encoder reading | [bmad-integration](../bmad-integration/README.md) |
| Navigation | Autonomous path following | [bmad-navigation](../bmad-navigation/README.md) |
| Comms | App ↔ ROS2 bridge | [bmad-lawnbot-comms](../bmad-lawnbot-comms/README.md) |
