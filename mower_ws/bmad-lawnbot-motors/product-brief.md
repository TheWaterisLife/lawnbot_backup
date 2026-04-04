# Product Brief: Lawnbot Motors Subsystem

## Vision

Provide safe, responsive motor control for the autonomous lawn mower's track motors and blade motor, with WebSocket-based remote operation and built-in safety features.

## Problem Statement

The mower needs reliable motor control that:
1. **Drives tracks**: Independent left/right track speed for differential steering
2. **Controls blade**: Separate blade motor with independent speed
3. **Ensures safety**: Auto-stop when communication is lost
4. **Smooth control**: No jerky movements from joystick input

## Solution

A standalone WebSocket server controlling three BTS7960 H-Bridge motor drivers:

| Component | Purpose |
|-----------|---------|
| **BTS7960Motor** | GPIO-based H-Bridge driver (gpiozero) |
| **WebSocket Server** | Command interface on port 8766 |
| **Watchdog** | Auto-stop after 0.4s command timeout |
| **Slew-Rate Limiter** | Smooth acceleration/deceleration |
| **YAML Config** | Pin mapping, speeds, tuning |

## Target Users

1. **Mobile App**: Sends drive/blade commands via WebSocket
2. **Lawnbot Comms Bridge**: Routes teleop commands
3. **Navigation System**: Future autonomous control integration

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Response Latency | < 20 ms | WebSocket command to motor |
| Watchdog Timeout | 0.4 sec | Auto-stop on disconnect |
| PWM Frequency | 1000 Hz | Smooth motor operation |
| Speed Range | -1.0 to +1.0 | Forward/reverse |

## Constraints

- **Platform**: Raspberry Pi 5 with gpiozero
- **Standalone**: No ROS2 dependency (plain Python + WebSocket)
- **Port**: 8766 (configurable in YAML)
- **Single client**: WebSocket handler per connection

## Key Risks

| Risk | Mitigation |
|------|------------|
| Runaway motors | Watchdog auto-stop (0.4s timeout) |
| WiFi disconnection | Watchdog kills motors on timeout |
| Jerky joystick | Deadzone filter + slew-rate ramping |
| GPIO conflicts | Dedicated pins, no overlap with encoders |
| Overcurrent | BTS7960 has built-in overcurrent protection |

## Hardware

- **Motor Drivers**: 3× BTS7960 H-Bridge modules
- **Left/Right Motors**: DFRobot FIT0403 (track drive)
- **Blade Motor**: DC motor with BTS7960
- **GPIO Library**: gpiozero (Pi 5 compatible)
- **PWM Frequency**: 1000 Hz
