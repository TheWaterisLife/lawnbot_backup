# Product Brief: Lawnbot Comms Subsystem

## Vision

Provide a reliable, low-latency communication bridge between the mobile app and the autonomous lawn mower's ROS2 system, enabling real-time teleoperation, mode control, and telemetry monitoring over WiFi.

## Problem Statement

The autonomous mower needs a way for users to:
1. **Control**: Teleoperate the mower via phone joystick
2. **Monitor**: See mower state and telemetry in real-time
3. **Command**: Switch between modes (IDLE, TELEOP, AUTO)
4. **Supervise**: Know the mower is connected and responsive

## Solution

A WebSocket server running on the Raspberry Pi that bridges JSON messages from the mobile app to ROS2 topics:

| Direction | Data | Transport |
|-----------|------|-----------|
| Phone → Mower | Heartbeat, mode, joystick | WebSocket JSON → ROS2 topics |
| Mower → Phone | State, telemetry | ROS2 topics → WebSocket JSON |

## Target Users

1. **Mobile App**: Sends commands, receives telemetry
2. **Navigation Subsystem**: Receives `/app/teleop_cmd` and `/app/mode`
3. **Status Monitor**: Publishes to `/mower/state` and `/mower/telemetry`

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Connection Latency | < 50 ms | WebSocket round-trip |
| Telemetry Rate | 5 Hz | Messages per second to app |
| Reconnect Time | < 2 sec | After WiFi dropout |
| Heartbeat Timeout | Detectable | `heartbeat_age_ms` reported |

## Constraints

- **Network**: Phone hotspot WiFi (same network as Pi)
- **Single Client**: Only one app connection at a time
- **Port**: 9002 (hardcoded)
- **Platform**: Raspberry Pi 5

## Key Risks

| Risk | Mitigation |
|------|------------|
| WiFi dropout | Heartbeat age monitoring, auto-reconnect |
| Multiple connections | Reject additional clients |
| Teleop delay | Low-latency WebSocket, direct topic publish |
| Unauthorized access | Single-client lock, local network only |
