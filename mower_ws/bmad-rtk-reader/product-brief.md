# Product Brief: RTK Reader Subsystem

## Vision

Provide centimeter-accurate GPS positioning for the autonomous lawn mower by leveraging RTK (Real-Time Kinematic) corrections, enabling precise boundary enforcement and systematic coverage patterns.

## Problem Statement

Standard GPS accuracy (~5 meters) is insufficient for lawn mowing:
- **Boundary violations**: Robot could mow into flower beds or driveways
- **Coverage gaps**: Overlap uncertainty leads to missed strips
- **Path following**: Cannot follow precise boustrophedon patterns

## Solution

Use a high-precision u-blox ZED-F9P receiver with RTK corrections:

| Component | Purpose |
|-----------|---------|
| **ZED-F9P** | Multi-band GNSS receiver (L1/L2) |
| **NTRIP Client** | Streams RTK corrections from base station |
| **GGA Parser** | Extracts position and fix quality |
| **ROS2 Publisher** | Publishes NavSatFix to `/rtk/fix` |

**Key Innovation:** By connecting to an NTRIP caster, the receiver achieves RTK-Fixed status with **~2cm accuracy**, enabling precise navigation within lawn boundaries.

## Target Users

1. **Sensor Integration**: Subscribes to `/rtk/fix` for EKF fusion
2. **Boundary Mapper**: Uses GPS for boundary recording
3. **Navigation**: Depends on accurate position for path following

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Position Accuracy | < 5 cm | RTK-Fixed solution |
| Fix Acquisition | < 60 sec | Time to RTK-Fixed |
| Publication Rate | ~1 Hz | Measure /rtk/fix rate |
| NTRIP Uptime | > 99% | Connection stability |
| Reconnect Time | < 5 sec | Auto-reconnect on disconnect |

## Constraints

- **Single Antenna**: No heading from GPS (uses IMU instead)
- **Internet Required**: NTRIP needs phone hotspot connection
- **Clear Sky**: RTK works best with open sky view
- **Base Station Range**: ~30km from NTRIP mountpoint

## Key Risks

| Risk | Mitigation |
|------|------------|
| NTRIP disconnection | Auto-reconnect with 2s retry |
| No RTK fix (trees) | Fall back to GPS-only (~5m), increase EKF covariance |
| Serial port change | Use `/dev/serial/by-id/` for stable device path |
| Multipath errors | Position covariance reflects fix quality |

## Hardware Configuration

- **Module**: SimpleRTK2B (u-blox ZED-F9P)
- **Port**: `/dev/ttyACM0` @ 115200 baud
- **Antenna**: Single multi-band GNSS antenna
- **Mounting**: Centered on robot, clear sky view

## Timeline

This is a foundational component that must be complete before Sensor Integration can achieve high-accuracy positioning.
