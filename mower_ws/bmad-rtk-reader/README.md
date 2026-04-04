# BMAD: RTK Reader Subsystem

## Overview

This BMAD project covers the **RTK Reader** subsystem of the autonomous lawn mower. It provides centimeter-accurate GPS positioning by reading a u-blox ZED-F9P receiver and streaming NTRIP corrections for RTK-Fixed solutions.

## Scope

**In Scope:**
- u-blox ZED-F9P serial communication via `/dev/ttyACM0`
- NMEA GGA sentence parsing (lat, lon, alt, fix quality)
- NTRIP client for RTK corrections
- Publishing `/rtk/fix` (sensor_msgs/NavSatFix)
- Auto-reconnect on NTRIP disconnection

**Out of Scope:**
- Sensor fusion (see [bmad-integration](../bmad-integration/README.md))
- Heading computation (single-antenna, no heading)
- Coordinate transforms (handled by sensor_integration)

## Hardware

| Component | Specification |
|-----------|---------------|
| **GPS Module** | SimpleRTK2B (u-blox ZED-F9P) |
| **Antenna** | Multi-Band GNSS Antenna |
| **Serial Port** | `/dev/ttyACM0` @ 115200 baud |
| **Connection** | USB to Raspberry Pi 5 |

## NTRIP Configuration

| Parameter | Value |
|-----------|-------|
| Caster Host | `3.143.243.81` |
| Caster Port | `2101` |
| Mountpoint | `RD1_STATION_SO` |
| Protocol | Ntrip/2.0 |

## BMAD Documents

| Document | Status | Description |
|----------|--------|-------------|
| [product-brief.md](product-brief.md) | ✅ Done | High-level vision |
| [PRD.md](PRD.md) | ✅ Done | Requirements & acceptance criteria |
| [architecture.md](architecture.md) | ✅ Done | Technical design |

## Quick Start

```bash
# Build the package
cd ~/mower_ws
colcon build --packages-select rtk_reader

# Run RTK reader
ros2 run rtk_reader rtk_reader --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baud:=115200

# Verify RTK fix
ros2 topic echo /rtk/fix --field status --field latitude --field longitude
```

## Output Topics

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/rtk/fix` | `sensor_msgs/NavSatFix` | ~1 Hz | RTK GPS position |

## GPS Status Codes

| Status | Quality | Meaning | Accuracy |
|--------|---------|---------|----------|
| -1 | 0 | No fix | N/A |
| 0 | 1 | GPS only | ~5m |
| 1 | 2 | DGPS | ~1m |
| 2 | 4 | **RTK Fixed** | **~2cm** ✓ |
| 2 | 5 | RTK Float | ~30cm |

## Success Criteria

1. ✅ GPS position published at ~1 Hz
2. ✅ RTK-Fixed achieved within 60s (status=2, quality=4)
3. ✅ NTRIP auto-reconnect on disconnection
4. ✅ Valid NMEA checksum validation
5. ✅ Centimeter-level accuracy with RTK-Fixed

## Related Subsystems

| Subsystem | Description | Link |
|-----------|-------------|------|
| Integration | Sensor fusion (EKF) | [bmad-integration](../bmad-integration/README.md) |
| Navigation | Path planning | [bmad-navigation](../bmad-navigation/README.md) |
