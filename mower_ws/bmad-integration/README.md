# BMAD: Sensor Integration Subsystem

## Overview

This BMAD project covers the **Sensor Integration** subsystem of the autonomous lawn mower. The goal is to fuse data from three redundant sensors (IMU, RTK-GPS, Wheel Encoders) using an Extended Kalman Filter to produce an accurate, reliable pose estimate.

## Scope

**In Scope:**
**In Scope:**
- Sensor Fusion (EKF) configuration using `robot_localization`
- Launching and coordinating sensor nodes:
    - IMU via `imu_reader` package
    - Wheel Encoders via `wheel_encoder` package
    - RTK-GPS via `rtk_reader` package
- Sensor data validation and health monitoring
- Position output visualization/demo
- Position output visualization/demo

**Out of Scope:**
- Navigation and path planning (see [bmad-navigation](../bmad-navigation/README.md))
- Motor control (see [bmad-navigation Motor Hardware](../bmad-navigation/README.md#motor-hardware-configuration))
- Camera/obstacle detection (see [bmad-camera](../bmad-camera/README.md))
- Mobile app integration

## Hardware

| Sensor | Model | Interface | Address/Port | Rate |
|--------|-------|-----------|--------------|------|
| IMU | MPU-6500 (MPU-6050 compatible) | I2C Bus 1 | 0x68 (WHO_AM_I: 0x70) | ~17 Hz |
| GPS | SimpleRTK2B (u-blox ZED-F9P) | USB Serial | `/dev/ttyACM0` @ 115200 | 1 Hz |
| Encoders | DFRobot FIT0403 (x2) | GPIO (Quadrature) | See pinout | 50 Hz |

### GPIO Pin Assignment (gpiozero - Pi 5 compatible)

| Function | GPIO | Physical Pin | Notes |
|----------|------|--------------|-------|
| Left Encoder A | 17 | 11 | Avoid motor pins |
| Left Encoder B | 27 | 13 | Avoid motor pins |
| Right Encoder A | 22 | 15 | Avoid motor pins |
| Right Encoder B | 4 | 7 | Avoid motor pins |
| Motor Driver | 5,6,12,13,18,19,23,24 | - | Reserved - DO NOT USE |
| I2C1 SDA | 2 | 3 | IMU Data |
| I2C1 SCL | 3 | 5 | IMU Clock |

**Note:** GPS uses `/dev/ttyACM0`, AI Camera uses USB. Motor control pins are documented in [bmad-navigation](../bmad-navigation/README.md#motor-hardware-configuration). Use **gpiozero** instead of RPi.GPIO for Pi 5 compatibility.

### RTK/NTRIP Configuration

| Parameter | Value |
|-----------|-------|
| NTRIP Caster | rtk2go.com |
| NTRIP Port | 2101 |
| Mount Point | Auto-select nearest (Montreal area) |
| Authentication | Email-based (RTK2Go free tier) |

## BMAD Documents

| Document | Status | Description |
|----------|--------|-------------|
| [product-brief.md](product-brief.md) | ✅ Done | High-level vision |
| [PRD.md](PRD.md) | ✅ Done | Requirements & acceptance criteria |
| [architecture.md](architecture.md) | ✅ Done | Technical design |
| [epics/](epics/) | ✅ Done | Feature groupings |
| [stories/](stories/) | ✅ Done | Implementation tasks |

## Implementation Status

| Epic | Stories | Status |
|------|---------|--------|
| **Epic 1: IMU Integration** | 1.1-1.3 | ✅ Complete |
| **Epic 2: GPS Integration** | 2.1-2.4 | ✅ Complete |
| **Epic 3: Wheel Encoders** | 3.1-3.3 | ✅ Complete |
| **Epic 4: EKF Fusion** | 4.1-4.3 | ✅ Complete |

## Quick Start

### Hardware Test (Standalone - No ROS2)

```bash
# SSH to your Raspberry Pi
ssh pi@raspberrypi

# Navigate to the package
cd ~/ros2_ws/src/sensor_integration

# Run the standalone demo (single terminal!)
python3 demo_standalone.py
```

This displays live output from all sensors in one terminal - perfect for verifying hardware works.

### Full System (ROS2)

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select sensor_integration

# Run the sensor fusion demo
ros2 launch sensor_integration demo.launch.py
```

## Output Topics

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | ~17 Hz | Angular velocity (orientation disabled) |
| `/rtk/fix` | `sensor_msgs/NavSatFix` | 1 Hz | RTK position |
| `/wheel/odom` | `nav_msgs/Odometry` | 50 Hz | Wheel odometry |
| `/gps/odom` | `nav_msgs/Odometry` | 10 Hz | GPS in odom frame |
| `/odom_filtered` | `nav_msgs/Odometry` | EKF-fused pose |

## Success Criteria

1. ✅ EKF produces stable pose at 50 Hz (`/odometry/filtered`)
2. ✅ Position accuracy within 5 cm (with RTK-Fixed, status=2)
3. ✅ IMU angular velocity at ~17 Hz (orientation disabled for drift prevention)
4. ✅ Graceful degradation during GPS dropout (< 10 sec)
5. ✅ NTRIP auto-reconnection with exponential backoff
6. ✅ Seamless IMU type switching via configuration
7. ✅ All sensors report health status via /diagnostics

## Related Subsystems

| Subsystem | Description | Link |
|-----------|-------------|------|
| IMU Reader | MPU6050/BNO085 handling | [bmad-imu-reader](../bmad-imu-reader/README.md) |
| Wheel Encoder | Odometry from encoders | [bmad-wheel-encoder](../bmad-wheel-encoder/README.md) |
| RTK Reader | GPS/NTRIP handling | [bmad-rtk-reader](../bmad-rtk-reader/README.md) |
| Navigation | Path planning, motor control | [bmad-navigation](../bmad-navigation/README.md) |
| Camera | AI obstacle detection | [bmad-camera](../bmad-camera/README.md) |

