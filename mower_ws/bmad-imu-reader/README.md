# BMAD: IMU Reader Subsystem

## Overview

The **IMU Reader** subsystem provides a robust interface for inertial sensors (MPU-6050, BNO085), abstracting hardware details and publishing standard ROS 2 IMU messages.

## Scope

**In Scope:**
- MPU-6050 support (Raw + Madgwick Filter)
- BNO085 support (Hardware Fusion)
- Diagnostics reporting
- I2C Bus Management

**Out of Scope:**
- Sensor Fusion (EKF) - See [bmad-integration](../bmad-integration/README.md)

## BMAD Documents

| Document | Status | Description |
|----------|--------|-------------|
| [product-brief.md](product-brief.md) | ✅ Done | High-level vision |
| [PRD.md](PRD.md) | ✅ Done | Requirements & acceptance criteria |
| [architecture.md](architecture.md) | ✅ Done | Technical design |

## Quick Start

```bash
# Install Dependencies
sudo pip3 install --break-system-packages smbus2 adafruit-circuitpython-bno08x

# Build
cd ~/mower_ws
colcon build --packages-select imu_reader

# Run (MPU-6050 default)
ros2 launch imu_reader imu.launch.py

# Run (BNO085)
ros2 launch imu_reader imu.launch.py imu_type:=bno085
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | Orientation & Acceleration |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Health Status |

## Use Cases

1.  **Sensor Fusion**: Providing orientation data to `robot_localization`.
2.  **Bump Detection**: Detecting collisions via accelerometer spikes.
3.  **Tilt Protection**: Detecting dangerous slopes.
