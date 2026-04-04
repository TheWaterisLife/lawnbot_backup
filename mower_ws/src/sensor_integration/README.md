# Sensor Integration Package

ROS 2 package for sensor fusion on the autonomous lawn mower.

> [!NOTE]
> **GPS is handled by `rtk_reader` package** - we subscribe to `/rtk/fix`.

## Overview

This package provides:
- **imu_node**: IMU data from MPU-6050 or BNO085
- **wheel_odom_node**: Wheel odometry from FIT0403 encoders
- **EKF fusion**: Configuration for robot_localization

## Topics

### Published
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/imu/data` | sensor_msgs/Imu | 100 Hz | IMU orientation + velocity |
| `/wheel/odom` | nav_msgs/Odometry | 50 Hz | Wheel odometry |
| `/odom_filtered` | nav_msgs/Odometry | 50 Hz | Fused pose (from EKF) |

### Subscribed
| Topic | Type | Source |
|-------|------|--------|
| `/rtk/fix` | sensor_msgs/NavSatFix | rtk_reader package |

## Hardware

### IMU (I2C)
| Device | Address | Notes |
|--------|---------|-------|
| MPU-6050 | 0x68 | Prototype |
| BNO085 | 0x4A | Production |

### Encoders (GPIO)
| Encoder | GPIO A | GPIO B |
|---------|--------|--------|
| Left | 17 | 27 |
| Right | 22 | 4 |

> ⚠️ **Motor pins 5,6,12,13,18,19,23,24 are reserved for lawnbot_motors**

## Quick Start

```bash
# Start sensor nodes
ros2 launch sensor_integration sensors.launch.py

# Start EKF fusion (requires rtk_reader running)
ros2 launch sensor_integration fusion.launch.py

# Full demo
ros2 launch sensor_integration demo.launch.py imu_type:=bno085
```

## Configuration

### IMU Type
```bash
ros2 launch sensor_integration sensors.launch.py imu_type:=bno085
```

### Config Files
- `config/sensors.yaml` - Sensor parameters
- `config/ekf.yaml` - EKF + navsat_transform config

## Dependencies

- robot_localization
- sensor_msgs, nav_msgs
- smbus2 (I2C)
- RPi.GPIO or pigpio (GPIO)

## External Dependencies

| Package | Topic | Purpose |
|---------|-------|---------|
| rtk_reader | /rtk/fix | RTK GPS position |
| lawnbot_motors | N/A | Motor control (independent) |
