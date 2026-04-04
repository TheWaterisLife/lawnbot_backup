# Product Requirements Document: Sensor Integration Subsystem

## 1. Overview

### 1.1 Purpose
This document defines the requirements for the Sensor Integration subsystem, which fuses IMU, RTK-GPS, and wheel encoder data to produce accurate pose estimates for the autonomous lawn mower.

### 1.2 Scope
- IMU support: MPU-6500/MPU-6050 (I2C @ 0x68)
- GPS integration via external `rtk_reader` package (subscribes to `/rtk/fix` via `/dev/ttyACM0`)
- Wheel encoder odometry with gpiozero quadrature decoding (FIT0403, Pi 5 compatible)
- Extended Kalman Filter fusion (robot_localization)
- Health monitoring and diagnostics

> [!NOTE]
> **GPS is handled by `rtk_reader` package**, not this package. We subscribe to `/rtk/fix`.

### 1.3 Definitions

| Term | Definition |
|------|------------|
| EKF | Extended Kalman Filter - sensor fusion algorithm |
| RTK | Real-Time Kinematic - cm-accurate GPS technique |
| NTRIP | Network Transport of RTCM via Internet Protocol |
| Pose | Position (x, y, z) + orientation (roll, pitch, yaw) |
| Odometry | Motion estimate from wheel rotation |
| Quadrature | Encoder decoding using both A and B channels for 4x resolution |

---

## 2. Functional Requirements

### 2.1 IMU Node (FR-IMU)

**Supports MPU-6500/MPU-6050 via `imu_type` parameter:**
- `mpu6050` - Works with MPU-6500 (WHO_AM_I accepts 0x68, 0x70, 0x71, 0x73)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-IMU-01 | Support MPU-6500/MPU-6050 via configuration | Must |
| FR-IMU-02 | Read IMU via I2C at ~17 Hz (tested rate) | Must |
| FR-IMU-03 | Publish sensor_msgs/Imu to /imu/data | Must |
| FR-IMU-04 | Provide angular velocity for EKF (orientation disabled for drift prevention) | Must |
| FR-IMU-05 | Accept WHO_AM_I values: 0x68, 0x70, 0x71, 0x73 for MPU-6500 | Must |
| FR-IMU-06 | Report connection status | Should |
| FR-IMU-07 | Handle I2C communication errors gracefully | Must |

**IMU Hardware Configuration:**

| Parameter | MPU-6500/6050 |
|-----------|---------------|
| I2C Bus | 1 |
| I2C Address | 0x68 |
| WHO_AM_I | 0x70 (MPU-6500) |
| DOF | 6 (no mag) |
| EKF Usage | Angular velocity only (orientation/accel disabled) |

**Acceptance Criteria:**
- IMU data published at ~17 Hz sustained
- Angular velocity populated for EKF
- No crashes on I2C timeout
- Handles MPU-6500 WHO_AM_I mismatch gracefully

### 2.2 GPS Integration (FR-GPS)

> [!IMPORTANT]
> GPS is handled by the **`rtk_reader`** package. This package **subscribes** to `/rtk/fix`.

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-GPS-01 | Subscribe to /rtk/fix from rtk_reader package | Must |
| FR-GPS-02 | Use navsat_transform_node to convert lat/lon to local ENU | Must |
| FR-GPS-03 | Publish /gps/odom for EKF fusion | Must |
| FR-GPS-04 | Handle GPS dropout gracefully | Must |

**rtk_reader Configuration (External Package):**

| Parameter | Value |
|-----------|-------|
| Topic | `/rtk/fix` |
| Serial Port | `/dev/ttyACM0` |
| Baud Rate | 115200 |
| NTRIP Host | `3.143.243.81` |
| NTRIP Port | 2101 |
| Mountpoint | `RD1_STATION_SO` |

**Acceptance Criteria:**
- GPS position received at 1 Hz from /rtk/fix
- RTK-Fixed status achieved within 60 seconds
- Position accuracy < 5 cm when RTK-Fixed

### 2.3 Wheel Odometry Node (FR-ODOM)

**Hardware:** DFRobot FIT0403 motors with Hall effect encoders (quadrature, gpiozero for Pi 5)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-ODOM-01 | Read encoder pulses from both motors via gpiozero (Pi 5 compatible) | Must |
| FR-ODOM-02 | Use interrupt-based counting via gpiozero DigitalInputDevice | Must |
| FR-ODOM-03 | Implement quadrature decoding (A + B channels) | Must |
| FR-ODOM-04 | Compute wheel velocities from pulse counts | Must |
| FR-ODOM-05 | Calculate robot velocity (vx, wz) using differential drive model | Must |
| FR-ODOM-06 | Publish nav_msgs/Odometry to /wheel/odom | Must |
| FR-ODOM-07 | Support runtime calibration parameters | Should |

**GPIO Pin Assignment:**

> [!WARNING]
> These pins must NOT conflict with motor driver pins.

| Encoder | Channel | GPIO | Physical Pin |
|---------|---------|------|--------------|
| Left | A | 17 | 11 |
| Left | B | 27 | 13 |
| Right | A | 22 | 15 |
| Right | B | 4 | 7 |

**Motor Pins (DO NOT USE):**
- Left motor: GPIO 5, 6, 13, 18
- Right motor: GPIO 23, 24, 12, 19
- Blade motor: GPIO 20, 16, 26, 21

**Encoder Specifications (FIT0403):**

| Parameter | Value | Notes |
|-----------|-------|-------|
| Motor PPR | 11 | Hall effect pulses per motor revolution |
| Gearbox Ratio | 90:1 | |
| Calibrated CPR | 3232 | Counts per revolution (calibrated on hardware) |

**Physical Parameters:**

| Parameter | Default | Notes |
|-----------|---------|-------|
| wheel_radius | 0.05 m | Sprocket radius (measure!) |
| track_width | 0.24 m | Distance between tracks |
| encoder_cpr | 3232 | Counts per revolution (calibrated) |

**Acceptance Criteria:**
- Odometry published at >= 50 Hz
- Velocity estimate within 10% of actual
- Direction detection working (forward/reverse)

### 2.4 EKF Fusion Node (FR-EKF)

**Key Design Decision:** Use BNO085 Magnetometer as primary yaw source.

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-EKF-01 | Use robot_localization package | Must |
| FR-EKF-02 | Fuse IMU roll, pitch, yaw, and angular velocities | Must |
| FR-EKF-03 | Fuse GPS position (x, y) via navsat_transform | Must |
| FR-EKF-04 | Fuse wheel odometry velocity (vx, wz) | Must |
| FR-EKF-05 | Publish nav_msgs/Odometry to /odom_filtered | Must |
| FR-EKF-06 | Publish tf2 transform odom -> base_link | Must |
| FR-EKF-07 | Handle sensor dropout gracefully | Must |

**EKF Sensor Fusion Matrix:**

| Sensor | x | y | roll | pitch | yaw | vx | wz |
|--------|---|---|------|-------|-----|----|----|
| IMU | - | - | - | - | - | - | ✓ |
| GPS (via navsat) | ✓ | ✓ | - | - | - | - | - |
| Wheel Odom | - | - | - | - | - | ✓ | ✓ |

> [!NOTE]
> IMU orientation and acceleration are **disabled** in EKF to prevent position drift. Only angular velocity (wz) is used.

**Acceptance Criteria:**
- Fused pose published at 50 Hz
- Position accuracy < 5 cm (with RTK-Fixed GPS)
- Heading accuracy < 5° (with Magnetometer)
- Maintains estimate during 10-second GPS dropout

---

## 3. Non-Functional Requirements

### 3.1 Performance (NFR-PERF)

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-PERF-01 | IMU latency (sensor to publish) | < 10 ms |
| NFR-PERF-02 | EKF computation time | < 5 ms per cycle |
| NFR-PERF-03 | Total CPU usage (all nodes) | < 25% of Pi 5 |

### 3.2 Reliability (NFR-REL)

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-REL-01 | IMU uptime during operation | > 99.9% |
| NFR-REL-02 | Node crash recovery | Auto-restart within 5 sec |

### 3.3 Diagnostics (NFR-DIAG)

| ID | Requirement | Priority |
|----|-------------|----------|
| NFR-DIAG-01 | Publish diagnostics to /diagnostics | Must |
| NFR-DIAG-02 | Report sensor health (OK/WARN/ERROR) | Must |
| NFR-DIAG-03 | Log warnings on sensor degradation | Must |

---

## 4. Interface Specifications

### 4.1 Published Topics

| Topic | Message Type | Rate | Frame ID |
|-------|--------------|------|----------|
| /imu/data | sensor_msgs/Imu | ~17 Hz | imu_link |
| /wheel/odom | nav_msgs/Odometry | 50 Hz | odom |
| /odom_filtered | nav_msgs/Odometry | 50 Hz | odom |
| /diagnostics | DiagnosticArray | 1 Hz | - |

### 4.2 Subscribed Topics (from external packages)

| Topic | Source Package | Message Type |
|-------|----------------|--------------|
| /rtk/fix | rtk_reader | sensor_msgs/NavSatFix |

### 4.3 TF Frames

```
map
 └── odom (published by EKF)
      └── base_link
           ├── imu_link (static)
           └── gps_link (static)
```

---

## 5. Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| rclpy | Jazzy | ROS 2 Python client |
| sensor_msgs | Jazzy | Standard message types |
| nav_msgs | Jazzy | Odometry message |
| robot_localization | Jazzy | EKF fusion |
| smbus2 | Latest | I2C communication |
| gpiozero | Latest | GPIO interrupts (Pi 5 compatible) |

### External Packages (not in this repo)

| Package | Purpose |
|---------|---------|
| rtk_reader | GPS/RTK handling, publishes /rtk/fix |
| lawnbot_motors | Motor control via WebSocket |

---

## 6. Testing Strategy

### 6.1 Unit Tests
- IMU driver initialization
- Quaternion normalization
- Odometry calculations
- Coordinate transformations

### 6.2 Integration Tests
- EKF fusion with simulated sensors
- TF tree validation
- GPS dropout recovery

### 6.3 Hardware Tests
- Position accuracy vs RTK ground truth
- Encoder pulse counting
- Long-duration stability

---

## 7. Acceptance Criteria Summary

The Sensor Integration subsystem is complete when:

1. IMU node publishes at ~17 Hz with angular velocity
2. Wheel odometry publishes at 50 Hz
3. EKF produces stable fused pose at 50 Hz
4. Position accuracy < 5 cm with RTK-Fixed GPS
5. System recovers from 10-second GPS dropout
6. All diagnostic topics report sensor health
7. Documentation and tests are complete
