# Product Requirements Document: IMU Reader Subsystem

## 1. Overview

### 1.1 Purpose
The IMU Reader subsystem abstracts specific inertial sensor hardware to provide a unified stream of orientation and acceleration data for navigation.

### 1.2 Scope
- MPU-6050 and BNO085 driver support
- `sensor_msgs/Imu` publication
- Calibration status monitoring
- Diagnostics integration

---

## 2. Functional Requirements

### 2.1 Sensor Data (FR-DATA)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-DATA-01 | Read Gyroscope (x,y,z) | Must |
| FR-DATA-02 | Read Accelerometer (x,y,z) | Must |
| FR-DATA-03 | Read/Fuse Magnetometer (BNO085 only) | Must |
| FR-DATA-04 | Publish data at configured rate (default 20Hz) | Must |

**Acceptance Criteria:**
- Output timestamps are monotonic.
- Frames are consistent (ENU standard).

### 2.2 Driver Abstraction (FR-DRV)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-DRV-01 | Support `imu_type` parameter ("mpu6050", "bno085") | Must |
| FR-DRV-02 | Auto-detect I2C address if not specified | Should |
| FR-DRV-03 | Gracefully handle connection failures | Must |

### 2.3 Diagnostics (FR-DIAG)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-DIAG-01 | Publish `/diagnostics` with hardware ID | Must |
| FR-DIAG-02 | Report calibration status (System, Gyro, Accel, Mag) | Must |
| FR-DIAG-03 | Report error counts (I2C failures) | Must |

---

## 3. Non-Functional Requirements

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-01 | Startup Time | < 1 second |
| NFR-02 | CPU Load | < 5% |
| NFR-03 | Reliability | MTBF > 100 hours |

---

## 4. Interface Specifications

### 4.1 Published Topics

| Topic | Type | Frequency |
|-------|------|-----------|
| `/imu/data` | sensor_msgs/Imu | 20 Hz |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | 1 Hz |

### 4.2 Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `imu_type` | string | "mpu6050" | Driver selection |
| `i2c_bus` | int | 1 | I2C Bus ID |
| `publish_rate` | double | 20.0 | Output Hz |

---

## 5. Acceptance Criteria Summary

1. Node launches with `imu_type:=bno085` or `mpu6050`.
2. `/imu/data` is publishing at target rate.
3. Rotating the sensor updates Quaternion/Angular Velocity correctly.
4. Disconnecting sensor triggers Diagnostic Error.
