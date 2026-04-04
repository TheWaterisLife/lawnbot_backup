# Product Brief: IMU Reader Subsystem

## Vision

Provide high-frequency, reliable inertial measurement data (orientation, angular velocity, and linear acceleration) to the sensor fusion system, enabling accurate attitude estimation and navigation.

## Problem Statement

The robot needs to know its orientation (heading) and detect bumps/tilts. Raw sensor data from MPU6050 or BNO085 needs to be:
1.  ** abstracted**: Common interface regardless of hardware.
2.  ** calibrated**: Free of significant drift.
3.  ** reliable**: Automatically recoverable from I2C bus errors.

## Solution

A dedicated ROS 2 node (`imu_reader`) that:
- Supports multiple hardware drivers (MPU6050, BNO085).
- Publishes standard `sensor_msgs/Imu` at 100Hz.
- Handles sensor calibration and initialization.
- Monitors health via standard diagnostics.

## Target Users

1.  **Sensor Integration**: Consumes IMU data for EKF fusion.
2.  **Navigation**: Uses acceleration for bump detection (future).
3.  **Diagnostics**: Monitors hardware health.

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Update Rate | > 95 Hz | Frequency analysis |
| Latency | < 10 ms | Driver processing time |
| Drift (Yaw) | < 1 deg/min | Static test (BNO085) |
| Recovery | < 2 sec | Time to restart after I2C error |

## Constraints

- **Hardware**: Pi 5 (I2C Bus 1).
- **Sensors**: MPU-6050 (Address 0x68) OR BNO085 (Address 0x4B).
- **Compute**: Low CPU usage (< 5% single core).

## Key Risks

| Risk | Mitigation |
|------|------------|
| I2C Bus Lockup | Driver timeout and reset logic |
| High Vibration Noise | Digital Low Pass Filter (DLPF) config |
| Magnetic Interference | Calibration requirement for magnetometer |
