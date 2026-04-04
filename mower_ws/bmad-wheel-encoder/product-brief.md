# Product Brief: Wheel Encoder Subsystem

## Vision

Enable precise odometry and velocity estimation by reliably counting wheel rotations from quadrature encoders, serving as the primary dead-reckoning source for the navigation system.

## Problem Statement

GPS is accurate globally but slow (1-5Hz) and prone to jumps. The robot needs a fast (50Hz), smooth local position estimate to:
1.  **Fill GPS gaps**: Connect the dots between GPS fixes.
2.  **Control motors**: Provide velocity feedback for PID control.
3.  **Detect slip**: Compare wheel speed vs IMU acceleration.

## Solution

A dedicated ROS 2 node (`wheel_odom_node`) that:
- Reads GPIO pins using `gpiozero` (Pi 5 compatible).
- Decodes quadrature signals (A/B phases) to determine direction.
- Publishes `nav_msgs/Odometry` and `TF` transforms.
- Handles track width and wheel radius configuration.

## Target Users

1.  **Sensor Integration**: Consumes odometry for EKF fusion.
2.  **Motor Controller**: Uses velocity feedback (future).
3.  **Navigation**: Uses TF tree for local planning.

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Update Rate | 50 Hz | Topic frequency |
| Distance Accuracy | < 2% error | 10m straight line test |
| Turn Accuracy | < 5% error | 360-degree turn test |
| Latency | < 5 ms | GPIO interrupt to Publication |

## Constraints

- **Hardware**: Pi 5 (GPIO).
- **Sensors**: Hall Effect Quadrature Encoders (DFRobot FIT0403).
- **Compute**: Low latency interrupt handling.

## Key Risks

| Risk | Mitigation |
|------|------------|
| Missed counts at high speed | Usage of `lgpio` / `gpiozero` C-based backend |
| Wheel slip | Fusion with IMU/GPS in EKF (Sensor Integration) |
| Electrical noise | Software debouncing (if needed) |
