# Product Brief: Sensor Integration Subsystem

## Vision

Create a robust, high-precision localization system that fuses multiple redundant sensors to provide centimeter-accurate position estimates for an autonomous lawn mower, enabling reliable virtual boundary enforcement and systematic coverage mowing.

## Problem Statement

Autonomous lawn mowers require accurate position knowledge to:
1. Stay within user-defined boundaries
2. Follow systematic mowing patterns
3. Return to charging station
4. Resume mowing after interruption

Single-sensor solutions are insufficient:
- **GPS alone**: Loses signal under trees, multipath errors near buildings
- **IMU alone**: Drifts over time (especially heading without magnetometer)
- **Wheel encoders alone**: Accumulates error from wheel slip on grass

## Solution

Fuse three complementary sensors using an Extended Kalman Filter (EKF):

| Sensor | Strength | Weakness | Role in Fusion |
|--------|----------|----------|----------------|
| SimpleRTK2B (F9P) | Absolute position (cm) | Dropout under trees | Ground truth position |
| IMU (MPU-6500) | Fast angular velocity updates | No magnetometer | Angular rates only |
| Encoders (FIT0403) | Consistent velocity | Wheel slip | Velocity + heading validation |

**Key Innovation:** The SimpleRTK2B provides **robust RTK-Fixed** solutions ensuring reliable centimeter-level positioning. IMU orientation/acceleration are **disabled** in EKF to prevent drift - only angular velocity is used.

The EKF combines these to produce a pose estimate that:
- Has cm-level accuracy when GPS is available
- Uses GPS position as primary position source
- Maintains reasonable accuracy during brief GPS dropouts (wheel odom + IMU angular velocity)
- Self-corrects when GPS signal returns

## Target Users

1. **Navigation Subsystem**: Consumes fused pose for path planning
2. **Boundary Manager**: Uses pose for boundary enforcement
3. **Status Monitor**: Displays position and sensor health to user
4. **Mobile App (via WebSocket)**: Real-time position display

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Position Accuracy | < 5 cm | Compare to RTK ground truth |
| IMU Rate | ~17 Hz | Measure /imu/data rate |
| Wheel Odom Rate | 50 Hz | Measure /wheel/odom rate |
| Update Rate | 50 Hz | Measure /odometry/filtered rate |
| GPS Rate | 1 Hz | Measure /rtk/fix rate |
| RTK Fix Time | < 60 sec | Time to RTK-Fixed from cold start |
| Sensor Health Reporting | 100% | All sensors report status |

## Constraints

- **Platform**: Raspberry Pi 5 (limited compute)
- **Power**: Battery-operated (minimize processing)
- **Real-time**: Must run alongside camera and navigation
- **ROS 2 Jazzy**: Must integrate with existing ai_camera_vision package
- **Connectivity**: Phone hotspot required for NTRIP (RTK2Go)

## Key Risks

| Risk | Mitigation |
|------|------------|
| GPS dropout in wooded areas | Wheel encoders + IMU angular velocity bridge gaps |
| Encoder slip on wet grass | GPS corrects errors when signal returns |
| NTRIP connection loss via hotspot | Auto-reconnect with exponential backoff, fall back to standalone GPS (~2m accuracy) |
| Position drift from IMU | IMU orientation/acceleration disabled in EKF, only angular velocity used |

## Hardware Configuration

### IMU Strategy
- **Current**: MPU-6500 (MPU-6050 compatible, WHO_AM_I: 0x70)
- **EKF Usage**: Angular velocity only (orientation/acceleration disabled to prevent drift)
- **GPIO Library**: gpiozero (Pi 5 compatible, replaces RPi.GPIO)

### GPS Setup
- **Module**: SimpleRTK2B (u-blox ZED-F9P)
- **Antenna**: Single Multi-Band Grid Antenna
- **Mounting**: Centered on robot for simplest transform

### NTRIP Service
- **Provider**: RTK2Go (free community service)
- **Coverage**: Montreal area bases
- **Connection**: Via phone hotspot (existing WebSocket infrastructure)

## Timeline

This is Phase 1 of the Navigation & Localization development. Must be completed before Navigation subsystem can begin integration testing.
