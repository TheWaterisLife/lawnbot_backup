# Epic 3: Wheel Encoder Odometry

## Goal
Implement a ROS 2 node that reads wheel encoder pulses via GPIO quadrature decoding and computes differential drive odometry for the tracked mower.

## Background
The DFRobot FIT0403 motors include Hall effect encoders with two channels (A and B) for quadrature decoding. By counting pulses and knowing the gear ratio, we compute wheel rotation and derive robot velocity (vx, wz).

## Hardware Specs

| Parameter | Value | Notes |
|-----------|-------|-------|
| Motor | DFRobot FIT0403 | 12V, 90:1 gearbox |
| Encoder Type | Hall effect | 2 channels (A, B) |
| Encoder PPR (motor) | 11 | Per motor revolution |
| Gearbox Ratio | 90:1 | |
| Encoder PPR (output) | 990 | 11 × 90 |
| Quadrature CPR | 3960 | 990 × 4 (both edges, both channels) |
| Track Width | 0.24 m | Center-to-center |
| Sprocket Radius | 0.05 m | TBD - measure actual |

---

## GPIO Pin Assignment

> [!WARNING]
> **These pins must NOT conflict with motor driver pins!**

| Encoder | Channel | GPIO | Physical Pin | Notes |
|---------|---------|------|--------------|-------|
| Left | A | 17 | 11 | Interrupt-capable |
| Left | B | 27 | 13 | Interrupt-capable |
| Right | A | 22 | 15 | Interrupt-capable |
| Right | B | 4 | 7 | Interrupt-capable |

### Motor Driver Pins (DO NOT USE for encoders)

| Motor | GPIO Pins |
|-------|-----------|
| Left motor | 5, 6, 13, 18 |
| Right motor | 23, 24, 12, 19 |
| Blade motor | 20, 16, 26, 21 |

### Reserved Pins

| Function | GPIO |
|----------|------|
| I2C (IMU) | 2, 3 |
| SPI (future) | 7-11 |
| UART (future) | 14, 15 |

---

## Stories

### Story 3.1: GPIO Encoder Reader
**As a** developer  
**I want** a class that counts encoder pulses via GPIO  
**So that** I can track wheel rotation

**Acceptance Criteria:**
- [ ] Set up GPIO pins with internal pull-up resistors
- [ ] Use interrupt-based counting (not polling) via pigpio or RPi.GPIO
- [ ] Implement full quadrature decoding (4x resolution)
- [ ] Track direction (forward/reverse) from A/B phase relationship
- [ ] Thread-safe counter access
- [ ] Handle GPIO errors gracefully

**GPIO Configuration:**
```python
# encoder.py
LEFT_ENCODER_A = 17
LEFT_ENCODER_B = 27
RIGHT_ENCODER_A = 22
RIGHT_ENCODER_B = 4
```

**Quadrature Decoding Logic:**
```
State Table (A, B transitions):
  Previous | Current | Direction
  00       | 01      | Forward
  01       | 11      | Forward
  11       | 10      | Forward
  10       | 00      | Forward
  00       | 10      | Reverse
  10       | 11      | Reverse
  11       | 01      | Reverse
  01       | 00      | Reverse
```

---

### Story 3.2: Differential Drive Kinematics
**As a** developer  
**I want** to compute robot velocity from wheel velocities  
**So that** I can publish odometry

**Acceptance Criteria:**
- [ ] Compute left wheel velocity from encoder count
- [ ] Compute right wheel velocity from encoder count
- [ ] Compute `vx = (v_left + v_right) / 2`
- [ ] Compute `wz = (v_right - v_left) / track_width`
- [ ] Handle configurable wheel_radius and track_width

**Formulas:**
```
v_wheel = (delta_count / cpr) * 2 * pi * wheel_radius / dt

vx = (v_left + v_right) / 2
wz = (v_right - v_left) / track_width
```

---

### Story 3.3: Odometry ROS 2 Node
**As a** navigation system  
**I want** wheel odometry published as nav_msgs/Odometry  
**So that** the EKF has velocity measurements

**Acceptance Criteria:**
- [ ] Node starts without errors
- [ ] Publishes to /wheel/odom at 50 Hz
- [ ] Message includes twist (vx, wz)
- [ ] Message includes integrated pose (x, y, theta)
- [ ] Covariance reflects encoder accuracy
- [ ] frame_id = "odom", child_frame_id = "base_link"

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| left_encoder_a | 17 | GPIO pin |
| left_encoder_b | 27 | GPIO pin |
| right_encoder_a | 22 | GPIO pin |
| right_encoder_b | 4 | GPIO pin |
| wheel_radius | 0.05 | Sprocket radius (meters) |
| track_width | 0.24 | Distance between tracks (meters) |
| encoder_cpr | 3960 | Counts per revolution |
| publish_rate | 50.0 | Hz |

---

### Story 3.4: Odometry Calibration
**As a** developer  
**I want** a calibration procedure for odometry parameters  
**So that** I can tune accuracy

**Calibration Tests:**
1. **Straight line test**: Drive 2m, measure actual vs reported
2. **Rotation test**: Rotate 360°, measure actual vs reported
3. **Square test**: Drive square pattern, check return to origin

---

### Story 3.5: Encoder Diagnostics
**As an** operator  
**I want** to monitor encoder health  
**So that** I know if odometry is reliable

**Acceptance Criteria:**
- [ ] Report pulse rate for each encoder
- [ ] Warn if one encoder stuck while other moving
- [ ] Warn if velocity exceeds physical limits
- [ ] Report in /diagnostics topic

---

## Definition of Done
- [ ] Encoder GPIO wiring verified
- [ ] Odometry publishing at 50 Hz
- [ ] Calibration documented
- [ ] Tests passing

## TODO Items
- [ ] Wire encoder signals to GPIO 17, 27, 22, 4
- [ ] Measure actual sprocket diameter
- [ ] Measure actual track width
