# Epic 4: Motor Control Interface

## Goal
Implement a ROS 2 node that converts /cmd_vel commands into PWM signals for the motor drivers, enabling the mower to execute navigation commands.

## Background
The Raspberry Pi 5 directly controls the motor drivers via GPIO. The navigation stack publishes geometry_msgs/Twist messages on /cmd_vel, which must be converted to left/right track PWM values using differential drive kinematics.

## Hardware Configuration

| Component | Specification |
|-----------|---------------|
| Motors | DFRobot FIT0403 x2 |
| Driver | H-Bridge (model TBD) |
| PWM Frequency | 1000 Hz |
| Direction | GPIO high/low |
| Interface | Pi GPIO |

## Stories

### Story 4.1: Differential Drive Kinematics
**As a** developer  
**I want** to convert Twist to wheel velocities  
**So that** I can control the tracks correctly

**Acceptance Criteria:**
- [ ] Convert (vx, wz) to (v_left, v_right)
- [ ] Handle configurable track_width
- [ ] Clamp velocities to motor limits
- [ ] Handle reverse motion correctly

**Formulas:**
```python
v_left  = vx - (wz * track_width / 2)
v_right = vx + (wz * track_width / 2)

# Clamp to max velocity
v_left  = clamp(v_left, -max_vel, max_vel)
v_right = clamp(v_right, -max_vel, max_vel)
```

---

### Story 4.2: Velocity to PWM Mapping
**As a** motor controller  
**I want** to convert velocity to PWM duty cycle  
**So that** motors run at correct speed

**Acceptance Criteria:**
- [ ] Map velocity (m/s) to PWM (0-100%)
- [ ] Handle dead zone (motor doesn't move below threshold)
- [ ] Calibration parameters for each motor
- [ ] Linear mapping with offset for friction

**Mapping:**
```python
# Calibrated linear mapping
pwm = (abs(velocity) - dead_zone) * scale + min_pwm
pwm = clamp(pwm, 0, 100)

# Direction
direction = 1 if velocity >= 0 else 0
```

**Calibration Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| dead_zone | 0.05 m/s | Below this, motor doesn't move |
| min_pwm | 15% | Minimum PWM to overcome friction |
| max_pwm | 100% | Maximum PWM |
| scale | TBD | PWM per m/s (calibrate) |

---

### Story 4.3: Motor Control Node
**As a** navigation system  
**I want** a ROS 2 node to control motors  
**So that** cmd_vel commands move the robot

**Acceptance Criteria:**
- [ ] Subscribe to /cmd_vel
- [ ] Output PWM via GPIO
- [ ] Output direction via GPIO
- [ ] Watchdog stops motors if no cmd_vel for 500ms
- [ ] Publish motor status (velocities, PWM values)

**Parameters:**
| Name | Type | Default | Description |
|------|------|---------|-------------|
| left_pwm_pin | int | 12 | GPIO pin |
| right_pwm_pin | int | 13 | GPIO pin |
| left_dir_pin | int | 5 | GPIO pin |
| right_dir_pin | int | 6 | GPIO pin |
| pwm_frequency | int | 1000 | Hz |
| track_width | float | 0.24 | m |
| max_velocity | float | 0.45 | m/s |
| cmd_timeout | float | 0.5 | seconds |

---

### Story 4.4: Smooth Acceleration
**As a** mower  
**I want** smooth acceleration and deceleration  
**So that** motion is not jerky

**Acceptance Criteria:**
- [ ] Limit acceleration rate
- [ ] Smooth velocity changes over multiple cycles
- [ ] No abrupt direction reversals
- [ ] Configurable ramp rates

**Implementation:**
```python
# Velocity ramping
max_accel = 0.5  # m/s^2
dt = 0.05  # 20 Hz
max_delta = max_accel * dt

new_vel = current_vel + clamp(target_vel - current_vel, -max_delta, max_delta)
```

---

### Story 4.5: Emergency Stop
**As a** safety system  
**I want** immediate motor stop capability  
**So that** the mower can halt quickly

**Acceptance Criteria:**
- [ ] /mower/emergency_stop service
- [ ] Sets PWM to 0 immediately (no ramping)
- [ ] Ignores cmd_vel until reset
- [ ] Reset via /mower/reset service
- [ ] Triggered by:
  - Service call
  - Human detection < 1m
  - Boundary violation
  - Pose uncertainty

---

### Story 4.6: Motor Calibration
**As a** developer  
**I want** a calibration procedure  
**So that** velocity commands are accurate

**Acceptance Criteria:**
- [ ] Document calibration procedure
- [ ] Measure actual vs commanded velocity
- [ ] Adjust scale factors per motor
- [ ] Verify straight-line driving
- [ ] Verify rotation in place

**Calibration Procedure:**
1. **Velocity calibration:**
   - Command 0.45 m/s forward
   - Measure actual distance over 5 seconds
   - Calculate scale factor

2. **Rotation calibration:**
   - Command 1.0 rad/s rotation
   - Measure actual rotation over 2*pi
   - Adjust track_width or scale

3. **Straight-line test:**
   - Command 0.45 m/s forward for 10m
   - Measure drift from straight line
   - Adjust left/right balance

---

### Story 4.7: Motor Diagnostics
**As an** operator  
**I want** motor health reporting  
**So that** I know if motors are working

**Acceptance Criteria:**
- [ ] Report PWM values to /diagnostics
- [ ] Report commanded vs actual velocity (from encoders)
- [ ] Warn if motor not responding to commands
- [ ] Report motor temperature (if available)

## Definition of Done
- All stories completed
- Motors respond correctly to cmd_vel
- Smooth motion without jerking
- Emergency stop works
- Calibration documented and tested
- Tests passing
- Code reviewed
