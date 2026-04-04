# Product Requirements Document: Wheel Encoder Subsystem

## 1. Overview

### 1.1 Purpose
The Wheel Encoder subsystem converts raw GPIO pulses from wheel sensors into metric odometry (position and velocity) for the robot.

### 1.2 Scope
- Reading GPIO interrupts (Left/Right, A/B phases)
- Computing differential drive kinematics
- Publishing `/wheel/odom` and TF

---

## 2. Functional Requirements

### 2.1 Encoder Reading (FR-ENC)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-ENC-01 | Read Left Encoder A/B (GPIO 17, 27) | Must |
| FR-ENC-02 | Read Right Encoder A/B (GPIO 22, 4) | Must |
| FR-ENC-03 | Decode quadrature for direction | Must |
| FR-ENC-04 | Count ticks (32-bit signed integer) | Must |

**Acceptance Criteria:**
- Forward wheel rotation increases counts.
- Backward rotation decreases counts.

### 2.2 Odometry Calculation (FR-ODOM)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-ODOM-01 | Compute X, Y, Theta (Pose) | Must |
| FR-ODOM-02 | Compute Vx, Wz (Velocity) | Must |
| FR-ODOM-03 | Use configurable `wheel_radius` and `track_width` | Must |

### 2.3 Publication (FR-PUB)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-PUB-01 | Publish `nav_msgs/Odometry` on `/wheel/odom` | Must |
| FR-PUB-02 | Broadcast `odom` -> `base_link` TF (optional) | Must |
| FR-PUB-03 | Publish at 50 Hz | Must |

---

## 3. Non-Functional Requirements

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-01 | Max Speed | Support up to 1 m/s (~1000 ticks/sec) |
| NFR-02 | CPU Load | < 10% |
| NFR-03 | Jitter | < 5 ms |

---

## 4. Interface Specifications

### 4.1 Published Topics

| Topic | Type | Frequency |
|-------|------|-----------|
| `/wheel/odom` | nav_msgs/Odometry | 50 Hz |
| `/tf` | tf2_msgs/TFMessage | 50 Hz (if enabled) |

### 4.2 Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `left_encoder_a` | int | 17 | GPIO Pin |
| `left_encoder_b` | int | 27 | GPIO Pin |
| `right_encoder_a` | int | 22 | GPIO Pin |
| `right_encoder_b` | int | 4 | GPIO Pin |
| `wheel_radius` | double | 0.05 | Meters |
| `track_width` | double | 0.24 | Meters |
| `encoder_cpr` | int | 3960 | Counts per Rev |

---

## 5. Acceptance Criteria Summary

1. Pushing robot forward increases X in `/wheel/odom`.
2. Spinning robot in place changes orientation (Theta) correctly.
3. Velocities (linear/angular) are non-zero during motion.
4. Parameter changes (e.g., wheel radius) affect output.
