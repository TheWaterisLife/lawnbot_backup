# Product Requirements Document: Mower Bringup Subsystem

## 1. Overview

### 1.1 Purpose
This document defines the requirements for the Mower Bringup package, which provides top-level launch orchestration for the autonomous lawn mower system.

### 1.2 Scope
- Launch files for full system, sensors-only, and teleop modes
- Startup sequencing and dependency management
- System-wide parameter configuration
- Health verification after startup

### 1.3 Current Status

> [!NOTE]
> This package is currently a skeleton. Launch files are planned but not yet implemented.

---

## 2. Functional Requirements

### 2.1 Full System Launch (FR-FULL)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-FULL-01 | Launch all subsystems with one command | Must |
| FR-FULL-02 | Start RTK reader first (GPS fix needed) | Must |
| FR-FULL-03 | Start sensor integration after RTK | Must |
| FR-FULL-04 | Start camera vision independently | Must |
| FR-FULL-05 | Start navigation after sensors ready | Must |
| FR-FULL-06 | Start comms bridge for app connectivity | Must |

**Launch Order:**
1. `rtk_reader` — GPS positioning
2. `sensor_integration` — IMU, encoders, EKF fusion
3. `ai_camera_vision` — Obstacle detection
4. `lawnbot_comms` — App bridge
5. `mower_navigation` — Path planning + motor control

**Acceptance Criteria:**
- All nodes running within 30 seconds
- All topics publishing within 60 seconds
- RTK fix within 120 seconds (sky-dependent)

### 2.2 Sensors-Only Launch (FR-SENS)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-SENS-01 | Launch only sensor subsystems | Should |
| FR-SENS-02 | Include RTK, IMU, encoders, EKF | Should |
| FR-SENS-03 | No motor or navigation nodes | Should |

### 2.3 Teleop Launch (FR-TELEOP)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-TELEOP-01 | Launch sensors + comms + motor server | Should |
| FR-TELEOP-02 | No autonomous navigation | Should |
| FR-TELEOP-03 | Allow manual joystick driving | Should |

### 2.4 Configuration Management (FR-CFG)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-CFG-01 | Pass IMU type parameter to sensor_integration | Must |
| FR-CFG-02 | Pass serial port to rtk_reader | Must |
| FR-CFG-03 | Support launch argument overrides | Should |

---

## 3. Non-Functional Requirements

### 3.1 Performance (NFR-PERF)

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-PERF-01 | Total system startup time | < 30 sec |
| NFR-PERF-02 | Total CPU usage (all subsystems) | < 60% |
| NFR-PERF-03 | Total RAM usage | < 1 GB |

### 3.2 Subsystem Resource Budgets

| Subsystem | CPU | RAM |
|-----------|-----|-----|
| Sensor Integration | 16% | 120 MB |
| AI Camera Vision | 15% | 100 MB |
| Nav2 Stack | 15% | 200 MB |
| RTK Reader | 3% | 20 MB |
| Motor Server | 3% | 20 MB |
| Comms Bridge | 2% | 20 MB |
| **Total** | **~54%** | **~480 MB** |

---

## 4. Interface Specifications

### 4.1 Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `imu_type` | `mpu6050` | IMU model (mpu6050, bno085) |
| `serial_port` | `/dev/ttyACM0` | RTK GPS serial port |
| `enable_camera` | `true` | Enable camera vision |
| `enable_nav` | `true` | Enable navigation |
| `zone` | `front_yard` | Default mowing zone |

### 4.2 Ports Used

| Port | Service | Package |
|------|---------|---------|
| 8766 | Motor WebSocket | lawnbot_motors |
| 9002 | App Bridge WebSocket | lawnbot_comms |

### 4.3 Serial Ports

| Port | Device | Package |
|------|--------|---------|
| `/dev/ttyACM0` | u-blox ZED-F9P | rtk_reader |

---

## 5. Dependencies

| Package | Purpose |
|---------|---------|
| rtk_reader | GPS positioning |
| sensor_integration | Sensor fusion |
| ai_camera_vision | Obstacle detection |
| lawnbot_comms | App communication |
| lawnbot_motors | Motor control |
| mower_navigation | Path planning |

---

## 6. Acceptance Criteria Summary

The Mower Bringup subsystem is complete when:

1. Full system launches with one command
2. All subsystems start in correct order
3. Sensors-only and teleop modes available
4. Launch arguments override defaults
5. All topics publishing within 60 seconds
6. Total resource usage within Pi 5 budget
