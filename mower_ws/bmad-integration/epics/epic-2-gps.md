# Epic 2: GPS Integration (via rtk_reader)

> [!IMPORTANT]
> GPS/RTK is handled by the **`rtk_reader`** package (your teammate's work). This package **subscribes** to `/rtk/fix` instead of implementing its own GPS driver.

## Goal
Integrate GPS position data from the external `rtk_reader` package into the EKF fusion via `navsat_transform_node`.

## Background

### Current Setup
The `rtk_reader` package already:
- Connects to SimpleRTK2B (u-blox F9P) via USB serial
- Handles NTRIP connection to `3.143.243.81:2101/RD1_STATION_SO`
- Parses GGA sentences
- Publishes `/rtk/fix` at 10Hz

### Key Design Decision
**IMU Magnetometer is the PRIMARY yaw source.** SimpleRTK2B is single-antenna (no heading).

---

## Stories

### Story 2.1: navsat_transform Configuration
**As a** sensor fusion system  
**I want** GPS lat/lon converted to local ENU coordinates  
**So that** the EKF can fuse GPS with local sensors

**Acceptance Criteria:**
- [ ] Configure navsat_transform_node from robot_localization
- [ ] Subscribe to `/rtk/fix` from rtk_reader
- [ ] Publish `/gps/odom` as nav_msgs/Odometry
- [ ] Use IMU yaw for orientation (`use_odometry_yaw: true`)
- [ ] Set datum from first RTK-Fixed position
- [ ] Set magnetic declination for Montreal (~-14.2°)

**Configuration:**
```yaml
navsat_transform_node:
  ros__parameters:
    frequency: 10.0
    delay: 0.0
    magnetic_declination_radians: -0.247  # Montreal
    yaw_offset: 0.0
    zero_altitude: true
    broadcast_utm_transform: false
    publish_filtered_gps: true
    use_odometry_yaw: true
    wait_for_datum: false
```

---

### Story 2.2: GPS Covariance Handling
**As a** sensor fusion system  
**I want** appropriate covariance values based on RTK fix status  
**So that** the EKF weights GPS correctly

**Acceptance Criteria:**
- [ ] Read fix status from NavSatFix.status.status
- [ ] Map status to covariance values
- [ ] Pass covariance to EKF via /gps/odom

**Covariance Mapping:**

| status.status | Meaning | Covariance (m²) |
|---------------|---------|-----------------|
| -1 | No fix | 999 |
| 0 | Fix | 6.25 (2.5m σ) |
| 1 | SBAS | 1.0 |
| 2 | GBAS (RTK) | 0.0004 (2cm σ) |

---

### Story 2.3: GPS Dropout Handling
**As a** navigation system  
**I want** graceful handling of GPS dropout  
**So that** the robot continues navigating using IMU + encoders

**Acceptance Criteria:**
- [ ] EKF continues with IMU + wheel odom during GPS dropout
- [ ] Log warning when GPS unavailable for > 5 seconds
- [ ] Resume GPS fusion when fix returns
- [ ] No pose jump on GPS return (EKF handles smoothly)

---

### Story 2.4: Launch File Integration
**As a** developer  
**I want** navsat_transform included in fusion.launch.py  
**So that** GPS is integrated with one launch command

**Acceptance Criteria:**
- [ ] fusion.launch.py starts navsat_transform_node
- [ ] Config loaded from config/ekf.yaml
- [ ] Topics remapped correctly (/rtk/fix → navsat input)

---

## External Dependency: rtk_reader

| Item | Value |
|------|-------|
| Package | `rtk_reader` |
| Topic | `/rtk/fix` |
| Message Type | `sensor_msgs/NavSatFix` |
| Rate | 10 Hz |
| Serial Port | `/dev/ttyUSB0` |
| NTRIP Host | `3.143.243.81:2101` |
| Mountpoint | `RD1_STATION_SO` |

---

## Definition of Done
- navsat_transform_node configured and tested
- GPS position fused in EKF via /gps/odom
- GPS dropout recovery verified
- Launch file updated
