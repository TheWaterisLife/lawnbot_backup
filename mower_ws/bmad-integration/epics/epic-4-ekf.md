# Epic 4: EKF Sensor Fusion

## Goal
Configure and deploy the robot_localization EKF node to fuse IMU, GPS (via rtk_reader), and wheel odometry into a single, accurate pose estimate.

## Background
The robot_localization package provides an Extended Kalman Filter (EKF) for ROS 2. Our configuration:
- **IMU**: orientation (roll, pitch, yaw from magnetometer), angular velocity
- **GPS**: absolute position via navsat_transform (subscribes to /rtk/fix from rtk_reader)
- **Wheel odometry**: velocity (vx, wz)

> [!IMPORTANT]
> GPS comes from `/rtk/fix` (rtk_reader package). We use `navsat_transform_node` to convert to local ENU.

---

## Stories

### Story 4.1: EKF Configuration
**As a** developer  
**I want** a robot_localization configuration file  
**So that** the EKF fuses our three sensors correctly

**Acceptance Criteria:**
- [ ] Configure odom0 for wheel odometry (velocity only)
- [ ] Configure imu0 for IMU (orientation + angular vel + accel)
- [ ] Configure odom1 for GPS position (via navsat_transform)
- [ ] Set 2D mode (z, roll, pitch constrained)
- [ ] Set 50 Hz output rate
- [ ] Publish odom → base_link transform

**Configuration:**
```yaml
# config/ekf.yaml
ekf_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true
    publish_tf: true
    odom_frame: "odom"
    base_link_frame: "base_link"
    world_frame: "odom"
    
    # Wheel odometry - velocity only
    odom0: /wheel/odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true, false, false,
                   false, false, true,
                   false, false, false]
    odom0_differential: false
    
    # IMU - orientation + angular velocity
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true, true, true,     # roll, pitch, yaw
                  false, false, false,
                  true, true, true,      # angular rates
                  true, true, false]     # accel x, y
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true
    
    # GPS position (from navsat_transform)
    odom1: /gps/odom
    odom1_config: [true, true, false,
                   false, false, false,
                   false, false, false,
                   false, false, false,
                   false, false, false]
    odom1_differential: false
```

---

### Story 4.2: navsat_transform Configuration
**As a** developer  
**I want** GPS coordinates converted to local ENU  
**So that** the EKF can fuse GPS position

**Acceptance Criteria:**
- [ ] navsat_transform_node subscribes to /rtk/fix (from rtk_reader)
- [ ] Publishes /gps/odom in odom frame
- [ ] Uses IMU yaw for orientation
- [ ] Origin set from first RTK-Fixed position

**Configuration:**
```yaml
navsat_transform_node:
  ros__parameters:
    frequency: 10.0
    delay: 0.0
    magnetic_declination_radians: -0.247  # Montreal ~-14.2°
    yaw_offset: 0.0
    zero_altitude: true
    broadcast_utm_transform: false
    publish_filtered_gps: true
    use_odometry_yaw: true
    wait_for_datum: false
```

**Topic Remapping:**
```python
# In launch file
('gps/fix', '/rtk/fix'),  # Remap to use rtk_reader topic
```

---

### Story 4.3: TF Configuration
**As a** navigation system  
**I want** correct TF transforms  
**So that** all data is in consistent frames

**TF Tree:**
```
map (from navsat_transform)
 └── odom (from EKF)
      └── base_link
           ├── imu_link (static: 0, 0, 0.1)
           └── gps_link (static: 0, 0, 0.3)
```

**Static Transforms:**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link imu_link
ros2 run tf2_ros static_transform_publisher 0 0 0.3 0 0 0 base_link gps_link
```

---

### Story 4.4: EKF Launch File
**As a** developer  
**I want** a launch file for the fusion stack  
**So that** I can start everything with one command

**fusion.launch.py starts:**
- robot_localization ekf_node
- robot_localization navsat_transform_node
- Static transform publishers

---

### Story 4.5: GPS Dropout Handling
**As a** navigation system  
**I want** the EKF to maintain accuracy during GPS dropout  
**So that** navigation continues using IMU + encoders

**Acceptance Criteria:**
- [ ] During 10-second GPS dropout: drift < 20 cm
- [ ] Position recovers when GPS returns
- [ ] No pose jump on recovery

---

## Definition of Done
- [ ] EKF publishing /odom_filtered at 50 Hz
- [ ] Position accuracy < 5 cm with RTK-Fixed
- [ ] GPS dropout recovery tested
- [ ] Launch files working
