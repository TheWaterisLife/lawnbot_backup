# Sensor Integration (Localization): Software Design

> **Note:** The `sensor_integration` package and EKF pipeline are deprecated. Localization is now actively performed by `mower_autonomy/localization_node.py`. This design document reflects the active localization implementation.

## 1. System / Component Diagram

This diagram shows how `localization_node` fuses raw sensor data into a continuous `Pose2D`.

```plantuml
@startuml
!include <C4/C4_Context>

title System Context — Localization Subsystem

System_Ext(in_ticks, "/encoders/ticks", "Int32MultiArray [l, r]")
System_Ext(in_imu, "/imu/data", "sensor_msgs/Imu (Yaw)")
System_Ext(in_rtk, "/rtk/fix", "sensor_msgs/NavSatFix (Lat/Lon)")

System(localization, "localization_node", "ROS 2 Node (mower_autonomy)\nFuses encoder, IMU, and RTK\nGenerates local Cartesian map")
System_Ext(out_pose, "/mower/pose", "Pose2D (x, y, θ)")

in_ticks --> localization : Differential Drive Kinematics
in_imu --> localization : Absolute Heading (Complementary Filter)
in_rtk --> localization : Global Position Offset

localization --> out_pose : Publishes continuous odometry @ 20 Hz
@enduml
```

### Component Details
- **localization_node**: Subscribes to three distinct sensor modalities. Uses encoder ticks for high-frequency motion tracking (differential drive kinematics), IMU for drift-free heading, and RTK-GPS for absolute Cartesian position correction.
- **/mower/pose**: Outputs a highly stable local Cartesian coordinate map (meters) suitable for exact waypoint navigation and path following.

---

## 2. Class Diagram

```plantuml
@startuml
!theme toy

class LocalizationNode {
    - x: float
    - y: float
    - theta: float
    - last_left_ticks: int
    - last_right_ticks: int
    - imu_heading_gain: float
    - origin_lat: float
    - origin_lon: float
    + timer_callback()
    + encoder_callback(msg)
    + imu_callback(msg)
    + rtk_callback(msg)
    - update_odometry(left_delta, right_delta)
    - gps_to_local_enu(lat, lon): Tuple[float, float]
}

class ComplementaryFilter {
    - weight: float
    - last_val: float
    + update(raw_accel, raw_gyro): float
}

LocalizationNode *-- ComplementaryFilter : heading fusion

@enduml
```

### Class Details
- **LocalizationNode**: Maintains the current integrated state vector `(x, y, theta)`. Because NavSat properties provide global position, the node establishes a *(0,0)* origin at the first GPS fix and uses the equirectangular approximation to continuously align the dead-reckoning (ticks + IMU) to the absolute world position. 
- **Complementary Filter**: Applies the `imu_heading_gain` to blend raw encoder heading (susceptible to wheel slip) with IMU heading (absolute but optionally noisy in sharp jerks).

---

## 3. Sequence Diagram

This sequence diagram illustrates how the different asynchronous sensor feeds are fused to provide a continuous Pose2D output.

```plantuml
@startuml
!theme toy

participant "encoder_node" as ENC
participant "imu_node" as IMU
participant "rtk_reader" as RTK
participant "localization_node" as LOC
participant "/mower/pose" as POSE

loop High Frequency (50 Hz)
    ENC -> LOC : /encoders/ticks [1040, 1042]
    LOC -> LOC : update_odometry(delta_l, delta_r)
    LOC -> LOC : Predict (x, y, theta)
end

loop Medium Frequency (20 Hz)
    IMU -> LOC : /imu/data
    LOC -> LOC : Extract Quaternion Yaw
    LOC -> LOC : Complementary Filter Fusion\n(theta = (1-k)*odom + (k)*imu)
end

loop Low Frequency (5 Hz)
    RTK -> LOC : /rtk/fix (Lat, Lon)
    LOC -> LOC : Convert to Local ENU (meters)
    LOC -> LOC : Overwrite Prediction (x=GPS_x, y=GPS_y)
end

loop Publisher Loop (20 Hz)
    LOC -> POSE : Publish Pose2D(x, y, theta)
end

@enduml
```

### Sequence Flow Details
- **Prediction Step**: Every time encoder ticks arrive, the node mathematically drives the current pose forward along `theta`. 
- **Orientation Correction**: The IMU updates asynchronously to drag the predicted heading towards truth, dampening cumulative wheel slip error.
- **Position Correction**: The RTK fix snaps the dead-reckoned X/Y coordinates to their true global offsets, eliminating coordinate drift without requiring a heavy EKF covariance matrix.
