# Story 2.1: navsat_transform Configuration (Updated)

> [!NOTE]
> **Original Story 2.1 was "NMEA Parser"** - This has been superseded. GPS/NMEA parsing is now handled by the `rtk_reader` package. This story covers the **navsat_transform integration**.

## User Story
**As a** sensor fusion system  
**I want** GPS lat/lon converted to local ENU coordinates  
**So that** the EKF can fuse GPS with local sensors

## Background
The `rtk_reader` package publishes `/rtk/fix` (sensor_msgs/NavSatFix) at 10Hz. We use `navsat_transform_node` from `robot_localization` to convert GPS coordinates to local odometry.

## Acceptance Criteria

- [ ] Configure navsat_transform_node in config/ekf.yaml
- [ ] Remap input from `/gps/fix` to `/rtk/fix`
- [ ] Publish `/gps/odom` as nav_msgs/Odometry
- [ ] Set magnetic declination for Montreal (-0.247 rad)
- [ ] Use IMU yaw for orientation (`use_odometry_yaw: true`)
- [ ] Test with RTK-Fixed GPS

## Configuration

```yaml
navsat_transform_node:
  ros__parameters:
    frequency: 10.0
    delay: 0.0
    magnetic_declination_radians: -0.247
    yaw_offset: 0.0
    zero_altitude: true
    broadcast_utm_transform: false
    publish_filtered_gps: true
    use_odometry_yaw: true
    wait_for_datum: false
```

## Topic Remapping

```python
# In fusion.launch.py
Node(
    package='robot_localization',
    executable='navsat_transform_node',
    remappings=[
        ('gps/fix', '/rtk/fix'),
        ('odometry/filtered', '/odom_filtered'),
    ],
)
```

## Definition of Done
- [ ] navsat_transform configured and tested
- [ ] /gps/odom published correctly
- [ ] EKF fuses GPS position
