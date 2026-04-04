# Architecture: Sensor Integration Subsystem

> [!NOTE]
> **Hardware Configuration**: This project uses a **SimpleRTK2B (u-blox ZED-F9P)** for RTK-GPS position via `/dev/ttyACM0`. The GPS handling is done by the **rtk_reader** package, which publishes to `/rtk/fix`. The `sensor_integration` package subscribes to this topic instead of implementing its own GPS driver.

> [!IMPORTANT]
> **Pi 5 Compatibility**: Uses **gpiozero** library instead of RPi.GPIO for encoder reading. **IMU**: MPU-6500 (MPU-6050 compatible, WHO_AM_I accepts 0x68, 0x70, 0x71, 0x73).

---

## 1. System Context

```
+------------------------------------------------------------------------------+
|                         Autonomous Lawn Mower                                 |
+------------------------------------------------------------------------------+
|                                                                               |
|  +------------------+     +------------------+     +-----------------+        |
|  |  AI Camera       |     |    Navigation    |     |   Mobile App    |        |
|  |  Vision          |     |    Subsystem     |     |  (via WebSocket)|        |
|  |  (existing)      |     |  (mower_nav)     |     |   Port 8766     |        |
|  +------------------+     +--------+---------+     +-----------------+        |
|                                    |                       |                  |
|                                    | subscribes to         |                  |
|                                    v                       v                  |
|  +----------------------------------------------------------------+          |
|  |               SENSOR INTEGRATION SUBSYSTEM                      |          |
|  |                      (this project)                             |          |
|  |                                                                 |          |
|  |   /odom_filtered  <--  EKF Fusion Node (robot_localization)     |          |
|  |                           ^                                     |          |
|  |         +-----------------+------------------+                  |          |
|  |         |                 |                  |                  |          |
|  |   IMU Node           (rtk_reader)      Odom Node                |          |
|  |  (MPU6050/BNO085)    /rtk/fix         (FIT0403 encoders)        |          |
|  |         ^                 ^                  ^                  |          |
|  |         | (Yaw+Accel)     | (Pos)            | (Velocity)       |          |
|  +---------|-----------------|-----------------^|------------------+          |
|            |                 |                  |                             |
+------------|-----------------|--[ EXTERNAL ]----|-----------------------------+
             |                 |                  |
        +----+----+   +--------+--------+   +-----+-----+
        | MPU6050 |   |   rtk_reader    |   | FIT0403   |
        | or      |   |  (separate pkg) |   | Hall Enc  |
        | BNO085  |   |  u-blox F9P     |   | Quadrature|
        | (I2C)   |   |  + NTRIP        |   | (GPIO)    |
        +---------+   +-----------------+   +-----------+
```

### External Dependencies

| Package | Topic | Message Type | Description |
|---------|-------|--------------|-------------|
| `rtk_reader` | `/rtk/fix` | `sensor_msgs/NavSatFix` | RTK GPS position @ 1Hz |
| `lawnbot_motors` | N/A | N/A | Motor control via WebSocket (independent) |

---

## 2. Component Architecture

### 2.1 Package Structure

```
src/sensor_integration/
├── sensor_integration/          # Main Python package
│   ├── __init__.py
│   └── dummy.py                 # (Optional placeholder)
│
├── config/
│   ├── ekf.yaml                 # robot_localization params
│   └── sensors.yaml             # Shared sensor parameters
│
├── launch/
│   ├── sensors.launch.py        # Includes imu_reader and wheel_encoder launch files
│   ├── fusion.launch.py         # Start EKF
│   └── demo.launch.py           # Full demo
│
├── package.xml
├── setup.py
└── setup.cfg
```

> [!NOTE]
> **No gps_node.py**: GPS is handled by `rtk_reader` package. This package just subscribes to `/rtk/fix`.

---

### 2.2 Node Descriptions

#### IMU Node (`imu_node.py`)

**Responsibility:** Read IMU sensor (MPU-6500/MPU-6050 compatible) and publish angular velocity for EKF fusion.

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | ~17 Hz | Angular velocity (orientation disabled for drift prevention) |
| `/diagnostics` | `DiagnosticArray` | 1 Hz | Health status |

**Parameters:**

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `imu_type` | string | "mpu6050" | "mpu6050" (also supports MPU-6500) |
| `i2c_bus` | int | 1 | I2C bus number |
| `i2c_address` | int | 0x68 | MPU-6500/6050 address |
| `publish_rate` | float | 17.0 | Hz (tested rate) |
| `frame_id` | string | "imu_link" | TF frame |
| `who_am_i_accepted` | list | [0x68, 0x70, 0x71, 0x73] | Valid WHO_AM_I values for MPU-6500 |

---

#### Wheel Odometry Node (`wheel_odom_node.py`)

**Responsibility:** Read encoders via GPIO quadrature, compute differential drive odometry.

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/wheel/odom` | `nav_msgs/Odometry` | 50 Hz | Velocity (vx, wz) + integrated pose |
| `/diagnostics` | `DiagnosticArray` | 1 Hz | Encoder health |

**GPIO Pin Assignment:**

> [!WARNING]
> **Pin Conflict Avoided**: These pins do NOT overlap with motor driver pins.

| Encoder | Channel | GPIO | Physical Pin | Notes |
|---------|---------|------|--------------|-------|
| Left | A | 17 | 11 | Interrupt-capable |
| Left | B | 27 | 13 | Interrupt-capable |
| Right | A | 22 | 15 | Interrupt-capable |
| Right | B | 4 | 7 | Interrupt-capable |

**Motor Pins (DO NOT USE for encoders):**
- Left motor: GPIO 5, 6, 13, 18
- Right motor: GPIO 23, 24, 12, 19
- Blade motor: GPIO 20, 16, 26, 21

**Parameters:**

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `left_encoder_a` | int | 17 | GPIO pin |
| `left_encoder_b` | int | 27 | GPIO pin |
| `right_encoder_a` | int | 22 | GPIO pin |
| `right_encoder_b` | int | 4 | GPIO pin |
| `wheel_radius` | float | 0.05 | Sprocket radius (meters) |
| `track_width` | float | 0.24 | Distance between tracks (meters) |
| `encoder_cpr` | int | 3232 | Counts per revolution (calibrated) |
| `publish_rate` | float | 50.0 | Hz |

---

## 3. Data Flow

### 3.1 Topic Graph

```
                    +---------------+
                    |  rtk_reader   |
                    |  (external)   |
                    +-------+-------+
                            |
                            | /rtk/fix (NavSatFix @ 1Hz)
                            v
+---------------+   +-------+-------+   +---------------+
|   imu_node    |   | navsat_trans  |   | wheel_odom    |
| (MPU/BNO085)  |   |    _node      |   |    _node      |
+-------+-------+   +-------+-------+   +-------+-------+
        |                   |                   |
        | /imu/data         | /gps/odom         | /wheel/odom
        | (~17Hz)           | (10Hz)            | (50Hz)
        v                   v                   v
+-------+-------------------+-------------------+-------+
|                     ekf_node                          |
|               (robot_localization)                    |
+-------------------------------------------------------+
                            |
                            | /odom_filtered (50Hz)
                            v
                +------------------------+
                |   Navigation Stack     |
                |   (mower_navigation)   |
                +------------------------+
```

### 3.2 EKF Configuration

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
    
    # Wheel odometry - velocity only (drift in position)
    odom0: /wheel/odom
    odom0_config: [false, false, false,   # x, y, z - NO
                   false, false, false,   # roll, pitch, yaw - NO
                   true, false, false,    # vx - YES
                   false, false, true,    # wz - YES
                   false, false, false]
    odom0_differential: false
    
    # IMU - angular velocity ONLY (orientation/accel disabled to prevent drift)
    imu0: /imu/data
    imu0_config: [false, false, false,   # x, y, z - NO
                  false, false, false,   # roll, pitch, yaw - NO (disabled for drift)
                  false, false, false,   # velocities - NO
                  false, false, true,    # angular rates - wz only YES
                  false, false, false]   # accel - NO (disabled for drift)
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true
    
    # GPS position (from navsat_transform)
    odom1: /gps/odom
    odom1_config: [true, true, false,    # x, y - YES
                   false, false, false,
                   false, false, false,
                   false, false, false,
                   false, false, false]
    odom1_differential: false

# NavSat Transform (converts /rtk/fix to /gps/odom)
navsat_transform_node:
  ros__parameters:
    frequency: 10.0
    delay: 0.0
    magnetic_declination_radians: -0.247  # Montreal ~-14.2°
    yaw_offset: 0.0
    zero_altitude: true
    broadcast_utm_transform: false
    publish_filtered_gps: true
    use_odometry_yaw: true  # Use IMU yaw
    wait_for_datum: false
```

---

## 4. Coordinate Frames

### 4.1 TF Tree

```
map (from navsat_transform)
 └── odom (from EKF)
      └── base_link
           ├── imu_link (static: 0, 0, 0.1)
           └── gps_link (static: 0, 0, 0.3)
```

### 4.2 Frame Definitions

| Frame | Description | Origin |
|-------|-------------|--------|
| `map` | Global ENU frame | First RTK fix |
| `odom` | Continuous odometry | Robot start |
| `base_link` | Robot center | Ground level, center of mower |
| `imu_link` | IMU sensor | 10cm above base_link |
| `gps_link` | GPS antenna | 30cm above base_link |

---

## 5. Hardware Configuration

### 5.1 I2C Devices

| Device | Address | Bus | Notes |
|--------|---------|-----|-------|
| MPU-6500 | 0x68 | 1 | IMU (MPU-6050 compatible, WHO_AM_I: 0x70) |
| I/O Expander | 0x27 | 1 | GPIO extender |

### 5.2 GPIO Summary

| Function | GPIO Pins | Notes |
|----------|-----------|-------|
| **Encoders** | 17, 27, 22, 4 | Quadrature inputs |
| **Motors** (lawnbot_motors) | 5, 6, 12, 13, 18, 19, 23, 24 | BTS7960 PWM |
| **Blade** (lawnbot_motors) | 16, 20, 21, 26 | BTS7960 PWM |
| **I2C** | 2, 3 | Reserved for IMU |

### 5.3 USB Devices

| Device | Port | Baud | Package |
|--------|------|------|---------|
| SimpleRTK2B | `/dev/ttyACM0` | 115200 | rtk_reader |

---

## 6. NTRIP Configuration

> [!NOTE]
> NTRIP is handled by `rtk_reader` package, not `sensor_integration`.

**Current Configuration (from rtk_reader):**

| Parameter | Value |
|-----------|-------|
| Host | `3.143.243.81` |
| Port | `2101` |
| Mountpoint | `RD1_STATION_SO` |
| Username | (configured in rtk_reader) |

---

## 7. Error Handling

### 7.1 Sensor Failure Modes

| Failure | Detection | Response |
|---------|-----------|----------|
| IMU I2C timeout | No data 100ms | Retry, then alert |
| GPS no fix | `/rtk/fix` status = 0 | Continue with high covariance |
| RTK degraded | status 4→5→1 | Log warning, increase covariance |
| Encoder stuck | No pulses while IMU moving | Increase velocity covariance |

### 7.2 EKF Degraded Modes

| Scenario | Sensors Available | Accuracy |
|----------|-------------------|----------|
| Normal | GPS + IMU + Encoders | < 5 cm |
| GPS dropout | IMU + Encoders | ~10 cm/10s drift |
| RTK Float | GPS (float) + IMU + Encoders | ~30 cm |
| IMU failure | GPS + Encoders | Position OK, heading degraded |

---

## 8. Launch Files

### 8.1 sensors.launch.py

Starts IMU and wheel odometry nodes:

```python
ros2 launch sensor_integration sensors.launch.py imu_type:=bno085
```

### 8.2 fusion.launch.py

Starts EKF and navsat_transform:

```python
ros2 launch sensor_integration fusion.launch.py
```

### 8.3 demo.launch.py

Full system with visualization:

```python
ros2 launch sensor_integration demo.launch.py
```

---

## 9. Dependencies

### 9.1 ROS 2 Packages

- `rclpy`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`
- `tf2_ros`
- `robot_localization`
- `diagnostic_msgs`

### 9.2 Python Libraries

- `smbus2` - I2C communication
- `gpiozero` - GPIO interrupts (Pi 5 compatible)
- `numpy` - Math operations

### 9.3 External Packages (not in this repo)

- `rtk_reader` - GPS/RTK handling
- `lawnbot_motors` - Motor control
