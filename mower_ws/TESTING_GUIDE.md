# Sensor Integration - Testing Guide

## Summary

Sensor fusion for the autonomous lawn mower using:
- **IMU**: MPU-6500 (MPU-6050 compatible)
- **Wheel Encoders**: FIT0403 with gpiozero
- **RTK GPS**: u-blox via rtk_reader
- **EKF Fusion**: robot_localization package

---

## Hardware Configuration

### GPIO Pin Assignments

| Component | GPIO Pins | Notes |
|-----------|-----------|-------|
| **Left Encoder** | A=17, B=27 | Avoid motor pins |
| **Right Encoder** | A=22, B=4 | Avoid motor pins |
| **Motor Driver** | 5,6,12,13,18,19,23,24 | Reserved - DO NOT USE |
| **I2C (IMU)** | SDA=2, SCL=3 | Shared bus |

### I2C Devices

| Address | Device |
|---------|--------|
| **0x68** | MPU-6500/6050 IMU |
| 0x27 | I/O Expander |

### Serial Ports

| Port | Device |
|------|--------|
| `/dev/ttyACM0` | u-blox RTK GPS |

---

## Library Solutions

### Pi 5: Use gpiozero instead of RPi.GPIO

```python
from gpiozero import DigitalInputDevice
enc_a = DigitalInputDevice(17, pull_up=True)
enc_a.when_activated = callback
```

### MPU-6500: Accept WHO_AM_I = 0x70

Driver accepts: `0x68, 0x70, 0x71, 0x73`

### EKF Drift: Disable IMU orientation/acceleration

Only use angular velocity from IMU, position from GPS.

---

## ROS 2 Topics

| Topic | Type | Rate |
|-------|------|------|
| `/imu/data` | sensor_msgs/Imu | ~17 Hz |
| `/wheel/odom` | nav_msgs/Odometry | 50 Hz |
| `/rtk/fix` | sensor_msgs/NavSatFix | 1 Hz |
| `/odometry/filtered` | nav_msgs/Odometry | 50 Hz |
| `/gps/odom` | nav_msgs/Odometry | 10 Hz |

---

## Demo Instructions

### Step 1: Source Workspace

```bash
cd ~/mower_ws
source venv/bin/activate
source install/setup.bash
```

### Step 2: Start RTK GPS (Terminal 1)

```bash
ros2 run rtk_reader rtk_reader --ros-args -p port:=/dev/ttyACM0 -p baud:=115200
```

### Step 3: Start Sensor Fusion (Terminal 2)

```bash
ros2 launch sensor_integration demo.launch.py imu_type:=mpu6050
```

### Step 4: Verify Topics (Terminal 3)

```bash
ros2 topic hz /imu/data        # ~17 Hz
ros2 topic hz /wheel/odom      # 50 Hz
ros2 topic hz /rtk/fix         # 1 Hz
ros2 topic hz /odometry/filtered  # 50 Hz
```

### Step 5: Monitor Position

```bash
ros2 topic echo /odometry/filtered --field pose.pose.position
ros2 topic echo /rtk/fix --field latitude --field longitude
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `No module named 'RPi'` | `sudo apt install python3-rpi.gpio`, symlink to venv |
| `Cannot determine SOC peripheral base address` | Use gpiozero (Pi 5) |
| `WHO_AM_I mismatch` | Normal for MPU-6500, driver accepts it |
| Position drifting | Disable IMU yaw/accel in ekf.yaml |
| `/rtk/fix` not publishing | Check `ls /dev/ttyACM*`, restart rtk_reader |

---

## GPS Status Codes

| Status | Meaning | Accuracy |
|--------|---------|----------|
| -1 | No fix | N/A |
| 0 | GPS only | ~5m |
| 2 | **RTK Fixed** | **~2cm** ✓ |

---

## Encoder Calibration

**3232 ticks per revolution** (calibrated)
