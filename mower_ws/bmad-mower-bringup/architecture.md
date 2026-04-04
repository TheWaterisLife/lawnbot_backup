# Architecture: Mower Bringup Subsystem

## 1. System Context

```
+------------------------------------------------------------------+
|                        mower_bringup                              |
|                    (launch orchestrator)                           |
+------------------------------------------------------------------+
|                                                                   |
|   full_system.launch.py                                           |
|   ├── rtk_reader ──────────────── /rtk/fix                        |
|   ├── sensor_integration                                          |
|   │   ├── imu_node ────────────── /imu/data                       |
|   │   ├── wheel_odom_node ─────── /wheel/odom                     |
|   │   ├── ekf_node ────────────── /odometry/filtered              |
|   │   └── navsat_transform ────── /gps/odom                       |
|   ├── ai_camera_vision ────────── /vision/detections              |
|   ├── lawnbot_comms ───────────── ws://0.0.0.0:9002               |
|   ├── lawnbot_motors ─────────── ws://0.0.0.0:8766 (standalone)   |
|   └── mower_navigation                                            |
|       ├── coverage_planner ────── /coverage/path                   |
|       ├── motor_controller ────── /cmd_vel                         |
|       └── nav2_stack ──────────── costmap, planner                 |
|                                                                   |
+------------------------------------------------------------------+
```

---

## 2. Component Architecture

### 2.1 Package Structure (Planned)

```
src/mower_bringup/
├── mower_bringup/
│   └── __init__.py
├── launch/                          # (planned)
│   ├── full_system.launch.py        # Everything
│   ├── sensors_only.launch.py       # Sensors + EKF only
│   └── teleop.launch.py            # Manual drive mode
├── config/                          # (planned)
│   └── bringup_params.yaml          # System-wide params
├── resource/
│   └── mower_bringup
├── package.xml
├── setup.py
└── setup.cfg
```

### 2.2 Launch File Descriptions

#### full_system.launch.py (Planned)

Starts all subsystems in dependency order:

```python
# Planned structure
def generate_launch_description():
    return LaunchDescription([
        # 1. GPS (needs to start first for fix)
        Node(package='rtk_reader', executable='rtk_reader'),

        # 2. Sensor fusion (needs /rtk/fix)
        IncludeLaunchDescription(
            'sensor_integration', 'demo.launch.py',
            launch_arguments={'imu_type': 'mpu6050'}
        ),

        # 3. Camera vision (independent)
        IncludeLaunchDescription(
            'ai_camera_vision', 'camera.launch.py'
        ),

        # 4. Comms bridge (needs ROS2 topics)
        Node(package='lawnbot_comms', executable='bridge_node'),

        # 5. Navigation (needs all inputs)
        IncludeLaunchDescription(
            'mower_navigation', 'navigation.launch.py'
        ),
    ])
```

> [!NOTE]
> `lawnbot_motors` is a standalone WebSocket server (not a ROS2 node). It should be started separately via `python3 ws_motor_server.py` or a systemd service.

#### sensors_only.launch.py (Planned)

For sensor testing without navigation:
- RTK reader
- Sensor integration (IMU + encoders + EKF)
- No motors, no navigation, no camera

#### teleop.launch.py (Planned)

For manual driving:
- RTK reader
- Sensor integration
- Comms bridge (for app joystick)
- No autonomous navigation

---

## 3. Startup Sequence

### 3.1 Dependency Graph

```
Phase 1 (Independent):
  ├── rtk_reader          → /rtk/fix
  ├── ai_camera_vision    → /vision/detections
  └── lawnbot_motors      → ws://8766 (standalone)

Phase 2 (Needs Phase 1):
  ├── sensor_integration  → /imu/data, /wheel/odom, /odometry/filtered
  └── lawnbot_comms       → ws://9002

Phase 3 (Needs Phase 2):
  └── mower_navigation    → /cmd_vel, /coverage/path
```

### 3.2 Topic Readiness

| Topic | Source | Expected Time |
|-------|--------|---------------|
| `/rtk/fix` | rtk_reader | < 5 sec |
| `/imu/data` | sensor_integration | < 5 sec |
| `/wheel/odom` | sensor_integration | < 5 sec |
| `/odometry/filtered` | sensor_integration | < 10 sec |
| `/vision/detections` | ai_camera_vision | < 15 sec |

---

## 4. Network Ports

| Port | Protocol | Service | Package |
|------|----------|---------|---------|
| 8766 | WebSocket | Motor control | lawnbot_motors |
| 9002 | WebSocket | App bridge | lawnbot_comms |

---

## 5. GPIO Allocation (System-Wide)

| GPIO Pins | Function | Package |
|-----------|----------|---------|
| 2, 3 | I2C (IMU) | sensor_integration |
| 5, 6, 13, 18 | Left motor (BTS7960) | lawnbot_motors |
| 12, 19, 23, 24 | Right motor (BTS7960) | lawnbot_motors |
| 16, 20, 21, 26 | Blade motor (BTS7960) | lawnbot_motors |
| 17, 27, 22, 4 | Wheel encoders | sensor_integration |

> [!WARNING]
> No pin conflicts between packages. Motor pins and encoder pins are fully separate.

---

## 6. Serial Ports

| Port | Device | Package |
|------|--------|---------|
| `/dev/ttyACM0` | u-blox ZED-F9P GPS | rtk_reader |
| USB | OAK-D Camera | ai_camera_vision |

---

## 7. Resource Budget (Full System)

### 7.1 CPU Budget (Raspberry Pi 5)

| Subsystem | Target CPU |
|-----------|------------|
| Sensor Integration | 16% |
| AI Camera Vision | 15% |
| Nav2 Stack | 15% |
| RTK Reader | 3% |
| Motor Server | 3% |
| Comms Bridge | 2% |
| **Total** | **~54%** |

### 7.2 RAM Budget (8 GB available)

| Subsystem | Target RAM |
|-----------|------------|
| Sensor Integration | 120 MB |
| AI Camera Vision | 100 MB |
| Nav2 Stack | 200 MB |
| Other nodes | 60 MB |
| **Total** | **~480 MB** |

---

## 8. Dependencies

All subsystem packages must be installed:

| Package | Type | Description |
|---------|------|-------------|
| `rtk_reader` | ROS2 node | GPS positioning |
| `sensor_integration` | ROS2 launch | Sensor fusion |
| `ai_camera_vision` | ROS2 node | Camera detection |
| `lawnbot_comms` | ROS2 node | App WebSocket bridge |
| `lawnbot_motors` | Standalone | Motor WebSocket server |
| `mower_navigation` | ROS2 launch | Nav2 + coverage |
