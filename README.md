# 🤖 Autonomous Lawn Mower — On-Board Software Backup

> **COEN/ELEC 490 Capstone — Team 25**
> Full Raspberry Pi 5 filesystem backup of the autonomous lawn mower's ROS 2 workspace, vision pipeline, RTK-GPS configuration, and utility scripts.

---

## Overview

This repository is a complete backup of the software running on the mower's Raspberry Pi 5 (8 GB, Ubuntu 24.04). The robot uses **ROS 2 Jazzy** with a custom Python-based autonomy stack—no Nav2 at runtime—communicating motor commands over WebSocket for minimal latency.

### Key Capabilities

| Feature | How |
|---------|-----|
| **Differential-drive motor control** | BTS7960 H-bridges via `gpiozero`, exposed as a WebSocket server (port 8766) |
| **RTK-GPS positioning** | u-blox ZED-F9P + NTRIP corrections → ~2 cm accuracy |
| **IMU-fused localization** | BNO085 quaternion heading + encoder odometry via complementary filter |
| **AI obstacle detection** | OAK-D Lite running YOLOv3-tiny + DeepLabV3+ on the Myriad X VPU |
| **Autonomous mowing** | Boustrophedon coverage planner with geofence enforcement |
| **Boundary mapping** | GPS-based perimeter recording with RDP simplification |
| **Mobile app control** | WebSocket bridge for teleoperation, mapping, and telemetry |
| **Battery monitoring** | LiFePO₄ voltage via ADS1015 ADC with 5-LED bar indicator |

---

## Repository Layout

```
lawnbot_backup/
├── mower_ws/                        # Main ROS 2 colcon workspace
│   ├── src/                         #   Source packages (see table below)
│   ├── bmad-*/                      #   BMAD design docs per subsystem
│   ├── scripts/                     #   Helper scripts (start_rtk_reader.sh)
│   ├── maps/                        #   Saved boundary map JSON files
│   └── TESTING_GUIDE.md             #   Sensor integration test guide
│
├── ai_camera_vision/                # OAK-D Lite vision pipeline (standalone copy)
│   ├── demo_headless.py             #   Production (no display) entry point
│   ├── demo_live_view.py            #   Debug visualization entry point
│   └── models/                      #   YOLOv3-tiny & DeepLabV3+ .blob files
│
├── rtk/                             # RTK rover helper script
├── rtk_lc29h.conf                   # RTK module configuration
├── libraries installed on the mower/# Freeze lists (apt, pip, ROS 2 packages)
├── motortest.py                     # Quick BTS7960 motor validation script
├── stepper_test.py                  # Stepper motor test script
├── section_5_software_design.md     # Capstone report — Section 5 (Software Design)
└── missing_paragraphs_and_diagrams.md
```

### ROS 2 Packages (`mower_ws/src/`)

| Package | Type | Description |
|---------|------|-------------|
| `lawnbot_motors` | Standalone | BTS7960 motor drivers via async WebSocket server (port 8766) |
| `imu_reader` | ROS 2 Node | BNO085 / MPU-6050 IMU driver with factory-pattern abstraction |
| `rtk_reader` | ROS 2 Node | ZED-F9P serial reader + NTRIP client → `/rtk/fix` |
| `lawn_mower_battery` | ROS 2 Node | ADS1015 ADC battery monitor + MCP23017 LED bar |
| `mower_autonomy` | ROS 2 Node | Encoder driver, localization, coverage planner, autonomy state machine |
| `mower_mapping` | ROS 2 Node | GPS boundary recording with WebSocket API (port 8770) |
| `obstacle_detection` | Standalone | OAK-D YOLO + depth-based avoidance controller |
| `lawnbot_comms` | ROS 2 Node | WebSocket bridge for mobile app (port 9002) |
| `mower_bringup` | Launch | `mower_stack.launch.py` — orchestrates full system startup |
| `ai_camera_vision` | Library | Dual-model DepthAI pipeline (YOLO + segmentation) |
| `sensor_integration` | Legacy | Superseded by `mower_autonomy` |
| `mower_navigation` | Legacy | Nav2-based navigation (never deployed) |
| `wheel_encoder` | Legacy | Superseded by `mower_autonomy` |

---

## Hardware

| Component | Model |
|-----------|-------|
| Compute | Raspberry Pi 5 (8 GB) |
| OS | Ubuntu 24.04 (Noble) |
| Motors | 2× BTS7960 H-bridge (tracks) + 1× BTS7960 (blade) |
| Camera | OAK-D Lite (Myriad X VPU) |
| IMU | BNO085 (9-DOF, on-chip fusion) |
| GPS | u-blox ZED-F9P (RTK, ~2 cm) |
| Encoders | 2× FIT0403 hall-effect quadrature (3232 CPR) |
| Battery | 12.8 V LiFePO₄ |
| ADC | ADS1015 (battery voltage) |
| GPIO Expander | MCP23017 (LED indicators) |

---

## Getting Started

### Prerequisites

- Raspberry Pi 5 running **Ubuntu 24.04**
- **ROS 2 Jazzy** installed (`ros-jazzy-desktop` or `ros-jazzy-ros-base`)
- Python 3.12

### Build

```bash
cd ~/mower_ws
python3 -m venv venv --system-site-packages
source venv/bin/activate
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### Run

**Motor server** (runs as a systemd service, not a ROS 2 node):
```bash
python3 ~/mower_ws/src/lawnbot_motors/lawnbot_motors/ws_motor_server.py
```

**Full sensor + autonomy stack**:
```bash
ros2 launch mower_bringup mower_stack.launch.py
```

### WebSocket Ports

| Port | Service |
|------|---------|
| 8766 | Motor control (`lawnbot_motors`) |
| 8770 | Map recording (`mower_mapping`) |
| 9002 | Mobile app bridge (`lawnbot_comms`) |

---

## Architecture

```
┌──────────────────────────── Sensor Layer ─────────────────────────────┐
│  rtk_reader (/rtk/fix)   imu_reader (/imu/data)   encoder_node      │
│                                                   (/encoders/ticks)  │
│  lawn_mower_battery (/battery)                                       │
└──────────────────────────────┬────────────────────────────────────────┘
                               │
┌──────────────────────────── Fusion Layer ─────────────────────────────┐
│  localization_node  →  /mower/pose  (complementary filter)           │
│      θ_fused = (1-k)·θ_encoder + k·θ_imu   (k = 0.30)              │
│      GPS overwrites (x, y) when RTK fix available                    │
└──────────────────────────────┬────────────────────────────────────────┘
                               │
┌──────────────────────────── Decision Layer ───────────────────────────┐
│  autonomy_node (state machine: IDLE→FOLLOW→TURN→RECOVER→DONE)       │
│  obstacle_detection (YOLO + depth → distance-layered avoidance)      │
│  mower_mapping (boundary record/load via WebSocket)                  │
└──────────────────────────────┬────────────────────────────────────────┘
                               │ WebSocket
                        ┌──────┴──────┐
                        │lawnbot_motors│ ← also serves mobile app
                        │  (port 8766) │
                        └─────────────┘
```

---

## Key Dependencies

| Library | Purpose |
|---------|---------|
| `rclpy` (ROS 2 Jazzy) | ROS 2 Python client |
| `gpiozero` / `lgpio` | GPIO & PWM on Pi 5 |
| `websockets` | Async WebSocket servers |
| `depthai` | OAK-D Lite camera SDK |
| `pyserial` | ZED-F9P serial comms |
| `shapely` | Polygon geometry (geofence, coverage planner) |
| `adafruit-circuitpython-bno08x` | BNO085 IMU driver |
| `adafruit-circuitpython-ads1x15` | ADS1015 ADC driver |
| `smbus2` | I2C register access |
| `numpy` | Numerical operations |

Full freeze lists are in `libraries installed on the mower/`.

---

## Team

| Name | Student ID |
|------|-----------|
| Mustafa Aboabdullah | 40199998 |
| Laith Qasem | 40200060 |
| Nicolas Gharzouzi | 40232064 |
| Iliass Bouhsane | 40263483 |
| Chadi El Tannir | 40211031 |
| Samy Belmihoub | 40251504 |

> **Concordia University — COEN/ELEC 490 Capstone, Phase 4 — March 2026**
