# Software Design — Mower Bringup (`mower_bringup`)

## 1. Overview

The `mower_bringup` package serves as the system-level launch orchestrator for the autonomous lawn mower. The actual launch file (`mower_stack.launch.py`) resides in the `mower_autonomy` package and starts all 8 ROS 2 nodes in dependency order. The `mower_bringup` package exists as a placeholder for future system-wide launch configuration. The `lawnbot_motors` WebSocket server runs separately as a standalone process or systemd service.

### 1.1 Design Rationale

A single launch file starts the entire ROS 2 stack in the correct dependency order. Nodes are grouped into three phases: independent sensor nodes (Phase 1), fusion/processing nodes (Phase 2), and decision/control nodes (Phase 3). This ordering ensures that topics are available before subscribers attempt to use them, minimizing startup race conditions.

### 1.2 Key Responsibilities

- Orchestrate the launch of all ROS 2 nodes in dependency order
- Define the startup configuration (parameters, node names, remappings)
- Document system-wide resource allocation (GPIO, ports, CPU, RAM)
- Ensure no GPIO pin conflicts between packages

---

## 2. Class Diagram

```plantuml
@startuml
skinparam classAttributeIconSize 0

package "System Launch Architecture" {

  class MowerStackLaunch <<launch file>> {
    +generate_launch_description() : LaunchDescription
  }

  class Phase1_Sensors <<group>> {
    rtk_reader : RTKReader
    imu_node : ImuNode
    encoder_node : EncoderNode
  }

  class Phase2_Fusion <<group>> {
    localization_node : LocalizationNode
    obstacle_detector_node : ObstacleDetectorNode
    map_server : MapServer
  }

  class Phase3_Control <<group>> {
    autonomy_node : AutonomyNode
    bridge_node : BridgeNode
  }

  class StandaloneServices <<group>> {
    lawnbot_motors : WSMotorServer
  }

  MowerStackLaunch --> Phase1_Sensors : launches first\n(no dependencies)
  MowerStackLaunch --> Phase2_Fusion : launches second\n(needs sensor topics)
  MowerStackLaunch --> Phase3_Control : launches third\n(needs fusion topics)
  StandaloneServices ..> MowerStackLaunch : runs separately\n(systemd service)
}

package "ROS 2 Topics (Inter-node)" {

  class SensorTopics <<topics>> {
    /rtk/fix : NavSatFix
    /imu/data : Imu
    /encoders/ticks : Int32MultiArray
    /battery : BatteryState
  }

  class FusionTopics <<topics>> {
    /mower/pose : Pose2D
    /obstacles/detections : String
  }

  class ControlTopics <<topics>> {
    /autonomy/start : Empty
    /autonomy/stop : Empty
    /autonomy/running : Bool
    /app/heartbeat : Empty
    /app/mode : String
    /app/teleop_cmd : Twist
  }

  Phase1_Sensors --> SensorTopics : publishes
  Phase2_Fusion --> FusionTopics : publishes
  Phase2_Fusion ..> SensorTopics : subscribes
  Phase3_Control ..> FusionTopics : subscribes
  Phase3_Control --> ControlTopics : publishes
}

@enduml
```

---

## 3. Sequence Diagram

```plantuml
@startuml
skinparam sequenceArrowThickness 2

participant "mower_stack\n.launch.py" as Launch
participant "rtk_reader" as RTK
participant "imu_node" as IMU
participant "encoder_node" as ENC
participant "localization_node" as LOC
participant "obstacle_detector\n_node" as OBS
participant "map_server" as MAP
participant "autonomy_node" as AUTO
participant "bridge_node" as BRIDGE
participant "lawnbot_motors\n(standalone)" as MOTORS

== Phase 0: Standalone Service ==
note over MOTORS : Started separately\nvia systemd service
MOTORS -> MOTORS : ws://0.0.0.0:8766 ready

== Phase 1: Sensor Nodes (Independent) ==
Launch -> RTK : start node
RTK -> RTK : open serial /dev/ttyACM0
RTK -> RTK : start NTRIP thread
note right of RTK : /rtk/fix ready (~5s)

Launch -> IMU : start node (bno085, 0x4A, 20Hz)
IMU -> IMU : IMUFactory.create("bno085")
note right of IMU : /imu/data ready (~3s)

Launch -> ENC : start node (GPIO 17,27,22,4)
ENC -> ENC : init quadrature interrupts
note right of ENC : /encoders/ticks ready (~2s)

== Phase 2: Fusion Nodes (Need Phase 1 Topics) ==
Launch -> LOC : start node (imu_heading_gain=0.30)
LOC -> LOC : subscribe /rtk/fix, /imu/data, /encoders/ticks
LOC -> LOC : wait for first sensor readings
note right of LOC : /mower/pose ready (~5s)

Launch -> OBS : start node (max_range=0.50m)
OBS -> OBS : init OAK-D pipeline
note right of OBS : /obstacles/detections ready (~15s)

Launch -> MAP : start node
MAP -> MAP : subscribe /rtk/fix
MAP -> MAP : start WebSocket ws://8770
note right of MAP : Map server ready (~3s)

== Phase 3: Control Nodes (Need Phase 2 Topics) ==
Launch -> AUTO : start node
AUTO -> AUTO : subscribe /mower/pose, /encoders/ticks
AUTO -> AUTO : connect WebSocket to ws://8766
note right of AUTO : Autonomy ready, IDLE state

Launch -> BRIDGE : start node
BRIDGE -> BRIDGE : start WebSocket ws://9002
BRIDGE -> BRIDGE : subscribe /mower/state, /mower/telemetry
note right of BRIDGE : Bridge ready for app

== System Running ==
note over RTK, BRIDGE : All nodes active\n~37% CPU, ~290 MB RAM\non Raspberry Pi 5

@enduml
```

---

## 4. System Context Diagram

```plantuml
@startuml
!include <C4/C4_Context>

title System Context — Full Mower Stack

Person(user, "Operator", "Mobile app user")

System_Boundary(mower, "Autonomous Lawn Mower (Raspberry Pi 5)") {

  System(rtk, "rtk_reader", "GPS positioning\n/rtk/fix")
  System(imu, "imu_reader", "IMU sensor driver\n/imu/data")
  System(encoder, "encoder_node", "Wheel encoders\n/encoders/ticks")
  System(battery, "lawn_mower_battery", "Battery monitoring\n/battery")
  System(localization, "localization_node", "Dead-reckoning\n+ IMU + GPS fusion\n/mower/pose")
  System(obstacle, "obstacle_detector", "OAK-D YOLO\n/obstacles/detections")
  System(autonomy, "autonomy_node", "Mowing state machine\nMotor commands via WS")
  System(mapping, "mower_mapping", "Boundary recording\nws://8770")
  System(comms, "lawnbot_comms", "App bridge\nws://9002")
  System(motors, "lawnbot_motors", "Motor control\nws://8766")
}

System_Ext(zedf9p, "ZED-F9P GPS", "RTK positioning")
System_Ext(bno085, "BNO085 IMU", "I2C 0x4A")
System_Ext(oakd, "OAK-D Lite", "Stereo camera + VPU")
System_Ext(bts7960, "BTS7960 Drivers", "3× H-bridge motors")

user --> comms : WiFi WebSocket
user --> mapping : WiFi WebSocket
zedf9p --> rtk
bno085 --> imu
oakd --> obstacle
motors --> bts7960
rtk --> localization
imu --> localization
encoder --> localization
localization --> autonomy
autonomy --> motors

@enduml
```

---

## 5. Detailed Design

### 5.1 Launch Sequence

| Order | Node | Package | Key Parameters | Dependencies |
|---|---|---|---|---|
| 1 | `rtk_reader` | `rtk_reader` | serial: /dev/ttyACM0 | None |
| 2 | `imu_node` | `imu_reader` | imu_type: bno085, addr: 0x4A | None |
| 3 | `encoder_node` | `mower_autonomy` | la:17, lb:27, ra:22, rb:4 | None |
| 4 | `localization_node` | `mower_autonomy` | imu_heading_gain: 0.30 | /rtk/fix, /imu/data, /encoders/ticks |
| 5 | `obstacle_detector_node` | `mower_autonomy` | max_detection_range_m: 0.50 | None |
| 6 | `autonomy_node` | `mower_autonomy` | — | /mower/pose, /encoders/ticks, map |
| 7 | `map_server` | `mower_mapping` | — | /rtk/fix |
| 8 | `bridge_node` | `lawnbot_comms` | — | ROS 2 topics |
| — | `ws_motor_server` | `lawnbot_motors` | systemd service | Standalone, port 8766 |

### 5.2 Network Port Allocation

| Port | Protocol | Service | Package |
|---|---|---|---|
| 8766 | WebSocket | Motor control | `lawnbot_motors` |
| 8770 | WebSocket | Map recording | `mower_mapping` |
| 9002 | WebSocket | Mobile app bridge | `lawnbot_comms` |

### 5.3 GPIO Allocation (System-Wide)

| GPIO Pins | Function | Package |
|---|---|---|
| 2, 3 | I2C bus 1 (IMU, ADC, GPIO expander) | `imu_reader`, `lawn_mower_battery` |
| 5, 6, 13, 18 | Left motor (BTS7960) | `lawnbot_motors` |
| 23, 24, 12, 19 | Right motor (BTS7960) | `lawnbot_motors` |
| 20, 16, 26, 21 | Blade motor (BTS7960) | `lawnbot_motors` |
| 17, 27, 22, 4 | Wheel encoders (quadrature) | `mower_autonomy` |

No pin conflicts exist between any packages.

### 5.4 Serial Port Allocation

| Port | Device | Package |
|---|---|---|
| `/dev/ttyACM0` | u-blox ZED-F9P GPS | `rtk_reader` |
| USB | OAK-D Lite Camera | `mower_autonomy` (obstacle detection) |

### 5.5 Resource Budget (Raspberry Pi 5, 8 GB RAM)

| Subsystem | CPU (%) | RAM (MB) |
|---|---|---|
| IMU reader + Encoder node | 5 | 40 |
| Localization node | 3 | 30 |
| AI Camera / Obstacle detection | 15 | 100 |
| Autonomy node | 5 | 50 |
| RTK Reader | 3 | 20 |
| Motor Server | 3 | 20 |
| Map Server + Comms Bridge | 3 | 30 |
| **Total** | **~37** | **~290** |

### 5.6 Complete ROS 2 Topic Registry

| Topic | Message Type | Publisher(s) | Subscriber(s) | Rate |
|---|---|---|---|---|
| `/rtk/fix` | `NavSatFix` | `rtk_reader` | `localization_node`, `map_server` | ~5 Hz |
| `/imu/data` | `Imu` | `imu_node` | `localization_node` | ~20 Hz |
| `/encoders/ticks` | `Int32MultiArray` | `encoder_node` | `autonomy_node`, `localization_node` | 50 Hz |
| `/encoders/delta` | `Int32MultiArray` | `encoder_node` | — | 50 Hz |
| `/mower/pose` | `Pose2D` | `localization_node` | `autonomy_node`, `map_server` | ~50 Hz |
| `/obstacles/detections` | `String` (JSON) | `obstacle_detector_node` | `autonomy_node` | ~10 Hz |
| `/battery` | `BatteryState` | `battery_monitor` | `ws_motor_server` | 1 Hz |
| `/battery/voltage` | `Float32` | `battery_monitor` | — | 1 Hz |
| `/autonomy/start` | `Empty` | `map_server` | `autonomy_node` | Event |
| `/autonomy/stop` | `Empty` | `map_server` | `autonomy_node` | Event |
| `/autonomy/running` | `Bool` | `autonomy_node` | — | Event |
| `/autonomy/debug` | `String` | `autonomy_node` | — | 4 Hz |
| `/app/heartbeat` | `Empty` | `bridge_node` | — | Event |
| `/app/mode` | `String` | `bridge_node` | — | Event |
| `/app/teleop_cmd` | `Twist` | `bridge_node` | — | Event |
| `/app/connected` | `Bool` | `bridge_node` | — | Event |
| `/mower/state` | `String` | — | `bridge_node` | — |
| `/mower/telemetry` | `String` | — | `bridge_node` | — |
| `/diagnostics` | `DiagnosticArray` | `imu_node` | — | 1 Hz |

### 5.7 Data Flow Layers

The system data flows through three logical layers:

1. **Sensor Layer** — `rtk_reader`, `imu_reader`, `encoder_node`, `lawn_mower_battery` publish raw sensor data
2. **Fusion Layer** — `localization_node` consumes sensor topics and produces the fused pose estimate (`/mower/pose`)
3. **Decision Layer** — `autonomy_node`, `obstacle_detector_node`, and `mower_mapping` consume the fused state and produce motor commands or map data
