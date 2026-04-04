# Missing Paragraphs & PlantUML Diagrams

---

## Part 1: Missing Paragraphs (with placement)

---

### 1. GPS Jump Rejection → Add to Section 5.9 (Localization), after the GPS correction paragraph

> **Place after the paragraph describing GPS complementary filter correction (Section 5.9.4).**

**GPS Jump Rejection.** To guard against erroneous GPS updates caused by multi-path reflection or momentary signal loss, the localization node implements a jump rejection filter. When a new GPS fix arrives, the Euclidean distance between the new position and the previous accepted GPS position is computed. If this distance exceeds the configurable `max_gps_jump_m` threshold (default: 1.5 m), the fix is discarded and a warning is logged. This prevents a single bad satellite solution from teleporting the mower's estimated position across the lawn, which would otherwise trigger a false geofence violation and an unnecessary recovery maneuver.

---

### 2. Map Origin Loading → Add to Section 5.9 (Localization), as a new paragraph before the GPS correction paragraph

> **Place as a new paragraph at the beginning of Section 5.9.4 (GPS Correction), before describing how GPS corrections work.**

**Map Origin Initialization.** On startup, the localization node attempts to load the most recently saved map file from `~/mower_ws/maps/` using the `load_latest_map()` utility. If a valid map is found, its stored origin latitude and longitude are adopted as the local coordinate frame origin [(lat₀, lon₀)](file:///c:/Users/samyb/Downloads/lawnbot_backup/mower_ws/src/obstacle_detection/obstacle_avoidance_pi.py#255-390). This ensures that the encoder odometry frame is aligned with the coordinate system used during boundary recording, eliminating the need to wait for a live GPS fix before the mower can localize within its saved map. If no valid map is found, the node falls back to setting the origin from the first RTK-quality GPS fix received at runtime.

---

### 3. Navigation Commands via Map Server → Add to Section 5.11.5

> **Add as a new paragraph at the end of Section 5.11.5 (State Machine and Storage), after the WebSocket commands paragraph.**

**Autonomy Control Relay.** In addition to the six mapping commands, the map server WebSocket interface exposes two navigation control commands: `start_navigation` and `stop_navigation`. When received from the mobile application, these commands publish `std_msgs/Empty` messages on the `/autonomy/start` and `/autonomy/stop` topics, respectively. This design allows the mobile application to trigger autonomous mowing through the same WebSocket connection used for map management, without requiring a direct connection to the autonomy node or knowledge of ROS 2 topic conventions.

---

### 4. Bridge `/app/connected` Topic → Add to Section 5.13.2

> **Add as an additional row to Table 5.10 (Inbound WebSocket Messages), and a brief sentence after the table.**

Add this row to **Table 5.10**:

| Message Type | ROS 2 Action | Published Topic |
|---|---|---|
| *(connection event)* | Publish `Bool` (true on connect, false on disconnect) | `/app/connected` |

And add this sentence after the table:

Additionally, the bridge publishes a `Bool` message on `/app/connected` whenever a client connects (`true`) or disconnects (`false`). This allows other ROS 2 nodes to monitor whether the mobile application is actively linked to the mower.

---

### 5. Fake Telemetry Node → Add to Section 5.13 (Communication Bridge)

> **Add as a brief note at the end of Section 5.13.3 (Threading Model) or as a short new subsection 5.13.4.**

**5.13.4 Testing Utility.** The `lawnbot_comms` package includes a `fake_telemetry_node` that publishes synthetic `/mower/state` and `/mower/telemetry` messages. This node was used during development to validate the WebSocket bridge and mobile application without requiring the full mower hardware stack to be running. It is not included in the production launch file.

---

### 6. Blade Speed Clamping → Add to Section 5.2.3

> **Add as a note at the end of the "Obstacle Avoidance Integration" paragraph or as a new paragraph after it in Section 5.2.3.**

**Blade Speed Limiting.** The WebSocket server enforces a hard clamp on blade motor speed, restricting it to the range [0.0, 0.1] regardless of the requested value. This safety limit was introduced during development to prevent the blade from reaching full speed during integration testing. The autonomy node commands blade speed 1.0 when starting autonomous mowing, but the motor server overrides this to 0.1 at the protocol level. This intentional mismatch ensures that the blade cannot exceed the tested safe operating speed without a deliberate configuration change on the motor server side.

> [!NOTE]
> Confirm whether 0.1 is the intended production value or a leftover testing safeguard so you can describe it accurately.

---

## Part 2: PlantUML Diagrams

---

### Diagram 1 — System Architecture (3-Layer Block Diagram)

```plantuml
@startuml system_architecture
!theme plain
skinparam backgroundColor #FEFEFE
skinparam componentStyle rectangle
skinparam defaultFontSize 11
skinparam packageBorderColor #555555

title System Architecture — Three-Layer Overview

package "Sensor Layer" #E8F5E9 {
  [rtk_reader\n/rtk/fix] as RTK
  [imu_node\n/imu/data] as IMU
  [encoder_node\n/encoders/ticks] as ENC
  [battery_monitor\n/battery] as BAT
  [OAK-D Lite\n(ai_camera_vision)] as CAM
}

package "Fusion Layer" #E3F2FD {
  [localization_node\n/mower/pose] as LOC
}

package "Decision Layer" #FFF3E0 {
  [autonomy_node\nState Machine] as AUTO
  [obstacle_avoidance_pi\n(standalone)] as OBS
  [map_server\nWS :8770] as MAP
}

package "Actuation" #FFEBEE {
  [ws_motor_server\nWS :8766] as MOT
}

package "External" #F3E5F5 {
  [Mobile App\n(Flutter)] as APP
  [bridge_node\nWS :9002] as BRIDGE
}

' Sensor -> Fusion
RTK --> LOC : NavSatFix
ENC --> LOC : Int32MultiArray

' Sensor -> Decision
RTK --> MAP : NavSatFix
CAM --> OBS : DepthAI queues

' Fusion -> Decision
LOC --> AUTO : Pose2D

' Decision -> Actuation
AUTO --> MOT : WebSocket :8766\n{"cmd":"drive"}
OBS --> MOT : WebSocket :8766\n{"cmd":"drive"}

' Actuation feedback
BAT --> MOT : /battery (ROS 2)

' External
APP <--> BRIDGE : WebSocket :9002
APP <--> MAP : WebSocket :8770
APP <--> MOT : WebSocket :8766

' Decision control
MAP --> AUTO : /autonomy/start\n/autonomy/stop

' Encoder to autonomy
ENC --> AUTO : Int32MultiArray

@enduml
```

---

### Diagram 2 — ROS 2 Topic Graph

```plantuml
@startuml ros2_topic_graph
!theme plain
skinparam backgroundColor #FEFEFE
skinparam defaultFontSize 10
skinparam nodesep 60
skinparam ranksep 50

title ROS 2 Topic Graph — Nodes and Topics

' Nodes
rectangle "rtk_reader" as RTK #C8E6C9
rectangle "imu_node" as IMU #C8E6C9
rectangle "encoder_node" as ENC #C8E6C9
rectangle "battery_monitor" as BAT #C8E6C9
rectangle "localization_node" as LOC #BBDEFB
rectangle "autonomy_node" as AUTO #FFE0B2
rectangle "map_server" as MAP #FFE0B2
rectangle "bridge_node" as BRIDGE #E1BEE7
rectangle "ws_motor_server\n(standalone)" as MOT #FFCDD2

' Topics as edges
RTK -[#2E7D32]-> LOC : <size:9>/rtk/fix\n<size:8>NavSatFix</size>
RTK -[#2E7D32]-> MAP : <size:9>/rtk/fix\n<size:8>NavSatFix</size>

IMU -[#2E7D32]-> LOC : <size:9>/imu/data\n<size:8>Imu</size>

ENC -[#2E7D32]-> LOC : <size:9>/encoders/ticks\n<size:8>Int32MultiArray</size>
ENC -[#2E7D32]-> AUTO : <size:9>/encoders/ticks\n<size:8>Int32MultiArray</size>

LOC -[#1565C0]-> AUTO : <size:9>/mower/pose\n<size:8>Pose2D</size>

BAT -[#2E7D32]-> MOT : <size:9>/battery\n<size:8>BatteryState</size>

MAP -[#E65100]-> AUTO : <size:9>/autonomy/start\n<size:8>Empty</size>
MAP -[#E65100]-> AUTO : <size:9>/autonomy/stop\n<size:8>Empty</size>

BRIDGE -[#6A1B9A]-> MAP : <size:9>/app/heartbeat\n<size:8>Empty</size>
BRIDGE -[#6A1B9A]-> MAP : <size:9>/app/mode\n<size:8>String</size>
BRIDGE -[#6A1B9A]-> MAP : <size:9>/app/teleop_cmd\n<size:8>Twist</size>

AUTO -[#E65100]-> BRIDGE : <size:9>/autonomy/running\n<size:8>Bool</size>

note bottom of IMU
  Although /imu/data is published,
  localization_node does NOT
  currently subscribe to it.
  Heading is encoder-only.
end note

legend right
  |= Color |= Layer |
  | <#C8E6C9> | Sensor |
  | <#BBDEFB> | Fusion |
  | <#FFE0B2> | Decision |
  | <#E1BEE7> | External |
  | <#FFCDD2> | Actuation |
endlegend

@enduml
```

---

### Diagram 3 — Autonomy State Machine

```plantuml
@startuml autonomy_state_machine
!theme plain
skinparam backgroundColor #FEFEFE
skinparam stateBackgroundColor #E3F2FD
skinparam stateBorderColor #1565C0
skinparam defaultFontSize 11

title Autonomy State Machine

[*] --> IDLE

state IDLE : Waiting for start command
state FOLLOW : Following waypoints\nalong current stripe
state TURN : Executing encoder-based\n180° pivot between stripes
state RECOVER : Driving back inside\nboundary polygon
state DONE : All stripes completed\nBlade off, motors stopped
state STOPPED #FFCDD2 : Fatal safety stop\n(pose/encoder timeout)

IDLE --> FOLLOW : /autonomy/start\n[map loaded, WS connected,\nstripes generated]

FOLLOW --> TURN : Last waypoint\nin stripe reached
FOLLOW --> RECOVER : Geofence violation\n(outside boundary)
FOLLOW --> DONE : All stripes complete\n(last stripe, last WP)
FOLLOW --> STOPPED : Pose or encoder\ntimeout (0.5 s)

TURN --> FOLLOW : Both encoder deltas\n≥ turn_ticks_90 (8674 ±150)

RECOVER --> FOLLOW : Re-entered\nboundary polygon

DONE --> IDLE : Autonomy ends\n[blade(0), stop()]

STOPPED --> [*] : Terminal state\n[hard stop + blade off]

note right of FOLLOW
  20 Hz control loop
  Proportional steering
  Pivot if |error| > 0.60 rad
end note

note left of RECOVER
  Drives toward centroid
  push_in = 0.40 m
end note

@enduml
```

---

### Diagram 4 — Motor Control Data Flow

```plantuml
@startuml motor_control_dataflow
!theme plain
skinparam backgroundColor #FEFEFE
skinparam defaultFontSize 11

title Motor Control — Drive Command Pipeline

start

:Receive WebSocket JSON\n{"cmd":"drive", "left":L, "right":R};

:Parse left/right as float\n(safe_float: default 0.0);

:Apply **deadzone** filter\n|value| < 0.08 → 0.0;

:Apply **clamping**\nclamp to [-MAX_WHEEL, +MAX_WHEEL]\n(default MAX_WHEEL = 1.0);

:Compute **dt** since last update;

:Apply **slew-rate limiter**\nramp_towards(current, target, dt, 1.5/s)\nMax Δ = 1.5 × dt per step;

:Set motor speeds\nleft_motor.set_speed(current_left)\nright_motor.set_speed(current_right);

:Send **ACK** JSON\n{"ok":true, "cmd":"drive"};

stop

note right
  **Concurrently:**
  Watchdog polls every 50 ms.
  If no command received within
  0.4 s → stop_all(motors).
end note

@enduml
```

---

### Diagram 5 — Localization Fusion

```plantuml
@startuml localization_fusion
!theme plain
skinparam backgroundColor #FEFEFE
skinparam defaultFontSize 11

title Localization — Encoder + GPS Complementary Filter

rectangle "encoder_node" as ENC #C8E6C9
rectangle "rtk_reader" as RTK #C8E6C9

rectangle "Localization Node" as LOC #BBDEFB {

  rectangle "Encoder Odometry\n(Differential Drive)" as ODOM {
    card "ΔL = dL_ticks × meters_per_tick" as DL
    card "ΔR = dR_ticks × meters_per_tick" as DR
    card "Δθ = (ΔL − ΔR) / track_width" as DTHETA
    card "Δs = (ΔL + ΔR) / 2" as DS
    card "x += Δs·cos(θ)\ny += Δs·sin(θ)" as INTEGRATE
  }

  rectangle "GPS Complementary Filter" as GPSF {
    card "Convert lat/lon → local (x,y)\nEquirectangular projection" as CONV
    card "Jump rejection\n|Δpos| > 1.5 m → discard" as JUMP
    card "x_odom += k·(x_gps − x_odom)\ny_odom += k·(y_gps − y_odom)\nk = 0.20" as BLEND
  }
}

rectangle "/mower/pose\n(Pose2D: x, y, θ)" as POSE #FFE0B2

ENC --> DL : /encoders/ticks\n[left, right]
DL --> DR
DR --> DTHETA
DTHETA --> DS
DS --> INTEGRATE

RTK --> CONV : /rtk/fix\nNavSatFix
CONV --> JUMP
JUMP --> BLEND

INTEGRATE --> POSE
BLEND --> POSE

note bottom of GPSF
  **θ (heading) is encoder-only.**
  IMU (/imu/data) is NOT used
  by the localization node.
end note

note right of BLEND
  GPS gain k = 0.20
  (smooth correction,
  not hard overwrite)
end note

@enduml
```

---

### Diagram 6 — Coverage Planner

```plantuml
@startuml coverage_planner
!theme plain
skinparam backgroundColor #FEFEFE
skinparam defaultFontSize 11

title Coverage Planner — Boustrophedon Path Generation

rectangle "Input: Boundary Polygon\n(from saved map JSON)" as INPUT #C8E6C9

rectangle "Step 1: Buffer Inward\nbuffer(-0.15 m)\nKeep mower inside edges" as STEP1 #E3F2FD

rectangle "Step 2: Generate Vertical Scan Lines\nspacing = 0.18 m (blade width)\nacross polygon bounding box" as STEP2 #E3F2FD

rectangle "Step 3: Intersect with Polygon\nShapely intersection()\n→ line segments inside boundary" as STEP3 #E3F2FD

rectangle "Step 4: Sample Waypoints\nwaypoint_spacing = 0.35 m\nalong each segment" as STEP4 #E3F2FD

rectangle "Step 5: Reverse Alternates\nOdd stripes → reverse direction\n= boustrophedon pattern" as STEP5 #E3F2FD

rectangle "Output: List[List[XY]]\nOrdered stripes of waypoints" as OUTPUT #FFE0B2

INPUT --> STEP1
STEP1 --> STEP2
STEP2 --> STEP3
STEP3 --> STEP4
STEP4 --> STEP5
STEP5 --> OUTPUT

note right of STEP5
  ┌─────────────┐
  │ → → → → → → │  stripe 0
  │ ← ← ← ← ← ← │  stripe 1
  │ → → → → → → │  stripe 2
  │ ← ← ← ← ← ← │  stripe 3
  └─────────────┘
  (top-down view)
end note

@enduml
```

---

### Diagram 7 — Obstacle Avoidance Zones

```plantuml
@startuml obstacle_zones
!theme plain
skinparam backgroundColor #FEFEFE
skinparam defaultFontSize 11

title Obstacle Avoidance — Distance Zones (Top-Down View)

rectangle "  CLEAR ZONE (> 0.75 m)\n  Action: Continue forward  " as CLEAR #C8E6C9 {

  rectangle "  SLOW ZONE (0.50–0.75 m)\n  Action: Reduce speed  " as SLOW #FFF9C4 {

    rectangle "  WARNING ZONE (0.35–0.50 m)\n  Action: Arc turn away  " as WARN #FFE0B2 {

      rectangle "  CRITICAL ZONE (0.30–0.35 m)\n  Action: Pivot away  " as CRIT #FFCCBC {

        rectangle "  EMERGENCY (< 0.30 m)\n  Action: FULL STOP\n  (pivot escape after 2 s)  " as EMERG #FFCDD2
      }
    }
  }
}

note right of CLEAR
  Escape direction chosen by
  **_best_escape_dir()** which
  scores clearance on each side
  using inverse-square weighting.

  **Stuck timeout:** 5 s in
  non-forward state → 180° pivot
end note

note bottom of EMERG
  Robot half-width = 0.45 m
  (22.5 cm body + 22.5 cm margin)
  Obstacle is "in path" if lateral
  offset < ROBOT_HALF_WIDTH
end note

@enduml
```

---

### Diagram 8 — Hardware I2C/GPIO Wiring

```plantuml
@startuml hardware_wiring
!theme plain
skinparam backgroundColor #FEFEFE
skinparam defaultFontSize 10
skinparam rectangleBorderColor #555555

title Hardware — GPIO and I2C Connections

rectangle "Raspberry Pi 5" as PI #E3F2FD {

  rectangle "GPIO (PWM/Digital)" as GPIO {
    card "Right Motor (BTS7960)\nR_EN=23, L_EN=24\nR_PWM=12, L_PWM=19" as MR
    card "Left Motor (BTS7960)\nR_EN=5, L_EN=6\nR_PWM=13, L_PWM=18" as ML
    card "Blade Motor (BTS7960)\nR_EN=20, L_EN=16\nR_PWM=26, L_PWM=21" as MB
    card "Left Encoder\nCh_A=17, Ch_B=27" as EL
    card "Right Encoder\nCh_A=22, Ch_B=4" as ER
  }

  rectangle "I2C Bus 1 (SDA=GPIO2, SCL=GPIO3)" as I2C {
    card "BNO085 IMU\naddr: 0x4A" as BNO
    card "ADS1015 ADC\naddr: 0x49\n(battery ch A3)" as ADC
    card "MCP23017 GPIO Expander\naddr: 0x27\n(5 battery LEDs)" as MCP
  }

  rectangle "Serial (USB)" as SERIAL {
    card "u-blox ZED-F9P\n115200 baud\n(NMEA out + RTCM in)" as GPS
  }

  rectangle "USB 3.0" as USB {
    card "OAK-D Lite\n(DepthAI pipeline)" as OAK
  }
}

@enduml
```

---

### Diagram 9 — WebSocket Protocol Sequence

```plantuml
@startuml websocket_protocol
!theme plain
skinparam backgroundColor #FEFEFE
skinparam defaultFontSize 10
skinparam sequenceMessageAlign center

title WebSocket Protocol — Phone ↔ Bridge ↔ Motor Server

actor "Mobile App\n(Flutter)" as APP
participant "bridge_node\n:9002" as BRIDGE
participant "ROS 2 Graph" as ROS
participant "ws_motor_server\n:8766" as MOTOR

== Connection ==
APP -> BRIDGE : WebSocket connect
BRIDGE -> APP : {"type":"ack","version":1}
BRIDGE -> ROS : pub /app/connected (Bool=true)

== Teleoperation ==
APP -> BRIDGE : {"type":"heartbeat"}
BRIDGE -> ROS : pub /app/heartbeat (Empty)

APP -> BRIDGE : {"type":"teleop","linear":0.5,"angular":0.1}
BRIDGE -> ROS : pub /app/teleop_cmd (Twist)

== Direct Motor Control ==
APP -> MOTOR : {"cmd":"drive","left":0.4,"right":0.4}
MOTOR -> APP : {"ok":true,"cmd":"drive"}

APP -> MOTOR : {"cmd":"get_battery"}
MOTOR -> APP : {"ok":true,"cmd":"battery",\n"percentage":85,"voltage":12.8}

== Telemetry (Mower → Phone) ==
ROS -> BRIDGE : /mower/state (String)
BRIDGE -> APP : {"type":"state","data":"..."}

ROS -> BRIDGE : /mower/telemetry (String)
BRIDGE -> APP : {"type":"telemetry","data":"..."}

... 200 ms idle ...
BRIDGE -> APP : {"type":"comms","heartbeat_age_ms":1500}

== Disconnect ==
APP -> BRIDGE : WebSocket close
BRIDGE -> ROS : pub /app/connected (Bool=false)

note right of BRIDGE
  Single-client only.
  Second client gets
  {"type":"error"} and
  connection is closed.
end note

@enduml
```

---

### Diagram 10 — DepthAI Pipeline

```plantuml
@startuml depthai_pipeline
!theme plain
skinparam backgroundColor #FEFEFE
skinparam defaultFontSize 11

title DepthAI Pipeline — OAK-D Lite Processing Streams

rectangle "OAK-D Lite Camera" as CAM #C8E6C9 {
  rectangle "Color Camera\n640×360 @ 30fps" as RGB
  rectangle "Left Mono\n(stereo pair)" as LMONO
  rectangle "Right Mono\n(stereo pair)" as RMONO
}

rectangle "Myriad X VPU (On-Device)" as VPU #BBDEFB {
  rectangle "YOLOv3-tiny\n6 shaves\n640×352 input" as YOLO
  rectangle "DeepLabV3+\n(headless mode only)" as SEG
  rectangle "Stereo Depth\nNode" as DEPTH
}

rectangle "Host Processing (Raspberry Pi)" as HOST #FFE0B2 {
  rectangle "parse_img_detections()\n+ adjust_bbox_from_letterbox()" as PARSE
  rectangle "sample_depth_robust()\nMedian of 5 samples\n→ distance (mm)" as SAMPLE
  rectangle "build_obstacles()\n→ TrackedObstacle[]\n(dist, angle, width, in_path)" as BUILD
  rectangle "decide_action()\nDistance-layered avoidance" as DECIDE
  rectangle "Zone Calculator\n(segmentation mask → ratios)" as ZONE
}

rectangle "MotorCommander\n→ WS :8766" as MOTOR #FFCDD2

RGB --> YOLO : preview frames\n(letterboxed to 640×352)
RGB --> SEG : preview frames
LMONO --> DEPTH
RMONO --> DEPTH

YOLO --> PARSE : detection queue\n(raw tensors)
DEPTH --> SAMPLE : depth queue\n(depth frame)
SEG --> ZONE : segmentation queue\n(class-id mask)

PARSE --> BUILD
SAMPLE --> BUILD
BUILD --> DECIDE
DECIDE --> MOTOR : execute(action)

note bottom of YOLO
  COCO 80 classes
  Confidence threshold: 0.30
  HFOV: 73° for bearing calc
end note

note right of HOST
  **Headless mode:** No OpenCV GUI
  **Visualization mode:** Overlays
  bounding boxes + depth on preview
end note

@enduml
```
