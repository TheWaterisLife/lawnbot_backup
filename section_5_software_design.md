COEN/ELEC 490 - Phase 4 Report
Automatic Lawnmower




March 28th, 2026







Team Number: 25
Team Members
 Mustafa Aboabdullah - 40199998
 Laith Qasem - 40200060
 Nicolas Gharzouzi - 40232064
 Iliass Bouhsane - 40263483
 Chadi El Tannir - 40211031
 Samy Belmihoub - 40251504
Abstract
1. Introduction
1.1 Project Overview
1.2 Objectives
1.3 Project Scope
1.4 Report Outline
2. System Requirements and Specifications
2.1 Functional Requirements
2.2 Non-Functional Requirements
2.3 Final Product Specifications
3. System Architecture
3.1 High-Level System Block Diagram
3.2 Hardware Architecture Overview
3.3 Software Architecture Overview
3.4 Communication Between Subsystems
4. Hardware Design
4.1 Mechanical Chassis and Frame
4.2 PCB Design
4.2.1 Schematic Design
4.2.2 PCB Layout
4.2.3 Component Selection and Bill of Materials
4.3 Power System and Battery
4.3.1 Battery Selection and Specifications
4.3.2 Battery Monitoring System (MCP23017 & LED Indicators)
4.3.3 Power Distribution
4.4 Motor Driver and Motor Selection
4.5 Sensor Hardware
4.5.1 OAK-D Lite Camera
4.5.2 BNO085 IMU
4.5.3 RTK-GPS Module
4.5.4 Wheel Encoders
4.6 Raspberry Pi and Peripheral Connections
4.7 Blade Assembly
5. Software Design
5.1 ROS 2 Workspace Structure
The software for the autonomous lawn mower is developed within a ROS 2 Jazzy workspace running on Ubuntu 24.04 (Noble) with Python 3.12. The workspace is organized into 8 actively deployed packages, each responsible for a distinct subsystem.
Table 5.1 — Workspace Package Summary
Package Name
Type
Primary Function
lawnbot_motors
Standalone
BTS7960 motor drivers via WebSocket server
imu_reader
ROS 2 node
IMU sensor driver (BNO085 / MPU6050)
rtk_reader
ROS 2 node
RTK-GPS positioning via u-blox ZED-F9P
lawn_mower_battery
ROS 2 node
Battery voltage and charge monitoring
ai_camera_vision
Library
OAK-D Lite dual-model vision pipeline
obstacle_detection
Standalone
Real-time obstacle avoidance controller
mower_mapping
ROS 2 node
GPS/local boundary recording and map storage
mower_autonomy
ROS 2 node
Encoder driver, IMU-fused localization, obstacle detection, and autonomous mowing state machine
lawnbot_comms
ROS 2 node
WebSocket bridge for mobile app control
mower_bringup
Launch
Main mower_stack.launch.py file orchestrating the system startup sequence.

The workspace uses ament_python as its build system. All custom packages are pure Python, with the exception of lawnbot_motors and obstacle_detection, which run as standalone asyncio processes outside the ROS 2 graph. Three legacy packages (sensor_integration, mower_navigation, wheel_encoder) remain in the workspace but are not launched or used — their functionality was superseded by the mower_autonomy package, which consolidates encoder driving, IMU-fused localization, obstacle detection publishing, and the autonomous mowing state machine into a single integrated package.
5.2 Motor Control Subsystem (lawnbot_motors)
5.2.1 Architecture Overview
The motor control subsystem manages three BTS7960 dual-H-bridge motor drivers: left track, right track, and blade. Unlike typical ROS 2 designs where motor commands arrive via a /cmd_vel subscription, this subsystem operates as a standalone WebSocket server on port 8766. This non-standard architecture was chosen because the motor server must also serve the mobile application for manual teleoperation, making WebSocket a more appropriate transport than ROS 2 topics for bidirectional, low-latency command and response.
5.2.2 Hardware Abstraction (motor_hw.py)
Motor control is abstracted through the BTS7960Motor class, which wraps the gpiozero library's PWMOutputDevice and DigitalOutputDevice primitives. Each BTS7960 driver requires four GPIO pins: two enable lines (R_EN, L_EN) and two PWM lines (R_PWM, L_PWM). The class accepts a normalized speed input in the range [-1.0, +1.0], where positive values drive R_PWM (forward) and negative values drive L_PWM (reverse). A MotorPins dataclass encapsulates the pin assignments, which are loaded from a YAML configuration file.
Table 5.2 — GPIO Pin Assignments for Motor Drivers
Motor
R_EN
L_EN
R_PWM
L_PWM
Right track
GPIO 23
GPIO 24
GPIO 12
GPIO 19
Left track
GPIO 5
GPIO 6
GPIO 13
GPIO 18
Blade
GPIO 20
GPIO 16
GPIO 26
GPIO 21

PWM frequency is configurable (default: 1000 Hz) and maximum duty cycle can be clamped per motor. An optional invert flag reverses the direction mapping.
5.2.3 WebSocket Server (ws_motor_server.py)
The server is implemented using the websockets library with Python's asyncio event loop. Upon client connection, the server sends a capabilities message listing all available motors and supported commands: on, stop, set, drive, blade, get_battery, start_obstacle_avoidance, and stop_obstacle_avoidance.
Drive Command Processing. The drive command implements joystick-style differential drive control. Incoming left/right speed values pass through three processing stages:
Deadzone filter — values below a configurable threshold (default: 0.08) are zeroed to prevent motor whine at low inputs.
Clamping — values are clamped to [-MAX_WHEEL, +MAX_WHEEL] (default: 1.0).
Slew-rate limiting — a ramp_towards() function limits speed change to 1.5 units per second, preventing mechanical shock.
Safety Watchdog. A watchdog timer runs as a concurrent asyncio task, polling every 50 ms. If no command is received within the configurable timeout (default: 0.4 seconds), all motors are stopped. This protects against network disconnection during teleoperation.
Battery Bridge. The server also hosts a BatterySubscriber ROS 2 node that subscribes to /battery (a sensor_msgs/BatteryState message) and exposes the latest voltage and percentage via the get_battery command. This hybrid approach — a standalone WebSocket server embedding a ROS 2 subscriber — allows the mobile application to query battery state without directly subscribing to ROS 2 topics.
Obstacle Avoidance Integration. The start_obstacle_avoidance command spawns the obstacle avoidance script (obstacle_avoidance_pi.py) as a child subprocess. The stop_obstacle_avoidance command terminates it, with SIGTERM followed by SIGKILL if the process does not exit within five seconds.
5.3 IMU Reader Node (imu_reader)
5.3.1 Architecture
The IMU reader package implements a driver-abstracted architecture using the Factory pattern. An IMUFactory class selects the appropriate sensor driver at runtime based on the imu_type parameter, supporting both the Adafruit BNO085 (9-DOF IMU with on-chip sensor fusion) and the MPU-6050/MPU-6500 (6-DOF accelerometer/gyroscope). This decoupling isolates the ROS 2 node from sensor-specific initialization, allowing hardware swaps without modifying the node logic.
5.3.2 Sensor Drivers
BNO085 Driver. Communicates over I2C at address 0x4A using the adafruit-circuitpython-bno08x library (v1.3.1). The sensor is configured in ROTATION_VECTOR mode, which provides a pre-fused quaternion orientation at approximately 100 Hz. The driver extracts angular velocity (gyro_x, gyro_y, gyro_z) and linear acceleration from the sensor's built-in ARVR stabilized output.
MPU-6050 Driver. Communicates over I2C at address 0x68 using the smbus2 library for direct register access. The driver reads raw 16-bit values from the accelerometer and gyroscope registers and applies configurable sensitivity scaling. The WHO_AM_I register check accepts values 0x68, 0x70, 0x71, and 0x73 to accommodate the MPU-6500 variant.
5.3.3 ROS 2 Interface
The node publishes sensor_msgs/Imu messages on /imu/data at a configurable rate (default: 17–20 Hz). Diagnostics are published on /diagnostics at 1 Hz using diagnostic_msgs/DiagnosticArray.
Table 5.3 — IMU Node Published Topics
Topic
Message Type
Rate
Content
/imu/data
sensor_msgs/Imu
~20 Hz
Angular velocity, linear acceleration, quaternion orientation
/diagnostics
diagnostic_msgs/DiagnosticArray
1 Hz
Sensor health, read count, error rate

5.4 RTK-GPS Reader Node (rtk_reader)
5.4.1 Architecture
The RTK reader interfaces with a u-blox ZED-F9P GNSS receiver (SimpleRTK2B board) over a serial connection at 115200 baud. The implementation uses the pyserial library for serial I/O and Python's socket module for NTRIP communication. Two concurrent threads manage data flow:
Read thread (ROS 2 timer at 50 Hz) — reads NMEA sentences from the serial buffer, parses GGA messages, and publishes sensor_msgs/NavSatFix on /rtk/fix.
NTRIP thread (background daemon) — connects to an NTRIP caster (IP: 3.143.243.81, port: 2101, mountpoint: RD1_STATION_SO), streams RTCM correction data, and writes it to the same serial port to enable RTK positioning.
5.4.2 NMEA Parsing
The parser is implemented without external NMEA libraries; it directly splits and validates GGA sentences. The parse_gga() function extracts latitude, longitude, altitude, and fix quality from comma-separated fields. The dm_to_deg() function converts NMEA's degrees-minutes format to decimal degrees. A checksum validation function (nmea_checksum_ok()) verifies data integrity using the XOR-based NMEA checksum.
Fix quality values are mapped to NavSatStatus as follows:
Table 5.4 — GGA Fix Quality Mapping
GGA Quality
Description
NavSatStatus
0
No fix
STATUS_NO_FIX
1
GNSS (autonomous)
STATUS_FIX
2
DGPS
STATUS_GBAS_FIX
4
RTK Fixed
STATUS_GBAS_FIX
5
RTK Float
STATUS_GBAS_FIX

5.4.3 NTRIP Client
The NTRIP client connects via raw TCP socket and sends an HTTP-style GET request with Basic authentication (Base64-encoded credentials). Upon receiving a 200 OK response, the thread enters a streaming loop that reads RTCM3 correction data (up to 4096 bytes per read) and writes it directly to the serial port. If the connection drops, the thread automatically reconnects after a two-second backoff.
5.5 Encoder Node (mower_autonomy/encoder_node)
5.5.1 Architecture
The encoder node is part of the mower_autonomy package (not a separate package). It reads two FIT0403 hall-effect quadrature encoders via gpiozero/lgpio interrupt callbacks on four GPIO pins. Each encoder produces two channels (A and B); the phase relationship determines rotation direction.
Table 5.5 — Encoder GPIO Assignments
Encoder
Channel A
Channel B
Left
GPIO 17
GPIO 27
Right
GPIO 22
GPIO 4

5.5.2 Tick Publishing
The encoder node runs at 50 Hz and publishes raw tick counts on /encoders/ticks (Int32MultiArray, two elements: [left_ticks, right_ticks]) and delta ticks on /encoders/delta. Odometry integration (converting ticks to pose) is performed by the localization_node, not the encoder node itself. This separation allows the localization node to fuse encoder ticks with IMU heading data.
5.5.3 Calibration
The encoder resolution is calibrated to 3232 counts per revolution with a wheel radius of 0.05 m and track width of 0.24 m. An invert_left/invert_right parameter allows reversing direction for wiring variations.
5.6 Battery Monitor Node (lawn_mower_battery)
5.6.1 Architecture
The battery monitor reads the 12.8 V LiFePO₄ battery voltage through a resistive voltage divider (R1 = 150 kΩ, R2 = 22 kΩ) connected to an ADS1015 12-bit ADC (I2C address 0x49, channel A3). The ADC is read using the adafruit-circuitpython-ads1x15 library. The measured ADC voltage is scaled through the divider formula:
V_battery = V_adc × (R1 + R2) / R2
5.6.2 Charge Estimation
State of charge is estimated using a piecewise-linear interpolation of the LiFePO₄ discharge curve, defined as a lookup table ranging from 13.0 V (100%) to 12.0 V (0%). Battery health is assessed based on voltage thresholds: below 12.0 V is reported as POWER_SUPPLY_HEALTH_DEAD, above 15.0 V as POWER_SUPPLY_HEALTH_OVERVOLTAGE.
5.6.3 LED Indicator
An optional MCP23017 GPIO expander (I2C address 0x27) drives five battery-bar LEDs on port B (GPB0–GPB4). The LEDs use active-low logic: a bit cleared to 0 turns the corresponding LED on. The smbus2 library is used for direct register access to set the output latch register (OLATB, address 0x15). The MCP23017 initializes independently of the ADC — if not detected, LEDs are silently disabled.
Table 5.6 — Battery LED Thresholds
LED (GPB Pin)
Color
Threshold
GPB0
Green
≥ 80%
GPB1
Green
≥ 60%
GPB2
Amber
≥ 40%
GPB3
Amber
≥ 20%
GPB4
Red
≥ 0%

5.6.4 ROS 2 Interface
The node publishes at 1 Hz on two topics: /battery (sensor_msgs/BatteryState, including voltage, percentage, health, and technology as POWER_SUPPLY_TECHNOLOGY_LIFE) and /battery/voltage (std_msgs/Float32). The ws_motor_server subscribes to /battery and relays the data to the mobile application.
5.7 AI Camera Vision Node (ai_camera_vision)
5.7.1 Pipeline Architecture
The AI camera vision system is the most complex software package in the project. It uses the OAK-D Lite stereo camera with the Luxonis DepthAI SDK to run dual neural network inference on the integrated Myriad X VPU.
The DepthAI pipeline is constructed on the host and deployed to the device. It consists of three processing streams:
YOLO Object Detection — A YOLOv3-tiny model compiled to OpenVINO .blob format (6 shaves) processes 640×352 input frames for object detection. The model outputs raw detection tensors that are decoded on the host into Detection2D structures containing normalized bounding box coordinates, class labels (COCO 80 classes), and confidence scores.
Semantic Segmentation — A DeepLabV3+ model processes the same camera frames to produce a per-pixel class-id mask. On the host, a zone calculator converts the segmentation mask into zone ratios (e.g., grass percentage, obstacle percentage) published as a zone summary.
Stereo Depth — The OAK-D Lite's stereo camera pair produces a depth frame. For each YOLO detection, the host samples depth values within the bounding box region using sample_depth_robust(), which takes the median of multiple samples to reject outliers. Depth in millimeters is combined with the detection's horizontal pixel position and the camera's horizontal field of view (73°) to estimate the 3D bearing angle and distance.
Letterbox Handling. The YOLO model expects 640×352 input while the camera produces 640×360 (16:9). Letterboxing is applied to preserve the full ~81° horizontal field of view, and an adjust_bbox_from_letterbox() function corrects bounding box coordinates to account for the padding offset.
5.7.2 Deployment Modes
Two deployment modes are supported:
Headless mode — Default on the Raspberry Pi. Publishes detections to ROS 2 topics without OpenCV visualization. Designed for low-overhead production operation.
Visualization mode — Used on development machines. Overlays bounding boxes, segmentation masks, and depth data on the preview frame using OpenCV for debugging and validation.
Both modes set header.frame_id = "oak_rgb_optical_frame" on all published messages. TF transforms are managed externally by the robot stack.
5.8 Obstacle Detection and Avoidance (obstacle_detection)
5.8.1 Architecture
The obstacle avoidance module is a standalone Python process (not a ROS 2 node) that combines OAK-D Lite YOLO inference with distance-layered motor control. It is spawned as a subprocess by the motor server (see Section 5.2.3) and communicates motor commands via WebSocket.
5.8.2 Detection Pipeline
The module creates a YOLO-only DepthAI pipeline (without segmentation) and reads detection and depth queues in a non-blocking loop. For each frame, raw detections are parsed, letterbox-corrected, and converted to TrackedObstacle objects containing distance, bearing angle, estimated physical width, lateral offset, and an in_path flag. An obstacle is "in path" if its lateral offset from the mower's centerline is less than the robot's half-width (0.45 m, including a 22.5 cm safety margin per side).
5.8.3 Avoidance Decision Engine
The decide_action() function implements a distance-layered avoidance strategy:
Table 5.7 — Obstacle Avoidance Zones
Zone
Distance Range
Action
Emergency
< 0.30 m
Full stop (pivot escape after 2 s timeout)
Critical
0.35 m
Pivot away from obstacle
Warning
0.50 m
Arc turn away from obstacle
Slow
0.75 m
Reduce forward speed
Clear
> 0.75 m
Continue forward normally

The escape direction is chosen by a scoring algorithm (_best_escape_dir()) that evaluates the clearance on each side using inverse-square distance weighting — nearby obstacles contribute exponentially more penalty. A minimum turn hold timer (1.5 s) prevents oscillation, and a stuck escape timeout (5 s) forces a 180° pivot if the mower cannot make forward progress.
A MotorCommander class wraps the WebSocket client, translating high-level actions (e.g., "pivot_left", "turn_right") into differential drive commands sent to the motor server.
5.9 Localization (mower_autonomy/localization_node)
5.9.1 Architecture
The localization node replaces a traditional EKF pipeline with a lightweight, custom dead-reckoning approach. It fuses three sensor sources — encoder ticks, IMU heading, and RTK-GPS corrections — into a Pose2D estimate (x, y, θ) published on /mower/pose.
5.9.2 Encoder-Based Odometry
The node subscribes to /encoders/ticks and integrates tick deltas into pose using the differential-drive kinematic model. Wheel radius (0.05 m), track width (0.24 m), and encoder CPR (3232) are used to convert ticks to displacement.
5.9.3 IMU Heading Fusion
To combat encoder-only heading drift, the node subscribes to /imu/data and extracts yaw from the BNO085's quaternion output. A complementary filter blends encoder-derived heading with IMU-derived heading:
θ_fused = (1 − k) × θ_encoder + k × θ_imu
where k = 0.30 (configurable via the imu_heading_gain parameter). On the first IMU reading, the node computes an offset to align the IMU frame with the encoder frame, avoiding startup discontinuities. If the IMU goes stale (no data for > 0.5 s), the node falls back to encoder-only heading.
5.9.4 GPS Correction
When an RTK-GPS fix is available on /rtk/fix, the node converts latitude/longitude to local meters using the equirectangular approximation and directly overwrites the x, y position. This provides absolute position correction that compensates for encoder drift.
5.10 Coverage Planning (mower_autonomy/coverage_planner)
5.10.1 Architecture
The coverage planner is integrated into the mower_autonomy package (not a separate mower_navigation package). A legacy mower_navigation package with Nav2 integration exists in the workspace but was never deployed — its functionality was replaced by the custom state machine described in Section 5.12.
5.10.2 Boustrophedon Path Generation
The coverage planner generates a space-filling path by computing vertical scan lines across the work polygon:
The boundary polygon is buffered inward by a configurable margin (default: 0.15 m) using Shapely's buffer(-margin) function to prevent the mower from driving on or beyond the boundary.
Vertical scan lines are generated at stripe_spacing intervals (default: 0.18 m, matching the blade width).
Each scan line is intersected with the buffered polygon using Shapely's intersection() method, producing one or more line segments.
Segments are sampled into waypoints at waypoint_spacing intervals (default: 0.35 m).
Alternate stripes are reversed to create a boustrophedon (back-and-forth) pattern, minimizing turn distance.
5.10.3 Motor Control
The autonomy node communicates motor commands directly via WebSocket to the lawnbot_motors server (port 8766), sending {"cmd":"drive","left":...,"right":...} and {"cmd":"stop"} commands. This bypasses the ROS 2 /cmd_vel topic entirely, providing minimal-latency direct control.
5.11 Mower Mapping (mower_mapping)
5.11.1 Architecture
The mapping subsystem records GPS boundary polygons that define the mowing area. It operates as a ROS 2 node with an embedded WebSocket server (port 8770) that allows the mobile application to control recording and retrieve saved maps.
5.11.2 Coordinate Conversion
GPS coordinates are converted from geodetic (latitude, longitude) to local Cartesian (x, y) meters using the equirectangular approximation:
x = (lon − lon₀) × R × cos(lat₀) × π/180
y = (lat − lat₀) × R × π/180
where R = 6 371 000 m and (lat₀, lon₀) is the first recorded GPS fix, which becomes the local origin.
5.11.3 Adaptive Sampling
Rather than recording every GPS fix, the map server uses an adaptive sampling algorithm that triggers a new point when any of the following conditions are met:
Distance trigger: the mower has moved ≥ 0.25 m since the last recorded point.
Heading trigger: the heading has changed by ≥ 8° (captures corners with higher resolution).
Time trigger: at least 1.0 second has elapsed since the last recording.
A minimum cooldown of 0.2 seconds prevents over-sampling.
This produces dense points on curves and sparse points on straight edges.
5.11.4 Path Simplification (RDP)
After recording stops, the raw boundary points are simplified using the Ramer-Douglas-Peucker (RDP) algorithm with an epsilon of 0.08 m. This typically reduces point count by ~80% while preserving boundary shape within 8 cm accuracy. If the endpoint is more than 0.5 m from the start point, the start point is appended to close the loop.
5.11.5 State Machine and Storage
The map server operates in three states: IDLE, MAPPING, and PAUSED. GPS signal loss for more than 2 seconds automatically transitions from MAPPING to PAUSED; signal recovery within 0.6 seconds triggers automatic resumption. Maps are saved as JSON files in ~/mower_ws/maps/ with a naming convention of <name>_<YYYYMMDD_HHMMSS>.json.
The WebSocket protocol supports six commands: map_start, map_stop, map_save, map_list, map_load, and map_cancel. During recording, the server broadcasts status, RTK pose, and boundary preview at 5 Hz.
5.12 Mower Autonomy (mower_autonomy)
5.12.1 Architecture
The autonomy node is the highest-level control module. It implements a finite state machine that orchestrates map loading, path planning, waypoint following, and safety enforcement. Unlike Nav2-based approaches, this module communicates motor commands directly via WebSocket, bypassing the ROS 2 /cmd_vel topic for minimal latency.
5.12.2 State Machine
The autonomy node operates in six states:
Table 5.9 — Autonomy State Machine
State
Description
Transitions
IDLE
Waiting for start command
→ FOLLOW (on /autonomy/start)
FOLLOW
Following waypoints along a stripe
→ TURN (stripe end), → RECOVER (outside boundary), → DONE (all stripes complete)
TURN
Executing a 180° pivot between stripes
→ FOLLOW (turn complete)
RECOVER
Navigating back inside the boundary
→ FOLLOW (re-entered boundary)
DONE
All stripes completed
→ IDLE
STOPPED
Fatal safety stop
Terminal

5.12.3 Waypoint Following (FOLLOW mode)
At each control tick (20 Hz), the node computes the bearing error between the current pose (/mower/pose, a Pose2D message) and the target waypoint. A proportional controller generates left/right differential drive commands:
If the heading error exceeds 0.60 radians (~34°), a pivot turn is executed (tracks spinning in opposite directions).
Otherwise, a proportional steering command blends forward speed (fwd_cmd = 0.14) with a turn correction (turn_cmd = 0.16 × error).
Within a slow zone (< 0.25 m from the waypoint), forward speed is scaled down to a minimum of 35%.
A waypoint is accepted when the distance drops below 0.25 m. A brief dwell period (0.15 s) at each waypoint improves tracking accuracy.
5.12.4 Turn Execution (TURN mode)
At the end of each stripe, the node captures encoder reference values and executes a pivot turn. Completion is determined by encoder ticks: when both the left and right delta ticks exceed the calibrated 90° threshold (8674 ticks, ±150 tolerance), the turn is accepted. This encoder-based approach is more reliable than timer-based turning on uneven terrain.
5.12.5 Geofence and Boundary Recovery (RECOVER mode)
A Geofence object, built from the boundary polygon with an inward margin (0.15 m), performs point-in-polygon tests at each control tick using Shapely. If the mower drifts outside the boundary, the RECOVER mode computes the nearest boundary point and drives the mower toward the polygon centroid with a configurable push-in distance (0.40 m).
5.12.6 Safety Enforcement
Two safety conditions trigger an immediate hard stop:
Pose timeout — No /mower/pose update within 0.5 seconds.
Encoder timeout — No /encoders/ticks update within 0.5 seconds.
On hard stop, the motor WebSocket client sends both stop and blade(0.0) commands. A failsafe mechanism (_failsafe_stop_sync()) ensures stop commands are sent even during unhandled exceptions or node destruction.
5.13 Communication Bridge (lawnbot_comms)
5.13.1 Architecture
The communication bridge provides bidirectional connectivity between the mobile application and the ROS 2 graph. It operates a WebSocket server on port 9002 using the websockets library, with the asyncio event loop interleaved with rclpy.spin_once() at 50 ms intervals.
5.13.2 Protocol
Phone → Mower (Commands):
Table 5.10 — Inbound WebSocket Messages
Message Type
ROS 2 Action
Published Topic
hello
Reply with ACK and version
—
heartbeat
Publish Empty
/app/heartbeat
mode
Publish String with mode value
/app/mode
teleop
Publish Twist with linear/angular
/app/teleop_cmd

Mower → Phone (Telemetry):
Table 5.11 — Outbound WebSocket Messages
Message Type
Source
Trigger
ack
Bridge node
On connection and hello
state
Subscription to /mower/state
On ROS 2 message
telemetry
Subscription to /mower/telemetry
On ROS 2 message
comms
Internal timer
Every 200 ms idle
error
Bridge node
On rejected second client

The bridge enforces single-client connectivity. If a second client attempts to connect, it receives an error message and the connection is closed.
5.13.3 Threading Model
The bridge uses a cooperative asyncio model. Two coroutines per client run concurrently:
rx_loop() — Receives JSON from the phone, parses the message type, and publishes to the appropriate ROS 2 topic.
tx_loop() — Dequeues outgoing ROS 2 messages (enqueued via call_soon_threadsafe() from ROS 2 callbacks) and sends them as JSON. If no messages are queued for 200 ms, a comms keepalive is sent containing the elapsed time since the last heartbeat.
5.14 System Bringup (mower_bringup)
5.14.1 Launch Architecture
The mower_stack.launch.py file orchestrates the full system startup. It launches seven nodes in dependency order:
Table 5.12 — Full System Launch Sequence
Order
Node
Package
Dependency
1
rtk_reader
rtk_reader
None (publishes /rtk/fix)
2
imu_node
imu_reader
None (publishes /imu/data)
3
encoder_node
mower_autonomy
None (publishes /encoders/ticks)
4
localization_node
mower_autonomy
Needs /rtk/fix, /imu/data, /encoders/ticks
5
obstacle_detector_node
mower_autonomy
None (publishes /obstacles/action)
6
autonomy_node
mower_autonomy
Needs /mower/pose, /encoders/ticks, map
7
map_server
mower_mapping
Needs /rtk/fix
8
bridge_node
lawnbot_comms
Needs ROS 2 topics
9
obstacle_detector_node
obstacle_detection
Spawned dynamically by lawnbot_motors WebSocket server
10
battery_monitor
lawn_mower_battery
Launched separately via battery_monitor.launch.py

Note: The lawnbot_motors WebSocket server (port 8766) is started separately as a systemd service, not as a ROS 2 node.
5.14.2 Network Ports
Table 5.13 — WebSocket Port Allocation
Port
Service
Package
8766
Motor control
lawnbot_motors
8770
Map recording
mower_mapping
9002
Mobile app bridge
lawnbot_comms

5.14.3 Resource Budget
Table 5.14 — Estimated Resource Usage (Raspberry Pi 5, 8 GB RAM)
Subsystem
CPU (%)
RAM (MB)
IMU reader + Encoder node
5
40
Localization node
3
30
AI Camera Vision / Obstacle detection
15
100
Autonomy node
5
50
RTK Reader
3
20
Motor Server
3
20
Map Server + Comms Bridge
3
30
Total
~37
~290

5.14.4 Topic Interconnection Summary
The data flows through three logical layers:
Sensor Layer — rtk_reader, imu_reader, encoder_node, and lawn_mower_battery publish raw sensor data.
Fusion Layer — localization_node consumes sensor topics (/rtk/fix, /imu/data, /encoders/ticks) and produces the fused pose estimate (/mower/pose).
Decision Layer — autonomy_node, obstacle_detector_node, and mower_mapping consume the fused state and produce motor commands or map data.
Figure 5.15 — Complete ROS 2 Topic Registry5.15 Software Dependencies
Table 5.16 — Key Software Dependencies
Library
Version
Purpose
ROS 2 Jazzy (rclpy)
7.1.6
ROS 2 Python client library
gpiozero
—
GPIO and PWM control (Pi 5 compatible)
lgpio
—
Low-level GPIO backend for gpiozero on Pi 5
pyserial
3.5
Serial communication with ZED-F9P
websockets
16.0
Async WebSocket servers and clients
depthai
—
OAK-D Lite camera SDK
adafruit-circuitpython-bno08x
1.3.1
BNO085 IMU driver
adafruit-circuitpython-ads1x15
—
ADS1015 ADC driver
smbus2
—
I2C register access (MPU6050, MCP23017)
shapely
—
Polygon geometry (coverage planner, geofence)
numpy
—
Numerical operations


6. Mobile Application
6.1 Application Architecture
6.2 User Interface Design
6.3 Communication with the Mower
6.4 Features and Functionality
7. Integration and Testing
7.1 Unit Testing
7.2 Subsystem Integration Testing
7.3 Full System Testing
7.4 Field Testing Conditions and Setup
8. Results
8.1 Navigation and Path Planning Results
8.2 Obstacle Detection and Avoidance Results
8.3 Battery Performance Results
8.4 Camera Vision and AI Model Results
8.5 RTK-GPS Accuracy Results
8.6 Motor and Odometry Results
8.7 Mobile App Performance Results
8.8 Overall System Performance
9. Discussion
9.1 Analysis of Results
9.2 Problems Encountered and Solutions Proposed
9.3 Deviations from Initial Schedule
9.4 Design Trade-offs
9.5 Sales Pitch
10. Conclusion
10.1 Summary of Achievements
10.2 Lessons Learned
10.3 Future Work and Recommendations
References
Appendices
5.3


5.6


5.13




Appendix A: ELSEE Assignment
Appendix B: Technical Manual
Appendix C: User Manual
Appendix D: Schematics and PCB Layout Drawings
Appendix E: Software Repository and Code Listing

