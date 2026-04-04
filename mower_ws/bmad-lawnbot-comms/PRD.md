# Product Requirements Document: Lawnbot Comms Subsystem

## 1. Overview

### 1.1 Purpose
This document defines the requirements for the Lawnbot Comms subsystem, which provides a WebSocket bridge between the mobile app and the ROS2 system for remote control and monitoring.

### 1.2 Scope
- WebSocket server (port 9002)
- Phone → ROS2 command bridging (heartbeat, mode, teleop)
- ROS2 → Phone telemetry streaming (state, telemetry)
- Connection management and health monitoring
- Fake telemetry node for testing

---

## 2. Functional Requirements

### 2.1 WebSocket Server (FR-WS)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-WS-01 | Listen on `0.0.0.0:9002` | Must |
| FR-WS-02 | Accept single WebSocket client | Must |
| FR-WS-03 | Reject additional connections | Must |
| FR-WS-04 | Send connection ACK with version | Must |
| FR-WS-05 | Handle client disconnect gracefully | Must |

**Acceptance Criteria:**
- Server binds and listens on startup
- First client receives `{"type":"ack","msg":"connected","version":1}`
- Second client receives error and is closed
- Disconnect logged, slot freed for new client

### 2.2 Phone → ROS2 Commands (FR-CMD)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-CMD-01 | Parse incoming JSON messages | Must |
| FR-CMD-02 | Handle `hello` → respond with ACK | Must |
| FR-CMD-03 | Handle `heartbeat` → publish to `/app/heartbeat` | Must |
| FR-CMD-04 | Handle `mode` → publish to `/app/mode` | Must |
| FR-CMD-05 | Handle `teleop` → publish Twist to `/app/teleop_cmd` | Must |
| FR-CMD-06 | Ignore unknown message types | Must |

**Message Schemas:**

```json
{"type": "hello"}
{"type": "heartbeat"}
{"type": "mode", "value": "TELEOP"}
{"type": "teleop", "linear": 0.3, "angular": 0.0}
```

**Acceptance Criteria:**
- Valid commands published to correct ROS2 topics
- Malformed JSON silently discarded
- Teleop Twist has `linear.x` and `angular.z` set

### 2.3 ROS2 → Phone Telemetry (FR-TEL)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-TEL-01 | Subscribe to `/mower/state` | Must |
| FR-TEL-02 | Subscribe to `/mower/telemetry` | Must |
| FR-TEL-03 | Forward state as `{"type":"state","data":...}` | Must |
| FR-TEL-04 | Forward telemetry as `{"type":"telemetry","data":...}` | Must |
| FR-TEL-05 | Send comms health every 200ms idle | Should |

**Acceptance Criteria:**
- State and telemetry forwarded to phone within 50ms
- Comms keepalive includes `heartbeat_age_ms`

### 2.4 Fake Telemetry Node (FR-FAKE)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-FAKE-01 | Publish fake state at 5 Hz | Should |
| FR-FAKE-02 | Publish fake telemetry at 5 Hz | Should |
| FR-FAKE-03 | Include WiFi IP in telemetry | Should |

---

## 3. Non-Functional Requirements

### 3.1 Performance (NFR-PERF)

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-PERF-01 | WebSocket message latency | < 50 ms |
| NFR-PERF-02 | CPU usage | < 3% |
| NFR-PERF-03 | Memory usage | < 30 MB |

### 3.2 Reliability (NFR-REL)

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-REL-01 | No crash on malformed input | 100% |
| NFR-REL-02 | Clean disconnect handling | Always |
| NFR-REL-03 | Node uptime | > 99.9% |

---

## 4. Interface Specifications

### 4.1 Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/app/heartbeat` | `std_msgs/Empty` | Phone keepalive |
| `/app/mode` | `std_msgs/String` | Mode command |
| `/app/teleop_cmd` | `geometry_msgs/Twist` | Joystick velocity |

### 4.2 Subscribed Topics

| Topic | Message Type | Source |
|-------|--------------|--------|
| `/mower/state` | `std_msgs/String` | Navigation/state machine |
| `/mower/telemetry` | `std_msgs/String` | System monitor |

### 4.3 WebSocket Interface

| Endpoint | Port | Protocol |
|----------|------|----------|
| `ws://0.0.0.0:9002` | 9002 | WebSocket (JSON) |

---

## 5. Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| rclpy | Jazzy | ROS2 Python client |
| std_msgs | Jazzy | String, Empty messages |
| geometry_msgs | Jazzy | Twist message |
| websockets | Latest | WebSocket server |

---

## 6. Acceptance Criteria Summary

The Lawnbot Comms subsystem is complete when:

1. WebSocket server accepts phone connection
2. Teleop joystick commands flow to ROS2 topics
3. Mode switching works from app
4. State and telemetry stream to phone in real-time
5. Heartbeat age tracked for connection health
6. Second client properly rejected
