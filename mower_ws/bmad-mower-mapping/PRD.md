# Product Requirements Document: Mower Mapping Subsystem

## 1. Overview

### 1.1 Purpose
This document defines the requirements for the Mower Mapping subsystem, which records lawn boundaries from RTK GPS data and provides map management through a WebSocket interface.

### 1.2 Scope
- GPS boundary recording from `/rtk/fix`
- Adaptive point sampling
- Path simplification (RDP)
- Map persistence (JSON)
- WebSocket server for app control

---

## 2. Functional Requirements

### 2.1 GPS Subscription (FR-GPS)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-GPS-01 | Subscribe to `/rtk/fix` (NavSatFix) | Must |
| FR-GPS-02 | Convert lat/lon to local XY (meters) | Must |
| FR-GPS-03 | Set origin at first fix or mapping start | Must |
| FR-GPS-04 | Track latest fix timestamp | Must |

**Acceptance Criteria:**
- Position updates processed at GPS rate (~1 Hz)
- Local XY accurate to cm level with RTK

### 2.2 Boundary Recording (FR-REC)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-REC-01 | Start/stop recording via WebSocket | Must |
| FR-REC-02 | Record points adaptively (distance, heading, time) | Must |
| FR-REC-03 | Auto-pause when GPS fix lost (>2s) | Must |
| FR-REC-04 | Auto-resume when GPS returns | Must |
| FR-REC-05 | Cancel recording and discard points | Should |

**Sampling Triggers:**

| Trigger | Threshold | Description |
|---------|-----------|-------------|
| Distance | ≥ 0.25 m | New point when moved enough |
| Heading | ≥ 8° | New point at corners/curves |
| Time | ≥ 1.0 s | New point at maximum interval |
| Minimum interval | 0.2 s | Prevent over-sampling |

**State Machine:**

| State | Description |
|-------|-------------|
| IDLE | Not recording |
| MAPPING | Actively recording GPS points |
| PAUSED | GPS lost, waiting for fix |

**Acceptance Criteria:**
- More points at corners (heading trigger)
- Fewer points on straight lines
- Auto-pause within 2s of GPS loss

### 2.3 Path Simplification (FR-SIMP)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-SIMP-01 | Simplify raw points using RDP algorithm | Must |
| FR-SIMP-02 | Tolerance: 0.08m (configurable) | Must |
| FR-SIMP-03 | Auto-close loop if within 0.5m of start | Must |
| FR-SIMP-04 | Require minimum 20 raw points | Must |

**Acceptance Criteria:**
- ~80% point reduction while preserving shape
- Closed polygon output
- Reject boundaries with < 20 points

### 2.4 Map Storage (FR-STORE)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-STORE-01 | Save maps as JSON to `~/mower_ws/maps/` | Must |
| FR-STORE-02 | File naming: `{name}_{timestamp}.json` | Must |
| FR-STORE-03 | Include origin lat/lon, raw points, simplified boundary | Must |
| FR-STORE-04 | List all saved maps | Must |
| FR-STORE-05 | Load map by ID | Must |

**Map JSON Schema:**
```json
{
  "id": "front_yard_20260210_183000",
  "name": "Front Yard",
  "origin_latlon": [45.123456, -73.654321],
  "raw_points_xy": [[0.0, 0.0], [0.25, 0.1], ...],
  "boundary_xy": [[0.0, 0.0], [2.5, 0.1], ...],
  "raw_count": 120,
  "boundary_count": 24,
  "created_at": "2026-02-10T18:30:00"
}
```

### 2.5 WebSocket Server (FR-WS)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-WS-01 | Listen on `0.0.0.0:8770` | Must |
| FR-WS-02 | Support multiple clients | Must |
| FR-WS-03 | Handle `map_start`, `map_stop`, `map_save`, `map_list`, `map_load`, `map_cancel` | Must |
| FR-WS-04 | Broadcast status updates to all clients | Must |
| FR-WS-05 | Broadcast live GPS pose at 5 Hz | Should |
| FR-WS-06 | Broadcast map preview during recording | Should |

**Broadcast Messages:**

| Type | Content | Rate |
|------|---------|------|
| `map_status` | State + point count | 5 Hz |
| `rtk_pose` | Current lat/lon | 5 Hz |
| `map_preview` | XY point array | 5 Hz (during mapping) |

---

## 3. Non-Functional Requirements

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-01 | CPU usage | < 5% |
| NFR-02 | Memory | < 30 MB |
| NFR-03 | WebSocket latency | < 50 ms |

---

## 4. Interface Specifications

### 4.1 Subscribed Topics

| Topic | Message Type | Source |
|-------|--------------|--------|
| `/rtk/fix` | sensor_msgs/NavSatFix | rtk_reader |

### 4.2 WebSocket Interface

| Endpoint | Port | Protocol |
|----------|------|----------|
| `ws://0.0.0.0:8770` | 8770 | WebSocket (JSON) |

---

## 5. Dependencies

| Dependency | Purpose |
|------------|---------|
| rclpy | ROS2 Python client |
| sensor_msgs | NavSatFix message |
| websockets | WebSocket server |

---

## 6. Acceptance Criteria Summary

1. Boundary recording works with adaptive sampling
2. Path simplified with RDP (< 0.08m error)
3. Loop auto-closed when near start
4. Maps saved/loaded from JSON files
5. Auto-pause on GPS loss, auto-resume on return
6. Live preview broadcast during mapping
