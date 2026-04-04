# Product Requirements Document: RTK Reader Subsystem

## 1. Overview

### 1.1 Purpose
This document defines the requirements for the RTK Reader subsystem, which provides centimeter-accurate GPS positioning by interfacing with a u-blox ZED-F9P receiver and NTRIP correction service.

### 1.2 Scope
- Serial communication with u-blox ZED-F9P
- NMEA GGA sentence parsing
- NTRIP client for RTK corrections
- ROS2 NavSatFix publishing

### 1.3 Definitions

| Term | Definition |
|------|------------|
| RTK | Real-Time Kinematic - cm-accurate GPS technique |
| NTRIP | Network Transport of RTCM via Internet Protocol |
| RTCM | Radio Technical Commission for Maritime - correction format |
| GGA | NMEA sentence containing position and fix quality |
| Fix Quality | 0=invalid, 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float |

---

## 2. Functional Requirements

### 2.1 Serial Communication (FR-SER)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-SER-01 | Open serial port to ZED-F9P | Must |
| FR-SER-02 | Configure baud rate (default 115200) | Must |
| FR-SER-03 | Read NMEA sentences continuously | Must |
| FR-SER-04 | Write RTCM corrections to receiver | Must |
| FR-SER-05 | Handle serial port errors gracefully | Must |

**Configuration:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyACM0` | Serial device path |
| `serial_baud` | 115200 | Baud rate |

**Acceptance Criteria:**
- Serial port opens successfully
- NMEA data received within 1 second
- RTCM data written without blocking NMEA reads

### 2.2 NMEA Parsing (FR-NMEA)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-NMEA-01 | Parse $GNGGA sentences | Must |
| FR-NMEA-02 | Validate NMEA checksum | Must |
| FR-NMEA-03 | Convert DDMM.MMMMM to decimal degrees | Must |
| FR-NMEA-04 | Extract fix quality indicator | Must |
| FR-NMEA-05 | Extract altitude (meters) | Should |
| FR-NMEA-06 | Handle malformed sentences gracefully | Must |

**GGA Fields Used:**

| Field | Index | Description |
|-------|-------|-------------|
| Latitude | 2-3 | DDMM.MMMMM, N/S |
| Longitude | 4-5 | DDDMM.MMMMM, E/W |
| Quality | 6 | Fix type (0-5) |
| Altitude | 9 | Meters above MSL |

**Acceptance Criteria:**
- Valid positions extracted from GGA
- Invalid checksum sentences discarded
- Correct hemisphere handling (N/S, E/W)

### 2.3 NTRIP Client (FR-NTRIP)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-NTRIP-01 | Connect to NTRIP caster | Must |
| FR-NTRIP-02 | Authenticate with base64 credentials | Must |
| FR-NTRIP-03 | Request specific mountpoint | Must |
| FR-NTRIP-04 | Stream RTCM data to receiver | Must |
| FR-NTRIP-05 | Auto-reconnect on disconnection | Must |
| FR-NTRIP-06 | Log connection status | Should |

**NTRIP Configuration:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `caster_host` | `3.143.243.81` | NTRIP caster IP |
| `caster_port` | 2101 | NTRIP caster port |
| `mountpoint` | `RD1_STATION_SO` | Base station stream |
| `username` | (configured) | NTRIP username |
| `password` | (configured) | NTRIP password |

**Acceptance Criteria:**
- NTRIP connection established within 10 seconds
- RTCM data flowing to receiver
- Automatic reconnect within 5 seconds on disconnect
- RTK-Fixed achieved within 60 seconds

### 2.4 ROS2 Publishing (FR-PUB)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-PUB-01 | Publish NavSatFix to `/rtk/fix` | Must |
| FR-PUB-02 | Set correct NavSatStatus based on quality | Must |
| FR-PUB-03 | Include timestamp in header | Must |
| FR-PUB-04 | Set frame_id to "gps" | Must |
| FR-PUB-05 | Log position periodically | Should |

**NavSatStatus Mapping:**

| GGA Quality | NavSatStatus | Meaning |
|-------------|--------------|---------|
| 0 | STATUS_NO_FIX (-1) | No fix |
| 1 | STATUS_FIX (0) | GPS only |
| 2, 4, 5 | STATUS_GBAS_FIX (2) | RTK/DGPS |

**Acceptance Criteria:**
- NavSatFix published at ~1 Hz
- Status correctly reflects fix quality
- Latitude/longitude in decimal degrees

---

## 3. Non-Functional Requirements

### 3.1 Performance (NFR-PERF)

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-PERF-01 | Serial read latency | < 50 ms |
| NFR-PERF-02 | Publication rate | ~1 Hz (GGA rate) |
| NFR-PERF-03 | CPU usage | < 5% |

### 3.2 Reliability (NFR-REL)

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-REL-01 | NTRIP reconnect time | < 5 sec |
| NFR-REL-02 | Serial error recovery | Auto-retry |
| NFR-REL-03 | Node uptime | > 99.9% |

---

## 4. Interface Specifications

### 4.1 Published Topics

| Topic | Message Type | Rate | Frame ID |
|-------|--------------|------|----------|
| `/rtk/fix` | sensor_msgs/NavSatFix | ~1 Hz | gps |

### 4.2 Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| serial_port | string | /dev/ttyACM0 | Serial device |
| serial_baud | int | 115200 | Baud rate |
| caster_host | string | 3.143.243.81 | NTRIP host |
| caster_port | int | 2101 | NTRIP port |
| mountpoint | string | RD1_STATION_SO | Base station |
| username | string | (configured) | NTRIP user |
| password | string | (configured) | NTRIP pass |

---

## 5. Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| rclpy | Jazzy | ROS2 Python client |
| sensor_msgs | Jazzy | NavSatFix message |
| pyserial | Latest | Serial communication |

---

## 6. Acceptance Criteria Summary

The RTK Reader subsystem is complete when:

1. Serial communication with ZED-F9P established
2. NTRIP corrections streaming to receiver
3. RTK-Fixed achieved within 60 seconds
4. NavSatFix published at ~1 Hz with correct status
5. Auto-reconnect on NTRIP disconnection
6. Position accuracy < 5 cm (RTK-Fixed)
