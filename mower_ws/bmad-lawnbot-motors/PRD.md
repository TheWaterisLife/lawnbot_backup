# Product Requirements Document: Lawnbot Motors Subsystem

## 1. Overview

### 1.1 Purpose
This document defines the requirements for the Lawnbot Motors subsystem, which provides low-level motor control via BTS7960 H-Bridge drivers, exposed through a WebSocket server interface.

### 1.2 Scope
- BTS7960 H-Bridge motor control (left, right, blade)
- WebSocket command server (port 8766)
- Teleop driving with smoothing and safety
- YAML-based configuration

### 1.3 Definitions

| Term | Definition |
|------|------------|
| BTS7960 | H-Bridge motor driver module |
| PWM | Pulse Width Modulation — controls motor speed |
| Slew Rate | Maximum rate of speed change per second |
| Deadzone | Small input values ignored as joystick noise |
| Watchdog | Timer that stops motors if no commands received |

---

## 2. Functional Requirements

### 2.1 Motor Hardware Control (FR-HW)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-HW-01 | Control 3 motors via BTS7960 H-Bridge (left, right, blade) | Must |
| FR-HW-02 | Use gpiozero PWMOutputDevice for PWM control | Must |
| FR-HW-03 | Use gpiozero DigitalOutputDevice for enable pins | Must |
| FR-HW-04 | Support forward/reverse via R_PWM/L_PWM | Must |
| FR-HW-05 | Speed range: -1.0 (full reverse) to +1.0 (full forward) | Must |
| FR-HW-06 | Support per-motor max_duty limit | Should |
| FR-HW-07 | Support per-motor direction inversion | Should |

**Pin Configuration:**

| Motor | R_EN | L_EN | R_PWM | L_PWM |
|-------|------|------|-------|-------|
| Right | GPIO 23 | GPIO 24 | GPIO 12 | GPIO 19 |
| Left | GPIO 5 | GPIO 6 | GPIO 13 | GPIO 18 |
| Blade | GPIO 20 | GPIO 16 | GPIO 26 | GPIO 21 |

**Acceptance Criteria:**
- Motors spin in correct direction at commanded speed
- Enable pins activated on startup, deactivated on shutdown
- PWM frequency configurable (default 1000 Hz)

### 2.2 WebSocket Server (FR-WS)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-WS-01 | Listen on configurable host:port (default 0.0.0.0:8766) | Must |
| FR-WS-02 | Accept JSON commands | Must |
| FR-WS-03 | Send JSON ACK with available motors and commands | Must |
| FR-WS-04 | Handle `stop` → stop all motors | Must |
| FR-WS-05 | Handle `on` → apply default profile | Must |
| FR-WS-06 | Handle `drive` → set left/right speeds with smoothing | Must |
| FR-WS-07 | Handle `blade` → set blade speed | Must |
| FR-WS-08 | Handle `set` → manual per-motor speed | Should |
| FR-WS-09 | Reject unknown commands with error | Must |

**Command Schemas:**

```json
{"cmd": "stop"}
{"cmd": "on"}
{"cmd": "drive", "left": 0.3, "right": 0.3}
{"cmd": "blade", "speed": 0.5}
{"cmd": "set", "right": 0.4, "left": 0.4, "blade": 0.0}
```

**Acceptance Criteria:**
- All commands produce correct motor output
- Malformed JSON returns error response
- Connection ACK includes motor list and command list

### 2.3 Safety Features (FR-SAF)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-SAF-01 | Watchdog: stop all motors if no command for 0.4s | Must |
| FR-SAF-02 | Clamp wheel speeds to `max_wheel` limit | Must |
| FR-SAF-03 | Apply deadzone filter to joystick input | Must |
| FR-SAF-04 | Slew-rate ramping for smooth acceleration | Should |
| FR-SAF-05 | Stop all motors on server shutdown | Must |
| FR-SAF-06 | Disable motor enables on cleanup | Must |

**Teleop Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| max_wheel | 0.5 | Maximum manual speed |
| deadzone | 0.08 | Ignore inputs below this |
| ramp_per_sec | 1.5 | Speed change per second (0 = disabled) |
| watchdog_timeout_s | 0.4 | Auto-stop timeout |

**Acceptance Criteria:**
- Motors stop within 0.4s of last command
- No sudden speed jumps from joystick
- Clean shutdown disables all GPIO

### 2.4 Configuration (FR-CFG)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-CFG-01 | Load all settings from YAML config file | Must |
| FR-CFG-02 | Define motor pins per motor | Must |
| FR-CFG-03 | Define teleop tuning parameters | Must |
| FR-CFG-04 | Define default "on" speed profile | Should |

---

## 3. Non-Functional Requirements

### 3.1 Performance (NFR-PERF)

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-PERF-01 | Command-to-motor latency | < 20 ms |
| NFR-PERF-02 | PWM frequency | 1000 Hz |
| NFR-PERF-03 | CPU usage | < 5% |

### 3.2 Reliability (NFR-REL)

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-REL-01 | Watchdog reliability | 100% (never miss timeout) |
| NFR-REL-02 | Clean GPIO shutdown | Always |

---

## 4. Interface Specifications

### 4.1 WebSocket Interface

| Endpoint | Port | Protocol |
|----------|------|----------|
| `ws://0.0.0.0:8766` | 8766 | WebSocket (JSON) |

### 4.2 Connection ACK

```json
{
  "ok": true,
  "motors": ["right", "left", "blade"],
  "commands": ["on", "stop", "set", "drive", "blade"],
  "port": 8766
}
```

---

## 5. Dependencies

| Dependency | Purpose |
|------------|---------|
| gpiozero | GPIO PWM and digital output |
| websockets | WebSocket server |
| pyyaml | YAML configuration parsing |
| asyncio | Async event loop |

---

## 6. Acceptance Criteria Summary

The Lawnbot Motors subsystem is complete when:

1. All 3 motors controlled via WebSocket commands
2. Watchdog stops motors on communication loss
3. Teleop drive has deadzone + ramping
4. Blade motor independently controllable
5. YAML config drives all pin/tuning settings
6. Clean GPIO shutdown on exit
