# Architecture: Lawnbot Motors Subsystem

## 1. System Context

```
+------------------------------------------------------------------+
|                    Autonomous Lawn Mower                          |
+------------------------------------------------------------------+
|                                                                   |
|  +-----------------+     +------------------+                     |
|  | Lawnbot Comms   |     |   Navigation     |                     |
|  | (app bridge)    |     |   (future)       |                     |
|  +--------+--------+     +------------------+                     |
|           |                                                       |
|           | WebSocket / future ROS2                               |
|           v                                                       |
|  +------------------------------------------+                    |
|  |       LAWNBOT MOTORS SERVER              |                    |
|  |          (this project)                  |                    |
|  |                                          |                    |
|  |  WebSocket Server (ws://0.0.0.0:8766)    |                    |
|  |       |                                  |                    |
|  |       v                                  |                    |
|  |  +----------+  +----------+  +--------+  |                    |
|  |  |  Right   |  |  Left    |  | Blade  |  |                    |
|  |  |  Motor   |  |  Motor   |  | Motor  |  |                    |
|  |  | BTS7960  |  | BTS7960  |  | BTS7960|  |                    |
|  |  +----+-----+  +----+-----+  +---+----+  |                    |
|  +-------|----------|--------|-------+-------+                    |
+-----------+-----------+------+-------+------------------------+
            |           |      |       |
            v           v      v       v
     +------+---+ +----+----+ +---+----+
     | GPIO PWM | | GPIO PWM| |GPIO PWM|
     | 12,19    | | 13,18   | | 26,21  |
     | EN 23,24 | | EN 5,6  | | EN 20,16|
     +----------+ +---------+ +--------+
```

---

## 2. Component Architecture

### 2.1 Package Structure

```
src/lawnbot_motors/
├── lawnbot_motors/
│   └── __init__.py
├── motor_hw.py                # BTS7960Motor driver (gpiozero)
├── ws_motor_server.py         # WebSocket command server
├── motor_config.yaml          # Pin mapping + tuning
├── resource/
│   └── lawnbot_motors
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
└── setup.cfg
```

### 2.2 Class Descriptions

#### BTS7960Motor (`motor_hw.py`)

**Responsibility:** Low-level motor control via gpiozero GPIO.

```
+---------------------------+
|      MotorPins (dataclass)|
+---------------------------+
| r_en: int                 |
| l_en: int                 |
| r_pwm: int                |
| l_pwm: int                |
+---------------------------+

+---------------------------+
|      BTS7960Motor         |
+---------------------------+
| - name: str               |
| - pins: MotorPins         |
| - max_duty: float         |
| - invert: bool            |
| - r_en: DigitalOutput     |
| - l_en: DigitalOutput     |
| - r_pwm: PWMOutput        |
| - l_pwm: PWMOutput        |
+---------------------------+
| + enable()                |
| + disable()               |
| + stop()                  |
| + set_speed(speed: float) |
+---------------------------+
```

**Speed Control Logic:**
```
speed > 0 (forward):  R_PWM = duty,  L_PWM = 0
speed < 0 (reverse):  L_PWM = duty,  R_PWM = 0
speed = 0 (stop):     R_PWM = 0,     L_PWM = 0
```

#### WebSocket Server (`ws_motor_server.py`)

**Responsibility:** Accept commands via WebSocket, apply safety features, drive motors.

**Helper Functions:**

| Function | Purpose |
|----------|---------|
| `load_config()` | Parse YAML configuration |
| `safe_float()` | Safe float conversion with default |
| `clamp()` | Limit value to range |
| `deadzone()` | Zero-out small inputs |
| `ramp_towards()` | Slew-rate limiter |
| `stop_all()` | Emergency stop all motors |
| `apply_speeds()` | Set speeds from dict |
| `build_motors_from_config()` | Factory from YAML |

---

## 3. Data Flow

### 3.1 Command Processing

```
Mobile App / Comms Bridge
       |
       | WebSocket JSON
       v
+------------------+
| ws_motor_server  |
|                  |
| 1. Parse JSON    |
| 2. Match command |
| 3. Apply safety  |
|    - deadzone    |
|    - clamp       |
|    - ramp        |
| 4. Set motor PWM |
| 5. Send ACK      |
+------------------+
       |
       v
+------------------+
|  BTS7960Motor    |
|  (gpiozero)      |
|                  |
| R_EN, L_EN → on  |
| R_PWM → duty     |
| L_PWM → 0        |
+------------------+
       |
       v
   Physical Motor
```

### 3.2 Watchdog Flow

```
+-------------------+
| Every 50ms check: |
|                   |
| time.now() -      |
| last_cmd_time     |
| > 0.4s ?          |
|                   |
|   YES → stop_all()|
|   NO  → continue  |
+-------------------+
```

### 3.3 Drive Command (Teleop) Pipeline

```
Raw Input → Deadzone → Clamp(max_wheel) → Slew-Rate Ramp → Motor PWM
  0.05        0.0         0.0                0.0              0.0
  0.30        0.30        0.30               0.15*            0.15
  0.60        0.60        0.50               0.35*            0.35

* ramp_per_sec=1.5, dt varies
```

---

## 4. GPIO Pin Mapping

### 4.1 BTS7960 Wiring

Each BTS7960 module has 4 control pins:

| Pin | Function | gpiozero Type |
|-----|----------|---------------|
| R_EN | Right enable (forward enable) | `DigitalOutputDevice` |
| L_EN | Left enable (reverse enable) | `DigitalOutputDevice` |
| R_PWM | Right PWM (forward speed) | `PWMOutputDevice` |
| L_PWM | Left PWM (reverse speed) | `PWMOutputDevice` |

### 4.2 Motor Pin Assignment

| Motor | R_EN | L_EN | R_PWM | L_PWM |
|-------|------|------|-------|-------|
| **Right** | 5 | 6 | 19 | 12 |
| **Left** | 23 | 24 | 18 | 13 |
| **Blade** | 4 | 21 | 16 | 20 |

> [!WARNING]
> These GPIO pins must NOT be used for encoders or other functions. Encoder pins (17, 27, 22, 4) are separate.

### 4.3 Full GPIO Usage Map

| GPIO | Function | Package |
|------|----------|---------|
| 5, 6, 19, 12 | Right motor | lawnbot_motors |
| 23, 24, 18, 13 | Left motor | lawnbot_motors |
| 16, 20, 21, 4 | Blade motor | lawnbot_motors |
| 17, 27, 22, 4 | Encoders | sensor_integration |
| 2, 3 | I2C (IMU) | sensor_integration |

---

## 5. Configuration

### 5.1 motor_config.yaml

```yaml
server:
  host: "0.0.0.0"
  port: 8766
  watchdog_timeout_s: 0.4

gpio:
  pwm_frequency_hz: 1000

teleop:
  max_wheel: 0.5
  deadzone: 0.08
  ramp_per_sec: 1.5

profiles:
  default_on:
    right: 0.6
    left: 0.6
    blade: 0.1

motors:
  right:
    r_en: 5
    l_en: 6
    r_pwm: 19
    l_pwm: 12
    max_duty: 1.0
    invert: false
  left:
    r_en: 23
    l_en: 24
    r_pwm: 18
    l_pwm: 13
    max_duty: 1.0
    invert: false
  blade:
    r_en: 4
    l_en: 21
    r_pwm: 16
    l_pwm: 20
    max_duty: 1.0
    invert: false
```

---

## 6. WebSocket Protocol

### 6.1 Connection

On connect, server sends:
```json
{"ok":true, "motors":["right","left","blade"], "commands":["on","stop","set","drive","blade"], "port":8766}
```

### 6.2 Commands

| Command | Request | Response |
|---------|---------|----------|
| `stop` | `{"cmd":"stop"}` | `{"ok":true,"cmd":"stop"}` |
| `on` | `{"cmd":"on"}` | `{"ok":true,"cmd":"on","applied":{...}}` |
| `drive` | `{"cmd":"drive","left":0.3,"right":0.3}` | `{"ok":true,"cmd":"drive"}` |
| `blade` | `{"cmd":"blade","speed":0.5}` | `{"ok":true,"cmd":"blade","speed":0.5}` |
| `set` | `{"cmd":"set","right":0.4}` | `{"ok":true,"cmd":"set","applied":{...}}` |

### 6.3 Error Responses

```json
{"ok":false, "error":"invalid_json"}
{"ok":false, "error":"unknown_cmd", "got":"xyz"}
{"ok":false, "error":"no_motor_fields"}
{"ok":false, "error":"missing_left_or_right_motor"}
{"ok":false, "error":"missing_blade_motor"}
```

---

## 7. Error Handling

| Failure | Detection | Response |
|---------|-----------|----------|
| No command for 0.4s | Watchdog timer | Stop all motors |
| Invalid JSON | Parse exception | Send error, continue |
| Unknown command | String match fails | Send error, continue |
| Missing motor | Motor dict lookup | Send error, skip |
| Server shutdown | Finally block | Stop + disable all GPIO |

---

## 8. Threading Model

```
+-------------------------------+
|      asyncio Event Loop       |
+-------------------------------+
|                               |
|  +----------+  +----------+  |
|  | WebSocket|  | Watchdog |  |
|  | handler  |  | task     |  |
|  | (per-    |  | (50ms    |  |
|  |  client) |  |  check)  |  |
|  +----------+  +----------+  |
|       |                       |
|  Parse cmd → safety → PWM    |
+-------------------------------+
```

---

## 9. Dependencies

| Dependency | Purpose |
|------------|---------|
| `gpiozero` | GPIO PWM and digital output (Pi 5) |
| `websockets` | Async WebSocket server |
| `pyyaml` | YAML config parsing |
| `asyncio` | Event loop |
