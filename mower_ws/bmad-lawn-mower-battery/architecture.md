# Architecture: Battery Monitor Subsystem

## 1. System Context

```
+------------------------------------------------------------------+
|                    Autonomous Lawn Mower                          |
+------------------------------------------------------------------+
|                                                                   |
|  +--------------+    +----------------+                           |
|  |   Battery    +--->+   ADS1015      |                           |
|  | (12.8V LiFe) |    | (0x49, Ch A3)  |                           |
|  +--------------+    +-------+--------+                           |
|       |                      | I2C                                |
|       |  R1=150kΩ            v                                    |
|       +---/\/\/---+  +------------------------------------------+ |
|                   |  |           BATTERY MONITORING              | |
|       +---/\/\/---+  |            (this project)                 | |
|       |  R2=27kΩ     |                                           | |
|       |              |  ADS1015 → Voltage → % Estimation         | |
|       +-- GND        |     ↓                                     | |
|                      |  MCP23017 → LEDs 1-5 (Optional)          | |
|                      |     ↓                                     | |
|                      |  ROS2 Pub: /battery (State)               | |
|                      +------------------------------------------+ |
|                                | ROS2                             |
|                                v                                  |
|                     +---------------------+                       |
|                     |    Navigation / UI  |                       |
|                     | (Consumes /battery) |                       |
|                     +---------------------+                       |
+------------------------------------------------------------------+
```

---

## 2. Component Architecture

### 2.1 Package Structure

```
src/lawn_mower_battery/
├── lawn_mower_battery/
│   ├── __init__.py
│   └── battery_monitor.py  # Main ROS2 node
├── config/
│   └── battery_params.yaml # Resistor, ADC & limit configs
├── launch/
│   └── battery_monitor.launch.py
├── resource/
│   └── lawn_mower_battery
├── package.xml
├── setup.py
└── setup.cfg              # Required for ament_python
```

### 2.2 Node Description

#### BatteryMonitor Node (`battery_monitor.py`)

**Responsibility:** Read ADC, estimate charge, update LEDs (optional), publish state.

**Class Diagram:**
```
+----------------------------------+
|         BatteryMonitor           |
+----------------------------------+
| - ads: ADS1015 (I2C, 0x49)      |
| - mcp: MCP23017 (I2C, optional) |
| - leds: List[DigitalInOut]      |
| - r1, r2: float (Divider)       |
| - min_v, max_v: float           |
| - adc_channel: int              |
+----------------------------------+
| + timer_callback()               |
| + voltage_to_percentage(v)       |
| + update_leds(pct)               |
+----------------------------------+
```

**Key Design Decisions:**
- ADS1015 and MCP23017 initialize independently — one failing does not disable the other
- ADS1015 address defaults to `0x49` (the working chip; `0x48` is faulty)
- ADC channel is configurable via parameter (default: A3, since A0 is used by blade RPM)
- MCP23017 is optional — if not detected, LEDs are disabled with a warning

---

## 3. Data Flow

```
ADS1015 (0x49, Channel A3)
    │
    ▼
Read Raw Voltage
    │
    ├── Apply Divider (Vin = Vout * (R1+R2)/R2)
    │
    ▼
Estimate % (LiFePO4 Curve)
    │
    ├── Pub /battery (BatteryState)
    │
    ├── Pub /battery/voltage (Float32)
    │
    ▼
Update LEDs (MCP23017, if present)
```

---

## 4. I2C Bus Map

| Address | Device | Usage |
|---------|--------|-------|
| `0x48` | ADS1015 | ❌ Faulty (constant offset) |
| `0x49` | ADS1015 | ✅ Battery (A3), Blade RPM (A0) |
| `0x20` | MCP23017 | Not installed (LED bar, optional) |
| `0x68` | MPU-6050 | IMU (separate subsystem) |

---

## 5. Dependencies

| Dependency | Purpose |
|------------|---------|
| `rclpy` | ROS2 Python client |
| `adafruit-blinka` | CircuitPython compatibility (board, busio, digitalio) |
| `adafruit-circuitpython-ads1x15` | ADC Driver |
| `adafruit-circuitpython-mcp230xx` | GPIO Extender Driver |
| `sensor_msgs` | BatteryState message |
| `std_msgs` | Float32 message |
