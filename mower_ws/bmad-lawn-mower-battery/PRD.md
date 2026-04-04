# Product Requirements Document: Battery Monitor Subsystem

## 1. Overview

### 1.1 Purpose
The Battery Monitor subsystem provides real-time state-of-charge estimation and visual feedback for the robot's power source.

### 1.2 Scope
- Voltage reading via ADS1015
- Voltage divider scaling
- LiFePO4 percentage estimation
- LED bar graph control (MCP23017)
- ROS 2 publication

---

## 2. Functional Requirements

### 2.1 Voltage Measurement (FR-VOLT)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-VOLT-01 | Read ADS1015 (0x49) Channel A3 at ≥1Hz | Must |
| FR-VOLT-02 | Apply voltage divider scaling | Must |
| FR-VOLT-03 | Filter noise (HW/SW) | Should |

**Acceptance Criteria:**
- Reading is accurate to ±0.1V compared to multimeter.

### 2.2 State Estimation (FR-EST)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-EST-01 | Map voltage to % using LiFePO4 curve | Must |
| FR-EST-02 | Handle ranges: 12.0V (0%) to 13.4V (100%) | Must |
| FR-EST-03 | Detect Critical Low (<12V) | Must |

**Acceptance Criteria:**
- 13.4V reports 100%, 12.8V reports ~50%, 12.0V reports 0%.

### 2.3 Visual Feedback (FR-LED)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-LED-01 | Control 5 LEDs via MCP23017 (if present) | Should |
| FR-LED-02 | Map % to LED count (20% per LED) | Must |
| FR-LED-03 | Flash all LEDs on critical low | Could |

### 2.4 ROS Interface (FR-ROS)

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-ROS-01 | Publish `/battery` (BatteryState) | Must |
| FR-ROS-02 | Publish `/battery/voltage` (Float32) | Must |
| FR-ROS-03 | Expose parameters for resistors/limits | Must |

---

## 3. Non-Functional Requirements

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-01 | CPU Usage | < 1% of one core |
| NFR-02 | I2C Usage | Minimized traffic |
| NFR-03 | Latency | < 500ms |

---

## 4. Interface Specifications

### 4.1 Published Topics

| Topic | Type | Frequency |
|-------|------|-----------|
| `/battery` | sensor_msgs/BatteryState | 1 Hz |
| `/battery/voltage` | std_msgs/Float32 | 1 Hz |

### 4.2 Parameters

| Name | Type | Default |
|------|------|---------|
| `voltage_divider_r1` | double | 150000.0 |
| `voltage_divider_r2` | double | 27000.0 |
| `min_voltage` | double | 12.0 |
| `max_voltage` | double | 13.4 |
| `ads1015_address` | int | 0x49 |
| `adc_channel` | int | 3 |
| `adc_gain` | int | 1 |
| `publish_rate` | double | 1.0 |

---

## 5. Acceptance Criteria Summary

1. ✅ Node starts without errors (ADS1015 connects, MCP23017 gracefully skipped if absent).
2. ✅ `/battery` topic publishes valid BatteryState data at 1 Hz.
3. ✅ Voltage and percentage match expected values (tested: 12.72V → 41.8%).
4. ✅ Parameters change calculation logic.
5. LEDs light up corresponding to input voltage (pending MCP23017 installation).
