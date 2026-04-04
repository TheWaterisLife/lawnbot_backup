# Software Design — Battery Monitor (`lawn_mower_battery`)

## 1. Overview

The `lawn_mower_battery` package implements a ROS 2 node that monitors a 12.8 V LiFePO4 battery through a resistive voltage divider connected to an ADS1015 12-bit ADC. It estimates state of charge using a piecewise-linear LiFePO4 discharge curve, optionally drives a 5-LED battery bar via an MCP23017 GPIO expander, and publishes battery state on ROS 2 topics.

### 1.1 Design Rationale

The ADS1015 ADC was chosen because the Raspberry Pi lacks native analog inputs. A voltage divider (R1=150 kOhm, R2=22 kOhm) scales the 12.8 V battery voltage to the ADC's 0–3.3 V input range. The MCP23017 LED bar is treated as optional — if not detected on the I2C bus, LEDs are silently disabled without affecting battery monitoring.

### 1.2 Key Responsibilities

- Read battery voltage through the ADS1015 ADC voltage divider
- Estimate state of charge using LiFePO4 discharge curve lookup
- Classify battery health (good, dead, overvoltage)
- Drive 5-LED battery bar via MCP23017 (optional)
- Publish `sensor_msgs/BatteryState` on `/battery` at 1 Hz
- Publish `std_msgs/Float32` on `/battery/voltage` at 1 Hz

---

## 2. Class Diagram

```plantuml
@startuml
skinparam classAttributeIconSize 0

package "lawn_mower_battery" {

  class BatteryMonitor {
    -ads : ADS1015
    -chan : AnalogIn
    -mcp : MCP23017
    -mcp_available : bool
    -r1 : float
    -r2 : float
    -min_voltage : float
    -max_voltage : float
    -adc_channel : int
    -pub_state : Publisher<BatteryState>
    -pub_voltage : Publisher<Float32>
    -timer : Timer
    -lut : List[Tuple[float, float]]
    +timer_callback() : void
    +read_voltage() : float
    +voltage_to_percentage(v: float) : float
    +voltage_to_health(v: float) : int
    +update_leds(pct: float) : void
  }

  class VoltageDivider <<concept>> {
    r1 : 150 kOhm
    r2 : 22 kOhm
    formula : V_bat = V_adc * (R1+R2)/R2
  }

  class LiFePO4Curve <<concept>> {
    13.0V : 100%
    12.9V : 90%
    12.8V : 70%
    12.6V : 40%
    12.4V : 20%
    12.0V : 0%
  }
}

package "adafruit_ads1x15" <<external>> {
  class ADS1015
  class AnalogIn
}

package "smbus2" <<external>> {
  class SMBus
}

package "rclpy" <<external>> {
  class Node
}

BatteryMonitor --|> Node
BatteryMonitor "1" *-- "1" ADS1015 : reads ADC
BatteryMonitor "1" ..> "1" SMBus : MCP23017 register access
BatteryMonitor ..> VoltageDivider : applies formula
BatteryMonitor ..> LiFePO4Curve : lookup table

note right of BatteryMonitor
  ADS1015 (0x49, channel A3)
  MCP23017 (0x27, optional)
  Publish rate: 1 Hz
end note

@enduml
```

---

## 3. Sequence Diagram

```plantuml
@startuml
skinparam sequenceArrowThickness 2

participant "BatteryMonitor\nNode" as Node
participant "ADS1015\n(I2C 0x49)" as ADC
participant "Voltage Divider\nCalculation" as Divider
participant "LiFePO4\nLookup Table" as LUT
participant "MCP23017\n(I2C 0x27)" as MCP
participant "ROS 2\n/battery" as BatTopic
participant "ROS 2\n/battery/voltage" as VoltTopic

== Initialization ==
Node -> ADC : init ADS1015(i2c, address=0x49)
ADC --> Node : ADC ready
Node -> MCP : init MCP23017(i2c, address=0x27)
alt MCP23017 detected
  MCP --> Node : MCP ready
  Node -> MCP : set port B as output
else MCP23017 not found
  MCP --> Node : I2C error
  Node -> Node : mcp_available = False\n(LEDs disabled)
end

== Timer Callback (1 Hz) ==
loop every 1s
  Node -> ADC : read channel A3 voltage
  ADC --> Node : V_adc = 1.52 V

  Node -> Divider : V_bat = V_adc × (150k + 22k) / 22k
  Divider --> Node : V_bat = 11.88 → scaled to 12.72 V

  Node -> LUT : voltage_to_percentage(12.72 V)
  LUT --> Node : 52%

  Node -> Node : voltage_to_health(12.72 V)
  note right : 12.0–15.0 V → GOOD\n< 12.0 V → DEAD\n> 15.0 V → OVERVOLTAGE

  Node -> BatTopic : publish(BatteryState)
  note right : voltage=12.72, pct=0.52\nhealth=GOOD, tech=LIFE
  Node -> VoltTopic : publish(Float32: 12.72)

  opt mcp_available == True
    Node -> MCP : update_leds(52%)
    MCP -> MCP : GPB0=OFF, GPB1=OFF\nGPB2=ON, GPB3=ON, GPB4=ON
    note right : Active-low logic:\n≥80%→GPB0, ≥60%→GPB1\n≥40%→GPB2, ≥20%→GPB3\n≥0%→GPB4
  end
end

@enduml
```

---

## 4. System Context Diagram

```plantuml
@startuml
!include <C4/C4_Context>

title System Context — Battery Monitor

System(battery, "lawn_mower_battery", "ROS 2 battery monitor\nPublishes /battery @ 1 Hz")
System(motor_server, "lawnbot_motors", "WebSocket motor server\nRelays battery to mobile app")
System_Ext(ads1015, "ADS1015 ADC", "I2C addr 0x49\nChannel A3\n12-bit resolution")
System_Ext(mcp23017, "MCP23017 GPIO\nExpander", "I2C addr 0x27\n5-LED battery bar\n(optional)")
System_Ext(batt, "LiFePO4 Battery", "12.8 V nominal\nVoltage divider:\nR1=150k, R2=22k")

batt --> ads1015 : Divided voltage\n(0–1.9 V)
ads1015 --> battery : I2C ADC reading
battery --> mcp23017 : I2C LED updates\n(active-low, GPB0–GPB4)
battery --> motor_server : /battery\n(BatteryState @ 1 Hz)

note right of battery
  Charge estimation:
  13.0V = 100%
  12.0V = 0%
  Piecewise-linear LiFePO4 curve
end note

@enduml
```

---

## 5. Detailed Design

### 5.1 Voltage Measurement

The battery voltage is measured through a resistive voltage divider:

```
V_battery = V_adc × (R1 + R2) / R2
V_battery = V_adc × (150000 + 22000) / 22000
V_battery = V_adc × 7.818
```

The ADS1015 is read at I2C address `0x49` on channel A3 (A0 is reserved for blade RPM sensing). Address `0x48` is faulty and produces a constant offset.

### 5.2 State of Charge Estimation

A piecewise-linear interpolation of the LiFePO4 discharge curve maps voltage to percentage:

| Voltage | Charge |
|---|---|
| ≥ 13.0 V | 100% |
| 12.9 V | 90% |
| 12.8 V | 70% |
| 12.6 V | 40% |
| 12.4 V | 20% |
| ≤ 12.0 V | 0% |

Linear interpolation is used between table entries for smooth percentage transitions.

### 5.3 Health Assessment

| Condition | Voltage | Health Status |
|---|---|---|
| Normal operation | 12.0–15.0 V | `POWER_SUPPLY_HEALTH_GOOD` |
| Depleted | < 12.0 V | `POWER_SUPPLY_HEALTH_DEAD` |
| Overcharge/fault | > 15.0 V | `POWER_SUPPLY_HEALTH_OVERVOLTAGE` |

### 5.4 LED Indicator

The MCP23017 GPIO expander at I2C address `0x27` drives 5 LEDs on port B (GPB0–GPB4) using active-low logic. A bit cleared to 0 turns the LED ON. The `smbus2` library writes directly to the OLATB register (address `0x15`).

| LED (GPB Pin) | Color | Threshold |
|---|---|---|
| GPB0 | Green | ≥ 80% |
| GPB1 | Green | ≥ 60% |
| GPB2 | Amber | ≥ 40% |
| GPB3 | Amber | ≥ 20% |
| GPB4 | Red | ≥ 0% |

### 5.5 Published Topics

| Topic | Message Type | Rate | Content |
|---|---|---|---|
| `/battery` | `sensor_msgs/BatteryState` | 1 Hz | voltage, percentage, health, technology (LIFE) |
| `/battery/voltage` | `std_msgs/Float32` | 1 Hz | Raw battery voltage in volts |

### 5.6 I2C Bus Map

| Address | Device | Status |
|---|---|---|
| `0x48` | ADS1015 | Faulty (constant offset, not used) |
| `0x49` | ADS1015 | Active — battery (A3), blade RPM (A0) |
| `0x27` | MCP23017 | Optional — LED bar |
| `0x68` | MPU-6050 | Separate subsystem (imu_reader) |
