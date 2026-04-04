# BMAD: Battery Monitor Subsystem

## Overview

This BMAD project covers the **Battery Monitor** subsystem. It ensures the autonomous lawn mower's power source is monitored accurately and visual feedback is provided to the operator.

## Status: ✅ Tested & Operational

Successfully deployed and tested on Raspberry Pi. Publishing accurate voltage and percentage readings via ROS 2 topics.

## Scope

**In Scope:**
- Voltage reading from ADS1015 (I2C, address `0x49`, channel A3)
- Computation of battery percentage for LiFePO4 chemistry
- LED bar graph display via MCP23017 (I2C) — optional
- Publishing `sensor_msgs/BatteryState` to ROS 2

**Out of Scope:**
- Battery charging control (handled by separate charger)
- Power distribution (hardware)

## BMAD Documents

| Document | Status | Description |
|----------|--------|-------------|
| [product-brief.md](product-brief.md) | ✅ Done | High-level vision |
| [PRD.md](PRD.md) | ✅ Done | Requirements & acceptance criteria |
| [architecture.md](architecture.md) | ✅ Done | Technical design |

## Quick Start

```bash
# Install Drivers
sudo pip3 install --break-system-packages adafruit-blinka adafruit-circuitpython-ads1x15 adafruit-circuitpython-mcp230xx

# Build
cd ~/mower_ws
colcon build --packages-select lawn_mower_battery
source install/setup.bash

# Run
ros2 launch lawn_mower_battery battery_monitor.launch.py
```

## Configuration

Edit `src/lawn_mower_battery/config/battery_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ads1015_address` | `0x49` | ADS1015 I2C address |
| `adc_channel` | `3` | ADC input channel (A3) |
| `voltage_divider_r1` | `150000.0` | Top resistor (Ω) |
| `voltage_divider_r2` | `27000.0` | Bottom resistor (Ω) |
| `adc_gain` | `1` | ADC gain (1 = ±4.096V) |
| `publish_rate` | `1.0` | Publishing rate (Hz) |

## Hardware Notes

- **ADS1015 at `0x48`** is faulty (constant ~292 count offset) — do not use
- **ADS1015 at `0x49`** is the working ADC; channel A0 is used by blade motor RPM, battery uses **A3**
- **MCP23017 at `0x20`** is not currently installed — LED bar graph is disabled but the node handles this gracefully

## Related Subsystems

| Subsystem | Description | Link |
|-----------|-------------|------|
| Navigation | Path planning | [bmad-navigation](../bmad-navigation/README.md) |
| Integration | Sensor fusion | [bmad-integration](../bmad-integration/README.md) |
