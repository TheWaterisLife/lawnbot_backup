# Battery Monitor ROS 2 Node

A ROS 2 node for monitoring a 12.8V LiFePO4 battery on a Raspberry Pi using an ADS1015 ADC and optional LED status display via MCP23017 GPIO extender.

## Features

- **Voltage Monitoring**: Reads battery voltage via ADS1015 12-bit ADC over I2C
- **Voltage Divider Support**: Configurable voltage divider with R1/R2 parameters
- **LiFePO4 Discharge Curve**: Accurate percentage estimation using piecewise linear interpolation
- **LED Status Display**: 5-LED battery level indicator (optional, via MCP23017)
- **ROS 2 Integration**: Publishes `sensor_msgs/BatteryState` and `std_msgs/Float32` messages
- **Configurable Parameters**: All settings accessible via ROS parameters and YAML

## Hardware Requirements

### Components
- **Raspberry Pi 5** (or compatible)
- **Battery**: 12.8V LiFePO4, 30Ah
- **ADC**: ADS1015 (I2C address `0x49`)
- **GPIO Extender**: MCP23017 (I2C address `0x20`) — optional for LED bar
- **LEDs**: 5x LEDs (optional)
- **Voltage Divider**: R1 = 150kΩ, R2 = 27kΩ

### Wiring
- **SDA/SCL**: I2C Bus 1 (GPIO 2, 3)
- **ADS1015 A3**: Battery Voltage Divider Output (channel configurable via `adc_channel`)
- **MCP23017 GPA0-GPA4**: LEDs 1-5 (optional)

### Voltage Divider Circuit
```
Battery (+) ─── R1 (150kΩ) ──┬── R2 (27kΩ) ─── Battery GND
                              │
                         ADS1015 A3
```

## Installation

### Dependencies
```bash
sudo pip3 install --break-system-packages adafruit-blinka adafruit-circuitpython-ads1x15 adafruit-circuitpython-mcp230xx
```

### Build
```bash
cd ~/mower_ws
colcon build --packages-select lawn_mower_battery
source install/setup.bash
```

## Usage

```bash
ros2 launch lawn_mower_battery battery_monitor.launch.py
```

### Verify Output
```bash
# Full battery state (voltage, percentage, health)
ros2 topic echo /battery

# Voltage only
ros2 topic echo /battery/voltage
```

## Configuration

Edit `config/battery_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `voltage_divider_r1` | 150000.0 | Top resistor (Ω) |
| `voltage_divider_r2` | 27000.0 | Bottom resistor (Ω) |
| `min_voltage` | 12.0 | Cutoff voltage |
| `max_voltage` | 13.4 | Full charge voltage |
| `ads1015_address` | 0x49 | ADS1015 I2C address |
| `adc_channel` | 3 | ADC input channel (0-3) |
| `gpio_extender_address` | 0x20 | MCP23017 I2C address |
| `adc_gain` | 1 | ADC gain (1 = ±4.096V) |
| `publish_rate` | 1.0 | Publishing rate (Hz) |

## Notes

- The MCP23017 (LED bar) is **optional** — if not detected, the node continues without LEDs
- ADS1015 at `0x48` on this system is faulty (constant offset); using `0x49` instead
- Channel A0 on `0x49` is used by blade motor RPM; battery uses A3
