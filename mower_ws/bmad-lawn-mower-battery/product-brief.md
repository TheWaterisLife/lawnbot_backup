# Product Brief: Battery Monitor Subsystem

## Vision

Ensure safe and reliable operation of the autonomous lawn mower by providing real-time battery status monitoring, LED visual feedback, and seamless integration with the robot's diagnostics system.

## Problem Statement

Without active battery monitoring, the robot risks:
1. **Deep Discharge**: Damaging the expensive LiFePO4 battery.
2. **Sudden Shutdown**: Stopping in the middle of a job or lawn.
3. **Safety Hazards**: Operating with unstable voltage.

The operator needs visual confirmation of charge level without needing a laptop.

## Solution

A dedicated ROS 2 node that monitors voltage via an ADS1015 ADC and displays status on LEDs:

| Feature | Implementation |
|---------|---------------|
| **Voltage Monitoring** | ADS1015 (I2C) with voltage divider |
| **Accurate Estimation** | LiFePO4-specific discharge curve |
| **Visual Feedback** | 5-LED bar graph via MCP23017 (optional) |
| **System Integration** | Publishes standard `sensor_msgs/BatteryState` |
| **Configuration** | TUNABLE ROS parameters (resistors, limits) |

## Target Users

1. **Operator**: Sees LED status at a glance.
2. **Navigation System**: Decides when to return to home (future).
3. **Remote UI**: Displays percentage on mobile app/dashboard.

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Voltage Accuracy | ±0.1V | Multimeter comparison |
| Update Rate | 1 Hz | Topic frequency |
| Visual Latency | < 1s | Time from voltage change to LED update |
| Reliability | 100% | ✅ Verified — no crashes on I2C errors or missing MCP23017 |

## Constraints

- **Hardware**: Pi 5, ADS1015 (0x49, A3), MCP23017 (optional)
- **Battery**: 4S LiFePO4 (12.8V Nom, 12.0V Cutoff)
- **Power**: 3.3V I2C logic

## Key Risks

| Risk | Mitigation |
|------|------------|
| Incorrect resistor values | Configurable parameters |
| I2C bus lockup | Error handling & auto-retry (future) |
| Non-linear discharge | Piecewise linear interpolation curve |
| Faulty ADS1015 chip | Using 0x49 (0x48 has constant offset); independent init prevents cascade failure |
