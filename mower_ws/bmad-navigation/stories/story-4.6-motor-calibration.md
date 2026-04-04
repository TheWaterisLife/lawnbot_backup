# Story 4.6: Motor Calibration

## Status: ✅ Complete

## Description

As a technician, I need documentation and procedures to calibrate the motor controller so that the mower drives straight and responds correctly to commands.

## Acceptance Criteria

- [x] Pin configuration documented
- [x] Step-by-step calibration procedure
- [x] Direction inversion configuration
- [x] Speed calibration (min/max)
- [x] Troubleshooting guide

## Technical Implementation

### Files Created
- `src/mower_navigation/docs/MOTOR_CALIBRATION.md`

### Documentation Contents

1. **Prerequisites**
   - Raspberry Pi connected to motor hardware
   - MCP23017 I2C expander wired correctly
   - Motors connected to PWM and enable pins

2. **Pin Configuration Reference**
   - Right Motor: GPIO 12/18 (PWM), GPB2/3 (Enable)
   - Left Motor: GPIO 13/19 (PWM), GPB0/1 (Enable)
   - Blade Motor: GPIO 5/6 (PWM), GPB4/5 (Enable)

3. **Calibration Steps**
   - Test motor direction
   - Invert direction if needed
   - Find minimum speed threshold
   - Set maximum safe speed
   - Adjust ramp rate
   - Measure track width

4. **Configuration Storage**
   ```yaml
   # config/motor_calibration.yaml
   motor_calibration:
     left_motor:
       invert_direction: false
       min_speed: 0.10
     right_motor:
       invert_direction: true
       min_speed: 0.12
     general:
       max_speed: 0.80
       ramp_rate: 2.0
       track_width: 0.24
   ```

5. **MCP23017 Verification**
   ```bash
   i2cdetect -y 1
   # Should show 0x20
   ```

6. **Troubleshooting Table**
   - Motor doesn't run → Check enable pins
   - Motor runs backward → Set invert_direction
   - Motor stutters → Increase min_speed
   - Jerky acceleration → Decrease ramp_rate

## Related Stories

- Story 4.1: Motor Controller Basics
- Story 4.7: Motor Diagnostics
