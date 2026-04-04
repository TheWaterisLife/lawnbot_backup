# Story 1.1: MPU-6050 I2C Driver

**Epic**: 1 - IMU Integration (MPU-6050 / BNO085)  
**Status**: Not Started  
**Priority**: Must  

## User Story

**As a** developer  
**I want** a Python class that communicates with the MPU-6050 via I2C  
**So that** I can read sensor data for prototype testing

## Acceptance Criteria

- [ ] Initialize MPU-6050 with appropriate sensitivity settings
- [ ] Read raw accelerometer (x, y, z) and convert to m/s²
- [ ] Read raw gyroscope (x, y, z) and convert to rad/s
- [ ] Read temperature (for drift compensation reference)
- [ ] Handle I2C communication errors with retries
- [ ] Support configurable sample rate and sensitivity

## Technical Details

### Hardware Interface

| Parameter | Value |
|-----------|-------|
| I2C Bus | 1 (default on Pi 5) |
| I2C Address | 0x68 (AD0 to GND) |
| Max I2C Speed | 400 kHz |

### MPU-6050 Registers

| Register | Address | Description |
|----------|---------|-------------|
| PWR_MGMT_1 | 0x6B | Power management |
| SMPLRT_DIV | 0x19 | Sample rate divider |
| CONFIG | 0x1A | DLPF configuration |
| GYRO_CONFIG | 0x1B | Gyroscope range |
| ACCEL_CONFIG | 0x1C | Accelerometer range |
| ACCEL_XOUT_H | 0x3B | Accelerometer data start |
| GYRO_XOUT_H | 0x43 | Gyroscope data start |
| TEMP_OUT_H | 0x41 | Temperature data |

### Sensitivity Settings

**Accelerometer:**
| Setting | Range | Sensitivity |
|---------|-------|-------------|
| 0 | ±2g | 16384 LSB/g |
| 1 | ±4g | 8192 LSB/g |
| 2 | ±8g | 4096 LSB/g |
| 3 | ±16g | 2048 LSB/g |

**Gyroscope:**
| Setting | Range | Sensitivity |
|---------|-------|-------------|
| 0 | ±250°/s | 131 LSB/(°/s) |
| 1 | ±500°/s | 65.5 LSB/(°/s) |
| 2 | ±1000°/s | 32.8 LSB/(°/s) |
| 3 | ±2000°/s | 16.4 LSB/(°/s) |

### Class Interface

```python
from typing import Tuple, Optional
import smbus2

class MPU6050Driver:
    """Driver for MPU-6050 6-DOF IMU (accelerometer + gyroscope)."""
    
    # Default I2C address (AD0 to GND)
    DEFAULT_ADDRESS = 0x68
    
    def __init__(self, bus: int = 1, address: int = DEFAULT_ADDRESS):
        """Initialize MPU-6050 driver.
        
        Args:
            bus: I2C bus number (default 1 on Pi 5)
            address: I2C address (0x68 with AD0 low, 0x69 with AD0 high)
        """
        pass
    
    def initialize(self, 
                   accel_range: int = 0,  # ±2g
                   gyro_range: int = 0,   # ±250°/s
                   dlpf_mode: int = 3     # ~44Hz bandwidth
                   ) -> bool:
        """Configure MPU-6050 and wake from sleep. Returns True on success."""
        pass
    
    def read_accel_raw(self) -> Tuple[int, int, int]:
        """Read raw accelerometer values (x, y, z)."""
        pass
    
    def read_gyro_raw(self) -> Tuple[int, int, int]:
        """Read raw gyroscope values (x, y, z)."""
        pass
    
    def read_accel(self) -> Tuple[float, float, float]:
        """Read accelerometer in m/s² (x, y, z)."""
        pass
    
    def read_gyro(self) -> Tuple[float, float, float]:
        """Read gyroscope in rad/s (x, y, z)."""
        pass
    
    def read_temperature(self) -> float:
        """Read temperature in Celsius."""
        pass
    
    def read_all(self) -> dict:
        """Read all sensor data in one I2C transaction.
        
        Returns:
            dict with keys: 'accel' (m/s²), 'gyro' (rad/s), 'temp' (°C)
        """
        pass
    
    def is_connected(self) -> bool:
        """Check if MPU-6050 responds on I2C bus."""
        pass
    
    def get_status(self) -> dict:
        """Get driver status for diagnostics."""
        pass
```

### Conversion Formulas

```python
# Accelerometer: raw to m/s²
accel_mps2 = (raw_value / sensitivity) * 9.80665

# Gyroscope: raw to rad/s
gyro_rads = (raw_value / sensitivity) * (math.pi / 180.0)

# Temperature: raw to Celsius
temp_c = (raw_temp / 340.0) + 36.53
```

## Dependencies

- `smbus2` - I2C communication library
- MPU-6050 connected to I2C bus 1, address 0x68

## Test File

`Tests/unit/integration/test_imu_1_1_mpu6050_driver.py`

## Test Cases

1. Initialize MPU-6050 successfully
2. Read accelerometer data within expected range
3. Read gyroscope data within expected range
4. Read temperature within reasonable range
5. Handle I2C timeout with retry
6. Handle I2C error (device not present)
7. Verify sensitivity settings affect output
8. Verify is_connected() returns correct status

## Notes

- MPU-6050 is a 6-DOF sensor (no magnetometer)
- Yaw drift is expected without magnetometer - GPS heading compensates
- Temperature reading useful for drift compensation (future improvement)
- Production will use BNO085, this is for prototype testing only

## Definition of Done

- [ ] MPU6050Driver class implemented
- [ ] All unit tests passing with mock I2C
- [ ] Hardware test passing on Pi with real sensor
- [ ] Code reviewed
- [ ] Documented in code
