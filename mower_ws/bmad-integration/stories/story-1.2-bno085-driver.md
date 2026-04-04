# Story 1.2: BNO085 I2C Driver

**Epic**: 1 - IMU Integration (MPU-6050 / BNO085)  
**Status**: Not Started  
**Priority**: Must (for production)  

## User Story

**As a** developer  
**I want** a Python class that communicates with the BNO085 via I2C  
**So that** I can read sensor data reliably

## Acceptance Criteria

- [ ] Initialize BNO085 in NDOF mode
- [ ] Read quaternion (w, x, y, z)
- [ ] Read angular velocity (x, y, z) in rad/s
- [ ] Read linear acceleration (x, y, z) in m/s^2
- [ ] Read calibration status (sys, gyro, accel, mag)
- [ ] Handle I2C communication errors with retries

## Technical Details

### Hardware Interface

| Parameter | Value |
|-----------|-------|
| I2C Bus | 1 (default on Pi 5) |
| I2C Address | 0x4A (default) |
| Max Speed | 400 kHz |

### BNO085 Configuration

- Mode: NDOF (Nine Degrees of Freedom)
- Output: Fused orientation + raw gyro/accel
- Internal fusion rate: 100 Hz

### Class Interface

```python
class BNO085Driver:
    def __init__(self, bus: int = 1, address: int = 0x4A):
        """Initialize BNO085 driver."""
        pass
    
    def initialize(self) -> bool:
        """Configure BNO085 for NDOF mode. Returns True on success."""
        pass
    
    def read_quaternion(self) -> Tuple[float, float, float, float]:
        """Read orientation as quaternion (w, x, y, z)."""
        pass
    
    def read_gyro(self) -> Tuple[float, float, float]:
        """Read angular velocity (x, y, z) in rad/s."""
        pass
    
    def read_accel(self) -> Tuple[float, float, float]:
        """Read linear acceleration (x, y, z) in m/s^2."""
        pass
    
    def get_calibration_status(self) -> Tuple[int, int, int, int]:
        """Get calibration status (sys, gyro, accel, mag) 0-3."""
        pass
    
    def is_calibrated(self) -> bool:
        """Check if all calibration values are >= 2."""
        pass
```

## Dependencies

- `smbus2` - I2C communication library
- BNO085 connected to I2C bus 1

## Test File

`Tests/unit/integration/test_imu_1_1_driver.py`

## Definition of Done

- [ ] BNO085Driver class implemented
- [ ] All unit tests passing
- [ ] Hardware test passing on Pi
- [ ] Code reviewed
- [ ] Documented in code
