"""
BNO085 IMU Driver via I2C using Adafruit CircuitPython library.

Story 1.1: BNO085 I2C Driver

This driver communicates with the BNO085 9-axis IMU to read:
- Orientation (quaternion)
- Angular velocity (gyroscope)
- Linear acceleration (accelerometer)
- Calibration status

The BNO085 has built-in sensor fusion that provides
high-quality orientation estimates including stable heading.

Hardware Notes:
- VIN must be 5V (board has onboard 3.3V regulator)
- SDA/SCL are 3.3V level (safe for Pi)
- RST on GPIO 10, INT on GPIO 26 (optional)

Requires: pip install adafruit-circuitpython-bno08x
"""

from dataclasses import dataclass
from typing import Tuple, Optional, Dict, Any
import time
import logging

logger = logging.getLogger(__name__)


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class IMUReading:
    """Complete IMU reading from BNO085."""
    quaternion: Tuple[float, float, float, float]  # w, x, y, z
    angular_velocity: Tuple[float, float, float]   # x, y, z in rad/s
    linear_acceleration: Tuple[float, float, float]  # x, y, z in m/s^2
    calibration_status: Tuple[int, int, int, int]  # sys, gyro, accel, mag (0-3)
    timestamp_us: int  # Microseconds since device boot


@dataclass
class CalibrationStatus:
    """Calibration status for each sensor."""
    system: int      # 0-3, 3 = fully calibrated
    gyroscope: int   # 0-3
    accelerometer: int  # 0-3
    magnetometer: int   # 0-3
    
    @property
    def is_calibrated(self) -> bool:
        """Check if all sensors are adequately calibrated (>= 2)."""
        return all(v >= 2 for v in [self.system, self.gyroscope, 
                                     self.accelerometer, self.magnetometer])


# =============================================================================
# BNO085 Driver (Adafruit Library)
# =============================================================================

class BNO085Driver:
    """
    Driver for BNO085 9-axis IMU via I2C using Adafruit library.
    
    Uses adafruit_bno08x which handles the SHTP protocol.
    
    IMPORTANT: The BNO085 board VIN must be connected to 5V, not 3.3V.
    The board has an onboard voltage regulator. At 3.3V the internal
    sensor fusion processor doesn't have enough power and SHTP 
    enable_feature commands will fail intermittently.
    
    Usage:
        driver = BNO085Driver(bus=1, address=0x4A)
        if driver.initialize():
            while True:
                quat = driver.read_quaternion()
                print(f"Quaternion: {quat}")
    
    Hardware Connection (Raspberry Pi):
        - VIN -> 5V (Pin 2 or 4) — NOT 3.3V!
        - GND -> GND (Pin 6)
        - SDA -> GPIO 2 (Pin 3)
        - SCL -> GPIO 3 (Pin 5)
        - RST -> GPIO 10 (optional)
        - INT -> GPIO 26 (optional)
    """
    
    DEFAULT_ADDRESS = 0x4A
    ALT_ADDRESS = 0x4B
    
    def __init__(
        self, 
        bus: int = 1, 
        address: int = DEFAULT_ADDRESS,
        i2c_instance=None,
    ):
        self._bus_num = bus
        self._address = address
        self._i2c_instance = i2c_instance
        self._bno = None
        self._initialized = False
        
        # Cached readings
        self._quaternion: Optional[Tuple[float, float, float, float]] = None
        self._gyro: Optional[Tuple[float, float, float]] = None
        self._accel: Optional[Tuple[float, float, float]] = None
        
        # Statistics
        self._read_count = 0
        self._error_count = 0
        
    @property
    def address(self) -> int:
        return self._address
    
    @property
    def is_initialized(self) -> bool:
        return self._initialized
        
    def initialize(self) -> bool:
        """Initialize the BNO085 using Adafruit library.
        
        Sequence: create BNO08X_I2C (triggers soft reset), wait for
        SHTP boot, then enable sensor reports with retries.
        """
        try:
            import board
            import busio
            from adafruit_bno08x.i2c import BNO08X_I2C
            from adafruit_bno08x import (
                BNO_REPORT_ROTATION_VECTOR,
                BNO_REPORT_GYROSCOPE,
                BNO_REPORT_ACCELEROMETER,
            )
            
            # Create I2C connection
            if self._i2c_instance is not None:
                i2c = self._i2c_instance
            else:
                i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
            
            # Create BNO085 (constructor performs soft reset via SHTP)
            self._bno = BNO08X_I2C(i2c, address=self._address)
            time.sleep(2.0)  # Wait for SHTP boot after soft reset
            
            # Enable sensor reports with retry logic
            features = [
                (BNO_REPORT_ROTATION_VECTOR, "Rotation Vector"),
                (BNO_REPORT_GYROSCOPE, "Gyroscope"),
                (BNO_REPORT_ACCELEROMETER, "Accelerometer"),
            ]
            
            for feature_id, name in features:
                for attempt in range(5):
                    try:
                        self._bno.enable_feature(feature_id, 10)
                        logger.info(f"Enabled {name}")
                        break
                    except Exception as e:
                        logger.warning(f"Attempt {attempt+1}/5 to enable {name} failed: {e}")
                        time.sleep(1.0)
                else:
                    logger.error(f"Failed to enable {name} after 5 attempts")
                    return False
            
            self._initialized = True
            logger.info(f"BNO085 initialized at address 0x{self._address:02X}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize BNO085: {e}")
            self._initialized = False
            return False
    
    def read_quaternion(self) -> Tuple[float, float, float, float]:
        """Read orientation as quaternion (w, x, y, z) in ENU frame."""
        if not self._initialized or self._bno is None:
            return (1.0, 0.0, 0.0, 0.0)
        
        try:
            quat = self._bno.quaternion
            if quat is not None and quat[0] is not None:
                # Adafruit returns (i, j, k, real) in NED frame
                # Reorder to (w, x, y, z)
                w_ned = quat[3]
                x_ned = quat[0]
                y_ned = quat[1]
                z_ned = quat[2]
                
                # Convert NED -> ENU: swap x<->y, negate z
                self._quaternion = (w_ned, y_ned, x_ned, -z_ned)
                self._read_count += 1
                return self._quaternion
        except Exception as e:
            logger.debug(f"Quaternion read error: {e}")
            self._error_count += 1
        
        if self._quaternion is not None:
            return self._quaternion
        return (1.0, 0.0, 0.0, 0.0)
    
    def read_gyro(self) -> Tuple[float, float, float]:
        """Read angular velocity (x, y, z) in rad/s, ENU frame."""
        if not self._initialized or self._bno is None:
            return (0.0, 0.0, 0.0)
        
        try:
            gyro = self._bno.gyro
            if gyro is not None and gyro[0] is not None:
                # Convert NED -> ENU: swap x<->y, negate z
                self._gyro = (gyro[1], gyro[0], -gyro[2])
                return self._gyro
        except Exception as e:
            logger.debug(f"Gyro read error: {e}")
            self._error_count += 1
        
        if self._gyro is not None:
            return self._gyro
        return (0.0, 0.0, 0.0)
    
    def read_accel(self) -> Tuple[float, float, float]:
        """Read linear acceleration (x, y, z) in m/s^2, ENU frame."""
        if not self._initialized or self._bno is None:
            return (0.0, 0.0, 9.81)
        
        try:
            accel = self._bno.acceleration
            if accel is not None and accel[0] is not None:
                # Convert NED -> ENU: swap x<->y, negate z
                self._accel = (accel[1], accel[0], -accel[2])
                return self._accel
        except Exception as e:
            logger.debug(f"Accel read error: {e}")
            self._error_count += 1
        
        if self._accel is not None:
            return self._accel
        return (0.0, 0.0, 9.81)
    
    def get_calibration_status(self) -> Tuple[int, int, int, int]:
        """Get calibration status (system, gyro, accel, mag). Each 0-3."""
        if not self._initialized or self._bno is None:
            return (0, 0, 0, 0)
        
        try:
            cal = self._bno.calibration_status
            if cal is not None:
                return (cal, cal, cal, cal)
        except Exception:
            pass
        return (0, 0, 0, 0)
    
    def is_calibrated(self) -> bool:
        """Check if sensors are adequately calibrated."""
        status = self.get_calibration_status()
        return all(v >= 2 for v in status)
    
    def read_all(self) -> Optional[IMUReading]:
        """Read all sensor data at once."""
        if not self._initialized:
            return None
        
        quat = self.read_quaternion()
        gyro = self.read_gyro()
        accel = self.read_accel()
        cal = self.get_calibration_status()
        
        return IMUReading(
            quaternion=quat,
            angular_velocity=gyro,
            linear_acceleration=accel,
            calibration_status=cal,
            timestamp_us=int(time.time() * 1_000_000),
        )
    
    def get_status(self) -> Dict[str, Any]:
        """Get driver status for diagnostics."""
        return {
            "initialized": self._initialized,
            "address": f"0x{self._address:02X}",
            "read_count": self._read_count,
            "error_count": self._error_count,
        }
    
    def close(self) -> None:
        """Close connection."""
        self._bno = None
        self._initialized = False
    
    def __enter__(self):
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
