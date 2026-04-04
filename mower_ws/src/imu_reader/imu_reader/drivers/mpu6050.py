"""
MPU-6050 IMU Driver via I2C.

Story 1.1: MPU-6050 I2C Driver

This driver communicates with the MPU-6050 6-axis IMU to read:
- Linear acceleration (accelerometer)
- Angular velocity (gyroscope)
- Temperature

The MPU-6050 is a 6-DOF sensor (no magnetometer), used for prototype testing
before BNO085 is integrated into the PCB.
"""

from dataclasses import dataclass
from typing import Tuple, Optional, Dict, Any
import struct
import time
import math
import logging

logger = logging.getLogger(__name__)


# =============================================================================
# MPU-6050 Register Definitions
# =============================================================================

# I2C Address
MPU6050_DEFAULT_ADDRESS = 0x68  # AD0 to GND
MPU6050_ALT_ADDRESS = 0x69      # AD0 to VCC

# Register addresses
REG_PWR_MGMT_1 = 0x6B
REG_PWR_MGMT_2 = 0x6C
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_TEMP_OUT_H = 0x41
REG_GYRO_XOUT_H = 0x43
REG_WHO_AM_I = 0x75

# Configuration values
PWR_MGMT_1_RESET = 0x80
PWR_MGMT_1_WAKE = 0x00
PWR_MGMT_1_CLKSEL_PLL = 0x01  # PLL with X-axis gyro reference

# Accelerometer sensitivity (LSB/g)
ACCEL_SENSITIVITY = {
    0: 16384.0,  # ±2g
    1: 8192.0,   # ±4g
    2: 4096.0,   # ±8g
    3: 2048.0,   # ±16g
}

# Gyroscope sensitivity (LSB/(°/s))
GYRO_SENSITIVITY = {
    0: 131.0,    # ±250°/s
    1: 65.5,     # ±500°/s
    2: 32.8,     # ±1000°/s
    3: 16.4,     # ±2000°/s
}

# Gravity constant
GRAVITY = 9.80665  # m/s²


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class MPU6050Reading:
    """Complete reading from MPU-6050."""
    accel: Tuple[float, float, float]    # x, y, z in m/s²
    gyro: Tuple[float, float, float]     # x, y, z in rad/s
    temperature: float                    # Celsius
    timestamp_us: int                     # Microseconds


# =============================================================================
# MPU-6050 Driver
# =============================================================================

class MPU6050Driver:
    """
    Driver for MPU-6050 6-axis IMU via I2C.
    
    The MPU-6050 is a 6-DOF sensor with accelerometer and gyroscope.
    It does NOT have a magnetometer, so absolute heading cannot be determined.
    For this project, BNO085 magnetometer provides heading.
    
    Usage:
        driver = MPU6050Driver(bus=1, address=0x68)
        if driver.initialize():
            while True:
                reading = driver.read_all()
                if reading:
                    print(f"Accel: {reading.accel}")
                    print(f"Gyro: {reading.gyro}")
    
    Hardware Connection (Raspberry Pi 5):
        - SDA -> GPIO 2 (Pin 3)
        - SCL -> GPIO 3 (Pin 5)
        - VCC -> 3.3V (Pin 1)
        - GND -> GND (Pin 6)
        - AD0 -> GND (for address 0x68)
    """
    
    def __init__(
        self,
        bus: int = 1,
        address: int = MPU6050_DEFAULT_ADDRESS,
        i2c_instance=None  # For testing with mock I2C
    ):
        """
        Initialize MPU-6050 driver.
        
        Args:
            bus: I2C bus number (1 for Raspberry Pi)
            address: I2C address (0x68 default, 0x69 with AD0 high)
            i2c_instance: Optional mock I2C instance for testing
        """
        self._bus_num = bus
        self._address = address
        self._i2c = i2c_instance
        self._initialized = False
        
        # Sensitivity settings
        self._accel_range = 0  # ±2g default
        self._gyro_range = 0   # ±250°/s default
        self._accel_scale = ACCEL_SENSITIVITY[0]
        self._gyro_scale = GYRO_SENSITIVITY[0]
        
        # Calibration offsets (can be set after calibration)
        self._accel_offset = (0.0, 0.0, 0.0)
        self._gyro_offset = (0.0, 0.0, 0.0)
        
        # Statistics
        self._read_count = 0
        self._error_count = 0
    
    @property
    def address(self) -> int:
        """Get I2C address."""
        return self._address
    
    @property
    def is_initialized(self) -> bool:
        """Check if driver is initialized."""
        return self._initialized
    
    def initialize(
        self,
        accel_range: int = 0,  # ±2g
        gyro_range: int = 0,   # ±250°/s
        dlpf_mode: int = 3     # ~44Hz bandwidth
    ) -> bool:
        """
        Initialize the MPU-6050 and configure sensors.
        
        Args:
            accel_range: Accelerometer range (0=±2g, 1=±4g, 2=±8g, 3=±16g)
            gyro_range: Gyroscope range (0=±250, 1=±500, 2=±1000, 3=±2000 °/s)
            dlpf_mode: Digital low-pass filter mode (0-6)
            
        Returns:
            True if initialization successful, False otherwise.
        """
        try:
            # Create I2C connection if not provided (mock)
            if self._i2c is None:
                import smbus2
                self._i2c = smbus2.SMBus(self._bus_num)
            
            # Check device identity
            # MPU-6050 = 0x68, MPU-6500 = 0x70, MPU-9250 = 0x71/0x73
            who_am_i = self._read_byte(REG_WHO_AM_I)
            valid_ids = (0x68, 0x70, 0x71, 0x73)
            if who_am_i not in valid_ids:
                logger.error(f"MPU WHO_AM_I mismatch: got 0x{who_am_i:02X}, expected one of {[hex(x) for x in valid_ids]}")
                return False
            logger.info(f"Detected IMU with WHO_AM_I = 0x{who_am_i:02X}")
            
            # Reset device
            self._write_byte(REG_PWR_MGMT_1, PWR_MGMT_1_RESET)
            time.sleep(0.1)  # Wait for reset
            
            # Wake up and set clock source to PLL with X-axis gyro
            self._write_byte(REG_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_PLL)
            time.sleep(0.01)
            
            # Configure sample rate divider (1kHz / (1 + div) = sample rate)
            # For 100Hz: div = 9
            self._write_byte(REG_SMPLRT_DIV, 9)
            
            # Configure DLPF (Digital Low Pass Filter)
            self._write_byte(REG_CONFIG, dlpf_mode & 0x07)
            
            # Configure accelerometer range
            self._accel_range = accel_range & 0x03
            self._accel_scale = ACCEL_SENSITIVITY[self._accel_range]
            self._write_byte(REG_ACCEL_CONFIG, self._accel_range << 3)
            
            # Configure gyroscope range
            self._gyro_range = gyro_range & 0x03
            self._gyro_scale = GYRO_SENSITIVITY[self._gyro_range]
            self._write_byte(REG_GYRO_CONFIG, self._gyro_range << 3)
            
            self._initialized = True
            logger.info(f"MPU-6050 initialized at address 0x{self._address:02X} "
                       f"(accel=±{2 << self._accel_range}g, gyro=±{250 << self._gyro_range}°/s)")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize MPU-6050: {e}")
            return False
    
    def _write_byte(self, reg: int, value: int) -> None:
        """Write a byte to a register."""
        self._i2c.write_byte_data(self._address, reg, value)
    
    def _read_byte(self, reg: int) -> int:
        """Read a byte from a register."""
        return self._i2c.read_byte_data(self._address, reg)
    
    def _read_bytes(self, reg: int, length: int) -> bytes:
        """Read multiple bytes from registers."""
        return bytes(self._i2c.read_i2c_block_data(self._address, reg, length))
    
    def read_accel_raw(self) -> Tuple[int, int, int]:
        """Read raw accelerometer values."""
        data = self._read_bytes(REG_ACCEL_XOUT_H, 6)
        x = struct.unpack('>h', data[0:2])[0]
        y = struct.unpack('>h', data[2:4])[0]
        z = struct.unpack('>h', data[4:6])[0]
        return (x, y, z)
    
    def read_gyro_raw(self) -> Tuple[int, int, int]:
        """Read raw gyroscope values."""
        data = self._read_bytes(REG_GYRO_XOUT_H, 6)
        x = struct.unpack('>h', data[0:2])[0]
        y = struct.unpack('>h', data[2:4])[0]
        z = struct.unpack('>h', data[4:6])[0]
        return (x, y, z)
    
    def read_temp_raw(self) -> int:
        """Read raw temperature value."""
        data = self._read_bytes(REG_TEMP_OUT_H, 2)
        return struct.unpack('>h', data)[0]
    
    def read_accel(self) -> Tuple[float, float, float]:
        """Read accelerometer in m/s²."""
        if not self._initialized:
            return (0.0, 0.0, GRAVITY)
        
        try:
            raw = self.read_accel_raw()
            x = (raw[0] / self._accel_scale) * GRAVITY - self._accel_offset[0]
            y = (raw[1] / self._accel_scale) * GRAVITY - self._accel_offset[1]
            z = (raw[2] / self._accel_scale) * GRAVITY - self._accel_offset[2]
            return (x, y, z)
        except Exception as e:
            logger.debug(f"Accel read error: {e}")
            self._error_count += 1
            return (0.0, 0.0, GRAVITY)
    
    def read_gyro(self) -> Tuple[float, float, float]:
        """Read gyroscope in rad/s."""
        if not self._initialized:
            return (0.0, 0.0, 0.0)
        
        try:
            raw = self.read_gyro_raw()
            deg_to_rad = math.pi / 180.0
            x = (raw[0] / self._gyro_scale) * deg_to_rad - self._gyro_offset[0]
            y = (raw[1] / self._gyro_scale) * deg_to_rad - self._gyro_offset[1]
            z = (raw[2] / self._gyro_scale) * deg_to_rad - self._gyro_offset[2]
            return (x, y, z)
        except Exception as e:
            logger.debug(f"Gyro read error: {e}")
            self._error_count += 1
            return (0.0, 0.0, 0.0)
    
    def read_temperature(self) -> float:
        """Read temperature in Celsius."""
        if not self._initialized:
            return 25.0
        try:
            raw = self.read_temp_raw()
            return (raw / 340.0) + 36.53
        except Exception as e:
            logger.debug(f"Temp read error: {e}")
            return 25.0
    
    def read_all(self) -> Optional[MPU6050Reading]:
        """Read all sensor data in one I2C transaction."""
        if not self._initialized:
            return None
        
        try:
            data = self._read_bytes(REG_ACCEL_XOUT_H, 14)
            
            ax_raw = struct.unpack('>h', data[0:2])[0]
            ay_raw = struct.unpack('>h', data[2:4])[0]
            az_raw = struct.unpack('>h', data[4:6])[0]
            
            ax = (ax_raw / self._accel_scale) * GRAVITY - self._accel_offset[0]
            ay = (ay_raw / self._accel_scale) * GRAVITY - self._accel_offset[1]
            az = (az_raw / self._accel_scale) * GRAVITY - self._accel_offset[2]
            
            temp_raw = struct.unpack('>h', data[6:8])[0]
            temperature = (temp_raw / 340.0) + 36.53
            
            gx_raw = struct.unpack('>h', data[8:10])[0]
            gy_raw = struct.unpack('>h', data[10:12])[0]
            gz_raw = struct.unpack('>h', data[12:14])[0]
            
            deg_to_rad = math.pi / 180.0
            gx = (gx_raw / self._gyro_scale) * deg_to_rad - self._gyro_offset[0]
            gy = (gy_raw / self._gyro_scale) * deg_to_rad - self._gyro_offset[1]
            gz = (gz_raw / self._gyro_scale) * deg_to_rad - self._gyro_offset[2]
            
            self._read_count += 1
            
            return MPU6050Reading(
                accel=(ax, ay, az),
                gyro=(gx, gy, gz),
                temperature=temperature,
                timestamp_us=int(time.time() * 1_000_000),
            )
        except Exception as e:
            logger.debug(f"Read all error: {e}")
            self._error_count += 1
            return None
    
    def set_accel_offset(self, x: float, y: float, z: float) -> None:
        """Set accelerometer calibration offsets (in m/s²)."""
        self._accel_offset = (x, y, z)
    
    def set_gyro_offset(self, x: float, y: float, z: float) -> None:
        """Set gyroscope calibration offsets (in rad/s)."""
        self._gyro_offset = (x, y, z)
    
    def calibrate_gyro(self, samples: int = 100, delay: float = 0.01) -> Tuple[float, float, float]:
        """Calibrate gyroscope by measuring offset at rest."""
        if not self._initialized:
            return (0.0, 0.0, 0.0)
        
        logger.info(f"Calibrating gyro with {samples} samples...")
        
        old_offset = self._gyro_offset
        self._gyro_offset = (0.0, 0.0, 0.0)
        
        sum_x, sum_y, sum_z = 0.0, 0.0, 0.0
        
        for _ in range(samples):
            gyro = self.read_gyro()
            sum_x += gyro[0]
            sum_y += gyro[1]
            sum_z += gyro[2]
            time.sleep(delay)
        
        offset = (sum_x / samples, sum_y / samples, sum_z / samples)
        self._gyro_offset = offset
        
        logger.info(f"Gyro calibration complete: offset = {offset}")
        return offset
    
    def is_connected(self) -> bool:
        """Check if MPU-6050 responds on I2C bus."""
        try:
            who_am_i = self._read_byte(REG_WHO_AM_I)
            return who_am_i == 0x68
        except Exception:
            return False
    
    def get_status(self) -> Dict[str, Any]:
        """Get driver status for diagnostics."""
        return {
            "initialized": self._initialized,
            "address": f"0x{self._address:02X}",
            "accel_range": f"±{2 << self._accel_range}g",
            "gyro_range": f"±{250 << self._gyro_range}°/s",
            "read_count": self._read_count,
            "error_count": self._error_count,
            "temperature": self.read_temperature() if self._initialized else None,
        }
    
    def close(self) -> None:
        """Close I2C connection."""
        if self._i2c is not None:
            try:
                self._i2c.close()
            except Exception:
                pass
            self._i2c = None
        self._initialized = False
    
    def __enter__(self):
        """Context manager entry."""
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
