"""
BNO085 IMU Driver via I2C.

Story 1.1: BNO085 I2C Driver

This driver communicates with the BNO085 9-axis IMU to read:
- Orientation (quaternion)
- Angular velocity (gyroscope)
- Linear acceleration (accelerometer)
- Calibration status

The BNO085 has built-in sensor fusion (NDOF mode) that provides
high-quality orientation estimates.
"""

from dataclasses import dataclass
from typing import Tuple, Optional
import struct
import time
import logging

logger = logging.getLogger(__name__)


# =============================================================================
# BNO085 Register Definitions
# =============================================================================

# I2C Address (default, can be 0x4B with address pin high)
BNO085_DEFAULT_ADDRESS = 0x4A
BNO085_ALT_ADDRESS = 0x4B

# Report IDs for SHTP (Sensor Hub Transport Protocol)
SHTP_REPORT_PRODUCT_ID = 0xF8
SHTP_REPORT_SET_FEATURE = 0xFD

# Sensor Report IDs
SENSOR_REPORTID_ROTATION_VECTOR = 0x05
SENSOR_REPORTID_GYROSCOPE = 0x02
SENSOR_REPORTID_ACCELEROMETER = 0x01
SENSOR_REPORTID_GAME_ROTATION_VECTOR = 0x08

# Channels
CHANNEL_COMMAND = 0
CHANNEL_EXECUTABLE = 1
CHANNEL_SENSOR_HUB = 2
CHANNEL_INPUT_SENSOR = 3
CHANNEL_WAKE_INPUT = 4
CHANNEL_GYRO_ROTATION = 5


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
# BNO085 Driver
# =============================================================================

class BNO085Driver:
    """
    Driver for BNO085 9-axis IMU via I2C.
    
    The BNO085 uses the Sensor Hub Transport Protocol (SHTP) which is
    more complex than simple register reads. This driver handles the
    protocol to enable sensor reports and read fused orientation data.
    
    Usage:
        driver = BNO085Driver(bus=1, address=0x4A)
        if driver.initialize():
            while True:
                reading = driver.read_all()
                if reading:
                    print(f"Quaternion: {reading.quaternion}")
    
    Hardware Connection (Raspberry Pi 5):
        - SDA -> GPIO 2 (Pin 3)
        - SCL -> GPIO 3 (Pin 5)
        - VCC -> 3.3V (Pin 1)
        - GND -> GND (Pin 6)
    """
    
    def __init__(
        self, 
        bus: int = 1, 
        address: int = BNO085_DEFAULT_ADDRESS,
        i2c_instance=None  # For testing with mock I2C
    ):
        """
        Initialize BNO085 driver.
        
        Args:
            bus: I2C bus number (1 for Raspberry Pi)
            address: I2C address (0x4A default, 0x4B alternate)
            i2c_instance: Optional mock I2C instance for testing
        """
        self._bus_num = bus
        self._address = address
        self._i2c = i2c_instance
        self._initialized = False
        
        # Cached readings
        self._quaternion: Optional[Tuple[float, float, float, float]] = None
        self._gyro: Optional[Tuple[float, float, float]] = None
        self._accel: Optional[Tuple[float, float, float]] = None
        self._calibration: Optional[CalibrationStatus] = None
        
        # Sequence numbers for SHTP
        self._sequence_number = [0, 0, 0, 0, 0, 0]
        
    @property
    def address(self) -> int:
        """Get I2C address."""
        return self._address
    
    @property
    def is_initialized(self) -> bool:
        """Check if driver is initialized."""
        return self._initialized
        
    def initialize(self) -> bool:
        """
        Initialize the BNO085 and configure sensor reports.
        
        Returns:
            True if initialization successful, False otherwise.
        """
        try:
            # Create I2C connection if not provided (mock)
            if self._i2c is None:
                import smbus2
                self._i2c = smbus2.SMBus(self._bus_num)
            
            # Soft reset the device
            self._soft_reset()
            time.sleep(0.1)  # Wait for reset
            
            # Wait for device to be ready
            if not self._wait_for_device():
                logger.error("BNO085 not responding after reset")
                return False
            
            # Enable rotation vector (quaternion) reports at 100Hz
            self._enable_report(SENSOR_REPORTID_ROTATION_VECTOR, 10000)  # 10ms = 100Hz
            
            # Enable gyroscope reports at 100Hz
            self._enable_report(SENSOR_REPORTID_GYROSCOPE, 10000)
            
            # Enable accelerometer reports at 100Hz
            self._enable_report(SENSOR_REPORTID_ACCELEROMETER, 10000)
            
            self._initialized = True
            logger.info(f"BNO085 initialized at address 0x{self._address:02X}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize BNO085: {e}")
            return False
    
    def _soft_reset(self) -> None:
        """Send soft reset command to BNO085."""
        # SHTP soft reset command
        reset_cmd = [0x01]  # Reset command
        self._send_packet(CHANNEL_EXECUTABLE, reset_cmd)
    
    def _wait_for_device(self, timeout: float = 1.0) -> bool:
        """Wait for device to be ready after reset."""
        start = time.time()
        while time.time() - start < timeout:
            try:
                # Try to read advertisement packet
                data = self._receive_packet()
                if data is not None:
                    return True
            except Exception:
                pass
            time.sleep(0.01)
        return False
    
    def _enable_report(self, report_id: int, interval_us: int) -> None:
        """
        Enable a sensor report at specified interval.
        
        Args:
            report_id: Sensor report ID
            interval_us: Report interval in microseconds
        """
        # Set Feature Command
        cmd = [
            SHTP_REPORT_SET_FEATURE,
            report_id,
            0,  # Feature flags
            0,  # Change sensitivity (LSB)
            0,  # Change sensitivity (MSB)
            interval_us & 0xFF,
            (interval_us >> 8) & 0xFF,
            (interval_us >> 16) & 0xFF,
            (interval_us >> 24) & 0xFF,
            0, 0, 0, 0,  # Batch interval
            0, 0, 0, 0,  # Sensor specific config
        ]
        self._send_packet(CHANNEL_SENSOR_HUB, cmd)
    
    def _send_packet(self, channel: int, data: list) -> None:
        """Send SHTP packet to BNO085."""
        # Build SHTP header
        length = len(data) + 4  # 4 byte header
        packet = [
            length & 0xFF,
            (length >> 8) & 0xFF,
            channel,
            self._sequence_number[channel],
        ]
        packet.extend(data)
        
        # Increment sequence number
        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256
        
        # Send via I2C
        try:
            self._i2c.write_i2c_block_data(self._address, 0, packet[:32])
            if len(packet) > 32:
                # Handle larger packets
                for i in range(32, len(packet), 32):
                    self._i2c.write_i2c_block_data(self._address, 0, packet[i:i+32])
        except Exception as e:
            logger.debug(f"I2C write error: {e}")
    
    def _receive_packet(self) -> Optional[bytes]:
        """Receive SHTP packet from BNO085."""
        try:
            # Read header first
            header = self._i2c.read_i2c_block_data(self._address, 0, 4)
            
            length = (header[1] << 8) | header[0]
            length &= 0x7FFF  # Remove continuation bit
            
            if length <= 4:
                return None
            
            # Read payload
            payload_len = min(length - 4, 128)  # Cap at 128 bytes
            data = self._i2c.read_i2c_block_data(self._address, 0, payload_len + 4)
            
            return bytes(data[4:])
            
        except Exception as e:
            logger.debug(f"I2C read error: {e}")
            return None
    
    def read_quaternion(self) -> Tuple[float, float, float, float]:
        """
        Read orientation as quaternion (w, x, y, z).
        
        Returns:
            Tuple of (w, x, y, z) quaternion components.
            Returns (1, 0, 0, 0) identity if no data available.
        """
        self._process_available_packets()
        
        if self._quaternion is not None:
            return self._quaternion
        return (1.0, 0.0, 0.0, 0.0)  # Identity quaternion
    
    def read_gyro(self) -> Tuple[float, float, float]:
        """
        Read angular velocity (x, y, z) in rad/s.
        
        Returns:
            Tuple of (x, y, z) angular velocity in rad/s.
        """
        self._process_available_packets()
        
        if self._gyro is not None:
            return self._gyro
        return (0.0, 0.0, 0.0)
    
    def read_accel(self) -> Tuple[float, float, float]:
        """
        Read linear acceleration (x, y, z) in m/s^2.
        
        Returns:
            Tuple of (x, y, z) acceleration in m/s^2.
        """
        self._process_available_packets()
        
        if self._accel is not None:
            return self._accel
        return (0.0, 0.0, 9.81)  # Default: gravity only
    
    def get_calibration_status(self) -> Tuple[int, int, int, int]:
        """
        Get calibration status for all sensors.
        
        Returns:
            Tuple of (system, gyro, accel, mag) calibration levels (0-3).
            3 = fully calibrated, 0 = not calibrated.
        """
        self._process_available_packets()
        
        if self._calibration is not None:
            return (
                self._calibration.system,
                self._calibration.gyroscope,
                self._calibration.accelerometer,
                self._calibration.magnetometer,
            )
        return (0, 0, 0, 0)
    
    def is_calibrated(self) -> bool:
        """
        Check if all sensors are adequately calibrated.
        
        Returns:
            True if all calibration values >= 2.
        """
        status = self.get_calibration_status()
        return all(v >= 2 for v in status)
    
    def read_all(self) -> Optional[IMUReading]:
        """
        Read all sensor data at once.
        
        Returns:
            IMUReading with all sensor data, or None if not available.
        """
        self._process_available_packets()
        
        if self._quaternion is None:
            return None
        
        return IMUReading(
            quaternion=self._quaternion,
            angular_velocity=self._gyro or (0.0, 0.0, 0.0),
            linear_acceleration=self._accel or (0.0, 0.0, 9.81),
            calibration_status=self.get_calibration_status(),
            timestamp_us=int(time.time() * 1_000_000),
        )
    
    def _process_available_packets(self) -> None:
        """Process all available packets from BNO085."""
        if not self._initialized:
            return
            
        # Read up to 10 packets
        for _ in range(10):
            packet = self._receive_packet()
            if packet is None or len(packet) < 5:
                break
            self._parse_sensor_report(packet)
    
    def _parse_sensor_report(self, data: bytes) -> None:
        """Parse a sensor report packet."""
        if len(data) < 5:
            return
            
        report_id = data[0]
        
        if report_id == SENSOR_REPORTID_ROTATION_VECTOR:
            self._parse_rotation_vector(data)
        elif report_id == SENSOR_REPORTID_GYROSCOPE:
            self._parse_gyroscope(data)
        elif report_id == SENSOR_REPORTID_ACCELEROMETER:
            self._parse_accelerometer(data)
    
    def _parse_rotation_vector(self, data: bytes) -> None:
        """Parse rotation vector (quaternion) report."""
        if len(data) < 14:
            return
        
        # Q point is 14 for rotation vector
        q_point = 14
        scale = 1.0 / (1 << q_point)
        
        # Parse quaternion components (i, j, k, real)
        i = struct.unpack('<h', data[4:6])[0] * scale
        j = struct.unpack('<h', data[6:8])[0] * scale
        k = struct.unpack('<h', data[8:10])[0] * scale
        real = struct.unpack('<h', data[10:12])[0] * scale
        
        # BNO085 reports as (i, j, k, real), convert to (w, x, y, z)
        self._quaternion = (real, i, j, k)
        
        # Accuracy is in data[12:14] if needed
        # accuracy = struct.unpack('<h', data[12:14])[0] * (1.0 / (1 << 12))
    
    def _parse_gyroscope(self, data: bytes) -> None:
        """Parse gyroscope report."""
        if len(data) < 10:
            return
        
        # Q point is 9 for gyroscope (rad/s)
        q_point = 9
        scale = 1.0 / (1 << q_point)
        
        x = struct.unpack('<h', data[4:6])[0] * scale
        y = struct.unpack('<h', data[6:8])[0] * scale
        z = struct.unpack('<h', data[8:10])[0] * scale
        
        self._gyro = (x, y, z)
    
    def _parse_accelerometer(self, data: bytes) -> None:
        """Parse accelerometer report."""
        if len(data) < 10:
            return
        
        # Q point is 8 for accelerometer (m/s^2)
        q_point = 8
        scale = 1.0 / (1 << q_point)
        
        x = struct.unpack('<h', data[4:6])[0] * scale
        y = struct.unpack('<h', data[6:8])[0] * scale
        z = struct.unpack('<h', data[8:10])[0] * scale
        
        self._accel = (x, y, z)
    
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
