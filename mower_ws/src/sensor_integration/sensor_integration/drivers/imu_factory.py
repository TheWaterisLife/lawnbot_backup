"""
IMU Driver Factory.

Story 1.3: IMU Driver Factory

Provides a factory to instantiate the correct IMU driver based on configuration.
Supports switching between MPU-6050 (prototype) and BNO085 (production) via
a single configuration parameter.
"""

from abc import ABC, abstractmethod
from typing import Tuple, Optional, Dict, Any
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class IMUType(Enum):
    """Supported IMU types."""
    MPU6050 = "mpu6050"
    BNO085 = "bno085"


class ImuDriver(ABC):
    """
    Abstract base class for IMU drivers.
    
    Defines the common interface that all IMU drivers must implement.
    This allows the IMU node to work with any IMU type without code changes.
    """
    
    @property
    @abstractmethod
    def is_initialized(self) -> bool:
        """Check if driver is initialized."""
        pass
    
    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the IMU sensor.
        
        Returns:
            True if initialization successful, False otherwise.
        """
        pass
    
    @abstractmethod
    def read_quaternion(self) -> Tuple[float, float, float, float]:
        """
        Read orientation as quaternion (w, x, y, z).
        
        Returns:
            Tuple of (w, x, y, z) quaternion components.
            For IMUs without built-in fusion (MPU6050), this may require
            external filtering (Madgwick).
        """
        pass
    
    @abstractmethod
    def read_gyro(self) -> Tuple[float, float, float]:
        """
        Read angular velocity (x, y, z) in rad/s.
        
        Returns:
            Tuple of (x, y, z) angular velocity in rad/s.
        """
        pass
    
    @abstractmethod
    def read_accel(self) -> Tuple[float, float, float]:
        """
        Read linear acceleration (x, y, z) in m/s².
        
        Returns:
            Tuple of (x, y, z) acceleration in m/s².
        """
        pass
    
    @abstractmethod
    def get_status(self) -> Dict[str, Any]:
        """
        Get driver status for diagnostics.
        
        Returns:
            Dictionary with status information.
        """
        pass
    
    @abstractmethod
    def close(self) -> None:
        """Close the driver and release resources."""
        pass


class MPU6050Wrapper(ImuDriver):
    """
    Wrapper for MPU6050Driver that conforms to ImuDriver interface.
    
    Includes Madgwick filter for orientation estimation.
    Note: Yaw from the filter will drift - use GPS heading externally.
    """
    
    def __init__(
        self,
        bus: int = 1,
        address: int = 0x68,
        i2c_instance=None,
        madgwick_beta: float = 0.1,
        sample_period: float = 0.01
    ):
        """
        Initialize MPU6050 wrapper with Madgwick filter.
        
        Args:
            bus: I2C bus number
            address: I2C address (0x68 or 0x69)
            i2c_instance: Optional mock I2C for testing
            madgwick_beta: Madgwick filter gain
            sample_period: Expected sample period in seconds
        """
        from .mpu6050 import MPU6050Driver
        from ..utils.filters import MadgwickFilter
        
        self._driver = MPU6050Driver(
            bus=bus,
            address=address,
            i2c_instance=i2c_instance
        )
        self._filter = MadgwickFilter(
            beta=madgwick_beta,
            sample_period=sample_period
        )
        self._sample_period = sample_period
        self._last_quaternion = (1.0, 0.0, 0.0, 0.0)
    
    @property
    def is_initialized(self) -> bool:
        return self._driver.is_initialized
    
    def initialize(self) -> bool:
        success = self._driver.initialize()
        if success:
            # Optionally calibrate gyro at startup
            # self._driver.calibrate_gyro(samples=100)
            pass
        return success
    
    def read_quaternion(self) -> Tuple[float, float, float, float]:
        """
        Read orientation using Madgwick filter.
        
        Note: Yaw will drift! Use get_roll_pitch_quaternion() with GPS heading
        for accurate orientation.
        """
        reading = self._driver.read_all()
        if reading is None:
            return self._last_quaternion
        
        # Update Madgwick filter
        self._filter.update(
            gyro=reading.gyro,
            accel=reading.accel,
            dt=self._sample_period
        )
        
        self._last_quaternion = self._filter.get_quaternion()
        return self._last_quaternion
    
    def get_roll_pitch_quaternion(self, external_yaw: float = 0.0) -> Tuple[float, float, float, float]:
        """
        Get quaternion with roll/pitch from filter and yaw from external source.
        
        This is the recommended method for use with GPS dual-antenna heading.
        
        Args:
            external_yaw: Yaw angle in radians from GPS HDT sentence
            
        Returns:
            Quaternion tuple (w, x, y, z) with accurate orientation.
        """
        # First update the filter
        reading = self._driver.read_all()
        if reading is not None:
            self._filter.update(
                gyro=reading.gyro,
                accel=reading.accel,
                dt=self._sample_period
            )
        
        return self._filter.get_quaternion_roll_pitch_only(external_yaw)
    
    def read_gyro(self) -> Tuple[float, float, float]:
        return self._driver.read_gyro()
    
    def read_accel(self) -> Tuple[float, float, float]:
        return self._driver.read_accel()
    
    def get_status(self) -> Dict[str, Any]:
        status = self._driver.get_status()
        status["imu_type"] = "mpu6050"
        status["has_magnetometer"] = False
        status["fusion_method"] = "madgwick_filter"
        return status
    
    def close(self) -> None:
        self._driver.close()


class BNO085Wrapper(ImuDriver):
    """
    Wrapper for BNO085Driver that conforms to ImuDriver interface.
    
    Uses built-in sensor fusion from BNO085 (NDOF mode).
    """
    
    def __init__(
        self,
        bus: int = 1,
        address: int = 0x4A,
        i2c_instance=None
    ):
        """
        Initialize BNO085 wrapper.
        
        Args:
            bus: I2C bus number
            address: I2C address (0x4A or 0x4B)
            i2c_instance: Optional mock I2C for testing
        """
        from .bno085 import BNO085Driver
        
        self._driver = BNO085Driver(
            bus=bus,
            address=address,
            i2c_instance=i2c_instance
        )
    
    @property
    def is_initialized(self) -> bool:
        return self._driver.is_initialized
    
    def initialize(self) -> bool:
        return self._driver.initialize()
    
    def read_quaternion(self) -> Tuple[float, float, float, float]:
        return self._driver.read_quaternion()
    
    def read_gyro(self) -> Tuple[float, float, float]:
        return self._driver.read_gyro()
    
    def read_accel(self) -> Tuple[float, float, float]:
        return self._driver.read_accel()
    
    def get_status(self) -> Dict[str, Any]:
        cal = self._driver.get_calibration_status()
        return {
            "imu_type": "bno085",
            "initialized": self._driver.is_initialized,
            "address": f"0x{self._driver.address:02X}",
            "has_magnetometer": True,
            "fusion_method": "built_in_ndof",
            "calibration_system": cal[0],
            "calibration_gyro": cal[1],
            "calibration_accel": cal[2],
            "calibration_mag": cal[3],
            "is_calibrated": self._driver.is_calibrated(),
        }
    
    def close(self) -> None:
        self._driver.close()


class IMUFactory:
    """
    Factory for creating IMU driver instances.
    
    Usage:
        # Create MPU6050 driver
        imu = IMUFactory.create("mpu6050", bus=1, address=0x68)
        
        # Create BNO085 driver
        imu = IMUFactory.create("bno085", bus=1, address=0x4A)
        
        # Use common interface
        if imu.initialize():
            quat = imu.read_quaternion()
            gyro = imu.read_gyro()
            accel = imu.read_accel()
    """
    
    @staticmethod
    def create(
        imu_type: str,
        bus: int = 1,
        address: Optional[int] = None,
        i2c_instance=None,
        **kwargs
    ) -> ImuDriver:
        """
        Create an IMU driver instance.
        
        Args:
            imu_type: IMU type string ("mpu6050" or "bno085")
            bus: I2C bus number (default: 1)
            address: I2C address (auto-detected based on type if None)
            i2c_instance: Optional mock I2C for testing
            **kwargs: Additional driver-specific arguments
            
        Returns:
            ImuDriver instance
            
        Raises:
            ValueError: If imu_type is not recognized
        """
        imu_type_lower = imu_type.lower().strip()
        
        if imu_type_lower == IMUType.MPU6050.value:
            # Default address for MPU6050
            addr = address if address is not None else 0x68
            
            return MPU6050Wrapper(
                bus=bus,
                address=addr,
                i2c_instance=i2c_instance,
                madgwick_beta=kwargs.get('madgwick_beta', 0.1),
                sample_period=kwargs.get('sample_period', 0.01)
            )
        
        elif imu_type_lower == IMUType.BNO085.value:
            # Default address for BNO085
            addr = address if address is not None else 0x4A
            
            return BNO085Wrapper(
                bus=bus,
                address=addr,
                i2c_instance=i2c_instance
            )
        
        else:
            raise ValueError(
                f"Unknown IMU type: '{imu_type}'. "
                f"Supported types: {[t.value for t in IMUType]}"
            )
    
    @staticmethod
    def get_supported_types() -> list:
        """Get list of supported IMU types."""
        return [t.value for t in IMUType]
    
    @staticmethod
    def get_default_address(imu_type: str) -> int:
        """Get default I2C address for an IMU type."""
        imu_type_lower = imu_type.lower().strip()
        
        if imu_type_lower == IMUType.MPU6050.value:
            return 0x68
        elif imu_type_lower == IMUType.BNO085.value:
            return 0x4A
        else:
            raise ValueError(f"Unknown IMU type: {imu_type}")
