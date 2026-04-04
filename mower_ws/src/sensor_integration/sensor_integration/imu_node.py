"""
IMU ROS 2 Node - Supports MPU-6050 and BNO085.

Story 1.5: IMU ROS 2 Node
Story 1.6: IMU Diagnostics

Publishes IMU data as sensor_msgs/Imu messages.
Supports switching between MPU-6050 (prototype) and BNO085 (production)
via the 'imu_type' parameter.

Note: Yaw/heading comes from IMU magnetometer (BNO085) for this project.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header

import math
import time
from typing import Optional

from .drivers.imu_factory import IMUFactory, ImuDriver


class IMUNode(Node):
    """
    ROS 2 node for IMU (MPU-6050 or BNO085).
    
    Publishes:
        /imu/data (sensor_msgs/Imu): IMU data at ~100 Hz
        /diagnostics (DiagnosticArray): Health status
    
    Parameters:
        imu_type (str): IMU type - "mpu6050" or "bno085" (default: "mpu6050")
        i2c_bus (int): I2C bus number (default: 1)
        i2c_address (int): I2C address (default: auto based on type)
        publish_rate (float): Publishing rate in Hz (default: 100.0)
        frame_id (str): TF frame ID (default: "imu_link")
        madgwick_beta (float): Madgwick filter gain for MPU6050 (default: 0.1)
    """
    
    def __init__(self):
        super().__init__('imu_node')
        
        # Declare parameters
        self.declare_parameter('imu_type', 'mpu6050')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', -1)  # -1 = auto
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('madgwick_beta', 0.1)
        
        # Get parameters
        self._imu_type = self.get_parameter('imu_type').value
        self._bus = self.get_parameter('i2c_bus').value
        self._address = self.get_parameter('i2c_address').value
        self._rate = self.get_parameter('publish_rate').value
        self._frame_id = self.get_parameter('frame_id').value
        self._madgwick_beta = self.get_parameter('madgwick_beta').value
        
        # Auto-detect address based on IMU type
        if self._address == -1:
            self._address = IMUFactory.get_default_address(self._imu_type)
        
        # Create driver
        self._driver: Optional[ImuDriver] = None
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self._imu_pub = self.create_publisher(Imu, '/imu/data', sensor_qos)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Timers
        self._publish_timer = self.create_timer(1.0 / self._rate, self._publish_callback)
        self._diag_timer = self.create_timer(1.0, self._diagnostics_callback)
        
        # Statistics
        self._publish_count = 0
        self._error_count = 0
        self._last_publish_time = time.time()
        
        # Initialize driver
        self._init_driver()
        
        self.get_logger().info(
            f"IMU node started (type={self._imu_type}, bus={self._bus}, "
            f"addr=0x{self._address:02X}, rate={self._rate}Hz, frame={self._frame_id})"
        )
    
    def _init_driver(self) -> None:
        """Initialize IMU driver using factory."""
        try:
            self._driver = IMUFactory.create(
                imu_type=self._imu_type,
                bus=self._bus,
                address=self._address,
                madgwick_beta=self._madgwick_beta,
                sample_period=1.0 / self._rate
            )
            
            if self._driver.initialize():
                self.get_logger().info(f"{self._imu_type.upper()} initialized successfully")
            else:
                self.get_logger().error(f"Failed to initialize {self._imu_type.upper()}")
                self._driver = None
                
        except Exception as e:
            self.get_logger().error(f"IMU driver error: {e}")
            self._driver = None
    
    def _publish_callback(self) -> None:
        """Timer callback to publish IMU data."""
        if self._driver is None:
            # Try to reinitialize
            if self._error_count % 100 == 0:  # Every ~1 second at 100Hz
                self._init_driver()
            self._error_count += 1
            return
        
        try:
            # Read IMU data
            quaternion = self._driver.read_quaternion()
            gyro = self._driver.read_gyro()
            accel = self._driver.read_accel()
            
            # Create message
            msg = Imu()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            
            # Orientation (quaternion)
            msg.orientation.w = quaternion[0]
            msg.orientation.x = quaternion[1]
            msg.orientation.y = quaternion[2]
            msg.orientation.z = quaternion[3]
            
            # Orientation covariance (diagonal)
            orientation_cov = 0.0001  # rad^2
            msg.orientation_covariance = [
                orientation_cov, 0.0, 0.0,
                0.0, orientation_cov, 0.0,
                0.0, 0.0, orientation_cov
            ]
            
            # Angular velocity
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]
            
            # Angular velocity covariance
            gyro_cov = 0.001  # (rad/s)^2
            msg.angular_velocity_covariance = [
                gyro_cov, 0.0, 0.0,
                0.0, gyro_cov, 0.0,
                0.0, 0.0, gyro_cov
            ]
            
            # Linear acceleration
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]
            
            # Linear acceleration covariance
            accel_cov = 0.01  # (m/s^2)^2
            msg.linear_acceleration_covariance = [
                accel_cov, 0.0, 0.0,
                0.0, accel_cov, 0.0,
                0.0, 0.0, accel_cov
            ]
            
            # Publish
            self._imu_pub.publish(msg)
            self._publish_count += 1
            self._last_publish_time = time.time()
            
        except Exception as e:
            self.get_logger().debug(f"IMU read error: {e}")
            self._error_count += 1
    
    def _diagnostics_callback(self) -> None:
        """Timer callback to publish diagnostics."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = f"IMU: {self._imu_type.upper()}"
        status.hardware_id = f"{self._imu_type}_0x{self._address:02X}"
        
        if self._driver is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "Driver not initialized"
            status.values = [
                KeyValue(key="imu_type", value=self._imu_type),
                KeyValue(key="error_count", value=str(self._error_count)),
            ]
        else:
            # Get driver status
            driver_status = self._driver.get_status()
            
            # Determine health based on IMU type
            if self._imu_type.lower() == 'bno085':
                # BNO085: Check calibration
                if driver_status.get('is_calibrated', False):
                    status.level = DiagnosticStatus.OK
                    status.message = "Calibrated and running"
                else:
                    status.level = DiagnosticStatus.WARN
                    status.message = "Calibration needed"
            else:
                # MPU6050: Just check if initialized
                if driver_status.get('initialized', False):
                    status.level = DiagnosticStatus.OK
                    status.message = "Running (no calibration required)"
                else:
                    status.level = DiagnosticStatus.ERROR
                    status.message = "Not initialized"
            
            # Calculate publish rate
            actual_rate = self._publish_count
            self._publish_count = 0
            
            # Add key-values from driver status
            status.values = [
                KeyValue(key="imu_type", value=str(driver_status.get('imu_type', 'unknown'))),
                KeyValue(key="has_magnetometer", value=str(driver_status.get('has_magnetometer', False))),
                KeyValue(key="fusion_method", value=str(driver_status.get('fusion_method', 'none'))),
                KeyValue(key="publish_rate_hz", value=f"{actual_rate}"),
                KeyValue(key="error_count", value=str(self._error_count)),
            ]
            
            # Add BNO085-specific calibration info
            if 'calibration_system' in driver_status:
                status.values.extend([
                    KeyValue(key="calibration_system", value=str(driver_status.get('calibration_system', 0))),
                    KeyValue(key="calibration_gyro", value=str(driver_status.get('calibration_gyro', 0))),
                    KeyValue(key="calibration_accel", value=str(driver_status.get('calibration_accel', 0))),
                    KeyValue(key="calibration_mag", value=str(driver_status.get('calibration_mag', 0))),
                ])
            
            # Add MPU6050-specific info
            if 'temperature' in driver_status and driver_status['temperature'] is not None:
                status.values.append(
                    KeyValue(key="temperature_c", value=f"{driver_status['temperature']:.1f}")
                )
        
        msg.status.append(status)
        self._diag_pub.publish(msg)
    
    def destroy_node(self):
        """Clean up on shutdown."""
        if self._driver:
            self._driver.close()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = IMUNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
