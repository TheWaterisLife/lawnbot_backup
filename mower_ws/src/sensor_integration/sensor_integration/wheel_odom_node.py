"""
Wheel Odometry ROS 2 Node.

Story 3.3: Odometry ROS 2 Node

Publishes wheel odometry from encoder readings.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

import math
import time
from typing import Optional

from .drivers.encoder import OdometryConfig, WheelOdometry


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class WheelOdomNode(Node):
    """
    ROS 2 node for wheel odometry.
    
    Publishes:
        /wheel/odom (nav_msgs/Odometry): Wheel odometry at ~50 Hz
        /diagnostics (DiagnosticArray): Health status
    
    Broadcasts:
        odom -> base_link transform (if publish_tf is true)
    
    Parameters:
        publish_rate (float): Publishing rate in Hz (default: 50.0)
        frame_id (str): Parent frame (default: "odom")
        child_frame_id (str): Child frame (default: "base_link")
        publish_tf (bool): Whether to publish TF (default: false)
        track_width (float): Distance between tracks in meters
        wheel_radius (float): Wheel/sprocket radius in meters
        encoder_cpr (int): Encoder counts per revolution
        left_encoder_a (int): GPIO pin for left encoder A
        left_encoder_b (int): GPIO pin for left encoder B
        right_encoder_a (int): GPIO pin for right encoder A
        right_encoder_b (int): GPIO pin for right encoder B
    """
    
    def __init__(self):
        super().__init__('wheel_odom_node')
        
        # Declare parameters with CORRECTED GPIO pins (17, 27, 22, 4)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('track_width', 0.24)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('encoder_cpr', 3960)
        self.declare_parameter('left_encoder_a', 17)
        self.declare_parameter('left_encoder_b', 27)
        self.declare_parameter('right_encoder_a', 22)
        self.declare_parameter('right_encoder_b', 4)  # FIXED: was 23, now 4
        
        # Get parameters
        self._rate = self.get_parameter('publish_rate').value
        self._frame_id = self.get_parameter('frame_id').value
        self._child_frame_id = self.get_parameter('child_frame_id').value
        self._publish_tf = self.get_parameter('publish_tf').value
        
        # Create odometry config
        config = OdometryConfig(
            track_width=self.get_parameter('track_width').value,
            wheel_radius=self.get_parameter('wheel_radius').value,
            encoder_cpr=self.get_parameter('encoder_cpr').value,
            left_encoder_a=self.get_parameter('left_encoder_a').value,
            left_encoder_b=self.get_parameter('left_encoder_b').value,
            right_encoder_a=self.get_parameter('right_encoder_a').value,
            right_encoder_b=self.get_parameter('right_encoder_b').value,
        )
        
        # Create odometry calculator
        self._odometry: Optional[WheelOdometry] = None
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self._odom_pub = self.create_publisher(Odometry, '/wheel/odom', sensor_qos)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # TF broadcaster
        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)
        else:
            self._tf_broadcaster = None
        
        # Timers
        self._publish_timer = self.create_timer(1.0 / self._rate, self._publish_callback)
        self._diag_timer = self.create_timer(1.0, self._diagnostics_callback)
        
        # Statistics
        self._publish_count = 0
        self._error_count = 0
        
        # Initialize odometry
        self._init_odometry(config)
        
        self.get_logger().info(
            f"Wheel odometry node started (rate={self._rate}Hz, "
            f"track_width={config.track_width}m, wheel_radius={config.wheel_radius}m, "
            f"GPIO: L({config.left_encoder_a},{config.left_encoder_b}), "
            f"R({config.right_encoder_a},{config.right_encoder_b}))"
        )
    
    def _init_odometry(self, config: OdometryConfig) -> None:
        """Initialize wheel odometry."""
        try:
            self._odometry = WheelOdometry(config)
            
            if self._odometry.start():
                self.get_logger().info("Wheel odometry started")
            else:
                self.get_logger().error("Failed to start wheel odometry")
                self._odometry = None
                
        except Exception as e:
            self.get_logger().error(f"Odometry error: {e}")
            self._odometry = None
    
    def _publish_callback(self) -> None:
        """Timer callback to publish odometry."""
        if self._odometry is None:
            self._error_count += 1
            return
        
        try:
            # Update odometry
            self._odometry.update()
            
            # Get pose and velocity
            x, y, theta = self._odometry.get_pose()
            vx, wz = self._odometry.get_velocity()
            
            # Create timestamp
            now = self.get_clock().now()
            
            # Create Odometry message
            msg = Odometry()
            msg.header = Header()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = self._frame_id
            msg.child_frame_id = self._child_frame_id
            
            # Pose
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, theta)
            
            # Pose covariance (grows over time due to drift)
            pos_cov = 0.1  # meters^2 (high - odometry drifts)
            rot_cov = 0.05  # radians^2
            msg.pose.covariance = [
                pos_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, pos_cov, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 999.0, 0.0, 0.0, 0.0,  # Z is unknown
                0.0, 0.0, 0.0, 999.0, 0.0, 0.0,  # Roll is unknown
                0.0, 0.0, 0.0, 0.0, 999.0, 0.0,  # Pitch is unknown
                0.0, 0.0, 0.0, 0.0, 0.0, rot_cov,
            ]
            
            # Twist (velocity)
            msg.twist.twist.linear.x = vx
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.linear.z = 0.0
            msg.twist.twist.angular.x = 0.0
            msg.twist.twist.angular.y = 0.0
            msg.twist.twist.angular.z = wz
            
            # Twist covariance
            vel_cov = 0.01  # (m/s)^2 - fairly accurate
            ang_cov = 0.01  # (rad/s)^2
            msg.twist.covariance = [
                vel_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 999.0, 0.0, 0.0, 0.0, 0.0,  # vy is always 0 for diff drive
                0.0, 0.0, 999.0, 0.0, 0.0, 0.0,  # vz is always 0
                0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, ang_cov,
            ]
            
            # Publish odometry
            self._odom_pub.publish(msg)
            self._publish_count += 1
            
            # Publish TF if enabled
            if self._tf_broadcaster:
                t = TransformStamped()
                t.header = msg.header
                t.child_frame_id = self._child_frame_id
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = 0.0
                t.transform.rotation = msg.pose.pose.orientation
                self._tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().debug(f"Odometry error: {e}")
            self._error_count += 1
    
    def _diagnostics_callback(self) -> None:
        """Publish diagnostics."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = "Wheel Odometry"
        status.hardware_id = "wheel_encoders"
        
        if self._odometry is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "Odometry not initialized"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Running"
            
            x, y, theta = self._odometry.get_pose()
            vx, wz = self._odometry.get_velocity()
            
            status.values = [
                KeyValue(key="publish_rate_hz", value=str(self._publish_count)),
                KeyValue(key="error_count", value=str(self._error_count)),
                KeyValue(key="x_m", value=f"{x:.3f}"),
                KeyValue(key="y_m", value=f"{y:.3f}"),
                KeyValue(key="theta_deg", value=f"{math.degrees(theta):.1f}"),
                KeyValue(key="vx_mps", value=f"{vx:.3f}"),
                KeyValue(key="wz_radps", value=f"{wz:.3f}"),
            ]
        
        msg.status.append(status)
        self._diag_pub.publish(msg)
        
        # Reset counter
        self._publish_count = 0
    
    def destroy_node(self):
        """Clean up on shutdown."""
        if self._odometry:
            self._odometry.stop()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = WheelOdomNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
