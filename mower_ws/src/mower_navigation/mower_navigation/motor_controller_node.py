"""
Motor Controller ROS 2 Node.

Story 4.1-4.3: Motor Control

Connects to the lawnbot_motors WebSocket server for motor control.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import json

from .motor_controller import MotorController, MotorConfig, VelocityController


class MotorControllerNode(Node):
    """
    ROS 2 node for motor control via lawnbot_motors WebSocket server.

    Subscriptions:
        /cmd_vel (geometry_msgs/Twist): Velocity commands

    Services:
        /motors/emergency_stop (std_srvs/Trigger): Emergency stop
        /motors/enable (std_srvs/Trigger): Enable motors
        /motors/disable (std_srvs/Trigger): Disable motors

    Publications:
        /motors/status (std_msgs/String): JSON status
        /motors/speeds (std_msgs/Float32MultiArray): [left, right] speeds
        /diagnostics (diagnostic_msgs/DiagnosticArray): Health status
    """

    def __init__(self):
        super().__init__('motor_controller_node')

        # Declare parameters (WebSocket connection to lawnbot_motors)
        self.declare_parameter('ws_host', 'localhost')
        self.declare_parameter('ws_port', 8766)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('track_width', 0.24)
        self.declare_parameter('max_linear_velocity', 0.8)
        self.declare_parameter('max_angular_velocity', 1.5)

        # Create motor config
        config = MotorConfig(
            ws_host=self.get_parameter('ws_host').value,
            ws_port=self.get_parameter('ws_port').value,
            max_speed=self.get_parameter('max_speed').value,
            watchdog_timeout=self.get_parameter('watchdog_timeout').value,
        )

        # Create controllers
        self._motor = MotorController(
            config,
            track_width=self.get_parameter('track_width').value,
        )
        self._velocity = VelocityController(
            self._motor,
            max_linear_velocity=self.get_parameter('max_linear_velocity').value,
            max_angular_velocity=self.get_parameter('max_angular_velocity').value,
        )

        # State
        self._enabled = False
        self._emergency_stopped = False

        # Subscribers
        self._cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )

        # Services
        self.create_service(
            Trigger, '/motors/emergency_stop', self._emergency_stop_callback
        )
        self.create_service(
            Trigger, '/motors/enable', self._enable_callback
        )
        self.create_service(
            Trigger, '/motors/disable', self._disable_callback
        )

        # Publishers
        self._status_pub = self.create_publisher(String, '/motors/status', 10)
        self._speeds_pub = self.create_publisher(
            Float32MultiArray, '/motors/speeds', 10
        )
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Timers
        self.create_timer(0.1, self._publish_status)
        self.create_timer(1.0, self._publish_diagnostics)

        # Set emergency stop callback
        self._motor.set_emergency_stop_callback(self._on_emergency_stop)

        self.get_logger().info(
            f"Motor controller node started "
            f"(ws://{config.ws_host}:{config.ws_port})"
        )

    def _cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands."""
        if not self._enabled or self._emergency_stopped:
            return

        self._velocity.set_velocity(msg.linear.x, msg.angular.z)

    def _emergency_stop_callback(self, request, response):
        """Emergency stop service."""
        self._motor.emergency_stop()
        self._emergency_stopped = True
        self._enabled = False

        response.success = True
        response.message = "EMERGENCY STOP activated"
        self.get_logger().warn(response.message)

        return response

    def _enable_callback(self, request, response):
        """Enable motors service."""
        if self._emergency_stopped:
            response.success = False
            response.message = "Clear emergency stop first"
            return response

        if not self._enabled:
            if self._motor.start():
                self._enabled = True
                response.success = True
                response.message = (
                    f"Motors enabled (connected={self._motor.is_connected})"
                )
            else:
                response.success = False
                response.message = "Failed to start motors"
        else:
            response.success = True
            response.message = "Motors already enabled"

        return response

    def _disable_callback(self, request, response):
        """Disable motors service."""
        self._motor.stop()
        self._enabled = False
        self._emergency_stopped = False

        response.success = True
        response.message = "Motors disabled"

        return response

    def _on_emergency_stop(self):
        """Called when emergency stop is triggered internally."""
        self._emergency_stopped = True
        self._enabled = False
        self.get_logger().error("Emergency stop triggered internally!")

    def _publish_status(self):
        """Publish motor status."""
        left, right = self._motor.get_speeds()
        target_left, target_right = self._motor.get_target_speeds()

        # JSON status
        status = {
            'enabled': self._enabled,
            'emergency_stopped': self._emergency_stopped,
            'connected': self._motor.is_connected,
            'left_speed': left,
            'right_speed': right,
            'target_left': target_left,
            'target_right': target_right,
            'is_moving': self._motor.is_moving,
        }

        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

        # Float array for speeds
        speeds_msg = Float32MultiArray()
        speeds_msg.data = [left, right]
        self._speeds_pub.publish(speeds_msg)

    def _publish_diagnostics(self):
        """Publish diagnostics."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "Motor Controller"
        status.hardware_id = "motors"

        if self._emergency_stopped:
            status.level = DiagnosticStatus.ERROR
            status.message = "EMERGENCY STOPPED"
        elif not self._motor.is_connected:
            status.level = DiagnosticStatus.WARN
            status.message = "Not connected to lawnbot_motors"
        elif not self._enabled:
            status.level = DiagnosticStatus.WARN
            status.message = "Disabled"
        elif self._motor.is_moving:
            status.level = DiagnosticStatus.OK
            status.message = "Running"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Ready"

        left, right = self._motor.get_speeds()
        status.values = [
            KeyValue(key="enabled", value=str(self._enabled)),
            KeyValue(key="emergency_stopped", value=str(self._emergency_stopped)),
            KeyValue(key="connected", value=str(self._motor.is_connected)),
            KeyValue(key="left_speed", value=f"{left:.2f}"),
            KeyValue(key="right_speed", value=f"{right:.2f}"),
        ]

        msg.status.append(status)
        self._diag_pub.publish(msg)

    def destroy_node(self):
        """Clean up on shutdown."""
        self._motor.stop()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = MotorControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
