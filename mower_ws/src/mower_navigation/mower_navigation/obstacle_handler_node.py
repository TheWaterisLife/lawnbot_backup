"""
Obstacle Handler ROS 2 Node.

Story 3.1-3.2: Obstacle Detection and Avoidance

Integrates with camera vision for obstacle handling.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray

import json
import math

from .obstacle_handler import ObstacleHandler, SafetyZone, Obstacle, ObstacleType, AvoidanceAction


class ObstacleHandlerNode(Node):
    """
    ROS 2 node for obstacle handling.
    
    Subscriptions:
        /vision/detections (vision_msgs/Detection2DArray): Camera detections
        /cmd_vel_raw (geometry_msgs/Twist): Raw velocity commands
        
    Publications:
        /cmd_vel (geometry_msgs/Twist): Filtered velocity commands
        /obstacles/status (std_msgs/String): JSON status
        /obstacles/alert (std_msgs/Bool): True if obstacle in path
        
    Services:
        /obstacles/clear (std_srvs/Trigger): Clear obstacle list
    """
    
    def __init__(self):
        super().__init__('obstacle_handler_node')
        
        # Declare parameters
        self.declare_parameter('critical_distance', 0.3)
        self.declare_parameter('warning_distance', 0.8)
        self.declare_parameter('detection_distance', 2.0)
        self.declare_parameter('filter_velocity', True)
        
        # Create safety zone
        safety_zone = SafetyZone(
            critical_distance=self.get_parameter('critical_distance').value,
            warning_distance=self.get_parameter('warning_distance').value,
            detection_distance=self.get_parameter('detection_distance').value,
        )
        
        # Create handler
        self._handler = ObstacleHandler(safety_zone)
        self._filter_velocity = self.get_parameter('filter_velocity').value
        
        # Last velocity command
        self._last_cmd_vel = Twist()
        
        # Subscribers
        self._detection_sub = self.create_subscription(
            Detection2DArray, '/vision/detections', self._detection_callback, 10
        )
        self._cmd_vel_raw_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self._cmd_vel_raw_callback, 10
        )
        
        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._status_pub = self.create_publisher(String, '/obstacles/status', 10)
        self._alert_pub = self.create_publisher(Bool, '/obstacles/alert', 10)
        
        # Services
        self.create_service(
            Trigger, '/obstacles/clear', self._clear_callback
        )
        
        # Timer for status
        self.create_timer(0.1, self._publish_status)
        
        self.get_logger().info("Obstacle handler node started")
    
    def _detection_callback(self, msg: Detection2DArray):
        """Handle camera detections."""
        obstacles = []
        
        for det in msg.detections:
            # Get class label
            label = "unknown"
            confidence = 0.5
            if det.results:
                label = det.results[0].hypothesis.class_id
                confidence = det.results[0].hypothesis.score
            
            # Get bounding box
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            width = det.bbox.size_x
            height = det.bbox.size_y
            
            # Estimate distance from bbox (rough)
            # Larger bbox = closer object
            distance = max(0.5, 3.0 / max(0.1, height / 300))  # Assumes 300px image
            
            # Estimate angle from center x
            angle = (cx - 0.5) * math.radians(60)  # Assumes 60 deg FOV
            
            # Determine type
            label_lower = label.lower()
            if 'person' in label_lower:
                obs_type = ObstacleType.PERSON
            elif any(a in label_lower for a in ['dog', 'cat', 'bird']):
                obs_type = ObstacleType.ANIMAL
            elif any(v in label_lower for v in ['car', 'bicycle', 'motorcycle']):
                obs_type = ObstacleType.VEHICLE
            else:
                obs_type = ObstacleType.OBJECT
            
            obstacles.append(Obstacle(
                type=obs_type,
                distance=distance,
                angle=angle,
                width=width / 300 * distance,  # Estimate width
                confidence=confidence,
            ))
        
        self._handler.update_obstacles(obstacles)
    
    def _cmd_vel_raw_callback(self, msg: Twist):
        """Handle raw velocity commands and filter based on obstacles."""
        self._last_cmd_vel = msg
        
        if not self._filter_velocity:
            self._cmd_vel_pub.publish(msg)
            return
        
        # Get avoidance action
        action = self._handler.get_avoidance_action()
        speed_factor = self._handler.get_speed_factor()
        
        # Create filtered command
        filtered_cmd = Twist()
        
        if action == AvoidanceAction.STOP:
            # Full stop
            filtered_cmd.linear.x = 0.0
            filtered_cmd.angular.z = 0.0
            self.get_logger().warn("Obstacle - STOP")
            
        elif action == AvoidanceAction.SLOW_DOWN:
            # Slow down
            filtered_cmd.linear.x = msg.linear.x * speed_factor
            filtered_cmd.angular.z = msg.angular.z
            
        elif action == AvoidanceAction.TURN_LEFT:
            # Turn away from obstacle (stop forward, turn)
            filtered_cmd.linear.x = msg.linear.x * 0.3
            filtered_cmd.angular.z = 0.5  # Turn left
            
        elif action == AvoidanceAction.TURN_RIGHT:
            # Turn away from obstacle
            filtered_cmd.linear.x = msg.linear.x * 0.3
            filtered_cmd.angular.z = -0.5  # Turn right
            
        else:
            # No obstacle - pass through
            filtered_cmd = msg
        
        self._cmd_vel_pub.publish(filtered_cmd)
    
    def _clear_callback(self, request, response):
        """Clear obstacles service."""
        self._handler.clear_obstacles()
        response.success = True
        response.message = "Obstacles cleared"
        return response
    
    def _publish_status(self):
        """Publish status."""
        obstacles = self._handler.get_obstacles()
        closest = self._handler.get_closest_obstacle()
        action = self._handler.get_avoidance_action()
        
        status = {
            'obstacle_count': len(obstacles),
            'closest_distance': closest.distance if closest else None,
            'closest_type': closest.type.value if closest else None,
            'avoidance_action': action.value,
            'speed_factor': self._handler.get_speed_factor(),
            'path_clear': self._handler.is_path_clear(),
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)
        
        # Alert
        alert_msg = Bool()
        alert_msg.data = not self._handler.is_path_clear()
        self._alert_pub.publish(alert_msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = ObstacleHandlerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
