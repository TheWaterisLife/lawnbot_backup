"""
Coverage Planner ROS 2 Node.

Story 2.1: Simple Coverage Planner

Generates and publishes coverage paths for mowing.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

import json
import math
import os

from .coverage_planner import CoveragePlanner, CoverageConfig, Waypoint


class CoveragePlannerNode(Node):
    """
    ROS 2 node for coverage path planning.
    
    Services:
        /coverage/generate (std_srvs/Trigger): Generate coverage path
        /coverage/start (std_srvs/Trigger): Start following path
        /coverage/pause (std_srvs/Trigger): Pause path following
        /coverage/stop (std_srvs/Trigger): Stop and clear path
        
    Publications:
        /coverage/path (nav_msgs/Path): Generated coverage path
        /coverage/status (std_msgs/String): JSON status
        /coverage/markers (visualization_msgs/MarkerArray): RViz visualization
        /cmd_vel (geometry_msgs/Twist): Velocity commands (when active)
    """
    
    def __init__(self):
        super().__init__('coverage_planner_node')
        
        # Declare parameters
        self.declare_parameter('boundary_file', '')
        self.declare_parameter('cutting_width', 0.30)
        self.declare_parameter('overlap_ratio', 0.10)
        self.declare_parameter('boundary_margin', 0.15)
        self.declare_parameter('mowing_angle', 0.0)
        
        # Get parameters
        self._boundary_file = self.get_parameter('boundary_file').value
        
        config = CoverageConfig(
            cutting_width=self.get_parameter('cutting_width').value,
            overlap_ratio=self.get_parameter('overlap_ratio').value,
            boundary_margin=self.get_parameter('boundary_margin').value,
            angle_deg=self.get_parameter('mowing_angle').value,
        )
        self._planner = CoveragePlanner(config)
        
        # Path state
        self._path: list = []
        self._current_waypoint_index = 0
        self._is_active = False
        self._is_paused = False
        self._boundary_points = []
        
        # Services
        self.create_service(
            Trigger, '/coverage/generate', self._generate_callback
        )
        self.create_service(
            Trigger, '/coverage/start', self._start_callback
        )
        self.create_service(
            Trigger, '/coverage/pause', self._pause_callback
        )
        self.create_service(
            Trigger, '/coverage/stop', self._stop_callback
        )
        
        # Publishers
        self._path_pub = self.create_publisher(Path, '/coverage/path', 10)
        self._status_pub = self.create_publisher(String, '/coverage/status', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/coverage/markers', 10
        )
        
        # Timer for status
        self.create_timer(1.0, self._publish_status)
        
        # Try to load boundary
        self._load_boundary()
        
        self.get_logger().info(
            f"Coverage planner node started (boundary file: {self._boundary_file})"
        )
        
    def _load_boundary(self):
        """Load boundary from JSON file."""
        if not self._boundary_file or not os.path.exists(self._boundary_file):
            self.get_logger().warn(f"Boundary file not found: {self._boundary_file}")
            self._boundary_points = []
            return None

        try:
            with open(self._boundary_file, 'r') as f:
                data = json.load(f)
                
            # Expecting 'boundary_xy' list of [x, y] lists
            if 'boundary_xy' in data:
                 # Convert list of lists to list of tuples/points if needed by planner
                 # Assuming planner expects list of (x, y) tuples or similar
                 self._boundary_points = [tuple(p) for p in data['boundary_xy']]
                 self.get_logger().info(f"Loaded {len(self._boundary_points)} boundary points from {self._boundary_file}")
                 return self._boundary_points
            else:
                 self.get_logger().error(f"Invalid boundary file format: 'boundary_xy' missing in {self._boundary_file}")
                 self._boundary_points = []
                 return None
                 
        except Exception as e:
            self.get_logger().error(f"Failed to load boundary file: {e}")
            self._boundary_points = []
            return None
    
    def _generate_callback(self, request, response):
        """Generate coverage path service."""
        # Reload boundary in case file changed
        boundary = self._load_boundary()
        
        if not boundary:
            response.success = False
            response.message = f"Boundary file '{self._boundary_file}' invalid or not found"
            return response
        
        # Generate path
        self._path = self._planner.generate_path(boundary)
        self._current_waypoint_index = 0
        
        if not self._path:
            response.success = False
            response.message = "Failed to generate path"
            return response
        
        # Get stats
        stats = self._planner.get_coverage_stats(boundary)
        
        response.success = True
        response.message = (
            f"Generated path: {len(self._path)} waypoints, "
            f"{stats.get('total_distance_meters', 0):.0f}m distance, "
            f"~{stats.get('estimated_time_seconds', 0) / 60:.1f} min"
        )
        
        self.get_logger().info(response.message)
        self._publish_path()
        
        return response
    
    def _start_callback(self, request, response):
        """Start path following service."""
        if not self._path:
            response.success = False
            response.message = "No path generated"
            return response
        
        self._is_active = True
        self._is_paused = False
        
        response.success = True
        response.message = "Coverage started"
        self.get_logger().info(response.message)
        
        return response
    
    def _pause_callback(self, request, response):
        """Pause path following service."""
        self._is_paused = not self._is_paused
        
        response.success = True
        response.message = f"Coverage {'paused' if self._is_paused else 'resumed'}"
        self.get_logger().info(response.message)
        
        return response
    
    def _stop_callback(self, request, response):
        """Stop path following service."""
        self._is_active = False
        self._is_paused = False
        self._path = []
        self._current_waypoint_index = 0
        
        response.success = True
        response.message = "Coverage stopped"
        self.get_logger().info(response.message)
        
        return response
    
    def _publish_status(self):
        """Publish status."""
        
        status = {
            'boundary_file': self._boundary_file,
            'boundary_loaded': len(self._boundary_points) > 0,
            'path_length': len(self._path),
            'current_waypoint': self._current_waypoint_index,
            'is_active': self._is_active,
            'is_paused': self._is_paused,
            'progress_percent': (
                100.0 * self._current_waypoint_index / len(self._path)
                if self._path else 0.0
            ),
        }
        
        if self._boundary_points:
            stats = self._planner.get_coverage_stats(self._boundary_points)
            status.update(stats)
        
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)
    
    def _publish_path(self):
        """Publish path as nav_msgs/Path and markers."""
        if not self._path:
            return
        
        # Publish nav_msgs/Path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for wp in self._path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            pose.pose.position.z = 0.0
            
            # Convert heading to quaternion
            pose.pose.orientation.w = math.cos(wp.heading / 2)
            pose.pose.orientation.z = math.sin(wp.heading / 2)
            
            path_msg.poses.append(pose)
        
        self._path_pub.publish(path_msg)
        
        # Publish markers for RViz
        markers = MarkerArray()
        
        # Path line
        line_marker = Marker()
        line_marker.header = path_msg.header
        line_marker.ns = "coverage_path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.02
        line_marker.color.r = 1.0
        line_marker.color.g = 0.5
        line_marker.color.b = 0.0
        line_marker.color.a = 0.8
        
        for wp in self._path:
            from geometry_msgs.msg import Point
            p = Point()
            p.x = wp.x
            p.y = wp.y
            p.z = 0.0
            line_marker.points.append(p)
        
        markers.markers.append(line_marker)
        
        # Waypoint spheres
        for i, wp in enumerate(self._path):
            wp_marker = Marker()
            wp_marker.header = path_msg.header
            wp_marker.ns = "waypoints"
            wp_marker.id = i + 1
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.pose.position.x = wp.x
            wp_marker.pose.position.y = wp.y
            wp_marker.pose.position.z = 0.0
            wp_marker.scale.x = 0.05
            wp_marker.scale.y = 0.05
            wp_marker.scale.z = 0.05
            wp_marker.color.r = 0.0
            wp_marker.color.g = 0.5
            wp_marker.color.b = 1.0
            wp_marker.color.a = 0.8
            
            markers.markers.append(wp_marker)
        
        self._marker_pub.publish(markers)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = CoveragePlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
