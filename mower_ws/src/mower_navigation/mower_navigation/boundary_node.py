"""
Boundary Management ROS 2 Node.

Story 1.1-1.3: Boundary Learning, Validation, Storage

Provides services and topics for boundary management.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker, MarkerArray

from .boundary import BoundaryManager, GPSPoint


class BoundaryNode(Node):
    """
    ROS 2 node for boundary management.
    
    Services:
        /boundary/start_learning (std_srvs/SetBool): Start learning (data = boundary name)
        /boundary/finish_learning (std_srvs/Trigger): Finish and save boundary
        /boundary/cancel_learning (std_srvs/Trigger): Cancel learning
        /boundary/save (std_srvs/Trigger): Save current boundary
        /boundary/load (std_srvs/SetBool): Load boundary by name
        
    Subscriptions:
        /gps/fix (sensor_msgs/NavSatFix): GPS position during learning
        
    Publications:
        /boundary/status (std_msgs/String): JSON status
        /boundary/polygon (geometry_msgs/PolygonStamped): Current boundary
        /boundary/markers (visualization_msgs/MarkerArray): RViz visualization
    """
    
    def __init__(self):
        super().__init__('boundary_node')
        
        # Declare parameters
        self.declare_parameter('storage_path', 'config/boundaries')
        self.declare_parameter('auto_add_points', True)
        self.declare_parameter('min_point_distance', 0.5)  # meters
        
        # Get parameters
        storage_path = self.get_parameter('storage_path').value
        self._auto_add = self.get_parameter('auto_add_points').value
        self._min_distance = self.get_parameter('min_point_distance').value
        
        # Create boundary manager
        self._manager = BoundaryManager(storage_path)
        
        # Last position for distance filtering
        self._last_lat = None
        self._last_lon = None
        
        # Services
        self.create_service(
            SetBool, '/boundary/start_learning', self._start_learning_callback
        )
        self.create_service(
            Trigger, '/boundary/finish_learning', self._finish_learning_callback
        )
        self.create_service(
            Trigger, '/boundary/cancel_learning', self._cancel_learning_callback
        )
        self.create_service(
            Trigger, '/boundary/save', self._save_callback
        )
        self.create_service(
            SetBool, '/boundary/load', self._load_callback
        )
        
        # Subscribers
        self._gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self._gps_callback, 10
        )
        
        # Publishers
        self._status_pub = self.create_publisher(String, '/boundary/status', 10)
        self._polygon_pub = self.create_publisher(
            PolygonStamped, '/boundary/polygon', 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, '/boundary/markers', 10
        )
        
        # Timer for status updates
        self.create_timer(1.0, self._publish_status)
        
        # Active boundary name for loading
        self._active_boundary = ""
        self._pending_name = ""
        
        self.get_logger().info(f"Boundary node started (storage: {storage_path})")
    
    def _start_learning_callback(self, request, response):
        """Start boundary learning service."""
        # The 'data' field is bool, we'll use a workaround with pending name
        # For now, use a default name or the pending name
        name = self._pending_name if self._pending_name else "lawn_boundary"
        
        if self._manager.start_learning(name):
            self._last_lat = None
            self._last_lon = None
            response.success = True
            response.message = f"Started learning boundary: {name}"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "Already in learning mode"
        
        return response
    
    def _finish_learning_callback(self, request, response):
        """Finish boundary learning service."""
        boundary = self._manager.finish_learning()
        
        if boundary:
            self._active_boundary = boundary.name
            self._manager.save_boundary(boundary.name)
            response.success = True
            response.message = f"Boundary '{boundary.name}' saved ({boundary.point_count} points)"
            self.get_logger().info(response.message)
            self._publish_boundary(boundary.name)
        else:
            response.success = False
            response.message = "Learning failed or not enough points"
        
        return response
    
    def _cancel_learning_callback(self, request, response):
        """Cancel boundary learning service."""
        self._manager.cancel_learning()
        response.success = True
        response.message = "Learning cancelled"
        return response
    
    def _save_callback(self, request, response):
        """Save current boundary service."""
        if self._active_boundary:
            if self._manager.save_boundary(self._active_boundary):
                response.success = True
                response.message = f"Saved boundary: {self._active_boundary}"
            else:
                response.success = False
                response.message = "Save failed"
        else:
            response.success = False
            response.message = "No active boundary"
        
        return response
    
    def _load_callback(self, request, response):
        """Load boundary service."""
        # Use pending name set by parameter or default
        name = self._pending_name if self._pending_name else "lawn_boundary"
        
        boundary = self._manager.load_boundary(name)
        if boundary:
            self._active_boundary = name
            response.success = True
            response.message = f"Loaded boundary: {name} ({boundary.point_count} points)"
            self._publish_boundary(name)
        else:
            response.success = False
            response.message = f"Could not load boundary: {name}"
        
        return response
    
    def _gps_callback(self, msg: NavSatFix):
        """GPS position callback - add points during learning."""
        if not self._manager.is_learning:
            return
        
        if not self._auto_add:
            return
        
        # Check minimum distance from last point
        if self._last_lat is not None:
            from math import radians, cos, sqrt
            
            dlat = msg.latitude - self._last_lat
            dlon = msg.longitude - self._last_lon
            
            # Approximate distance in meters
            lat_rad = radians(msg.latitude)
            dx = dlon * 6371000 * cos(lat_rad) * (3.14159 / 180)
            dy = dlat * 6371000 * (3.14159 / 180)
            distance = sqrt(dx * dx + dy * dy)
            
            if distance < self._min_distance:
                return
        
        # Add point
        self._manager.add_gps_point(msg.latitude, msg.longitude, msg.altitude)
        self._last_lat = msg.latitude
        self._last_lon = msg.longitude
    
    def _publish_status(self):
        """Publish current status."""
        import json
        
        progress = self._manager.get_learning_progress()
        progress['active_boundary'] = self._active_boundary
        progress['saved_boundaries'] = self._manager.list_saved_boundaries()
        
        msg = String()
        msg.data = json.dumps(progress)
        self._status_pub.publish(msg)
    
    def _publish_boundary(self, name: str):
        """Publish boundary as polygon and markers."""
        boundary = self._manager.get_boundary(name)
        if not boundary or not boundary.local_points:
            return
        
        # Publish polygon
        polygon_msg = PolygonStamped()
        polygon_msg.header.stamp = self.get_clock().now().to_msg()
        polygon_msg.header.frame_id = "map"
        
        for point in boundary.local_points:
            p = Point32()
            p.x = float(point.x)
            p.y = float(point.y)
            p.z = 0.0
            polygon_msg.polygon.points.append(p)
        
        # Close the polygon
        if boundary.local_points:
            p = Point32()
            p.x = float(boundary.local_points[0].x)
            p.y = float(boundary.local_points[0].y)
            p.z = 0.0
            polygon_msg.polygon.points.append(p)
        
        self._polygon_pub.publish(polygon_msg)
        
        # Publish markers for RViz
        markers = MarkerArray()
        
        # Line strip for boundary
        line_marker = Marker()
        line_marker.header = polygon_msg.header
        line_marker.ns = "boundary"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        
        for point in boundary.local_points:
            from geometry_msgs.msg import Point
            p = Point()
            p.x = float(point.x)
            p.y = float(point.y)
            p.z = 0.0
            line_marker.points.append(p)
        
        # Close the loop
        if boundary.local_points:
            from geometry_msgs.msg import Point
            p = Point()
            p.x = float(boundary.local_points[0].x)
            p.y = float(boundary.local_points[0].y)
            p.z = 0.0
            line_marker.points.append(p)
        
        markers.markers.append(line_marker)
        self._marker_pub.publish(markers)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = BoundaryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
