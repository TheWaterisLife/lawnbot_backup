"""
Navigation Controller ROS2 Node.

Story 3.7: Path Following Launch

Interfaces between coverage planner and Nav2 waypoint follower.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import NavigateToPose, FollowWaypoints

import json
import math
from typing import Optional

from .navigation_controller import (
    NavigationController,
    NavigationState,
    Waypoint,
)


class NavigationControllerNode(Node):
    """ROS2 node for navigation control with Nav2."""
    
    def __init__(self):
        super().__init__('navigation_controller_node')
        
        # Callback group for async operations
        self._cb_group = ReentrantCallbackGroup()
        
        # Parameters
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('heading_tolerance', 0.25)
        self.declare_parameter('navigation_speed', 0.35)
        
        goal_tol = self.get_parameter('goal_tolerance').value
        heading_tol = self.get_parameter('heading_tolerance').value
        nav_speed = self.get_parameter('navigation_speed').value
        
        # Navigation controller
        self._controller = NavigationController(
            goal_tolerance=goal_tol,
            heading_tolerance=heading_tol,
            navigation_speed=nav_speed,
        )
        
        # Set up callbacks
        self._controller.set_goal_callback(self._send_goal_to_nav2)
        self._controller.set_cancel_callback(self._cancel_nav2_goal)
        self._controller.set_state_callback(self._on_state_change)
        
        # Nav2 action client
        self._nav2_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._cb_group,
        )
        self._current_goal_handle = None
        
        # Publishers
        self._status_pub = self.create_publisher(String, 'navigation/status', 10)
        self._goal_pub = self.create_publisher(PoseStamped, 'navigation/current_goal', 10)
        
        # Subscribers
        self._odom_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self._odom_callback,
            10,
        )
        self._path_sub = self.create_subscription(
            Path,
            'coverage/path',
            self._path_callback,
            10,
        )
        
        # Services
        self._start_srv = self.create_service(
            Trigger,
            'navigation/start',
            self._start_callback,
        )
        self._pause_srv = self.create_service(
            Trigger,
            'navigation/pause',
            self._pause_callback,
        )
        self._stop_srv = self.create_service(
            Trigger,
            'navigation/stop',
            self._stop_callback,
        )
        self._skip_srv = self.create_service(
            Trigger,
            'navigation/skip_waypoint',
            self._skip_callback,
        )
        
        # Timer for status publishing
        self._status_timer = self.create_timer(1.0, self._publish_status)
        
        # Timer for progress checking
        self._progress_timer = self.create_timer(5.0, self._check_progress)
        
        self.get_logger().info('Navigation controller node started')
    
    def _odom_callback(self, msg: Odometry) -> None:
        """Update controller with current pose."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        
        self._controller.update_pose(x, y, yaw)
    
    def _path_callback(self, msg: Path) -> None:
        """Receive coverage path and load into controller."""
        waypoints = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            
            q = pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            
            waypoints.append(Waypoint(x=x, y=y, heading=yaw))
        
        if waypoints:
            self._controller.set_path(waypoints)
            self.get_logger().info(f'Loaded path with {len(waypoints)} waypoints')
    
    def _send_goal_to_nav2(self, waypoint: Waypoint) -> bool:
        """Send a waypoint goal to Nav2."""
        if not self._nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('Nav2 action server not available')
            return False
        
        # Create goal message
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint.x
        goal.pose.pose.position.y = waypoint.y
        goal.pose.pose.position.z = 0.0
        
        # Convert heading to quaternion
        goal.pose.pose.orientation.z = math.sin(waypoint.heading / 2.0)
        goal.pose.pose.orientation.w = math.cos(waypoint.heading / 2.0)
        
        # Publish current goal for visualization
        self._goal_pub.publish(goal.pose)
        
        # Send goal
        send_goal_future = self._nav2_client.send_goal_async(
            goal,
            feedback_callback=self._nav2_feedback_callback,
        )
        send_goal_future.add_done_callback(self._nav2_goal_response_callback)
        
        return True
    
    def _nav2_goal_response_callback(self, future) -> None:
        """Handle Nav2 goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 goal rejected')
            self._controller.on_goal_failed('Goal rejected')
            return
        
        self._current_goal_handle = goal_handle
        self.get_logger().info('Nav2 goal accepted')
        
        # Get result async
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_callback)
    
    def _nav2_result_callback(self, future) -> None:
        """Handle Nav2 goal completion."""
        result = future.result().result
        status = future.result().status
        
        # ActionGoalStatus values
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Nav2 goal succeeded')
            self._controller.on_goal_reached()
        else:
            self.get_logger().warning(f'Nav2 goal failed with status: {status}')
            self._controller.on_goal_failed(f'Status: {status}')
        
        self._current_goal_handle = None
    
    def _nav2_feedback_callback(self, feedback_msg) -> None:
        """Handle Nav2 feedback."""
        # Could use this for more detailed progress tracking
        pass
    
    def _cancel_nav2_goal(self) -> None:
        """Cancel current Nav2 goal."""
        if self._current_goal_handle:
            self.get_logger().info('Cancelling Nav2 goal')
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None
    
    def _on_state_change(self, state: NavigationState) -> None:
        """Handle navigation state changes."""
        self.get_logger().info(f'Navigation state: {state.value}')
        self._publish_status()
    
    def _start_callback(self, request, response) -> Trigger.Response:
        """Service callback to start navigation."""
        success = self._controller.start_navigation()
        response.success = success
        response.message = 'Navigation started' if success else 'Failed to start'
        return response
    
    def _pause_callback(self, request, response) -> Trigger.Response:
        """Service callback to pause navigation."""
        self._controller.pause_navigation()
        response.success = True
        response.message = 'Navigation paused'
        return response
    
    def _stop_callback(self, request, response) -> Trigger.Response:
        """Service callback to stop navigation."""
        self._controller.stop_navigation()
        response.success = True
        response.message = 'Navigation stopped'
        return response
    
    def _skip_callback(self, request, response) -> Trigger.Response:
        """Service callback to skip current waypoint."""
        success = self._controller.skip_waypoint()
        response.success = success
        response.message = 'Waypoint skipped' if success else 'Cannot skip'
        return response
    
    def _publish_status(self) -> None:
        """Publish navigation status."""
        progress = self._controller.get_progress()
        
        status = {
            'state': self._controller.state.value,
            'current_waypoint': progress.current_waypoint,
            'total_waypoints': progress.total_waypoints,
            'percent_complete': round(progress.percent_complete, 1),
            'distance_remaining': round(progress.distance_remaining, 2),
            'estimated_time_remaining': round(progress.estimated_time_remaining, 0),
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)
    
    def _check_progress(self) -> None:
        """Check for stuck condition."""
        if self._controller.check_progress_timeout():
            self.get_logger().warning('Navigation progress timeout - may be stuck')
            # Could trigger recovery here


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = NavigationControllerNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
