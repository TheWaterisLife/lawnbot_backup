"""
Launch file for mower navigation nodes.

Starts:
- coverage_planner_node: Coverage path planning
- motor_controller_node: Motor control (BTS7960 via gpiozero)
- obstacle_handler_node: Obstacle handling
- navigation_controller_node: Nav2 waypoint follower

Note: Boundary recording/loading is handled by mower_mapping (map_server).
      Motor hardware is handled by lawnbot_motors (standalone WebSocket server).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    


    boundary_file_arg = DeclareLaunchArgument(
        'boundary_file',
        default_value='',
        description='Absolute path to boundary JSON file'
    )

    cutting_width_arg = DeclareLaunchArgument(
        'cutting_width',
        default_value='0.30',
        description='Cutting deck width in meters'
    )
    
    track_width_arg = DeclareLaunchArgument(
        'track_width',
        default_value='0.24',
        description='Track width in meters'
    )
    
    # =========================================================================
    # Nodes
    # =========================================================================
    
    # Coverage Planner Node
    coverage_node = Node(
        package='mower_navigation',
        executable='coverage_planner_node',
        name='coverage_planner_node',
        parameters=[{
            'boundary_file': LaunchConfiguration('boundary_file'),
            'cutting_width': LaunchConfiguration('cutting_width'),
            'overlap_ratio': 0.10,
            'boundary_margin': 0.15,
        }],
        output='screen',
    )
    

    
    # Motor Controller Node
    motor_node = Node(
        package='mower_navigation',
        executable='motor_controller_node',
        name='motor_controller_node',
        parameters=[{
            'track_width': LaunchConfiguration('track_width'),
            'max_linear_velocity': 0.8,
            'max_angular_velocity': 1.5,
            'watchdog_timeout': 0.5,
        }],
        output='screen',
    )
    
    # Obstacle Handler Node
    obstacle_node = Node(
        package='mower_navigation',
        executable='obstacle_handler_node',
        name='obstacle_handler_node',
        parameters=[{
            'critical_distance': 0.3,
            'warning_distance': 0.8,
            'detection_distance': 2.0,
            'filter_velocity': True,
        }],
        output='screen',
    )
    
    # Navigation Controller Node
    navigation_node = Node(
        package='mower_navigation',
        executable='navigation_controller_node',
        name='navigation_controller_node',
        parameters=[{
            'goal_tolerance': 0.15,
            'heading_tolerance': 0.25,
            'navigation_speed': 0.35,
        }],
        output='screen',
    )
    
    return LaunchDescription([
        # Arguments
        boundary_file_arg,
        cutting_width_arg,
        track_width_arg,
        
        # Nodes
        coverage_node,
        motor_node,
        obstacle_node,
        navigation_node,
    ])

