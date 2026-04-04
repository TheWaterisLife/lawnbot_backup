"""
Nav2 Launch file for autonomous mower navigation.

This launch file starts the Nav2 navigation stack configured for the mower.
Requires: ros-jazzy-nav2-bringup

Usage:
    ros2 launch mower_navigation nav2.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Generate Nav2 launch description."""
    
    # Package paths
    pkg_mower_nav = get_package_share_directory('mower_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Configuration file
    nav2_params_file = os.path.join(pkg_mower_nav, 'config', 'nav2_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Lifecycle manager nodes to manage
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    
    # Rewrite params with substitutions
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'
        ),
        
        # Nav2 Controller Server (DWB)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),  # Nav2 outputs here
                ('odom', 'odometry/filtered'),
            ],
        ),
        
        # Nav2 Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
        ),
        
        # Nav2 Behavior Server (recoveries)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
        ),
        
        # Nav2 BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
        ),
        
        # Nav2 Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
        ),
        
        # Nav2 Velocity Smoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),          # Input from controller
                ('cmd_vel_smoothed', 'cmd_vel'),     # Output to motors
                ('odom', 'odometry/filtered'),
            ],
        ),
        
        # Nav2 Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])
