"""
fusion.launch.py - Start EKF fusion with navsat_transform

Usage:
    ros2 launch sensor_integration fusion.launch.py

Prerequisites:
    - rtk_reader must be running (publishes /rtk/fix)
    - sensor_integration sensors.launch.py should be running
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('sensor_integration')
    
    # Config file path
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    # EKF Node (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[ekf_config],
        output='screen'
    )
    
    # NavSat Transform Node (GPS lat/lon -> local ENU)
    # Subscribes to /rtk/fix from rtk_reader package
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        parameters=[ekf_config],
        remappings=[
            ('gps/fix', '/rtk/fix'),  # Use rtk_reader topic
            ('odometry/filtered', '/odom_filtered'),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        ekf_node,
        navsat_node,
    ])
