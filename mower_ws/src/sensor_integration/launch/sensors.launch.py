"""
sensors.launch.py - Start sensor nodes (IMU + Wheel Odometry)

Usage:
    ros2 launch sensor_integration sensors.launch.py
    ros2 launch sensor_integration sensors.launch.py imu_type:=bno085
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('sensor_integration')
    
    # Config file paths
    sensors_config = os.path.join(pkg_share, 'config', 'sensors.yaml')
    
    # Launch arguments
    imu_type_arg = DeclareLaunchArgument(
        'imu_type',
        default_value='mpu6050',
        description='IMU type: mpu6050 or bno085'
    )
    
    # IMU Launch
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('imu_reader'), 'launch', 'imu.launch.py')
        ),
        launch_arguments={'imu_type': LaunchConfiguration('imu_type')}.items()
    )

    # Wheel Encoder Launch
    wheel_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('wheel_encoder'), 'launch', 'wheel_odom.launch.py')
        )
    )
    
    # Static transforms
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.3', '0', '0', '0', 'base_link', 'gps_link']
    )
    
    return LaunchDescription([
        imu_type_arg,
        imu_launch,
        wheel_odom_launch,
        imu_tf,
        gps_tf,
    ])
