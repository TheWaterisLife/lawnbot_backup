"""
demo.launch.py - Full sensor integration demo

Starts all sensor nodes + EKF fusion.

Usage:
    ros2 launch sensor_integration demo.launch.py
    ros2 launch sensor_integration demo.launch.py imu_type:=bno085

Prerequisites:
    - rtk_reader must be running (publishes /rtk/fix)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_integration')
    
    # Launch arguments
    imu_type_arg = DeclareLaunchArgument(
        'imu_type',
        default_value='mpu6050',
        description='IMU type: mpu6050 or bno085'
    )
    
    # Include sensors launch
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sensors.launch.py')
        ),
        launch_arguments={'imu_type': LaunchConfiguration('imu_type')}.items()
    )
    
    # Include fusion launch
    fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'fusion.launch.py')
        )
    )
    
    return LaunchDescription([
        imu_type_arg,
        sensors_launch,
        fusion_launch,
    ])
