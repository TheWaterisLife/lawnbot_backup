from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lawn_mower_battery'),
        'config',
        'battery_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='lawn_mower_battery',
            executable='battery_monitor',
            name='battery_monitor',
            output='screen',
            parameters=[config]
        )
    ])
