from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('publish_rate', default_value='50.0'),
        DeclareLaunchArgument('track_width', default_value='0.24'),
        DeclareLaunchArgument('wheel_radius', default_value='0.05'),
        DeclareLaunchArgument('encoder_cpr', default_value='3960'),
        
        Node(
            package='wheel_encoder',
            executable='wheel_odom_node',
            name='wheel_odom_node',
            output='screen',
            parameters=[{
                'publish_rate': LaunchConfiguration('publish_rate'),
                'track_width': LaunchConfiguration('track_width'),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'encoder_cpr': LaunchConfiguration('encoder_cpr'),
                'frame_id': 'odom',
                'child_frame_id': 'base_link'
            }]
        )
    ])
