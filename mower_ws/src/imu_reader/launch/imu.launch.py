from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('imu_type', default_value='bno085'),
        DeclareLaunchArgument('i2c_bus', default_value='1'),
        DeclareLaunchArgument('i2c_address', default_value='74'),  # 74 = 0x4A (BNO085), 104 = 0x68 (MPU6050)
        DeclareLaunchArgument('publish_rate', default_value='20.0'),
        
        Node(
            package='imu_reader',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'imu_type': LaunchConfiguration('imu_type'),
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'frame_id': 'imu_link'
            }]
        )
    ])
