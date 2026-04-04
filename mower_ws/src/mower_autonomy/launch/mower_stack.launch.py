from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # RTK reader (publishes /rtk/fix)
        Node(
            package="rtk_reader",
            executable="rtk_reader",
            name="rtk_reader",
            output="screen",
        ),

        # IMU reader (publishes /imu/data from BNO085 or MPU6050)
        Node(
            package="imu_reader",
            executable="imu_node",
            name="imu_node",
            output="screen",
            parameters=[{
                "imu_type": "bno085",
                "i2c_bus": 1,
                "i2c_address": 74,
                "publish_rate": 20.0,
                "frame_id": "imu_link",
            }],
        ),

        # Encoders (publishes /encoders/ticks and /encoders/delta)
        Node(
            package="mower_autonomy",
            executable="encoder_node",
            name="encoder_node",
            output="screen",
            parameters=[{
                "la": 17,
                "lb": 27,
                "ra": 22,
                "rb": 4,
                "publish_hz": 50.0,
                "invert_left": False,
                "invert_right": True,
            }],
        ),

        # Localization (consumes /rtk/fix + /encoders/ticks, publishes /mower/pose)
        Node(
            package="mower_autonomy",
            executable="localization_node",
            name="localization_node",
            output="screen",
        ),

        # Autonomy (consumes /mower/pose + map, sends motor commands via WebSocket)
        Node(
            package="mower_autonomy",
            executable="autonomy_node",
            name="autonomy_node",
            output="screen",
        ),

        # Mapping server (WebSocket for app)
        Node(
            package="mower_mapping",
            executable="map_server",
            name="map_server",
            output="screen",
        ),

        # Mobile app bridge (WebSocket :9002 <-> ROS2 topics)
        Node(
            package="lawnbot_comms",
            executable="bridge_node",
            name="lawnbot_bridge",
            output="screen",
        ),
    ])

