from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['0', '0', '1', '0', '0', '0',
                       'map', '/mechmind_profiler/point_cloud']
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['0', '0', '1', '0', '0', '0', 'map',
                       '/mechmind_profiler/textured_point_cloud']
        ),
        Node(
            package="mecheye_profiler_ros_interface",
            executable="start",
            name="mechmind_profiler_publisher_service",
            output="screen",
            prefix="xterm -e",
            parameters=[
                {"save_file": True},
                {"profiler_ip": "172.20.112.1"}   # change to your profiler ip
            ]
        )
    ])
