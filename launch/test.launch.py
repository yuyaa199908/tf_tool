import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('tf_tool'),
        'config',
        'frame_connector.yaml'
    )
    return LaunchDescription([
        Node(
            package='tf_tool',
            namespace='frame_connector',
            executable='frame_connector',
            remappings=[],
            parameters=[config]
        ),
    ])