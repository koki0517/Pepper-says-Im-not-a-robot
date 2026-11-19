import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('pepper_omni'), 'config', 'omni_driver.yaml')

    return LaunchDescription([
        Node(
            package='pepper_omni',
            executable='omni_driver',
            name='omni_driver',
            output='screen',
            parameters=[params],
        ),
    ])
