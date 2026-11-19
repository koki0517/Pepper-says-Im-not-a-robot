import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_to_cmd_vel_params = os.path.join(
        get_package_share_directory('pepper_omni'), 'config', 'joy_to_cmd_vel.yaml')

    spacenav_node = Node(
        package='spacenav',
        executable='spacenav_node',
        name='spacenav',
        output='screen',
    )

    joy_to_cmd_vel_node = Node(
        package='pepper_omni',
        executable='joy_to_cmd_vel',
        name='joy_to_cmd_vel',
        output='screen',
        parameters=[joy_to_cmd_vel_params],
        remappings=[('joy', '/spacenav/joy')],
    )

    return LaunchDescription([
        spacenav_node,
        joy_to_cmd_vel_node,
    ])
