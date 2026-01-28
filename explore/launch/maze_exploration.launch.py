#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    explorer_node = Node(
        package='explore',
        executable='explorer',
        name='maze_explorer',
        output='screen'
    )

    manager_node = Node(
        package='explore',
        executable='manager',
        name='mission_manager',
        output='screen'
    )

    return LaunchDescription([
        manager_node,
        explorer_node
    ])
