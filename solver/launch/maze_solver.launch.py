#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')

    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')
    pkg_maze = get_package_share_directory('solver')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                pkg_maze, 'config', 'nav2_params.yaml'
            )
        ),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                pkg_maze, 'config', 'slam_toolbox_params.yaml'
            )
        ),

        # ----------------------
        # SLAM Toolbox (online async)
        # ----------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg_slam, 'launch', 'online_async_launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file
            }.items()
        ),

        # ----------------------
        # Nav2 (mapping mode)
        # ----------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg_nav2, 'launch', 'navigation_launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true'
            }.items()
        ),
    ])
