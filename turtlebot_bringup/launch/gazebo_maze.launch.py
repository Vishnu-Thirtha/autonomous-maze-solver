#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --------------------------
    # Launch arguments
    # --------------------------
    world_arg = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Path to world file. If empty or missing, empty_world will be loaded.'
    )
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    declare_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # --------------------------
    # Gazebo and TB3 paths
    # --------------------------
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch'
    )

    empty_world_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )

    # --------------------------
    # Function to set up Gazebo launch
    # --------------------------
    def launch_setup(context, *args, **kwargs):

        # Determine actual world path
        world = world_arg.perform(context)
        if world == '' or not os.path.isfile(world):
            if world != '':
                print(f"[WARN] World file {world} not found, loading empty world instead.")
            world = empty_world_path

        # --------------------------
        # Gazebo server
        # --------------------------
        gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items()
        )

        # --------------------------
        # Gazebo client
        # --------------------------
        gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        )

        # --------------------------
        # Robot state publisher
        # --------------------------
        robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        )

        # --------------------------
        # Spawn a single TurtleBot3
        # --------------------------
        spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': '0.0',  # original launch uses 0
            'yaw': yaw
        }.items()
)


        return [gzserver, gzclient, robot_state_publisher, spawn_tb3]

    return LaunchDescription([
        declare_world,
        declare_x,
        declare_y,
        declare_yaw,
        declare_sim_time,
        OpaqueFunction(function=launch_setup)
    ])
