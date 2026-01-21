#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

CELL_SIZE = 1.0  # size of a maze cell in meters

def generate_launch_description():

    # --------------------------
    # Launch arguments
    # --------------------------
    world_arg = LaunchConfiguration('world')
    maze_rows = LaunchConfiguration('maze_rows')
    maze_cols = LaunchConfiguration('maze_cols')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Path to world file. If empty or missing, empty_world will be loaded.'
    )
    declare_maze_rows = DeclareLaunchArgument('maze_rows', default_value='10')
    declare_maze_cols = DeclareLaunchArgument('maze_cols', default_value='10')
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

        # Maze dimensions
        rows = int(maze_rows.perform(context))
        cols = int(maze_cols.perform(context))

        # --------------------------
        # Compute top-left cell spawn position
        # --------------------------
        x_spawn = 0 * CELL_SIZE + CELL_SIZE / 2            # leftmost column
        y_spawn = (rows - 1) * CELL_SIZE + CELL_SIZE / 2  # top row

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
        # Spawn a single TurtleBot3 at top-left
        # --------------------------
        spawn_tb3 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': str(x_spawn),
                'y_pose': str(y_spawn),
                'z_pose': '0.01',  # small offset above ground
                'yaw': yaw
            }.items()
        )

        return [gzserver, gzclient, robot_state_publisher, spawn_tb3]

    return LaunchDescription([
        declare_world,
        declare_maze_rows,
        declare_maze_cols,
        declare_yaw,
        declare_sim_time,
        OpaqueFunction(function=launch_setup)
    ])
