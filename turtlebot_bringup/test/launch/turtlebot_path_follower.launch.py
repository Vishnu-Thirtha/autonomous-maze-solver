from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ----------------------------------
    # Package directories
    # ----------------------------------
    turtlebot_bringup_dir = get_package_share_directory('turtlebot_bringup')
    path_planner_dir = get_package_share_directory('path_planner')
    controller_dir = get_package_share_directory('controller')

    # ----------------------------------
    # 1. Empty world (TurtleBot)
    # ----------------------------------
    empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot_bringup_dir,
                'launch',
                'turtlebot_empty_world.launch.py'
            )
        )
    )

    # ----------------------------------
    # 2. Waypoints loader
    # ----------------------------------
    waypoints_loader = Node(
        package='path_planner',
        executable='waypoints_loader',
        name='waypoints_loader',
        output='screen'
    )

    # ----------------------------------
    # 3. Controller + GUI
    # ----------------------------------
    controller_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                controller_dir,
                'launch',
                'controller_with_gui.launch.py'
            )
        )
    )

    # ----------------------------------
    # 4. RViz
    # ----------------------------------
    rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('turtlebot_bringup'),
                'rviz',
                'path_follower_rivz_config.rviz'
            ])
        ]
    )

    # ----------------------------------
    # Launch description
    # ----------------------------------
    return LaunchDescription([
        empty_world,
        waypoints_loader,
        controller_with_gui,
        rviz,
    ])
