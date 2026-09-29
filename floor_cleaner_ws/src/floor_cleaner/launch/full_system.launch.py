"""
full_system.launch.py
---------------------
Convenience launcher that starts the simulation AND the software stack
in one terminal, by including the separate launch files.

  mode:=slam        (default)  simulation + SLAM Toolbox + RViz2   -> build a map
  mode:=navigation             simulation + AMCL/Nav2 + RViz2       -> navigate on a saved map

In navigation mode the robot has just been spawned at (0,0,0), so AMCL is
told that position automatically (set_initial_pose:=true) and you can
immediately click "Nav2 Goal".

Usage:
  ros2 launch floor_cleaner full_system.launch.py
  ros2 launch floor_cleaner full_system.launch.py mode:=navigation
  ros2 launch floor_cleaner full_system.launch.py mode:=navigation map:=/full/path/my_map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('floor_cleaner'), 'launch')

    default_map = os.path.join(
        os.path.expanduser('~'), 'floor_cleaner_ws', 'src', 'floor_cleaner', 'maps', 'my_map.yaml')

    mode = LaunchConfiguration('mode')
    map_yaml = LaunchConfiguration('map')
    gui = LaunchConfiguration('gui')

    declare_args = [
        DeclareLaunchArgument('mode', default_value='slam',
                              description='"slam" or "navigation"'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Map yaml used in navigation mode'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo GUI'),
    ]

    is_slam = IfCondition(PythonExpression(["'", mode, "' == 'slam'"]))
    is_nav = IfCondition(PythonExpression(["'", mode, "' == 'navigation'"]))

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'simulation.launch.py')),
        launch_arguments={'gui': gui}.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam.launch.py')),
        condition=is_slam,
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
        launch_arguments={'map': map_yaml, 'set_initial_pose': 'true'}.items(),
        condition=is_nav,
    )

    # Give Gazebo a few seconds to load the world and spawn the robot
    # before the mapping / navigation software starts.
    delayed_stack = TimerAction(period=8.0, actions=[slam, navigation])

    return LaunchDescription(declare_args + [simulation, delayed_stack])
