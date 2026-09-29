"""
cleaning.launch.py
------------------
Starts the zig-zag cleaning_pattern_node. Nav2 (navigation.launch.py or
full_system.launch.py mode:=navigation) must already be running and the
robot must be localised.

Usage:
  ros2 launch floor_cleaner cleaning.launch.py
  ros2 launch floor_cleaner cleaning.launch.py dry_run:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('floor_cleaner')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    dry_run = LaunchConfiguration('dry_run')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'cleaning_pattern.yaml'),
            description='Cleaning area / lane parameters'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true = only show the planned path'),

        Node(
            package='floor_cleaner',
            executable='cleaning_pattern_node.py',
            name='cleaning_pattern_node',
            output='screen',
            parameters=[params_file,
                        {'use_sim_time': use_sim_time, 'dry_run': dry_run}],
        ),
    ])
