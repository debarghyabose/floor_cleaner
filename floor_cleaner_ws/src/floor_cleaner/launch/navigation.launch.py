"""
navigation.launch.py
--------------------
Localisation + autonomous navigation on a SAVED map.
Run it while simulation.launch.py is running and SLAM is STOPPED.

Localisation (lifecycle_manager_localization):
  * map_server : loads my_map.yaml, publishes /map
  * amcl       : /scan + /map + odom TF  ->  TF map->odom (robot position on the map)

Navigation (lifecycle_manager_navigation):
  * controller_server : follows the path, publishes /cmd_vel   (local costmap inside)
  * planner_server    : computes the global path /plan        (global costmap inside)
  * behavior_server   : recovery behaviours (spin, back up, wait)
  * bt_navigator      : receives the goal (RViz "Nav2 Goal") and runs the behaviour tree

Plus RViz2 (optional).

Usage:
  ros2 launch floor_cleaner navigation.launch.py
  ros2 launch floor_cleaner navigation.launch.py map:=/full/path/to/other_map.yaml
  ros2 launch floor_cleaner navigation.launch.py set_initial_pose:=true   # robot is at the spawn pose
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('floor_cleaner')

    # The map is saved into the SOURCE folder (see README), so load it from there.
    default_map = os.path.join(
        os.path.expanduser('~'), 'floor_cleaner_ws', 'src', 'floor_cleaner', 'maps', 'my_map.yaml')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    set_initial_pose = LaunchConfiguration('set_initial_pose')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_args = [
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Full path to the map yaml file'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo /clock'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Nav2 parameter file'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Automatically configure + activate Nav2 nodes'),
        DeclareLaunchArgument(
            'set_initial_pose', default_value='false',
            description='true = AMCL assumes the robot is at the spawn pose (0,0,0). '
                        'false = set it yourself with "2D Pose Estimate" in RViz2'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz2'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'rviz', 'floor_cleaner.rviz'),
            description='RViz2 config file'),
    ]

    common = {'use_sim_time': use_sim_time}

    # ------------------------- Localisation -------------------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, common, {'yaml_filename': map_yaml}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, common, {'set_initial_pose': set_initial_pose}],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[common, {
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # ------------------------- Navigation -------------------------
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, common],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, common],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, common],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, common],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[common, {
            'autostart': autostart,
            'node_names': ['controller_server',
                           'planner_server',
                           'behavior_server',
                           'bt_navigator'],
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[common],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(declare_args + [
        map_server,
        amcl,
        lifecycle_manager_localization,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
        rviz_node,
    ])
