"""
simulation.launch.py
--------------------
Starts the "physical" part of the system:

  * Gazebo Classic (server + optional GUI) with the apartment world
  * robot_state_publisher  (URDF -> /robot_description + TF of the robot's links)
  * spawn_entity           (puts the robot into Gazebo)

After this launch file you should have:  /scan  /odom  /joint_states  /clock  /tf  /tf_static
and the robot reacts to /cmd_vel.

Usage:
  ros2 launch floor_cleaner simulation.launch.py
  ros2 launch floor_cleaner simulation.launch.py gui:=false        # headless
  ros2 launch floor_cleaner simulation.launch.py x_pose:=1.0 y_pose:=0.5
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('floor_cleaner')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(pkg_share, 'worlds', 'floor_cleaner_world.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'floor_cleaner.urdf.xacro')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    declare_args = [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Full path to the Gazebo world file'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo GUI (gzclient)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo /clock'),
        DeclareLaunchArgument('x_pose', default_value='0.0',
                              description='Robot spawn x [m]'),
        DeclareLaunchArgument('y_pose', default_value='0.0',
                              description='Robot spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description='Robot spawn heading [rad]'),
    ]

    # xacro -> URDF string. ParameterValue(..., value_type=str) stops ROS from
    # trying to parse the XML as YAML.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=true']),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # Gazebo server (physics, sensors, ROS plugins). Also publishes /clock.
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
    )

    # Gazebo window (optional)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    # Reads /robot_description and creates the robot inside Gazebo.
    # z = 0.01 drops the robot 1 cm so it never starts inside the floor.
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_floor_cleaner',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'floor_cleaner',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw,
        ],
    )

    return LaunchDescription(declare_args + [
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
    ])
