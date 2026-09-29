import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_path = get_package_share_directory('floor_cleaner')

    urdf_path = os.path.join(
        package_path,
        'urdf',
        'floor_cleaner.urdf'
    )

    world_path = os.path.join(
        package_path,
        'worlds',
        'floor.world'
    )

    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    # Start Gazebo directly with ROS plugins
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
            world_path
        ],
        output='screen'
    )

    # Publish robot TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description
            }
        ]
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic',
            'robot_description',
            '-entity',
            'floor_cleaner'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])