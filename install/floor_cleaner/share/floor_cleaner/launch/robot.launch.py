import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_path = get_package_share_directory('floor_cleaner')

    urdf_path = os.path.join(
        package_path,
        'urdf',
        'floor_cleaner.urdf'
    )

    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',

        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    return LaunchDescription([
        robot_state_publisher
    ])