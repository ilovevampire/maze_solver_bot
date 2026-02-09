import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # =========================================================
    # Robot & package info
    # =========================================================
    robot_name = 'differential_drive_robot'
    description_package_name = 'maze_robot'

    # =========================================================
    # Robot description (Xacro → URDF)
    # =========================================================
    xacro_file_path = os.path.join(
        get_package_share_directory(description_package_name),
        'urdf',
        'robot.xacro'
    )

    robot_description_xml = xacro.process_file(
        xacro_file_path
    ).toxml()

    # =========================================================
    # Gazebo world (Fuel maze)
    # =========================================================
    world_file = os.path.join(
        get_package_share_directory('gazebo_world'),
        'worlds',
        'maze_fuel.world'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # =========================================================
    # Spawn robot at maze entrance
    # =========================================================
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',

            # ---- Spawn pose (maze entrance) ----
            '-x', '0.51',
            '-y', '5.6',
            '-z', '0.2',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '-2.11'
        ],
        output='screen'
    )

    # =========================================================
    # Robot State Publisher
    # =========================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_xml,
                'use_sim_time': True
            }
        ]
    )

    # =========================================================
    # ROS ↔ Gazebo bridge
    # =========================================================
    bridge_config_file = os.path.join(
        get_package_share_directory(description_package_name),
        'config',
        'bridge_param.yaml'
    )

    gazebo_ros_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config_file}',
        ],
        output='screen'
    )

    # =========================================================
    # Launch
    # =========================================================
    return LaunchDescription([
        gazebo_launch,
        spawn_robot_node,
        robot_state_publisher_node,
        gazebo_ros_bridge_node
    ])
