import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # =========================================================
    # Robot and package identifiers
    # =========================================================

    robot_name = 'differential_drive_robot'
    description_package_name = 'maze_robot'

    # =========================================================
    # Robot description (Xacro → URDF)
    # =========================================================

    xacro_relative_path = 'urdf/robot.xacro'
    xacro_file_path = os.path.join(
        get_package_share_directory(description_package_name),
        xacro_relative_path
    )

    robot_description_xml = xacro.process_file(
        xacro_file_path
    ).toxml()

    # =========================================================
    # Gazebo (gz-sim) launch
    # =========================================================

    gazebo_launch_source = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )

    gazebo_launch = IncludeLaunchDescription(
        gazebo_launch_source,
        launch_arguments={
            'gz_args': '-r -v -v4 empty.sdf',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # =========================================================
    # Spawn robot into Gazebo
    # =========================================================

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description'
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
    # EKF (robot_localization)  ★ ADDED ★
    # =========================================================

    ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[
        os.path.join(
            get_package_share_directory(description_package_name),
            'config',
            'ekf.yaml'
        ),
        {'use_sim_time': True}
    ]
)


    # =========================================================
    # Launch description assembly
    # =========================================================

    launch_description = LaunchDescription()

    launch_description.add_action(gazebo_launch)
    launch_description.add_action(spawn_robot_node)
    launch_description.add_action(robot_state_publisher_node)
    launch_description.add_action(gazebo_ros_bridge_node)
    launch_description.add_action(ekf_node)

    return launch_description
