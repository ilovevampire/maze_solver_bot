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

    # Robot name (must match the <robot name=""> in robot.xacro)
    robot_name = 'differential_drive_robot'

    # ROS 2 description package name
    description_package_name = 'maze_robot'

    # =========================================================
    # Robot description (Xacro → URDF)
    # =========================================================

    # Relative path to robot Xacro file inside the package
    xacro_relative_path = 'urdf/robot.xacro'

    # Absolute path to the Xacro file
    xacro_file_path = os.path.join(
        get_package_share_directory(description_package_name),
        xacro_relative_path
    )

    # Process Xacro and generate robot_description XML
    robot_description_xml = xacro.process_file(
        xacro_file_path
    ).toxml()

    # =========================================================
    # Gazebo (gz-sim) launch
    # =========================================================

    # Path to Gazebo simulator launch file
    gazebo_launch_source = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )

    # Launch Gazebo with empty world
    gazebo_launch = IncludeLaunchDescription(
        gazebo_launch_source,
        launch_arguments={
            'gz_args': '-r -v -v4 empty.sdf',  # run, verbose, empty world
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
            '-name', robot_name,              # Name of spawned model
            '-topic', 'robot_description'     # Robot description topic
        ],
        output='screen'
    )

    # =========================================================
    # Robot State Publisher
    # Publishes TF using robot_description
    # =========================================================

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_xml,
                'use_sim_time': True            # Use Gazebo simulation clock
            }
        ]
    )

    # =========================================================
    # ROS ↔ Gazebo bridge configuration
    # =========================================================

    # Path to bridge parameter YAML file
    bridge_config_file = os.path.join(
        get_package_share_directory(description_package_name),
        'config',
        'bridge_param.yaml'
    )

    # Start parameter bridge node
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
    # Launch description assembly
    # =========================================================

    launch_description = LaunchDescription()

    # Start Gazebo simulator
    launch_description.add_action(gazebo_launch)

    # Spawn robot model
    launch_description.add_action(spawn_robot_node)

    # Publish robot state (TF)
    launch_description.add_action(robot_state_publisher_node)

    # Start ROS–Gazebo bridge
    launch_description.add_action(gazebo_ros_bridge_node)

    return launch_description
