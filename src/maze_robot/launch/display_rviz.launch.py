from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # =====================================================
    # Robot description (Xacro → URDF)
    # =====================================================
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('maze_robot'),   # Correct package name
            'urdf',
            'robot.xacro'                     # Correct folder
        ])
    ])

    # =====================================================
    # Robot State Publisher
    # Publishes TF tree from robot_description
    # =====================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True              # IMPORTANT for Gazebo
        }]
    )

    # =====================================================
    # RViz2
    # =====================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
