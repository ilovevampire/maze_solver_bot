#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class SquareMotion(Node):
    """
    This node commands the robot to move in a square trajectory
    using velocity commands published on /cmd_vel.
    """

    def __init__(self):
        # Initialize ROS 2 node
        super().__init__('square_motion_node')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # =====================================================
        # Motion configuration parameters
        # =====================================================

        # Linear speed while moving straight (m/s)
        self.linear_speed = 0.2

        # Length of each side of the square (meters)
        self.square_side_length = 1.0

        # Angular speed while turning (rad/s)
        self.angular_speed = 0.5

        # Turn angle for each corner (90 degrees)
        self.turn_angle_rad = math.pi / 2

        # Start square motion routine
        self.execute_square_motion()

    def move_straight(self):
        """
        Move the robot forward for one side of the square.
        """
        velocity_msg = Twist()
        velocity_msg.linear.x = self.linear_speed

        # Time required to move one side
        move_duration = self.square_side_length / self.linear_speed

        self.get_logger().info('Moving straight')
        start_time = time.time()

        while time.time() - start_time < move_duration:
            self.cmd_vel_publisher.publish(velocity_msg)
            time.sleep(0.05)

        self.stop_robot()

    def turn_90_degrees(self):
        """
        Rotate the robot 90 degrees in place.
        """
        velocity_msg = Twist()
        velocity_msg.angular.z = self.angular_speed

        # Time required to rotate 90 degrees
        turn_duration = self.turn_angle_rad / self.angular_speed

        self.get_logger().info('Turning 90 degrees')
        start_time = time.time()

        while time.time() - start_time < turn_duration:
            self.cmd_vel_publisher.publish(velocity_msg)
            time.sleep(0.05)

        self.stop_robot()

    def stop_robot(self):
        """
        Stop all robot motion.
        """
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)

        # Short pause to ensure robot fully stops
        time.sleep(0.5)

    def execute_square_motion(self):
        """
        Perform square motion: move straight and turn 4 times.
        """
        # Allow Gazebo, bridge, and controllers to initialize
        time.sleep(2.0)

        for side_index in range(4):
            self.get_logger().info(f'Starting side {side_index + 1}')
            self.move_straight()
            self.turn_90_degrees()

        self.get_logger().info('Square motion complete')


def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create and run the square motion node
    square_motion_node = SquareMotion()

    # Spin once since motion logic is time-based
    rclpy.spin_once(square_motion_node, timeout_sec=0)

    # Cleanup
    square_motion_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
