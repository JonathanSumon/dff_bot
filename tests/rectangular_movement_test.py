# Author: Jonathan Sumon
# Date: Oct 15, 2023
# Description: Launch a basic mobile robot URDF file using Rviz for DFF
# jonathansumon@gmail.com

import rclpy
from nav_msgs.msg import Odometry
import math
import time

# Define the test function
def test_rectangular_movement(node):
    # Define a list of expected waypoints for a rectangular path
    waypoints = [
        (0.0, 0.0),
        (0.0, 4.0),
        (2.0, 4.0),
        (2.0, 0.0),
        (0.0, 0.0)
    ]

    current_waypoint = 0
    max_distance_deviation = 1.0  # Adjust this value as needed
    max_consecutive_failures = 3  # Maximum allowable consecutive failures

    # Initialize variables to track accumulated distance
    accumulated_distance = 0.0
    last_x, last_y = 0.0, 0.0
    failure_count = 0

    # Callback function to handle odometry messages
    def odom_callback(msg):
        nonlocal current_waypoint, accumulated_distance, last_x, last_y, failure_count
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Calculate the distance traveled since the last odom callback
        distance_traveled = math.sqrt((x - last_x) ** 2 + (y - last_y) ** 2)

        # Update the accumulated distance
        accumulated_distance += distance_traveled

        # Update the last known position
        last_x, last_y = x, y

        # Check if the robot is close to the expected waypoint
        expected_x, expected_y = waypoints[current_waypoint]
        distance_to_waypoint = math.sqrt((x - expected_x) ** 2 + (y - expected_y) ** 2)

        if distance_to_waypoint < max_distance_deviation:
            current_waypoint += 1
            failure_count = 0
            print(f"Reached waypoint {current_waypoint}: ({x}, {y})")

        # If the robot fails to reach the expected waypoint within a reasonable number of failures, consider it a failure
        elif failure_count >= max_consecutive_failures:
            print(f"Failed to reach waypoint {current_waypoint}: ({x}, {y})")
            node.get_logger().error("Test case failed: Robot deviated from the expected path.")
            rclpy.shutdown()

        # If the robot has followed the entire path, the test passes
        if current_waypoint == len(waypoints) - 1:
            print("Robot has completed the rectangular path. Test PASSED.")
            node.get_logger().info("Robot has completed the rectangular path. Test PASSED.")
            rclpy.shutdown()

    # Subscribe to the odometry topic
    node.create_subscription(Odometry, '/odometry/filtered', odom_callback, 0)

    # Spin the node to run the test
    rclpy.spin(node)

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('rectangular_movement_test_node')
    test_rectangular_movement(node)
