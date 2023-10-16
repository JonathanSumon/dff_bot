"""
This module contains a ROS 2 node for monitoring a robot's goal and publishing goal status.
Author: Jonathan Sumon
Date: Oct 12, 2023
Email: jonathansumon@gmail.com
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile

class GoalMonitorNode(Node):
    """Node for monitoring robot goal and publishing goal status."""

    def __init__(self):
        super().__init__('goal_monitor_node')
        self.distance_threshold = 0.4  # Adjustable to requirements.
        self.goal_position = None
        self.goal_reached = False
        self.goal_status_publisher = self.create_publisher(String, '/goal_status', QoSProfile(depth=10))

        # Subscribe to the filtered odometry topic
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odometry_callback, 20)

        # Subscribe to the goal pose topic
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 20)

    def odometry_callback(self, msg):
        """Callback for odometry data."""
        if self.goal_position is not None and not self.goal_reached:
            robot_position = msg.pose.pose.position

            # Calculate the distance between the robot and the current goal
            distance_to_goal = math.sqrt(
                (robot_position.x - self.goal_position.x) ** 2 +
                (robot_position.y - self.goal_position.y) ** 2
            )


            if distance_to_goal < self.distance_threshold:
                if not self.goal_reached:
                    self.goal_status_publisher.publish(String(data="Goal reached!"))
                    self.goal_reached = True
                    self.get_logger().info("Goal reached!")
            else:
                if self.goal_reached:
                    self.goal_status_publisher.publish(String(data="Goal not reached"))
                    self.goal_reached = False
                    self.get_logger().info("Goal not reached")

    def goal_pose_callback(self, msg):
        """Callback for goal pose data."""
        self.goal_position = msg.pose.position
        self.goal_status_publisher.publish(String(data="Goal not reached"))
        self.goal_reached = False
        self.get_logger().info("New goal set")

def main(args=None):
    rclpy.init(args=args)
    node = GoalMonitorNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
