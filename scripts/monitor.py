#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile

class GoalMonitorNode(Node):
    def __init__(self):
        super().__init__('goal_monitor_node')
        self.distance_threshold = 0.7  # Adjust this value based on your requirements
        self.goal_position = None
        self.goal_reached = False
        self.goal_status_publisher = self.create_publisher(String, '/goal_status', QoSProfile(depth=10))

        # Subscribe to the filtered odometry topic
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odometry_callback, 10)
        
        # Subscribe to the goal pose topic
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        
        # Define the file name and location
        self.file_path = 'goal_status.txt'
        self.file_handle = open(self.file_path, 'w')

    def odometry_callback(self, msg):
        if self.goal_position is not None and not self.goal_reached:
            robot_position = msg.pose.pose.position

            # Calculate the distance between the robot and the current goal
            distance_to_goal = math.sqrt((robot_position.x - self.goal_position.x) ** 2 +
                                        (robot_position.y - self.goal_position.y) ** 2)

            if distance_to_goal < self.distance_threshold:
                if not self.goal_reached:
                    self.goal_status_publisher.publish(String(data="Goal reached!"))
                    self.goal_reached = True
                    self.get_logger().info("Goal reached!")
                    self.write_goal_status("Goal reached!")
            else:
                if self.goal_reached:
                    self.goal_status_publisher.publish(String(data="Goal not reached"))
                    self.goal_reached = False
                    self.get_logger().info("Goal not reached")
                    self.write_goal_status("Goal not reached")

    def goal_pose_callback(self, msg):
        self.goal_position = msg.pose.position
        self.goal_status_publisher.publish(String(data="Goal not reached"))
        self.goal_reached = False
        self.get_logger().info("New goal set")
        self.write_goal_status("New goal set")

    def write_goal_status(self, status):
        self.file_handle.write(f"{status}\n")
        self.file_handle.flush()

def main(args=None):
    rclpy.init(args=args)
    node = GoalMonitorNode()
    
    rclpy.spin(node)
    node.file_handle.close()  # Close the file before shutting down
    rclpy.shutdown()

if __name__ == '__main__':
    main()
