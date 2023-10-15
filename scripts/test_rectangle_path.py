import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class RectangleTestNode(Node):
    def __init__(self):
        super().__init__('rectangle_test_node')
        self.robot_positions = []
        self.length = 3.0  # Length of the rectangle
        self.breadth = 4.0  # Breadth of the rectangle
        self.position_tolerance = 0.1  # Tolerance for position comparison
        self.points_to_collect = 4  # Number of points to collect for a full rectangle

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odometry_callback, 10)

    def odometry_callback(self, msg):
        robot_position = msg.pose.pose.position
        self.robot_positions.append(robot_position)

        # Check if we have collected enough points to make a full rectangle
        if len(self.robot_positions) >= self.points_to_collect:
            if self.has_moved in a rectangle():
                self.get_logger().info("Robot has moved in a rectangle.")
            else:
                self.get_logger().info("Robot has not moved in a rectangle.")
            self.destroy_node()

    def has_moved_in_a_rectangle(self):
        # Calculate the expected positions for the corners of the rectangle
        expected_positions = [
            (0, 0),
            (0, self.breadth),
            (self.length, self.breadth),
            (self.length, 0)
        ]

        # Check if the robot positions are within tolerance of the expected positions
        for i, position in enumerate(self.robot_positions):
            expected_position = expected_positions[i]
            distance = math.sqrt((position.x - expected_position[0]) ** 2 + (position.y - expected_position[1]) ** 2)
            if distance > self.position_tolerance:
                return False
        return True

def main(args=None):
    rclpy.init(args=args)
    node = RectangleTestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
