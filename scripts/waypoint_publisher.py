import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class RectanglePathPlanner(Node):

    def __init__(self):
        super().__init__('rectangle_path_planner')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(Bool, '/goal_reached', self.goal_completion_callback, 10)
        self.waypoints = [
            (0, 0),
            (-4, 0),
            (-4,4),
            (0,4),
            (0, 0),
        ]
        self.current_waypoint_index = 0
        self.publish_next_goal()

    def goal_completion_callback(self, msg):
        if msg.data and self.current_waypoint_index < len(self.waypoints) - 1:
            self.current_waypoint_index += 1
            self.publish_next_goal()

    def publish_next_goal(self):
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = float(self.waypoints[self.current_waypoint_index][0])
        pose_msg.pose.position.y = float(self.waypoints[self.current_waypoint_index][1])
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(pose_msg)
        self.get_logger().info(f"Publishing waypoint {self.current_waypoint_index + 1}")

def main(args=None):
    rclpy.init(args=args)
    node = RectanglePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
