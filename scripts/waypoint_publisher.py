import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped, Point, Quaternion

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.goal_reached = True  # Assume the goal is reached initially
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(String, '/goal_status', self.goal_reached_callback, 10)
        self.waypoints = []  # Initialize an empty list of waypoints
        self.enqueue_waypoints()  # Queue up the waypoints

        # Publish the first waypoint as soon as the script starts
        if self.waypoints:
            self.publisher.publish(self.waypoints[0])

    def goal_reached_callback(self, msg):
        if msg.data == "Goal reached!" and self.waypoints:
            self.get_logger().info("Goal reached. Publishing the next set of points...")

            # Dequeue and publish the next waypoint
            waypoint = self.waypoints.pop(0)
            self.publisher.publish(waypoint)

    def enqueue_waypoints(self):
        # Define the waypoints to be enqueued to form a rectangle with length 3 and breadth 4
        waypoints = []

        # # Start at (0, 0)
        # waypoint = PoseStamped()
        # waypoint.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        # waypoint.pose.position = Point(x=0.0, y=0.0, z=0.0)
        # waypoint.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        # waypoints.append(waypoint)

        # Move to (0, 4)
        waypoint = PoseStamped()
        waypoint.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        waypoint.pose.position = Point(x=0.0, y=4.0, z=0.0)
        waypoint.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        waypoints.append(waypoint)

        # Move to (3, 4)
        waypoint = PoseStamped()
        waypoint.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        waypoint.pose.position = Point(x=2.0, y=4.0, z=0.0)
        waypoint.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        waypoints.append(waypoint)

        # Move back to (3, 0)
        waypoint = PoseStamped()
        waypoint.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        waypoint.pose.position = Point(x=2.0, y=0.0, z=0.0)
        waypoint.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        waypoints.append(waypoint)

        # Move back to (0, 0)
        waypoint = PoseStamped()
        waypoint.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        waypoint.pose.position = Point(x=0.0, y=0.0, z=0.0)
        waypoint.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        waypoints.append(waypoint)


        self.waypoints = waypoints

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
