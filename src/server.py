import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import MarkerArray
from path_planner import PathPlanner
from navigator import Navigator
from rclpy.qos import QoSProfile

class PathPlanningServer(Node):
    def __init__(self):
        super().__init__('path_planning_server')

        # Create subscription to /goal_pose with detailed logging
        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped,  # Corrected to PoseStamped
            '/goal_pose', 
            self.goal_pose_callback, 
            QoSProfile(depth=10)  # Match the QoS settings with the publisher
        )

        # Create subscription to /obstacles topic
        self.obstacles_subscriber = self.create_subscription(
            MarkerArray, 
            '/obstacles', 
            self.obstacles_callback, 
            QoSProfile(depth=10)
        )

        # Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            QoSProfile(depth=10)
        )

        # Initialize the planner and navigator
        self.planner = PathPlanner()
        self.navigator = Navigator(self.velocity_publisher)

        # Default initializations
        self.goal_pose = None
        self.obstacles = []

    def goal_pose_callback(self, msg: PoseStamped):
        """
        Callback for receiving the goal pose.
        """
        # Print the received goal pose data
        print(f"Received goal pose: x={msg.pose.position.x}, y={msg.pose.position.y}")

        # Log the received goal pose
        self.get_logger().info(f"Received goal pose: x={msg.pose.position.x}, y={msg.pose.position.y}")

        # Store the received goal pose
        self.goal_pose = msg

        # Start path planning if both obstacles and goal pose are received
        if self.obstacles and self.goal_pose:
            self.plan_and_navigate()

    def obstacles_callback(self, msg: MarkerArray):
        """
        Callback for receiving obstacles as a marker array.
        """
        self.obstacles = []
        for marker in msg.markers:
            # Assuming the markers are cylinders representing obstacles
            if marker.type == 3:  # CYLINDER
                obstacle = {
                    "x": marker.pose.position.x,
                    "y": marker.pose.position.y,
                    "radius": marker.scale.x / 2.0  # Assumes the obstacle radius is half the scale.x
                }
                self.obstacles.append(obstacle)

        # Start path planning if the goal pose is already set
        #if self.goal_pose:
        #    self.plan_and_navigate()

    def plan_and_navigate(self):
        """
        Plans the path using the path planner and then navigates using the navigator.
        """
        self.get_logger().info("Planning path...")
        path = self.planner.plan_path(self.goal_pose, self.obstacles)
        if path:
            self.get_logger().info(f"Generated path: {path}")
            self.navigator.navigate(path)
        else:
            self.get_logger().warn("No valid path found!")

def main(args=None):
    rclpy.init(args=args)
    path_planning_server = PathPlanningServer()
    rclpy.spin(path_planning_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
