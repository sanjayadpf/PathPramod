from geometry_msgs.msg import Twist
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class Navigator(Node):
    def __init__(self, velocity_publisher):
        super().__init__('navigator')
        self.velocity_publisher = velocity_publisher

        # Create a subscription to /pose to get current robot position
        self.pose_subscriber = self.create_subscription(
            Pose2D, 
            '/pose', 
            self.pose_callback, 
            10  # QoS depth of 10
        )

        # Initialize the current position of the robot
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

    def pose_callback(self, msg: Pose2D):
        """
        Callback function to update the robot's current position.
        """
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def navigate(self, path):
        """
        Navigate along the generated path.

        Parameters:
        path (list): A list of waypoints (x, y).
        """
        for i, (target_x, target_y) in enumerate(path):
            # Navigate to each waypoint
            self.get_logger().info(f"Navigating to waypoint: ({target_x}, {target_y})")
            
            while True:
                # Calculate the difference between current position and target
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                distance_to_target = math.hypot(dx, dy)

                # If the robot is close enough to the target, stop moving to this waypoint
                if distance_to_target < 0.05:
                    self.get_logger().info(f"Arrived at waypoint: ({target_x}, {target_y})")
                    break

                # Calculate the angle to the target
                target_angle = math.atan2(dy, dx)
                angle_diff = self.normalize_angle(target_angle - self.current_theta)

                # Align robot's orientation first
                if abs(angle_diff) > 0.05:  # Allow small angle tolerance
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.0  # Stop linear motion while turning
                    cmd_vel.angular.z = max(-0.5, min(0.5, angle_diff))  # Clamp angular speed
                    self.velocity_publisher.publish(cmd_vel)
                else:
                    # Move forward once aligned
                    cmd_vel = Twist()
                    cmd_vel.linear.x = min(0.5, distance_to_target)  # Linear speed towards the target
                    cmd_vel.angular.z = 0.0  # No angular motion while moving forward
                    self.velocity_publisher.publish(cmd_vel)

                # Sleep for a short time to avoid flooding the topic
                rclpy.spin_once(self)

        # Stop the robot after reaching the final waypoint
        self.correct_orientation()
        self.stop_robot()
        self.get_logger().info("Path complete. Robot stopped.")

    def correct_orientation(self):
        None
    
    def stop_robot(self):
        """
        Publishes a zero-velocity command to stop the robot.
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.velocity_publisher.publish(cmd_vel)

    def normalize_angle(self, angle):
        """
        Normalize the angle to be within [-pi, pi].

        Parameters:
        angle (float): The angle to normalize.

        Returns:
        float: The normalized angle.
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
