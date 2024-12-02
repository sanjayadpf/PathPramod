import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.follow_path)
        self.path = [
            (1.0, 1.0),  # Example coordinates (x, y)
            (2.0, 2.0),
            (3.0, 3.0)
        ]
        self.current_index = 0

    def follow_path(self):
        if self.current_index >= len(self.path):
            return

        x_goal, y_goal = self.path[self.current_index]

        # Current robot position (replace with actual robot position from TF or Odometry)
        x_current = 0.0
        y_current = 0.0
        theta_current = 0.0  # Assuming robot is facing in the x-direction

        # Compute distance to goal
        dx = x_goal - x_current
        dy = y_goal - y_current
        distance = math.sqrt(dx**2 + dy**2)

        # If we're close enough to the goal, move to the next point
        if distance < 0.1:
            self.current_index += 1
            return

        # Compute desired angle to the goal
        angle_to_goal = math.atan2(dy, dx)
        angular_velocity = angle_to_goal - theta_current
        linear_velocity = 0.5  # m/s

        # Create velocity message
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
