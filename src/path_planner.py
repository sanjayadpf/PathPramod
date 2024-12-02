import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.get_logger().info("Path Planner Initialized")

    def plan_path(self, goal_pose: PoseStamped, obstacles: list):
        """
        Greedy path planner that generates a path avoiding obstacles.

        Parameters:
        goal_pose (PoseStamped): The goal position with x, y.
        obstacles (list): A list of obstacles with x, y, and radius.

        Returns:
        list: A list of waypoints [(x1, y1), (x2, y2), ...] to follow towards the goal.
        """
        # Placeholder for the start position, assuming robot starts at (0, 0)
        start_x, start_y = 0.0, 0.0

        # Define the start and goal positions
        start = (start_x, start_y)
        goal = (goal_pose.pose.position.x, goal_pose.pose.position.y)

        # Generate the path
        path = self.greedy_path_planner(start, goal, obstacles)
        
        if path:
            self.get_logger().info(f"Generated path: {path}")
        else:
            self.get_logger().warn("No valid path found!")

        return path

    def greedy_path_planner(self, start, goal, obstacles):
        """
        Generates a path that avoids obstacles.

        Parameters:
        start (tuple): Starting position (x, y).
        goal (tuple): Goal position (x, y).
        obstacles (list): A list of obstacles with x, y, and radius.

        Returns:
        list: A list of waypoints [(x1, y1), ...].
        """
        path = [start]
        current_position = start
        max_step_size = 0.2  # Max step size, you can adjust this value

        while True:
            # Calculate direction to goal
            dx = goal[0] - current_position[0]
            dy = goal[1] - current_position[1]
            distance_to_goal = math.hypot(dx, dy)

            # If close enough to the goal, stop
            if distance_to_goal < max_step_size:
                path.append(goal)
                break

            # Move toward the goal
            angle_to_goal = math.atan2(dy, dx)
            step_size = min(distance_to_goal, max_step_size)  # Ensure step size does not exceed max
            new_x = current_position[0] + step_size * math.cos(angle_to_goal)
            new_y = current_position[1] + step_size * math.sin(angle_to_goal)

            # Check for obstacles
            if self.is_obstacle_in_path(new_x, new_y, obstacles):
                self.get_logger().info(f"Obstacle detected near ({new_x}, {new_y}). Recalculating path...")
                new_x, new_y = self.avoid_obstacle(current_position, goal, obstacles, max_step_size)

            path.append((new_x, new_y))
            current_position = (new_x, new_y)

        return path

    def is_obstacle_in_path(self, x, y, obstacles):
        """
        Checks if a point (x, y) collides with any obstacle.

        Parameters:
        x (float): X-coordinate.
        y (float): Y-coordinate.
        obstacles (list): A list of obstacles with x, y, and radius.

        Returns:
        bool: True if the point collides, False otherwise.
        """
        for obstacle in obstacles:
            distance = math.hypot(x - obstacle["x"], y - obstacle["y"])
            if distance < obstacle["radius"]:
                return True
        return False

    def avoid_obstacle(self, current_position, goal, obstacles, max_step_size):
        """
        Adjusts the robot's path to avoid obstacles.

        Parameters:
        current_position (tuple): Current position (x, y).
        goal (tuple): Goal position (x, y).
        obstacles (list): A list of obstacles with x, y, and radius.
        max_step_size (float): Maximum step size the robot can take.

        Returns:
        tuple: New position (x, y) avoiding the obstacle.
        """
        robot_radius = 0.5  # Robot's radius
        safe_distance = 0.2  # Additional buffer distance

        # Inflate obstacles
        inflated_obstacles = [
            {"x": obs["x"], "y": obs["y"], "radius": obs["radius"] + robot_radius + safe_distance}
            for obs in obstacles
        ]

        # Attempt to move around the obstacle
        angle_increment = math.pi / 6  # 30-degree increments
        for i in range(-3, 4):  # Try left and right turns
            avoid_angle = math.atan2(goal[1] - current_position[1], goal[0] - current_position[0]) + i * angle_increment
            step_size = max_step_size  # Maintain max step size even when avoiding
            new_x = current_position[0] + step_size * math.cos(avoid_angle)
            new_y = current_position[1] + step_size * math.sin(avoid_angle)

            if not self.is_obstacle_in_path(new_x, new_y, inflated_obstacles):
                return new_x, new_y

        # If no path is clear, stay in place
        self.get_logger().warn("Failed to find a clear path. Stopping.")
        return current_position
