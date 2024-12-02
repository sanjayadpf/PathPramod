import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
import math
import yaml
import numpy as np
import sys
from pathlib import Path

class NavigateRobotNode(Node):
    def __init__(self, yaml_path):
        super().__init__('navigate_robot')
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacles_pub = self.create_publisher(MarkerArray, '/obstacles', 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Load obstacles from YAML
        self.obstacles = self.load_obstacles(yaml_path)

        # Publish obstacles for visualization
        self.publish_obstacles()

        # Robot state
        self.robot_pose = [0.0, 0.0]  # Assume [x, y] for simplicity
        self.current_goal = None
        self.current_path = None

    def load_obstacles(self, yaml_path):
        """Load obstacles from a specified YAML file."""
        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
            self.get_logger().info(f"Successfully loaded obstacles from: {yaml_path}")
            return [{'x': obs['x'], 'y': obs['y'], 'radius': obs['radius']} for obs in data['obstacles']]
        except Exception as e:
            self.get_logger().error(f"Error loading obstacles: {e}")
            return []

    def publish_obstacles(self):
        """Publish obstacle markers for RViz."""
        marker_array = MarkerArray()
        for i, obs in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = obs['x']
            marker.pose.position.y = obs['y']
            marker.pose.position.z = 0.0  # Flat obstacles
            marker.scale.x = obs['radius'] * 2  # Diameter
            marker.scale.y = obs['radius'] * 2  # Diameter
            marker.scale.z = 0.1  # Height for visualization
            marker.color.a = 0.8  # Transparency
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.obstacles_pub.publish(marker_array)

    def goal_callback(self, msg):
        """Handle a new goal."""
        self.get_logger().info("Received a new goal.")
        self.current_goal = [msg.pose.position.x, msg.pose.position.y]
        self.current_path = self.plan_path(self.robot_pose, self.current_goal)

        if not self.current_path:
            self.get_logger().warn("No valid path found to the goal.")

    def control_loop(self):
        """Control loop to move the robot along the path."""
        if self.current_path:
            next_point = self.current_path.pop(0)
            self.move_towards(next_point)

    def move_towards(self, target):
        """Move the robot towards the target point."""
        dx = target[0] - self.robot_pose[0]
        dy = target[1] - self.robot_pose[1]
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance < 0.1:  # Close enough to the target
            return

        # Compute velocity
        angle = math.atan2(dy, dx)
        twist = Twist()
        twist.linear.x = min(0.5, distance)  # Scale speed based on distance
        twist.angular.z = angle  # Simplified angular control

        self.cmd_vel_pub.publish(twist)

    def plan_path(self, start, goal):
        """Plan a path using A* algorithm."""
        grid_size = 100
        resolution = 0.1
        grid = np.zeros((grid_size, grid_size))
        for obs in self.obstacles:
            self.add_obstacle_to_grid(grid, obs, resolution)

        start_grid = self.world_to_grid(start, grid_size, resolution)
        goal_grid = self.world_to_grid(goal, grid_size, resolution)

        path = self.a_star(grid, start_grid, goal_grid)
        if path:
            return [self.grid_to_world(p, grid_size, resolution) for p in path]
        return None

    def add_obstacle_to_grid(self, grid, obstacle, resolution):
        """Add obstacles to the grid."""
        cx, cy = self.world_to_grid([obstacle['x'], obstacle['y']], grid.shape[0], resolution)
        radius = int(obstacle['radius'] / resolution)
        for x in range(max(0, cx - radius), min(grid.shape[0], cx + radius + 1)):
            for y in range(max(0, cy - radius), min(grid.shape[1], cy + radius + 1)):
                if math.sqrt((x - cx) ** 2 + (y - cy) ** 2) <= radius:
                    grid[x, y] = 1  # Mark as obstacle

    def a_star(self, grid, start, goal):
        """A* algorithm implementation."""
        from heapq import heappop, heappush

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        open_list = []
        heappush(open_list, (0, start))
        came_from = {}
        cost_so_far = {tuple(start): 0}

        while open_list:
            _, current = heappop(open_list)
            if current == goal:
                path = []
                while current != start:
                    path.append(current)
                    current = came_from[tuple(current)]
                path.reverse()
                return path

            neighbors = [
                [current[0] + 1, current[1]],
                [current[0] - 1, current[1]],
                [current[0], current[1] + 1],
                [current[0], current[1] - 1],
            ]
            for neighbor in neighbors:
                x, y = neighbor
                if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1] and grid[x, y] == 0:
                    new_cost = cost_so_far[tuple(current)] + 1
                    if tuple(neighbor) not in cost_so_far or new_cost < cost_so_far[tuple(neighbor)]:
                        cost_so_far[tuple(neighbor)] = new_cost
                        priority = new_cost + heuristic(goal, neighbor)
                        heappush(open_list, (priority, neighbor))
                        came_from[tuple(neighbor)] = current
        return None

    def world_to_grid(self, point, grid_size, resolution):
        """Convert world coordinates to grid coordinates."""
        return [int(point[0] / resolution), int(point[1] / resolution)]

    def grid_to_world(self, grid_point, grid_size, resolution):
        """Convert grid coordinates to world coordinates."""
        return [grid_point[0] * resolution, grid_point[1] * resolution]

def main():
    rclpy.init()

    if len(sys.argv) < 2:
        print("Usage: ros2 run <package_name> <node_name> <yaml_path>")
        return

    yaml_path = sys.argv[1]
    if not Path(yaml_path).exists():
        print(f"Error: File {yaml_path} does not exist.")
        return

    node = NavigateRobotNode(yaml_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
