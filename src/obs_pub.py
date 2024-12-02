import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import os

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        
        # Declare and get the parameter for the YAML file path
        self.declare_parameter('yaml_file', 'obs1.yaml') #obs1.yaml is in current ditectory
        yaml_file = self.get_parameter('yaml_file').get_parameter_value().string_value
        
        if not yaml_file or not os.path.exists(yaml_file):
            self.get_logger().error(f"YAML file '{yaml_file}' not found or not provided!")
            self.obstacles = []
        else:
            self.obstacles = self.load_yaml(yaml_file)
        
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacles', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)  # Publish every second
        
        self.get_logger().info(f"Loaded obstacles: {self.obstacles}")
        
    def load_yaml(self, yaml_file):
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
            return data.get('obstacles', [])
        
    def publish_markers(self):
        marker_array = MarkerArray()
        
        for i, obstacle in enumerate(self.obstacles):
            x = obstacle['x']
            y = obstacle['y']
            radius = obstacle['radius']
            
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0  # Centered on the floor
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2 * radius  # Diameter in x
            marker.scale.y = 2 * radius  # Diameter in y
            marker.scale.z = 0.1  # Thin cylinder
            marker.color.a = 1.0  # Fully opaque
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
