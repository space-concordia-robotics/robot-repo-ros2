import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import yaml

class KeymapNode(Node):
    def __init__(self):
        super().__init__('keymap_node')
        self.declare_parameter('keymap_file', 'config/keymap.yaml')
        keymap_file = self.get_parameter('keymap_file').get_parameter_value().string_value

        self.get_logger().info(f'Loading keymap from {keymap_file}')
        with open(keymap_file, 'r') as f:
            self.keymap = yaml.safe_load(f)
    
        self.marker_pub = self.create_publisher(MarkerArray, 'keymap_markers', 10)
        self.markers = MarkerArray()

        for i, (key, pos) in enumerate(self.keymap.items()):
            #Cube marker for each key
            m = Marker()
            m.header.frame_id = "keyboard"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = pos['x']
            m.pose.position.y = pos['y']
            m.pose.position.z = pos['z_top']
            m.scale.x = pos.get('width')    #size for keycaps
            m.scale.y = 0.018
            m.scale.z = 0.005
            m.color.r, m.color.g, m.color.b, m.color.a = (0.0, 0.5, 1.0, 0.8)
            m.lifetime.sec = 0
            self.markers.markers.append(m)

            #Text marker for each key
            t = Marker()
            t.header.frame_id = "keyboard"
            t.id = i + 1000  # need different IDs for each marker
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = pos['x']
            t.pose.position.y = pos['y']
            t.pose.position.z = pos['z_top'] + 0.001
            t.scale.x = 0.01
            t.scale.z = 0.01
            t.color.r, t.color.g, t.color.b, t.color.a = (1.0, 1.0, 1.0, 1.0)
            t.text = key  # Display the key label
            t.lifetime.sec = 0
            self.markers.markers.append(t)

        self.timer = self.create_timer(1.0, self.publish_markers)

    
    def publish_markers(self):
        for m in self.markers.markers:
            m.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.markers)

def main(args=None):
    rclpy.init(args=args)
    node = KeymapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()