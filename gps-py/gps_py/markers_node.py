import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import sys
class gps_publisher(Node):
    def __init__(self):
        super().__init__('gps_node')
        topic_name = self.declare_parameter('topic_name', '/gps/marker').value
        self.publisher = self.create_publisher(NavSatFix, topic_name, 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.lat = self.declare_parameter('lat', 0.0).value
        self.long = self.declare_parameter('long', 0.0).value


    def timer_callback(self):
        msg = NavSatFix()
        msg.latitude = self.lat
        msg.longitude = self.long
        msg.altitude = 0.0
        self.publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    gps_node = gps_publisher()
    rclpy.spin(gps_node)
    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
