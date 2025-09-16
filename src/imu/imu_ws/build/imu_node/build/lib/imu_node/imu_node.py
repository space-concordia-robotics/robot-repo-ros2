import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import serial

class imu_publisher(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_IMU = self.create_publisher(Imu, 'IMU', 10)
        self.publisher_MAG = self.create_publisher(MagneticField, 'Mag', 10)


        timer_period = 0.1
        self.baud = self.declare_parameter('baud', 115200).value
        self.serial_port = self.declare_parameter('serial_port', '/dev/ttyUSB0').value
        self.ser = serial.Serial(self.serial_port, self.baud)
        self.timer =  self.create_timer(timer_period, self.timer_callback)

    
    def timer_callback(self):
        data = self.ser.readline().decode('latin-1').strip()
        if data:
            self.get_logger().info(f"Received: {data}")
            ar = data.split(' ')
            values = [float(e) for e in ar if e != ' ']
            
            IMU_converted = Imu()

            IMU_converted.header.stamp = self.get_clock().now().to_msg()
            IMU_converted.header.frame_id = "IMU"

            IMU_converted.orientation_covariance[0] = -1.0

            IMU_converted.angular_velocity = Vector3(x = values[6], y = values[7], z = values[8])
            IMU_converted.angular_velocity_covariance = [0.0]*9

            IMU_converted.linear_acceleration = Vector3(x = values[3], y = values[4], z = values[5])
            IMU_converted.linear_acceleration_covariance = [0.0]*9

        
            MAG_converted = MagneticField()
            
            MAG_converted.header.stamp = self.get_clock().now().to_msg()
            MAG_converted.header.frame_id = "Mag"

            MAG_converted.magnetic_field = Vector3(x = values[0], y = values[1], z = values[2])
            MAG_converted.magnetic_field_covariance = [0.0]*9
            

            self.publisher_IMU.publish(IMU_converted)
            self.publisher_MAG.publish(MAG_converted)

def main(args = None):
    rclpy.init(args=args)
    imu_node = imu_publisher()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
