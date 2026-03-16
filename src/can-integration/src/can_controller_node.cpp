#include "can_controller_node.hpp"


void CanControllerNode::getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr twist_msg){

    //get linear and angular motion in x, and y direction from /cmd_vel

    auto linear_y = twist_msg->linear.x; 
    auto angular_z = twist_msg->angular.z;

    //multiply them together for exponential control

    linear_y *= linear_y * linear_y; 
    angular_z *= angular_z * angular_z; 

    //calculate the wheel velocity and multiply them by a multiplier
    float distance_between_wheels = 1.2f;
    uint32_t right_wheel_velocity = -(linear_y - (angular_z*distance_between_wheels*0.5f)); 
    uint32_t left_wheel_velocity = -(linear_y + (angular_z*distance_between_wheels*0.5f)); 


    //convert velocities into rpm
    uint32_t right_wheel_velocity_rpm = right_wheel_velocity * 2000;
    uint32_t left_wheel_velocity_rpm = left_wheel_velocity * 2000;



    //send commands out to be processed by the framer 


    RCLCPP_INFO(this->get_logger(), "Wheel Motor Commands Sent: Right RPM = %.2f, Left RPM = %2.f", 
            right_wheel_velocity_rpm, left_wheel_velocity_rpm);
}



int main(int argc, char *argv[]){

    rclcpp::init(argc, argv); 

    auto node = std::make_shared<CanControllerNode>(); 
    rclcpp::spin(node); 


    rclcpp::shutdown(); 
    return 0; 

}