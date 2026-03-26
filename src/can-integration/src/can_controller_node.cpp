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


void CanControllerNode::getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state_msg){

    if (joint_state_msg->velocity.size() < 7) {
        RCLCPP_ERROR(this->get_logger(), "Received JointState message with insufficient velocity data. Joint size is %zu, expected at least 7.", joint_state_msg->velocity.size());
        return;
    }

    // Create a buffer to send motor commands
    uint8_t out_buf[1 + 1 + sizeof(float) * 6 + 1] = {}; // Correct buffer size
    out_buf[0] = 0x4E;
    out_buf[1] = sizeof(float) * 6;

    // Map JointState velocities to motor speeds
    for (int i = 0; i < 6; i++) {
        float speed = static_cast<float>(joint_state_msg->velocity[i]) * 1024.f;
        memcpy(&out_buf[(i * sizeof(float)) + 2], &speed, sizeof(float));
    }
    out_buf[27] = 0x0A; // End of message

    // Send the motor commands to be processed by framer
   

}



int main(int argc, char *argv[]){

    rclcpp::init(argc, argv); 

    auto node = std::make_shared<CanControllerNode>(); 
    rclcpp::spin(node); 


    rclcpp::shutdown(); 
    return 0; 

}