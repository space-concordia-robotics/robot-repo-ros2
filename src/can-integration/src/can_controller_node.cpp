#include "can_controller_node.hpp"
#include "prefixes.hpp"


static bool arm_command_extraction_enabled_ = true;
// static bool wheel_command_extraction_enabled_ = true; 
//------------------------------ Obtaining Joystick input from controller -------------------------------
void CanControllerNode::getjoyfeedback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg){
    
    constexpr int FORCE_STOP_COMPAT = 3; //Some random button
    constexpr int RESTART_ARM_MOTORS = 2; 
    // constexpr int FORCE_STOP_WHEELS = 6;
    // constexpr int RESTART_WHEEL_MOTORS = 7; 
    // constexpr int FORCE_STOP_BAB = 8; 
    // constexpr int RESTART_BAB = 9; 

    // each force stop and restart expression wil: have its on boolean value, and when activated, it will send a forceStop or resume request
    // to its respective motor
    static bool arm_force_stop_active = false;
    // static bool wheel_force_stop_active = false; 

    bool force_stop_compat = msg->buttons[FORCE_STOP_COMPAT] == 1 ;
    bool restart_arm_pressed = msg->buttons[RESTART_ARM_MOTORS] == 1; 

    // bool force_stop_wheels = msg->buttons[FORCE_STOP_WHEELS] == 1; 
    // bool restart_wheels = msg->buttons[RESTART_WHEEL_MOTORS] == 1;


    //Force stop active prevents the sendForceStop command to be called more than once. 
    //For command to execute, both conditions in if statement must be true. when force_stop_active 
    //is first called, it is false, therefore, !false = true, and so the command is executed. But, when it is true,
    //the condition becomes !true = false, hence the true && false conditions will cause the command to not be activated. 
    if(force_stop_compat && !arm_force_stop_active){
        arm_force_stop_active = true; 
        arm_command_extraction_enabled_ = false; 
        systemframebuilder_.sendForceStop(deviceType::DeviceType::COMPAT, DeviceId::ID::COMPAT_BOARD_ID); 
        RCLCPP_WARN(this->get_logger(), "Force stop arm motors request sent"); 
    } else if(restart_arm_pressed && arm_force_stop_active){
        arm_force_stop_active = false; 
        arm_command_extraction_enabled_ = true; 
        systemframebuilder_.sendResume(deviceType::DeviceType::COMPAT, DeviceId::ID::COMPAT_BOARD_ID); 
        RCLCPP_INFO(this->get_logger(), "Restart arm motors request sent"); 
    }

    // if(force_stop_wheels && !wheel_force_stop_active){

    //     wheel_force_stop_active = true; 
    //     wheel_command_extraction_enabled_ = false;
    //     systemframebuilder_.sendForceStop(deviceType::DeviceType::RELAY, DeviceId::ID::HUB);
    //     RCLCPP_WARN(this->get_logger(), "Wheel force stop request sent"); 

    // } else if(restart_wheels && wheel_force_stop_active){
    //     wheel_force_stop_active = false; 
    //     wheel_command_extraction_enabled_ = true; 
    //     systemframebuilder_.sendResume(deviceType::DeviceType::RELAY, DeviceId::ID::HUB); 
    //     RCLCPP_INFO(this->get_logger(), "Wheel resume request sent"); 
    // }
}

// ------------------------------ Obtainig Wheel twist Data from controller -------------------------------- 
// void CanControllerNode::getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg){

//     //extract linear and angular motion in x, and y direction from /cmd_vel
//     float linear_y = twist_msg->linear.x; 
//     float angular_z = twist_msg->angular.z;

//     //multiply them together for exponential control
//     linear_y *= linear_y * linear_y; 
//     angular_z *= angular_z * angular_z; 

//     //calculate the wheel velocity and multiply them by a multiplier
//     float distance_between_wheels = 1.2f;
//     float right_wheel_velocity = -(linear_y - (-angular_z*distance_between_wheels*0.5f)); 
//     float left_wheel_velocity = -(linear_y + (-angular_z*distance_between_wheels*0.5f)); 


//     //convert velocities into rpm
//     float right_wheel_velocity_rpm = right_wheel_velocity * this->multiplier;
//     float left_wheel_velocity_rpm = left_wheel_velocity * this->multiplier;


//     if(wheel_command_extraction_enabled_){
//         //sending the velocity commands out to be processed by the framer 
//         RevMotorController::velocityControl(1, right_wheel_velocity_rpm);
//         RevMotorController::velocityControl(2, right_wheel_velocity_rpm);
//         RevMotorController::velocityControl(3, right_wheel_velocity_rpm);

//         RevMotorController::velocityControl(4, left_wheel_velocity_rpm);
//         RevMotorController::velocityControl(5, left_wheel_velocity_rpm);
//         RevMotorController::velocityControl(6, left_wheel_velocity_rpm);
    
//         RCLCPP_INFO(this->get_logger(), "Wheel Motor Commands Sent: Right RPM = %.2f, Left RPM = %2.f", 
//             right_wheel_velocity_rpm, left_wheel_velocity_rpm);
//     }  
// }


// ---------------------------- Obtaining Joint State Data from controller -------------------------- 
void CanControllerNode::getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg){

    RCLCPP_INFO(this->get_logger(), "hello"); 
// @brief Publisher sends out JoinState topic data. Here, we subscribe to the Joinstate to extract oncoming velocity messgaes and then send em off
    if(joint_state_msg->velocity.size() < 5){
        RCLCPP_ERROR(this->get_logger(), "Received JointState message with insufficient velocity data. Joint size is %zu, expected at least 5.", joint_state_msg->velocity.size());
        return; 
    }

    static constexpr Instructions::Inst MOTOR_MAP[5] = {
        Instructions::Inst::ARM_MOTOR_1,
        Instructions::Inst::ARM_MOTOR_2,
        Instructions::Inst::ARM_MOTOR_3,
        Instructions::Inst::ARM_MOTOR_4,
        Instructions::Inst::ARM_MOTOR_5,
    };

    if(arm_command_extraction_enabled_){
        // sendMotorVelocity() takes in: 1. device type, 2. instruction, 3. motor id, 4. payload (velocities) 
        RCLCPP_INFO(this->get_logger(), "Extracting velocity data from JointState");
        for(size_t i = 0; i < 5; i++){

            const float velocity = static_cast<float>(joint_state_msg->velocity[i]); 
            systemframebuilder_.sendArmMotorVelocity(
                deviceType::DeviceType::COMPAT, 
                MOTOR_MAP[i], 
                DeviceId::ID::COMPAT_BOARD_ID, 
                velocity);
            
            RCLCPP_DEBUG(this->get_logger(), "Motor %zu → %.3f rad/s", i + 1, velocity);
        }
    }
}



int main(int argc, char *argv[]){

    rclcpp::init(argc, argv); 

    auto node = std::make_shared<CanControllerNode>(); 
    rclcpp::spin(node); 


    rclcpp::shutdown(); 
    return 0; 

}