#include "ros2_wrapper/can_controller_node.hpp"
#include "can-utils/prefixes.hpp"




CanControllerNode::CanControllerNode(const rclcpp::NodeOptions& options) : 
    Node("can_controller_node", options), logger(this->get_logger().get_child("can_controller_node")){

        this->declare_parameter("can_path", "can0");
        multiplier = this->declare_parameter("multiplier", 500);

        can_interface_ = this->declare_parameter<std::string>("can_interface", "vcan0");

        can_controller_ = std::make_shared<can_util::CANController>(can_interface_, this->get_logger());
        if (!can_controller_->configureCan()) {
            logger.fatal("Failed to configure CAN on interface {}", can_interface_); 
            throw std::runtime_error("CAN configure failed");
        }
        
        parameter_event_handler = std::make_shared<rclcpp::ParameterEventHandler>(this);

        auto multiplier_callback = [this](const rclcpp::Parameter parameter) {
        if (parameter.get_name() == "multiplier")
        multiplier = parameter.as_int();
        };

        multiplier_callback_handle = parameter_event_handler->add_parameter_callback("multiplier", multiplier_callback);
        
        frame_builder_ = std::make_unique<SystemFrameBuilder>(can_controller_);

        

        //intiallize subscriptions 

        twist_msgs_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS(), [this]
        (const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {getTwistMessages(msg);});
        joint_state_msgs_ = this->create_subscription<sensor_msgs::msg::JointState>("/arm_xyz_cmd", rclcpp::SensorDataQoS(), [this]
        (const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {getJointStateMessages(msg);});
        joy_msgs_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::SystemDefaultsQoS(), [this]
        (const sensor_msgs::msg::Joy::ConstSharedPtr& msg){getjoyfeedback(msg);});


        // Start the motors
        uint64_t mask = 0x7E;
        frame_builder_->startMotors(mask); 

        logger.info("All subscriptions initialized"); 
        
        //initialize can start up function call. 
            
}
        
static bool inhibit_arm_cmds = true; 
static bool inhibit_wheel_cmds = true; 
        //------------------------------ Obtaining Joystick input from controller -------------------------------

//TODO: FIX the command inhibition logic. 
void CanControllerNode::getjoyfeedback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg){
    
    constexpr int FORCE_STOP_COMPAT = 3; //Some random button
    constexpr int RESTART_ARM_MOTORS = 2; 
    constexpr int FORCE_STOP_WHEELS = 6;
    constexpr int RESTART_WHEEL_MOTORS = 7; 

    // each force stop and restart expression will: have its on boolean value, and when activated, it will send a forceStop or resume request
    // to its respective motor
    static bool arm_force_stop = false;
    static bool wheel_force_stop = false; 

    bool force_stop_compat = msg->buttons[FORCE_STOP_COMPAT] == 1 ;
    bool restart_arm_pressed = msg->buttons[RESTART_ARM_MOTORS] == 1; 

    bool force_stop_wheels = msg->buttons[FORCE_STOP_WHEELS] == 1; 
    bool restart_wheels = msg->buttons[RESTART_WHEEL_MOTORS] == 1;


    //Force stop active prevents the sendForceStop command to be called more than once. 
    //For command to execute, both conditions in if statement must be true. when force_stop_active 
    //is first called, it is false, therefore, !false = true, and so the command is executed. But, when it is true,
    //the condition becomes !true = false, hence the true && false conditions will cause the command to not be activated. 
    if(force_stop_compat && !arm_force_stop){
        arm_force_stop = true; 
        inhibit_arm_cmds = false; 
        frame_builder_->sendForceStop(deviceType::DeviceType::COMPAT, DeviceId::ID::COMPAT_BOARD_ID); 
        logger.info("Sending force stop command to arm motors");  
    }else if(restart_arm_pressed && arm_force_stop){
        arm_force_stop = false; 
        inhibit_arm_cmds = true; 
        frame_builder_->sendResume(deviceType::DeviceType::COMPAT, DeviceId::ID::COMPAT_BOARD_ID); 
        logger.info("Sending restart request to arm motors"); 
    }

    if(force_stop_wheels && !wheel_force_stop){

        wheel_force_stop = true; 
        inhibit_wheel_cmds = false;
        frame_builder_->sendForceStop(deviceType::DeviceType::RELAY, DeviceId::ID::HUB);
        logger.info("Sending force stop command to wheel motors");
    }else if(restart_wheels && wheel_force_stop){
        wheel_force_stop = false; 
        inhibit_wheel_cmds = true;
        frame_builder_->sendResume(deviceType::DeviceType::RELAY, DeviceId::ID::HUB); 
        logger.info("Sending restart command to wheel motors");
    }
}

//------------------------------ Obtainig Wheel twist Data from controller -------------------------------- 
void CanControllerNode::getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg){

    //extract linear and angular motion in x, and y direction from /cmd_vel
    auto linear_y = twist_msg->linear.x; 
    auto angular_z = twist_msg->angular.z;

    //multiply them together for exponential control
    linear_y *= linear_y * linear_y; 
    angular_z *= angular_z * angular_z; 

    //calculate the wheel velocity and multiply them by a multiplier
    float distance_between_wheels = 1.2f;
    float right_wheel_velocity = -(linear_y - (-angular_z * distance_between_wheels * 0.5f)); 
    float left_wheel_velocity = -(linear_y + (-angular_z * distance_between_wheels * 0.5f)); 


    //convert velocities into rpm
    float right_wheel_velocity_rpm = static_cast<float>(right_wheel_velocity) * multiplier;
    float left_wheel_velocity_rpm = static_cast<float>(left_wheel_velocity) * multiplier;


    if(inhibit_wheel_cmds){
        //send commands out to be processed by the framer 
        frame_builder_->sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT1, right_wheel_velocity_rpm);
        frame_builder_->sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT2, right_wheel_velocity_rpm);
        frame_builder_->sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT3, right_wheel_velocity_rpm);

        frame_builder_->sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT4, left_wheel_velocity_rpm);
        frame_builder_->sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT5, left_wheel_velocity_rpm);
        frame_builder_->sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT6, left_wheel_velocity_rpm);
        

        logger.info("Wheel Motor Commands Sent: Right RPM = {:.2f}, Left RPM = {:.2f}", 
                right_wheel_velocity_rpm, left_wheel_velocity_rpm);
    }
    
}


// ---------------------------- Obtaining Joint State Data from controller -------------------------- 
void CanControllerNode::getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg){

// @brief Publisher sends out JoinState topic data. Here, we subscribe to the Joinstate to extract oncoming velocity messgaes and then send em off
    if(joint_state_msg->velocity.size() < 5){
        logger.error("Received joint state messages with insufficient velocity data. joint size is {}, expected at least 5", joint_state_msg->velocity.size()); 
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
    static constexpr size_t ARM_MOTOR_COUNT = sizeof(MOTOR_MAP) / sizeof(MOTOR_MAP[0]);

    if(inhibit_arm_cmds){
        // sendMotorVelocity() takes in: 1. device type, 2. instruction, 3. device id, 4. payload (velocities) 
        logger.info("Extracting joint state data");
        for(size_t i = 0; i < ARM_MOTOR_COUNT; i++){

            const float velocities = static_cast<float>(joint_state_msg->velocity[i]) * MAX_MOTOR_SPEED; 
            frame_builder_->sendArmMotorVelocity(deviceType::DeviceType::ARM_MOTOR_CONTROLLER, MOTOR_MAP[i], DeviceId::ID::ARM_MOTOR_CONTROLLER, velocities); 
 
            RCLCPP_DEBUG(this->get_logger(), "Motor %zu → %.3f rad/s", i + 1, velocities);
        }
        RCLCPP_INFO(this->get_logger(), "Arm motor commands sent: Motor 1 = %.2f\n, Motor 2 = %.2f\n, Motor 3 = %.2f\n, Motor 4 = %.2f\n, Motor 5 = %.2f", 
            static_cast<float>(joint_state_msg->velocity[0]),
            static_cast<float>(joint_state_msg->velocity[1]),
            static_cast<float>(joint_state_msg->velocity[2]),
            static_cast<float>(joint_state_msg->velocity[3]),
            static_cast<float>(joint_state_msg->velocity[4]));
    }
}



int main(int argc, char *argv[]){

    rclcpp::init(argc, argv); 

    auto node = std::make_shared<CanControllerNode>(); 
    rclcpp::spin(node); 


    rclcpp::shutdown(); 
    return 0; 

}