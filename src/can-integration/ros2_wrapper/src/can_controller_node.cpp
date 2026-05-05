#include "ros2_wrapper/can_controller_node.hpp"
#include "can-utils/prefixes.hpp"
#include <cmath>


CanControllerNode::CanControllerNode(const rclcpp::NodeOptions& options) : 
    Node("can_controller_node", options), logger(this->get_logger().get_child("can_controller_node")){

        this->declare_parameter("can_path", "can0");
        multiplier = this->declare_parameter("multiplier", 500);
        arm_velocity_scale_ = this->declare_parameter("arm_velocity_scale", 4096.0);
        can_send_rate_hz_ = this->declare_parameter("can_send_rate_hz", 9);

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

        auto arm_scale_callback = [this](const rclcpp::Parameter& parameter) {
            if (parameter.get_name() == "arm_velocity_scale") {
                arm_velocity_scale_ = parameter.as_double();
            }
        };
        arm_velocity_scale_callback_handle_ =
            parameter_event_handler->add_parameter_callback("arm_velocity_scale", arm_scale_callback);
        
        frame_builder_ = std::make_unique<SystemFrameBuilder>(can_controller_);

        twist_msgs_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS(), [this]
        (const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {getTwistMessages(msg);});
        joint_state_msgs_ = this->create_subscription<sensor_msgs::msg::JointState>("/arm_xyz_cmd", rclcpp::SensorDataQoS(), [this]
        (const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {getJointStateMessages(msg);});
        joy_msgs_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::SystemDefaultsQoS(), [this]
        (const sensor_msgs::msg::Joy::ConstSharedPtr& msg){getjoyfeedback(msg);});

        const auto period = std::chrono::milliseconds(1000 / can_send_rate_hz_);
        can_send_timer_ = this->create_wall_timer(period, [this]{ sendCanFrames(); });

        uint64_t mask = 0x7E;
        frame_builder_->startMotors(mask); 

        logger.info("All subscriptions initialized, CAN send rate = {} Hz", can_send_rate_hz_); 
}
        
static bool inhibit_arm_cmds = true; 
static bool inhibit_wheel_cmds = true; 

//------------------------------ Obtaining Joystick input from controller -------------------------------

void CanControllerNode::getjoyfeedback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg){
    
    constexpr int FORCE_STOP_COMPAT = 3;
    constexpr int RESTART_ARM_MOTORS = 2; 
    constexpr int FORCE_STOP_WHEELS = 6;
    constexpr int RESTART_WHEEL_MOTORS = 7; 

    static bool arm_force_stop = false;
    static bool wheel_force_stop = false; 

    bool force_stop_compat = msg->buttons[FORCE_STOP_COMPAT] == 1 ;
    bool restart_arm_pressed = msg->buttons[RESTART_ARM_MOTORS] == 1; 

    bool force_stop_wheels = msg->buttons[FORCE_STOP_WHEELS] == 1; 
    bool restart_wheels = msg->buttons[RESTART_WHEEL_MOTORS] == 1;

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

//------------------------------ Cache incoming topic data (no CAN I/O here) -------------------------------- 

void CanControllerNode::getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg){
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_twist_ = twist_msg;
    twist_dirty_ = true;
}

void CanControllerNode::getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg){
    if(joint_state_msg->velocity.size() < 5){
        logger.error("Received joint state messages with insufficient velocity data. joint size is {}, expected at least 5", joint_state_msg->velocity.size()); 
        return; 
    }
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_joint_state_ = joint_state_msg;
    joint_state_dirty_ = true;
}

//------------------------------ Timer: send cached commands at fixed rate -------------------------------- 

void CanControllerNode::sendCanFrames(){

    geometry_msgs::msg::Twist::ConstSharedPtr twist;
    sensor_msgs::msg::JointState::ConstSharedPtr joint_state;
    bool send_twist = false;
    bool send_arm = false;

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        if(twist_dirty_){
            twist = latest_twist_;
            twist_dirty_ = false;
            send_twist = true;
        }
        if(joint_state_dirty_){
            joint_state = latest_joint_state_;
            joint_state_dirty_ = false;
            send_arm = true;
        }
    }

    if(send_twist && inhibit_wheel_cmds){
        auto linear_y = twist->linear.x; 
        auto angular_z = twist->angular.z;

        linear_y *= linear_y * linear_y; 
        angular_z *= angular_z * angular_z; 

        float distance_between_wheels = 1.2f;
        float right_wheel_velocity = -(linear_y - (-angular_z * distance_between_wheels * 0.5f)); 
        float left_wheel_velocity = -(linear_y + (-angular_z * distance_between_wheels * 0.5f)); 

        float right_wheel_velocity_rpm = static_cast<float>(right_wheel_velocity) * multiplier;
        float left_wheel_velocity_rpm = static_cast<float>(left_wheel_velocity) * multiplier;

        frame_builder_->sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT1, right_wheel_velocity_rpm);
        frame_builder_->sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT2, right_wheel_velocity_rpm);
        frame_builder_->sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT3, right_wheel_velocity_rpm);

        frame_builder_->sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT4, left_wheel_velocity_rpm);
        frame_builder_->sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT5, left_wheel_velocity_rpm);
        frame_builder_->sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT6, left_wheel_velocity_rpm);

        logger.info("Wheel Motor Commands Sent: Right RPM = {:.2f}, Left RPM = {:.2f}", 
                right_wheel_velocity_rpm, left_wheel_velocity_rpm);
    }

    if(send_arm && inhibit_arm_cmds){
        static constexpr Instructions::Inst MOTOR_MAP[5] = {
            Instructions::Inst::ARM_MOTOR_1,
            Instructions::Inst::ARM_MOTOR_2,
            Instructions::Inst::ARM_MOTOR_3,
            Instructions::Inst::ARM_MOTOR_4,
            Instructions::Inst::ARM_MOTOR_5,
        };
        static constexpr size_t ARM_MOTOR_COUNT = sizeof(MOTOR_MAP) / sizeof(MOTOR_MAP[0]);

        for(size_t i = 0; i < ARM_MOTOR_COUNT; i++){
            const double raw = joint_state->velocity[i];
            constexpr double DEADZONE = 0.05;
            const float velocities = (std::abs(raw) < DEADZONE)
                ? 0.0f
                : static_cast<float>(raw * arm_velocity_scale_);
            frame_builder_->sendArmMotorVelocity(deviceType::DeviceType::ARM_MOTOR_CONTROLLER, MOTOR_MAP[i], DeviceId::ID::ARM_MOTOR_CONTROLLER, velocities); 
            RCLCPP_DEBUG(this->get_logger(), "Motor %zu → %.3f", i + 1, velocities);
        }
        RCLCPP_INFO(this->get_logger(), "Arm motor commands sent: Motor 1 = %.2f, Motor 2 = %.2f, Motor 3 = %.2f, Motor 4 = %.2f, Motor 5 = %.2f", 
            static_cast<float>(joint_state->velocity[0]),
            static_cast<float>(joint_state->velocity[1]),
            static_cast<float>(joint_state->velocity[2]),
            static_cast<float>(joint_state->velocity[3]),
            static_cast<float>(joint_state->velocity[4]));
    }
}


int main(int argc, char *argv[]){

    rclcpp::init(argc, argv); 

    auto node = std::make_shared<CanControllerNode>(); 
    rclcpp::spin(node); 

    rclcpp::shutdown(); 
    return 0; 

}
