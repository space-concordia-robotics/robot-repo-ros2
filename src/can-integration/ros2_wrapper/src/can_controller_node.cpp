#include "ros2_wrapper/can_controller_node.hpp"
#include "can-utils/buildAddress.hpp"
#include "can-utils/prefixes.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <thread>
#include <chrono>


CanControllerNode::CanControllerNode(const rclcpp::NodeOptions& options) :
    Node("can_controller_node", options), logger(this->get_logger().get_child("can_controller_node")){

        this->declare_parameter("can_path", "can0");
        multiplier = this->declare_parameter("multiplier", 500);
        arm_velocity_scale_ = this->declare_parameter("arm_velocity_scale", 2.0);
        can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");
        can_send_rate_hz_ = this->declare_parameter("can_send_rate_hz", 100);

        // Servo command shaping: pick instruction mode and scale joystick [-1,1]
        // to physical units (rad for position, rad/s for speed) without recompiling.
        spin_servo_mode_  = this->declare_parameter<std::string>("spin_servo_mode",  "speed");
        clamp_servo_mode_ = this->declare_parameter<std::string>("clamp_servo_mode", "speed");
        spin_servo_max_rad_   = static_cast<float>(this->declare_parameter("spin_servo_max_rad",   3.14159265));
        spin_servo_max_rad_s_ = static_cast<float>(this->declare_parameter("spin_servo_max_rad_s", 3.14159265));
        clamp_servo_max_rad_   = static_cast<float>(this->declare_parameter("clamp_servo_max_rad",   1.5707963));
        clamp_servo_max_rad_s_ = static_cast<float>(this->declare_parameter("clamp_servo_max_rad_s", 1.5707963));

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

        auto arm_scale_callback = [this](const rclcpp::Parameter& parameter){
            if(parameter.get_name() == "arm_velocity_scale"){
                arm_velocity_scale_ = parameter.as_double();
            }
        };
        arm_velocity_scale_callback_handle = parameter_event_handler->add_parameter_callback("arm_velocity_scale", arm_scale_callback);

        frame_builder_ = std::make_unique<SystemFrameBuilder>(can_controller_);

        twist_msgs_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS(), [this]
        (const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {getTwistMessages(msg);});
        joint_state_msgs_ = this->create_subscription<sensor_msgs::msg::JointState>("/arm_xyz_cmd", rclcpp::SensorDataQoS(), [this]
        (const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {getJointStateMessages(msg);});

        // The wheel force-stop / resume joystick logic moved to the standalone
        // can_safety_node package. We still subscribe to /joy but only to keep
        // the wheel_stopped status in sync; the actual STOP/RESUME CAN frames
        // are emitted from the safety node.
        wheel_stopped_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/can_safety/wheel_stopped",
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::Bool::ConstSharedPtr & msg){
                inhibit_wheel_cmds_ = !msg->data;
            });

        const auto period = std::chrono::milliseconds(1000 / can_send_rate_hz_);
        can_send_timer_ = this->create_wall_timer(period, [this]{ sendCanFrames(); });

        uint64_t mask = 0x7E;
        frame_builder_->startMotors(mask);
        frame_builder_->requestStatusFrame();

        logger.info("All subscriptions initialized, CAN send rate = {} Hz", can_send_rate_hz_);
}

//------------------------------ Cache incoming topic data (no CAN I/O here) --------------------------------

void CanControllerNode::getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg){
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_twist_ = twist_msg;
    twist_dirty_ = true;
}

void CanControllerNode::getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg){
    if(joint_state_msg->velocity.size() < 5){
        logger.error("Received joint state messages with insufficient velocity data. joint size is {}, expected at least 5 (and 7 for servos)", joint_state_msg->velocity.size());
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

    if(inhibit_wheel_cmds_){
        frame_builder_->startMotors(0x7E);
    }

    if(send_twist && inhibit_wheel_cmds_){
        constexpr float DEADZONE = 0.05f;
        constexpr float kPureAxisEps = 1e-5f;

        auto linear_x = static_cast<float>(twist->linear.x);
        auto angular_z = static_cast<float>(twist->angular.z);

        if(std::abs(linear_x) < DEADZONE) linear_x = 0.0f;
        if(std::abs(angular_z) < DEADZONE) angular_z = 0.0f;

        const float half_track = 1.2f * 0.5f;
        float right_cmd = 0.F;
        float left_cmd = 0.F;

        const bool yaw_only = std::abs(linear_x) < kPureAxisEps;
        const bool translate_only = std::abs(angular_z) < kPureAxisEps;

        if(translate_only && yaw_only){
            right_cmd = 0.F;
            left_cmd = 0.F;
        }else if(translate_only){
            right_cmd = -linear_x;
            left_cmd = -linear_x;
        }else if(yaw_only){
            right_cmd = -(-angular_z * half_track);
            left_cmd = -((-angular_z * half_track));
        }else{
            right_cmd = -(linear_x - (-angular_z * half_track));
            left_cmd = -(linear_x + (-angular_z * half_track));
        }

        const float mult = static_cast<float>(multiplier);
        const std::array<float, 6> target_rpm = {
            right_cmd * mult,
            right_cmd * mult,
            right_cmd * mult,
            left_cmd * mult,
            left_cmd * mult,
            left_cmd * mult,
        };

        const float dt = 1.F / static_cast<float>(std::max(1, can_send_rate_hz_));
        const float max_step = (wheel_rpm_slew_rate_ > 0.F)
            ? wheel_rpm_slew_rate_ * dt
            : std::numeric_limits<float>::max();

        static constexpr std::array<DeviceId::ID, 6> kWheelIds = {
            DeviceId::ID::WHEEL_MOT1,
            DeviceId::ID::WHEEL_MOT2,
            DeviceId::ID::WHEEL_MOT3,
            DeviceId::ID::WHEEL_MOT4,
            DeviceId::ID::WHEEL_MOT5,
            DeviceId::ID::WHEEL_MOT6,
        };

        for(size_t i = 0; i < kWheelIds.size(); ++i){
            float delta = target_rpm[i] - wheel_rpm_smoothed_[i];
            if(delta > max_step) delta = max_step;
            if(delta < -max_step) delta = -max_step;
            wheel_rpm_smoothed_[i] += delta;
            frame_builder_->sendWheelMotorVelocity(kWheelIds[i], wheel_rpm_smoothed_[i]);
        }

        frame_builder_->startMotors(0x7E);
    }

    if(send_arm && inhibit_arm_cmds_){
        static constexpr Instructions::Inst MOTOR_MAP[5] = {
            Instructions::Inst::ARM_MOTOR_1,
            Instructions::Inst::ARM_MOTOR_2,
            Instructions::Inst::ARM_MOTOR_3,
            Instructions::Inst::ARM_MOTOR_4,
            Instructions::Inst::ARM_MOTOR_5,
        };
        static constexpr size_t ARM_MOTOR_COUNT = sizeof(MOTOR_MAP) / sizeof(MOTOR_MAP[0]);

        std::array<float, ARM_MOTOR_COUNT> cmd_sent{};
        for(size_t i = 0; i < ARM_MOTOR_COUNT; i++){
            const double raw = joint_state->velocity[i];
            constexpr double DEADZONE = 0.05;
            float velocities = (std::abs(raw) < DEADZONE)
                ? 0.0f
                : static_cast<float>(raw * arm_velocity_scale_ * buildAddress::BuildAddress::ARM_MOTOR_VELOCITY_MAX);
            // motor 4 is a special case, it needs to be scaled down to 50% of the other motors because it is faster
            if (i == 3) {
                velocities *= 0.5f;
            }
            const float payload = std::clamp(
                velocities,
                buildAddress::BuildAddress::ARM_MOTOR_VELOCITY_MIN,
                buildAddress::BuildAddress::ARM_MOTOR_VELOCITY_MAX);
            cmd_sent[i] = payload;
            frame_builder_->sendArmMotorVelocity(
                deviceType::DeviceType::ARM_MOTOR_CONTROLLER, MOTOR_MAP[i], DeviceId::ID::ARM_MOTOR_CONTROLLER, payload);
            std::this_thread::sleep_for(std::chrono::microseconds(400));
            RCLCPP_DEBUG(this->get_logger(), "Motor %zu -> %.3f (payload)", i + 1, payload);
        }
        RCLCPP_INFO(this->get_logger(), "Arm motor commands sent: Motor 1 = %.2f, Motor 2 = %.2f, Motor 3 = %.2f, Motor 4 = %.2f, Motor 5 = %.2f",
            cmd_sent[0], cmd_sent[1], cmd_sent[2], cmd_sent[3], cmd_sent[4]);

        if(joint_state->velocity.size() >= 7){
            constexpr double SERVO_DEADZONE = 0.05;
            constexpr size_t SPIN_IDX  = 5;
            constexpr size_t CLAMP_IDX = 6;

            const auto shape_input = [](double raw) -> float {
                return (std::abs(raw) < SERVO_DEADZONE) ? 0.0f : static_cast<float>(raw);
            };
            const float spin_in  = shape_input(joint_state->velocity[SPIN_IDX]);
            const float clamp_in = shape_input(joint_state->velocity[CLAMP_IDX]);

            const float spin_payload = std::clamp(spin_in, -1.0f, 1.0f) *
                ((spin_servo_mode_ == "position") ? spin_servo_max_rad_ : spin_servo_max_rad_s_);
            if(spin_servo_mode_ == "position"){
                frame_builder_->sendSpinServoPosition(spin_payload);
            } else {
                frame_builder_->sendSpinServoSpeed(spin_payload);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(400));

            const float clamp_payload = std::clamp(clamp_in, -1.0f, 1.0f) *
                ((clamp_servo_mode_ == "position") ? clamp_servo_max_rad_ : clamp_servo_max_rad_s_);
            if(clamp_servo_mode_ == "position"){
                frame_builder_->sendClampServoPosition(clamp_payload);
            } else {
                frame_builder_->sendClampServoSpeed(clamp_payload);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(400));

            RCLCPP_INFO(this->get_logger(),
                "Servo commands sent: spin (%s) = %.3f, clamp (%s) = %.3f",
                spin_servo_mode_.c_str(),  spin_payload,
                clamp_servo_mode_.c_str(), clamp_payload);
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
