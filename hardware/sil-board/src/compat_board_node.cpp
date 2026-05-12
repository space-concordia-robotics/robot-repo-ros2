#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>

#include <linux/can.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "can-utils/can_interface.hpp"

namespace {

// SCRB motor board — host→board IDs from Compat-README.md (29-bit extended, MAKE_ID layout).
constexpr uint32_t kScrbForceStopId = 0x0001808Cu;
constexpr uint32_t kScrbResumeId = 0x0001810Cu;
constexpr std::array<uint32_t, 5> kScrbMotorIds = {
    0x0001848Cu, 0x0001850Cu, 0x0001858Cu, 0x0001860Cu, 0x0001868Cu};

/// Protocol full-scale command magnitude (see “Motor velocity scaling” in Compat-README.md).
constexpr float kMaxRads = 1024.0F;

// PS4-style layout (matches sil_board/scripts/sil_board_joy_teleop.py).
constexpr int kAxisLeftStickX = 0;
constexpr int kAxisLeftStickY = 1;
constexpr int kAxisLeftTrigger = 2;
constexpr int kAxisRightStickX = 3;
constexpr int kAxisRightStickY = 4;
constexpr int kAxisRightTrigger = 5;

constexpr int kBtnSquareForceStop = 3;
constexpr int kBtnTriangleResume = 2;
constexpr int kBtnLeftBumperDeadman = 4;

constexpr std::size_t kMinAxes = 6;
constexpr std::size_t kMinButtons = 8;

} // namespace

class CompatBoardNode : public rclcpp::Node {
public:
    explicit CompatBoardNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("compat_board_node", options)
    {
        const auto can_path = this->declare_parameter<std::string>("can_interface", "can0");
        joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
        velocity_scale_ =
            static_cast<float>(this->declare_parameter<double>("velocity_scale", 1024.0));
        can_send_rate_hz_ = this->declare_parameter<int>("can_send_rate_hz", 50);
        invert_left_y_ = this->declare_parameter<bool>("invert_left_y", true);
        invert_right_y_ = this->declare_parameter<bool>("invert_right_y", true);
        axis_deadzone_ =
            static_cast<float>(this->declare_parameter<double>("axis_deadzone", 0.08));

        can_controller_ = std::make_shared<can_util::CANController>(can_path, this->get_logger());
        if (!can_controller_->configureCan()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to configure CAN on %s", can_path.c_str());
            throw std::runtime_error("CAN configure failed");
        }

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) { onJoy(msg); });

        const int hz = std::max(1, can_send_rate_hz_);
        const auto period = std::chrono::milliseconds(1000 / hz);
        send_timer_ = this->create_wall_timer(period, [this] { onSendTimer(); });

        RCLCPP_INFO(this->get_logger(),
                    "compat_board_node — CAN %s, joy %s, scale %.1f, %d Hz", can_path.c_str(),
                    joy_topic_.c_str(), static_cast<double>(velocity_scale_), hz);
    }

private:
    static float applyDeadzone(float v, float dz)
    {
        const float a = std::fabs(v);
        if (a <= dz) {
            return 0.0F;
        }
        const float s = (a - dz) / (1.0F - dz);
        return std::copysign(s, v);
    }

    void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joy_mutex_);
        latest_joy_ = std::move(msg);
    }

    bool sendDlc0(uint32_t raw_id)
    {
        struct can_frame frame {};
        frame.can_id = raw_id | CAN_EFF_FLAG;
        frame.len = 0;
        return can_controller_->sendBlockingFrame(frame);
    }

    bool sendMotorFloat(uint32_t raw_id, float velocity_rads)
    {
        struct can_frame frame {};
        frame.can_id = raw_id | CAN_EFF_FLAG;
        frame.len = 4;
        const float clamped = std::clamp(velocity_rads, -kMaxRads, kMaxRads);
        std::memcpy(frame.data, &clamped, sizeof(float));
        return can_controller_->sendBlockingFrame(frame);
    }

    void onSendTimer()
    {
        sensor_msgs::msg::Joy::SharedPtr joy_copy;
        {
            std::lock_guard<std::mutex> lock(joy_mutex_);
            if (!latest_joy_) {
                return;
            }
            joy_copy = latest_joy_;
        }

        const auto& joy = *joy_copy;
        if (joy.axes.size() < kMinAxes || joy.buttons.size() < kMinButtons) {
            return;
        }

        const bool deadman = joy.buttons[kBtnLeftBumperDeadman] != 0;

        const bool force_stop_now = joy.buttons[kBtnSquareForceStop] != 0;
        const bool resume_now = joy.buttons[kBtnTriangleResume] != 0;

        if (force_stop_now && !prev_force_stop_) {
            if (!sendDlc0(kScrbForceStopId)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Failed to send SCRB force stop");
            }
        }
        if (resume_now && !prev_resume_) {
            if (!sendDlc0(kScrbResumeId)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Failed to send SCRB resume");
            }
        }
        prev_force_stop_ = force_stop_now;
        prev_resume_ = resume_now;

        auto ax = [&joy](int i) -> float {
            return (i >= 0 && static_cast<std::size_t>(i) < joy.axes.size()) ? joy.axes[i] : 0.0F;
        };

        float lx = applyDeadzone(ax(kAxisLeftStickX), axis_deadzone_);
        float ly = applyDeadzone(ax(kAxisLeftStickY), axis_deadzone_);
        float rx = applyDeadzone(ax(kAxisRightStickX), axis_deadzone_);
        float ry = applyDeadzone(ax(kAxisRightStickY), axis_deadzone_);
        float lt = applyDeadzone(ax(kAxisLeftTrigger), axis_deadzone_);
        float rt = applyDeadzone(ax(kAxisRightTrigger), axis_deadzone_);

        if (invert_left_y_) {
            ly = -ly;
        }
        if (invert_right_y_) {
            ry = -ry;
        }

        std::array<float, 5> cmd{};
        if (deadman) {
            // One axis per motor: left stick → M1/M2, right stick → M3/M4, triggers → M5.
            cmd[0] = ly * velocity_scale_;
            cmd[1] = lx * velocity_scale_;
            cmd[2] = ry * velocity_scale_;
            cmd[3] = rx * velocity_scale_;
            cmd[4] = 0.5F * (lt + rt) * velocity_scale_;
        }

        for (std::size_t i = 0; i < kScrbMotorIds.size(); ++i) {
            if (!sendMotorFloat(kScrbMotorIds[i], cmd[i])) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Failed to send motor %zu CAN frame", i + 1);
            }
        }
    }

    std::string joy_topic_;
    float velocity_scale_{1024.0F};
    int can_send_rate_hz_{50};
    bool invert_left_y_{true};
    bool invert_right_y_{true};
    float axis_deadzone_{0.08F};

    bool prev_force_stop_{false};
    bool prev_resume_{false};

    std::mutex joy_mutex_;
    sensor_msgs::msg::Joy::SharedPtr latest_joy_;

    std::shared_ptr<can_util::CANController> can_controller_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr send_timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompatBoardNode>());
    rclcpp::shutdown();
    return 0;
}
