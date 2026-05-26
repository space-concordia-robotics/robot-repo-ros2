// Standalone ROS 2 node that prints BAB (Battery Arbiter Board) telemetry.
//
// Usage:
//   ros2 run bab-board bab_telemetry_node
//   ros2 run bab-board bab_telemetry_node --ros-args
//       -p can_interface:=can0
//       -p print_period_ms:=500
//       -p log_unknown_bab_frames:=true
//
// Parameters:
//   can_interface           (string, default "can0") SocketCAN device.
//   print_period_ms         (int,    default 1000)   Telemetry print period.
//   log_unknown_bab_frames  (bool,   default false)  If true, every received
//                                                    frame whose DeviceType
//                                                    field equals 0x01 (BAB)
//                                                    but does not match the
//                                                    documented Manufacturer
//                                                    or DeviceID is logged
//                                                    (throttled). Use this to
//                                                    diagnose CAN-ID mismatch
//                                                    on the bus.

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <fmt/ranges.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>

#include "bab-board/battery_board.hpp"
#include "can_util/can_util.hpp"

// Match the values actually used by the BAB firmware on the rover
// (see comments at the top of battery_board.cpp).
static constexpr auto BAB_DEVICE_TYPE = can_util::constants::DeviceType::BROADCAST_MESSAGE; // should really be POWER_DISTRIBUTION_MODULE
static constexpr auto BAB_MANUFACTURER = can_util::constants::Manufacturer::TEAM_USE; // 0x08
static constexpr auto BAB_ID = 0x00;

namespace {
    // TODO 2026-05-26 (Will Free): convert this to using fmt with ros2_fmt_logger
    std::string formatRow(
        const char* label,
        const bool fresh,
        const bool ever_received,
        const float v, const float i, const float p_or_t,
        const char* p_or_t_label,
        const char* extra = nullptr
    ) {
        std::ostringstream ss;
        ss << "  " << std::left << std::setw(10) << label
            << " V=" << std::setw(7) << std::fixed << std::setprecision(2) << v
            << " I=" << std::setw(7) << std::fixed << std::setprecision(2) << i
            << " " << p_or_t_label << "=" << std::setw(7) << std::fixed << std::setprecision(2) << p_or_t;
        if (extra) {
            ss << " " << extra;
        }
        if (!ever_received) {
            ss << "  [NO FRAMES YET]";
        } else if (!fresh) {
            ss << "  [STALE]";
        } else {
            ss << "  [ok]";
        }
        return ss.str();
    }
}


class BabTelemetryNode : public rclcpp::Node {
public:
    BabTelemetryNode()
        : Node("bab_telemetry_node"),
          logger(this->get_logger().get_child("bab_telemetry_node"), *get_clock()) {
        can_interface = declare_parameter<std::string>("can_interface", "can0");
        const int period_ms = declare_parameter<int>("print_period_ms", 1000);
        log_unknown = declare_parameter<bool>("log_unknown_bab_frames", false);

        can_controller = std::make_shared<can_util::CANController>(can_interface, this->get_logger());
        if (!can_controller->initialize()) {
            logger.fatal("Failed to initialize canbus");
            throw std::runtime_error("CAN failed to initialize");
        }

        bab = std::make_shared<BAB>(
            get_logger(),
            can_controller,
            BAB_ID
        );

        if (log_unknown) {
            // Independent callback so we can warn about BAB-typed frames whose
            // Manufacturer / DeviceID do not match BAB-docs.md (i.e. frames
            // that the BAB parser will silently ignore).
            unknown_frame_callback = can_controller->registerFrameCallback(
                [this](const uint32_t id, const std::vector<uint8_t>& data) {
                    onAnyFrame(id, data);
                }
            );
        }

        timer = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            [this] {
                printTelemetry();
            }
        );

        logger.info(
            "bab_telemetry_node ready — interface={}, period={} ms, log_unknown={}",
            can_interface, period_ms, log_unknown);
    }

    ~BabTelemetryNode() override {}

private:
    void printTelemetry() const {
        std::ostringstream ss;
        ss << "\n--- BAB Telemetry (iface=" << can_interface << ") ---";

        for (size_t i = 0; i < BAB::BATTERIES_COUNT; ++i) {
            std::string label = "Battery " + std::to_string(i + 1);
            ss << "\n"
                << formatRow(label.c_str(),
                             bab->batteryFresh(i),
                             bab->batteryEverReceived(i),
                             bab->getBatteryVoltageLevel(i),
                             bab->getBatteryCurrentLevel(i),
                             bab->getBatteryTemp(i),
                             "T");
        }

        // TODO 2026-05-26 (Will Free): convert this to using fmt
        for (size_t i = 0; i < BAB::RAILS_COUNT; ++i) {
            // TODO 2026-05-26 (Will Free): this ternary sucks
            const char* name = i == 0
                                   ? "Rail 1 (5V)"
                                   : i == 1
                                   ? "Rail 2 (Arm)"
                                   : "Rail 3 (Whl)";
            std::ostringstream extra;
            extra << "P=" << std::fixed << std::setprecision(1)
                << bab->getRailPower(i) << "W"
                << " sw=" << (bab->getRailSwitchOn(i) ? "ON " : "OFF");
            ss << "\n"
                << formatRow(name,
                             bab->railFresh(i),
                             bab->railEverReceived(i),
                             bab->getRailVoltageLevel(i),
                             bab->getRailCurrent(i),
                             bab->getRailPower(i),
                             "P",
                             extra.str().c_str());
        }

        logger.info("{}", ss.str());
    }

    void onAnyFrame(const uint32_t id, const std::vector<uint8_t>& data) const {
        if (const uint8_t device_type = id >> 24 & 0x1F; device_type != static_cast<uint8_t>(BAB_DEVICE_TYPE)) {
            return;
        }
        const uint8_t manufacturer = id >> 16 & 0xFF;
        const uint8_t device_id = id & 0x3F;
        if (manufacturer == static_cast<uint8_t>(BAB_MANUFACTURER) && device_id == BAB_ID) {
            return; // already handled by BAB parser
        }

        using namespace std::chrono_literals;
        logger.warn_throttle(2s, "Unmatched BAB-typed frame id={:#08X} (mfr={:#02X}, devid={:#02X}) data={:#02X}", id, manufacturer, device_id, data);
    }

    ros2_fmt_logger::Logger logger;
    std::string can_interface;
    bool log_unknown = false;
    std::shared_ptr<can_util::CANController> can_controller;
    // std::unique_ptr<buildAddress::BuildAddress> build_address_;
    std::shared_ptr<BAB> bab;
    std::shared_ptr<can_util::CANFrameCallback> unknown_frame_callback;
    rclcpp::TimerBase::SharedPtr timer;
};


int main(const int argc, char** argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        const auto node = std::make_shared<BabTelemetryNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        ros2_fmt_logger::Logger(rclcpp::get_logger("bab_telemetry_node"))
            .fatal("Node failed to start: {}", e.what());
        exit_code = 1;
    } catch (...) {
        ros2_fmt_logger::Logger(rclcpp::get_logger("bab_telemetry_node"))
            .fatal("Node failed to start: unknown exception");
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
