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

#include "bab-board/battery_board.hpp"
#include "can-utils/buildAddress.hpp"
#include "can-utils/can_connect.hpp"
#include "can-utils/can_interface.hpp"
#include "can-utils/prefixes.hpp"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

namespace {

constexpr uint8_t BAB_DEVTYPE = static_cast<uint8_t>(deviceType::DeviceType::BAB);
constexpr uint8_t BAB_FIRMWARE_MFR = Manufacturer::SCC;
constexpr uint8_t BAB_FIRMWARE_DEVID = 0x01;

std::string formatRow(const char * label,
                      bool fresh,
                      bool ever_received,
                      float v, float i, float p_or_t,
                      const char * p_or_t_label,
                      const char * extra = nullptr) {
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

}  // namespace


class BabTelemetryNode : public rclcpp::Node {
public:
    BabTelemetryNode()
    : Node("bab_telemetry_node") {

        can_interface_ = declare_parameter<std::string>("can_interface", "can0");
        const int period_ms = declare_parameter<int>("print_period_ms", 1000);
        log_unknown_ = declare_parameter<bool>("log_unknown_bab_frames", false);

        can_ = can_util::createConfiguredCanController(can_interface_, get_logger());
        if (!can_) {
            throw std::runtime_error(
                "CAN configure failed on '" + can_interface_ + "' — see log for hints");
        }

        rclcpp::on_shutdown([weak_can = std::weak_ptr<can_util::CANController>(can_)] {
            if (auto can = weak_can.lock()) {
                can->stop();
            }
        });

        build_address_ = std::make_unique<buildAddress::BuildAddress>(can_);
        bab_ = std::make_shared<BAB>(
            get_logger(),
            *can_,
            *build_address_,
            static_cast<uint32_t>(DeviceId::ID::BAB));

        if (log_unknown_) {
            // Independent callback so we can warn about BAB-typed frames whose
            // Manufacturer / DeviceID do not match BAB-docs.md (i.e. frames
            // that the BAB parser will silently ignore).
            unknown_frame_cb_ = can_->registerFrameCallback(
                [this](uint32_t id, const std::vector<uint8_t> & data) {
                    onAnyFrame(id, data);
                });
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            [this]() { printTelemetry(); });

        RCLCPP_INFO(get_logger(),
                    "bab_telemetry_node ready — interface=%s, period=%d ms, log_unknown=%s",
                    can_interface_.c_str(), period_ms, log_unknown_ ? "true" : "false");
    }

    ~BabTelemetryNode() override {
        if (can_) {
            can_->stop();
        }
    }

private:
    void printTelemetry() {
        std::ostringstream ss;
        ss << "\n--- BAB Telemetry (iface=" << can_interface_ << ") ---";

        for (size_t i = 0; i < BAB::NUM_BATTERIES; ++i) {
            std::string label = "Battery " + std::to_string(i + 1);
            ss << "\n"
               << formatRow(label.c_str(),
                            bab_->batteryFresh(i),
                            bab_->batteryEverReceived(i),
                            bab_->getBatteryVoltageLevel(i),
                            bab_->getBatteryCurrentLevel(i),
                            bab_->getBatteryTemp(i),
                            "T");
        }

        for (size_t i = 0; i < BAB::NUM_RAILS; ++i) {
            const char * name = (i == 0) ? "Rail 1 (5V)"
                             :  (i == 1) ? "Rail 2 (Arm)"
                             :              "Rail 3 (Whl)";
            std::ostringstream extra;
            extra << "P=" << std::fixed << std::setprecision(1)
                  << bab_->getRailPower(i) << "W"
                  << " sw=" << (bab_->getRailSwitchOn(i) ? "ON " : "OFF");
            ss << "\n"
               << formatRow(name,
                            bab_->railFresh(i),
                            bab_->railEverReceived(i),
                            bab_->getRailVoltageLevel(i),
                            bab_->getRailCurrent(i),
                            bab_->getRailPower(i),
                            "P",
                            extra.str().c_str());
        }

        RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
    }

    void onAnyFrame(uint32_t id, const std::vector<uint8_t> & data) {
        const uint8_t devtype = (id >> 24) & 0x1F;
        if (devtype != BAB_DEVTYPE) {
            return;
        }
        const uint8_t mfr   = (id >> 16) & 0xFF;
        const uint8_t devid = id & 0x3F;
        if (mfr == BAB_FIRMWARE_MFR && devid == BAB_FIRMWARE_DEVID) {
            return;  // already handled by BAB parser
        }

        std::ostringstream ss;
        ss << "Unmatched BAB-typed frame id=0x" << std::hex << std::uppercase
           << std::setw(8) << std::setfill('0') << id
           << " (mfr=0x" << std::setw(2) << static_cast<int>(mfr)
           << " devid=0x" << std::setw(2) << static_cast<int>(devid)
           << ") data=";
        for (auto b : data) {
            ss << std::hex << std::setw(2) << std::setfill('0')
               << static_cast<int>(b) << " ";
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", ss.str().c_str());
    }

    std::string can_interface_;
    bool log_unknown_{false};
    std::shared_ptr<can_util::CANController> can_;
    std::unique_ptr<buildAddress::BuildAddress> build_address_;
    std::shared_ptr<BAB> bab_;
    std::shared_ptr<can_util::CANFrameCallback> unknown_frame_cb_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        auto node = std::make_shared<BabTelemetryNode>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("bab_telemetry_node"),
                     "Node failed to start: %s", e.what());
        exit_code = 1;
    } catch (...) {
        RCLCPP_FATAL(rclcpp::get_logger("bab_telemetry_node"),
                     "Node failed to start: unknown exception");
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
