#pragma once

#include <functional>
#include <linux/can.h>
#include <ros2_fmt_logger/logger.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>

namespace can_util {
    using CANFrameCallback = std::function<void(uint32_t id, const std::vector<uint8_t>& data)>;

    class CANController : public std::enable_shared_from_this<CANController> {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(CANController);

        CANController(std::string path, rclcpp::Logger logger);

        ~CANController();

        bool initialize();

        std::shared_ptr<CANFrameCallback> registerFrameCallback(CANFrameCallback callback);

        template <std::size_t N>
        bool sendBlockingFrame(const uint32_t id, const std::array<uint8_t, N>& data) const requires (N <= 8) {
            auto frame = can_frame{};
            frame.can_id = id | CAN_EFF_FLAG;

            // NOLINTNEXTLINE(*-pro-type-union-access): it's only a union to preserve backwards compatibility of the name
            frame.len = static_cast<uint8_t>(data.size());
            memcpy(frame.data, data.data(), data.size());

            return trySendBlockingFrame(frame);
        }

        template <std::size_t N>
        bool sendBlockingFrame(const uint32_t id, const uint8_t (&data)[N]) const requires (N <= 8) {
            return sendBlockingFrame(id, std::to_array(data));
        }

        bool sendBlockingFrame(const uint32_t id, const std::vector<uint8_t>& data) const {
            if (data.size() > 8) {
                logger.error("CAN frame too large");
                return false;
            }

            auto frame = can_frame{};
            frame.can_id = id | CAN_EFF_FLAG;

            // NOLINTNEXTLINE(*-pro-type-union-access): it's only a union to preserve backwards compatibility of the name
            frame.len = static_cast<uint8_t>(data.size());
            memcpy(frame.data, data.data(), data.size()); // NOLINT(*-pro-bounds-array-to-pointer-decay)

            return trySendBlockingFrame(frame);
        }

        // TODO 2026-02-25 (Will Free): add a way to read can frames with a specific id, ignoring all others?
        bool readFrameIfAvailable(can_frame& frame) const;

        bool readFrame(can_frame& frame) const;

    private:
        bool trySendBlockingFrame(const can_frame& frame) const;

        bool sendFrame(const can_frame& frame) const;

        mutable std::mutex mtx;
        ros2_fmt_logger::Logger logger;
        std::string path;
        int socket_descriptor = 0;
        std::thread readThread;
        std::vector<std::weak_ptr<CANFrameCallback>> frame_callbacks = {};

        static constexpr auto READ_TIMEOUT_US = 20000;
    };
}
