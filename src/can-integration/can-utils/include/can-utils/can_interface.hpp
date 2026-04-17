#pragma once

#include <functional>
#include <linux/can.h>
#include <thread>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include "ros2_fmt_logger/ros2_fmt_logger.hpp"


namespace can_util {

    using CANFrameCallback = std::function<void(uint32_t id, const std::vector<uint8_t>& data)>;

    class CANController : public std::enable_shared_from_this<CANController> {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(CANController)

        CANController(std::string path, rclcpp::Logger logger);

        ~CANController();

        bool configureCan();

        std::shared_ptr<CANFrameCallback> registerFrameCallback(CANFrameCallback callback);

        //bool sendBlockingFrame(uint32_t id, const std::vector<uint8_t>& data) const;

        // TODO 2026-02-25 (Will Free): add a way to read can frames with a specific id, ignoring all others?
        bool readFrameIfAvailable(can_frame& frame) const;

        bool readFrame(can_frame& frame) const;
        
        bool sendBlockingFrame(const can_frame& frame) const;
        
        private:
        
        bool writeFrame(const can_frame& frame) const;

        mutable std::mutex mtx;
        ros2_fmt_logger::Logger logger;
        std::string path;
        int socket_descriptor = 0;
        std::thread readThread;
        std::vector<std::weak_ptr<CANFrameCallback>> frame_callbacks = {};

        static constexpr auto READ_TIMEOUT_US = 20000;
    };
}