#include "can-utils/can_interface.hpp"
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <utility>
#include <net/if.h>
#include <rclcpp/logging.hpp>
#include <sys/ioctl.h>
#include <cassert>
#include <sys/socket.h>


namespace can_util {
    CANController::CANController(std::string path, rclcpp::Logger logger) : logger(logger.get_child("can_controller")), path(std::move(path)){}

    CANController::~CANController() {
        readThread.join();
    }

    bool CANController::configureCan() {
        // open canbus socket
        socket_descriptor = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (socket_descriptor == -1) {
            logger.fatal("socket error: {} ({})\nPossible causes:\n1. CAN modules not loaded\n2. System resource limitations",
                         strerror(errno), errno);
            return false;
        }

        if (path.length() > IF_NAMESIZE) {
            logger.fatal("CAN interface name is longer than the maximum length of {}.", IF_NAMESIZE);
            return false;
        }

        // setup canbus socket

        auto ifr = ifreq{};
        strncpy(ifr.ifr_name, path.c_str(), IF_NAMESIZE);

        if (ioctl(socket_descriptor, SIOCGIFINDEX, &ifr) == -1) {
            logger.fatal("ioctl error: {} ({})\nPossible causes:\n1. CAN interface does not exist\n2. CAN bus not initialized\n3. CAN interface is not up",
                         strerror(errno), errno);
            close(socket_descriptor);
            return false;
        }

        auto addr = sockaddr_can{
            .can_family = AF_CAN,
            .can_ifindex = ifr.ifr_ifindex,
            .can_addr = {}
        };

        // cast sockaddr_can* to a sockaddr*, as bind() uses a sockaddr* even though it can accept a sockaddr_can* for SocketCan
        if (bind(socket_descriptor, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            logger.fatal("bind error: {} ({})\nPossible cause: Another program may be using this interface", strerror(errno), errno);
            close(socket_descriptor);
            return false;
        }

        // TODO 2026-02-18 (Will Free): This is broken right now
        // make socket non blocking
        // const int flags = fcntl(socket_descriptor, F_GETFL, 0);
        // if (flags == -1) {
        //     logger.fatal("fcntl error: {} ({})", strerror(errno), errno);
        //     return false;
        // }
        //
        // if (fcntl(socket_descriptor, F_SETFL, flags | O_NONBLOCK) < 0) {
        //     logger.fatal("fcntl error: {} ({})", strerror(errno), errno);
        // }

        readThread = std::thread([this] {
            while (rclcpp::ok()) {
                if (auto frame = can_frame{}; readFrameIfAvailable(frame) == true) {
                    const auto id = frame.can_id & CAN_EFF_MASK;
                    // ReSharper disable once CppTemplateArgumentsCanBeDeduced
                    const auto data = std::vector<uint8_t>(std::begin(frame.data), std::end(frame.data));

                    // copy callbacks to avoid race condition where a callback is deleted as we're iterating
                    for (const auto callbacks = frame_callbacks; const auto& weak_ptr : callbacks) {
                        if (const auto callback_ptr = weak_ptr.lock()) {
                            const auto callback = *callback_ptr.get();
                            callback(id, data);
                        }
                    }
                }
            }
        });

        logger.info("CAN configuration on {} successful", path);
        return true;
    }

    std::shared_ptr<CANFrameCallback> CANController::registerFrameCallback(CANFrameCallback callback) {
        auto shared_pointer = std::shared_ptr<CANFrameCallback>(
            new CANFrameCallback(std::move(callback)),
            [this](CANFrameCallback* cb) {
                frame_callbacks.erase(
                    std::remove_if(
                        frame_callbacks.begin(),
                        frame_callbacks.end(),
                        [cb](const std::weak_ptr<CANFrameCallback>& weak_pointer) {
                            const auto sp = weak_pointer.lock();
                            return !sp || sp.get() == cb;
                        }),
                    frame_callbacks.end());
                delete cb;
            }
        );

        frame_callbacks.push_back(shared_pointer);

        return shared_pointer;
    }

    // bool CANController::sendBlockingFrame(const uint32_t id, const std::vector<uint8_t>& data) const {
            
    //     assert(data.size() < 8 && "CAN frame is too large");
    //     auto frame = can_frame{};
    //     frame.can_id = id | CAN_EFF_FLAG;

    //     frame.len = static_cast<uint8_t>(data.size());
    //     memcpy(frame.data, data.data(), data.size());

    //     return sendBlockingFrame(frame);
    // }

    bool CANController::readFrameIfAvailable(can_frame& frame) const {
        // Set up the file descriptor set
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(socket_descriptor, &read_fds);

        // Set up the timeout with zero seconds for non-blocking
        // TODO 2026-02-17 (Will Free): Should we set a timeout here?
        auto timeout = timeval{
            .tv_sec = 0,
            .tv_usec = READ_TIMEOUT_US,
        };

        // Use select to check if data is available
        if (
            const int result = select(socket_descriptor + 1, &read_fds, nullptr, nullptr, &timeout);
            result > 0 && FD_ISSET(socket_descriptor, &read_fds)
        ) {
            readFrame(frame);
            return true;
        } else if (result == 0) {
            // timeout
            return false;
        }

        return false;
    }

    // ReSharper disable once CppDFAUnreachableFunctionCall
    bool CANController::readFrame(can_frame& frame) const {
        // clear the previous frame contents
        memset(&frame, 0, sizeof(frame));

        const auto byte_count = read(socket_descriptor, &frame, sizeof(struct can_frame));

        if (byte_count == -1) {
            logger.fatal("read error: {} ({})", strerror(errno), errno);
            return false;
        }

        if (byte_count < static_cast<ssize_t>(sizeof(can_frame))) {
            logger.fatal("read error: incomplete CAN frame");
            return false;
        }
        return true;
    }

    // TODO 2026-02-14 (Will Free): what is this? what is it doing? why do we need it?
    bool CANController::sendBlockingFrame(const can_frame& frame) const {
        constexpr auto WAIT_TIME = std::chrono::microseconds(1);
        constexpr int MAX_ATTEMPTS = 32;

        for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
            if (writeFrame(frame))
                return true;

            if (errno == ENOBUFS || errno == EAGAIN) {
                std::this_thread::sleep_for(WAIT_TIME);
                continue;
            }

            logger.warn("Failed to send CAN frame: {} ({})", strerror(errno), errno);
            return false;
        }

        logger.error("Failed to send CAN frame after multiple retries, buffer consistently full.");

        return false;
    }

    bool CANController::writeFrame(const can_frame& frame) const {
        std::lock_guard lock(mtx);

        const auto byte_count = write(socket_descriptor, &frame, sizeof(can_frame));
        const auto original_errno = errno;
        if (byte_count == -1) {
            logger.fatal("write error: {} ({})", strerror(original_errno), original_errno);
            return false;
        }

        if (byte_count != sizeof(can_frame)) {
            logger.fatal("write error: did not write all the data, but no error code was returned");
            return false;
        }
        errno = original_errno;

        return true;
    }
}