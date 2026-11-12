#pragma once

#include <string>
#include <ros2_fmt_logger/logger.hpp>
#include <rclcpp/node.hpp>

namespace tf2_ros {
    class Buffer;
    class TransformListener;
}

struct ImApplicationState;

class ImApplication : public rclcpp::Node {
public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(ImApplication)

    explicit ImApplication(const std::string& node_name, std::string title = "Main Window");
    ~ImApplication() override;

    int init();
    int run();

    tf2_ros::Buffer& tfBuffer() const;

protected:
    ros2_fmt_logger::Logger logger;

    // a separate state struct mainly exists to avoid needing to have imports for SDL in here
    std::unique_ptr<ImApplicationState> state;

    virtual void onWindow() {}

    virtual void onInit();

    virtual void onFrame();
    virtual void onShutdown() {}

private:
    std::string title;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;

    void frame() const;
    void render() const;
    void quit() const;
};
