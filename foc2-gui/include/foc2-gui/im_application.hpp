#pragma once

#include <string>
#include <ros2_fmt_logger/logger.hpp>
#include <rclcpp/node.hpp>

namespace tf2_ros {
    class Buffer;
    class TransformListener;
}

typedef struct SDL_GLContextState* SDL_GLContext;
typedef struct SDL_Window SDL_Window;

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

    SDL_GLContext gl_context;
    SDL_Window* window;

    virtual void onWindow() {}

    virtual void onInit();

    virtual void onFrame();
    virtual void onShutdown() {}

private:
    std::string title{};
    bool done = false;
    std::chrono::time_point<std::chrono::steady_clock> last_frame;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;

    void frame();
    void render() const;
    void quit() const;
};
