#pragma once

#include <string>
#include <ros2_fmt_logger/logger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <rclcpp/node.hpp>

typedef struct SDL_GLContextState* SDL_GLContext;
typedef struct SDL_Window SDL_Window;

class ImApplication : public rclcpp::Node {
public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(ImApplication)

    explicit ImApplication(const std::string& node_name, std::string title = "Main Window");
    ~ImApplication() override;

    int init();
    int run();

    tf2_ros::Buffer& tfBuffer() {
        return this->tf_buffer;
    }

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

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    void frame();
    void render() const;
    void quit() const;
};
