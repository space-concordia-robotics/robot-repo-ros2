#pragma once

#include <string>
#include <rclcpp/node.hpp>
#include <ros2_fmt_logger/logger.hpp>
#include <SDL3/SDL.h>

class ImApplication : public rclcpp::Node {
public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(ImApplication)

    explicit ImApplication(const std::string& node_name, const std::string& title = "Main Window");
    ~ImApplication() override;

    int init();
    int run();

protected:
    ros2_fmt_logger::Logger logger;

    SDL_GLContext gl_context;
    SDL_Window* window;

    virtual void onWindow() {}

    virtual void onInit() {}
    virtual void onFrame();
    virtual void onShutdown() {}

private:
    std::string title{};
    bool done = false;
    std::chrono::time_point<std::chrono::steady_clock> last_frame;

    void frame();
    void render() const;
    void quit() const;
};

#include <SDL3/SDL.h>
