#pragma once

#include <vector>
#include <memory>

#include "im_application.hpp"

class UiElement {
public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(UiElement)

    explicit UiElement(ImApplication& application) : application(application), logger(application.get_logger()) {}

    virtual ~UiElement() = default;

    virtual void onInit() {
        for (const auto& child : children)
            child->onInit();
    }

    virtual void onFrame() {}

    virtual void onShutdown() {
        for (const auto& child : children)
            child->onShutdown();
    }

protected:
    ImApplication& application;
    ros2_fmt_logger::Logger logger;
    std::vector<std::shared_ptr<UiElement>> children;

    void addChild(std::shared_ptr<UiElement> child) {
        children.push_back(std::move(child));
    }

    void drawChildren() const {
        for (const auto& child : children)
            child->onFrame();
    }
};
