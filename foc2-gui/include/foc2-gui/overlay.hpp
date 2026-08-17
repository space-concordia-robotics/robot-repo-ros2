#pragma once

#include <imgui_internal.h>

#include "im_application.hpp"

class UiOverlay {
public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(UiOverlay)

    explicit UiOverlay(ImApplication& application)
        : application(application), logger(application.get_logger()) {}

    virtual ~UiOverlay() = default;

    virtual void onInit() {
        for (const auto& child : children)
            child->onInit();
    }

    // TODO 2026-05-05 (Will Free): wrap ImDrawList, so that I can provide add useful functions to it
    // TODO 2026-05-05 (Will Free): get rid of things like ImRect (part of imgui_internal.h) & replace it with eigen
    virtual void onDraw(ImDrawList* draw_list, const ImRect& bounds) = 0;

    virtual void onShutdown() {
        for (const auto& child : children)
            child->onShutdown();
    }

protected:
    ImApplication& application;
    ros2_fmt_logger::Logger logger;
    std::vector<std::shared_ptr<UiOverlay>> children;

    void addChild(std::shared_ptr<UiOverlay> child) {
        children.push_back(std::move(child));
    }

    void drawChildren(ImDrawList* dl, const ImRect& rect) const {
        for (const auto& child : children)
            child->onDraw(dl, rect);
    }
};
