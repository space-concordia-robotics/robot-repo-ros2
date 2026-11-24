#pragma once

#include "element.hpp"
#include "subscription_group.hpp"

class UiWidget : public UiElement {
public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(UiWidget)

    explicit UiWidget(ImApplication& application)
        : UiElement(application) {}

    void onFrame() override {
        draw();
        drawChildren();
    }

protected:
    virtual void draw() = 0;
};
