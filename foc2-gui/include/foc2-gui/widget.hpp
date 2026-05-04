#pragma once

#include "element.hpp"

class UiWidget : public UiElement {
public:
    explicit UiWidget(ImApplication& application) : UiElement(application) {}

    void onFrame() override {
        draw();
        drawChildren();
    }

protected:
    virtual void draw() = 0;
};
