#pragma once

#include "foc2-gui/overlay.hpp"
#include "foc2-gui/util/imgui-util.h"

class CrosshairOverlay : public UiOverlay {
public:
    explicit CrosshairOverlay(ImApplication& application) : UiOverlay(application) {}

protected:
    void onDraw(ImDrawList* draw_list, const ImRect& bounds) override {
        const auto length = std::min(bounds.GetHeight(), bounds.GetWidth()) * 0.05;
        const auto center = bounds.GetCenter();

        static constexpr auto CROSSHAIR_COLOR = ImColor(255, 255, 255, 255);

        draw_list->AddLine(
            ImVec2(center.x - length, center.y),
            ImVec2(center.x + length, center.y),
            CROSSHAIR_COLOR,
            2.0
        );

        draw_list->AddLine(
            ImVec2(center.x, center.y - length),
            ImVec2(center.x, center.y + length),
            CROSSHAIR_COLOR,
            2.0
        );
    }
};
