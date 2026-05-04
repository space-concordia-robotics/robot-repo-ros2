#pragma once

#include <imgui_internal.h>
#include <memory>
#include <vector>

#include "overlay.hpp"


// TODO 2026-05-03 (Will Free): This name kinda sucks ngl
class UiOverlayable {
public:
    virtual ~UiOverlayable() = default;

protected:
    void drawOverlays(ImDrawList* dl, const ImRect& rect) const {
        for (const auto& overlay : overlays) {
            overlay->onDraw(dl, rect);
        }
    }

    void addOverlay(std::shared_ptr<UiOverlay> overlay) {
        overlays.push_back(std::move(overlay));
    }

private:
    std::vector<std::shared_ptr<UiOverlay>> overlays;
};
