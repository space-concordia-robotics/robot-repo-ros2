#pragma once

#include <imgui_internal.h>
#include <memory>
#include <vector>

#include "overlay.hpp"


// TODO 2026-05-03 (Will Free): This name kinda sucks ngl
class UiOverlayable {
public:
    virtual ~UiOverlayable() = default;

    virtual void onInit() {
        for (const auto& overlay : overlays)
            overlay->onInit();
    }

    virtual void onShutdown() {
        for (const auto& overlay : overlays)
            overlay->onShutdown();
    }

protected:
    void drawOverlays(ImDrawList* dl, const ImRect& rect) const {
        // TODO 2026-05-05 (Will Free): adjust clipping rectangle
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
