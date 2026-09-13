#pragma once

#include <imgui_internal.h>
#include <memory>
#include <vector>

#include "overlay.hpp"


// TODO 2026-05-03 (Will Free): This name kinda sucks ngl
class UiOverlayable {
public:
    virtual ~UiOverlayable() = default;

    virtual void onInit() const {
        for (auto&& [_, overlay] : overlays)
            overlay->onInit();
    }

    virtual void onShutdown() const {
        for (auto&& [_, overlay] : overlays)
            overlay->onShutdown();
    }

protected:
    void drawOverlays(ImDrawList* draw_list, const ImRect& bounds) const {
        // TODO 2026-05-05 (Will Free): adjust clipping rectangle
        for (auto&& [name, overlay] : overlays) {
            overlay->onDraw(draw_list, bounds);
        }
    }

    bool addOverlay(std::string name, UiOverlay::SharedPtr overlay) {
        if (findOverlay(name) == nullptr)
            return false;

        overlays.emplace_back(std::move(name), std::move(overlay));
        return true;
    }

    void addOverlay(UiOverlay::SharedPtr overlay) {
        overlays.emplace_back(std::nullopt, std::move(overlay));
    }

    UiOverlay::SharedPtr findOverlay(const std::string& name) {
        const auto iterator = std::ranges::find_if(overlays, [name](auto&& overlay) {
            return overlay.name.has_value() && overlay.name.value() == name;
        });

        if (iterator != overlays.end())
            return iterator->overlay;
        else
            return nullptr;
    }

    bool removeOverlay(const std::string& name) {
        return !std::ranges::remove_if(overlays, [name](auto&& overlay) {
            return overlay.name.has_value() && overlay.name.value() == name;
        }).empty();
    }

private:
    struct OverlayMeta {
        std::optional<std::string> name;
        UiOverlay::SharedPtr overlay;
    };

    std::vector<OverlayMeta> overlays;
};
