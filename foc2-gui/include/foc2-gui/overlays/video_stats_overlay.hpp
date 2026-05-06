#pragma once
#include <mutex>
#include <fmt/core.h>

#include "foc2-gui/overlay.hpp"
#include "foc2-gui/util/imgui_util.h"


class VideoStatsOverlay : public UiOverlay {
public:
    explicit VideoStatsOverlay(ImApplication& application) : UiOverlay(application) {}

    void updateStats(const double fps, /*const double bitrate,*/ const int width, const int height) {
        std::lock_guard lock(mutex_);
        this->fps = fps;
        // this->bitrate = bitrate;
        this->width = width;
        this->height = height;
    }

    void onDraw(ImDrawList* draw_list, const ImRect& bounds) override {
        std::lock_guard lock(mutex_);

        static constexpr auto BOX_OUTER_PADDING = 8;
        static constexpr auto BOX_INNER_PADDING = 4;

        const auto stats_text = fmt::format("FPS: {:.2f}\n{}x{}", fps, width, height);

        const auto top_right = bounds.GetTL();
        const auto size = ImGui::CalcTextSize(stats_text);
        const auto max_x = std::max(size.x, 120.0f);

        draw_list->AddRectFilled(
            ImVec2(top_right.x + BOX_OUTER_PADDING, top_right.y + BOX_OUTER_PADDING),
            ImVec2(top_right.x + max_x + BOX_OUTER_PADDING + BOX_INNER_PADDING * 2, top_right.y + size.y + BOX_OUTER_PADDING + BOX_INNER_PADDING * 2),
            ImGui::ImColor(0, 0, 0, 128),
            6,
            ImDrawFlags_RoundCornersAll
        );

        draw_list->AddText(
            ImVec2(top_right.x + BOX_OUTER_PADDING + BOX_INNER_PADDING, top_right.y + BOX_OUTER_PADDING + BOX_INNER_PADDING),
            ImGui::ImColor(255, 255, 255, 255),
            stats_text.c_str()
        );
    }

private:
    double fps = 0.0;
    // double bitrate = 0.0;
    int width = 0;
    int height = 0;
    std::mutex mutex_;
};
