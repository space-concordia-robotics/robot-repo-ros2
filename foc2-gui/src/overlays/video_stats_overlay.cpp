#include "foc2-gui/overlays/video_stats_overlay.hpp"

#include <implot.h>
#include <implot_demo.cpp>
#include <fmt/chrono.h>

using namespace std::chrono_literals;

VideoStatsOverlay::VideoStatsOverlay(ImApplication& application)
    : UiOverlay(application),
      // video stats is updated every 50ms (see video widget, both must be updated at the same time))
      fps_buffer(CircularBuffer<double>(10s / 50ms)) {
    for (auto i = 0u; i < fps_buffer.capacity(); ++i) {
        fps_buffer.push_back(0);
    }
}

void VideoStatsOverlay::updateStats(const VideoStats& stats) {
    std::lock_guard lock(stats_mutex);
    this->stats = stats;

    fps_buffer.push_back(stats.fps);
}

void VideoStatsOverlay::onDraw(ImDrawList* draw_list, const ImRect& bounds) {
    std::lock_guard lock(stats_mutex);

    static constexpr auto BOX_OUTER_PADDING = 8;
    static constexpr auto BOX_INNER_PADDING = 4;

    // TODO 2026-05-07 (Will Free): technically here we only get the instantaneous bitrate,
    //  so we might not see it if the bitrate spikes extremely high but only for a very short period.
    static constexpr auto STATS_MESSAGE =
        "FPS: {:.2f}\n"
        "Bitrate: {:.2f} kbps\n"
        "Latency: {} ms\n"
        "Jitter: {} ms\n"
        "RTT: {} ns\n"
        "{}x{}";

    auto mean_fps = 0.0;

    for (const auto& fps : fps_buffer) {
        mean_fps += fps;
    }

    mean_fps /= fps_buffer.size();

    const auto stats_text = fmt::format(
        STATS_MESSAGE,
        mean_fps, stats.bitrate,
        stats.rtsp_latency.count(),
        stats.average_jitter.count(), stats.rtt.count(),
        stats.width, stats.height
    );

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

    // TODO 2026-05-07 (Will Free): figure out a way to draw the plot in an overlay
    // static constexpr ImPlotAxisFlags AXIS_FLAGS = ImPlotAxisFlags_NoTickLabels;
    //
    // static constexpr ImPlotFlags PLOT_FLAGS = ImPlotFlags_NoTitle | ImPlotFlags_NoLegend | ImPlotFlags_NoFrame;
    // if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, ImGui::GetTextLineHeight() * 10), PLOT_FLAGS)) {
    //     ImPlot::SetupAxes(nullptr, nullptr, AXIS_FLAGS, AXIS_FLAGS);
    //     ImPlot::SetupAxisLimits(ImAxis_X1, 0, fps_buffer.size(), ImGuiCond_Always);
    //     ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 40);
    //     ImPlotSpec spec;
    //     spec.Offset = fps_buffer.position();
    //     spec.Stride = sizeof(double);
    //     ImPlot::PlotLine("Mouse Y", fps_buffer.first(), fps_buffer.size(), 1, 0, spec);
    //     ImPlot::EndPlot();
    // }
}
