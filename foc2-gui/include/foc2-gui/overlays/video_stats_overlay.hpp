#pragma once

#include <mutex>

#include "foc2-gui/overlay.hpp"
#include "foc2-gui/util/circular_buffer.hpp"
#include "foc2-gui/util/imgui_util.h"


class VideoStatsOverlay : public UiOverlay {
public:
    struct VideoStats {
        double fps = 0;
        long width = 0;
        long height = 0;
        //! bitrate (kbps)
        double bitrate = 0;
        std::chrono::milliseconds rtsp_latency = std::chrono::milliseconds(0);
        //! average jitter
        std::chrono::milliseconds average_jitter = std::chrono::milliseconds(0);
        //! percent (0-1)
        double packet_loss = 0;
        std::chrono::nanoseconds rtt = std::chrono::nanoseconds(0);
    };

    explicit VideoStatsOverlay(ImApplication& application);

    void updateStats(const VideoStats& stats);

    void onDraw(ImDrawList* draw_list, const ImRect& bounds) override;

private:
    std::mutex stats_mutex;
    VideoStats stats = {};
    CircularBuffer<double> fps_buffer;
};
