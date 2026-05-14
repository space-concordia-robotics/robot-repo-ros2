#pragma once

#include <thread>
#include <gst/gstelement.h>
#include <gst/app/gstappsink.h>
#include <opencv2/core/mat.hpp>
#include <SDL3/SDL_opengl.h>

#include "foc2-gui/overlayable.hpp"
#include "foc2-gui/widget.hpp"
#include "foc2-gui/overlays/video_stats_overlay.hpp"


class VideoWidget : public UiWidget, public UiOverlayable {
public:
    explicit VideoWidget(ImApplication& application, const std::string& source_url, bool minimap, const std::string& videoflip);

    void onInit() override;
    void onShutdown() override;

    [[nodiscard]] ImVec2 expectedSize(ImVec2 available) const;

protected:
    void draw() override;

private:
    std::string source_url;
    bool minimap;
    std::string videoflip;

    rclcpp::TimerBase::SharedPtr stats_timer;

    std::thread gst_thread;
    std::atomic<bool> running = true;
    cv::Mat next_frame;
    cv::Mat current_frame;
    std::mutex frame_mutex;

    GstElement* pipeline = nullptr;
    GstAppSink* appsink = nullptr;
    GstElement* jitterbuffer = nullptr;

    std::mutex stats_mutex;
    std::chrono::time_point<std::chrono::steady_clock> last_probe;
    std::chrono::time_point<std::chrono::steady_clock> last_frame;

    VideoStatsOverlay::VideoStats video_stats;
    std::shared_ptr<VideoStatsOverlay> video_stats_overlay;

    bool new_frame_available = false;

    GLuint texture_id = 0;
    int texture_width = 0;
    int texture_height = 0;

    void videoThread();

    GstFlowReturn onNewSample(GstAppSink* sink);
    void updateTexture();
};
