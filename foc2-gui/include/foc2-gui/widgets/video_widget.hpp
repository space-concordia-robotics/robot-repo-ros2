#pragma once

#include <opencv2/videoio.hpp>
#include <opencv2/core/mat.hpp>

#include "foc2-gui/overlayable.hpp"
#include "foc2-gui/widget.hpp"
#include "foc2-gui/overlays/video_stats_overlay.hpp"


class VideoWidget : public UiWidget, public UiOverlayable {
public:
    explicit VideoWidget(ImApplication& application);

    void onInit() override;
    void onShutdown() override;

    ImVec2 expectedSize(ImVec2 available) const;

protected:
    void draw() override;

private:
    std::thread video_thread;
    std::atomic<bool> running = true;
    cv::Mat next_frame;
    cv::Mat current_frame;
    std::mutex frame_mutex;
    std::shared_ptr<cv::VideoCapture> video_capture;

    std::shared_ptr<VideoStatsOverlay> video_stats_overlay;

    bool new_frame_available = false;

    GLuint texture_id = 0;
    int texture_width = 0;
    int texture_height = 0;

    void videoThread();

    void updateTexture();
};
