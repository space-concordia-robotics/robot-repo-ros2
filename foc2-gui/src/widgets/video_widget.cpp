#include "foc2-gui/widgets/video_widget.hpp"

#include <imgui.h>
#include <opencv2/videoio.hpp>
#include <SDL3/SDL_opengl.h>

#include "foc2-gui/overlays/crosshair_overlay.hpp"
#include "foc2-gui/overlays/video_stats_overlay.hpp"

VideoWidget::VideoWidget(ImApplication& application) : UiWidget(application) {
    addOverlay(std::make_shared<CrosshairOverlay>(application));
    video_stats_overlay = std::make_shared<VideoStatsOverlay>(application);
    addOverlay(video_stats_overlay);
}

void VideoWidget::onInit() {
    UiWidget::onInit();

    video_thread = std::thread(std::bind(&VideoWidget::videoThread, this));
}

void VideoWidget::onShutdown() {
    UiWidget::onShutdown();

    running = false;
    video_thread.join();
}

void VideoWidget::draw() {
    updateTexture();

    ImGui::BeginChild("VideoRegion", ImVec2(0, 0), 0, ImGuiWindowFlags_NoScrollbar);

    if (texture_id) {
        const auto avail = ImGui::GetContentRegionAvail();

        const float video_aspect = static_cast<float>(current_frame.cols) / static_cast<float>(current_frame.rows);

        ImVec2 size = avail;

        if (const float window_aspect = avail.x / avail.y; window_aspect > video_aspect) {
            size.x = avail.y * video_aspect;
        } else {
            size.y = avail.x / video_aspect;
        }

        ImGui::SetCursorPosX((avail.x - size.x) * 0.5f);
        ImGui::SetCursorPosY((avail.y - size.y) * 0.5f);

        ImGui::Image(texture_id, size);

        drawOverlays(ImGui::GetWindowDrawList(), ImRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax()));
    } else {
        ImGui::Text("Waiting for video frame...");
    }

    ImGui::EndChild();
}

void VideoWidget::videoThread() {
    auto video_capture = cv::VideoCapture(
        "rtspsrc location=rtsp://127.0.0.1:8554/test ! decodebin ! videoconvert "
        "! video/x-raw,format=RGBA ! appsink drop=true max-buffers=1 sync=true",
        cv::CAP_GSTREAMER
    );

    video_capture.set(cv::CAP_PROP_BUFFERSIZE, 1);

    std::cout << "Backend: " << video_capture.getBackendName() << std::endl;

    if (!video_capture.isOpened()) {
        logger.error("Failed to open RTSP stream");
        return;
    }

    // ReSharper disable once CppTooWideScope
    cv::Mat frame;

    while (running) {
        if (!video_capture.read(frame)) {
            logger.warn("RTSP read failed");
            continue;
        }

        {
            std::lock_guard lock(frame_mutex);
            frame.copyTo(next_frame);
            new_frame_available = true;
        }

        const auto fps = video_capture.get(cv::CAP_PROP_FPS);
        // const auto bitrate = video_capture.get(cv::CAP_PROP_BITRATE);
        const auto width = frame.cols;
        const auto height = frame.rows;

        video_stats_overlay->updateStats(fps, /*bitrate,*/ width, height);
    }
}

void VideoWidget::updateTexture() {
    {
        auto lock = std::lock_guard(frame_mutex);
        if (!new_frame_available)
            return;

        if (next_frame.empty())
            return;

        std::swap(current_frame, next_frame);
        new_frame_available = false;
    }

    if (texture_id == 0) {
        glGenTextures(1, &texture_id);
        glBindTexture(GL_TEXTURE_2D, texture_id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    glBindTexture(GL_TEXTURE_2D, texture_id);

    // we do this because glTexImage2D() is more expensive than glTexSubImage2D().
    // if the texture resolution hasn't changed recently, then we can just call
    // glTexSubImage2D().
    if (current_frame.cols != texture_width || current_frame.rows != texture_height) {
        texture_width = current_frame.cols;
        texture_height = current_frame.rows;

        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGBA,
            texture_width,
            texture_height,
            0,
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            current_frame.data
        );
    } else {
        glTexSubImage2D(
            GL_TEXTURE_2D,
            0,
            0, 0,
            texture_width,
            texture_height,
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            current_frame.data
        );
    }
}
